#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <memory>
#include <vector>
#include <thread>
#include <atomic>
#include <signal.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

// OpenCV related
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

// Libcamera related
#include <libcamera/libcamera.h>

using namespace cv;
using namespace cv::dnn;
using namespace std;
using namespace libcamera;
using namespace std::chrono_literals;

// Global variables
atomic<bool> g_running{true};
const char* PIPE_PATH = "/tmp/detection_pipe";
int pipe_fd = -1;

// Neural network parameters
float confThreshold = 0.3; // Lower confidence threshold to increase detection rate
float nmsThreshold = 0.5;  // Higher NMS threshold to keep more detection results
int inpWidth = 300;        // Restored to 300 to match model design
int inpHeight = 300;       // Restored to 300 to match model design
vector<string> classes;    // Class list

// Function declarations
void processCameraFrame(cv::Mat& frame, Net& net, int& peopleCount);
void postprocess(cv::Mat& frame, const vector<cv::Mat>& outs, int& peopleCount);
void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame);
vector<String> getOutputsNames(const Net& net);
void signalHandler(int signum);
bool sendFrameToWebSocket(const cv::Mat& frame, int peopleCount);

// Function to send frame to WebSocket server via named pipe
bool sendFrameToWebSocket(const cv::Mat& frame, int peopleCount) {
    if (pipe_fd == -1) {
        return false;
    }
    
    try {
        // Encode frame as JPEG
        vector<uchar> buffer;
        cv::imencode(".jpg", frame, buffer, {cv::IMWRITE_JPEG_QUALITY, 80});
        
        // Prepare data packet: <people_count>:<jpg_data>FRAME_END
        string countStr = to_string(peopleCount) + ":";
        string frameEnd = "FRAME_END";
        
        // Write people count
        if (write(pipe_fd, countStr.c_str(), countStr.size()) == -1) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Pipe buffer is full, try again later
                cerr << "Pipe buffer full, skipping frame" << endl;
                return false;
            } else {
                // Other write error
                cerr << "Error writing to pipe: " << strerror(errno) << endl;
                close(pipe_fd);
                pipe_fd = -1;
                return false;
            }
        }
        
        // Write image data
        if (write(pipe_fd, buffer.data(), buffer.size()) == -1) {
            cerr << "Error writing image data to pipe: " << strerror(errno) << endl;
            return false;
        }
        
        // Write frame separator
        if (write(pipe_fd, frameEnd.c_str(), frameEnd.size()) == -1) {
            cerr << "Error writing frame separator to pipe: " << strerror(errno) << endl;
            return false;
        }
        
        return true;
    } catch (const cv::Exception& e) {
        cerr << "OpenCV exception in sendFrameToWebSocket: " << e.what() << endl;
        return false;
    } catch (const std::exception& e) {
        cerr << "Exception in sendFrameToWebSocket: " << e.what() << endl;
        return false;
    } catch (...) {
        cerr << "Unknown exception in sendFrameToWebSocket" << endl;
        return false;
    }
}

// Function to get the absolute path for model files
string getModelPath(const string& filename) {
    char buffer[1024];
    string execPath = "/home/pu/Desktop/Dual-Wheel-Balancing-Drone/PeopleDetection/Model/";
    return execPath + filename;
}

class PeopleDetector {
private:
    std::unique_ptr<libcamera::CameraManager> mCameraManager;
    std::shared_ptr<libcamera::Camera> mCamera;
    std::unique_ptr<libcamera::CameraConfiguration> mConfig;
    FrameBufferAllocator *mAllocator;
    std::vector<std::unique_ptr<libcamera::Request>> mRequests;
    std::map<libcamera::FrameBuffer *, uint8_t*> mMappedBuffers;
    Net mNet;
    cv::HOGDescriptor hog; // HOG detector
    
    bool setupCamera() {
        mCameraManager = std::make_unique<CameraManager>();
        int ret = mCameraManager->start();
        if (ret) {
            cerr << "Failed to start camera manager: " << ret << endl;
            return false;
        }

        // Get the first available camera
        auto cameras = mCameraManager->cameras();
        if (cameras.empty()) {
            cerr << "No cameras available" << endl;
            return false;
        }
        
        mCamera = cameras[0];
        ret = mCamera->acquire();
        if (ret) {
            cerr << "Failed to acquire camera: " << ret << endl;
            return false;
        }
        
        // Configure camera parameters
        mConfig = mCamera->generateConfiguration({StreamRole::Viewfinder});
        if (!mConfig) {
            cerr << "Failed to generate camera configuration" << endl;
            return false;
        }
        
        StreamConfiguration &streamConfig = mConfig->at(0);
        cout << "Default viewfinder configuration: " << streamConfig.toString() << endl;
        
        // Optimization: lower resolution to reduce memory usage
        streamConfig.size.width = 320;   // Lower resolution
        streamConfig.size.height = 240;  // Lower resolution
        streamConfig.pixelFormat = libcamera::formats::YUYV;
        streamConfig.bufferCount = 2;    // Reduce buffer count to save memory
        
        // Try to validate the configuration
        ret = mConfig->validate();
        if (ret) {
            cerr << "Failed to validate camera configuration with YUYV format: " << ret << endl;
            // Try alternative configuration
            streamConfig.pixelFormat = libcamera::formats::MJPEG;
            ret = mConfig->validate();
            if (ret) {
                cerr << "Failed to validate camera configuration with MJPEG format: " << ret << endl;
                return false;
            }
            cout << "Using MJPEG format" << endl;
        } else {
            cout << "Using YUYV format" << endl;
        }
        
        ret = mCamera->configure(mConfig.get());
        if (ret) {
            cerr << "Failed to configure camera: " << ret << endl;
            return false;
        }
        
        cout << "Camera configured successfully" << endl;
        
        // Create frame buffer allocator
        mAllocator = new FrameBufferAllocator(mCamera);
        
        for (StreamConfiguration &cfg : *mConfig) {
            ret = mAllocator->allocate(cfg.stream());
            if (ret < 0) {
                cerr << "Failed to allocate buffers for stream" << endl;
                return false;
            }
            
            const std::vector<std::unique_ptr<FrameBuffer>> &buffers = mAllocator->buffers(cfg.stream());
            cout << "Allocated " << buffers.size() << " buffers for stream" << endl;
        }
        
        return true;
    }
    
    // HOG person detection function
    void detectWithHOG(cv::Mat& frame, int& peopleCount) {
        try {
            if (frame.empty()) {
                cerr << "Error: Frame is empty for HOG detection" << endl;
                return;
            }
            
            // Resize to speed up detection
            cv::Mat resized;
            cv::resize(frame, resized, cv::Size(320, 240));
            
            // Detect people
            vector<cv::Rect> found, found_filtered;
            hog.detectMultiScale(resized, found, 0, cv::Size(8,8), cv::Size(32,32), 1.05, 2);
            
            cout << "HOG detected " << found.size() << " potential people" << endl;
            
            // Filter overlapping bounding boxes
            for (size_t i = 0; i < found.size(); i++) {
                cv::Rect r = found[i];
                int j;
                for (j = 0; j < found.size(); j++) 
                    if (j != i && (r & found[j]) == r)
                        break;
                if (j == found.size())
                    found_filtered.push_back(r);
            }
            
            // Update people count
            peopleCount += found_filtered.size();
            
            // Draw bounding boxes
            for (size_t i = 0; i < found_filtered.size(); i++) {
                cv::Rect r = found_filtered[i];
                // Adjust back to original image size
                r.x *= frame.cols / 320.0;
                r.y *= frame.rows / 240.0;
                r.width *= frame.cols / 320.0;
                r.height *= frame.rows / 240.0;
                
                cv::rectangle(frame, r.tl(), r.br(), cv::Scalar(0, 255, 0), 2);
                cv::putText(frame, "Person", r.tl(), cv::FONT_HERSHEY_SIMPLEX, 
                           0.75, cv::Scalar(0, 255, 0), 2);
                cout << "Drew HOG detection at: " << r << endl;
            }
            
            resized.release();
        } catch (const cv::Exception& e) {
            cerr << "Exception in HOG detection: " << e.what() << endl;
        } catch (const std::exception& e) {
            cerr << "Exception in HOG detection: " << e.what() << endl;
        } catch (...) {
            cerr << "Unknown exception in HOG detection" << endl;
        }
    }
    
    // Process captured requests
    void requestComplete(libcamera::Request *request) {
        if (request->status() == libcamera::Request::RequestCancelled)
            return;
            
        try {
            libcamera::FrameBuffer *buffer = request->buffers().begin()->second;
            const libcamera::FrameMetadata &metadata = buffer->metadata();
            
            // Check pixel format
            libcamera::PixelFormat pixelFormat = mConfig->at(0).pixelFormat;
            cv::Mat frame;
            
            if (pixelFormat == libcamera::formats::YUYV) {
                // Create YUYV format Mat
                cv::Mat yuyv(mConfig->at(0).size.height, mConfig->at(0).size.width, 
                           CV_8UC2, mMappedBuffers[buffer]);
                
                // Convert YUYV to RGB
                cv::cvtColor(yuyv, frame, cv::COLOR_YUV2BGR_YUYV);
            } else if (pixelFormat == libcamera::formats::MJPEG) {
                const auto& planes = metadata.planes();
                if (planes.empty()) {
                    cerr << "No planes found in buffer metadata" << endl;
                    goto requeue;
                }
                
                const libcamera::FrameMetadata::Plane& metadataPlane = planes[0];
                
                // Decode MJPEG to Mat
                cv::Mat mjpegData(1, metadataPlane.bytesused, CV_8UC1, mMappedBuffers[buffer]);
                frame = cv::imdecode(mjpegData, cv::IMREAD_COLOR);
                
                if (frame.empty()) {
                    cerr << "Failed to decode MJPEG frame" << endl;
                    goto requeue;
                }
            } else if (pixelFormat == libcamera::formats::RGB888) {
                // Directly use RGB data
                frame = cv::Mat(mConfig->at(0).size.height, mConfig->at(0).size.width, 
                              CV_8UC3, mMappedBuffers[buffer]);
            } else {
                cerr << "Unsupported pixel format" << endl;
                goto requeue;
            }
            
            // Safely process image
            try {
                // Check if frame is valid before processing
                if (frame.empty() || frame.rows <= 0 || frame.cols <= 0) {
                    cerr << "Invalid frame: empty or has invalid dimensions" << endl;
                } else {
                    // Initialize people count for this frame
                    int peopleCount = 0;
                    
                    // First try using DNN model detection
                    processCameraFrame(frame, mNet, peopleCount);
                    
                    // Then use HOG detector as backup
                    detectWithHOG(frame, peopleCount);
                    
                    // Add timestamp to the frame
                    auto now = std::chrono::system_clock::now();
                    auto now_time = std::chrono::system_clock::to_time_t(now);
                    std::string timeStr = std::ctime(&now_time);
                    // Remove the newline character from ctime output
                    timeStr.erase(std::remove(timeStr.begin(), timeStr.end(), '\n'), timeStr.end());
                    
                    cv::putText(frame, timeStr, cv::Point(10, frame.rows - 10), 
                              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                    
                    // Send frame to WebSocket server
                    sendFrameToWebSocket(frame, peopleCount);
                    
                    // Display image locally (optional)
                    cv::imshow("People Detection Test", frame);
                    cv::waitKey(1);
                }
            } catch (const cv::Exception& e) {
                cerr << "Exception in processCameraFrame: " << e.what() << endl;
            } catch (const std::exception& e) {
                cerr << "Exception in processCameraFrame: " << e.what() << endl;
            } catch (...) {
                cerr << "Unknown exception in processCameraFrame" << endl;
            }
            
            // Explicitly release memory
            frame.release();
            
        } catch (const std::exception& e) {
            cerr << "Exception in requestComplete: " << e.what() << endl;
        } catch (...) {
            cerr << "Unknown exception in requestComplete" << endl;
        }
        
    requeue:
        // Re-queue the request to the camera
        libcamera::Stream *stream = mConfig->at(0).stream();
        const std::vector<std::unique_ptr<FrameBuffer>> &buffers = mAllocator->buffers(stream);
        
        size_t index = request->cookie();
        libcamera::FrameBuffer *buffer = buffers[index].get();
        
        request->reuse();
        request->addBuffer(stream, buffer);
        mCamera->queueRequest(request);
    }
    
public:
    PeopleDetector() : hog(cv::HOGDescriptor::getDefaultPeopleDetector()), mAllocator(nullptr) {
        // Initialize HOG detector with default people detector
        hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
    }
    
    ~PeopleDetector() {
        // Clean up resources
        if (mCamera) {
            mCamera->stop();
            mCamera->release();
        }
        
        if (mAllocator) {
            delete mAllocator;
        }
        
        if (mCameraManager) {
            mCameraManager->stop();
        }
    }
    
    bool initialize() {
        try {
            // Set up camera
            if (!setupCamera()) {
                cerr << "Failed to set up camera" << endl;
                return false;
            }
            
            // Load neural network
            string modelPath = getModelPath("MobileNetSSD_deploy.caffemodel");
            string configPath = getModelPath("MobileNetSSD_deploy.prototxt");
            
            try {
                mNet = cv::dnn::readNetFromCaffe(configPath, modelPath);
                
                if (mNet.empty()) {
                    cerr << "Failed to load model" << endl;
                    // Continue with HOG detector only
                } else {
                    cout << "Neural network model loaded successfully" << endl;
                    
                    // Load class names
                    string classesFile = getModelPath("coco.names");
                    ifstream ifs(classesFile.c_str());
                    string line;
                    while (getline(ifs, line)) {
                        classes.push_back(line);
                    }
                }
            } catch (const cv::Exception& e) {
                cerr << "Exception loading neural network: " << e.what() << endl;
                // Continue with HOG detector only
            }
            
            return true;
        } catch (const std::exception& e) {
            cerr << "Exception in initialize: " << e.what() << endl;
            return false;
        } catch (...) {
            cerr << "Unknown exception in initialize" << endl;
            return false;
        }
    }
    
    bool start() {
        try {
            // Set up request completion handler
            mCamera->requestCompleted.connect(this, &PeopleDetector::requestComplete);
            
            // Create and queue requests
            libcamera::Stream *stream = mConfig->at(0).stream();
            const std::vector<std::unique_ptr<FrameBuffer>> &buffers = mAllocator->buffers(stream);
            
            for (size_t i = 0; i < buffers.size(); i++) {
                std::unique_ptr<libcamera::Request> request = mCamera->createRequest();
                if (!request) {
                    cerr << "Failed to create request" << endl;
                    return false;
                }
                
                // Set cookie for tracking which buffer this request is using
                request->setCookie(i);
                
                // Map buffers for CPU access
                const libcamera::FrameBuffer *buffer = buffers[i].get();
                const std::vector<libcamera::FrameBuffer::Plane> &planes = buffer->planes();
                
                for (const auto &plane : planes) {
                    void *memory = mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
                    if (memory == MAP_FAILED) {
                        cerr << "Failed to map buffer memory" << endl;
                        return false;
                    }
                    
                    mMappedBuffers[buffers[i].get()] = static_cast<uint8_t*>(memory);
                }
                
                // Add buffer to request
                if (request->addBuffer(stream, buffer) < 0) {
                    cerr << "Failed to add buffer to request" << endl;
                    return false;
                }
                
                mRequests.push_back(std::move(request));
            }
            
            // Start the camera
            if (mCamera->start()) {
                cerr << "Failed to start camera" << endl;
                return false;
            }
            
            // Queue requests
            for (auto &request : mRequests) {
                if (mCamera->queueRequest(request.get()) < 0) {
                    cerr << "Failed to queue request" << endl;
                    return false;
                }
            }
            
            cout << "Camera started successfully" << endl;
            return true;
        } catch (const std::exception& e) {
            cerr << "Exception in start: " << e.what() << endl;
            return false;
        } catch (...) {
            cerr << "Unknown exception in start" << endl;
            return false;
        }
    }
    
    void stop() {
        try {
            if (mCamera) {
                mCamera->stop();
            }
            
            // Unmap buffers
            for (auto &pair : mMappedBuffers) {
                libcamera::FrameBuffer *buffer = pair.first;
                uint8_t *memory = pair.second;
                
                const std::vector<libcamera::FrameBuffer::Plane> &planes = buffer->planes();
                for (const auto &plane : planes) {
                    munmap(memory, plane.length);
                }
            }
            
            mMappedBuffers.clear();
            mRequests.clear();
            
            cout << "Camera stopped" << endl;
        } catch (const std::exception& e) {
            cerr << "Exception in stop: " << e.what() << endl;
        } catch (...) {
            cerr << "Unknown exception in stop" << endl;
        }
    }
};

// Process the camera frame with neural network
void processCameraFrame(cv::Mat& frame, Net& net, int& peopleCount) {
    try {
        if (net.empty()) {
            // Skip neural network processing if model wasn't loaded
            return;
        }
        
        if (frame.empty()) {
            cerr << "Empty frame passed to processCameraFrame" << endl;
            return;
        }
        
        // Create a 4D blob from the frame
        Mat blob;
        blobFromImage(frame, blob, 0.007843, Size(inpWidth, inpHeight), Scalar(127.5, 127.5, 127.5), false);
        
        // Set the input to the network
        net.setInput(blob);
        
        // Forward pass to get output
        Mat detection = net.forward();
        
        Mat detectionMat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());
        
        for (int i = 0; i < detectionMat.rows; i++) {
            float confidence = detectionMat.at<float>(i, 2);
            
            if (confidence > confThreshold) {
                int classId = static_cast<int>(detectionMat.at<float>(i, 1));
                
                // Filter for person class (class ID 15 in SSD MobileNet)
                if (classId == 15) {
                    int left = static_cast<int>(detectionMat.at<float>(i, 3) * frame.cols);
                    int top = static_cast<int>(detectionMat.at<float>(i, 4) * frame.rows);
                    int right = static_cast<int>(detectionMat.at<float>(i, 5) * frame.cols);
                    int bottom = static_cast<int>(detectionMat.at<float>(i, 6) * frame.rows);
                    
                    // Draw the bounding box
                    rectangle(frame, Point(left, top), Point(right, bottom), Scalar(0, 0, 255), 2);
                    
                    // Display the label and confidence
                    string label = format("Person: %.2f", confidence);
                    int baseLine;
                    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                    top = max(top, labelSize.height);
                    putText(frame, label, Point(left, top - 5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
                    
                    // Increment people counter
                    peopleCount++;
                }
            }
        }
        
        // Display the people count on the frame
        putText(frame, "People: " + to_string(peopleCount), Point(10, 30), 
              FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
              
        blob.release();
        detection.release();
    } catch (const cv::Exception& e) {
        cerr << "Exception in processCameraFrame: " << e.what() << endl;
    } catch (const std::exception& e) {
        cerr << "Exception in processCameraFrame: " << e.what() << endl;
    } catch (...) {
        cerr << "Unknown exception in processCameraFrame" << endl;
    }
}

void signalHandler(int signum) {
    cout << "Signal " << signum << " received. Terminating..." << endl;
    g_running = false;
}

int main(int argc, char* argv[]) {
    try {
        // Register signal handler
        signal(SIGINT, signalHandler);
        signal(SIGTERM, signalHandler);
        
        // Create named pipe for communication with WebSocket server
        if (access(PIPE_PATH, F_OK) == -1) {
            if (mkfifo(PIPE_PATH, 0666) != 0) {
                cerr << "Failed to create named pipe: " << strerror(errno) << endl;
                return 1;
            }
        }
        
        // Open the pipe in non-blocking mode
        pipe_fd = open(PIPE_PATH, O_WRONLY | O_NONBLOCK);
        if (pipe_fd == -1) {
            cerr << "Failed to open pipe for writing: " << strerror(errno) << endl;
            cerr << "Make sure the WebSocket server is running and listening to the pipe" << endl;
            return 1;
        }
        
        cout << "Connected to named pipe " << PIPE_PATH << endl;
        
        // Initialize people detector
        PeopleDetector detector;
        if (!detector.initialize()) {
            cerr << "Failed to initialize people detector" << endl;
            return 1;
        }
        
        if (!detector.start()) {
            cerr << "Failed to start camera" << endl;
            return 1;
        }
        
        cout << "People detection started. Press Ctrl+C to exit." << endl;
        
        // Main loop
        while (g_running) {
            // Just sleep and let the camera callbacks do the work
            std::this_thread::sleep_for(100ms);
            
            // Check if pipe is still valid
            if (pipe_fd == -1) {
                // Try to reopen the pipe
                pipe_fd = open(PIPE_PATH, O_WRONLY | O_NONBLOCK);
                if (pipe_fd != -1) {
                    cout << "Reconnected to named pipe" << endl;
                }
            }
        }
        
        cout << "Shutting down..." << endl;
        
        // Clean up
        detector.stop();
        
        if (pipe_fd != -1) {
            close(pipe_fd);
        }
        
        cv::destroyAllWindows();
        
        return 0;
    } catch (const cv::Exception& e) {
        cerr << "OpenCV exception in main: " << e.what() << endl;
        return 1;
    } catch (const std::exception& e) {
        cerr << "Exception in main: " << e.what() << endl;
        return 1;
    } catch (...) {
        cerr << "Unknown exception in main" << endl;
        return 1;
    }
}