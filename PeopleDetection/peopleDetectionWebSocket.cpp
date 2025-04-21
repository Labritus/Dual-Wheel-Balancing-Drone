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