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

// OpenCV related
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

// Libcamera related
#include <libcamera/libcamera.h>

// Platform detection
#if defined(__linux__)
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#else
// Windows dummy I2C implementation
#define O_RDWR 0
#define I2C_SLAVE 0
int open(const char*, int) { return 1; }
int ioctl(int, int, int) { return 0; }
int close(int) { return 0; }
ssize_t write(int, const void*, size_t) { return 0; }
#endif

using namespace cv;
using namespace cv::dnn;
using namespace std;
using namespace libcamera;
using namespace std::chrono_literals;

// Global variables
atomic<bool> g_running{true};
atomic<bool> g_personDetected{false};

// Neural network parameters
float confThreshold = 0.3; // Lower threshold for better detection rate
float nmsThreshold = 0.5;  // Higher NMS threshold to keep more detection results
int inpWidth = 300;        // Width of network's input image
int inpHeight = 300;       // Height of network's input image
vector<string> classes;    // Class list

// I2C configuration
const char *I2C_DEVICE = "/dev/i2c-1";
const int STM32_ADDRESS = 0x08; // STM32 I2C address
int i2cFile;

// Function declarations
void processCameraFrame(cv::Mat& frame, Net& net);
void postprocess(cv::Mat& frame, const vector<cv::Mat>& outs);
void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame);
vector<String> getOutputsNames(const Net& net);
void sendI2CMessage(const string& message);
void signalHandler(int signum);

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
    cv::HOGDescriptor hog; // HOG detector for backup
    
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
        
        // Optimize: lower resolution to reduce memory usage
        streamConfig.size.width = 320;
        streamConfig.size.height = 240;
        streamConfig.pixelFormat = libcamera::formats::YUYV;
        streamConfig.bufferCount = 2;
        
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
    void detectWithHOG(cv::Mat& frame) {
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
                
                // Send I2C message when person is detected with HOG
                if (!g_personDetected) {
                    string message = "PERSON_DETECTED";
                    sendI2CMessage(message);
                    g_personDetected = true;
                }
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
            
            // Save a test frame (only once)
            static bool savedTestFrame = false;
            if (!savedTestFrame) {
                cv::imwrite("/home/pu/camera_test.jpg", frame);
                cout << "Saved test frame to /home/pu/camera_test.jpg" << endl;
                savedTestFrame = true;
            }
            
            // Reset person detection flag for each frame
            g_personDetected = false;
            
            // Safely process image
            try {
                // Check if frame is valid
                if (frame.empty() || frame.rows <= 0 || frame.cols <= 0) {
                    cerr << "Invalid frame: empty or has invalid dimensions" << endl;
                } else {
                    // First try using DNN model
                    processCameraFrame(frame, mNet);
                    
                    // Then use HOG detector as backup
                    detectWithHOG(frame);
                    
                    // Display image
                    cv::imshow("People Detection", frame);
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
        // Reuse this request
        request->reuse(Request::ReuseBuffers);
        if (g_running) {
            mCamera->queueRequest(request);
        }
    }

public:
    PeopleDetector() : mAllocator(nullptr) {}
    
    ~PeopleDetector() {
        // Stop the camera
        if (mCamera) {
            mCamera->stop();
            mCamera->release();
        }
        
        // Release buffers
        for (auto &p : mMappedBuffers) {
            munmap(p.second, mConfig->at(0).frameSize);
        }
        mMappedBuffers.clear();
        
        // Free allocator
        delete mAllocator;
        
        // Clean up camera manager
        if (mCameraManager)
            mCameraManager->stop();
    }

    bool initialize() {
        // Initialize HOG detector
        hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
        cout << "HOG detector initialized" << endl;
        
        // Load class names
        string classesFile = getModelPath("coco.names");
        ifstream ifs(classesFile.c_str());
        if (!ifs.is_open()) {
            cerr << "Could not open classes file: " << classesFile << endl;
            return false;
        }
        
        string line;
        while (getline(ifs, line)) classes.push_back(line);
        cout << "Loaded " << classes.size() << " class names" << endl;
        
        // Verify first class is "person"
        if (!classes.empty()) {
            cout << "First class (index 0) is: " << classes[0] << endl;
        }
        
        // Load model
        String modelConfiguration = getModelPath("deploy.prototxt");
        String modelWeights = getModelPath("mobilenet_iter_73000.caffemodel");
        
        cout << "Loading model configuration from: " << modelConfiguration << endl;
        cout << "Loading model weights from: " << modelWeights << endl;
        
        try {
            // Check if files exist
            ifstream configFile(modelConfiguration);
            ifstream weightsFile(modelWeights);
            
            if (!configFile.is_open()) {
                cerr << "Could not open model configuration file: " << modelConfiguration << endl;
                return false;
            }
            if (!weightsFile.is_open()) {
                cerr << "Could not open model weights file: " << modelWeights << endl;
                return false;
            }
            
            // Get file sizes
            configFile.seekg(0, ios::end);
            size_t configSize = configFile.tellg();
            weightsFile.seekg(0, ios::end);
            size_t weightsSize = weightsFile.tellg();
            
            cout << "Model configuration file size: " << configSize << " bytes" << endl;
            cout << "Model weights file size: " << weightsSize << " bytes" << endl;
            
            if (configSize == 0 || weightsSize == 0) {
                cerr << "Model files are empty" << endl;
                return false;
            }
            
            // Properly load and configure network
            mNet = cv::dnn::readNetFromCaffe(modelConfiguration, modelWeights);
            if (mNet.empty()) {
                cerr << "Failed to load network" << endl;
                return false;
            }
            
            // Set backend and target to optimize performance
            mNet.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            mNet.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            
            // Print network layer information
            vector<String> layerNames = mNet.getLayerNames();
            cout << "Network has " << layerNames.size() << " layers" << endl;
            
            cout << "Network loaded successfully" << endl;
        } catch (const cv::Exception& e) {
            cerr << "Exception loading network: " << e.what() << endl;
            return false;
        }
        
        // Setup camera
        if (!setupCamera()) {
            return false;
        }
        
        // Map buffers
        for (StreamConfiguration &cfg : *mConfig) {
            Stream *stream = cfg.stream();
            if (!stream) {
                cerr << "Invalid stream" << endl;
                return false;
            }
            
            const std::vector<std::unique_ptr<FrameBuffer>> &buffers = mAllocator->buffers(stream);
            if (buffers.empty()) {
                cerr << "No buffers allocated for stream" << endl;
                return false;
            }
            
            for (unsigned int i = 0; i < buffers.size(); ++i) {
                const std::unique_ptr<FrameBuffer> &buffer = buffers[i];
                if (!buffer) {
                    cerr << "Invalid buffer" << endl;
                    return false;
                }
                
                // Map buffer memory
                const FrameBuffer::Plane &plane = buffer->planes()[0];
                void *memory = mmap(NULL, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
                if (memory == MAP_FAILED) {
                    cerr << "Failed to mmap buffer: " << strerror(errno) << endl;
                    return false;
                }
                
                mMappedBuffers[buffer.get()] = static_cast<uint8_t*>(memory);
                
                // Create request
                std::unique_ptr<Request> request = mCamera->createRequest();
                if (!request) {
                    cerr << "Failed to create request" << endl;
                    return false;
                }
                
                int ret = request->addBuffer(stream, buffer.get());
                if (ret < 0) {
                    cerr << "Failed to add buffer to request: " << ret << endl;
                    return false;
                }
                
                mRequests.push_back(std::move(request));
            }
        }
        
        cout << "Initialization complete" << endl;
        return true;
    }
    
    void start() {
        // Set request completed callback
        mCamera->requestCompleted.connect(this, &PeopleDetector::requestComplete);
        
        // Start camera
        int ret = mCamera->start();
        if (ret) {
            cerr << "Failed to start camera: " << ret << endl;
            return;
        }
        
        // Queue all requests
        for (std::unique_ptr<Request> &request : mRequests) {
            mCamera->queueRequest(request.release());
        }
        
        cout << "Camera started successfully" << endl;
        
        // Main loop - keep application running until signal is received
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // Stop camera
        mCamera->stop();
    }
};

// Main function
int main(int argc, char** argv)
{
    // Set up signal handling
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // Check OpenCV version
    cout << "OpenCV version: " << CV_VERSION << endl;

    // Initialize I2C
#if defined(__linux__)
    if ((i2cFile = open(I2C_DEVICE, O_RDWR)) < 0) {
        cerr << "Failed to open the I2C bus" << endl;
        return -1;
    }
    if (ioctl(i2cFile, I2C_SLAVE, STM32_ADDRESS) < 0) {
        cerr << "Failed to acquire bus access and/or talk to slave" << endl;
        return -1;
    }
#else
    cout << "[I2C Simulation] Initializing I2C interface" << endl;
    i2cFile = 1;
#endif

    // Create and initialize people detector
    PeopleDetector detector;
    if (!detector.initialize()) {
        cerr << "Failed to initialize detector" << endl;
        close(i2cFile);
        return -1;
    }
    
    // Start detection
    detector.start();
    
    close(i2cFile); // Close I2C connection
    return 0;
}

// Signal handling function
void signalHandler(int signum) {
    cout << "Interrupt signal (" << signum << ") received.\n";
    g_running = false;
}

// Optimized frame processing function
void processCameraFrame(cv::Mat& frame, Net& net) {
    try {
        // Additional checks
        if (net.empty()) {
            cerr << "Error: Neural network is empty" << endl;
            return;
        }
        
        if (frame.empty()) {
            cerr << "Error: Frame is empty" << endl;
            return;
        }
        
        cout << "Processing frame, size: " << frame.cols << "x" << frame.rows << endl;
        
        // Create 4D blob - modified preprocessing parameters
        cv::Mat blob;
        try {
            blobFromImage(frame, blob, 0.007843f, cv::Size(inpWidth, inpHeight), 
                          cv::Scalar(127.5, 127.5, 127.5), false, false);
            
            if (blob.empty()) {
                cerr << "Error: Failed to create blob from image" << endl;
                return;
            }
        } catch (const cv::Exception& e) {
            cerr << "Exception in blobFromImage: " << e.what() << endl;
            return;
        }
        
        // Set network input
        try {
            net.setInput(blob);
        } catch (const cv::Exception& e) {
            cerr << "Exception in setInput: " << e.what() << endl;
            blob.release(); // Explicitly release memory
            return;
        }
        
        // Run forward pass
        vector<cv::Mat> outs;
        try {
            // Get output layer names
            vector<String> outNames = getOutputsNames(net);
            if (outNames.empty()) {
                cerr << "Error: Failed to get output layer names" << endl;
                blob.release(); // Explicitly release memory
                return;
            }
            
            // Run forward pass
            net.forward(outs, outNames);
            
            if (outs.empty()) {
                cerr << "Error: Network produced no outputs" << endl;
                blob.release(); // Explicitly release memory
                return;
            }
            
            // Output network output shape for debugging
            cout << "Network output shape: rows=" << outs[0].rows 
                 << ", cols=" << outs[0].cols << endl;
            
        } catch (const cv::Exception& e) {
            cerr << "Exception in forward: " << e.what() << endl;
            blob.release(); // Explicitly release memory
            return;
        }
        
        // Remove low confidence bounding boxes
        try {
            postprocess(frame, outs);
        } catch (const cv::Exception& e) {
            cerr << "Exception in postprocess: " << e.what() << endl;
        }
        
        // Display performance information
        try {
            vector<double> layersTimes;
            double freq = getTickFrequency() / 1000;
            double t = net.getPerfProfile(layersTimes) / freq;
            string label = format("Inference time: %.2f ms", t);
            putText(frame, label, cv::Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
        } catch (const cv::Exception& e) {
            cerr << "Exception in performance display: " << e.what() << endl;
        }
        
        // Explicitly release memory
        blob.release();
        for(auto& out : outs) {
            out.release();
        }
        outs.clear();
        
    } catch (const cv::Exception& e) {
        cerr << "Exception in processCameraFrame: " << e.what() << endl;
    } catch (const std::exception& e) {
        cerr << "Exception in processCameraFrame: " << e.what() << endl;
    } catch (...) {
        cerr << "Unknown exception in processCameraFrame" << endl;
    }
}

// Fixed postprocessing function
void postprocess(cv::Mat& frame, const vector<cv::Mat>& outs)
{
    vector<int> classIds;
    vector<float> confidences;
    vector<cv::Rect> boxes;
    
    cout << "Network outputs size: " << outs.size() << endl;
    
    for (size_t i = 0; i < outs.size(); ++i)
    {
        // Scan through all bounding boxes output from the network and keep only high confidence ones
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            cv::Point classIdPoint;
            double confidence;
            
            // Get the value and location of the maximum score
            cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            
            if (confidence > confThreshold)
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;
                
                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }
    }
    
    // Candidates before NMS
    cout << "Candidates before NMS: " << classIds.size() << endl;
    
    // Perform non-maximum suppression
    vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    
    cout << "Detections after NMS: " << indices.size() << endl;
    
    // Reset person detection flag
    g_personDetected = false;
    
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        int classId = classIds[idx];
        
        cout << "  Detected class " << classId << " (";
        if (classId < classes.size()) {
            cout << classes[classId];
        } else {
            cout << "unknown";
        }
        cout << ") with confidence " << confidences[idx] << endl;
        
        cv::Rect box = boxes[idx];
        drawPred(classId, confidences[idx], box.x, box.y,
                 box.x + box.width, box.y + box.height, frame);
    }
}

// Draw predicted bounding box
void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame)
{
    // Draw a rectangle displaying the bounding box
    rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(255, 178, 50), 3);
    
    // Get the label for the class name and its confidence
    string label = format("%.2f", conf);
    if (!classes.empty() && classId < (int)classes.size())
    {
        label = classes[classId] + ":" + label;
    }
    
    // Display the label at the top of the bounding box
    int baseLine;
    cv::Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = max(top, labelSize.height);
    rectangle(frame, cv::Point(left, top - round(1.5*labelSize.height)),
              cv::Point(left + round(1.5*labelSize.width), top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
    putText(frame, label, cv::Point(left, top), FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,0), 1);
    
    // Send I2C message when person is detected (only for person class which is 0)
    if (classId == 0 && !g_personDetected) {
        string message = "PERSON_DETECTED";
        sendI2CMessage(message);
        g_personDetected = true;
    }
    
    // Output detection log
    cout << "Drew detection: ";
    if (classId < (int)classes.size()) {
        cout << classes[classId];
    } else {
        cout << "unknown class " << classId;
    }
    cout << " with confidence: " << conf << endl;
}

// Get the names of the output layers
vector<String> getOutputsNames(const Net& net)
{
    static vector<String> names;
    if (names.empty())
    {
        // Get the indices of the output layers, i.e. the layers with unconnected outputs
        vector<int> outLayers = net.getUnconnectedOutLayers();
        
        // Get the names of all the layers in the network
        vector<String> layersNames = net.getLayerNames();
        
        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
            names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}

// Function to send I2C message
void sendI2CMessage(const string& message) {
#if defined(__linux__)
    if (write(i2cFile, message.c_str(), message.length()) != message.length()) {
        cerr << "Failed to write to the I2C bus" << endl;
    } else {
        cout << "Sent I2C message: " << message << endl;
    }
#else
    cout << "[I2C Simulation] Sending message: " << message << endl;
#endif