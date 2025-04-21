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
#include <mutex>
#include <condition_variable>

// OpenCV related
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

// Libcamera related
#include <libcamera/libcamera.h>

// HTTP server related
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <fcntl.h>

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
atomic<int> g_numClients{0};
mutex g_frameMutex;
condition_variable g_frameCV;
cv::Mat g_currentFrame;
bool g_frameReady = false;

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

// HTTP boundary for MJPEG stream
const string boundary = "mjpegboundary";

// Function declarations
void processCameraFrame(cv::Mat& frame, Net& net);
void postprocess(cv::Mat& frame, const vector<cv::Mat>& outs);
void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame);
vector<String> getOutputsNames(const Net& net);
void sendI2CMessage(const string& message);
void signalHandler(int signum);
void httpServerThread();
void handleClient(int clientSocket);

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
        
        // Optimize: use higher resolution for better streaming quality
        streamConfig.size.width = 640;
        streamConfig.size.height = 480;
        streamConfig.pixelFormat = libcamera::formats::YUYV;
        streamConfig.bufferCount = 4;
        
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
                    
                    // Add client connection info
                    std::string stats = "Clients: " + std::to_string(g_numClients);
                    cv::putText(frame, stats, cv::Point(10, frame.rows - 10), 
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                    
                    // Update the global frame for streaming
                    {
                        std::lock_guard<std::mutex> lock(g_frameMutex);
                        frame.copyTo(g_currentFrame);
                        g_frameReady = true;
                    }
                    g_frameCV.notify_all();
                    
                    // Display image locally if needed
                    if (false) { // Set to true to show local window
                        cv::imshow("People Detection", frame);
                        cv::waitKey(1);
                    }
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

// HTTP server implementation
void handleClient(int clientSocket) {
    char buffer[1024] = {0};
    ssize_t bytesRead = read(clientSocket, buffer, sizeof(buffer) - 1);
    
    if (bytesRead <= 0) {
        close(clientSocket);
        return;
    }
    
    string requestData(buffer);
    cout << "Received request: " << requestData.substr(0, requestData.find('\n')) << endl;
    
    // Check if this is a request for the stream or for a static file
    if (requestData.find("GET /stream") != string::npos) {
        // Increment client counter
        g_numClients++;
        
        // Send MJPEG stream headers
        string headers = "HTTP/1.1 200 OK\r\n";
        headers += "Content-Type: multipart/x-mixed-replace; boundary=" + boundary + "\r\n";
        headers += "Cache-Control: no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0\r\n";
        headers += "Pragma: no-cache\r\n";
        headers += "Connection: close\r\n";
        headers += "\r\n";
        send(clientSocket, headers.c_str(), headers.size(), 0);
        
        // Keep sending frames until connection closes or program stops
        while (g_running) {
            cv::Mat frame;
            
            // Wait for a new frame
            {
                std::unique_lock<std::mutex> lock(g_frameMutex);
                g_frameCV.wait(lock, [] { return g_frameReady; });
                if (!g_currentFrame.empty()) {
                    g_currentFrame.copyTo(frame);
                }
                g_frameReady = false;
            }
            
            if (frame.empty()) {
                continue;
            }
            
            // Encode the frame as JPEG
            vector<uchar> buffer;
            cv::imencode(".jpg", frame, buffer, {cv::IMWRITE_JPEG_QUALITY, 80});
            
            // Create MJPEG frame
            string frameHeader = "--" + boundary + "\r\n";
            frameHeader += "Content-Type: image/jpeg\r\n";
            frameHeader += "Content-Length: " + to_string(buffer.size()) + "\r\n";
            frameHeader += "\r\n";
            
            // Send the frame header
            if (send(clientSocket, frameHeader.c_str(), frameHeader.length(), 0) <= 0) {
                break;
            }
            
            // Send the actual JPEG data
            if (send(clientSocket, buffer.data(), buffer.size(), 0) <= 0) {
                break;
            }
            
            // Send boundary delimiter
            string delimiter = "\r\n";
            if (send(clientSocket, delimiter.c_str(), delimiter.length(), 0) <= 0) {
                break;
            }
            
            // Free memory
            frame.release();
            buffer.clear();
            
            // Small delay to manage frame rate
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
        
        // Decrement client counter on disconnect
        g_numClients--;
    }
    else if (requestData.find("GET / ") != string::npos || 
             requestData.find("GET /index.html") != string::npos) {
        // Print current working directory for debugging
        char cwd[1024];
        if (getcwd(cwd, sizeof(cwd)) != NULL) {
            cout << "Current working directory: " << cwd << endl;
        }
        
        // Try to read from both current directory and absolute path
        ifstream file("index.html", ios::binary | ios::ate);
        if (!file) {
            cout << "Couldn't find index.html in current directory, using embedded HTML" << endl;
            // If file doesn't exist, send an embedded HTML response
            string basicHtml =
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: text/html\r\n"
                "Connection: close\r\n"
                "\r\n"
                "<!DOCTYPE html>\n"
                "<html lang=\"en\">\n"
                "<head>\n"
                "  <meta charset=\"UTF-8\" />\n"
                "  <title>People Detection Stream</title>\n"
                "  <style>\n"
                "    body { font-family: Arial, sans-serif; text-align: center; padding: 20px; background-color: #f0f0f0; }\n"
                "    .container { max-width: 800px; margin: 0 auto; background-color: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }\n"
                "    h1 { color: #333; margin-bottom: 20px; }\n"
                "    .video-container { margin: 20px auto; width: 100%; max-width: 640px; height: auto; position: relative; }\n"
                "    #videoStream { width: 100%; height: auto; border: 3px solid #ddd; border-radius: 5px; }\n"
                "    .status { margin: 15px 0; padding: 10px; background-color: #e7f4e4; border-radius: 5px; font-weight: bold; }\n"
                "    .controls { margin-top: 20px; }\n"
                "    button { background-color: #4CAF50; border: none; color: white; padding: 10px 20px; text-align: center; display: inline-block; font-size: 16px; margin: 4px 2px; cursor: pointer; border-radius: 4px; }\n"
                "    button:hover { background-color: #45a049; }\n"
                "    button:disabled { background-color: #cccccc; cursor: not-allowed; }\n"
                "    .detection-info { margin-top: 15px; text-align: left; padding: 10px; background-color: #f9f9f9; border-radius: 5px; font-size: 14px; }\n"
                "  </style>\n"
                "</head>\n"
                "<body>\n"
                "  <div class=\"container\">\n"
                "    <h1>People Detection Live Stream</h1>\n"
                "    <div class=\"status\" id=\"status\">Stream Status: Disconnected</div>\n"
                "    <div class=\"video-container\">\n"
                "      <img id=\"videoStream\" src=\"\" alt=\"Video stream not available\">\n"
                "    </div>\n"
                "    <div class=\"controls\">\n"
                "      <button id=\"startBtn\" onclick=\"startStream()\">Start Stream</button>\n"
                "      <button id=\"stopBtn\" onclick=\"stopStream()\" disabled>Stop Stream</button>\n"
                "    </div>\n"
                "    <div class=\"detection-info\">\n"
                "      <h3>Detection Information:</h3>\n"
                "      <p>This stream analyzes video frames in real-time to identify and highlight people in the scene.</p>\n"
                "    </div>\n"
                "  </div>\n"
                "  <script>\n"
                "    let streaming = false;\n"
                "    const videoElement = document.getElementById('videoStream');\n"
                "    const statusElement = document.getElementById('status');\n"
                "    const startButton = document.getElementById('startBtn');\n"
                "    const stopButton = document.getElementById('stopBtn');\n"
                "    \n"
                "    function startStream() {\n"
                "      if (!streaming) {\n"
                "        videoElement.src = '/stream';\n"
                "        videoElement.onerror = function() {\n"
                "          statusElement.textContent = 'Stream Status: Error connecting to the stream';\n"
                "          statusElement.style.backgroundColor = '#ffdddd';\n"
                "          stopStream();\n"
                "        };\n"
                "        \n"
                "        videoElement.onload = function() {\n"
                "          statusElement.textContent = 'Stream Status: Connected';\n"
                "          statusElement.style.backgroundColor = '#d4edda';\n"
                "          streaming = true;\n"
                "          startButton.disabled = true;\n"
                "          stopButton.disabled = false;\n"
                "        };\n"
                "      }\n"
                "    }\n"
                "    \n"
                "    function stopStream() {\n"
                "      if (streaming) {\n"
                "        videoElement.src = '';\n"
                "        statusElement.textContent = 'Stream Status: Disconnected';\n"
                "        statusElement.style.backgroundColor = '#e7f4e4';\n"
                "        streaming = false;\n"
                "        startButton.disabled = false;\n"
                "        stopButton.disabled = true;\n"
                "      }\n"
                "    }\n"
                "    \n"
                "    videoElement.addEventListener('error', function() {\n"
                "      if (streaming) {\n"
                "        statusElement.textContent = 'Stream Status: Connection lost. Attempting to reconnect...';\n"
                "        statusElement.style.backgroundColor = '#fff3cd';\n"
                "        setTimeout(startStream, 3000);\n"
                "      }\n"
                "    });\n"
                "    \n"
                "    window.onload = function() {\n"
                "      setTimeout(startStream, 1000);\n"
                "    };\n"