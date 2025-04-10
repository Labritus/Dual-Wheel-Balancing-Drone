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

using namespace cv;
using namespace cv::dnn;
using namespace std;
using namespace libcamera;
using namespace std::chrono_literals;

// Global variables
atomic<bool> g_running{true};

// Neural network parameters
float confThreshold = 0.5; // Confidence threshold
float nmsThreshold = 0.4;  // Non-maximum suppression threshold
int inpWidth = 300;        // Width of network's input image
int inpHeight = 300;       // Height of network's input image
vector<string> classes;    // Class list

// Function declarations
void processCameraFrame(cv::Mat& frame, Net& net);
void postprocess(cv::Mat& frame, const vector<cv::Mat>& outs);
void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame);
vector<String> getOutputsNames(const Net& net);
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
        
        // Set resolution and format for USB camera
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
    
    // Process captured requests
    void requestComplete(libcamera::Request *request) {
        if (!request || request->status() == libcamera::Request::RequestCancelled) {
            return;
        }
            
        auto buffers = request->buffers();
        if (buffers.empty()) {
            cerr << "No buffers in request" << endl;
            return;
        }
            
        libcamera::FrameBuffer *buffer = buffers.begin()->second;
        if (!buffer) {
            cerr << "Invalid buffer" << endl;
            return;
        }
            
        // Check if buffer is mapped
        if (mMappedBuffers.find(buffer) == mMappedBuffers.end()) {
            cerr << "Buffer not mapped" << endl;
            return;
        }
            
        // Get buffer metadata
        const libcamera::FrameMetadata &metadata = buffer->metadata();
        if (metadata.status != libcamera::FrameMetadata::FrameSuccess) {
            cerr << "Frame error: " << metadata.status << endl;
            return;
        }
            
        // Convert buffer data to OpenCV Mat
        try {
            // Create source Mat from buffer
            cv::Mat sourceFrame;
            
            // Handle different pixel formats
            if (mConfig->at(0).pixelFormat == libcamera::formats::YUYV) {
                // For YUYV format
                cv::Mat yuyv(mConfig->at(0).size.height, mConfig->at(0).size.width, 
                             CV_8UC2, mMappedBuffers[buffer]);
                
                // Convert YUYV to BGR
                cv::cvtColor(yuyv, sourceFrame, cv::COLOR_YUV2BGR_YUYV);
            } else if (mConfig->at(0).pixelFormat == libcamera::formats::MJPEG) {
                // For MJPEG format
                const auto& planes = metadata.planes();
                if (planes.empty()) {
                    cerr << "No planes in metadata" << endl;
                    return;
                }
                const FrameMetadata::Plane& metadataPlane = planes[0];
                std::vector<uint8_t> jpegData(mMappedBuffers[buffer], 
                                             mMappedBuffers[buffer] + metadataPlane.bytesused);
                
                sourceFrame = cv::imdecode(jpegData, cv::IMREAD_COLOR);
                if (sourceFrame.empty()) {
                    cerr << "Failed to decode MJPEG frame" << endl;
                    return;
                }
            } else {
                // Default RGB format
                sourceFrame = cv::Mat(mConfig->at(0).size.height, mConfig->at(0).size.width, 
                                     CV_8UC3, mMappedBuffers[buffer]);
            }
            
            if (sourceFrame.empty()) {
                cerr << "Empty frame" << endl;
                return;
            }
            
            // Process image
            processCameraFrame(sourceFrame, mNet);
            
            // Display image
            cv::imshow("People Detection Test", sourceFrame);
            cv::waitKey(1);
            
            // Requeue this request for reuse
            request->reuse(Request::ReuseBuffers);
            if (g_running) {
                mCamera->queueRequest(request);
            }
        } catch (const std::exception& e) {
            cerr << "Exception in requestComplete: " << e.what() << endl;
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
        // Load class names
        string classesFile = getModelPath("coco.names");
        ifstream ifs(classesFile.c_str());
        if (!ifs.is_open()) {
            cerr << "Could not open classes file: " << classesFile << endl;
            return false;
        }
        
        string line;
        while (getline(ifs, line)) classes.push_back(line);
        
        // Load model
        String modelConfiguration = getModelPath("deploy.prototxt");
        String modelWeights = getModelPath("mobilenet_iter_73000.caffemodel");
        
        try {
            mNet = readNetFromCaffe(modelConfiguration, modelWeights);
            mNet.setPreferableBackend(DNN_BACKEND_OPENCV);
            mNet.setPreferableTarget(DNN_TARGET_CPU);
        } catch (const cv::Exception& e) {
            cerr << "Exception loading model: " << e.what() << endl;
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

    // Create and initialize people detector
    PeopleDetector detector;
    if (!detector.initialize()) {
        cerr << "Failed to initialize detector" << endl;
        return -1;
    }
    
    // Start detection
    detector.start();
    
    return 0;
}

// Signal handling function
void signalHandler(int signum) {
    cout << "Interrupt signal (" << signum << ") received.\n";
    g_running = false;
}

// Process camera frame
void processCameraFrame(cv::Mat& frame, Net& net) {
    // Create 4D blob from frame
    cv::Mat blob;
    cv::dnn::blobFromImage(frame, blob, 1/127.5, cv::Size(inpWidth, inpHeight), cv::Scalar(127.5, 127.5, 127.5), true, false);
    
    // Set network input
    net.setInput(blob);
    
    // Run forward pass to get output of the output layers
    vector<cv::Mat> outs;
    net.forward(outs, getOutputsNames(net));
    
    // Remove bounding boxes with low confidence
    postprocess(frame, outs);
    
    // Display performance information
    vector<double> layersTimes;
    double freq = cv::getTickFrequency() / 1000;
    double t = net.getPerfProfile(layersTimes) / freq;
    string label = cv::format("Inference time: %.2f ms", t);
    cv::putText(frame, label, cv::Point(0, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
}

// Remove low confidence bounding boxes using non-maximum suppression
void postprocess(cv::Mat& frame, const vector<cv::Mat>& outs)
{
    vector<int> classIds;
    vector<float> confidences;
    vector<cv::Rect> boxes;
    
    for (size_t i = 0; i < outs.size(); ++i)
    {
        // Scan through all bounding boxes output from the network and keep only the ones with high confidence scores
        // Assign the box's class label as the class with the highest score
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
    
    // Perform non-maximum suppression to eliminate redundant overlapping boxes with lower confidences
    vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        cv::Rect box = boxes[idx];
        drawPred(classIds[idx], confidences[idx], box.x, box.y,
                 box.x + box.width, box.y + box.height, frame);
    }
}

// Draw predicted bounding box
void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame)
{
    // Only draw person class (classId 0)
    if (classId == 0) {
        // Draw a rectangle displaying the bounding box
        cv::rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(255, 178, 50), 3);
        
        // Get the label for the class name and its confidence
        string label = cv::format("%.2f", conf);
        if (!classes.empty())
        {
            CV_Assert(classId < (int)classes.size());
            label = classes[classId] + ":" + label;
        }
        
        // Display the label at the top of the bounding box
        int baseLine;
        cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        top = max(top, labelSize.height);
        cv::rectangle(frame, cv::Point(left, top - round(1.5*labelSize.height)),
                  cv::Point(left + round(1.5*labelSize.width), top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
        cv::putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,0), 1);
        
        // Log the detection
        cout << "Person detected with confidence: " << conf << endl;
    }
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