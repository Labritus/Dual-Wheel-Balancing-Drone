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
float confThreshold = 0.3; // 降低置信度阈值，提高检测率
float nmsThreshold = 0.5;  // 提高NMS阈值，保留更多检测结果
int inpWidth = 300;        // 恢复为300，与模型设计匹配
int inpHeight = 300;       // 恢复为300，与模型设计匹配
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
    cv::HOGDescriptor hog; // HOG检测器
    
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
        
        // 优化：降低分辨率以减少内存使用
        streamConfig.size.width = 320;   // 降低分辨率
        streamConfig.size.height = 240;  // 降低分辨率
        streamConfig.pixelFormat = libcamera::formats::YUYV;
        streamConfig.bufferCount = 2;    // 减少缓冲区数量以节省内存
        
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
    
    // HOG人员检测函数
    void detectWithHOG(cv::Mat& frame) {
        try {
            if (frame.empty()) {
                cerr << "Error: Frame is empty for HOG detection" << endl;
                return;
            }
            
            // 调整大小以加速检测
            cv::Mat resized;
            cv::resize(frame, resized, cv::Size(320, 240));
            
            // 检测人
            vector<cv::Rect> found, found_filtered;
            hog.detectMultiScale(resized, found, 0, cv::Size(8,8), cv::Size(32,32), 1.05, 2);
            
            cout << "HOG detected " << found.size() << " potential people" << endl;
            
            // 过滤重叠的边界框
            for (size_t i = 0; i < found.size(); i++) {
                cv::Rect r = found[i];
                int j;
                for (j = 0; j < found.size(); j++) 
                    if (j != i && (r & found[j]) == r)
                        break;
                if (j == found.size())
                    found_filtered.push_back(r);
            }
            
            // 绘制边界框
            for (size_t i = 0; i < found_filtered.size(); i++) {
                cv::Rect r = found_filtered[i];
                // 调整回原始图像尺寸
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
            
            // 检查像素格式
            libcamera::PixelFormat pixelFormat = mConfig->at(0).pixelFormat;
            cv::Mat frame;
            
            if (pixelFormat == libcamera::formats::YUYV) {
                // 创建YUYV格式的Mat
                cv::Mat yuyv(mConfig->at(0).size.height, mConfig->at(0).size.width, 
                           CV_8UC2, mMappedBuffers[buffer]);
                
                // 转换YUYV到RGB
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
                // 直接使用RGB数据
                frame = cv::Mat(mConfig->at(0).size.height, mConfig->at(0).size.width, 
                              CV_8UC3, mMappedBuffers[buffer]);
            } else {
                cerr << "Unsupported pixel format" << endl;
                goto requeue;
            }
            
            // 新增：保存一帧用于测试（仅第一次）
            static bool savedTestFrame = false;
            if (!savedTestFrame) {
                cv::imwrite("/home/pu/camera_test.jpg", frame);
                cout << "Saved test frame to /home/pu/camera_test.jpg" << endl;
                savedTestFrame = true;
            }
            
            // 安全处理图像
            try {
                // 在处理前检查图像是否有效
                if (frame.empty() || frame.rows <= 0 || frame.cols <= 0) {
                    cerr << "Invalid frame: empty or has invalid dimensions" << endl;
                } else {
                    // 首先尝试使用DNN模型检测
                    processCameraFrame(frame, mNet);
                    
                    // 然后使用HOG检测器作为备选
                    detectWithHOG(frame);
                    
                    // 显示图像
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
            
            // 显式释放内存
            frame.release();
            
        } catch (const std::exception& e) {
            cerr << "Exception in requestComplete: " << e.what() << endl;
        } catch (...) {
            cerr << "Unknown exception in requestComplete" << endl;
        }
        
    requeue:
        // 重用此请求
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
        // 初始化HOG检测器
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
        
        // 新增：验证第一个类别是否为"person"
        if (!classes.empty()) {
            cout << "First class (index 0) is: " << classes[0] << endl;
        }
        
        // Load model
        String modelConfiguration = getModelPath("deploy.prototxt");
        String modelWeights = getModelPath("mobilenet_iter_73000.caffemodel");
        
        cout << "Loading model configuration from: " << modelConfiguration << endl;
        cout << "Loading model weights from: " << modelWeights << endl;
        
        try {
            // 检查文件是否存在
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
            
            // 获取文件大小
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
            
            // 修正：正确加载和配置网络
            mNet = cv::dnn::readNetFromCaffe(modelConfiguration, modelWeights);
            if (mNet.empty()) {
                cerr << "Failed to load network" << endl;
                return false;
            }
            
            // 设置后端和目标以优化性能
            mNet.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            mNet.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            
            // 新增：打印网络层信息
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

// 优化后的图像处理函数
void processCameraFrame(cv::Mat& frame, Net& net) {
    try {
        // 添加额外的检查
        if (net.empty()) {
            cerr << "Error: Neural network is empty" << endl;
            return;
        }
        
        if (frame.empty()) {
            cerr << "Error: Frame is empty" << endl;
            return;
        }
        
        cout << "Processing frame, size: " << frame.cols << "x" << frame.rows << endl;
        
        // 创建4D blob - 修改预处理参数
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
        
        // 设置网络输入
        try {
            net.setInput(blob);
        } catch (const cv::Exception& e) {
            cerr << "Exception in setInput: " << e.what() << endl;
            blob.release(); // 显式释放内存
            return;
        }
        
        // 运行前向传播
        vector<cv::Mat> outs;
        try {
            // 获取输出层名称
            vector<String> outNames = getOutputsNames(net);
            if (outNames.empty()) {
                cerr << "Error: Failed to get output layer names" << endl;
                blob.release(); // 显式释放内存
                return;
            }
            
            // 运行前向传播
            net.forward(outs, outNames);
            
            if (outs.empty()) {
                cerr << "Error: Network produced no outputs" << endl;
                blob.release(); // 显式释放内存
                return;
            }
            
            // 新增：输出网络输出的形状
            cout << "Network output shape: rows=" << outs[0].rows 
                 << ", cols=" << outs[0].cols << endl;
            
            // 新增：打印第一行数据进行调试
            if (outs[0].rows > 0) {
                float* data = (float*)outs[0].data;
                cout << "First detection data: ";
                for (int i = 0; i < min(10, outs[0].cols); i++) {
                    cout << data[i] << " ";
                }
                cout << endl;
            }
            
        } catch (const cv::Exception& e) {
            cerr << "Exception in forward: " << e.what() << endl;
            blob.release(); // 显式释放内存
            return;
        }
        
        // 移除低置信度边界框
        try {
            postprocess(frame, outs);
        } catch (const cv::Exception& e) {
            cerr << "Exception in postprocess: " << e.what() << endl;
        }
        
        // 显示性能信息
        try {
            vector<double> layersTimes;
            double freq = getTickFrequency() / 1000;
            double t = net.getPerfProfile(layersTimes) / freq;
            string label = format("Inference time: %.2f ms", t);
            putText(frame, label, cv::Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
        } catch (const cv::Exception& e) {
            cerr << "Exception in performance display: " << e.what() << endl;
        }
        
        // 显式释放内存
        blob.release();
        for(auto& out : outs) {
            out.release();
        }
        outs.clear();
        
        cout << "Frame processed successfully" << endl;
        
    } catch (const cv::Exception& e) {
        cerr << "Exception in processCameraFrame: " << e.what() << endl;
    } catch (const std::exception& e) {
        cerr << "Exception in processCameraFrame: " << e.what() << endl;
    } catch (...) {
        cerr << "Unknown exception in processCameraFrame" << endl;
    }
}

// 修复的后处理函数
void postprocess(cv::Mat& frame, const vector<cv::Mat>& outs)
{
    vector<int> classIds;
    vector<float> confidences;
    vector<cv::Rect> boxes;
    
    cout << "Network outputs size: " << outs.size() << endl;
    
    for (size_t i = 0; i < outs.size(); ++i)
    {
        // 扫描网络输出的所有边界框，只保留置信度高的
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            cv::Point classIdPoint;
            double confidence;
            
            // 获取最高分值及其位置
            cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            
            // 新增：输出所有潜在检测结果，用于调试
            if (confidence > 0.01) {  // 使用非常低的阈值进行调试输出
                cout << "Found potential detection: class=" << classIdPoint.x 
                     << ", confidence=" << confidence << endl;
            }
            
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
    
    // NMS之前的候选数量
    cout << "Candidates before NMS: " << classIds.size() << endl;
    
    // 使用非极大值抑制去除低置信度的重叠边界框
    vector<int> indices;
    
    // 根据 OpenCV 4.9.0 版本的 NMSBoxes 使用方式
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    
    cout << "Detections after NMS: " << indices.size() << endl;
    
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        cout << "  Detected class " << classIds[idx] << " (";
        if (classIds[idx] < classes.size()) {
            cout << classes[classIds[idx]];
        } else {
            cout << "unknown";
        }
        cout << ") with confidence " << confidences[idx] << endl;
        
        cv::Rect box = boxes[idx];
        drawPred(classIds[idx], confidences[idx], box.x, box.y,
                 box.x + box.width, box.y + box.height, frame);
    }
}

// 修复的绘制预测函数 - 不再限制只显示 person 类
void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame)
{
    // 绘制边界框
    cv::rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(255, 178, 50), 3);
    
    // 获取类名和置信度标签
    string label = cv::format("%.2f", conf);
    if (!classes.empty() && classId < (int)classes.size())
    {
        label = classes[classId] + ":" + label;
    }
    
    // 在边界框顶部显示标签
    int baseLine;
    cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = max(top, labelSize.height);
    cv::rectangle(frame, cv::Point(left, top - round(1.5*labelSize.height)),
              cv::Point(left + round(1.5*labelSize.width), top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
    cv::putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,0), 1);
    
    // 输出检测日志
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