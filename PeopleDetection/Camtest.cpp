#include <iostream>
#include <chrono>
#include <thread>
#include <signal.h>

#include <libcamera/libcamera.h>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;
using namespace libcamera;

// Global variables
std::atomic<bool> g_running{true};

// Signal handling function
void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    g_running = false;
}

class CameraTest {
private:
    std::unique_ptr<CameraManager> mCameraManager;
    std::shared_ptr<Camera> mCamera;
    std::unique_ptr<CameraConfiguration> mConfig;
    FrameBufferAllocator *mAllocator;
    std::vector<std::unique_ptr<Request>> mRequests;
    std::map<FrameBuffer *, uint8_t*> mMappedBuffers;

    bool setupCamera() {
        mCameraManager = std::make_unique<CameraManager>();
        int ret = mCameraManager->start();
        if (ret) {
            std::cerr << "Failed to start camera manager: " << ret << std::endl;
            return false;
        }

        // Get the first available camera
        auto cameras = mCameraManager->cameras();
        if (cameras.empty()) {
            std::cerr << "No cameras available" << std::endl;
            return false;
        }
        
        mCamera = cameras[0];
        ret = mCamera->acquire();
        if (ret) {
            std::cerr << "Failed to acquire camera: " << ret << std::endl;
            return false;
        }
        
        // Configure camera parameters
        mConfig = mCamera->generateConfiguration({StreamRole::Viewfinder});
        if (!mConfig) {
            std::cerr << "Failed to generate camera configuration" << std::endl;
            return false;
        }
        
        StreamConfiguration &streamConfig = mConfig->at(0);
        std::cout << "Default viewfinder configuration: " << streamConfig.toString() << std::endl;
        
        // Set resolution and format
        streamConfig.size.width = 640;
        streamConfig.size.height = 480;
        streamConfig.pixelFormat = libcamera::formats::RGB888;
        streamConfig.bufferCount = 4;
        
        ret = mConfig->validate();
        if (ret) {
            std::cerr << "Failed to validate camera configuration: " << ret << std::endl;
            return false;
        }
        
        ret = mCamera->configure(mConfig.get());
        if (ret) {
            std::cerr << "Failed to configure camera: " << ret << std::endl;
            return false;
        }
        
        std::cout << "Camera configured successfully" << std::endl;
        
        // Create frame buffer allocator
        mAllocator = new FrameBufferAllocator(mCamera);
        
        for (StreamConfiguration &cfg : *mConfig) {
            ret = mAllocator->allocate(cfg.stream());
            if (ret < 0) {
                std::cerr << "Failed to allocate buffers for stream" << std::endl;
                return false;
            }
            
            const std::vector<std::unique_ptr<FrameBuffer>> &buffers = mAllocator->buffers(cfg.stream());
            std::cout << "Allocated " << buffers.size() << " buffers for stream" << std::endl;
        }
        
        return true;
    }
    
    // Process captured requests
    void requestComplete(Request *request) {
        if (request->status() == Request::RequestCancelled)
            return;
            
        FrameBuffer *buffer = request->buffers().begin()->second;
        const FrameMetadata &metadata = buffer->metadata();
        
        // Convert buffer data to OpenCV Mat
        cv::Mat frame(mConfig->at(0).size.height, mConfig->at(0).size.width, 
                      CV_8UC3, mMappedBuffers[buffer]);
        
        // Display image
        cv::imshow("Camera Test", frame);
        cv::waitKey(1);
        
        // Requeue this request for reuse
        request->reuse(Request::ReuseBuffers);
        if (g_running) {
            mCamera->queueRequest(request);
        }
    }

public:
    CameraTest() : mAllocator(nullptr) {}
    
    ~CameraTest() {
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
        // Setup camera
        if (!setupCamera()) {
            return false;
        }
        
        // Map buffers
        for (StreamConfiguration &cfg : *mConfig) {
            Stream *stream = cfg.stream();
            const std::vector<std::unique_ptr<FrameBuffer>> &buffers = mAllocator->buffers(stream);
            
            for (unsigned int i = 0; i < buffers.size(); ++i) {
                const std::unique_ptr<FrameBuffer> &buffer = buffers[i];
                
                // Map buffer memory
                const FrameBuffer::Plane &plane = buffer->planes()[0];
                void *memory = mmap(NULL, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
                if (memory == MAP_FAILED) {
                    std::cerr << "Failed to mmap buffer" << std::endl;
                    return false;
                }
                
                mMappedBuffers[buffer.get()] = static_cast<uint8_t*>(memory);
                
                // Create request
                std::unique_ptr<Request> request = mCamera->createRequest();
                if (!request) {
                    std::cerr << "Failed to create request" << std::endl;
                    return false;
                }
                
                int ret = request->addBuffer(stream, buffer.get());
                if (ret < 0) {
                    std::cerr << "Failed to add buffer to request" << std::endl;
                    return false;
                }
                
                mRequests.push_back(std::move(request));
            }
        }
        
        std::cout << "Initialization complete" << std::endl;
        return true;
    }
    
    void start() {
        // Set request completed callback
        mCamera->requestCompleted.connect(this, &CameraTest::requestComplete);
        
        // Start camera
        int ret = mCamera->start();
        if (ret) {
            std::cerr << "Failed to start camera: " << ret << std::endl;
            return;
        }
        
        // Queue all requests
        for (std::unique_ptr<Request> &request : mRequests) {
            mCamera->queueRequest(request.release());
        }
        
        std::cout << "Camera started successfully" << std::endl;
        
        // Main loop - keep application running until signal is received
        while (g_running) {
            std::this_thread::sleep_for(100ms);
        }
        
        // Stop camera
        mCamera->stop();
    }
};

int main() {
    // Set up signal handling
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // Create window
    cv::namedWindow("Camera Test", cv::WINDOW_AUTOSIZE);
    
    // Create and initialize camera test
    CameraTest camTest;
    if (!camTest.initialize()) {
        std::cerr << "Failed to initialize camera test" << std::endl;
        return -1;
    }
    
    // Start camera test
    camTest.start();
    
    // Cleanup
    cv::destroyAllWindows();
    
    return 0;
}
