#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    cv::VideoCapture cap(0); // Open default camera
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera" << std::endl;
        return -1;
    }

    cv::namedWindow("Camera Test", cv::WINDOW_AUTOSIZE);
    
    while (true) {
        cv::Mat frame;
        cap >> frame; // Capture frame
        
        if (frame.empty()) {
            std::cerr << "Error: No captured frame" << std::endl;
            break;
        }
        
        cv::imshow("Camera Test", frame);
        
        if (cv::waitKey(30) == 'q') { // Exit on 'q' key
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
