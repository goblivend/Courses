#include <iostream>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv){
    cv::VideoCapture cap;
    cap.open(0);
    if(!cap.isOpened()){
        std::cout << "Impossible to open camera" << std::endl;
        return -1;
    }
    cv::namedWindow("Camera", cv::WINDOW_AUTOSIZE);
    cv::Mat frame;
    for(;;){
        cap >> frame;
        imshow("Camera",frame);
        if(cv::waitKey(33)!=-1) break;
        //if(cv::waitKey(33)!=255) break;//for openCV3
    }
    cv::destroyWindow("Camera");
    return 0;
}
