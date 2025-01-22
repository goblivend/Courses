#include <iostream>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv){
    cv::VideoCapture cap;
    cap.open(argv[1]);
    if(!cap.isOpened()){
        std::cerr << "Impossible to open " << argv[1] << std::endl;
        return -1;
    }
    cv::namedWindow("Video", cv::WINDOW_AUTOSIZE);
    cv::Mat frame;
    for(;;){
        cap >> frame;
        if(frame.empty()) {printf("ko\n");break;}
        cv::imshow("Video", frame);
        if (cv::waitKey(33)!=-1) break;
        //if (cv::waitKey(33)!=255) break;//opencv3
    }
    cv::destroyWindow("Video");
    return 0;
}
