#include <iostream>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv){
    cv::VideoCapture cap;
    cap.open(argv[1]);
    if(!cap.isOpened()){
        std::cout << "Impossible to open " << argv[1] << std::endl;
        return -1;
    }
    cv::VideoWriter out;
    cv::Size capSize(cap.get(cv::CAP_PROP_FRAME_WIDTH),
    cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    out.open("videoFlip.avi", cv::VideoWriter::fourcc('M','J', 'P', 'G'),
    cap.get(cv::CAP_PROP_FPS), capSize, true);
    if (!out.isOpened()) {
        std::cerr << "Could not open the output video file for write\n";
        return -1;
    }
    cv::namedWindow("Video", cv::WINDOW_AUTOSIZE);
    cv::Mat frame;
    cv::namedWindow("Video flip", cv::WINDOW_AUTOSIZE);
    cv::Mat frameFlip;
    for(;;){
        cap >> frame;
        if(frame.empty()) break;
        cv::imshow("Video", frame);
        cv::flip(frame, frameFlip, 0);
        cv::imshow("Video flip", frameFlip);
        out.write(frameFlip);
        if (cv::waitKey(33)!=-1) break;
        //if (cv::waitKey(33)!=255) break;//for openCV3
    }
    cv::destroyAllWindows();
    return 0;
}
