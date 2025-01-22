#include <iostream>
#include <opencv2/opencv.hpp>

int main ( int argc, char** argv){
    cv::Mat img = cv::imread( argv[1], cv::IMREAD_COLOR);
    if( img.empty() ) {
        std::cerr << "File not found!!!" << std::endl;
    exit(-1);
    }
    cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Image", img);
    cv::waitKey(0);

    cv::flip(img, img, 0);
    cv::imshow("Image", img);
    cv::waitKey(0);
    cv::destroyWindow("Image");
    return 0;
}
