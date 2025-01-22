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

    cv::resize(img, img, cv::Size(), 0.5, 0.5);
    cv::imshow("Image2", img);

    cv::resize(img, img, cv::Size(), 0.5, 0.5);
    cv::imshow("Image3", img);

    cv::resize(img, img, cv::Size(), 0.5, 0.5);
    cv::imshow("Image4", img);

    cv::waitKey(0);
    cv::destroyWindow("Image");
    return 0;
}
