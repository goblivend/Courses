#include <iostream>
#include <opencv2/opencv.hpp>


int main(int argc, char** argv){
    cv::Mat img = cv::imread( argv[1], cv::IMREAD_COLOR);
    if( img.empty() ) {
        std::cerr << "File not found!!!" << std::endl;
        exit(-1);
    }

    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);


    cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Image", img);
    cv::waitKey(0);
    cv::destroyWindow("Image");

    cv::Mat m1;
    cv::threshold(img, m1, 130, 255, cv::THRESH_BINARY);

    cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Image", m1);
    cv::waitKey(0);
    cv::destroyWindow("Image");

    cv::threshold(img, m1, 130, 255, cv::THRESH_BINARY_INV);

    cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Image", m1);
    cv::waitKey(0);
    cv::destroyWindow("Image");

    cv::threshold(img, m1, 130, 255, cv::THRESH_TRUNC);

    cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Image", m1);
    cv::waitKey(0);
    cv::destroyWindow("Image");

    cv::threshold(img, m1, 130, 255, cv::THRESH_TOZERO);

    cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Image", m1);
    cv::waitKey(0);
    cv::destroyWindow("Image");

    cv::threshold(img, m1, 130, 255, cv::THRESH_TOZERO_INV);

    cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Image", m1);
    cv::waitKey(0);
    cv::destroyWindow("Image");

    return 0;
}
