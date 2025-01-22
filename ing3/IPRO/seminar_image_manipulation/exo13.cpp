#include <iostream>
#include <opencv2/opencv.hpp>

cv::Mat to_gray(cv::Mat m){
    cv::Mat m1(m.size(), CV_8UC1);
    for(int row = 0; row < m.size().height; row++){
        for(int col = 0; col < m.size().width; col++){
            m1.at<unsigned char>(row,col) = m.at<cv::Vec3b>(row,col)[0] * 0.114;
            m1.at<unsigned char>(row,col) += m.at<cv::Vec3b>(row,col)[1] * 0.587;
            m1.at<unsigned char>(row,col) += m.at<cv::Vec3b>(row,col)[2] * 0.299;
        }
    }

    return m1;
}

int main(int argc, char** argv){
    cv::Mat img = cv::imread( argv[1], cv::IMREAD_COLOR);
    if( img.empty() ) {
        std::cerr << "File not found!!!" << std::endl;
        exit(-1);
    }


    cv::Mat m1 = to_gray(img);

    cv::Mat m2;

    cv::cvtColor(img, m2, cv::COLOR_BGR2GRAY);


    cv::Mat m = (m2-m1)*20;

    cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Image", m);
    cv::waitKey(0);
    cv::destroyWindow("Image");

    return 0;
}
