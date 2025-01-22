#include <iostream>
#include <opencv2/opencv.hpp>

cv::Mat to_magenta(cv::Mat m){
    cv::Mat m1(m.size(), CV_8UC3);
    for(int row = 0; row < m.size().height; row++){
        for(int col = 0; col < m.size().width; col++){
            m1.at<cv::Vec3b>(row,col)[0] = m.at<cv::Vec3b>(row,col)[1];
            m1.at<cv::Vec3b>(row,col)[1] = m.at<cv::Vec3b>(row,col)[0];
            m1.at<cv::Vec3b>(row,col)[2] = m.at<cv::Vec3b>(row,col)[2];
        }
    }

    return m1;
}

cv::Mat to_yellow(cv::Mat m){
    cv::Mat m1(m.size(), CV_8UC3);
    for(int row = 0; row < m.size().height; row++){
        for(int col = 0; col < m.size().width; col++){
            m1.at<cv::Vec3b>(row,col)[0] = m.at<cv::Vec3b>(row,col)[0];
            m1.at<cv::Vec3b>(row,col)[1] = m.at<cv::Vec3b>(row,col)[2];
            m1.at<cv::Vec3b>(row,col)[2] = m.at<cv::Vec3b>(row,col)[2];
        }
    }

    return m1;
}

cv::Mat to_cyan(cv::Mat m){
    cv::Mat m1(m.size(), CV_8UC3);
    for(int row = 0; row < m.size().height; row++){
        for(int col = 0; col < m.size().width; col++){
            m1.at<cv::Vec3b>(row,col)[0] = m.at<cv::Vec3b>(row,col)[0];
            m1.at<cv::Vec3b>(row,col)[1] = m.at<cv::Vec3b>(row,col)[0];
            m1.at<cv::Vec3b>(row,col)[2] = m.at<cv::Vec3b>(row,col)[1];
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
    cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);


    cv::Mat m = to_yellow(img);
    cv::imshow("Image", m);
    cv::waitKey(0);


    m = to_magenta(m);
    cv::imshow("Image", m);
    cv::waitKey(0);


    m = to_cyan(m);
    cv::imshow("Image", m);
    cv::waitKey(0);
    cv::destroyWindow("Image");

    return 0;
}
