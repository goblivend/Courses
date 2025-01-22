#include <iostream>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv){
    cv::Mat m1(512,512, CV_8UC1);
    for(int row = 0; row < 256; row++){
        for(int col = 0; col < 512; col++){
            m1.at<unsigned char>(row,col) = row;
        }
    }
    for(int row = 0; row < 256; row++){

        for(int col = 0; col < 512; col++){
            m1.at<unsigned char>(row + 256,col) = 255 - row;
        }
    }

    cv::Mat m2(512,512, CV_8UC1);
    for(int row = 0; row < 512; row++){
        for(int col = 0; col < 256; col++){
            m2.at<unsigned char>(row,col) = col;
        }


        for(int col = 0; col < 256; col++){
            m2.at<unsigned char>(row,256 + col) = 255 - col;
        }
    }

    cv::Mat m(512,512, CV_8UC1);
    m = 0.5*m1 + 0.5*m2;

    cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Image", m);
    cv::waitKey(0);
    cv::destroyWindow("Image");

    return 0;
}
