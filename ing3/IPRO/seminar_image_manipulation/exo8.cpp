#include <iostream>
#include <opencv2/opencv.hpp>

cv::Mat make_plus(int width, int height){
    double stepCol = 256.0 / width * 2;
    double stepRow = 256.0 / height * 2;
    cv::Mat m1(width,height, CV_8UC1);
    for(int row = 0; row < height/2; row++){
        for(int col = 0; col < width; col++){
            m1.at<unsigned char>(row,col) = row * stepRow;
        }
    }

    for(int row = 0; row < height/2; row++){
        for(int col = 0; col < width; col++){
            m1.at<unsigned char>(row + height/2, col) = 255 - (row * stepRow);
        }
    }

    cv::Mat m2(width, height, CV_8UC1);
    for(int row = 0; row < height; row++){
        for(int col = 0; col < width / 2; col++){
            m2.at<unsigned char>(row,col) = stepCol * col;
        }


        for(int col = 0; col < width / 2; col++){
            m2.at<unsigned char>(row, width/2 + col) = 255 - (col * stepCol);
        }
    }

    return 0.5*m1 + 0.5*m2;
}

int main(int argc, char** argv){
    cv::Mat img = cv::imread( argv[1], cv::IMREAD_COLOR);
    if( img.empty() ) {
        std::cerr << "File not found!!!" << std::endl;
        exit(-1);
    }


    cv::Mat img2 = make_plus(img.size[0], img.size[1]);
    // img2 *= 1./255;

    cv::cvtColor(img2, img2, cv::COLOR_GRAY2BGR);
    cv::Mat m = 0.5*img2 + 0.5*img;


    cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Image", m);
    cv::waitKey(0);
    cv::destroyWindow("Image");

    return 0;
}
