#include <iostream>
#include <opencv2/opencv.hpp>

cv::Mat veget_index(cv::Mat vis6, cv::Mat vis8){
    cv::Mat m1(vis6.size(), CV_8UC1);
    for(int row = 0; row < m1.size().height; row++){
        for(int col = 0; col < m1.size().width; col++){
            double vis = (double) vis6.at<unsigned char>(row,col);
            double nir = (double) vis8.at<unsigned char>(row,col);
            if (nir + vis == 0){
                m1.at<unsigned char>(row,col) = 0;
                continue;
            }
            int ndvi = (nir - vis) / (nir + vis) * 255;
            if (ndvi < 0){
                m1.at<unsigned char>(row,col) = 0;
                continue;
            }
            m1.at<unsigned char>(row,col) = ndvi;
        }
    }

    return m1;
}

int main(int argc, char** argv){
    cv::Mat vis6 = cv::imread( argv[1], cv::IMREAD_GRAYSCALE);
    if( vis6.empty() ) {
        std::cerr << "File vis6 not found!!!" << std::endl;
        exit(-1);
    }

    cv::Mat vis8 = cv::imread( argv[2], cv::IMREAD_GRAYSCALE);
    if( vis8.empty() ) {
        std::cerr << "File vis8 not found!!!" << std::endl;
        exit(-1);
    }
    cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);


    cv::Mat m = veget_index(vis6, vis8);
    cv::imshow("Image", m);
    cv::waitKey(0);


    return 0;
}
