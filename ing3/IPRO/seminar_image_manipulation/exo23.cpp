#include <string>
#include <iostream>
#include <filesystem>
#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

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
            double ndvi = (nir - vis) / (nir + vis) * 255;
            if (ndvi < 0){
                m1.at<unsigned char>(row,col) = 0;
                continue;
            }
            m1.at<unsigned char>(row,col) = ndvi;
        }
    }

    return m1;
}




int main ( int argc, char** argv) {
    std::string pathv6 = argv[1];
    pathv6 += "/VIS6";
    std::string pathv8 = argv[1];
    pathv8 += "/VIS8";
    cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
    cv::Mat prev;

    // iterate over the files in the directories
    for (const auto & entry : fs::directory_iterator(pathv6)) {
        std::string path6 = entry.path();
        std::string path8 = pathv8 + "/" + entry.path().filename().string();
        cv::Mat vis6 = cv::imread(path6, cv::IMREAD_GRAYSCALE);
        cv::Mat vis8 = cv::imread(path8, cv::IMREAD_GRAYSCALE);
        cv::Mat m = veget_index(vis6, vis8);
        if (prev.empty()){
            prev = m;
        } else {
            prev = cv::max(prev, m);
        }
        cv::imshow("Image", prev);
        cv::waitKey(0);

    }

    cv::waitKey(0);


    return 0;
}
