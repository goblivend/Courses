#include <string>
#include <iostream>
#include <filesystem>
#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;


int main ( int argc, char** argv) {
    std::string path = argv[1];
    cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);

    for (const auto & entry : fs::directory_iterator(path)) {
        std::cout << entry.path() << std::endl;
        cv::Mat img = cv::imread(entry.path(), cv::IMREAD_COLOR);
        cv::imshow("Image", img);
        cv::waitKey(0);

    }

    cv::destroyWindow("Image");
}
