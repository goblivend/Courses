#pragma once

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

void process_frame(cv::Mat& frame, int* direction, int *finish, rclcpp::Logger logger);
