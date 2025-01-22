#include <iostream>
#include <opencv2/opencv.hpp>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringSerial.h>

/*******************************************************************************
 * Function algo
 ******************************************************************************/

enum Color
{
    RED,
    YELLOW
};

// Equalization with histogramm using hsv temp image
void equalizationHisto(cv::Mat& frame, cv::Mat& res)
{
    cv::Mat res_hsv(frame.size(), CV_8UC3);
    unsigned char min_pix = 255;
    unsigned char max_pix = 0;

    cv::Mat frame_hsv;
    cv::cvtColor(frame, frame_hsv, cv::COLOR_BGR2HSV);

    for (int i = 0; i < frame_hsv.size().width; i++)
    {
        for (int j = 0; j < frame_hsv.size().height; j++)
        {
            min_pix = std::min(min_pix, frame_hsv.at<cv::Vec3b>(j, i)[2]);
            max_pix = std::max(max_pix, frame_hsv.at<cv::Vec3b>(j, i)[2]);
        }
    }

    float a = 255 / (float)(max_pix - min_pix);
    float b = (float)(-255 * min_pix) / (float)(max_pix - min_pix);

    for (int i = 0; i < res_hsv.size().width; i++)
    {
        for (int j = 0; j < res_hsv.size().height; j++)
        {
            res_hsv.at<cv::Vec3b>(j, i)[0] = frame_hsv.at<cv::Vec3b>(j, i)[0];
            res_hsv.at<cv::Vec3b>(j, i)[1] = frame_hsv.at<cv::Vec3b>(j, i)[1];
            res_hsv.at<cv::Vec3b>(j, i)[2] =
                a * frame_hsv.at<cv::Vec3b>(j, i)[2] + b;
        }
    }

    cv::cvtColor(res_hsv, res, cv::COLOR_HSV2BGR);
}

// Get mask for detecting red areas
void getRedMask(cv::Mat& frame_hsv, cv::Mat& res_mask)
{
    cv::Scalar lower_red1 = cv::Scalar(160, 100, 150);
    cv::Scalar upper_red1 = cv::Scalar(180, 255, 255);

    cv::Mat mask1;
    cv::inRange(frame_hsv, lower_red1, upper_red1, mask1);

    cv::Scalar lower_red2 = cv::Scalar(0, 100, 150);
    cv::Scalar upper_red2 = cv::Scalar(10, 255, 255);

    cv::Mat mask2;
    cv::inRange(frame_hsv, lower_red2, upper_red2, mask2);

    cv::bitwise_or(mask1, mask2, res_mask);
}

// Get mask for detecting yellow areas
void getYellowMask(cv::Mat& frame_hsv, cv::Mat& res_mask)
{
    cv::Scalar lower_yellow = cv::Scalar(20, 150, 150);
    cv::Scalar upper_yellow = cv::Scalar(50, 255, 255);
    cv::inRange(frame_hsv, lower_yellow, upper_yellow, res_mask);
}

// Function to detect markers and return the center points of detected markers
std::vector<cv::Point> detectMarkers(cv::Mat& frame, Color color_type)
{
    std::vector<cv::Point> markerCenters;
    cv::Mat hsv, mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    if (color_type == Color::RED)
        getRedMask(hsv, mask);
    else
        getYellowMask(hsv, mask);

    // Remove small areas with are noise with erosion / dilatation
    cv::erode(mask, mask,
              cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25, 25)));
    cv::dilate(mask, mask,
               cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25, 25)));

    // Find contours for detected markers
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours)
    {
        // Calculate the center of each detected marker
        cv::Moments moments = cv::moments(contour, true);
        if (moments.m00 != 0)
        {
            int centerX = static_cast<int>(moments.m10 / moments.m00);
            int centerY = static_cast<int>(moments.m01 / moments.m00);
            markerCenters.push_back(cv::Point(centerX, centerY));
        }
    }
    return markerCenters;
}

// Function to calculate the midpoint between the nearest red and yellow markers
cv::Point calculateMidpoint(cv::Mat& frame,
                            const std::vector<cv::Point>& redMarkers,
                            const std::vector<cv::Point>& yellowMarkers)
{
    if (redMarkers.empty() || yellowMarkers.empty()) // Invalid midpoint
        return cv::Point(-1, -1);

    // Find the nearest red and yellow markers (nearest mean lower in the image)
    cv::Point nearestRed = redMarkers[0];
    for (auto p : redMarkers)
    {
        if (p.y > nearestRed.y)
            nearestRed = p;
    }

    cv::Point nearestYellow = yellowMarkers[0];
    for (auto p : yellowMarkers)
    {
        if (p.y > nearestYellow.y)
            nearestYellow = p;
    }

    double minDistance = cv::norm(nearestRed - nearestYellow);

    // Calculate midpoint
    int midX = (nearestRed.x + nearestYellow.x) / 2;
    int midY = (nearestRed.y + nearestYellow.y) / 2;
    return cv::Point(midX, midY);
}

/*******************************************************************************
 * Uart communication
 ******************************************************************************/

int init_uart(int* serial_port)
{
    if ((*serial_port = serialOpen("/dev/ttyAMA0", 115200)) < 0)
    {
        std::cerr << "ERROR: Unable to open serial device" << std::endl;
        return 1;
    }
    if (wiringPiSetup() == -1)
    {
        std::cerr << "ERROR: Unable to start wiringPi" << std::endl;
        return 1;
    }
    return 0;
}

void send_uart_value(int serial_port, int value)
{
    std::string str = std::to_string(value);
    std::cout << "DEBUG: UART is sending " << str << std::endl;
    for (auto& ch : str)
        serialPutchar(serial_port, ch);
    serialPutchar(serial_port, '\n');
}

/*******************************************************************************
 * Main
 ******************************************************************************/

int libcamera_pid = -1;

// Signal handler to kill libcamera before exiting
void sighandler(int signum)
{
    std::cout << "Received SIGINT. Exiting gracefully..." << std::endl;
    kill(libcamera_pid, SIGINT);
    exit(0);
}

int main(int argc, char* argv[])
{
    // Init serial port to communicate with the arduino
    int serial_port;
    init_uart(&serial_port);

    int pid = fork();

    if (pid < 0)
    {
        std::cerr << "Failed to fork process" << std::endl;
        return -1;
    }

    const char* frame_name = "frame.jpeg";

    // Child process => use to run libcamera in background
    if (pid == 0)
    {
        // Close stdout and stderr to keep it for the parent process
        close(STDOUT_FILENO);
        close(STDERR_FILENO);

        // Exec libcamera
        execlp("libcamera-still",
               "libcamera-still", "-n", "--verbose", "0", "--timeout", "0", 
               "--width", "640", "--height", "480", "--mode", "3280:2464", 
               "-o", frame_name, "--signal", NULL);
        return 0;
    }

    // Parent process => use to process frame

    // Signal handler => kill libcamera if receive SIGINT (ctrl + C) signal
    libcamera_pid = pid;
    signal(SIGINT, sighandler);

    // Send SIGUSR1 for the first frame.
    usleep(500000); // 500 ms (waiting for execvp ready)
    kill(libcamera_pid, SIGUSR1);
    usleep(1000000); // 1 s

    int frame_idx = 0;
    while (true)
    {
        // Read frame.
        cv::Mat frame = cv::imread(frame_name);
        if (frame.empty())
        {
            std::cerr << "ERROR: Could not read frame." << std::endl;
            continue;
        }
        std::cout << "Frame " << ++frame_idx << std::endl;

        // Send SIGUSR1 for next frame.
        kill(libcamera_pid, SIGUSR1);

        // Median filter to remove noise
        cv::medianBlur(frame, frame, 5);

        // Equalization with histogramm
        equalizationHisto(frame, frame);

        // Detect red and yellow markers
        std::vector<cv::Point> redMarkers = detectMarkers(frame, Color::RED);
        std::vector<cv::Point> yellowMarkers =
            detectMarkers(frame, Color::YELLOW);

        // Calculate midpoint
        cv::Point midpoint =
            calculateMidpoint(frame, redMarkers, yellowMarkers);
        cv::Point imageCenter(frame.cols / 2, frame.rows / 2);

        if (redMarkers.empty() && yellowMarkers.empty())
        {
            std::cerr << "WARN: No markers detected." << std::endl;
        }
        else if (redMarkers.empty() && !yellowMarkers.empty())
        {
            std::cout
                << "INFO: Only yellow markers detected, going left (-128)."
                << std::endl;
            send_uart_value(serial_port, -128);
        }
        else if (!redMarkers.empty() && yellowMarkers.empty())
        {
            std::cout << "INFO: Only red markers detected, going right (128)."
                      << std::endl;
            send_uart_value(serial_port, 128);
        }
        else
        {
            int diff = std::clamp(midpoint.x - imageCenter.x, -128, 128) / 2;
            std::cout << "INFO: Turning " << diff << std::endl;
            send_uart_value(serial_port, diff);
        }

        usleep(300000); // 300ms

        // Save image for debugging purposes.
        std::stringstream ss;
        ss << "frames/frame-" << frame_idx << ".jpeg";
        cv::imwrite(ss.str(), frame);
    }

    return 0;
}
