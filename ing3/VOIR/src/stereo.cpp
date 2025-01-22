#include <opencv2/opencv.hpp>

// initialize values for StereoSGBM parameters
int numDisparities = 8;
int blockSize = 5;
int preFilterType = 1;
int preFilterSize = 1;
int preFilterCap = 31;
int minDisparity = 0;
int textureThreshold = 10;
int uniquenessRatio = 15;
int speckleRange = 0;
int speckleWindowSize = 0;
int disp12MaxDiff = -1;
int dispType = CV_16S;

// Creating an object of StereoSGBM algorithm
cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create();

static void on_trackbar1(int, void*) {
    stereo->setNumDisparities(numDisparities * 16);
    numDisparities = numDisparities * 16;
}

static void on_trackbar2(int, void*) {
    stereo->setBlockSize(blockSize * 2 + 5);
    blockSize = blockSize * 2 + 5;
}

static void on_trackbar3(int, void*) {
    stereo->setPreFilterType(preFilterType);
}

static void on_trackbar4(int, void*) {
    stereo->setPreFilterSize(preFilterSize * 2 + 5);
    preFilterSize = preFilterSize * 2 + 5;
}

static void on_trackbar5(int, void*) {
    stereo->setPreFilterCap(preFilterCap);
}

static void on_trackbar6(int, void*) {
    stereo->setTextureThreshold(textureThreshold);
}

static void on_trackbar7(int, void*) {
    stereo->setUniquenessRatio(uniquenessRatio);
}

static void on_trackbar8(int, void*) {
    stereo->setSpeckleRange(speckleRange);
}

static void on_trackbar9(int, void*) {
    stereo->setSpeckleWindowSize(speckleWindowSize * 2);
    speckleWindowSize = speckleWindowSize * 2;
}

static void on_trackbar10(int, void*) {
    stereo->setDisp12MaxDiff(disp12MaxDiff);
}

static void on_trackbar11(int, void*) {
    stereo->setMinDisparity(minDisparity);
}

int main(int argc, char* argv[]) {
    // Reading the left and right images.
    // cv::VideoCapture capL("./samples/book_left.avi");
    // if (!capL.isOpened()) {
    //     std::cerr << "Impossible to open left capture" << std::endl;
    //     return -1;
    // }
    //
    // cv::VideoCapture capR("./samples/book_right.avi");
    // if (!capR.isOpened()) {
    //     std::cerr << "Impossible to open " << std::endl;
    //     return -1;
    // }

    // TODO: Read config file
    const std::string inputSettingsFile = "./improved_params2_cpp.xml";
    cv::FileStorage fs(inputSettingsFile, cv::FileStorage::READ);  // Read the settings
    if (!fs.isOpened()) {
        std::cout << "Could not open the configuration file: \"" << inputSettingsFile << "\""
                  << std::endl;
        return -1;
    }

    cv::Mat Left_Stereo_Map1, Left_Stereo_Map2;
    cv::Mat Right_Stereo_Map1, Right_Stereo_Map2;
    cv::Mat Left_nice, Right_nice;

    fs["Left_Stereo_Map_x"] >> Left_Stereo_Map1;
    fs["Left_Stereo_Map_y"] >> Left_Stereo_Map2;
    fs["Right_Stereo_Map_x"] >> Right_Stereo_Map1;
    fs["Right_Stereo_Map_y"] >> Right_Stereo_Map2;
    fs.release();

    // Creating a named window to be linked to the trackbars
    cv::namedWindow("disparity", cv::WINDOW_NORMAL);
    cv::resizeWindow("disparity", 600, 600);

    cv::createTrackbar("numDisparities", "disparity", &numDisparities, 18, on_trackbar1);
    cv::createTrackbar("blockSize", "disparity", &blockSize, 50, on_trackbar2);
    cv::createTrackbar("preFilterType", "disparity", &preFilterType, 1, on_trackbar3);
    cv::createTrackbar("preFilterSize", "disparity", &preFilterSize, 25, on_trackbar4);
    cv::createTrackbar("preFilterCap", "disparity", &preFilterCap, 62, on_trackbar5);
    cv::createTrackbar("textureThreshold", "disparity", &textureThreshold, 100, on_trackbar6);
    cv::createTrackbar("uniquenessRatio", "disparity", &uniquenessRatio, 100, on_trackbar7);
    cv::createTrackbar("speckleRange", "disparity", &speckleRange, 100, on_trackbar8);
    cv::createTrackbar("speckleWindowSize", "disparity", &speckleWindowSize, 25, on_trackbar9);
    cv::createTrackbar("disp12MaxDiff", "disparity", &disp12MaxDiff, 25, on_trackbar10);
    cv::createTrackbar("minDisparity", "disparity", &minDisparity, 25, on_trackbar11);

    // Treat images
    cv::Mat imgL, imgR, imgL_gray, imgR_gray;
    // capL >> imgL;
    // capR >> imgR;

    for (;;) {
        std::string image_path_l = cv::samples::findFile("./samples/data/left01.jpg");
        imgL = imread(image_path_l, cv::IMREAD_COLOR);
        std::string image_path_r = cv::samples::findFile("./samples/data/right01.jpg");
        imgR = imread(image_path_r, cv::IMREAD_COLOR);

        // Converting images to grayscale
        cv::cvtColor(imgL, imgL_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(imgR, imgR_gray, cv::COLOR_BGR2GRAY);

        // Apply the calculated maps for rectification and undistortion
        cv::remap(imgL_gray,
                  Left_nice,
                  Left_Stereo_Map1,
                  Left_Stereo_Map2,
                  cv::INTER_LANCZOS4,
                  cv::BORDER_CONSTANT,
                  0);

        cv::remap(imgR_gray,
                  Right_nice,
                  Right_Stereo_Map1,
                  Right_Stereo_Map2,
                  cv::INTER_LANCZOS4,
                  cv::BORDER_CONSTANT,
                  0);

        // Calculating disparith using the StereoSGBM algorithm
        cv::Mat disp;
        stereo->compute(Left_nice, Right_nice, disp);

        // Normalizing the disparity map for better visualisation
        cv::normalize(disp, disp, 0, 255, cv::NORM_MINMAX, CV_8UC1);

        // Displaying the disparity map
        // cv::imshow("imgL", Left_nice);
        // cv::imshow("imgR", Right_nice);
        cv::imshow("disparity", disp);
        if (cv::waitKey(1) == 27)
            break;
    }

    return 0;
}
