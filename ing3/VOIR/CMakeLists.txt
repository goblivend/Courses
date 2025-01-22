cmake_minimum_required(VERSION 2.8)

project(exemplesOpencv)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
#-----------compilation step
add_executable(voir src/image-getter.cpp)
add_executable(brain src/stereo.cpp)
add_executable(calib src/calibrate.cpp)
#-----------linking step

target_link_libraries(voir ${OpenCV_LIBS})
target_link_libraries(brain ${OpenCV_LIBS})
target_link_libraries(calib ${OpenCV_LIBS})
