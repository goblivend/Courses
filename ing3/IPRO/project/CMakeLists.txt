cmake_minimum_required(VERSION 2.8)
project(exemplesOpencv)
find_library(wiringPi_LIB wiringPi)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#-----------compilation step
add_executable(algo algo.cpp)
add_executable(algo-frames algo-frames.cpp)
add_executable(raspi_code raspi_code.cpp)

#-----------linking step
target_link_libraries(algo ${OpenCV_LIBS})
target_link_libraries(algo-frames ${OpenCV_LIBS})
target_link_libraries(raspi_code ${OpenCV_LIBS} ${wiringPi_LIB})
