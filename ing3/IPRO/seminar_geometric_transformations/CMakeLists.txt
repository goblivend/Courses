cmake_minimum_required( VERSION 2.8 )
project( exemplesOpencv )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
#-----------compilation step
add_executable( exo1 exo1.cpp )
add_executable( exo3 exo3.cpp )
add_executable( exo4 exo4.cpp )
#-----------linking step
target_link_libraries( exo1 ${OpenCV_LIBS} )
target_link_libraries( exo3 ${OpenCV_LIBS} )
target_link_libraries( exo4 ${OpenCV_LIBS} )
