cmake_minimum_required(VERSION 3.5)
project(ros_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge)
find_package(OpenCV REQUIRED)

# Executable
add_executable(control_by_camera_node code/control_by_camera.cpp code/algo_frame.cpp)
ament_target_dependencies(control_by_camera_node rclcpp sensor_msgs geometry_msgs cv_bridge OpenCV)
install(TARGETS control_by_camera_node DESTINATION lib/${PROJECT_NAME})


# Install launch files
install(
  DIRECTORY config description launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
