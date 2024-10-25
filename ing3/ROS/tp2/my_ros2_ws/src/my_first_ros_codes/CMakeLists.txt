cmake_minimum_required(VERSION 3.8)
project(my_first_ros_codes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_first_ros_interfaces REQUIRED)

add_executable(hello_node src/hello_world.cpp)
ament_target_dependencies(hello_node rclcpp)

add_executable(hello_custom_node src/hello_custom_node.cpp)
ament_target_dependencies(hello_custom_node rclcpp)

add_executable(publish_node src/publish_node.cpp)
ament_target_dependencies(publish_node rclcpp std_msgs)

add_executable(subscribe_node src/subscribe_node.cpp)
ament_target_dependencies(subscribe_node rclcpp std_msgs)

add_executable(turtle_random_node src/turtle_random_node.cpp)
ament_target_dependencies(turtle_random_node rclcpp geometry_msgs)

add_executable(publish_circle_node src/publish_circle_node.cpp)
ament_target_dependencies(publish_circle_node rclcpp my_first_ros_interfaces)

add_executable(subscribe_circle_node src/subscribe_circle_node.cpp)
ament_target_dependencies(subscribe_circle_node rclcpp my_first_ros_interfaces)

add_executable(add_two_ints_server_node src/add_two_ints_server.cpp)
ament_target_dependencies(add_two_ints_server_node rclcpp my_first_ros_interfaces)

add_executable(add_two_ints_client_node src/add_two_ints_client.cpp)
ament_target_dependencies(add_two_ints_client_node rclcpp my_first_ros_interfaces)

add_executable(hello_world_parameter_node src/hello_world_parameter.cpp)
ament_target_dependencies(hello_world_parameter_node rclcpp)

add_executable(change_hello_world_parameter_node src/change_hello_world_parameter.cpp)
ament_target_dependencies(change_hello_world_parameter_node rclcpp)

add_executable(timer_action_server_node src/timer_action_server.cpp)
ament_target_dependencies(timer_action_server_node rclcpp rclcpp_action my_first_ros_interfaces)

add_executable(timer_action_client_node src/timer_action_client.cpp)
ament_target_dependencies(timer_action_client_node rclcpp rclcpp_action my_first_ros_interfaces)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
    hello_node
    hello_custom_node
    publish_node
    publish_circle_node
    subscribe_node
    subscribe_circle_node
    turtle_random_node
    add_two_ints_server_node
    add_two_ints_client_node
    hello_world_parameter_node
    change_hello_world_parameter_node
    timer_action_server_node
    timer_action_client_node
    DESTINATION lib/${PROJECT_NAME})

#Install launch files
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)


ament_package()
