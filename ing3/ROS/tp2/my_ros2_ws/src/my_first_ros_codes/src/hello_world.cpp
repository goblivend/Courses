#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv); // Launch the necessary initializations
    auto node = std::make_shared<rclcpp::Node>("hello_world_node"); // Create a new node
    RCLCPP_INFO(node->get_logger(), "Hello, world!"); // Send a message to the console
    rclcpp::spin(node); // Handle callback functions
    rclcpp::shutdown(); // Close the node properly
    return 0;
}
