#include "rclcpp/rclcpp.hpp"
#include "my_first_ros_interfaces/srv/add_two_ints.hpp"
using namespace std::placeholders;
class MyClientNode : public rclcpp::Node
{
public:
    MyClientNode() : Node("add_two_ints_client_node")
    {
        client_ = this->create_client<my_first_ros_interfaces::srv::AddTwoInts>("add_two_ints");
    }
    void call_service(char *argv[])
    {
        while (!client_->wait_for_service(std::chrono::milliseconds(1000))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                rclcpp::shutdown();
                std::exit(1);
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
        auto request = std::make_shared<my_first_ros_interfaces::srv::AddTwoInts::Request>();
        request->a = atoll(argv[1]);
        request->b = atoll(argv[2]);
        RCLCPP_INFO(this->get_logger(), "Sending request");
        auto future = client_->async_send_request(request);
        if ( rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::TIMEOUT )
        {
            client_->remove_pending_request(future);
            RCLCPP_ERROR(this->get_logger(), "Timeout : failed to receive service response");
            rclcpp::shutdown();
            std::exit(1);
        }
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Result of add_two_ints: %ld", response.get()->sum);
    }
private:
    rclcpp::Client<my_first_ros_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    if (argc != 3) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client_node X Y");
        rclcpp::shutdown();
        return 1;
    }
    auto node = std::make_shared<MyClientNode>();
    node->call_service(argv);
    rclcpp::shutdown();
    return 0;
}
