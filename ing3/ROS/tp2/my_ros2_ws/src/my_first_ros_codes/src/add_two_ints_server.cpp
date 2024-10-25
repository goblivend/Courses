#include "rclcpp/rclcpp.hpp"
#include "my_first_ros_interfaces/srv/add_two_ints.hpp"
using namespace std::placeholders;
class MyServerNode : public rclcpp::Node
{
public:
    MyServerNode() : Node("add_two_ints_server_node")
    {
        service_ = this->create_service<my_first_ros_interfaces::srv::AddTwoInts>("add_two_ints",
        std::bind(&MyServerNode::addCallback, this, _1, _2));
    }
private:
    void addCallback(
        const std::shared_ptr<my_first_ros_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<my_first_ros_interfaces::srv::AddTwoInts::Response> response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "Incoming request\na: %ld" " b: %ld",
            request->a, request->b);
        RCLCPP_INFO(this->get_logger(), "sending back response: [%ld]",
            (long int)response->sum);
    }
    rclcpp::Service<my_first_ros_interfaces::srv::AddTwoInts>::SharedPtr service_;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");
    rclcpp::spin(std::make_shared<MyServerNode>());
    rclcpp::shutdown();
}
