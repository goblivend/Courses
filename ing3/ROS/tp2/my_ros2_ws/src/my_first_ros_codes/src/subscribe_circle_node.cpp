#include "rclcpp/rclcpp.hpp"
#include "my_first_ros_interfaces/msg/circle.hpp"
using std::placeholders::_1;

class MySubscriber : public rclcpp::Node
{
public:
    MySubscriber() : Node("my_subscriber_circle_node")
    {
        subscription_ = this->create_subscription<my_first_ros_interfaces::msg::Circle>(
        "topic_circles", 10, std::bind(&MySubscriber::topic_callback, this, _1));

    }
private:
    void topic_callback(const my_first_ros_interfaces::msg::Circle & msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Receiving: (center (%.5f, %.5f), rad %.5f) Answering : Pong !",
        msg.center.x, msg.center.y, msg.radius);
    }
    rclcpp::Subscription<my_first_ros_interfaces::msg::Circle>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MySubscriber>());
    rclcpp::shutdown();
    return 0;
}
