#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MyPublisher : public rclcpp::Node
{
public:
    MyPublisher() : Node("turtle_random_node"), count_(0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MyPublisher::timer_callback, this));
    }
private:
    void timer_callback()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 2.0;
        message.angular.z = 3. * double(rand()) / RAND_MAX - 1.5;

        RCLCPP_INFO(this->get_logger(), "Publishing: linear=%.3f, angular=%.3f",  message.linear.x, message.angular.z);
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    srand(time(0));
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyPublisher>());
    rclcpp::shutdown();
return 0;
}
