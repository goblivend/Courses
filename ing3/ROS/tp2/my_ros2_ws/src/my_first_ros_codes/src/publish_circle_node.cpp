#include "rclcpp/rclcpp.hpp"
#include "my_first_ros_interfaces/msg/circle.hpp"

class MyPublisher : public rclcpp::Node
{
public:
    MyPublisher() : Node("turtle_random_node"), count_(0)
    {
        publisher_ = this->create_publisher<my_first_ros_interfaces::msg::Circle>("topic_circles", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MyPublisher::timer_callback, this));
    }
private:
    void timer_callback()
    {
        auto message = my_first_ros_interfaces::msg::Circle();
        message.center.x = 4.*double(rand())/RAND_MAX;
        message.radius = double(rand())/RAND_MAX;

        RCLCPP_INFO(this->get_logger(), "Publishing: center=%.3f, radius=%.3f",  message.center.x, message.radius);
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<my_first_ros_interfaces::msg::Circle>::SharedPtr publisher_;
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
