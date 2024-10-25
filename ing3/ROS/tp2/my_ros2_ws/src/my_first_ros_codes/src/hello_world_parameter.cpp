#include "rclcpp/rclcpp.hpp"
class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("hello_world_parameter_node")
    {
        this->declare_parameter("target_parameter", "world");
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&MyNode::
        timerCallback, this));
        param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        auto cb = [this](const rclcpp::Parameter & p) {
            RCLCPP_INFO(
                this->get_logger(), "parameter \"%s\" of type %s update with: \"%s\"",
                p.get_name().c_str(),
                p.get_type_name().c_str(),
                p.as_string().c_str()
            );
        };
        cb_handle_ = param_subscriber_->add_parameter_callback("target_parameter", cb);
    }
private:
    void timerCallback()
    {
        std::string target_param = this->get_parameter("target_parameter").get_parameter_value().
        get<std::string>();
        RCLCPP_INFO(this->get_logger(), "Hello %s!", target_param.c_str());
    }
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
