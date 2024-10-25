#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("change_hello_world_parameter_node")
    {
    }

    void changeParameter(char *argv[]){
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "hello_world_parameter_node");
        while (!parameters_client->wait_for_service(std::chrono::milliseconds(1000))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                rclcpp::shutdown();
                std::exit(1);
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
        parameters_client->set_parameters({rclcpp::Parameter("target_parameter", argv[1])});
    }
private:
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    if (argc != 2) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: change_hello__world_paramater newStringName ");
        return 1;
    }
    auto node = std::make_shared<MyNode>();
    node->changeParameter(argv);
    rclcpp::shutdown();
    return 0;
}
