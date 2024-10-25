#include "my_first_ros_interfaces/action/timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
using namespace std::placeholders;

class TimerActionClient : public rclcpp::Node
{
public:
    TimerActionClient() : Node("Timer_action_client_node")
    {
        this->client_ptr_ = rclcpp_action::create_client<my_first_ros_interfaces::action::Timer>(this, "timer");
    }
    void send_goal(char argv[])
    {
        if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }
        auto goal_msg = my_first_ros_interfaces::action::Timer::Goal();
        goal_msg.timer_target = std::atof(argv);
        RCLCPP_INFO(this->get_logger(), "Sending goal");
        auto send_goal_options = rclcpp_action::Client<my_first_ros_interfaces::action::Timer>::
        SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&TimerActionClient::goal_response_callback, this, _1);

        send_goal_options.feedback_callback = std::bind(&TimerActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&TimerActionClient::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }
private:
    rclcpp_action::Client<my_first_ros_interfaces::action::Timer>::SharedPtr client_ptr_;
    void goal_response_callback(const rclcpp_action::ClientGoalHandle<my_first_ros_interfaces::action::Timer>::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }
    void feedback_callback(rclcpp_action::ClientGoalHandle<my_first_ros_interfaces::action::Timer>::SharedPtr, const std::shared_ptr<const my_first_ros_interfaces::action::Timer::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Time elapsed : %.2f", feedback->timer_elapsed);
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<my_first_ros_interfaces::action::Timer>::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }
        RCLCPP_INFO(this->get_logger(), "Success : final time elapsed %.2f", result.result->
        timer_final);
        rclcpp::shutdown();
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    if (argc != 2) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: timer_action_client_node X (seconds)");
        rclcpp::shutdown();
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Action client Timer ready.");
    auto node = std::make_shared<TimerActionClient>();
    node->send_goal(argv[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
}
