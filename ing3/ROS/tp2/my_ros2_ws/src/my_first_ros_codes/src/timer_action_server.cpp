#include "my_first_ros_interfaces/action/timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
using namespace std::placeholders;
class TimerActionServer : public rclcpp::Node
{
public:
    TimerActionServer() : Node("timer_action_server_node")
    {
        this->action_server_ = rclcpp_action::create_server<my_first_ros_interfaces::action::Timer>(this, "timer", std::bind(&TimerActionServer::handle_goal, this, _1, _2), std::bind(&
        TimerActionServer::handle_cancel, this, _1), std::bind(&TimerActionServer::handle_accepted, this, _1));
}
private:
    rclcpp_action::Server<my_first_ros_interfaces::action::Timer>::SharedPtr action_server_;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const my_first_ros_interfaces::action::Timer::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request %.2f s", goal->timer_target);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<my_first_ros_interfaces::action::Timer>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<my_first_ros_interfaces::action::Timer>> goal_handle)
    {
        // we need a quick return to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&TimerActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<my_first_ros_interfaces::action::Timer>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        float timer_step = 4.0; // timer step in hertz
        rclcpp::Rate loop_rate(timer_step);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<my_first_ros_interfaces::action::Timer::Feedback>();
        auto result = std::make_shared<my_first_ros_interfaces::action::Timer::Result>();
        float elapsed_time = 0.;
        for (int num_step = 0; (num_step/timer_step < goal->timer_target) && rclcpp::ok(); num_step++)
        {
            elapsed_time = elapsed_time + 1./timer_step;
            // Check if there is a cancel request
            if (goal_handle->is_canceling()) {
                result->timer_final = elapsed_time;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }
            feedback->timer_elapsed = elapsed_time;
            // Publish feedback
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publish feedback");
            loop_rate.sleep();
        }
        // Check if goal is reached
        if (rclcpp::ok()) {
            result->timer_final = elapsed_time;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded : result = %.2f s", result->timer_final);
        }
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Action server Timer ready.");
    rclcpp::spin(std::make_shared<TimerActionServer>());
    rclcpp::shutdown();
}
