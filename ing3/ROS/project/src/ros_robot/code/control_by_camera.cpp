#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/twist.hpp>

#include "algo_frame.hpp"

using std::placeholders::_1;


class ControlByCamera : public rclcpp::Node 
{
public:
	ControlByCamera() : Node("control_by_camera")
	{
		subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
			"/camera/image_raw", 10, std::bind(&ControlByCamera::frame_callback, this, _1));
		
		publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/ackermann_steering_controller/reference_unstamped", 10);
	}

private:
	void frame_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
	{
		try {
			int direction = 0;
			int finish = 0;

			cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
			cv::imshow("Frame", frame);
			cv::waitKey(10);

			process_frame(frame, &direction, &finish, this->get_logger());

			if (! finish)
			{
				geometry_msgs::msg::Twist resp = geometry_msgs::msg::Twist();
				resp.linear.x = 1.5;
				resp.angular.z = (float) direction / 128.0 / 2.5;
				publisher_->publish(resp);
			}
		}
		catch (cv_bridge::Exception& e) {
			RCLCPP_ERROR(this->get_logger(), "cv_bridge exception : %s", e.what());
		}
	}
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ControlByCamera>());
	rclcpp::shutdown();
	return 0;
}
