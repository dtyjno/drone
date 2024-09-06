#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include <mavros_msgs/srv/command_home.hpp>
#include <mavros_msgs/msg/home_position.hpp>

using namespace std::chrono_literals;

//set_home_position() 设置家位置为当前位置

class OffboardControl : public rclcpp::Node {
public:
	// OffboardControl(const std::string ardupilot_namespace,std::shared_ptr<YOLO> yolo_,std::shared_ptr<ServoController> servo_controller_) :
	OffboardControl():
    	Node("offboard_control_srv")
        {

			set_home_client_ = this->create_client<mavros_msgs::srv::CommandHome>("mavros/cmd/set_home");
			// * /mavros/home_position/home [mavros_msgs/msg/HomePosition] 1 subscriber
			// ros2 topic pub /mavros/home_position/home mavros_msgs/msg/HomePosition '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, geo: {latitude: 0.0, longitude: 0.0, altitude: 0.0}, position: {x: 0.0, y: 0.0, z: 0.0}}'
			home_position_subscription_ = this->create_subscription<mavros_msgs::msg::HomePosition>("mavros/home_position/home", 10,
			std::bind(&OffboardControl::home_position_callback, this, std::placeholders::_1));

        }
private:
	typedef struct{
		double x;
		double y;
		double z;
	}Location;
	Location home_position;
	rclcpp::Subscription<mavros_msgs::msg::HomePosition>::SharedPtr home_position_subscription_;
	rclcpp::Client<mavros_msgs::srv::CommandHome>::SharedPtr set_home_client_;
	void home_position_callback(const mavros_msgs::msg::HomePosition::SharedPtr msg);
	void set_home_position();
};


// 设置无人机家的位置
// /mavros/cmd/set_home [mavros_msgs/srv/CommandHome]
void OffboardControl::set_home_position()
{
	auto request = std::make_shared<mavros_msgs::srv::CommandHome::Request>();
	request->current_gps = true;

	while (!set_home_client_->wait_for_service(std::chrono::seconds(1))) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
			return;
		}
		RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
	}
	RCLCPP_INFO(this->get_logger(), "set home command send");
	auto result_future = set_home_client_->async_send_request(request,
		[this](rclcpp::Client<mavros_msgs::srv::CommandHome>::SharedFuture future) {
			auto status = future.wait_for(0.5s);
			if (status == std::future_status::ready) {
				auto reply = future.get()->success;
				RCLCPP_INFO(this->get_logger(), "Set Home Position: %s", reply ? "success" : "failed");
				if (reply) {
					// Code to execute if the future is successful
				}
				else {
					// Code to execute if the future is unsuccessful
					RCLCPP_ERROR(this->get_logger(), "Failed to call service mavros/cmd/set_home");
				}
			} else {
				// Wait for the result.
				RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
			}
		});
}

// 接收home位置数据
void OffboardControl::home_position_callback(const mavros_msgs::msg::HomePosition::SharedPtr msg) 
{
	home_position.x = msg->position.x;
	home_position.y = msg->position.y;
	home_position.z = msg->position.z;
	// RCLCPP_INFO(this->get_logger(), "Received home position data");
	// RCLCPP_INFO(this->get_logger(), "Latitude: %f", msg->geo.latitude);
	// RCLCPP_INFO(this->get_logger(), "Longitude: %f", msg->geo.longitude);
	// RCLCPP_INFO(this->get_logger(), "Altitude: %f", msg->geo.altitude);
	// RCLCPP_INFO(this->get_logger(), "X: %f", msg->position.x);
	// RCLCPP_INFO(this->get_logger(), "Y: %f", msg->position.y);
	// RCLCPP_INFO(this->get_logger(), "Z: %f", msg->position.z);
}

