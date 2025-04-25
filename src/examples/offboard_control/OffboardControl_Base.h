#ifndef OFFBOARD_CONTROL_BASE_H
#define OFFBOARD_CONTROL_BASE_H

#include "rclcpp/rclcpp.hpp"
#include "Vector4.h"
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <sensor_msgs/msg/nav_sat_fix.hpp>
// #include <geometry_msgs/msg/twist_stamped.hpp>
// #include <mavros_msgs/msg/altitude.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
// #include <mavros_msgs/msg/home_position.hpp>
// #include <mavros_msgs/msg/state.hpp>

#define DEFAULT_YAW (3 * M_PI_2) // default yaw for position control

class OffboardControl_Base : public rclcpp::Node
{
public:
	OffboardControl_Base(std::string ardupilot_namespace) : Node("offboard_control_srv"),
																													mode_switch_client_{this->create_client<mavros_msgs::srv::SetMode>(ardupilot_namespace + "set_mode")}
	{
		RCLCPP_INFO(this->get_logger(), "Starting Offboard Control example");
	}

	// virtual void set_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	// virtual void set_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
	// virtual void set_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
	// virtual void set_altitude(const mavros_msgs::msg::Altitude::SharedPtr msg);
	// virtual void set_state(const mavros_msgs::msg::State::SharedPtr msg);
	// virtual void set_home_position(const mavros_msgs::msg::HomePosition::SharedPtr msg);

	// virtual void set_pose();
	// virtual void set_gps();
	// virtual void set_velocity();
	// virtual void set_altitude();
	// virtual void set_state();
	// virtual void set_home_position();
	rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_switch_client_;
	static Vector4f start;

private:
	// class GlobalFrame{
	// public:
	// 	float lat;
	// 	float lon;
	// 	float alt;
	// };

	// Vector3f local_frame;
	// Vector3f velocity;
	// GlobalFrame global_frame;
	// float heading;
	// Eigen::Quaterniond quaternion;// 四元数
	// Eigen::Vector3d euler;// 欧拉角
	// float yaw;
};
#endif // OFFBOARDCONTROL_BASE_H