#pragma once

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/global_position_target.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>

#include "../ROS2drone/ROS2PosPublisher.h"

class APMROS2PosPublisher : public ROS2PosPublisher
{
public:
    APMROS2PosPublisher()
    {
        RCLCPP_INFO(rclcpp::get_logger("APMROS2PosPublisher"), "Starting Pos Publisher example");
    }

	// 初始化ROS2话题和服务
	void init_ROS2_topics() override;

    // 重写基类的虚函数
    void publish_setpoint_raw(Vector4f p, Vector4f v) override;
    void publish_setpoint_raw_global(double latitude, double longitude, double altitude, double yaw) override;
    void send_local_setpoint_command(double x, double y, double z, double yaw) override;
    bool local_setpoint_command(Vector4f now, Vector4f target, double accuracy) override;
    // void send_velocity_command_world(double linear_x, double linear_y, double linear_z, double angular_z) override;
    // void send_velocity_command_world(Vector4f v) override;
    void send_velocity_command(Vector4f v) override;
    bool send_velocity_command_with_time(Vector4f v, double time) override;
    void send_accel_command(Vector4f v) override;
    
private:
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_publisher_;
	// rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr global_gps_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_setpoint_publisher_;
	rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_raw_local_publisher_;
	rclcpp::Publisher<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr setpoint_raw_global_publisher_;
	// rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr trajectory_publisher_;
	// /mavros/setpoint_accel/accel (geometry_msgs/Vector3Stamped)
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr setpoint_accel_publisher_;
	// /mavros/setpoint_raw/attitude mavros_msgs/msg/AttitudeTarget
	rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr setpoint_raw_attitude_publisher_;
};


