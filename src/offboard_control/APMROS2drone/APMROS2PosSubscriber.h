#pragma once


#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/altitude.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/range.hpp>

#include "../utils/math.h"

#include "../ROS2drone/ROS2PosSubscriber.h"

class APMROS2PosSubscriber: public ROS2PosSubscriber
{
public:
    APMROS2PosSubscriber()
    {
		RCLCPP_INFO(rclcpp::get_logger("APMROS2PosSubscriber"), "Default constructor called.");
	}

	// 初始化ROS2话题和服务
	void init_ROS2_topics() override;

	// 继承抽象类PosSubscriber中定义的读写接口

	// bool get_position(Vector3f& position){
	// 	position = APMROS2PosSubscriber::position;
	// 	return position.isZero();
	// }
	// bool get_forward_speed(float & velocity){
	// 	Vector2f _velocity = {APMROS2PosSubscriber::velocity.x(),APMROS2PosSubscriber::velocity.y()};
	// 	velocity = _velocity.norm(); // 长度
	// 	return _velocity.isZero();	
	// }
	// Vector2f groundspeed_vector(){
	// 	return {velocity.x(),velocity.y()};
	// }
private:
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr status_subscription_;
	rclcpp::Subscription<mavros_msgs::msg::Altitude>::SharedPtr altitude_subscription_;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_subscription_;
	rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr rangefinder_subscription_; // 激光雷达高度

	void status_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
	void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
	void altitude_callback(const mavros_msgs::msg::Altitude::SharedPtr msg);
	void imu_data_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
	void rangefinder_callback(const sensor_msgs::msg::Range::SharedPtr msg);
	
};