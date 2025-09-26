#pragma once

#include <string>
#include "rclcpp/rclcpp.hpp"

#include "../utils/math.h"
#include "../drone/StatusController.h"

class ROS2StatusController : public StatusController{
public:
	ROS2StatusController()
	{
		RCLCPP_INFO(rclcpp::get_logger("ROS2StatusController"), "Default constructor called.");
	}

	void CreateROS2Topics(rclcpp::Node::SharedPtr node, const std::string& topic_namespace){
		this->node = node;
		this->topic_namespace = topic_namespace;
		RCLCPP_INFO(node->get_logger(), "ROS2StatusController: Starting ROS2 status controller");
		init_ROS2_topics();
	}

	virtual void init_ROS2_topics(){
		// 初始化ROS2话题和服务
	}

	// State: GUIDED, Armed: 0, Connected: 1, Guided: 1, System Status: 
	// State: GUIDED, Armed: 1, Connected: 1, Guided: 1, System Status:
	// State: RTL, Armed: 1, Connected: 1, Guided: 1, System Status:  
	// State: LAND, Armed: 1, Connected: 1, Guided: 1, System Status:
	// bool is_takeoff = false;
	// bool armed;
	// bool connected;
	// bool guided;
	// std::string mode;
	// uint8_t system_status; // MAV系统状态
	// Vector3f home_position;//{FLT_MIN,0,0};
	// Vector3f home_position_global;
	// Quaternionf home_quaternion;// 四元数
protected:
	rclcpp::Node::SharedPtr node;
	std::string topic_namespace;
};
