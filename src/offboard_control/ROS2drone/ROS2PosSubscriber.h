#pragma once


#include "rclcpp/rclcpp.hpp"

#include "../utils/math.h"

#include "../drone/PosSubscriber.h"

class ROS2PosSubscriber: public PosSubscriber
{
public:
    ROS2PosSubscriber()
    {
		RCLCPP_INFO(rclcpp::get_logger("ROS2PosSubscriber"), "Default constructor called.");
	}

	void CreateROS2Topics(rclcpp::Node::SharedPtr node, const std::string& ardupilot_namespace) {
		this->node = node;
		this->topic_namespace = ardupilot_namespace;
		RCLCPP_INFO(node->get_logger(), "ROS2PosSubscriber: Starting ROS2 position data subscriptions");
		init_ROS2_topics();
	}

	virtual void init_ROS2_topics(){
		// 初始化ROS2话题和服务
	}
protected:
	rclcpp::Node::SharedPtr node;
	std::string topic_namespace;
};