#pragma once

#include "rclcpp/rclcpp.hpp"
#include "../drone/PosPublisher.h"

class ROS2PosPublisher : public PosPublisher
{
public:
    ROS2PosPublisher()
    {
        RCLCPP_INFO(rclcpp::get_logger("ROS2PosPublisher"), "Default constructor called.");
    }

    void CreateROS2Topics(rclcpp::Node::SharedPtr node, const std::string topic_namespace){
        this->node = node;
        this->topic_namespace = topic_namespace;
        RCLCPP_INFO(node->get_logger(), "ROS2PosPublisher: Initializing ROS2 publishers");
        init_ROS2_topics();
	}

	virtual void init_ROS2_topics(){
		// 初始化ROS2话题和服务
	}
protected:
    rclcpp::Node::SharedPtr node;
    std::string topic_namespace;
};


