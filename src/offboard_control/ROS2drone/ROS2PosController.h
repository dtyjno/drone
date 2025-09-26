#pragma once

#include "rclcpp/rclcpp.hpp"
#include <vector>

#include "../drone/PosController.h"
#include "../drone/PosPublisher.h"
#include "../drone/PosSubscriber.h"

#ifdef PAL_STATISTIC_VISIBILITY
#include <pal_statistics_msgs/msg/statistics.hpp>
#include <pal_statistics_msgs/msg/statistic.hpp>
#endif


 
class ROS2PosController : public PosController {
public:
    ROS2PosController(std::shared_ptr<PosSubscriber> pos_subscriber, std::shared_ptr<PosPublisher> pos_publisher) :
		PosController(pos_subscriber, pos_publisher)  // 调用基类构造函数
	{
		RCLCPP_INFO(rclcpp::get_logger("ROS2PosController"), "ROS2PosController: Starting Pos Controller example");
	}

	void CreateROS2Topics(rclcpp::Node::SharedPtr node, const std::string& topic_namespace){
		this->node = node;
		this->topic_namespace = topic_namespace;
		RCLCPP_INFO(node->get_logger(), "ROS2PosController: Starting ROS2 position controller");
        init_ROS2_topics();
	}

	virtual void init_ROS2_topics(){
		// 初始化ROS2话题和服务
	}

#ifdef PAL_STATISTIC_VISIBILITY
	virtual void publish_statistic(std::vector<pal_statistics_msgs::msg::Statistic> &statistics) {
		// 基类默认实现为空
		(void)statistics; // 避免未使用参数警告
	}
#endif

protected:
	rclcpp::Node::SharedPtr node;
	std::string topic_namespace;
};