#pragma once

#include "rclcpp/rclcpp.hpp"

#include "../ROS2drone/ROS2PosController.h"
#include "APMROS2PosSubscriber.h"
#include "APMROS2PosPublisher.h"
#include "../drone/PosPublisher.h"
#include "../drone/PosSubscriber.h"

#ifdef PAL_STATISTIC_VISIBILITY
#include <pal_statistics_msgs/msg/statistics.hpp>
#include <pal_statistics_msgs/msg/statistic.hpp>
#endif


 
class APMROS2PosController : public ROS2PosController {
public:
    APMROS2PosController(std::shared_ptr<PosSubscriber> pos_subscriber, std::shared_ptr<PosPublisher> pos_publisher) :
		ROS2PosController(pos_subscriber, pos_publisher)  // 调用基类构造函数
	{
		RCLCPP_INFO(rclcpp::get_logger("APMROS2PosController"), "Starting Pos Controller example");
	}

#ifdef PAL_STATISTIC_VISIBILITY
	void publish_statistic(std::vector<pal_statistics_msgs::msg::Statistic> &statistics);
#endif

};