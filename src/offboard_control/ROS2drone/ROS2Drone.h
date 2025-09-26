#pragma once

#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include "../utils/Readyaml.h"

// Forward declaration
class TaskBase;
#include "ROS2PosController.h"
#include "ROS2StatusController.h"
#include "../drone/AbstractDrone.h"
#include "../drone/PosSubscriber.h"
#include "../drone/PosPublisher.h"
#include "../utils/math.h"
#include "../utils/utils.h"

// #include "../YOLO.h"
// #include "StatusController.h"
// #include "InertialNav.h"
// #include "PosController.h"
// #include "../CameraGimbal.h"
// #include "../ServoController.h"

class ROS2Drone : public AbstractDrone {
public:
    // Private constructor
    ROS2Drone(const std::string ardupilot_namespace) :
        topic_namespace{ardupilot_namespace}
    {
        node = rclcpp::Node::make_shared("offboard_control_node");
		RCLCPP_INFO(node->get_logger(), "ROS2Drone: Starting Offboard Control example");


		// std::shared_ptr<PosPublisher> pos_publisher = std::make_shared<PosPublisher>();
		// std::shared_ptr<PosSubscriber> pos_subscriber = std::make_shared<PosSubscriber>();

		// // Create the controllers
		// sta_ctl = std::make_shared<ROS2StatusController>();
		// pos_publisher->CreateROS2Topics(node, topic_namespace);
		// pos_subscriber->CreateROS2Topics(node, topic_namespace);
		
		// pos_ctl = std::make_shared<ROS2PosController>(pos_subscriber, pos_publisher);
		// pos_ctl->CreateROS2Topics(node, topic_namespace);
		// sta_ctl->CreateROS2Topics(node, topic_namespace);

		// // Initialize controllers
		// initialize_controllers(sta_ctl, pos_ctl);

        // Declare and get parameters
        std::stringstream ss;
        ss << "--ros-args -p";
        node->declare_parameter("sim_mode", false);
        node->get_parameter("sim_mode", sim_mode_);
        ss << " sim_mode:=" << (sim_mode_ ? "true" : "false");
        node->declare_parameter("debug_mode", false);
        node->get_parameter("debug_mode", debug_mode_);
        ss << " debug_mode:=" << (debug_mode_ ? "true" : "false");
        node->declare_parameter("print_info", false);
        node->get_parameter("print_info", print_info_);
        ss << " print_info:=" << (print_info_ ? "true" : "false");
        node->declare_parameter("fast_mode", false);
        node->get_parameter("fast_mode", fast_mode_);
        ss << " fast_mode:=" << (fast_mode_ ? "true" : "false");
        RCLCPP_INFO_STREAM(node->get_logger(), ss.str());

        timer_ = node->create_wall_timer(
            wait_time,
            std::bind(&ROS2Drone::timer_callback, this)
        );

		// Initialize timestamp
		timestamp_init = node->get_clock()->now().seconds();
    }

     ~ROS2Drone() = default;
    
	std::shared_ptr<rclcpp::Node> get_node() {
        return node;
    }
    
	// 运行接口
	virtual void timer_callback(void);

    // // 日志接口
    //  int save_log(bool finish = false) = 0;

    // 发布接口
    void publish_current_state();

    // 计时器相关接口
	double get_cur_time() {
		auto now = node->get_clock()->now();
		return now.seconds() - timestamp_init;  // 直接计算时间差并转为秒
	}

	// 参数接口
	bool sim_mode_ = false; // 是否为仿真模式
	bool debug_mode_ = false; // 是否验证状态
	bool print_info_ = false; // 是否打印信息
	bool fast_mode_ = false; // 是否快速模式

	std::shared_ptr<ROS2StatusController> sta_ctl;
	std::shared_ptr<ROS2PosController> pos_ctl;

    // 使用输出方法
    void log_info(const std::string& format, ...);
    void log_warn(const std::string& format, ...);
    void log_error(const std::string& format, ...);
    void log_debug(const std::string& format, ...);
	void log_debug_throttle(const std::chrono::milliseconds& wait_time, const std::string& format, ...);
	void log_info_throttle(const std::chrono::milliseconds& wait_time, const std::string& format, ...);
	void log_warn_throttle(const std::chrono::milliseconds& wait_time, const std::string& format, ...);
	void log_error_throttle(const std::chrono::milliseconds& wait_time, const std::string& format, ...);

    // 访问接口
    void accept(std::shared_ptr<TaskBase> visitor) override;

protected:
	rclcpp::Node::SharedPtr node;
    std::string topic_namespace;
	rclcpp::TimerBase::SharedPtr timer_;

	double timestamp_init = 0; // 初始化时间戳
};