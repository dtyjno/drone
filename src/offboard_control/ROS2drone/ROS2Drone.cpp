#include "ROS2Drone.h"
#include <cstdarg>
#include <cstdio>
#include "../task/TaskBase.h"

void ROS2Drone::timer_callback(void){
	static int call_count = 0;
	call_count++;
	if (call_count % 20 == 0) { // 每20次调用打印一次
		RCLCPP_INFO(node->get_logger(), "ROS2Drone: Timer callback called %d times", call_count);
	}
	// 发布当前状态
	// publish_current_state();
}

void ROS2Drone::accept(std::shared_ptr<TaskBase> visitor) {
	// std::cout << "ROS2Drone accepting visitor." << std::endl;
	std::shared_ptr<TaskBase> final_visitor = visitor->final_task();
	// std::cout << "Final task to execute: " << final_visitor->get_string() << std::endl;
	final_visitor->visit(std::shared_ptr<ROS2Drone>(this, [](ROS2Drone*){})); 
	// visitor->final_task()->visit(std::shared_ptr<ROS2Drone>(this, [](ROS2Drone*){}));
}

// template<typename ... Args>
// void ROS2Drone::log_info(const Args&... args) {
// 	std::ostringstream oss;
// 	(oss << ... << args);  // C++17 折叠表达式
// 	RCLCPP_INFO_STREAM(node->get_logger(), oss.str());
// }

// template<typename ... Args>
// void ROS2Drone::log_warn(const Args&... args) {
// 	std::ostringstream oss;
// 	(oss << ... << args);  // C++17 折叠表达式
// 	RCLCPP_WARN_STREAM(node->get_logger(), oss.str());
// }

// template<typename ... Args>
// void ROS2Drone::log_debug(const Args&... args) {
// 	std::ostringstream oss;
// 	(oss << ... << args);  // C++17 折叠表达式
// 	RCLCPP_DEBUG_STREAM(node->get_logger(), oss.str());
// }

// template<typename ... Args>
// void ROS2Drone::log_error(const Args&... args) {
// 	std::ostringstream oss;
// 	(oss << ... << args);  // C++17 折叠表达式
// 	RCLCPP_ERROR_STREAM(node->get_logger(), oss.str());
// }

// template<typename ... Args>
// void ROS2Drone::log_debug_throttle(const std::chrono::milliseconds& wait_time, const Args&... args) {
// 	std::ostringstream oss;
// 	(oss << ... << args);  // C++17 折叠表达式
// 	RCLCPP_DEBUG_THROTTLE(node->get_logger(), *node->get_clock(), wait_time.count(), "%s", oss.str().c_str());
// }

// template<typename ... Args>
// void ROS2Drone::log_info_throttle(const std::chrono::milliseconds& wait_time, const Args&... args) {
// 	std::ostringstream oss;
// 	(oss << ... << args);  // C++17 折叠表达式
// 	RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), wait_time.count(), "%s", oss.str().c_str());
// }

// template<typename ... Args>
// void ROS2Drone::log_warn_throttle(const std::chrono::milliseconds& wait_time, const Args&... args) {
// 	std::ostringstream oss;
// 	(oss << ... << args);  // C++17 折叠表达式
// 	RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), wait_time.count(), "%s", oss.str().c_str());
// }

// template<typename ... Args>
// void ROS2Drone::log_error_throttle(const std::chrono::milliseconds& wait_time, const Args&... args) {
// 	std::ostringstream oss;
// 	(oss << ... << args);  // C++17 折叠表达式
// 	RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), wait_time.count(), "%s", oss.str().c_str());
// }
