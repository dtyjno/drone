#pragma once

#include <string>
#include "rclcpp/rclcpp.hpp"
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/command_home.hpp>
#include <mavros_msgs/msg/home_position.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/param_set_v2.hpp>

#include "../utils/math.h"
#include "../ROS2drone/ROS2StatusController.h"

class APMROS2StatusController : public ROS2StatusController{
public:
	APMROS2StatusController()
	{
		RCLCPP_INFO(rclcpp::get_logger("APMROS2StatusController"), "Default constructor called.");
	}

	// 初始化ROS2话题和服务
	void init_ROS2_topics() override;

	// # see https://mavlink.io/en/messages/common.html#MAV_STATE
	void state_callback(const mavros_msgs::msg::State::SharedPtr msg);
	// enum class State__system_status {
	// 	MAV_STATE_UNINIT = 0, // 未初始化的系统，状态未知
	// 	MAV_STATE_BOOT = 1, // 系统正在启动
	// 	MAV_STATE_CALIBRATING = 2, // 系统正在校准，尚未准备好飞行
	// 	MAV_STATE_STANDBY = 3, // 系统接地并处于待机状态
	// 	MAV_STATE_ACTIVE = 4, // 系统处于活动状态，可能已经在空中
	// 	MAV_STATE_CRITICAL = 5, // 系统处于非正常飞行模式，故障保护
	// 	MAV_STATE_EMERGENCY = 6, // 失控，正在下降
	// 	MAV_STATE_POWEROFF = 7, // 断电序列，准备关机
	// 	MAV_STATE_FLIGHT_TERMINATION = 8 // 飞行终止
	// };
	// State__system_status get_system_status() const {
	// 	return static_cast<State__system_status>(system_status);
	// }
	// uint8_t get_system_status_uint8_t() const {
	// 	return system_status;
	// }
	// inline std::string get_state_name() const {
	// 	switch (system_status) {
	// 		case 0: return "MAV_STATE_UNINIT (未初始化的系统，状态未知)";
	// 		case 1: return "MAV_STATE_BOOT (系统正在启动)";
	// 		case 2: return "MAV_STATE_CALIBRATING (系统正在校准，尚未准备好飞行)";
	// 		case 3: return "MAV_STATE_STANDBY (系统接地并处于待机状态)";
	// 		case 4: return "MAV_STATE_ACTIVE (系统处于活动状态，可能已经在空中)";
	// 		case 5: return "MAV_STATE_CRITICAL (系统处于非正常飞行模式，故障保护)";
	// 		case 6: return "MAV_STATE_EMERGENCY (失控，正在下降)";
	// 		case 7: return "MAV_STATE_POWEROFF (断电序列，准备关机)";
	// 		case 8: return "MAV_STATE_FLIGHT_TERMINATION (飞行终止)";
	// 		default: return "UNKNOWN";
	// 	}
	// }

	void home_position_callback(const mavros_msgs::msg::HomePosition::SharedPtr msg);

	// "GUIDED"/"RTL"
	void switch_mode(const std::string& mode) override;
	// void switch_to_guided_mode();
	// void switch_to_flip_mode();
	// void switch_to_rtl_mode();
	
	void arm_motors(bool arm) override;

	//"TAKEOFF" or "LAND"
	void command_takeoff_or_land(std::string mode, float altitude = 5.0f, float yaw = 0.0f) override;

	void set_home_position(float lat, float lon, float alt, float yaw = 0.0f) override;
	void set_home_position(float yaw = 0.0f) override; // 使用当前位置

	auto get_set_home_client() {
		return set_home_client_;
	}

	bool takeoff(float local_frame_z ,float takeoff_altitude = 5.0f, float yaw = 0.0f) override;
	void set_param(const std::string& param_id, double value) override;

	enum class TakeoffState{
		init,
		wait_for_takeoff_command,
		wait_for_stable_offboard_mode,
		arm_requested,
		takeoff,
		end
	} state_ = TakeoffState::init;

private:
	rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscription_;
	rclcpp::Subscription<mavros_msgs::msg::HomePosition>::SharedPtr home_position_subscription_;
	rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_motors_client_;
	rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_switch_client_;
	rclcpp::Client<mavros_msgs::srv::CommandHome>::SharedPtr set_home_client_;
	rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
	rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
	rclcpp::Client<mavros_msgs::srv::ParamSetV2>::SharedPtr param_set_client_;
// 	系统时间
// 用于时间同步。
// Time:
// /mavros/time_reference (sensor_msgs/TimeReference)
	

};
