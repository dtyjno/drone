#pragma once

#include "../utils/math.h"

class StatusController {
public:
    virtual ~StatusController() = default;
    
    // 添加 Motors 中使用的虚函数
    virtual bool takeoff(float current_z, float target_altitude, float yaw) = 0;
    virtual void switch_mode(const std::string& mode) = 0;
    virtual void arm_motors(bool arm) = 0;
    virtual void command_takeoff_or_land(std::string mode, float altitude, float yaw) = 0;

    virtual void set_home_position(float lat, float lon, float alt, float yaw) = 0;
    virtual void set_home_position(float yaw = 0.0f) = 0;
    virtual void set_param(const std::string& param_name, double value) = 0;

	std::string get_mode() const {
		return mode;
	}

    enum class State__system_status {
		MAV_STATE_UNINIT = 0, // 未初始化的系统，状态未知
		MAV_STATE_BOOT = 1, // 系统正在启动
		MAV_STATE_CALIBRATING = 2, // 系统正在校准，尚未准备好飞行
		MAV_STATE_STANDBY = 3, // 系统接地并处于待机状态
		MAV_STATE_ACTIVE = 4, // 系统处于活动状态，可能已经在空中
		MAV_STATE_CRITICAL = 5, // 系统处于非正常飞行模式，故障保护
		MAV_STATE_EMERGENCY = 6, // 失控，正在下降
		MAV_STATE_POWEROFF = 7, // 断电序列，准备关机
		MAV_STATE_FLIGHT_TERMINATION = 8 // 飞行终止
	};
    State__system_status get_system_status() const {
        return static_cast<State__system_status>(system_status);
    }
    uint8_t get_system_status_uint8_t() const {
        return system_status;
    }
    inline std::string get_state_name() const {
		switch (system_status) {
			case 0: return "MAV_STATE_UNINIT (未初始化的系统，状态未知)";
			case 1: return "MAV_STATE_BOOT (系统正在启动)";
			case 2: return "MAV_STATE_CALIBRATING (系统正在校准，尚未准备好飞行)";
			case 3: return "MAV_STATE_STANDBY (系统接地并处于待机状态)";
			case 4: return "MAV_STATE_ACTIVE (系统处于活动状态，可能已经在空中)";
			case 5: return "MAV_STATE_CRITICAL (系统处于非正常飞行模式，故障保护)";
			case 6: return "MAV_STATE_EMERGENCY (失控，正在下降)";
			case 7: return "MAV_STATE_POWEROFF (断电序列，准备关机)";
			case 8: return "MAV_STATE_FLIGHT_TERMINATION (飞行终止)";
			default: return "UNKNOWN";
		}
	}

    // 状态相关接口
    bool is_takeoff = false;
    bool armed = false;
    bool connected = false;
    bool guided = false;
    std::string mode;
    uint8_t system_status;
    // Home位置接口
	Vector3f home_position;
	Vector3f home_position_global;
	Quaternionf home_quaternion;  // 四元数

	// 命令字段
	bool takeoff_command = false;  // 起飞命令


};
