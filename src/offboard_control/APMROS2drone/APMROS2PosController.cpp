#include "APMROS2PosController.h"

// // 输入位置 pid控制速度
// // 初始时 now > target : direction = true 记为正方向 否则记为负方向
// Vector3f APMROS2PosController::input_pos_xyz(Vector3f now, Vector3f target, bool fuzzy)
// {
// 	if(fuzzy){
// 		Vector3f f = pos_controller.input_pos_xyz(now, target, fuzzy);
// 		RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz: x:%f y:%f z:%f", f.x(), f.y(), f.z());
// 		return f;
// 	} else{
// 		Vector3f f = pos_controller.input_pos_xyz(now, target, fuzzy);
// 		RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz: x:%f y:%f z:%f", f.x(), f.y(), f.z());
// 		return f;
// 	}
// }

// // 输入位置 pid控制速度 yaw
// Vector4f APMROS2PosController::input_pos_xyz_yaw(Vector4f now, Vector4f target, bool fuzzy, bool calculate_or_get_vel, float vel_x, float vel_y)
// {
// 	if(fuzzy){
// 		float delta_k = 2.0f;

// 		Vector4f f = pos_controller.input_pos_xyz_yaw(now, target, fuzzy, calculate_or_get_vel, vel_x, vel_y);
// 		RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: x:%f y:%f z:%f w:%f", f.x(), f.y(), f.z(), f.w());
// 		return f;
// 	} else{
// 		Vector4f f = pos_controller.input_pos_xyz_yaw(now, target, fuzzy, calculate_or_get_vel, vel_x, vel_y);
// 		RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: x:%f y:%f z:%f w:%f", f.x(), f.y(), f.z(), f.w());
// 		return f;
// 	}
// }

// // 输入位置 pid控制速度
// Vector4f APMROS2PosController::input_pos_xyz_yaw_without_vel(Vector4f now, Vector4f target)
// {
// 	Vector4f f = pos_controller.input_pos_xyz_yaw_without_vel(now, target);
// 	RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: x:%f y:%f z:%f yaw:%f", f.x(), f.y(), f.z(), f.w());
// 	return f;
// }

// // 输入位置 bushi串级pid控制速度
// Vector4f APMROS2PosController::input_pos_vel_1_xyz_yaw(Vector4f now, Vector4f target)
// {
// 	Vector4f f = pos_controller.input_pos_vel_1_xyz_yaw(now, target);
// 	RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: x:%f y:%f z:%f", f.x(), f.y(), f.z());
// 	return f;
// }

// // 输入位置 串级pid控制速度
// Vector4f APMROS2PosController::input_pos_vel_xyz_yaw(Vector4f now, Vector4f target)
// {
// 	Vector4f f = pos_controller.input_pos_vel_xyz_yaw(now, target);
// 	RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: x:%f y:%f z:%f", f.x(), f.y(), f.z());
// 	return f;
// }

// // 发布 publish_setpoint_raw 飞行到指定位置（相对于当前位置）
// bool APMROS2PosController::publish_setpoint_world(Vector4f now, Vector4f target, double accuracy, double yaw_accuracy)
// {
// 	bool result = pos_controller.publish_setpoint_world(now, target, accuracy, yaw_accuracy);
// 	if(result){
// 		RCLCPP_INFO(node->get_logger(), "at_check_point");
// 	}
// 	return result;
// }

// // pid飞行到指定位置（相对于当前位置）
// // x:前后方向位置(m) y:左右方向位置(m) z:高度(m) yaw:偏航角(°) accuracy=DEFAULT_ACCURACY:精度(m)
// bool APMROS2PosController::trajectory_setpoint(Vector4f pos_now, Vector4f pos_target, double accuracy, double yaw_accuracy)
// {
// 	bool result = pos_controller.trajectory_setpoint(pos_now, pos_target, accuracy, yaw_accuracy);
// 	// if(first) {
// 	// 	RCLCPP_INFO(node->get_logger(), "trajectory_setpoint: pos_target_x:%f pos_target_y:%f", pos_target.x(), pos_target.y());
// 	// }
// 	// if (!is_equal(pos_target, pos_target_temp, 0.0000001f))
// 	// {
// 	// 	RCLCPP_INFO(node->get_logger(), "trajectory_setpoint: change point");
// 	// }
// 	if (result) {
// 		RCLCPP_INFO(node->get_logger(), "trajectory_setpoint: at_check_point");
// 	}
// 	return result;
// }

// // pid飞行到指定位置（相对于起飞点/世界坐标系）
// // x:前后方向位置(m) y:左右方向位置(m) z:高度(m) yaw:偏航角(°) accuracy=DEFAULT_ACCURACY:精度(m)
// bool APMROS2PosController::trajectory_setpoint_world(Vector4f pos_now, Vector4f pos_target, double accuracy, double yaw_accuracy)
// {
// 	bool result = pos_controller.trajectory_setpoint_world(pos_now, pos_target, accuracy, yaw_accuracy);
// 	// if(first) {
// 	// 	RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "(THROTTLE 1s) trajectory_setpoint: x:%f y:%f z:%f", _pos_target.x(), _pos_target.y(), _pos_target.z());
// 	// }
// 	// if (!is_equal(pos_target, pos_target_temp, 0.0000001f))
// 	// {
// 	// 	RCLCPP_INFO(node->get_logger(), "trajectory_setpoint: change point");
// 	// }
// 	if (result) {
// 		RCLCPP_INFO(node->get_logger(), "trajectory_setpoint: at_check_point");
// 	}
// 	return result;
// }

// // pid飞行到指定位置(闭环)，指定xy方向的pid参数（相对于起飞点/世界坐标系）
// bool APMROS2PosController::trajectory_setpoint_world(Vector4f pos_now, Vector4f pos_target, PID::Defaults defaults, double accuracy, double yaw_accuracy, bool calculate_or_get_vel, float vel_x, float vel_y)
// {
// 	bool result = pos_controller.trajectory_setpoint_world(pos_now, pos_target, defaults, accuracy, yaw_accuracy, calculate_or_get_vel, vel_x, vel_y);
// 	// if(first) {
// 	// 	RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "(THROTTLE 1s)trajectory_setpoint_world: pos_target_x:%f pos_target_y:%f", pos_target.x(), pos_target.y());
// 	// }
// 	// if (!is_equal(pos_target, pos_target_temp, 0.0000001f))
// 	// {
// 	// 	RCLCPP_INFO(node->get_logger(), "trajectory_setpoint: change point");
// 	// }
// 	if (result) {
// 		RCLCPP_INFO(node->get_logger(), "trajectory_setpoint_world: at_check_point");
// 	}
// 	return result;
// }

// bool APMROS2PosController::trajectory_circle(float a, float b, float height, float dt, float default_yaw, float yaw)
// {
// 	return pos_controller.trajectory_circle(a, b, height, dt, default_yaw, yaw);
// }

// // 航点设置，s型速度规划(开环)，飞行经过指定位置（相对于起飞点/世界坐标系）
// bool APMROS2PosController::trajectory_generator_world(double speed_factor, std::array<double, 3> q_goal, Vector3f max_speed, Vector3f max_accel, float tar_yaw)
// {
// 	return pos_controller.trajectory_generator_world(speed_factor, q_goal, max_speed, max_accel, tar_yaw);
// }

// float APMROS2PosController::get_speed_max()
// {
// 	return pos_controller.get_speed_max();
// }
// float APMROS2PosController::get_accel_max()
// {
// 	return pos_controller.get_accel_max();
// }
// float APMROS2PosController::get_decel_max()
// {
// 	return pos_controller.get_decel_max();
// }
// float APMROS2PosController::get_turn_rate_speed_max()
// {
// 	return pos_controller.get_turn_rate_speed_max();
// }
// float APMROS2PosController::get_jerk_max()
// {
// 	return pos_controller.get_jerk_max();
// }

// struct PosController::Limits_t APMROS2PosController::readLimits(const std::string &filename, const std::string &section)
// {
// 	return pos_controller.readLimits(filename, section);
// }

// // 最大值最大限制在宏定义范围内
// void APMROS2PosController::set_limits(struct PosController::Limits_t limits)
// {
// 	pos_controller.set_limits(limits);
// }

// void APMROS2PosController::reset_limits()
// {
// 	pos_controller.reset_limits();
// }

// void APMROS2PosController::set_pid(PID &pid, PID::Defaults defaults)
// {
// 	pos_controller.set_pid(pid, defaults);
// }

// void APMROS2PosController::reset_pid()
// {
// 	pos_controller.reset_pid();
// }


// void APMROS2PosController::reset_pid_config()
// {
// 	pos_controller.reset_pid_config();
// }

// // PID自整定 x/y/z/yaw:是否自整定对应的轴 tune_x/y/z/yaw默认为true 
// // delayMsec:自整定周期
// // 返回值：是否自整定完成 false:未完成 true:完成
// bool APMROS2PosController::auto_tune(Vector4f pos_now, Vector4f pos_target, uint32_t delayMsec, bool tune_x, bool tune_y, bool tune_z, bool tune_yaw)
// {
// 	return pos_controller.auto_tune(pos_now, pos_target, delayMsec, tune_x, tune_y, tune_z, tune_yaw);
// }

#ifdef PAL_STATISTIC_VISIBILITY
#include "vector"

void APMROS2PosController::publish_statistic(std::vector<pal_statistics_msgs::msg::Statistic> &statistics){
	// if (!node || !node->get_stats_publisher()) {
	// 	RCLCPP_ERROR(node->get_logger(), "Node or stats_publisher_ is not initialized.");
	// 	return;
	// }
    // auto msg = pal_statistics_msgs::msg::Statistics();
    // msg.header.stamp = node->get_clock()->now();
    // msg.header.frame_id = "base_link";
    
	PID* pids[] = {&pid_x, &pid_y, &pid_z, &pid_yaw};

	for(PID* pid : pids) {
		// PID输出
		auto output_stat = pal_statistics_msgs::msg::Statistic();
		output_stat.name = pid->pid_name + "_output";
		output_stat.value = pid->_pid_info.output;
		statistics.push_back(output_stat);

		// PID输入
		auto actual_stat = pal_statistics_msgs::msg::Statistic();
		actual_stat.name = pid->pid_name + "_actual";
		actual_stat.value = pid->_pid_info.actual;
		statistics.push_back(actual_stat);

		// PID目标
		auto target_stat = pal_statistics_msgs::msg::Statistic();
		target_stat.name = pid->pid_name + "_target";
		target_stat.value = pid->_pid_info.target;
		statistics.push_back(target_stat);

		// PID误差
		auto error_stat = pal_statistics_msgs::msg::Statistic();
		error_stat.name = pid->pid_name + "_error";
		error_stat.value = pid->_pid_info.error;
		statistics.push_back(error_stat);

		// kP项
		auto kP_stat = pal_statistics_msgs::msg::Statistic();
		kP_stat.name = pid->pid_name + "_kP";
		kP_stat.value = pid->_pid_info._kP;
		statistics.push_back(kP_stat);

		// kI项
		auto kI_stat = pal_statistics_msgs::msg::Statistic();
		kI_stat.name = pid->pid_name + "_kI";
		kI_stat.value = pid->_pid_info._kI;
		statistics.push_back(kI_stat);

		// kD项
		auto kD_stat = pal_statistics_msgs::msg::Statistic();
		kD_stat.name = pid->pid_name + "_kD";
		kD_stat.value = pid->_pid_info._kD;
		statistics.push_back(kD_stat);
		
		// P项
		auto p_term_stat = pal_statistics_msgs::msg::Statistic();
		p_term_stat.name = pid->pid_name + "_p_term";
		p_term_stat.value = pid->_pid_info.P;
		statistics.push_back(p_term_stat);
		
		// I项
		auto i_term_stat = pal_statistics_msgs::msg::Statistic();
		i_term_stat.name = pid->pid_name + "_i_term";
		i_term_stat.value = pid->_pid_info.I;
		statistics.push_back(i_term_stat);
		
		// D项
		auto d_term_stat = pal_statistics_msgs::msg::Statistic();
		d_term_stat.name = pid->pid_name + "_d_term";
		d_term_stat.value = pid->_pid_info.D;
		statistics.push_back(d_term_stat);
	}
    
    // msg.statistics = statistics;
    // node->get_stats_publisher()->publish(msg);
}
#endif