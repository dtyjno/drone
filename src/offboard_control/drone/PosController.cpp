#include "PosController.h"

// 输入位置 pid控制速度
// 初始时 now > target : direction = true 记为正方向 否则记为负方向
Vector3f PosController::input_pos_xyz(Vector3f now, Vector3f target, bool fuzzy)
{
	if(fuzzy){
		float delta_k = 2.0f;

		Vector3f f;
		float kp = 0, ki = 0, kd = 0;

		pid_x.get_pid(kp, ki, kd);
		fuzzy_pid.fuzzy_pid_control(now.x(), target.x(), 0, kp, ki, kd, delta_k);
		pid_x.set_gains(kp, ki, kd);
		f.x() = pid_x.update_all(now.x(), target.x(), dt, max_speed_xy, this->pos_data->get_velocity().x());

		pid_y.get_pid(kp, ki, kd);
		fuzzy_pid.fuzzy_pid_control(now.y(), target.y(), 1, kp, ki, kd, delta_k);
		pid_y.set_gains(kp, ki, kd);
		f.y() = pid_y.update_all(now.y(), target.y(), dt, max_speed_xy, this->pos_data->get_velocity().y());
		
		pid_z.get_pid(kp, ki, kd);
		fuzzy_pid.fuzzy_pid_control(now.z(), target.z(), 2, kp, ki, kd, delta_k);
		pid_z.set_gains(kp, ki, kd);
		f.z() = pid_z.update_all(now.z(), target.z(), dt, max_speed_z, this->pos_data->get_velocity().z());
		
		// RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz: x:%f y:%f z:%f", f.x(), f.y(), f.z());
		return f;
	} else{
		Vector3f f;
		f.x() = pid_x.update_all(now.x(), target.x(), dt, max_speed_xy, this->pos_data->get_velocity().x());
		f.y() = pid_y.update_all(now.y(), target.y(), dt, max_speed_xy, this->pos_data->get_velocity().y());
		f.z() = pid_z.update_all(now.z(), target.z(), dt, max_speed_z, this->pos_data->get_velocity().z());
		// RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz: x:%f y:%f z:%f", f.x(), f.y(), f.z());
		return f;
	}
}

// 输入位置 pid控制速度 yaw
Vector4f PosController::input_pos_xyz_yaw(Vector4f now, Vector4f target, bool fuzzy, bool calculate_or_get_vel, float vel_x, float vel_y)
{
	// 处理yaw跨越点问题
	float yaw_diff_last = 0;
	float yaw_diff = target.w() - now.w();
	// 检测并调整yaw差值，确保其在-π到π范围内
	while (yaw_diff > M_PI)
	{
		yaw_diff -= 2 * M_PI; // 如果差值大于π，减去2π调整
		yaw_diff_last -= -2 * M_PI;
	}
	while (yaw_diff < -M_PI)
	{
		yaw_diff += 2 * M_PI; // 如果差值小于-π，加上2π调整
		yaw_diff_last += 2 * M_PI;
	}
	// std::cout << "yaw_diff: " << yaw_diff << ", yaw_diff_last: " << yaw_diff_last << std::endl;
	// std::cout << "now: " << now.x() << ", " << now.y() << ", " << now.z() << ", " << now.w() + yaw_diff_last << std::endl;
	if(fuzzy){
		float delta_k = 2.0f;

		Vector4f f;
		float kp = 0, ki = 0, kd = 0;

		pid_x.get_pid(kp, ki, kd);
		// RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: x   p:%f i:%f d:%f", 
		// 	kp, ki, kd);
		fuzzy_pid.fuzzy_pid_control(now.x(), target.x(), 0, kp, ki, kd, delta_k);
		pid_x.set_gains(kp, ki, kd);
		f.x() = pid_x.update_all(now.x(), target.x(), dt, max_speed_xy, calculate_or_get_vel ? vel_x : this->pos_data->get_velocity().x());
		// RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: x   p:%f i:%f d:%f", 
		// 	kp, ki, kd);
		pid_y.get_pid(kp, ki, kd);
		fuzzy_pid.fuzzy_pid_control(now.y(), target.y(), 1, kp, ki, kd, delta_k);
		pid_y.set_gains(kp, ki, kd);
		f.y() = pid_y.update_all(now.y(), target.y(), dt, max_speed_xy, calculate_or_get_vel ? vel_y : this->pos_data->get_velocity().y());
		// RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: y   p:%f i:%f d:%f", 
		// 	kp, ki, kd);		
		pid_z.get_pid(kp, ki, kd);
		fuzzy_pid.fuzzy_pid_control(now.z(), target.z(), 2, kp, ki, kd, delta_k);
		pid_z.set_gains(kp, ki, kd);
		f.z() = pid_z.update_all(now.z(), target.z(), dt, max_speed_z, this->pos_data->get_velocity().z());
		// RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: z   p:%f i:%f d:%f", 
		// 	kp, ki, kd);
		pid_yaw.get_pid(kp, ki, kd);
		fuzzy_pid.fuzzy_pid_control(now.w(), target.w(), 3, kp, ki, kd, delta_k);
		pid_yaw.set_gains(kp, ki, kd);
		f.w() = pid_yaw.update_all(now.w(), target.w() + yaw_diff_last, dt, max_speed_yaw, this->pos_data->get_velocity_yaw());
		// RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: yaw p:%f i:%f d:%f", 
		// 	kp, ki, kd);
		RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: px:%f py:%f pz:%f pyaw:%f",
			now.x(), now.y(), now.z(), now.w() + yaw_diff_last);
		RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: tx:%f ty:%f tz:%f tyaw:%f",
			target.x(), target.y(), target.z(), target.w());
		RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: vx:%f vy:%f vz:%f vyaw:%f",
			f.x(), f.y(), f.z(), f.w());
		return f;
	} else{
		Vector4f f;
		f.x() = pid_x.update_all(now.x(), target.x(), dt, max_speed_xy, calculate_or_get_vel ? vel_x : this->pos_data->get_velocity().x());
		f.y() = pid_y.update_all(now.y(), target.y(), dt, max_speed_xy, calculate_or_get_vel ? vel_y : this->pos_data->get_velocity().y());
		f.z() = pid_z.update_all(now.z(), target.z(), dt, max_speed_z, this->pos_data->get_velocity().z());
		f.w() = pid_yaw.update_all(now.w(), target.w() + yaw_diff_last, dt, max_speed_yaw, this->pos_data->get_velocity_yaw());
		// RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: vx:%f vy:%f vz:%f vyaw:%f", f.x(), f.y(), f.z(), f.w());
		return f;
	}
}

// 输入位置 pid控制速度
Vector4f PosController::input_pos_xyz_yaw_without_vel(Vector4f now, Vector4f target)
{
	Vector4f f;
	f.x() = pid_x.update_all(now.x(), target.x(), dt, max_speed_xy);
	f.y() = pid_y.update_all(now.y(), target.y(), dt, max_speed_xy);
	f.z() = pid_z.update_all(now.z(), target.z(), dt, max_speed_z);
	// 处理yaw跨越点问题
	static float yaw_diff_last = 0;
	float yaw_diff = target.w() - now.w() + yaw_diff_last;
	// 检测并调整yaw差值，确保其在-π到π范围内
	if (yaw_diff > M_PI)
	{
		yaw_diff -= 2 * M_PI; // 如果差值大于π，减去2π调整
		yaw_diff_last -= -2 * M_PI;
	}
	else if (yaw_diff < -M_PI)
	{
		yaw_diff += 2 * M_PI; // 如果差值小于-π，加上2π调整
		yaw_diff_last += 2 * M_PI;
	}
	// if (target.w() > M_PI)
	// 	target.w() -= 2 * M_PI;
	// else if (target.w() < -M_PI)
	// 	target.w() += 2 * M_PI;
	f.w() = pid_yaw.update_all(now.w(), target.w() + yaw_diff_last, dt, max_speed_yaw, this->pos_data->get_velocity_yaw());
	// RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: x:%f y:%f z:%f yaw:%f", f.x(), f.y(), f.z(), f.w());

	return f;
}

// 输入位置 bushi串级pid控制速度
Vector4f PosController::input_pos_vel_1_xyz_yaw(Vector4f now, Vector4f target)
{
	Vector4f f;
	static Vector4f v_v = {0, 0, 0, 0};
	// RCLCPP_INFO(node->get_logger(),"acc: %f,%f,%f",this->linear_acceleration.x(),this->linear_acceleration.y,this->linear_acceleration.z());
	f.x() = pid_vx.update_all(this->pos_data->get_velocity().x(), pid_px.update_all(now.x(), target.x(), 0, 2 * max_speed_xy, this->pos_data->get_velocity().x()), dt_pid_p_v, max_speed_xy); //,this->linear_acceleration.x());
	f.y() = pid_vy.update_all(this->pos_data->get_velocity().y(), pid_py.update_all(now.y(), target.y(), 0, 2 * max_speed_xy, this->pos_data->get_velocity().y()), dt_pid_p_v, max_speed_xy); //,this->linear_acceleration.y);
	f.z() = pid_vz.update_all(this->pos_data->get_velocity().z(), pid_pz.update_all(now.z(), target.z(), 0, 2 * max_speed_z, this->pos_data->get_velocity().z()), dt_pid_p_v, max_speed_z);		//,this->linear_acceleration.z()-9.80665);
																																																																																// 处理yaw跨越点问题
	float yaw_diff = target.w() - now.w();
	// 检测并调整yaw差值，确保其在-π到π范围内
	if (yaw_diff > M_PI)
	{
		yaw_diff -= 2 * M_PI; // 如果差值大于π，减去2π调整
	}
	else if (yaw_diff < -M_PI)
	{
		yaw_diff += 2 * M_PI; // 如果差值小于-π，加上2π调整
	}
	if (target.w() > M_PI)
		target.w() -= 2 * M_PI;
	else if (target.w() < -M_PI)
		target.w() += 2 * M_PI;
	f.w() = pid_yaw.update_all(now.w(), target.w(), dt, max_speed_yaw, this->pos_data->get_velocity_yaw());
	Vector3f vel_3f = this->pos_data->get_velocity();
	v_v = Vector4f(vel_3f.x(), vel_3f.y(), vel_3f.z(), this->pos_data->get_velocity_yaw());
	// RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: x:%f y:%f z:%f", f.x(), f.y(), f.z());

	return f;
}
// 输入位置 串级pid控制速度
Vector4f PosController::input_pos_vel_xyz_yaw(Vector4f now, Vector4f target)
{
	Vector4f f;
	static Vector4f v_v = {0, 0, 0, 0};
	// RCLCPP_INFO(node->get_logger(),"acc: %f,%f,%f",this->linear_acceleration.x(),this->linear_acceleration.y,this->linear_acceleration.z());
	f.x() = pid_vx.update_all(this->pos_data->get_velocity().x(), pid_px.update_all(now.x(), target.x(), 0, max_speed_xy, this->pos_data->get_velocity().x()), dt_pid_p_v, max_accel_xy); //,this->linear_acceleration.x());
	f.y() = pid_vy.update_all(this->pos_data->get_velocity().y(), pid_py.update_all(now.y(), target.y(), 0, max_speed_xy, this->pos_data->get_velocity().y()), dt_pid_p_v, max_accel_xy); //,this->linear_acceleration.y);
	f.z() = pid_vz.update_all(this->pos_data->get_velocity().z(), pid_pz.update_all(now.z(), target.z(), 0, max_speed_z, this->pos_data->get_velocity().z()), dt_pid_p_v, max_accel_z);		//,this->linear_acceleration.z()-9.80665);
																																																																														// 处理yaw跨越点问题
	float yaw_diff = target.w() - now.w();
	// 检测并调整yaw差值，确保其在-π到π范围内
	if (yaw_diff > M_PI)
	{
		yaw_diff -= 2 * M_PI; // 如果差值大于π，减去2π调整
	}
	else if (yaw_diff < -M_PI)
	{
		yaw_diff += 2 * M_PI; // 如果差值小于-π，加上2π调整
	}
	f.w() = pid_yaw.update_all(now.w(), target.w(), dt, max_speed_yaw, this->pos_data->get_velocity_yaw());
	Vector3f vel_3f = this->pos_data->get_velocity();
	v_v = Vector4f(vel_3f.x(), vel_3f.y(), vel_3f.z(), this->pos_data->get_velocity_yaw());
	// RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: x:%f y:%f z:%f", f.x(), f.y(), f.z());

	return f;
}
// 发布 publish_setpoint_raw 飞行到指定位置（相对于当前位置）
bool PosController::publish_setpoint_world(Vector4f now, Vector4f target, double accuracy, double yaw_accuracy)
{
	(void)yaw_accuracy;
	static bool first = true;
	static Vector4f pos_start;
	if (first)
	{
		pos_start = now;
		reset_pid();
	}
	// Vector4f pos = {
	// 	pid_px.update_all(now.x(),target.x(),0,2*max_speed_xy,this->pos_data->get_velocity().x()),
	// 	pid_py.update_all(now.y,target.y,0,2*max_speed_xy,this->pos_data->get_velocity().y),
	// 	pid_pz.update_all(now.z(),target.z(),0,2*max_speed_z,this->pos_data->get_velocity().z()),
	// 	pid_yaw.update_all(now.w(),target.w(),dt,max_speed_yaw,this->pos_data->get_velocity_yaw())
	// };
	// Vector4f vel = {
	// 	pid_vx.update_all(this->pos_data->get_velocity().x(),pos.x(),dt_pid_p_v,max_speed_xy),//,this->linear_acceleration.x());
	// 	pid_vy.update_all(this->pos_data->get_velocity().y,pos.y,dt_pid_p_v,max_speed_xy),//,this->linear_acceleration.y);
	// 	pid_vz.update_all(this->pos_data->get_velocity().z(),pos.z(),dt_pid_p_v,max_speed_z),//,this->linear_acceleration.z()-9.80665);
	// 	0
	// };(*objectname) [1ms]
	Vector4f pos = {
			pid_px.update_all(now.x(), target.x(), 0, 2 * max_speed_xy, this->pos_data->get_velocity().x()),
			pid_py.update_all(now.y(), target.y(), 0, 2 * max_speed_xy, this->pos_data->get_velocity().y()),
			pid_pz.update_all(now.z(), target.z(), 0, 2 * max_speed_z, this->pos_data->get_velocity().z()),
			pid_yaw.update_all(now.w(), target.w(), dt, max_speed_yaw, this->pos_data->get_velocity_yaw())};
	Vector4f vel;
	pos_publisher->publish_setpoint_raw(pos, vel);
	if (abs(now.x() - target.x()) <= accuracy &&
			abs(now.y() - target.y()) <= accuracy &&
			abs(now.z() - target.z()) <= accuracy
			// #ifndef PID_P
			// && abs(pos_now.w() - _pos_target.w())<=yaw_accuracy
			// #endif
	)
	{
		// RCLCPP_INFO(node->get_logger(), "at_check_point");
		first = true;
		return true;
	}
	return false;
}
// pid飞行到指定位置（相对于当前位置）
// x:前后方向位置(m) y:左右方向位置(m) z:高度(m) yaw:偏航角(°) accuracy=DEFAULT_ACCURACY:精度(m)
bool PosController::trajectory_setpoint(Vector4f pos_now, Vector4f pos_target, double accuracy, double yaw_accuracy)
{
	(void)yaw_accuracy;
	static bool first = true;
	static Vector4f pos_target_temp;
	if (first)
	{
		// pos_target.rotate_xy(default_yaw);
		_pos_target = pos_now + pos_target;
		pos_target_temp = pos_target;
		// RCLCPP_INFO(node->get_logger(), "trajectory_setpoint: x:%f y:%f", _pos_target.x(), _pos_target.y());
		reset_pid();
		
		first = false;
	}
	// if(pose_.header.stamp.sec == 0){
		// RCLCPP_INFO(this->get_logger(), "No pose data received yet");
	// 	return;
	// }
	if (is_equal(pos_target, pos_target_temp, 0.0000001f))
	{
#ifdef PID_P
		// send_velocity_command(input_pos_vel_1_xyz_yaw(pos_now,_pos_target));
		pos_publisher->send_accel_command(input_pos_vel_xyz_yaw(pos_now, _pos_target));
#endif
#ifndef PID_P
		pos_publisher->send_velocity_command(input_pos_xyz_yaw(pos_now, _pos_target));
		// send_velocity_command(input_pos_xyz_yaw(pos_now, _pos_target, true, direction));
#endif
	}
	else
	{
		printf("pos_target_temp:%f,%f,%f,%f\n", pos_target_temp.x(), pos_target_temp.y(), pos_target_temp.z(), pos_target_temp.w());
		printf("pos_target:%f,%f,%f,%f\n", pos_target.x(), pos_target.y(), pos_target.z(), pos_target.w());
		// RCLCPP_INFO(node->get_logger(), "trajectory_setpoint: change point");
		first = true;
	}
	if (abs(pos_now.x() - _pos_target.x()) <= accuracy &&
			abs(pos_now.y() - _pos_target.y()) <= accuracy &&
			abs(pos_now.z() - _pos_target.z()) <= accuracy
			// #ifndef PID_P
			// && abs(pos_now.w() - _pos_target.w())<=yaw_accuracy
			// #endif
	)
	{
		// RCLCPP_INFO(node->get_logger(), "trajectory_setpoint: at_check_point");
		first = true;
		return true;
	}
	return false;
}
// pid飞行到指定位置（相对于起飞点/世界坐标系）
// x:前后方向位置(m) y:左右方向位置(m) z:高度(m) yaw:偏航角(°) accuracy=DEFAULT_ACCURACY:精度(m)
bool PosController::trajectory_setpoint_world(Vector4f pos_now, Vector4f pos_target, double accuracy, double yaw_accuracy)
{
	(void)yaw_accuracy;
	static bool first = true;
	if (first) {
		_pos_target = pos_target;
		// RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "(THROTTLE 1s) trajectory_setpoint: x:%f y:%f z:%f", _pos_target.x(), _pos_target.y(), _pos_target.z());
		reset_pid();
		first = false;
	}
	if (is_equal(pos_target, _pos_target, 0.0000001f)) {
#ifdef PID_P
		// send_velocity_command(input_pos_vel_1_xyz_yaw(pos_now,_pos_target));
		pos_publisher->send_accel_command(input_pos_vel_xyz_yaw(pos_now, _pos_target));
#endif
#ifndef PID_P
		// send_velocity_command(input_pos_xyz_yaw(pos_now, _pos_target));
		pos_publisher->send_velocity_command(input_pos_xyz_yaw(pos_now, _pos_target, true));
#endif
	}
	else {
		// RCLCPP_INFO(node->get_logger(), "trajectory_setpoint: change point");
		first = true;
	}
	if (abs(pos_now.x() - _pos_target.x()) <= accuracy &&
			abs(pos_now.y() - _pos_target.y()) <= accuracy &&
			abs(pos_now.z() - _pos_target.z()) <= accuracy
	) {
		// RCLCPP_INFO(node->get_logger(), "trajectory_setpoint: at_check_point");
		first = true;
		return true;
	}
	return false;
}

// pid飞行到指定位置(闭环)，指定xy方向的pid参数（相对于起飞点/世界坐标系）
bool PosController::trajectory_setpoint_world(Vector4f pos_now, Vector4f pos_target, PID::Defaults defaults, double accuracy, double yaw_accuracy, bool calculate_or_get_vel, float vel_x, float vel_y)
{
	(void)yaw_accuracy;
	// RCLCPP_INFO(node->get_logger(), "p:%f i:%f d:%f", defaults.p, defaults.i, defaults.d);
	static bool first = true;
	static Vector4f pos_target_temp;
	if (first)
	{
		pos_target_temp = pos_target;
		// RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "(THROTTLE 1s)trajectory_setpoint_world: pos_target_x:%f pos_target_y:%f", pos_target.x(), pos_target.y());
		// 设置pid参数
		set_pid(pid_x, defaults);
		set_pid(pid_y, defaults);
		// pid_x.set_pid_info();
		// pid_y.set_pid_info();
		// pid_z.set_pid_info();
		// pid_yaw.set_pid_info();
		first = false;
	}
	if (is_equal(pos_target, pos_target_temp, 0.0000001f))
	{
		// 运行时更新pid参数
		set_pid(pid_x, defaults);
		set_pid(pid_y, defaults);

		// #ifdef PID_P
		// // send_velocity_command(input_pos_vel_1_xyz_yaw(pos_now,_pos_target));
		// send_accel_command(input_pos_vel_xyz_yaw(pos_now,_pos_target));
		// #endif
		// #ifndef PID_P
		// send_velocity_command(input_pos_xyz_yaw_without_vel(pos_now, pos_target));
		pos_publisher->send_velocity_command(input_pos_xyz_yaw(pos_now, pos_target, true, calculate_or_get_vel, vel_x, vel_y));
		// #endif
	}
	else
	{
		// RCLCPP_INFO(node->get_logger(), "trajectory_setpoint_world: change point");
		first = true;
	}
	if (abs(pos_now.x() - pos_target.x()) <= accuracy &&
			abs(pos_now.y() - pos_target.y()) <= accuracy &&
			abs(pos_now.z() - pos_target.z()) <= accuracy
			// && abs(pos_now.w() - _pos_target.w())<=yaw_accuracy
	)
	{
		
		// RCLCPP_INFO(node->get_logger(), "trajectory_setpoint_world: at_check_point");
		// 重设为默认pid参数
		set_pid(pid_x, pid_x_defaults);
		set_pid(pid_y, pid_y_defaults);
		pid_x.reset_all();
		pid_y.reset_all();
		pid_z.reset_all();
		pid_yaw.reset_all();
		first = true;
		return true;
	}
	return false;
}

bool PosController::trajectory_circle(float a, float b, float height, float dt, float default_yaw, float yaw)
{
	static float t;
	static bool first = true;
	if (first)
	{
		t = default_yaw + yaw;
		first = false;
	}
	// RCLCPP_INFO(node->get_logger(),"trajectory_circle: t:%f t_t:%f", dt, t);

	t += dt;
	float c = cos(t);
	float s = sin(t);
	Vector4f p = {a * c, b * s, height, 0};
	Vector4f v = {-a * s, b * c, 0, 0};
	pos_publisher->publish_setpoint_raw(p, v);

	// RCLCPP_INFO(node->get_logger(), "trajectory_circle: t_n:%f t_t:%f", t, M_PI + default_yaw + yaw);
	// if (t > M_PI + default_yaw + yaw){
	if (false)
	{
		first = true;
		return true;
		pos_publisher->send_velocity_command(Vector4f::Zero());
	}
	else
	{
		return false;
	}
}
// 航点设置，s型速度规划(开环)，飞行经过指定位置（相对于起飞点/世界坐标系）
bool PosController::trajectory_generator_world(double speed_factor, std::array<double, 3> q_goal, Vector3f max_speed, Vector3f max_accel, float tar_yaw)
{
	static RobotState current_state;
	static bool first = true;
	static uint64_t count = 0;
	bool isFinished = false;
	static Vector4f pos_start_temp;
	static Vector4f pos_target_temp;

	if (first)
	{
		reset_pid();
		std::cout << this->pos_data->get_position().x() << " " << this->pos_data->get_position().y() << " " << this->pos_data->get_position().z() << " " << std::endl;
		current_state.q_d = {this->pos_data->get_position().x(), this->pos_data->get_position().y(), this->pos_data->get_position().z()};
		_trajectory_generator = std::make_unique<TrajectoryGenerator>(speed_factor, q_goal); // Use smart pointer for memory management
		_trajectory_generator->set_dq_max({MIN(max_speed_xy, max_speed.x()), MIN(max_speed_xy, max_speed.y()), MIN(max_speed_z, max_speed.z())});
		_trajectory_generator->set_dqq_max({MIN(max_accel_xy, max_accel.x()), MIN(max_accel_xy, max_accel.y()), MIN(max_accel_z, max_accel.z())});
		// _trajectory_generator->set_dq_max({1, 1, 1});
		// _trajectory_generator->set_dqq_max({0.2, 0.2, 0.2});
		pos_start_temp = Vector4f(this->pos_data->get_position().x(), this->pos_data->get_position().y(), this->pos_data->get_position().z(), 0.0f);
		pos_target_temp = Vector4f(static_cast<float>(q_goal[0]), static_cast<float>(q_goal[1]), static_cast<float>(q_goal[2]), 0);
		count = 0;
		isFinished = false;
		first = false; // Ensure that initialization block runs only once
	}
	if (is_equal(Vector4f(q_goal[0], q_goal[1], q_goal[2], 0), pos_target_temp, 0.0000001f))
	{
		if (!isFinished)
		{
			isFinished = (*_trajectory_generator)(current_state, count); // count / 10
			count++; // Increment count to simulate time passing
			// RCLCPP_INFO(node->get_logger(), "trajectory_generator: is_equal count:%ld, isFinished:%d", count, isFinished);
		}
		
		if (isFinished)
		{
			// RCLCPP_INFO(node->get_logger(), "trajectory_generator: Motion completed!");
			first = true; // Reset first to true to reinitialize the trajectory generator
			pid_x.reset_all();
			pid_y.reset_all();
			pid_z.reset_all();
			pid_yaw.reset_all();
			return true; // 返回完成状态
		}
	}
	else
	{
		// RCLCPP_INFO(node->get_logger(), "trajectory_generator: change point");
		first = true;
		return false;
	}
	std::cout << "_trajectory_generator: " << _trajectory_generator->delta_q_d[0] << " "
						<< _trajectory_generator->delta_q_d[1] << " "
						<< _trajectory_generator->delta_q_d[2] << " "
						<< std::endl;
	
	// send_local_setpoint_command(
	// 	pos_start_temp.x() + static_cast<float>(_trajectory_generator->delta_q_d[0]),
	// 	pos_start_temp.y() + static_cast<float>(_trajectory_generator->delta_q_d[1]),
	// 	pos_start_temp.z() + static_cast<float>(_trajectory_generator->delta_q_d[2]),
	// 	0
	// );

	// trajectory_setpoint_world(
	// 	{this->pos_data->get_position().x(), this->pos_data->get_position().y, this->pos_data->get_position().z(), 0},
	// 	{
	// 		// MIN(_trajectory_generator->delta_q_d[0], max_speed_xy),
	// 		// MIN(_trajectory_generator->delta_q_d[1], max_speed_xy),
	// 		// MIN(_trajectory_generator->delta_q_d[2], max_speed_xy),
	// 		pos_start_temp.x() + static_cast<float>(_trajectory_generator->delta_q_d[0]),
	// 		pos_start_temp.y + static_cast<float>(_trajectory_generator->delta_q_d[1]),
	// 		pos_start_temp.z() + static_cast<float>(_trajectory_generator->delta_q_d[2]),
	// 		0
	// 	}
	// 	,0.01f, 0.1f
	// );

	pos_publisher->send_velocity_command(
		input_pos_xyz_yaw(
			Vector4f(this->pos_data->get_position().x() - pos_start_temp.x(), this->pos_data->get_position().y() - pos_start_temp.y(), this->pos_data->get_position().z() - pos_start_temp.z(), this->pos_data->get_yaw()),
			Vector4f(static_cast<float>(_trajectory_generator->delta_q_d[0]), static_cast<float>(_trajectory_generator->delta_q_d[1]), static_cast<float>(_trajectory_generator->delta_q_d[2]), tar_yaw),
			true)
	);
	
	// send_accel_command(input_pos_vel_xyz_yaw(
	// 		{this->pos_data->get_position().x() - pos_start_temp.x(), this->pos_data->get_position().y - pos_start_temp.y, this->pos_data->get_position().z() - pos_start_temp.z(), 0},
	// 		{static_cast<float>(_trajectory_generator->delta_q_d[0]), static_cast<float>(_trajectory_generator->delta_q_d[1]), static_cast<float>(_trajectory_generator->delta_q_d[2]), 0}));

	return isFinished; // Return the status of trajectory generation
}

float PosController::get_speed_max()
{
	return max_speed_xy;
}
float PosController::get_accel_max()
{
	return max_accel_xy;
}
float PosController::get_decel_max()
{
	if (is_positive(max_dccel_xy))
	{
		return max_dccel_xy;
	}
	else
	{
		return max_accel_xy;
	}
}
float PosController::get_turn_rate_speed_max()
{
	return max_speed_yaw;
}
float PosController::get_jerk_max()
{
	return max_jerk;
}

struct PosController::Limits_t PosController::readLimits(const std::string &filename, const std::string &section)
{
	YAML::Node config = Readyaml::readYAML(filename);
	struct Limits_t limits;
	try
	{
		limits.speed_max_xy = config[section]["speed_max_xy"].as<double>();
		limits.speed_max_z = config[section]["speed_max_z"].as<double>();
		limits.speed_max_yaw = config[section]["speed_max_yaw"].as<double>();
		limits.accel_max_xy = config[section]["accel_max_xy"].as<double>();
		limits.accel_max_z = config[section]["accel_max_z"].as<double>();
	}
	catch (YAML::InvalidNode &e)
	{
		// RCLCPP_ERROR(node->get_logger(), "Error reading limits from file: %s", e.what());
		std::cerr << "Error reading limits from file: " << e.what() << std::endl;
	}
	return limits;
}

// 最大值最大限制在宏定义范围内
void PosController::set_limits(struct Limits_t limits)
{
	max_speed_xy = MAX(limits.speed_max_xy, 0);
	max_speed_z = MAX(limits.speed_max_z, 0);
	max_speed_yaw = MAX(limits.speed_max_yaw, 0);
	max_accel_xy = MAX(limits.accel_max_xy, 0);
	max_accel_z = MAX(limits.accel_max_z, 0);
	// max_dccel_xy = limits.decel_max;
	// max_jerk = MAX(limits.jerk_max, 0);
	// _p_pos.set_limits(max_speed_xy, MIN(max_accel_xy, max_dccel_xy), max_jerk);
}

void PosController::reset_limits()
{
	// max_speed_xy = PosController_VEL_XY_MAX;
	// max_speed_z = PosController_VEL_Z_MAX;
	// max_speed_yaw = PosController_VEL_YAW_MAX;
	// max_accel_xy = PosController_ACC_XY_MAX;
	// max_accel_z = PosController_ACC_Z_MAX;
	set_limits(limit_defaults);
	// max_dccel_xy = PosController_ACC_XY_MAX;
	// max_jerk = PosController_JERK_MAX;
	// _p_pos.set_limits(max_speed_xy, MIN(max_accel_xy, max_dccel_xy), max_jerk);
}

void PosController::set_pid(PID &pid, PID::Defaults defaults)
{
	pid.set_pid(defaults);
}

void PosController::reset_pid()
{
	pid_x.reset_all();
	pid_y.reset_all();
	pid_z.reset_all();
	pid_yaw.reset_all();
	pid_px.reset_all();
	pid_py.reset_all();
	pid_pz.reset_all();
	pid_vx.reset_all();
	pid_vy.reset_all();
	pid_vz.reset_all();

	reset_pid_config();
}


void PosController::reset_pid_config()
{
	pid_x.set_pid(pid_x_defaults);
	pid_y.set_pid(pid_y_defaults);
	pid_z.set_pid(pid_z_defaults);
	pid_yaw.set_pid(pid_yaw_defaults);
	pid_px.set_pid(pid_px_defaults);
	pid_py.set_pid(pid_py_defaults);
	pid_pz.set_pid(pid_pz_defaults);
	pid_vx.set_pid(pid_vx_defaults);
	pid_vy.set_pid(pid_vy_defaults);
	pid_vz.set_pid(pid_vz_defaults);
}

TUNE_ID_t PosController::get_autotuneID()
{
	TUNE_CFG_PARAM_t tuneCfgParam;
	tuneCfgParam.acterType = NEGATIVE_NATION;
	tuneCfgParam.ampStdDeviation = 0.1f;

	tuneCfgParam.cTrlType = CONTROLER_TYPE_PID;
	tuneCfgParam.cycleStdDeviation = 100.0f;
	tuneCfgParam.hysteresisNum = 0;
	tuneCfgParam.maxOutputStep = 0.1f;
	tuneCfgParam.minOutputStep = -0.1f;
	tuneCfgParam.setpoint = 0.0;
	// TUNE_ID_t tune_id;

	// tune_id = TUNE_New(&tuneCfgParam);
	return TUNE_New(&tuneCfgParam);
}

// 在一个固定时长进行循环的函数中调用函数TUNE_Work
float PosController::autotuneWORKCycle(float feedbackVal, TUNE_ID_t tune_id, bool &result, uint32_t delayMsec)
{

	// float feedbackVal;
	// uint32_t delayMsec = delayMsec;
	// std::chrono::nanoseconds delayDuration = std::chrono::milliseconds(delayMsec);
	// static std::chrono::nanoseconds delayDuration = std::chrono::milliseconds(100);
	static TUNE_STAT_t tuneStat = TUNE_INIT; /*整定状态*/

	// while(1)
	// {
	// rclcpp::sleep_for(delayDuration); // 循环间隔时间

	// feedbackVal = GetFeedBackVal();//获取实时反馈值
	if (tuneStat != TUNE_SUCESS || tuneStat != TUNE_FAIL)
	{
		result = false;
		float outputVal;
		tuneStat = TUNE_Work(tune_id, feedbackVal, &outputVal, delayMsec);
		// PWM_SetDuty(PWM_CH[k],outputVal);//输出输出值，控制响应单元执行
		return outputVal;
	}
	else
	{
		result = true;
		// PWM_SetDuty(PWM_CH[k],0.0f);
		// 此处已计算出PID值，将其更新到PID参数中
		float paramP, paramI, paramD;
		TUNE_GedPID(tune_id, &paramP, &paramI, &paramD);
		// RCLCPP_INFO(node->get_logger(), "id:%d,paramP%f, paramI%f, paramD%f\n\n\n\n\n", tune_id, paramP, paramI, paramD);
		// PID_Release(tune_id);//释放PID资源
		return 0;
	}
	// }
}

// PID自整定 x/y/z/yaw:是否自整定对应的轴 tune_x/y/z/yaw默认为true 
// delayMsec:自整定周期
// 返回值：是否自整定完成 false:未完成 true:完成
bool PosController::auto_tune(Vector4f pos_now, Vector4f pos_target, uint32_t delayMsec, bool tune_x, bool tune_y, bool tune_z, bool tune_yaw)
{
	static bool first = true, result_x = false, result_y = false, result_z = false, result_yaw = false;
	static TUNE_ID_t id_x, id_y, id_z, id_yaw;
	if (first)
	{
		result_x = !tune_x;
		result_y = !tune_y;
		result_z = !tune_z;
		result_yaw = !tune_yaw;
		PID::Defaults pid_defaults = {1.0, 0.0, 0.0};
		!result_x?set_pid(pid_x, pid_defaults):set_pid(pid_x, pid_x_defaults);
		!result_y?set_pid(pid_y, pid_defaults):set_pid(pid_y, pid_y_defaults);
		!result_z?set_pid(pid_z, pid_defaults):set_pid(pid_z, pid_z_defaults);
		!result_yaw?set_pid(pid_yaw, pid_defaults):set_pid(pid_yaw, pid_yaw_defaults);
		TUNE_Init();
		!result_x?id_x = get_autotuneID():id_x = 0;
		!result_y?id_y = get_autotuneID():id_y = 0;
		!result_z?id_z = get_autotuneID():id_z = 0;
		!result_yaw?id_yaw = get_autotuneID():id_yaw = 0;
		printf("id_x:%d,id_y:%d,id_z:%d,id_yaw:%d\n", id_x, id_y, id_z, id_yaw);
		// send_velocity_command((pos_target - pos_now));
		first = false;
	}
	Vector4f feedbackVal = input_pos_xyz_yaw_without_vel(pos_now, pos_target);
	Vector4f outputVel = {
			!result_x?autotuneWORKCycle(feedbackVal.x(), id_x, result_x, delayMsec):feedbackVal.x(),
			!result_y?autotuneWORKCycle(feedbackVal.y(), id_y, result_y, delayMsec):feedbackVal.y(),
			!result_z?autotuneWORKCycle(feedbackVal.z(), id_z, result_z, delayMsec):feedbackVal.z(),
			!result_yaw?autotuneWORKCycle(feedbackVal.w(), id_yaw, result_yaw, delayMsec):feedbackVal.w()
		};
	// RCLCPP_INFO(node->get_logger(), "vx=%f,vy=%f,vz=%f,yaw=%f", outputVel.x(), outputVel.y(), outputVel.z(), outputVel.w());
	pos_publisher->send_velocity_command(outputVel);
	if (result_x && result_y && result_z && result_yaw)
	{
		reset_pid();
		TUNE_Release(id_x);
		TUNE_Release(id_y);
		TUNE_Release(id_z);
		TUNE_Release(id_yaw);
		result_x = false;
		result_y = false;
		result_z = false;
		result_yaw = false;
		first = true;
		return true;
	}
	return false;
}
