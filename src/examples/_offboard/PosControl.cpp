#include "PosControl.h"
#include "InertialNav.h"
#include "math.h"
#include "PID.h"
#include "FuzzyPID.h"


// #define PID_P

void PosControl::publish_setpoint_raw(Vector4f p, Vector4f v)
{
	// RCLCPP_INFO(node->get_logger(), "Publishing setpoint: latitude=%f, longitude=%f, altitude=%f, yaw=%f", latitude, longitude, altitude, yaw);
	mavros_msgs::msg::PositionTarget msg;
	msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_BODY_NED;
	// mavros_msgs::msg::GlobalPositionTarget::FRAME_GLOBAL_INT;
	msg.position.x = p.x();
	msg.position.y = p.y();
	msg.position.z = p.z();
	msg.yaw = p.w();
	msg.velocity.x = v.x();
	msg.velocity.y = v.y();
	msg.velocity.z = v.z();
	msg.yaw_rate = v.w();
	// msg.acceleration_or_force.x() = 0;
	// msg.acceleration_or_force.y = 0;
	// msg.acceleration_or_force.z() = 0;
	msg.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_AFX |
									mavros_msgs::msg::PositionTarget::IGNORE_AFY |
									mavros_msgs::msg::PositionTarget::IGNORE_AFZ; //|
																																// mavros_msgs::msg::PositionTarget::IGNORE_PX |
																																// mavros_msgs::msg::PositionTarget::IGNORE_PY |
																																// mavros_msgs::msg::PositionTarget::IGNORE_PZ |
																																// mavros_msgs::msg::PositionTarget::IGNORE_VX |
																																// mavros_msgs::msg::PositionTarget::IGNORE_VY |
																																// mavros_msgs::msg::PositionTarget::IGNORE_VZ |
																																// mavros_msgs::msg::PositionTarget::IGNORE_YAW |
																																// mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
	// 设置type_mask以忽略加速度值，只使用位置、速度和偏航
	// 注意：根据需要调整IGNORE_PX, IGNORE_PY, IGNORE_PZ, IGNORE_VX, IGNORE_VY, IGNORE_VZ, IGNORE_YAW, IGNORE_YAW_RATE的值

	msg.header.stamp = node->now();
	msg.header.frame_id = "";
	setpoint_raw_local_publisher_->publish(msg);
}

// 发布全局位置控制指令
// publish_setpoint_raw_global(latitude, longitude, altitude);
// ros2 topic pub /mavros/setpoint_raw/global mavros_msgs/msg/GlobalPositionTarget '{header: "auto", pose: {position: {x: 1.0, y: 2.0, z: 3.0}}}'
// ros2 topic pub /mavros/setpoint_raw/global mavros_msgs/msg/GlobalPositionTarget '{header: "auto", latitude: 1.0, longitude: 2.0, altitude: 3.0}'
//
// publishing : mavros_msgs.msg.GlobalPositionTarget(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1713969082, nanosec=321245648), frame_id=''), coordinate_frame=0, type_mask=0, latitude=1.0, longitude=2.0, altitude=3.0, velocity=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), acceleration_or_force=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), yaw=0.0, yaw_rate=0.0)
// ros2 topic pub /mavros/setpoint_raw/global mavros_msgs/msg/GlobalPositionTarget '{header: "auto", latitude: -35.363262, longitude: 149.165237, altitude: 700.788637}'
void PosControl::publish_setpoint_raw_global(double latitude, double longitude, double altitude, double yaw)
{
	RCLCPP_INFO(node->get_logger(), "Publishing setpoint: latitude=%f, longitude=%f, altitude=%f, yaw=%f", latitude, longitude, altitude, yaw);
	mavros_msgs::msg::GlobalPositionTarget msg;
	msg.coordinate_frame = mavros_msgs::msg::GlobalPositionTarget::FRAME_GLOBAL_INT;
	// mavros_msgs::msg::GlobalPositionTarget::FRAME_GLOBAL_INT;
	msg.latitude = latitude;
	msg.longitude = longitude;
	msg.altitude = altitude;
	msg.yaw = yaw;
	msg.velocity.x = 0;
	msg.velocity.y = 0;
	msg.velocity.z = 0;(*objectname) [1ms]

	msg.acceleration_or_force.z = 0;

	msg.header.stamp = node->now();
	msg.header.frame_id = "";
	setpoint_raw_global_publisher_->publish(msg);
}

// 发布本地位置控制指令
// send_local_setpoint_command(x, y, z, yaw); 飞行到
// 飞行到相对于世界坐标系的(x,y,z)位置
// ros2 topic pub /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, pose: {position: {x: 0.0, y: 0.0, z: 5.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}'
void PosControl::send_local_setpoint_command(double x, double y, double z, double yaw)
{
	geometry_msgs::msg::PoseStamped msg;
	//时间戳等header信息
	msg.header.stamp = node->now();
	msg.header.frame_id = "base_link";
	// 坐标位置 单位为m
	msg.pose.position.x = x;
	msg.pose.position.y = y;
	msg.pose.position.z = z;
	//旋转四元数
	
	double radians_angle = yaw;
	msg.pose.orientation.x = 0;
	msg.pose.orientation.y = 0;
	msg.pose.orientation.z = sin(radians_angle / 2);
	msg.pose.orientation.w = cos(radians_angle / 2);
	RCLCPP_INFO(node->get_logger(), "Publishing local setpoint: x=%f, y=%f, z=%f, yaw=%f", x, y, z, yaw);
	local_setpoint_publisher_->publish(msg);
}

// 发布本地位置控制指令
// send_local_setpoint_command(x, y, z, yaw); 飞行到	
// 飞行到相对于世界坐标系的(x,y,z)位置
bool PosControl::local_setpoint_command(Vector4f now, Vector4f target, double accuracy)
{
	static bool first = true;
	if (first){
		// RCLCPP_INFO(node->get_logger(), "local_setpoint_command");
		send_local_setpoint_command(target.x(), target.y(), target.z(), target.w());
		first = false;
	}
	if (abs(now.x() - target.x()) <= accuracy &&
			abs(now.y() - target.y()) <= accuracy &&
			abs(now.z() - target.z()) <= accuracy
			// #ifndef PID_P
			// && abs(pos_now.w() - _pos_target.w())<=yaw_accuracy
			// #endif
	)
	{
		RCLCPP_INFO(node->get_logger(), "at_check_point");
		first = true;
		return true;
	}
	return false;
}

// 发布速度控制指令
// linear_x, linear_y, linear_z 为速度(m/s), angular_z为角速度(度/s)
//
// ros2 topic pub /mavros/setpoint_velocity/cmd_vel geometry_msgs/msg/TwistStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}'
void PosControl::send_velocity_command_world(double linear_x, double linear_y, double linear_z, double angular_z)
{
	geometry_msgs::msg::TwistStamped msg;
	msg.twist.linear.x = linear_x;
	msg.twist.linear.y = linear_y;
	msg.twist.linear.z = linear_z;
	msg.twist.angular.z = angular_z;
	msg.header.stamp = node->now();
	msg.header.frame_id = "base_link";
	twist_stamped_publisher_->publish(msg);
}
void PosControl::send_velocity_command_world(Vector4f v)
{
	geometry_msgs::msg::TwistStamped msg;
	msg.twist.linear.x = v.x();
	msg.twist.linear.y = v.y();
	msg.twist.linear.z = v.z();
	msg.twist.angular.z = v.w();
	msg.header.stamp = node->now();
	msg.header.frame_id = "base_link";
	twist_stamped_publisher_->publish(msg);
}

void PosControl::send_velocity_command(Vector4f v)
{
	geometry_msgs::msg::TwistStamped msg;
	msg.twist.linear.x = v.x();
	msg.twist.linear.y = v.y();
	msg.twist.linear.z = v.z();
	msg.twist.angular.z = v.w();
	msg.header.stamp = node->now();
	msg.header.frame_id = "base_link";
	twist_stamped_publisher_->publish(msg);
}

// 发布定时速度控制指令 v(m/s) time(s)
// linear_x, linear_y, linear_z 为速度(m/s), angular_z为角速度(度/s)
// send_velocity_command_with_time(3, 0, 0, 0, 3);//前进3m/s,3s后停止
bool PosControl::send_velocity_command_with_time(Vector4f v, double time)
{
	static bool first = true;
	static double find_start;
	if (first)
	{
		find_start = node->get_clock()->now().nanoseconds() / 1000;
		first = false;
	}
	geometry_msgs::msg::TwistStamped msg;
	if ((node->get_clock()->now().nanoseconds() / 1000 - find_start) > 1000000 * time)
	{
		send_velocity_command_world(0, 0, 0, 0);
		first = true;
		return true;
	}
	else
	{
		send_velocity_command(v);
		return false;
	}
}
void PosControl::send_accel_command(Vector4f v)
{
	// (void)v;
	geometry_msgs::msg::Vector3Stamped msg;
	msg.vector.x = v.x();
	msg.vector.y = v.y();
	msg.vector.z = v.z();
	msg.header.stamp = node->now();
	setpoint_accel_publisher_->publish(msg);

	// mavros_msgs::msg::AttitudeTarget msg;
	// msg.header.stamp = node->now();
	// msg.header.frame_id = "base_link";
	// msg.body_rate.x() ()= -v.y();
	// msg.body_rate.y = v.x();
	// msg.body_rate.z() = v.z();
	// msg.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_THRUST;
	// msg.thrust = 1;
	// msg.orientation.w =
	// setpoint_raw_attitude_publisher_->publish(msg);

	// 	 geometry_msgs::msg::TwistStamped msg1;
	// 	msg1.twist.angular.x() = v.x();
	// 	msg1.twist.angular.y() = v.y();
	// // msg1.twist.linear.z()
	// 	msg1.header.stamp = node->now();
	// 	msg1.header.frame_id = "base_link";
	// 	twist_stamped_publisher_->publish(msg1);

	// RCLCPP_INFO(node->get_logger(), "Publishing setpoint: latitude=%f, longitude=%f, altitude=%f, yaw=%f", latitude, longitude, altitude, yaw);
	// mavros_msgs::msg::PositionTarget msg;
	// msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_BODY_NED;
	// //mavros_msgs::msg::GlobalPositionTarget::FRAME_GLOBAL_INT;
	// msg.acceleration_or_force.x() = v.x();
	// msg.acceleration_or_force.y() = v.y();
	// msg.acceleration_or_force.z() = v.z();
	// msg.type_mask = // mavros_msgs::msg::PositionTarget::IGNORE_AFX |
	//                 // mavros_msgs::msg::PositionTarget::IGNORE_AFY |
	//                 // mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
	//                 mavros_msgs::msg::PositionTarget::IGNORE_PX |
	//                 mavros_msgs::msg::PositionTarget::IGNORE_PY |
	//                 mavros_msgs::msg::PositionTarget::IGNORE_PZ |
	//                 mavros_msgs::msg::PositionTarget::IGNORE_VX |
	//                 mavros_msgs::msg::PositionTarget::IGNORE_VY |
	//                 mavros_msgs::msg::PositionTarget::IGNORE_VZ |
	//                 mavros_msgs::msg::PositionTarget::IGNORE_YAW |
	//                 mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
	// msg.header.stamp = node->now();
	// msg.header.frame_id = "";
	// setpoint_raw_local_publisher_->publish(msg);
}
// void PosControl::setYawRate(float yaw_rate) {
//         // 创建并填充TwistStamped消息
//         geometry_msgs::msg::TwistStamped twist_msg;
//         twist_msg.header.stamp = node->now();
//         twist_msg.twist.angular.z() = yaw_rate; // 设置偏航角速度，正值为顺时针，负值为逆时针

//         // 发布消息
//         twist_stamped_publisher_->publish(twist_msg);
// }

// 输入位置 pid控制速度
// 初始时 now > target : direction = true 记为正方向 否则记为负方向
Vector3f PosControl::input_pos_xyz(Vector3f now, Vector3f target, bool fuzzy, Vector3b direction)
{
	if(fuzzy){
		float delta_k = 2.0f;

		Vector3f f;
		float kp = 0, ki = 0, kd = 0;

        pid_x.get_pid(kp, ki, kd);
        if (direction.x())
            fuzzy_pid.fuzzy_pid_control(now.x(), target.x(), 0, kp, ki, kd, delta_k);
        else
            fuzzy_pid.fuzzy_pid_control(target.x(), now.x(), 0, kp, ki, kd, delta_k);
        pid_x.set_gains(kp, ki, kd);
		f.x() = pid_x.update_all(now.x(), target.x(), dt, max_speed_xy, InertialNav::velocity.x());

		pid_y.get_pid(kp, ki, kd);
		if (direction.y())
			fuzzy_pid.fuzzy_pid_control(now.y(), target.y(), 1, kp, ki, kd, delta_k);
		else
			fuzzy_pid.fuzzy_pid_control(target.y(), now.y(), 1, kp, ki, kd, delta_k);
		pid_y.set_gains(kp, ki, kd);
		f.y() = pid_y.update_all(now.y(), target.y(), dt, max_speed_xy, InertialNav::velocity.y());
		
		pid_z.get_pid(kp, ki, kd);
		if (direction.z())
			fuzzy_pid.fuzzy_pid_control(now.z(), target.z(), 2, kp, ki, kd, delta_k);
		else
			fuzzy_pid.fuzzy_pid_control(target.z(), now.z(), 2, kp, ki, kd, delta_k);
		pid_z.set_gains(kp, ki, kd);
		f.z() = pid_z.update_all(now.z(), target.z(), dt, max_speed_z, InertialNav::velocity.z());
		return f;
	}
	else{
		Vector3f f;
		f.x() = pid_x.update_all(now.x(), target.x(), dt, max_speed_xy, InertialNav::velocity.x());
		f.y() = pid_y.update_all(now.y(), target.y(), dt, max_speed_xy, InertialNav::velocity.y());
		f.z() = pid_z.update_all(now.z(), target.z(), dt, max_speed_z, InertialNav::velocity.z());
		return f;
	}
}
// 输入位置 pid控制速度
Vector4f PosControl::input_pos_xyz_yaw(Vector4f now, Vector4f target, bool fuzzy, Vector4b direction)
{
	if(fuzzy){
		float delta_k = 2.0f;

		Vector4f f;
		float kp = 0, ki = 0, kd = 0;

        pid_x.get_pid(kp, ki, kd);
        if (direction.x())
            fuzzy_pid.fuzzy_pid_control(now.x(), target.x(), 0, kp, ki, kd, delta_k);
        else
            fuzzy_pid.fuzzy_pid_control(target.x(), now.x(), 0, kp, ki, kd, delta_k);
        pid_x.set_gains(kp, ki, kd);
		f.x() = pid_x.update_all(now.x(), target.x(), dt, max_speed_xy, InertialNav::velocity.x());

		pid_y.get_pid(kp, ki, kd);
		if (direction.y())
			fuzzy_pid.fuzzy_pid_control(now.y(), target.y(), 1, kp, ki, kd, delta_k);
		else
			fuzzy_pid.fuzzy_pid_control(target.y(), now.y(), 1, kp, ki, kd, delta_k);
		pid_y.set_gains(kp, ki, kd);
		f.y() = pid_y.update_all(now.y(), target.y(), dt, max_speed_xy, InertialNav::velocity.y());
		
		pid_z.get_pid(kp, ki, kd);
		if (direction.z())
			fuzzy_pid.fuzzy_pid_control(now.z(), target.z(), 2, kp, ki, kd, delta_k);
		else
			fuzzy_pid.fuzzy_pid_control(target.z(), now.z(), 2, kp, ki, kd, delta_k);
		pid_z.set_gains(kp, ki, kd);
		f.z() = pid_z.update_all(now.z(), target.z(), dt, max_speed_z, InertialNav::velocity.z());
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
		if (target.w() > M_PI)
			target.w() -= 2 * M_PI;
		else if (target.w() < -M_PI)
			target.w() += 2 * M_PI;
		pid_yaw.get_pid(kp, ki, kd);
		if (direction.w())
			fuzzy_pid.fuzzy_pid_control(now.w(), target.w(), 3, kp, ki, kd, delta_k);
		else
			fuzzy_pid.fuzzy_pid_control(target.w(), now.w(), 3, kp, ki, kd, delta_k);
		pid_yaw.set_gains(kp, ki, kd);
		f.w() = pid_yaw.update_all(now.w(), target.w(), dt, max_speed_yaw, InertialNav::velocity.w());
		RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: x:%f y:%f z:%f yaw:%f", f.x(), f.y(), f.z(), f.w());
		return f;

	} else{
		Vector4f f;
		f.x() = pid_x.update_all(now.x(), target.x(), dt, max_speed_xy, InertialNav::velocity.x());
		f.y() = pid_y.update_all(now.y(), target.y(), dt, max_speed_xy, InertialNav::velocity.y());
		f.z() = pid_z.update_all(now.z(), target.z(), dt, max_speed_z, InertialNav::velocity.z());
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
		if (target.w() > M_PI)
			target.w() -= 2 * M_PI;
		else if (target.w() < -M_PI)
			target.w() += 2 * M_PI;
		f.w() = pid_yaw.update_all(now.w(), target.w(), dt, max_speed_yaw, InertialNav::velocity.w());
		RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: x:%f y:%f z:%f yaw:%f", f.x(), f.y(), f.z(), f.w());
		return f;
	}
}
// 输入位置 pid控制速度
Vector4f PosControl::input_pos_xyz_yaw_without_vel(Vector4f now, Vector4f target)
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
	if (target.w() > M_PI)
		target.w() -= 2 * M_PI;
	else if (target.w() < -M_PI)
		target.w() += 2 * M_PI;
	f.w() = pid_yaw.update_all(now.w(), target.w(), dt, max_speed_yaw, InertialNav::velocity.w());
	RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: x:%f y:%f z:%f", f.x(), f.y(), f.z());

	return f;
}

// 输入位置 bushi串级pid控制速度
Vector4f PosControl::input_pos_vel_1_xyz_yaw(Vector4f now, Vector4f target)
{
	Vector4f f;
	static Vector4f v_v = {0, 0, 0, 0};
	// RCLCPP_INFO(node->get_logger(),"acc: %f,%f,%f",InertialNav::linear_acceleration.x(),InertialNav::linear_acceleration.y,InertialNav::linear_acceleration.z());
	f.x() = pid_vx.update_all(InertialNav::velocity.x(), pid_px.update_all(now.x(), target.x(), 0, 2 * max_speed_xy, InertialNav::velocity.x()), dt_pid_p_v, max_speed_xy); //,InertialNav::linear_acceleration.x());
	f.y() = pid_vy.update_all(InertialNav::velocity.y(), pid_py.update_all(now.y(), target.y(), 0, 2 * max_speed_xy, InertialNav::velocity.y()), dt_pid_p_v, max_speed_xy); //,InertialNav::linear_acceleration.y);
	f.z() = pid_vz.update_all(InertialNav::velocity.z(), pid_pz.update_all(now.z(), target.z(), 0, 2 * max_speed_z, InertialNav::velocity.z()), dt_pid_p_v, max_speed_z);		//,InertialNav::linear_acceleration.z()-9.80665);
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
	f.w() = pid_yaw.update_all(now.w(), target.w(), dt, max_speed_yaw, InertialNav::velocity.w());
	v_v = InertialNav::velocity;
	RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: x:%f y:%f z:%f", f.x(), f.y(), f.z());

	return f;
}
// 输入位置 串级pid控制速度
Vector4f PosControl::input_pos_vel_xyz_yaw(Vector4f now, Vector4f target)
{
	Vector4f f;
	static Vector4f v_v = {0, 0, 0, 0};
	// RCLCPP_INFO(node->get_logger(),"acc: %f,%f,%f",InertialNav::linear_acceleration.x(),InertialNav::linear_acceleration.y,InertialNav::linear_acceleration.z());
	f.x() = pid_vx.update_all(InertialNav::velocity.x(), pid_px.update_all(now.x(), target.x(), 0, max_speed_xy, InertialNav::velocity.x()), dt_pid_p_v, max_accel_xy); //,InertialNav::linear_acceleration.x());
	f.y() = pid_vy.update_all(InertialNav::velocity.y(), pid_py.update_all(now.y(), target.y(), 0, max_speed_xy, InertialNav::velocity.y()), dt_pid_p_v, max_accel_xy); //,InertialNav::linear_acceleration.y);
	f.z() = pid_vz.update_all(InertialNav::velocity.z(), pid_pz.update_all(now.z(), target.z(), 0, max_speed_z, InertialNav::velocity.z()), dt_pid_p_v, max_accel_z);		//,InertialNav::linear_acceleration.z()-9.80665);
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
	f.w() = pid_yaw.update_all(now.w(), target.w(), dt, max_speed_yaw, InertialNav::velocity.w());
	v_v = InertialNav::velocity;
	RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: x:%f y:%f z:%f", f.x(), f.y(), f.z());

	return f;
}
// 发布 publish_setpoint_raw 飞行到指定位置（相对于当前位置）
bool PosControl::publish_setpoint_world(Vector4f now, Vector4f target, double accuracy, double yaw_accuracy)
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
	// 	pid_px.update_all(now.x(),target.x(),0,2*max_speed_xy,InertialNav::velocity.x()),
	// 	pid_py.update_all(now.y,target.y,0,2*max_speed_xy,InertialNav::velocity.y),
	// 	pid_pz.update_all(now.z(),target.z(),0,2*max_speed_z,InertialNav::velocity.z()),
	// 	pid_yaw.update_all(now.w(),target.w(),dt,max_speed_yaw,InertialNav::velocity.w())
	// };
	// Vector4f vel = {
	// 	pid_vx.update_all(InertialNav::velocity.x(),pos.x(),dt_pid_p_v,max_speed_xy),//,InertialNav::linear_acceleration.x());
	// 	pid_vy.update_all(InertialNav::velocity.y,pos.y,dt_pid_p_v,max_speed_xy),//,InertialNav::linear_acceleration.y);
	// 	pid_vz.update_all(InertialNav::velocity.z(),pos.z(),dt_pid_p_v,max_speed_z),//,InertialNav::linear_acceleration.z()-9.80665);
	// 	0
	// };(*objectname) [1ms]
pid_py.update_all(now.y(), target.y(), 0, 2 * max_speed_xy, InertialNav::velocity.y()),
			pid_pz.update_all(now.z(), target.z(), 0, 2 * max_speed_z, InertialNav::velocity.z()),
			pid_yaw.update_all(now.w(), target.w(), dt, max_speed_yaw, InertialNav::velocity.w())};
	Vector4f vel;
	publish_setpoint_raw(pos, vel);
	if (abs(now.x() - target.x()) <= accuracy &&
			abs(now.y() - target.y()) <= accuracy &&
			abs(now.z() - target.z()) <= accuracy
			// #ifndef PID_P
			// && abs(pos_now.w() - _pos_target.w())<=yaw_accuracy
			// #endif
	)
	{
		RCLCPP_INFO(node->get_logger(), "at_check_point");
		first = true;
		return true;
	}
	return false;
}
// pid飞行到指定位置（相对于当前位置）
// x:前后方向位置(m) y:左右方向位置(m) z:高度(m) yaw:偏航角(°) accuracy=DEFAULT_ACCURACY:精度(m)
bool PosControl::trajectory_setpoint(Vector4f pos_now, Vector4f pos_target, double accuracy, double yaw_accuracy)
{
	(void)yaw_accuracy;
	static bool first = true;
	static Vector4f pos_target_temp;
	static Vector4b direction;
	if (first)
	{
		// pos_target.rotate_xy(default_yaw);
		_pos_target = pos_now + pos_target;
		pos_target_temp = pos_target;
		RCLCPP_INFO(node->get_logger(), "trajectory_setpoint: x:%f y:%f", _pos_target.x(), _pos_target.y());
		reset_pid();
		// pid_x.set_pid_info();
		// pid_y.set_pid_info();
		// pid_z.set_pid_info();
		// pid_yaw.set_pid_info();
		
		// direction = {pos_now.x() < _pos_target.x(), pos_now.y() < _pos_target.y(), pos_now.z() < _pos_target.z(), pos_now.w() < _pos_target.w()};
		// Update the assignment
		direction << (pos_now.x() < _pos_target.x()),
								 (pos_now.y() < _pos_target.y()),
								 (pos_now.z() < _pos_target.z()),
								 (pos_now.w() < _pos_target.w());

		first = false;
	}
	// if(pose_.header.stamp.sec == 0){
	// 	RCLCPP_INFO(this->get_logger(), "No pose data received yet");
	// 	return;
	// }
	if (is_equal(pos_target, pos_target_temp, 0.1f))
	{
#ifdef PID_P
		// send_velocity_command_world(input_pos_vel_1_xyz_yaw(pos_now,_pos_target));
		send_accel_command(input_pos_vel_xyz_yaw(pos_now, _pos_target));
#endif
#ifndef PID_P
		// send_velocity_command_world(input_pos_xyz_yaw(pos_now, _pos_target));
		send_velocity_command_world(input_pos_xyz_yaw(pos_now, _pos_target, true, direction));
#endif
	}
	else
	{
		printf("pos_target_temp:%f,%f,%f,%f\n", pos_target_temp.x(), pos_target_temp.y(), pos_target_temp.z(), pos_target_temp.w());
		printf("pos_target:%f,%f,%f,%f\n", pos_target.x(), pos_target.y(), pos_target.z(), pos_target.w());
		RCLCPP_INFO(node->get_logger(), "change point");
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
		RCLCPP_INFO(node->get_logger(), "at_check_point");
		first = true;
		return true;
	}
	return false;
}
// pid飞行到指定位置（相对于起飞点/世界坐标系）
// x:前后方向位置(m) y:左右方向位置(m) z:高度(m) yaw:偏航角(°) accuracy=DEFAULT_ACCURACY:精度(m)
bool PosControl::trajectory_setpoint_world(Vector4f pos_now, Vector4f pos_target, double accuracy, double yaw_accuracy)
{
	(void)yaw_accuracy;
	static bool first = true;
	static Vector4f pos_target_temp;
	static Vector4b direction;
	if (first)
	{
		// pos_target.rotate_xy(default_yaw);
		_pos_target = pos_target;
		pos_target_temp = pos_target;
		RCLCPP_INFO(node->get_logger(), "trajectory_setpoint: x:%f y:%f z:%f", _pos_target.x(), _pos_target.y(), _pos_target.z());
		reset_pid();
		// pid_x.set_pid_info();
		// pid_y.set_pid_info();
		// pid_z.set_pid_info();
		// pid_yaw.set_pid_info();
		direction = Vector4b(
			pos_now.x() < _pos_target.x(),
			pos_now.y() < _pos_target.y(),
			pos_now.z() < _pos_target.z(),
			pos_now.w() < _pos_target.w()
		);
		first = false;
	}
	// if(pose_.header.stamp.sec == 0){
	// 	RCLCPP_INFO(this->get_logger(), "No pose data received yet");
	// 	return;
	// }
	if (is_equal(pos_target, pos_target_temp, 0.1f))
	{
#ifdef PID_P
		// send_velocity_command_world(input_pos_vel_1_xyz_yaw(pos_now,_pos_target));
		send_accel_command(input_pos_vel_xyz_yaw(pos_now, _pos_target));
#endif
#ifndef PID_P
		// send_velocity_command_world(input_pos_xyz_yaw(pos_now, _pos_target));
		send_velocity_command_world(input_pos_xyz_yaw(pos_now, _pos_target, true, direction));
#endif
	}
	else
	{
		RCLCPP_INFO(node->get_logger(), "change point");
		first = true;
	}
	if (abs(pos_now.x() - _pos_target.x()) <= accuracy &&
			abs(pos_now.y() - _pos_target.y()) <= accuracy &&
			abs(pos_now.z() - _pos_target.z()) <= accuracy
			// && abs(pos_now.w() - _pos_target.w())<=yaw_accuracy
	)
	{
		RCLCPP_INFO(node->get_logger(), "at_check_point");
		first = true;
		return true;
	}
	return false;
}
// pid飞行到指定位置(闭环)，指定pid参数（相对于起飞点/世界坐标系）
bool PosControl::trajectory_setpoint_world(Vector4f pos_now, Vector4f pos_target, PID::Defaults defaults, double accuracy, double yaw_accuracy)
{
	(void)yaw_accuracy;
	RCLCPP_INFO(node->get_logger(), "p:%f i:%f d:%f", defaults.p, defaults.i, defaults.d);
	static bool first = true;
	static Vector4f pos_target_temp;
	if (first)
	{
		// pos_target.rotate_xy(default_yaw);
		// _pos_target = pos_target;
		pos_target_temp = pos_target;
		RCLCPP_INFO(node->get_logger(), "trajectory_setpoint: x:%f y:%f", pos_target.x(), pos_target.y());
		// 设置pid参数
		set_pid(pid_x, defaults);
		set_pid(pid_y, defaults);
		// pid_x.set_pid_info();
		// pid_y.set_pid_info();
		// pid_z.set_pid_info();
		// pid_yaw.set_pid_info();
		first = false;
	}
	// if(pose_.header.stamp.sec == 0){
	// 	RCLCPP_INFO(this->get_logger(), "No pose data received yet");
	// 	return;
	// }
	if (is_equal(pos_target, pos_target_temp, 0.1f))
	{
		// 运行时更新pid参数
		set_pid(pid_x, defaults);
		set_pid(pid_y, defaults);

		// #ifdef PID_P
		// // send_velocity_command_world(input_pos_vel_1_xyz_yaw(pos_now,_pos_target));
		// send_accel_command(input_pos_vel_xyz_yaw(pos_now,_pos_target));
		// #endif
		// #ifndef PID_P
		send_velocity_command_world(input_pos_xyz_yaw_without_vel(pos_now, pos_target));
		// #endif
	}
	else
	{
		RCLCPP_INFO(node->get_logger(), "change point");
		first = true;
	}
	if (abs(pos_now.x() - pos_target.x()) <= accuracy &&
			abs(pos_now.y() - pos_target.y()) <= accuracy &&
			abs(pos_now.z() - pos_target.z()) <= accuracy
			// && abs(pos_now.w() - _pos_target.w())<=yaw_accuracy
	)
	{
		RCLCPP_INFO(node->get_logger(), "at_check_point");
		// 重设为默认pid参数
		set_pid(pid_px, pid_px_defaults);
		set_pid(pid_py, pid_py_defaults);
		reset_limits();
		first = true;
		return true;
	}
	return false;
}

bool PosControl::trajectory_circle(float a, float b, float height, float dt, float default_yaw, float yaw)
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
	publish_setpoint_raw(p, v);

	RCLCPP_INFO(node->get_logger(), "trajectory_circle: t_n:%f t_t:%f", t, M_PI + default_yaw + yaw);
	// if (t > M_PI + default_yaw + yaw){
	if (false)
	{
		first = true;
		return true;
		send_velocity_command_world(0, 0, 0, 0);
	}
	else
	{
		return false;
	}
}
// 航点设置，s形速度规划？(开环)，飞行经过指定位置（相对于起飞点/世界坐标系）
bool PosControl::trajectory_generator_world(double speed_factor, std::array<double, 3> q_goal, Vector3f max_speed, Vector3f max_accel)
{
	static RobotState current_state;
	static bool first = true;
	static uint64_t count = 0;
	bool isFinished = false;
	static Vector4f pos_start_temp;
	static Vector4f pos_target_temp;
	static Vector4b direction;

	if (first)
	{
		reset_pid();
		std::cout << InertialNav::position.x() << " " << InertialNav::position.y() << " " << InertialNav::position.z() << " " << std::endl;
		// current_state.q_d = {InertialNav::position.x(), InertialNav::position.y(), InertialNav::position.z()};
		current_state.q_d = {InertialNav::position.x(), InertialNav::position.y(), InertialNav::position.z()};
		_trajectory_generator = std::make_unique<TrajectoryGenerator>(speed_factor, q_goal); // Use smart pointer for memory management
		_trajectory_generator->set_dq_max({MIN(max_speed_xy, max_speed.x()), MIN(max_speed_xy, max_speed.y()), MIN(max_speed_z, max_speed.z())});
		_trajectory_generator->set_dqq_max({MIN(max_accel_xy, max_accel.x()), MIN(max_accel_xy, max_accel.y()), MIN(max_accel_z, max_accel.z())});
		// pos_start_temp = {InertialNav::position.x(), InertialNav::position.y, InertialNav::position.z(), 0};
		pos_start_temp = Vector4f(InertialNav::position.x(), InertialNav::position.y(), InertialNav::position.z(), 0.0f);
		// pos_target_temp = {static_cast<float>(q_goal[0]), static_cast<float>(q_goal[1]), static_cast<float>(q_goal[2]), 0};
		// pos_target_temp << static_cast<float>(q_goal[0]), static_cast<float>(q_goal[1]), static_cast<float>(q_goal[2]), 0;
		pos_target_temp = Vector4f(static_cast<float>(q_goal[0]), static_cast<float>(q_goal[1]), static_cast<float>(q_goal[2]), 0);
		count = 0;
		isFinished = false;
		// direction = {InertialNav::position.x() < q_goal[0], InertialNav::position.y() < q_goal[1], InertialNav::position.z() < q_goal[2], false};
		direction << (InertialNav::position.x() < q_goal[0]),
                 (InertialNav::position.y() < q_goal[1]),
             		 (InertialNav::position.z() < q_goal[2]),
             		 false;
		first = false; // Ensure that initialization block runs only once
	}
	if (is_equal(Vector4f(q_goal[0], q_goal[1], q_goal[2], 0), pos_target_temp, 0.1f))
	{
		if (!isFinished)
		{
			isFinished = (*_trajectory_generator)(current_state, count / 10.0);
			count++; // Increment count to simulate time passing
		}
		else
		{
			first = true; // Reset first to true to reinitialize the trajectory generator
		}
	}
	else
	{
		RCLCPP_INFO(node->get_logger(), "trajectory_generator: change point");
		first = true;
		return false;
	}
	std::cout << "_trajectory_generator: " << _trajectory_generator->delta_q_d[0] << " "
						<< _trajectory_generator->delta_q_d[1] << " "
						<< _trajectory_generator->delta_q_d[2] << " "
						<< std::endl;
	// trajectory_setpoint_world(
	// 	{InertialNav::position.x(), InertialNav::position.y, InertialNav::position.z(), 0},
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
	send_velocity_command_world(
		input_pos_xyz_yaw(
			Vector4f(InertialNav::position.x() - pos_start_temp.x(), InertialNav::position.y() - pos_start_temp.y(), InertialNav::position.z() - pos_start_temp.z(), 0),
			Vector4f(static_cast<float>(_trajectory_generator->delta_q_d[0]), static_cast<float>(_trajectory_generator->delta_q_d[1]), static_cast<float>(_trajectory_generator->delta_q_d[2]), 0),
			true, direction)
	);
	// send_accel_command(input_pos_vel_xyz_yaw(
	// 		{InertialNav::position.x() - pos_start_temp.x(), InertialNav::position.y - pos_start_temp.y, InertialNav::position.z() - pos_start_temp.z(), 0},
	// 		{static_cast<float>(_trajectory_generator->delta_q_d[0]), static_cast<float>(_trajectory_generator->delta_q_d[1]), static_cast<float>(_trajectory_generator->delta_q_d[2]), 0}));

	return isFinished; // Return the status of trajectory generation
}

float PosControl::get_speed_max()
{
	return max_speed_xy;
}
float PosControl::get_accel_max()
{
	return max_accel_xy;
}
float PosControl::get_decel_max()
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
float PosControl::get_turn_rate_speed_max()
{
	return max_speed_yaw;
}
float PosControl::get_jerk_max()
{
	return max_jerk;
}
// // / set limits
// void PosControl::set_limits(float speed_max, float accel_max, float lat_accel_max, float jerk_max)
// {
//     max_speed_xy = MAX(speed_max, 0);
//     max_accel_xy = MAX(accel_max, 0);
// 	(void)lat_accel_max;
//     // _lat_accel_max = MAX(lat_accel_max, 0);
//     max_jerk = MAX(jerk_max, 0);
// }
// //
// set limits

struct PosControl::Limits_t PosControl::readLimits(const std::string &filename, const std::string &section)
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
		RCLCPP_ERROR(node->get_logger(), "Error reading limits from file: %s", e.what());
	}
	return limits;
}

// 最大值最大限制在宏定义范围内
void PosControl::set_limits(struct Limits_t limits)
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

void PosControl::reset_limits()
{
	// max_speed_xy = POSCONTROL_VEL_XY_MAX;
	// max_speed_z = POSCONTROL_VEL_Z_MAX;
	// max_speed_yaw = POSCONTROL_VEL_YAW_MAX;
	// max_accel_xy = POSCONTROL_ACC_XY_MAX;
	// max_accel_z = POSCONTROL_ACC_Z_MAX;
	set_limits(limit_defaults);
	// max_dccel_xy = POSCONTROL_ACC_XY_MAX;
	// max_jerk = POSCONTROL_JERK_MAX;
	// _p_pos.set_limits(max_speed_xy, MIN(max_accel_xy, max_dccel_xy), max_jerk);
}

void PosControl::set_pid(PID &pid, PID::Defaults defaults)
{
	pid.set_gains(defaults);
}

void PosControl::reset_pid()
{
	pid_x.set_gains(pid_x_defaults);
	pid_y.set_gains(pid_y_defaults);
	pid_z.set_gains(pid_z_defaults);
	pid_yaw.set_gains(pid_yaw_defaults);
	pid_px.set_gains(pid_px_defaults);
	pid_py.set_gains(pid_py_defaults);
	pid_pz.set_gains(pid_pz_defaults);
	pid_vx.set_gains(pid_vx_defaults);
	pid_vy.set_gains(pid_vy_defaults);
	pid_vz.set_gains(pid_vz_defaults);
}

TUNE_ID_t PosControl::get_autotuneID()
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
float PosControl::autotuneWORKCycle(float feedbackVal, TUNE_ID_t tune_id, bool &result, uint32_t delayMsec)
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
		RCLCPP_INFO(node->get_logger(), "id:%d,paramP%f, paramI%f, paramD%f\n\n\n\n\n", tune_id, paramP, paramI, paramD);
		// PID_Release(tune_id);//释放PID资源
		return 0;
	}
	// }
}
// PID自整定 x/y/z/yaw:是否自整定对应的轴 tune_x/y/z/yaw默认为true 
// delayMsec:自整定周期
// 返回值：是否自整定完成 false:未完成 true:完成
bool PosControl::auto_tune(Vector4f pos_now, Vector4f pos_target, uint32_t delayMsec, bool tune_x, bool tune_y, bool tune_z, bool tune_yaw)
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
		// send_velocity_command_world((pos_target - pos_now));
		first = false;
	}
	Vector4f feedbackVal = input_pos_xyz_yaw_without_vel(pos_now, pos_target);
	Vector4f outputVel = {
			!result_x?autotuneWORKCycle(feedbackVal.x(), id_x, result_x, delayMsec):feedbackVal.x(),
			!result_y?autotuneWORKCycle(feedbackVal.y(), id_y, result_y, delayMsec):feedbackVal.y(),
			!result_z?autotuneWORKCycle(feedbackVal.z(), id_z, result_z, delayMsec):feedbackVal.z(),
			!result_yaw?autotuneWORKCycle(feedbackVal.w(), id_yaw, result_yaw, delayMsec):feedbackVal.w()
		};
	RCLCPP_INFO(node->get_logger(), "vx=%f,vy=%f,vz=%f,yaw=%f", outputVel.x(), outputVel.y(), outputVel.z(), outputVel.w());
	send_velocity_command_world(outputVel);
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
// }
// {
// 	static bool first = true, result_x = false, result_y = false, result_z = false, result_yaw = false;
// 	static TUNE_ID_t id_x, id_y;
// 	if (first)
// 	{
// 		result_x =
// 		PID::Defaults pid_defaults = {0.5, 0.0, 0.0};
// 		set_pid(pid_x, pid_defaults);
// 		set_pid(pid_y, pid_defaults);
// 		TUNE_Init();
// 		id_x = get_autotuneID();
// 		id_y = get_autotuneID();
// 		printf("id_x:%d,id_y:%d\n", id_x, id_y);
// 		// send_velocity_command_world((pos_target - pos_now));
// 		first = false;
// 	}
// 	Vector4f feedbackVal = input_pos_xyz_yaw_without_vel(pos_now, pos_target);
// 	Vector4f outputVel = {
// 			autotuneWORKCycle(feedbackVal.x(), id_x, result_x),
// 			autotuneWORKCycle(feedbackVal.y, id_y, result_y),
// 			0,
// 			0};
// 	RCLCPP_INFO(node->get_logger(), "vx=%f,vy=%f", outputVel.x(), outputVel.y);
// 	send_velocity_command_world(outputVel);
// 	if (result_x && result_y)
// 	{
// 		reset_pid();
// 		TUNE_Release(id_x);
// 		TUNE_Release(id_y);
// 		first = true;
// 		return false;
// 	}
// 	return true;
// }

// // 定高悬停//定角度悬停
// // vx=0:前后方向速度(m/s) vy=0:左右方向速度(m/s) z:高度(m) yaw:偏航角(°) time:持续时间(s) accuracy=DEFAULT_ACCURACY:精度(m)
// // 返回值：是否到达规定时间
// bool OffboardControl::alt_hold(double vx,double vy ,double z ,double yaw,double time,double accuracy){
// 	static bool first=true;
// 	static double vx1=0,vy1=0;
// 	if(first){
// 	get_target_location(&vx, &vy);
// 	vx1=vx;
// 	vy1=vy;
// 	RCLCPP_INFO(this->get_logger(),"trajectory_setpoint_start: vx:%f vy:%f",vx,vy);
// 	start_temp=end_temp;
// 	set_target_point("base_link",0,0,z,yaw);
// 	first=false;
// 	}
// 	if(pose_.header.stamp.sec == 0){
// 		RCLCPP_INFO(this->get_logger(), "No pose data received yet");
// 		return false;
// 	}
// 	z=end_temp.z()+z;
// 	yaw=end_temp.w()+yaw;
// 	if(
// 		!set_time(time)
// 	){
// 		if(
// 			//publish_trajectory_setpoint_yaw(&yaw)&
// 			publish_trajectory_setpoint_z(&z,accuracy)
// 		){
// 			RCLCPP_INFO(this->get_logger(), "at_check_point");
// 			return false;
// 		}else{
// 			RCLCPP_INFO(this->get_logger(), "alt_hold: z=%f,yaw=%f", z,yaw);
// 		send_velocity_command(vx1, vy1, z, yaw);
// 		return false;
// 		}
// 	}else{
// 		first=true;
// 		return true;
// 	}
// }

// // //
// // while True:
// //     x=get_value(2)
// //     y=get_value(3)
// //     if x!=0 and y!=0:
// //         dx=x
// //         dy=y
// //         PidRTL(dx,dy,frtl,vehicle)
// //         frtl=get_value(1)
// //     elif time.time()-time0>290:
// //         vehicle.mode = VehicleMode("LAND")
// //         time.sleep(10)
// //         vehicle.armed = False
// //         break
// // print((time.time()-time0))
// void OffboardControl::PidRTL(double x,double y,double frtl){
// 	// PID控制
// 	double Kp = 0.5;  // 比例系数 0.5/0.47
// 	double Ki = 0.1;  // 积分系数 0.1
// 	double Kd = 0.02;  // 微分系数
// 	double target_X = 69;  // 目标X轴坐标/70
// 	double current_X = 0;  // 当前X轴坐标
// 	double target_Y = 51;  // 目标Y轴坐标/53
// 	double current_Y = 0;  // 当前Y轴坐标
// 	double error_priorX = 0;  // 上一次误差
// 	double integralX = 0;  // 积分
// 	double derivativeX = 0;  // 微分
// 	double error_priorY = 0;  // 上一次误差
// 	double integralY = 0;  // 积分
// 	double derivativeY = 0;  // 微分
// 	current_X=x/640*140; // 测量当前X坐标
// 	current_Y=y/480*105; // 测量当前Y坐标
// 	if (current_X<=67 || current_X>=71){
// 		// 计算误差
// 		double errorX = target_X - current_X;
// 		// 计算积分
// 		integralX = integralX + errorX;
// 		// 计算微分
// 		derivativeX = errorX - error_priorX;
// 		// 计算控制量
// 		double controlX = Kp * errorX + Ki * integralX + Kd * derivativeX;
// 		// 更新上一次误差
// 		error_priorX = errorX;
// 		// 应用控制量到无人机
// 		double vy=controlX*-0.01;
// 		send_velocity_command(0,vy,0,0);
// 	}
// 	if (current_Y<=50 || current_Y>=52){
// 		// 计算误差
// 		double errorY = target_Y - current_Y;
// 		// 计算积分
// 		integralY = integralY + errorY;
// 		// 计算微分
// 		derivativeY = errorY - error_priorY;
// 		// 计算控制量
// 		double controlY = Kp * errorY + Ki * integralY + Kd * derivativeY;
// 		// 更新上一次误差
// 		error_priorY = errorY;
// 		// 应用控制量到无人机
// 		double vx=controlY*0.01;
// 		send_velocity_command(vx,0,0,0);
// 	}
// 	if (location.local_frame.z()>=3.5){
// 		double z = 3.2;
// 		publish_trajectory_setpoint_z(&z,0.1);
// 		send_velocity_command(0, 0, z, 0);
// 	}
// 	if (current_X>=67 && current_X<=71 && current_Y>=50 && current_Y<=52){// and vehicle.location.global_relative_frame.alt < 2.0):
// 		frtl=frtl+1;
// 		if(frtl>= 288){
// 			send_velocity_command(0,0,0.8,0);
// 			rclcpp::sleep_for(std::chrono::seconds(3));
// 			command_takeoff_or_land("LAND");
// 			rclcpp::sleep_for(std::chrono::seconds(5));
// 			arm_motors(false);
// 		}
// 	}else if(location.local_frame.z()<=1.5){
// 		send_velocity_command(0,0,0,0);
// 		rclcpp::sleep_for(std::chrono::seconds(3));
// 		command_takeoff_or_land("LAND");
// 		rclcpp::sleep_for(std::chrono::seconds(10));
// 		arm_motors(false);
// 	}
// }
