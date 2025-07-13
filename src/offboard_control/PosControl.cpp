#include "PosControl.h"
#include "InertialNav.h"
#include "math.h"
#include "PID.h"
#include "FuzzyPID.h"


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
	msg.velocity.z = 0;

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
	RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Publishing local setpoint: x=%f, y=%f, z=%f, yaw=%f", x, y, z, yaw);
	local_setpoint_publisher_->publish(msg);
}

// 发布本地位置控制指令 PID
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
}

// 输入位置 pid控制速度
// 初始时 now > target : direction = true 记为正方向 否则记为负方向
Vector3f PosControl::input_pos_xyz(Vector3f now, Vector3f target, bool fuzzy)
{
	if(fuzzy){
		float delta_k = 2.0f;

		Vector3f f;
		float kp = 0, ki = 0, kd = 0;

		pid_x.get_pid(kp, ki, kd);
		fuzzy_pid.fuzzy_pid_control(now.x(), target.x(), 0, kp, ki, kd, delta_k);
		pid_x.set_gains(kp, ki, kd);
		f.x() = pid_x.update_all(now.x(), target.x(), dt, max_speed_xy, InertialNav::velocity.x());

		pid_y.get_pid(kp, ki, kd);
		fuzzy_pid.fuzzy_pid_control(now.y(), target.y(), 1, kp, ki, kd, delta_k);
		pid_y.set_gains(kp, ki, kd);
		f.y() = pid_y.update_all(now.y(), target.y(), dt, max_speed_xy, InertialNav::velocity.y());
		
		pid_z.get_pid(kp, ki, kd);
		fuzzy_pid.fuzzy_pid_control(now.z(), target.z(), 2, kp, ki, kd, delta_k);
		pid_z.set_gains(kp, ki, kd);
		f.z() = pid_z.update_all(now.z(), target.z(), dt, max_speed_z, InertialNav::velocity.z());
		
		RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz: x:%f y:%f z:%f", f.x(), f.y(), f.z());
		return f;
	} else{
		Vector3f f;
		f.x() = pid_x.update_all(now.x(), target.x(), dt, max_speed_xy, InertialNav::velocity.x());
		f.y() = pid_y.update_all(now.y(), target.y(), dt, max_speed_xy, InertialNav::velocity.y());
		f.z() = pid_z.update_all(now.z(), target.z(), dt, max_speed_z, InertialNav::velocity.z());
		RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz: x:%f y:%f z:%f", f.x(), f.y(), f.z());
		return f;
	}
}

// 输入位置 pid控制速度
Vector4f PosControl::input_pos_xyz_yaw(Vector4f now, Vector4f target, bool fuzzy)
{
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
	if(fuzzy){
		float delta_k = 2.0f;

		Vector4f f;
		float kp = 0, ki = 0, kd = 0;

		pid_x.get_pid(kp, ki, kd);
		// RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: x   p:%f i:%f d:%f", 
		// 	kp, ki, kd);
		fuzzy_pid.fuzzy_pid_control(now.x(), target.x(), 0, kp, ki, kd, delta_k);
		pid_x.set_gains(kp, ki, kd);
		f.x() = pid_x.update_all(now.x(), target.x(), dt, max_speed_xy, InertialNav::velocity.x());
		// RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: x   p:%f i:%f d:%f", 
		// 	kp, ki, kd);
		pid_y.get_pid(kp, ki, kd);
		fuzzy_pid.fuzzy_pid_control(now.y(), target.y(), 1, kp, ki, kd, delta_k);
		pid_y.set_gains(kp, ki, kd);
		f.y() = pid_y.update_all(now.y(), target.y(), dt, max_speed_xy, InertialNav::velocity.y());
		// RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: y   p:%f i:%f d:%f", 
		// 	kp, ki, kd);		
		pid_z.get_pid(kp, ki, kd);
		fuzzy_pid.fuzzy_pid_control(now.z(), target.z(), 2, kp, ki, kd, delta_k);
		pid_z.set_gains(kp, ki, kd);
		f.z() = pid_z.update_all(now.z(), target.z(), dt, max_speed_z, InertialNav::velocity.z());
		// RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: z   p:%f i:%f d:%f", 
		// 	kp, ki, kd);
		pid_yaw.get_pid(kp, ki, kd);
		fuzzy_pid.fuzzy_pid_control(now.w(), target.w(), 3, kp, ki, kd, delta_k);
		pid_yaw.set_gains(kp, ki, kd);
		f.w() = pid_yaw.update_all(now.w(), target.w(), dt, max_speed_yaw, InertialNav::velocity.w());
		// RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: yaw p:%f i:%f d:%f", 
		// 	kp, ki, kd);
		RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: px:%f py:%f pz:%f pyaw:%f",
			now.x(), now.y(), now.z(), now.w());
		RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: tx:%f ty:%f tz:%f tyaw:%f",
			target.x(), target.y(), target.z(), target.w());
		RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: vx:%f vy:%f vz:%f vyaw:%f",
			f.x(), f.y(), f.z(), f.w());
		return f;
	} else{
		Vector4f f;
		f.x() = pid_x.update_all(now.x(), target.x(), dt, max_speed_xy, InertialNav::velocity.x());
		f.y() = pid_y.update_all(now.y(), target.y(), dt, max_speed_xy, InertialNav::velocity.y());
		f.z() = pid_z.update_all(now.z(), target.z(), dt, max_speed_z, InertialNav::velocity.z());
		f.w() = pid_yaw.update_all(now.w(), target.w(), dt, max_speed_yaw, InertialNav::velocity.w());
		RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: vx:%f vy:%f vz:%f vyaw:%f", f.x(), f.y(), f.z(), f.w());
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
	// if (target.w() > M_PI)
	// 	target.w() -= 2 * M_PI;
	// else if (target.w() < -M_PI)
	// 	target.w() += 2 * M_PI;
	f.w() = pid_yaw.update_all(now.w(), target.w(), dt, max_speed_yaw, InertialNav::velocity.w());
	RCLCPP_INFO(node->get_logger(), "input_pos_vel_xyz_yaw: x:%f y:%f z:%f yaw:%f", f.x(), f.y(), f.z(), f.w());

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
	Vector4f pos = {
			pid_px.update_all(now.x(), target.x(), 0, 2 * max_speed_xy, InertialNav::velocity.x()),
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
	if (first)
	{
		// pos_target.rotate_xy(default_yaw);
		_pos_target = pos_now + pos_target;
		pos_target_temp = pos_target;
		RCLCPP_INFO(node->get_logger(), "trajectory_setpoint: x:%f y:%f", _pos_target.x(), _pos_target.y());
		reset_pid();
		
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
		send_velocity_command_world(input_pos_xyz_yaw(pos_now, _pos_target));
		// send_velocity_command_world(input_pos_xyz_yaw(pos_now, _pos_target, true, direction));
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
	if (first) {
		_pos_target = pos_target;
		RCLCPP_INFO(node->get_logger(), "trajectory_setpoint: x:%f y:%f z:%f", _pos_target.x(), _pos_target.y(), _pos_target.z());
		reset_pid();
		first = false;
	}
	if (is_equal(pos_target, _pos_target, 0.1f)) {
#ifdef PID_P
		// send_velocity_command_world(input_pos_vel_1_xyz_yaw(pos_now,_pos_target));
		send_accel_command(input_pos_vel_xyz_yaw(pos_now, _pos_target));
#endif
#ifndef PID_P
		// send_velocity_command_world(input_pos_xyz_yaw(pos_now, _pos_target));
		send_velocity_command_world(input_pos_xyz_yaw(pos_now, _pos_target, true));
#endif
	}
	else {
		RCLCPP_INFO(node->get_logger(), "change point");
		first = true;
	}
	if (abs(pos_now.x() - _pos_target.x()) <= accuracy &&
			abs(pos_now.y() - _pos_target.y()) <= accuracy &&
			abs(pos_now.z() - _pos_target.z()) <= accuracy
	) {
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
		// send_velocity_command_world(input_pos_xyz_yaw_without_vel(pos_now, pos_target));
		send_velocity_command_world(input_pos_xyz_yaw(pos_now, pos_target, true));
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
// 航点设置，s型速度规划(开环)，飞行经过指定位置（相对于起飞点/世界坐标系）
bool PosControl::trajectory_generator_world(double speed_factor, std::array<double, 3> q_goal, Vector3f max_speed, Vector3f max_accel)
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
		std::cout << InertialNav::position.x() << " " << InertialNav::position.y() << " " << InertialNav::position.z() << " " << std::endl;
		current_state.q_d = {InertialNav::position.x(), InertialNav::position.y(), InertialNav::position.z()};
		_trajectory_generator = std::make_unique<TrajectoryGenerator>(speed_factor, q_goal); // Use smart pointer for memory management
		_trajectory_generator->set_dq_max({MIN(max_speed_xy, max_speed.x()), MIN(max_speed_xy, max_speed.y()), MIN(max_speed_z, max_speed.z())});
		_trajectory_generator->set_dqq_max({MIN(max_accel_xy, max_accel.x()), MIN(max_accel_xy, max_accel.y()), MIN(max_accel_z, max_accel.z())});
		// _trajectory_generator->set_dq_max({1, 1, 1});
		// _trajectory_generator->set_dqq_max({0.2, 0.2, 0.2});
		pos_start_temp = Vector4f(InertialNav::position.x(), InertialNav::position.y(), InertialNav::position.z(), 0.0f);
		pos_target_temp = Vector4f(static_cast<float>(q_goal[0]), static_cast<float>(q_goal[1]), static_cast<float>(q_goal[2]), 0);
		count = 0;
		isFinished = false;
		first = false; // Ensure that initialization block runs only once
	}
	if (is_equal(Vector4f(q_goal[0], q_goal[1], q_goal[2], 0), pos_target_temp, 0.1f))
	{
		if (!isFinished)
		{
			isFinished = (*_trajectory_generator)(current_state, count); // count / 10
			count++; // Increment count to simulate time passing
			RCLCPP_INFO(node->get_logger(), "trajectory_generator: is_equal count:%ld, isFinished:%d", count, isFinished);
		}
		
		if (isFinished)
		{
			RCLCPP_INFO(node->get_logger(), "trajectory_generator: Motion completed!");
			first = true; // Reset first to true to reinitialize the trajectory generator
			return true; // 返回完成状态
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
	
	// send_local_setpoint_command(
	// 	pos_start_temp.x() + static_cast<float>(_trajectory_generator->delta_q_d[0]),
	// 	pos_start_temp.y() + static_cast<float>(_trajectory_generator->delta_q_d[1]),
	// 	pos_start_temp.z() + static_cast<float>(_trajectory_generator->delta_q_d[2]),
	// 	0
	// );

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
			true)
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
	pid.set_pid(defaults);
}

void PosControl::reset_pid()
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

#ifdef PAL_STATISTIC_VISIBILITY
#include "vector"

void PosControl::publish_statistic(std::vector<pal_statistics_msgs::msg::Statistic> &statistics){
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