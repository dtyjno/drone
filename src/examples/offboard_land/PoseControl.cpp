#include "PoseControl.h"
#include "InertialNav.h"
#include "math.h"
#include "PID.h"

#define PID_P

void PoseControl::publish_setpoint_raw(Vector4f p, Vector4f v){
	// RCLCPP_INFO(node->get_logger(), "Publishing setpoint: latitude=%f, longitude=%f, altitude=%f, yaw=%f", latitude, longitude, altitude, yaw);
	mavros_msgs::msg::PositionTarget msg;
	msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_BODY_NED;
	//mavros_msgs::msg::GlobalPositionTarget::FRAME_GLOBAL_INT;
	msg.position.x = p.x;
	msg.position.y = p.y;
	msg.position.z = p.z;
	msg.yaw = p.yaw;
	msg.velocity.x = v.x;
	msg.velocity.y = v.y;
	msg.velocity.z = v.z;
	msg.yaw_rate = v.yaw;
	// msg.acceleration_or_force.x = 0;
	// msg.acceleration_or_force.y = 0;
	// msg.acceleration_or_force.z = 0;
    msg.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFZ |//|
                    // mavros_msgs::msg::PositionTarget::IGNORE_PX |
                    // mavros_msgs::msg::PositionTarget::IGNORE_PY |
                    // mavros_msgs::msg::PositionTarget::IGNORE_PZ |
                    mavros_msgs::msg::PositionTarget::IGNORE_VX |
                    mavros_msgs::msg::PositionTarget::IGNORE_VY |
                    mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                    // mavros_msgs::msg::PositionTarget::IGNORE_YAW |
                    mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
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
//publishing : mavros_msgs.msg.GlobalPositionTarget(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1713969082, nanosec=321245648), frame_id=''), coordinate_frame=0, type_mask=0, latitude=1.0, longitude=2.0, altitude=3.0, velocity=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), acceleration_or_force=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), yaw=0.0, yaw_rate=0.0)
//ros2 topic pub /mavros/setpoint_raw/global mavros_msgs/msg/GlobalPositionTarget '{header: "auto", latitude: -35.363262, longitude: 149.165237, altitude: 700.788637}'
void PoseControl::publish_setpoint_raw_global(double latitude, double longitude, double altitude, double yaw){
	RCLCPP_INFO(node->get_logger(), "Publishing setpoint: latitude=%f, longitude=%f, altitude=%f, yaw=%f", latitude, longitude, altitude, yaw);
	mavros_msgs::msg::GlobalPositionTarget msg;
	msg.coordinate_frame = mavros_msgs::msg::GlobalPositionTarget::FRAME_GLOBAL_INT;
	//mavros_msgs::msg::GlobalPositionTarget::FRAME_GLOBAL_INT;
	msg.latitude = latitude;
	msg.longitude = longitude;
	msg.altitude = altitude;
	msg.yaw = yaw;
	msg.velocity.x = 0;
	msg.velocity.y = 0;
	msg.velocity.z = 0;
	msg.yaw_rate = 0;
	msg.acceleration_or_force.x = 0;
	msg.acceleration_or_force.y = 0;
	msg.acceleration_or_force.z = 0;
	
	msg.header.stamp = node->now();
	msg.header.frame_id = "";
	setpoint_raw_global_publisher_->publish(msg);
}


// 发布本地位置控制指令
// send_local_setpoint_command(x, y, z, yaw); 飞行到
// 飞行到相对于世界坐标系的(x,y,z)位置
// ros2 topic pub /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, pose: {position: {x: 0.0, y: 0.0, z: 5.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}'
void PoseControl::send_local_setpoint_command(double x, double y, double z,double yaw){
	geometry_msgs::msg::PoseStamped msg;
	yaw = yaw/2/M_PI;
  	msg.pose.position.x = x;
	msg.pose.position.y = y;
	msg.pose.position.z = z;
	msg.pose.orientation.x = 0;
	msg.pose.orientation.y = 0;
	msg.pose.orientation.z = 0;
	msg.pose.orientation.w = 0;
	msg.header.stamp = node->now();
	msg.header.frame_id = "base_link";
	local_setpoint_publisher_->publish(msg);
}

// 发布速度控制指令
// linear_x, linear_y, linear_z 为速度(m/s), angular_z为角速度(度/s)
//
// ros2 topic pub /mavros/setpoint_velocity/cmd_vel geometry_msgs/msg/TwistStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}'
void PoseControl::send_velocity_command_world(double linear_x, double linear_y, double linear_z, double angular_z)
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
void PoseControl::send_velocity_command_world(Vector4f v)
{
  geometry_msgs::msg::TwistStamped msg;
  msg.twist.linear.x = v.x;
  msg.twist.linear.y = v.y;
  msg.twist.linear.z = v.z;
  msg.twist.angular.z = v.yaw;
  msg.header.stamp = node->now();
  msg.header.frame_id = "base_link";
  twist_stamped_publisher_->publish(msg);
}

void PoseControl::send_velocity_command(Vector4f v)
{
  v.rotate_xy(default_yaw);
  geometry_msgs::msg::TwistStamped msg;
  msg.twist.linear.x = v.x;
  msg.twist.linear.y = v.y;
  msg.twist.linear.z = v.z;
  msg.twist.angular.z = v.yaw;
  msg.header.stamp = node->now();
  msg.header.frame_id = "base_link";
  twist_stamped_publisher_->publish(msg);
}

// 发布定时速度控制指令 v(m/s) time(s)
// linear_x, linear_y, linear_z 为速度(m/s), angular_z为角速度(度/s)
// send_velocity_command_with_time(3, 0, 0, 0, 3);//前进3m/s,3s后停止
bool PoseControl::send_velocity_command_with_time(Vector4f v, double time){
	static bool first=true;
	static double find_start;	
	if(first){
		find_start = node->get_clock()->now().nanoseconds() / 1000;
		first=false;
	}
	geometry_msgs::msg::TwistStamped msg;
	if((node->get_clock()->now().nanoseconds() / 1000-find_start)>1000000*time){
		send_velocity_command_world(0, 0, 0, 0);
		first=true;
		return true;
	}
	else{
		send_velocity_command(v);
		return false;
  	}
}
void PoseControl::send_accel_command(Vector4f v){
	// geometry_msgs::msg::Vector3Stamped msg;
	// msg.vector.x = x;
	// msg.vector.y = y;
	// msg.vector.z = z;
	// msg.header.stamp = node->now();
	// setpoint_accel_publisher_->publish(msg);

	mavros_msgs::msg::AttitudeTarget msg;
	msg.header.stamp = node->now();
    msg.header.frame_id = "base_link";
	msg.body_rate.x = v.y;
	msg.body_rate.y = -v.x;
	msg.body_rate.z = v.z;
	// msg.thrust = -v.z;
	// msg.orientation.w = 
	setpoint_raw_attitude_publisher_->publish(msg);
	
	//  geometry_msgs::msg::TwistStamped msg1;
	// msg1.twist.angular.x = x;
	// msg1.twist.angular.y = y;

	// msg1.header.stamp = node->now();
	// msg1.header.frame_id = "base_link";
	// twist_stamped_publisher_->publish(msg1);
}
// void PoseControl::setYawRate(float yaw_rate) {
//         // 创建并填充TwistStamped消息
//         geometry_msgs::msg::TwistStamped twist_msg;
//         twist_msg.header.stamp = node->now();
//         twist_msg.twist.angular.z = yaw_rate; // 设置偏航角速度，正值为顺时针，负值为逆时针

//         // 发布消息
//         twist_stamped_publisher_->publish(twist_msg);
// }
// 输入位置 pid控制速度
Vector3f PoseControl::input_pos_xyz(Vector3f now, Vector3f target){
	Vector3f f;
	f.x = pid_x.update_all(now.x,target.x,dt,max_speed_xy,InertialNav::velocity.x);
	f.y = pid_y.update_all(now.y,target.y,dt,max_speed_xy,InertialNav::velocity.y);
	f.z = pid_z.update_all(now.z,target.z,dt,max_speed_z,InertialNav::velocity.z);
	return f;
}
// 输入位置 pid控制速度
Vector4f PoseControl::input_pos_xyz_yaw(Vector4f now, Vector4f target){
	Vector4f f;
	f.x = pid_x.update_all(now.x,target.x,dt,max_speed_xy,InertialNav::velocity.x);
	f.y = pid_y.update_all(now.y,target.y,dt,max_speed_xy,InertialNav::velocity.y);
	f.z = pid_z.update_all(now.z,target.z,dt,max_speed_z,InertialNav::velocity.z);
	// 处理yaw跨越点问题
    float yaw_diff = target.yaw - now.yaw;
	 // 检测并调整yaw差值，确保其在-π到π范围内
    if (yaw_diff > M_PI) {
        yaw_diff -= 2 * M_PI; // 如果差值大于π，减去2π调整
    } else if (yaw_diff < -M_PI) {
        yaw_diff += 2 * M_PI; // 如果差值小于-π，加上2π调整
    }
	f.yaw = pid_yaw.update_all(now.yaw,target.yaw,dt,max_speed_yaw,InertialNav::velocity.yaw);
	RCLCPP_INFO(node->get_logger(),"input_pos_vel_xyz_yaw: x:%f y:%f z:%f",f.x,f.y,f.z);

	return f;
}
// 输入位置 串级pid控制速度
Vector4f PoseControl::input_pos_vel_1_xyz_yaw(Vector4f now, Vector4f target){
	Vector4f f;
	static Vector4f v_v= {0,0,0,0};
	// RCLCPP_INFO(node->get_logger(),"acc: %f,%f,%f",InertialNav::linear_acceleration.x,InertialNav::linear_acceleration.y,InertialNav::linear_acceleration.z);
	f.x = pid_vx.update_all(InertialNav::velocity.x,pid_px.update_all(now.x,target.x,0,2*max_speed_xy,InertialNav::velocity.x),dt_pid_p_v,max_speed_xy);//,InertialNav::linear_acceleration.x);
	f.y = pid_vy.update_all(InertialNav::velocity.y,pid_py.update_all(now.y,target.y,0,2*max_speed_xy,InertialNav::velocity.y),dt_pid_p_v,max_speed_xy);//,InertialNav::linear_acceleration.y);
	f.z = pid_vz.update_all(InertialNav::velocity.z,pid_pz.update_all(now.z,target.z,0,2*max_speed_z,InertialNav::velocity.z),dt_pid_p_v,max_speed_z);//,InertialNav::linear_acceleration.z-9.80665);
	// 处理yaw跨越点问题
    float yaw_diff = target.yaw - now.yaw;
    // 检测并调整yaw差值，确保其在-π到π范围内
    if (yaw_diff > M_PI) {
        yaw_diff -= 2 * M_PI; // 如果差值大于π，减去2π调整
    } else if (yaw_diff < -M_PI) {
        yaw_diff += 2 * M_PI; // 如果差值小于-π，加上2π调整
    }
	f.yaw = pid_yaw.update_all(now.yaw,target.yaw,dt,max_speed_yaw,InertialNav::velocity.yaw);
	v_v = InertialNav::velocity;
	RCLCPP_INFO(node->get_logger(),"input_pos_vel_xyz_yaw: x:%f y:%f z:%f",f.x,f.y,f.z);

	return f;
}
// 输入位置 串级pid控制速度
Vector4f PoseControl::input_pos_vel_xyz_yaw(Vector4f now, Vector4f target){
	Vector4f f;
	static Vector4f v_v= {0,0,0,0};
	// RCLCPP_INFO(node->get_logger(),"acc: %f,%f,%f",InertialNav::linear_acceleration.x,InertialNav::linear_acceleration.y,InertialNav::linear_acceleration.z);
	f.x = pid_vx.update_all(InertialNav::velocity.x,pid_px.update_all(now.x,target.x,0,max_speed_xy,InertialNav::velocity.x),dt_pid_p_v,max_accel_xy);//,InertialNav::linear_acceleration.x);
	f.y = pid_vy.update_all(InertialNav::velocity.y,pid_py.update_all(now.y,target.y,0,max_speed_xy,InertialNav::velocity.y),dt_pid_p_v,max_accel_xy);//,InertialNav::linear_acceleration.y);
	f.z = pid_vz.update_all(InertialNav::velocity.z,pid_pz.update_all(now.z,target.z,0,max_speed_z,InertialNav::velocity.z),dt_pid_p_v,max_accel_z);//,InertialNav::linear_acceleration.z-9.80665);
	// 处理yaw跨越点问题
    float yaw_diff = target.yaw - now.yaw;
    // 检测并调整yaw差值，确保其在-π到π范围内
    if (yaw_diff > M_PI) {
        yaw_diff -= 2 * M_PI; // 如果差值大于π，减去2π调整
    } else if (yaw_diff < -M_PI) {
        yaw_diff += 2 * M_PI; // 如果差值小于-π，加上2π调整
    }
	f.yaw = pid_yaw.update_all(now.yaw,target.yaw,dt,max_speed_yaw,InertialNav::velocity.yaw);
	v_v = InertialNav::velocity;
	RCLCPP_INFO(node->get_logger(),"input_pos_vel_xyz_yaw: x:%f y:%f z:%f",f.x,f.y,f.z);

	return f;
}
// 飞行到指定位置（相对于当前位置）
bool PoseControl::publish_setpoint_world(Vector4f now, Vector4f target, double accuracy, double yaw_accuracy){
		(void)yaw_accuracy;
	static bool first = true;
	static Vector4f pos_start;
	if(first){
		pos_start = now;
	}
	// Vector4f pos = {
	// 	pid_px.update_all(now.x,target.x,0,2*max_speed_xy,InertialNav::velocity.x),
	// 	pid_py.update_all(now.y,target.y,0,2*max_speed_xy,InertialNav::velocity.y),
	// 	pid_pz.update_all(now.z,target.z,0,2*max_speed_z,InertialNav::velocity.z),
	// 	pid_yaw.update_all(now.yaw,target.yaw,dt,max_speed_yaw,InertialNav::velocity.yaw)
	// };
	// Vector4f vel = {
	// 	pid_vx.update_all(InertialNav::velocity.x,pos.x,dt_pid_p_v,max_speed_xy),//,InertialNav::linear_acceleration.x);
	// 	pid_vy.update_all(InertialNav::velocity.y,pos.y,dt_pid_p_v,max_speed_xy),//,InertialNav::linear_acceleration.y);
	// 	pid_vz.update_all(InertialNav::velocity.z,pos.z,dt_pid_p_v,max_speed_z),//,InertialNav::linear_acceleration.z-9.80665);
	// 	0
	// };
	Vector4f pos = {
		pid_px.update_all(now.x,target.x,0,2*max_speed_xy,InertialNav::velocity.x),
		pid_py.update_all(now.y,target.y,0,2*max_speed_xy,InertialNav::velocity.y),
		pid_pz.update_all(now.z,target.z,0,2*max_speed_z,InertialNav::velocity.z),
		pid_yaw.update_all(now.yaw,target.yaw,dt,max_speed_yaw,InertialNav::velocity.yaw)
	};
	Vector4f vel;
	publish_setpoint_raw(pos, vel);
	if( abs(now.x - target.x)<=accuracy &&
	    abs(now.y - target.y)<=accuracy &&
		abs(now.z - target.z)<=accuracy
		// #ifndef PID_P
		// && abs(pos_now.yaw - _pos_target.yaw)<=yaw_accuracy
		// #endif
	){
		RCLCPP_INFO(node->get_logger(), "at_check_point");
		first=true;
		return true;
	}
	return false;
}
// 飞行到指定位置（相对于当前位置）
// x:前后方向位置(m) y:左右方向位置(m) z:高度(m) yaw:偏航角(°) accuracy=DEFAULT_ACCURACY:精度(m)
bool PoseControl::trajectory_setpoint(Vector4f pos_now,Vector4f pos_target,double accuracy,double yaw_accuracy){
		(void)yaw_accuracy;
	static bool first=true;
	static Vector4f pos_target_temp;
	if(first){
		pos_target.rotate_xy(default_yaw);
		_pos_target += pos_target;
		pos_target_temp = pos_target;
		RCLCPP_INFO(node->get_logger(),"trajectory_setpoint: x:%f y:%f",_pos_target.x,_pos_target.y);
		pid_x.set_pid_info();
		pid_y.set_pid_info();
		pid_z.set_pid_info();
		//pid_yaw.set_pid_info();
		first=false;
	}
	// if(pose_.header.stamp.sec == 0){
	// 	RCLCPP_INFO(this->get_logger(), "No pose data received yet");
	// 	return;
	// }
	if(is_equal(pos_target, pos_target_temp,0.01f)){
		#ifdef PID_P
		// send_velocity_command_world(input_pos_vel_1_xyz_yaw(pos_now,_pos_target));
		send_accel_command(input_pos_vel_xyz_yaw(pos_now,_pos_target));
		#endif
		#ifndef PID_P
		send_velocity_command_world(input_pos_xyz_yaw(pos_now,_pos_target));
		#endif	
	}else{
		printf("pos_target_temp:%f,%f,%f,%f\n",pos_target_temp.x,pos_target_temp.y,pos_target_temp.z,pos_target_temp.yaw);
		printf("pos_target:%f,%f,%f,%f\n",pos_target.x,pos_target.y,pos_target.z,pos_target.yaw);
		RCLCPP_INFO(node->get_logger(), "change point");
		first = true;
	}
	if( abs(pos_now.x - _pos_target.x)<=accuracy &&
	    abs(pos_now.y - _pos_target.y)<=accuracy &&
		abs(pos_now.z - _pos_target.z)<=accuracy
		// #ifndef PID_P
		// && abs(pos_now.yaw - _pos_target.yaw)<=yaw_accuracy
		// #endif
	){
		RCLCPP_INFO(node->get_logger(), "at_check_point");
		first=true;
		return true;
	}
	return false;
}
// 飞行到指定位置（相对于起飞点/世界坐标系）
// x:前后方向位置(m) y:左右方向位置(m) z:高度(m) yaw:偏航角(°) accuracy=DEFAULT_ACCURACY:精度(m)
bool PoseControl::trajectory_setpoint_world(Vector4f pos_now,Vector4f pos_target,double accuracy,double yaw_accuracy){
		(void)yaw_accuracy;
	static bool first=true;
	static Vector4f pos_target_temp;
	if(first){
		pos_target.rotate_xy(default_yaw);
		_pos_target = pos_target;
		pos_target_temp = pos_target;
		RCLCPP_INFO(node->get_logger(),"trajectory_setpoint: x:%f y:%f z:%f",_pos_target.x,_pos_target.y,_pos_target.z);
		pid_x.set_pid_info();
		pid_y.set_pid_info();
		pid_z.set_pid_info();
		// pid_yaw.set_pid_info();
		first=false;
	}
	// if(pose_.header.stamp.sec == 0){
	// 	RCLCPP_INFO(this->get_logger(), "No pose data received yet");
	// 	return;
	// }
	if(is_equal(pos_target, pos_target_temp,0.01f)){
		#ifdef PID_P
		// send_velocity_command_world(input_pos_vel_1_xyz_yaw(pos_now,_pos_target));
		send_accel_command(input_pos_vel_xyz_yaw(pos_now,_pos_target));
		#endif
		#ifndef PID_P
		send_velocity_command_world(input_pos_xyz_yaw(pos_now,_pos_target));
		#endif
	}else{
		RCLCPP_INFO(node->get_logger(), "change point");
		first = true;
	}
	if( abs(pos_now.x - _pos_target.x)<=accuracy &&
	    abs(pos_now.y - _pos_target.y)<=accuracy &&
		abs(pos_now.z - _pos_target.z)<=accuracy
		// && abs(pos_now.yaw - _pos_target.yaw)<=yaw_accuracy
	){
		RCLCPP_INFO(node->get_logger(), "at_check_point");
		first=true;
		return true;
	}
	return false;
}

bool PoseControl::trajectory_setpoint(Vector4f pos_now,Vector4f pos_target,PID::Defaults defaults,double accuracy, double yaw_accuracy){
		(void)yaw_accuracy;
	static PID pid_xy = PID(defaults);
	static bool first=true;
	static Vector4f pos_target_temp;
	if(first){
		pos_target.rotate_xy(default_yaw);
		_pos_target += pos_target;
		pos_target_temp = pos_target;
		RCLCPP_INFO(node->get_logger(),"trajectory_setpoint: x:%f y:%f",_pos_target.x,_pos_target.y);
		pid_xy.set_pid_info();
		pid_z.set_pid_info();
		// pid_yaw.set_pid_info();
		first=false;
	}
	// if(pose_.header.stamp.sec == 0){
	// 	RCLCPP_INFO(this->get_logger(), "No pose data received yet");
	// 	return;
	// }
	if(is_equal(pos_target, pos_target_temp,0.01f)){
		#ifdef PID_P
		send_velocity_command_world(input_pos_vel_1_xyz_yaw(pos_now,_pos_target));
		#endif
		#ifndef PID_P
		send_velocity_command_world(input_pos_xyz_yaw(pos_now,_pos_target));
		#endif	
	}else{
		RCLCPP_INFO(node->get_logger(), "change point");
		first = true;
	}
	if( abs(pos_now.x - _pos_target.x)<=accuracy &&
	    abs(pos_now.y - _pos_target.y)<=accuracy &&
		abs(pos_now.z - _pos_target.z)<=accuracy
		// && abs(pos_now.yaw - _pos_target.yaw)<=yaw_accuracy
	){
		RCLCPP_INFO(node->get_logger(), "at_check_point");
		first=true;
		return true;
	}
	return false;
}

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
// 	z=end_temp.z+z;
// 	yaw=end_temp.yaw+yaw;
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
// 	if (location.local_frame.z>=3.5){
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
// 	}else if(location.local_frame.z<=1.5){
// 		send_velocity_command(0,0,0,0);
// 		rclcpp::sleep_for(std::chrono::seconds(3));
// 		command_takeoff_or_land("LAND");
// 		rclcpp::sleep_for(std::chrono::seconds(10));
// 		arm_motors(false);
// 	}
// }
