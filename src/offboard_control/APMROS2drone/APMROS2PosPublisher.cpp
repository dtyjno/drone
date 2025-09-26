#include "APMROS2PosPublisher.h"
#include "../utils/utils.h"

void APMROS2PosPublisher::init_ROS2_topics() {
	//global_gps_publisher_{this->create_publisher<ardupilot_msgs::msg::GlobalPosition>(ardupilot_namespace+"cmd_gps_pose", 5)},
	twist_stamped_publisher_=node->create_publisher<geometry_msgs::msg::TwistStamped>(topic_namespace+"setpoint_velocity/cmd_vel", 5);
	//  * /mavros/setpoint_position/global [geographic_msgs/msg/GeoPoseStamped] 1 subscriber
	// global_gps_publisher_=this->create_publisher<geographic_msgs::msg::GeoPoseStamped>(ardupilot_namespace+"setpoint_position/global", 5);
	// * /mavros/setpoint_position/local [geometry_msgs/msg/PoseStamped] 1 subscriber
	local_setpoint_publisher_=node->create_publisher<geometry_msgs::msg::PoseStamped>(topic_namespace+"setpoint_position/local", 5);
	// * /mavros/setpoint_raw/local [mavros_msgs/msg/PositionTarget] 1 subscriber
	setpoint_raw_local_publisher_=node->create_publisher<mavros_msgs::msg::PositionTarget>(topic_namespace+"setpoint_raw/local", 5);
	//  * /mavros/setpoint_raw/global [mavros_msgs/msg/GlobalPositionTarget] 1 subscriber
	setpoint_raw_global_publisher_=node->create_publisher<mavros_msgs::msg::GlobalPositionTarget>(topic_namespace+"setpoint_raw/global", 5);
	// /mavros/setpoint_trajectory/local [trajectory_msgs/msg/MultiDOFJointTrajectory] 1 subscriber
	// trajectory_publisher_=this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(ardupilot_namespace+"setpoint_trajectory/local", 5);
	setpoint_accel_publisher_=node->create_publisher<geometry_msgs::msg::Vector3Stamped>(topic_namespace+"setpoint_accel/accel", 5);
	setpoint_raw_attitude_publisher_=node->create_publisher<mavros_msgs::msg::AttitudeTarget>(topic_namespace+"setpoint_raw/attitude", 5);
}

void APMROS2PosPublisher::publish_setpoint_raw(Vector4f p, Vector4f v)
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
void APMROS2PosPublisher::publish_setpoint_raw_global(double latitude, double longitude, double altitude, double yaw)
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
void APMROS2PosPublisher::send_local_setpoint_command(double x, double y, double z, double yaw)
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
bool APMROS2PosPublisher::local_setpoint_command(Vector4f now, Vector4f target, double accuracy)
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
		RCLCPP_INFO(node->get_logger(), "pc-local_setpoint_command:at_check_point");
		first = true;
		return true;
	}
	return false;
}

// 发布速度控制指令
// linear_x, linear_y, linear_z 为速度(m/s), angular_z为角速度(度/s)
//
// ros2 topic pub /mavros/setpoint_velocity/cmd_vel geometry_msgs/msg/TwistStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}'
// void APMROS2PosPublisher::send_velocity_command_world(double linear_x, double linear_y, double linear_z, double angular_z)
// {
// 	geometry_msgs::msg::TwistStamped msg;
// 	msg.twist.linear.x = linear_x;
// 	msg.twist.linear.y = linear_y;
// 	msg.twist.linear.z = linear_z;
// 	msg.twist.angular.z = angular_z;
// 	msg.header.stamp = node->now();
// 	msg.header.frame_id = "base_link";
// 	twist_stamped_publisher_->publish(msg);
// }
// void APMROS2PosPublisher::send_velocity_command_world(Vector4f v)
// {
// 	geometry_msgs::msg::TwistStamped msg;
// 	msg.twist.linear.x = v.x();
// 	msg.twist.linear.y = v.y();
// 	msg.twist.linear.z = v.z();
// 	msg.twist.angular.z = v.w();
// 	msg.header.stamp = node->now();
// 	msg.header.frame_id = "base_link";
// 	twist_stamped_publisher_->publish(msg);
// }

void APMROS2PosPublisher::send_velocity_command(Vector4f v)
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
bool APMROS2PosPublisher::send_velocity_command_with_time(Vector4f v, double time)
{
	static Timer timer = Timer();
	static bool first = true;
	if (first)
	{
		timer.reset();
		first = false;
	}
	if (timer.elapsed() >= time)
	{
		send_velocity_command(Vector4f::Zero());
		first = true;
		return true;
	}
	else
	{
		send_velocity_command(v);
		return false;
	}
}
void APMROS2PosPublisher::send_accel_command(Vector4f v)
{
	// (void)v;
	geometry_msgs::msg::Vector3Stamped msg;
	msg.vector.x = v.x();
	msg.vector.y = v.y();
	msg.vector.z = v.z();
	msg.header.stamp = node->now();
	setpoint_accel_publisher_->publish(msg);
}
