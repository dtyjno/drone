// #include "rclcpp/rclcpp.hpp"

#include "APMROS2PosSubscriber.h"
// #include "math.h"

void APMROS2PosSubscriber::init_ROS2_topics() {
	// 质量服务配置
	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
	// 声明回调组,实例化回调组，类型为：可重入的
	rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_= node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
	// Each of these callback groups is basically a thread
	// Everything assigned to one of them gets bundled into the same thread
	auto sub_opt = rclcpp::SubscriptionOptions();
	sub_opt.callback_group = callback_group_subscriber_;
	
	// topic /mavros/local_position/odom/ msg nav_msgs/msg/Odometry

	//ros2 topic echo /mavros/local_position/pose geometry_msgs/msg/PoseStamped
	status_subscription_ = node->create_subscription<nav_msgs::msg::Odometry>("/mavros/local_position/odom", qos,
	std::bind(&APMROS2PosSubscriber::status_callback, this, std::placeholders::_1),sub_opt);
	//ros2 topic echo /mavros/global_position/global sensor_msgs/msg/NavSatFix
	gps_subscription_ = node->create_subscription<sensor_msgs::msg::NavSatFix>(topic_namespace+"global_position/global", qos,
	std::bind(&APMROS2PosSubscriber::gps_callback, this, std::placeholders::_1),sub_opt);
	altitude_subscription_ = node->create_subscription<mavros_msgs::msg::Altitude>(topic_namespace+"altitude", qos,
	std::bind(&APMROS2PosSubscriber::altitude_callback, this, std::placeholders::_1),sub_opt);
	// /mavros/imu/data (sensor_msgs/Imu)
	imu_data_subscription_ = node->create_subscription<sensor_msgs::msg::Imu>(topic_namespace+"imu/data", qos,
	std::bind(&APMROS2PosSubscriber::imu_data_callback, this, std::placeholders::_1),sub_opt);
	// 激光雷达高度
	rangefinder_subscription_ = node->create_subscription<sensor_msgs::msg::Range>(topic_namespace+"rangefinder/rangefinder", qos,
	std::bind(&APMROS2PosSubscriber::rangefinder_callback, this, std::placeholders::_1),sub_opt);
}

// 接收位置数据
void APMROS2PosSubscriber::status_callback(const nav_msgs::msg::Odometry::SharedPtr msg) 
{
	position = Vector3f(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
	orientation = Quaternionf(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
	velocity = Vector4f(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z, msg->twist.twist.angular.z);
	notify_position_update(position.x(), position.y(), position.z());
	notify_velocity_update(velocity.x(), velocity.y(), velocity.z());
	calculate_euler(roll, pitch, yaw);
	notify_orientation_update(roll, pitch, yaw);
	// node->set_pose();
	// quaternion.w() = msg->pose.orientation.w;
	// quaternion.x() = msg->pose.orientation.x;
	// quaternion.y() = msg->pose.orientation.y;
	// quaternion.z() = msg->pose.orientation.z;
	// location.local_frame.x = loc.pose.position.x;
	// location.local_frame.y = loc.pose.position.y;
	// location.local_frame.z = loc.pose.position.z;
	// quaternion.w() = loc.pose.orientation.w;
	// quaternion.x() = loc.pose.orientation.x;
	// quaternion.y() = loc.pose.orientation.y;
	// quaternion.z() = loc.pose.orientation.z;
	// euler = quaternion_to_euler(quaternion);
	// location.global_frame.lat = loc.latitude;
	// location.global_frame.lon = loc.longitude;
	// location.global_frame.alt = loc.altitude;
	
	// timestamp_seconds = msg->header.stamp.sec;       // 秒部分
	// timestamp_nanoseconds = msg->header.stamp.nanosec; // 纳秒部分

	// RCLCPP_INFO(node->get_logger(),"position x: %lf", pose.position.x);

	// std::cout << "position x: " << msg->pose.pose.position.x << std::endl;
	// std::cout << "position y: " << msg->pose.pose.position.y  << std::endl;
	// std::cout << "position z: " << msg->pose.pose.position.z  << std::endl;
	
	// RCLCPP_INFO(this->get_logger(), "Received yaw: %f", quaternion_to_yaw(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w));
}
// 接收GPS数据
void APMROS2PosSubscriber::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) 
{
	gps = Vector3f(msg->latitude, msg->longitude, msg->altitude);
	// node->set_gps();
	// RCLCPP_INFO(node->get_logger(), "Latitude: %f", (*msg).latitude);
	// RCLCPP_INFO(node->get_logger(), "Longitude: %f", (*msg).longitude);
	// RCLCPP_INFO(node->get_logger(), "Altitude: %f", (*msg).altitude);

}
// 接收高度数据=0
void APMROS2PosSubscriber::altitude_callback(const mavros_msgs::msg::Altitude::SharedPtr msg) 
{
	altitude = msg->monotonic;
	// node->set_altitude();
	// this->location.global_frame.alt = msg->amsl;
	// RCLCPP_INFO(this->get_logger(), "Received altitude data");
	// RCLCPP_INFO(this->get_logger(), "Monotonic: %f", msg->monotonic);
	// RCLCPP_INFO(this->get_logger(), "Amsl: %f", msg->amsl);
	// RCLCPP_INFO(this->get_logger(), "Local: %f", msg->local);
	// RCLCPP_INFO(this->get_logger(), "Relative: %f", msg->relative);
	// RCLCPP_INFO(this->get_logger(), "Terrain: %f", msg->terrain);
	// RCLCPP_INFO(this->get_logger(), "Bottom clear: %f", msg->bottom_clearance);
}
// 接收IMU数据
void APMROS2PosSubscriber::imu_data_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
	// orientation = Quaternionf(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
	linear_acceleration = Vector3f(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
	angular_velocity = Vector3f(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
	// node->set_imu_data();
	// RCLCPP_INFO(this->get_logger(), "Received IMU data");
	// RCLCPP_INFO(this->get_logger(), "Orientation x: %f", msg->orientation.x);
	// RCLCPP_INFO(this->get_logger(), "Orientation y: %f", msg->orientation.y);
	// RCLCPP_INFO(this->get_logger(), "Orientation z: %f", msg->orientation.z);
	// RCLCPP_INFO(this->get_logger(), "Orientation w: %f", msg->orientation.w);
	// RCLCPP_INFO(this->get_logger(), "Angular velocity x: %f", msg->angular_velocity.x);
	// RCLCPP_INFO(this->get_logger(), "Angular velocity y: %f", msg->angular_velocity.y);
	// RCLCPP_INFO(this->get_logger(), "Angular velocity z: %f", msg->angular_velocity.z);
	// RCLCPP_INFO(this->get_logger(), "Linear acceleration x: %f", msg->linear_acceleration.x);
	// RCLCPP_INFO(this->get_logger(), "Linear acceleration y: %f", msg->linear_acceleration.y);
	// RCLCPP_INFO(this->get_logger(), "Linear acceleration z: %f", msg->linear_acceleration.z);
}

// 接收激光雷达数据
void APMROS2PosSubscriber::rangefinder_callback(const sensor_msgs::msg::Range::SharedPtr msg)
{
	rangefinder_height = msg->range;
	// node->set_range();
	// RCLCPP_INFO(this->get_logger(), "Received range data");
	// RCLCPP_INFO(this->get_logger(), "Range: %f", msg->range);
	// RCLCPP_INFO(this->get_logger(), "Field of view: %f", msg->field_of_view);
	// RCLCPP_INFO(this->get_logger(), "Min range: %f", msg->min_range);
	// RCLCPP_INFO(this->get_logger(), "Max range: %f", msg->max_range);
}