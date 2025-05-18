#include "rclcpp/rclcpp.hpp"

#include "InertialNav.h"
#include "math.h"

Vector3f InertialNav::position;
Vector4f InertialNav::velocity;
Vector3f InertialNav::gps;
Quaternionf InertialNav::orientation;
float InertialNav::altitude;
Vector3f InertialNav::linear_acceleration;
Vector3f InertialNav::angular_velocity;
float InertialNav::rangefinder_height;


// 接收位置数据
void InertialNav::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
{
	position = Vector3f(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
	orientation = Quaternionf(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
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

	// RCLCPP_INFO(this->get_logger(),"position x: %lf", pose_.pose.position.x);
	// std::cout << "position x: " << msg->pose.position.x << std::endl;
	// std::cout << "position y: " << msg->pose.position.y  << std::endl;
	// std::cout << "position z: " << msg->pose.position.z  << std::endl;
	//RCLCPP_INFO(this->get_logger(), "Received yaw: %f", quaternion_to_yaw(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w));
}
// 接收GPS数据
void InertialNav::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) 
{
	gps = Vector3f(msg->latitude, msg->longitude, msg->altitude);
	// node->set_gps();
	// RCLCPP_INFO(node->get_logger(), "Latitude: %f", (*msg).latitude);
	// RCLCPP_INFO(node->get_logger(), "Longitude: %f", (*msg).longitude);
	// RCLCPP_INFO(node->get_logger(), "Altitude: %f", (*msg).altitude);

}
// 接收速度数据
void InertialNav::velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) 
{
	velocity = Vector4f(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z, msg->twist.angular.z);

	// node->set_velocity();
	// RCLCPP_INFO(this->get_logger(), "Received velocity data");
	// RCLCPP_INFO(this->get_logger(), "Linear x: %f", msg->twist.linear.x);
	// RCLCPP_INFO(this->get_logger(), "Linear y: %f", msg->twist.linear.y);
	// RCLCPP_INFO(this->get_logger(), "Linear z: %f", msg->twist.linear.z);
	// RCLCPP_INFO(this->get_logger(), "Angular x: %f", msg->twist.angular.x);
	// RCLCPP_INFO(this->get_logger(), "Angular y: %f", msg->twist.angular.y);
	// RCLCPP_INFO(this->get_logger(), "Angular z: %f", msg->twist.angular.z);
}
// 接收高度数据=0
void InertialNav::altitude_callback(const mavros_msgs::msg::Altitude::SharedPtr msg) 
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
void InertialNav::imu_data_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
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
void InertialNav::rangefinder_callback(const sensor_msgs::msg::Range::SharedPtr msg)
{
	rangefinder_height = msg->range;
	// node->set_range();
	// RCLCPP_INFO(this->get_logger(), "Received range data");
	// RCLCPP_INFO(this->get_logger(), "Range: %f", msg->range);
	// RCLCPP_INFO(this->get_logger(), "Field of view: %f", msg->field_of_view);
	// RCLCPP_INFO(this->get_logger(), "Min range: %f", msg->min_range);
	// RCLCPP_INFO(this->get_logger(), "Max range: %f", msg->max_range);
}