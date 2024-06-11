#include "offboard_control.hpp"
bool OffboardControl::send_velocity_command_with_time(double linear_x, double linear_y, double linear_z, double angular_z,double time){
	static bool first=true;
	static double find_start;
	if(first){
		set_start_temp_point(pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z, quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w));
		find_start = this->get_clock()->now().nanoseconds() / 1000;
		first=false;
	}
	geometry_msgs::msg::TwistStamped msg;
	if((this->get_clock()->now().nanoseconds() / 1000-find_start)>1000000*time){
		msg.twist.linear.x = 0;
		msg.twist.linear.y = 0;
		msg.twist.linear.z = 0;
		msg.twist.angular.z = 0;
		msg.header.stamp = this->now();
		msg.header.frame_id = "base_link";
		twist_stamped_publisher_->publish(msg);
		set_end_temp_point(pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z, quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w));
		first=true;
		return true;
	}
	else{
		msg.twist.linear.x = linear_x;
		msg.twist.linear.y = linear_y;
		msg.twist.linear.z = linear_z;
		msg.twist.angular.z = angular_z/2/PI;
		msg.header.stamp = this->now();
		msg.header.frame_id = "base_link";
		twist_stamped_publisher_->publish(msg);
		return false;
  	}
}

void OffboardControl::send_velocity_command(double linear_x, double linear_y, double linear_z, double angular_z)
{
  geometry_msgs::msg::TwistStamped msg;
  msg.twist.linear.x = linear_x;
  msg.twist.linear.y = linear_y;
  msg.twist.linear.z = linear_z;
  msg.twist.angular.z = angular_z/2/PI;
  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";
  twist_stamped_publisher_->publish(msg);
}
