#include "OffboardControl.h"
#include <iostream>
#include <ncurses.h>
#include <unistd.h>

void OffboardControl::timer_callback(void){
	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "--------timer_callback----------");
	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "x:   %f, y:  %f, z: %f", get_x_pos(), get_y_pos(), get_z_pos());
	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "vx:  %f, vy: %f, vz: %f", get_x_vel(), get_y_vel(), get_z_vel());
	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "yaw: %f", get_yaw());
	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "yaw_vel: %f", get_yaw_vel());
	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "lat: %f, lon: %f, alt: %f", get_lat(), get_lon(), get_alt());
	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "armed:     %d", get_armed());
	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "connected: %d", get_connected());
	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "guided:    %d", get_guided());
	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "mode:      %s", get_mode().c_str());
	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "system_status:  %s", get_system_status().c_str());
	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "z_home_pos:     %f", get_z_home_pos());
	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "cur_time:       %f", get_cur_time());
}