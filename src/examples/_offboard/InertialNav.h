#ifndef INERTIALNAV_H
#define INERTIALNAV_H

#include "rclcpp/rclcpp.hpp"
#include "OffboardControl_Base.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/altitude.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "sensor_msgs/msg/range.hpp"

#include "math.h"
class InertialNav
{
public:
    InertialNav(const std::string ardupilot_namespace,OffboardControl_Base* node)
    {
		this->node = node;
		// RCLCPP_INFO(node->get_logger(), "Starting Inertial Navigation example");
        // 质量服务配置
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		// 声明回调组,实例化回调组，类型为：可重入的
		rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_= node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
		// Each of these callback groups is basically a thread
    	// Everything assigned to one of them gets bundled into the same thread
		auto sub_opt = rclcpp::SubscriptionOptions();
    	sub_opt.callback_group = callback_group_subscriber_;

		//ros2 topic echo /mavros/local_position/pose geometry_msgs/msg/PoseStamped
		// /mavros/local_position/odom
		pose_subscription_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(ardupilot_namespace+"local_position/pose", qos,
		std::bind(&InertialNav::pose_callback, this, std::placeholders::_1),sub_opt);
		//ros2 topic echo /mavros/global_position/global sensor_msgs/msg/NavSatFix
		gps_subscription_ = node->create_subscription<sensor_msgs::msg::NavSatFix>(ardupilot_namespace+"global_position/global", qos,
		std::bind(&InertialNav::gps_callback, this, std::placeholders::_1),sub_opt);
		velocity_subscription_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(ardupilot_namespace+"local_position/velocity_local", qos,
		std::bind(&InertialNav::velocity_callback, this, std::placeholders::_1),sub_opt);
		altitude_subscription_ = node->create_subscription<mavros_msgs::msg::Altitude>(ardupilot_namespace+"altitude", qos,
        std::bind(&InertialNav::altitude_callback, this, std::placeholders::_1),sub_opt);
		// /mavros/imu/data (sensor_msgs/Imu)
		imu_data_subscription_ = node->create_subscription<sensor_msgs::msg::Imu>(ardupilot_namespace+"imu/data", qos,
		std::bind(&InertialNav::imu_data_callback, this, std::placeholders::_1),sub_opt);
		// 激光雷达高度
		rangefinder_subscription_ = node->create_subscription<sensor_msgs::msg::Range>(ardupilot_namespace+"rangefinder/rangefinder", qos,
		std::bind(&InertialNav::rangefinder_callback, this, std::placeholders::_1),sub_opt);

    }

	// Vector4f get_position(){
	// 	return position;
	// }
	// float get_x(){
	// 	return position.x;
	// }
	// float get_y(){
	// 	return position.y;
	// }
	// float get_z(){
	// 	return position.z;
	// }
	// Vector4f get_velocity(){
	// 	return velocity;
	// }
	// float get_vx(){
	// 	return velocity.x;
	// }
	// float get_vy(){
	// 	return velocity.y;
	// }
	// float get_vz(){
	// 	return velocity.z;
	// }
	// Vector4f get_orientation(){
	// 	return orientation;
	// }
	// float get_ox(){
	// 	return orientation.x;
	// }
	// float get_oy(){
	// 	return orientation.y;
	// }
	// float get_oz(){
	// 	return orientation.z;
	// }
	// float get_ow(){
	// 	return orientation.yaw;
	// }
	// Vector4f get_gps(){
	// 	return gps;
	// }
	// float get_lat(){
	// 	return gps.x;
	// }
	// float get_lon(){
	// 	return gps.y;
	// }
	// float get_alt(){
	// 	return gps.z;
	// }
	// float get_amsl(){
	// 	return altitude;
	// }
	static Vector3f position;
	static Vector4f velocity;
	static Quaternionf orientation;
	static Vector3f gps;
	static float altitude;
	static Vector3f linear_acceleration;
	static Vector3f angular_velocity;
	static float rangefinder_height;

	// // pose_subscription_时间戳
	// static float timestamp_seconds;
	// static long timestamp_nanoseconds;

	static Vector3f get_position(){
		return position;
	}
	static bool get_position(Vector3f& position){
		position = InertialNav::position;
		return position.isZero();
	}
	static Vector3f get_velocity(){
		return {velocity.x(),velocity.y(),velocity.z()};
	}
	static bool get_forward_speed(float & velocity){
		Vector2f _velocity = {InertialNav::velocity.x(),InertialNav::velocity.y()};
		velocity = _velocity.norm(); // 长度
		return _velocity.isZero();	
	}
	static Vector2f groundspeed_vector(){
		return {velocity.x(),velocity.y()};
	}
	// float get_time(void){
	// 	return (node->get_clock()->now().nanoseconds() / 1000)/1000000.0;
	// }
private:
	OffboardControl_Base* node;
	
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
	rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscription_;
	rclcpp::Subscription<mavros_msgs::msg::Altitude>::SharedPtr altitude_subscription_;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_subscription_;
	rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr rangefinder_subscription_; // 激光雷达高度

	void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
	void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
	void altitude_callback(const mavros_msgs::msg::Altitude::SharedPtr msg);
	void imu_data_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
	void rangefinder_callback(const sensor_msgs::msg::Range::SharedPtr msg);
	
};
#endif // INERTIALNAV_H