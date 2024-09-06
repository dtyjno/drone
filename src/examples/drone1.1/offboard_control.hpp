#ifndef OFFBOARD_CONTROL_H
#define OFFBOARD_CONTROL_H

#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol_local.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
//#include <ardupilot_msgs/msg/global_position.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <string>
#include "tools.hpp"

#define PI 3.14
//#define delta_heading -0.044156
//#define delta_heading -60  //yaw=-2.53
//M_PI dhaeding: 0.000000
#define DEFAULT_ACCURACY 0.3 //+-0.3m
//using namespace std::chrono;
using namespace std::chrono_literals;


class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl(std::string ardupilot_namespace);
    void switch_mode(std::string mode);
	void arm_motors(bool arm);
	//double quaternion_to_yaw(double x, double y, double z, double w);
	//void yaw_to_quaternion(double yaw, double* x, double* y, double* z, double* w);
private:
	enum class State{
		init,
		send_geo_grigin,
		wait_for_stable_offboard_mode,
		arm_requested,
		takeoff,
		autotune_mode,
		
	} state_;
	enum class FlyState{
		init,
		request,
		takeoff,
		goto_shot_area,
		findtarget,
		goto_scout_area,
		scout,
		land,
		end
	} fly_state_;
	std::string ardupilot_namespace_;
	uint8_t service_result_;
	bool service_done_;
	bool arm_done_;

	double k=0.002;//控制vx和vy
	// 初始化PID控制器
	double dt=0.1;
	double kp = 0.55;  // 比例参数
	double ki = 0.10;  // 积分参数
	double kd = 0.125;  // 微分参数
	double kp_yaw = 0.20;  // 比例参数
	double ki_yaw = 0.04;  // 积分参数
	double kd_yaw = 0.04;  // 微分参数
	double max_vx=2; //前后方向最大速度
	double max_vy=2; //左右方向最大速度
	double max_vz=2; //上下方向最大速度
	double max_yaw=10; //最大角速度(°/s)

	float delta_heading=60;//-2;

	double timestamp0;
	struct Point{
		public:
		float x;
		float y;
		float z;
		float yaw;
	};
	Point start = {0,0,0,0};
	Point start_temp= {0,0,0,0};
	Point end_temp= {0,0,0,0};
	Point end= {0,0,0,0};
	float heading;

	rclcpp::TimerBase::SharedPtr timer_;

	geometry_msgs::msg::PoseStamped pose_{};
	sensor_msgs::msg::NavSatFix global_gps_{};
	geometry_msgs::msg::TwistStamped velocity_{};
	//sensor_msgs::msg::NavSatFix global_gps_start{};
	//ardupilot_msgs::msg::GlobalPosition global_gps_start{};

	//rclcpp::Publisher<ardupilot_msgs::msg::GlobalPosition>::SharedPtr global_gps_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_publisher_;
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
	
	rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_motors_client_;
	rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_switch_client_;
	rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscription_;
	
	void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
	void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

	void switch_to_guided_mode();
	void switch_to_takeoff();
	void switch_to_rtl_mode();
	// void switch_to_auto_mode();
	//void publish_global_gps(double latitude, double longitude, double altitude);
	
	void command_takeoff_or_land_local(std::string mode);
	void command_takeoff_or_land(std::string mode);
	
	void trajectory_setpoint_takeoff(float x,float y ,float z ,float yaw);
	void trajectory_setpoint(float x,float y ,float z ,float yaw, float accuracy=DEFAULT_ACCURACY);
	void trajectory_setpoint_start(float x,float y ,float z ,float yaw,float accuracy=DEFAULT_ACCURACY);
	void publish_trajectory_setpoint(float x,float y ,float z ,float yaw);
	bool send_velocity_command_with_time(double linear_x, double linear_y, double linear_z, double angular_z, double time);
	void send_velocity_command(double linear_x, double linear_y, double linear_z, double angular_z);

	bool surrending_shot_area(void);
	bool surrending_scout_area(void);

	void set_target_point(std::string mode,float x,float y,float z,float yaw);
	void set_start_temp_point(float x,float y,float z,float yaw);
	void set_end_temp_point(float x,float y,float z,float yaw);
	void set_drone_target_point_local(float x,float y,float z,float yaw);
	void set_start_point_local(float x,float y,float z,float yaw);
	void set_world_point_local(float x,float y,float z,float yaw);
	bool at_check_point(float accuracy=DEFAULT_ACCURACY);	

	void get_target_location(float delta_heading, float* x, float* y);

	void timer_callback(void);
};
#endif // OFFBOARD_CONTROL_H

