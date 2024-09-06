// OffboardControl.h
#ifndef OFFBOARDCONTROL_H  // 如果OFFBOARDCONTROL_H没有被定义
#define OFFBOARDCONTROL_H  // 定义OFFBOARDCONTROL_H

#include <rclcpp/rclcpp.hpp>
// #include <stdint.h>
#include <chrono>
// #include <iostream>
// #include <string>
// #include <cmath>


#include "Yolo.h"
#include "ServoController.h"
#include "InertialNav.h"
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <sensor_msgs/msg/nav_sat_fix.hpp>
// #include <geometry_msgs/msg/twist_stamped.hpp>
// #include <mavros_msgs/msg/altitude.hpp>
#include "Motors.h"
#include "PoseControl.h"

// #include "math.h"
#include "memory"

// #include <Eigen/Eigen>
#include "math.h"
// #include "Vector3.h"
// #include "Vector4.h"
// #define DEFAULT_ACCURACY 0.3 //+-0.3m
// #define DEFAULT_ACCURACY_YAW 1 //+-1°
// #define DEFAULT_HEADING 90.00//角度
#define DEFAULT_X_POS FLT_MAX

using namespace std::chrono_literals;

#include "OffboardControl_Base.h"

class OffboardControl : public OffboardControl_Base{
public:
	OffboardControl(const std::string ardupilot_namespace,std::shared_ptr<YOLO> yolo_,std::shared_ptr<ServoController> servo_controller_) :
		OffboardControl_Base(ardupilot_namespace),
		ardupilot_namespace_copy_{ardupilot_namespace},
		_yolo{yolo_},
		_servo_controller{servo_controller_},
		_inav(std::make_shared<InertialNav>(ardupilot_namespace_copy_, this)),
        _motors(std::make_shared<Motors>(ardupilot_namespace_copy_,this)),
        _pose_control(std::make_shared<PoseControl>(ardupilot_namespace_copy_,this)),
		fly_state_{FlyState::init}//,
		// service_result_{0},
		// service_done_{false}
	{
		// RCLCPP_INFO(this->get_logger(), "Starting Offboard Control example with PX4 services");
		//RCLCPP_INFO_STREAM(geometry_msgs::msg::PoseStampedthis->get_logger(), "Waiting for " << px4_namespace << "vehicle_command service");
        InertialNav::position.x = DEFAULT_X_POS;
		Motors::home_position.x = DEFAULT_X_POS;
		// rclcpp::Rate rate(1s);
		while (!mode_switch_client_->wait_for_service(std::chrono::seconds(1))) {
			if (!rclcpp::ok()||is_equal(get_x_pos(),DEFAULT_X_POS)) {
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
				return;
			}
			RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
			// rate.sleep();
		}
		// std::thread t1(DaemonThread::run);
		// t1.detach();
		timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));
	}

	float get_x_pos(void){
		// return local_frame.x;
		return InertialNav::position.x;
	}
	float get_y_pos(void){
		// return local_frame.y;
		return InertialNav::position.y;
	}
	float get_z_pos(void){
		// return local_frame.z;
		return InertialNav::position.z;
	}
	Vector3f get_pos_3f(void){
		// return local_frame;
		return InertialNav::position;
	}
	Vector4f get_pos_4f(void){
		return {InertialNav::position.x,InertialNav::position.y,InertialNav::position.z,get_yaw()};
	}
	float get_x_vel(void){
		// return local_frame.x;
		return InertialNav::velocity.x;
	}
	float get_y_vel(void){
		// return local_frame.y;
		return InertialNav::velocity.y;
	}
	float get_z_vel(void){
		// return local_frame.z;
		return InertialNav::velocity.z;
	}
	float get_yaw_vel(void){
		// return local_frame.z;
		return InertialNav::velocity.yaw;
	}
	Vector3f get_vel_3f(void){
		// return local_frame;
		return {InertialNav::velocity.x,InertialNav::velocity.y,InertialNav::velocity.z};
	}
	Vector4f get_vel_4f(void){
		return InertialNav::velocity;
	}
	// float get_n_yaw(){
	// 	Eigen::Quaterniond quaternion;// 四元数
	// 	quaternion.w() = InertialNav::orientation.yaw;
	// 	quaternion.x() = InertialNav::orientation.x;
	// 	quaternion.y() = InertialNav::orientation.y;
	// 	quaternion.z() = InertialNav::orientation.z;
	// 	return quaternion.toRotationMatrix().eulerAngles(2, 1, 0).reverse()(2);
	// }
	// #define YAW_TOLERANCE 0.1
	float get_yaw(void){
		float w = InertialNav::orientation.yaw;
		float x = InertialNav::orientation.x;
		float y = InertialNav::orientation.y;
		float z = InertialNav::orientation.z;
		// 计算欧拉角
		float yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
		// float pitch = asin(2.0 * (w * y - z * x));
		// float roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
		return yaw;//弧度制
	}
	Vector3f get_gps(void){
		return InertialNav::gps;
	}
	float get_lat(void){
		return InertialNav::gps.x;
	}
	float get_lon(void){
		return InertialNav::gps.y;
	}
	float get_alt(void){
		return InertialNav::gps.z;
	}
	bool get_armed(void){
		return Motors::armed;
	}
	bool get_connected(void){
		return Motors::connected;
	}
	bool get_guided(void){
		return Motors::guided;
	}
	std::string get_mode(void){
		return Motors::mode;
	}
	std::string get_system_status(void){
		return Motors::system_status;
	}
	float get_z_home_pos(){
		return Motors::home_position.z;
	}
	float get_cur_time(void){
		return (this->get_clock()->now().nanoseconds() / 1000- timestamp_init)/1000000.0;
	}
private:
	std::string ardupilot_namespace_copy_;
	std::shared_ptr<YOLO> _yolo;
    std::shared_ptr<ServoController> _servo_controller;
    std::shared_ptr<InertialNav> _inav;
    std::shared_ptr<Motors> _motors;
    std::shared_ptr<PoseControl> _pose_control;

	enum class FlyState{
		init,
		//request,
		takeoff,
		goto_shot_area,
		findtarget,
		goto_scout_area,
		scout,
		land,
		end
	} fly_state_;
	class GlobalFrame{
	public:
		float lat;
		float lon;
		float alt;
	};
	// uint8_t service_result_;
	// bool service_done_;

	// '''定义一个全局变量'''
	// float _rngfnd_distance;
	float timestamp_init;
	// float heading=0;
	// const float default_heading=DEFAULT_YAW;//初始偏转角

	// std_msgs::msg::Header header;

	// Vector3f local_frame{DEFAULT_X_POS,0,0};
	// Vector4f velocity;
	// Vector4f position;
	// GlobalFrame global_frame;
    // // float heading;
    // Eigen::Quaterniond quaternion;// 四元数
    // Eigen::Vector3d euler;// 欧拉角
	// float yaw; //偏转角 弧度制

	// bool is_takeoff = false;

	// bool armed;
    // bool connected;
    // bool guided;
    // std::string mode;
    // std::string system_status;
	// Vector3f home_position{DEFAULT_X_POS,0,0};
	// Vector3f home_position_global;
	// Eigen::Quaterniond home_quaternion;// 四元数

	// int yaw_n = 0;

	Vector4f start{0,0,0,0};
	// Vector4f start_temp{0,0,0,0};
	// Vector4f end_temp={0,0,0,0};
	//Vector3f end={0,0,0,0};

	GlobalFrame start_global{0,0,0};
	// GlobalFrame start_global_temp{0,0,0};
	// GlobalFrame end_global_temp={0,0,0};
	//Vector3f end_global={0,0,0,0};

	
	// float _D_max = 2; //位置环最大速度
	// float _D = 1; //位置环比例系数

	// float k=0.002;//控制vx和vy
	// // 初始化PID控制器
	// float dt=0.1;
	// float kp = 0.41;  // 比例参数
	// float ki = 0.06;  // 积分参数
	// float kd = 0.28;  // 微分参数
	// float kp_yaw = 0.20;  // 比例参数
	// float ki_yaw = 0.04;  // 积分参数
	// float kd_yaw = 0.04;  // 微分参数
	// float max_vx=2; //前后方向最大速度
	// float max_vy=2; //左右方向最大速度
	// float max_vz=1; //上下方向最大速度
	// float max_yaw=10; //最大角速度(°/s)

	// GlobalFrame shot_area_start{};
	// GlobalFrame scout_area_start{};



	rclcpp::TimerBase::SharedPtr timer_;
	
	// void set_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	// void set_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
	// void set_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
	// void set_altitude(const mavros_msgs::msg::Altitude::SharedPtr msg);
	// void set_state(const mavros_msgs::msg::State::SharedPtr msg);
	// void set_home_position(const mavros_msgs::msg::HomePosition::SharedPtr msg);
	void set_pose();
	void set_gps();
	void set_velocity();
	void set_altitude();
	void set_state();
	void set_home_position();


	void timer_callback(void);
	void FlyState_init(void);

	//control.cpp
	bool catch_target_bucket(bool &result);
	bool surrounding_shot_area(void);
	bool surrounding_scout_area(void);
	bool trajectory_setpoint(float x,float y,float z,float yaw,double accuracy = DEFAULT_ACCURACY);
	bool trajectory_setpoint(float x,float y,float z,float yaw,PID::Defaults defaults,double accuracy = DEFAULT_ACCURACY);
	bool trajectory_setpoint_world(float x,float y,float z,float yaw,double accuracy = DEFAULT_ACCURACY);
	bool publish_setpoint_world(float x,float y,float z,float yaw,double accuracy = DEFAULT_ACCURACY);
};
#endif // OFFBOARDCONTROL_H