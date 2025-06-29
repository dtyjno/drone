// OffboardControl.h
#ifndef OFFBOARDCONTROL_H // 如果OFFBOARDCONTROL_H没有被定义
#define OFFBOARDCONTROL_H // 定义OFFBOARDCONTROL_H

#include <rclcpp/rclcpp.hpp>
// #include <stdint.h>
#include <chrono>
// #include <iostream>
// #include <string>
// #include <cmath>
#include "MYPID.h"
#include "Yolo.h"
#include "ServoController.h"
#include "InertialNav.h"
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <sensor_msgs/msg/nav_sat_fix.hpp>
// #include <geometry_msgs/msg/twist_stamped.hpp>
// #include <mavros_msgs/msg/altitude.hpp>
#include "Motors.h"
#include "PosControl.h"

// #include "math.h"
#include "memory"
#include "Readyaml.h"
// #include <Eigen/Eigen>
#include "math.h"
// #include "Vector3.h"
// #include "Vector4.h"
// #define DEFAULT_ACCURACY 0.3 //+-0.3m
// #define DEFAULT_ACCURACY_YAW 1 //+-1°
// #define DEFAULT_HEADING 90.00//角度

#include "std_msgs/msg/int32.hpp"


#define DEFAULT_X_POS FLT_MAX

#define TRAIN_PID

using namespace std::chrono_literals;

#include "OffboardControl_Base.h"

#include <fstream>
#include <iostream>

#include "CameraGimbal.h"

#include "utils.h" // 包含自定义的工具函数

enum class FlyState
{
	init,
	// request,
	takeoff,
	end,
	Goto_shotpoint, 
	Doshot,
	Goto_scoutpoint,
	Surround_see,
	Doland,
	MYPID,
	Print_Info,
	Termial_Control, // 终端控制
	Reflush_config,
} ;

inline const std::map<std::string, FlyState> FlyStateMap = {
	{"INIT", FlyState::init},
	// {"REQUEST", FlyState::request},
	{"TAKEOFF", FlyState::takeoff},
	{"END", FlyState::end},
	{"GOTO_SHOTPOINT", FlyState::Goto_shotpoint},
	{"DOSHOT", FlyState::Doshot},
	{"GOTO_SCOUTPOINT", FlyState::Goto_scoutpoint},
	{"SURROUND_SEE", FlyState::Surround_see},
	{"DOLAND", FlyState::Doland},
	{"PRINT_INFO", FlyState::Print_Info},
	{"TERMINAL_CONTROL", FlyState::Termial_Control},
	{"REFLUSH_CONFIG", FlyState::Reflush_config},
};

// 将当前状态发布到currentstate 1=circle:shot/sco 2=h:land
inline int fly_state_to_int(FlyState state) {
  switch (state) {
    case FlyState::init: return 1;
    case FlyState::takeoff: return 1;
    case FlyState::end: return 2;
    case FlyState::Goto_shotpoint: return 1;
    case FlyState::Doshot: return 0;
    case FlyState::Goto_scoutpoint: return 1;
    case FlyState::Surround_see: return 3;
    case FlyState::Doland: return 4;
    case FlyState::Print_Info: return 1;
    default: return 1;
  }
}

class OffboardControl : public OffboardControl_Base
{
public:
	OffboardControl(const std::string ardupilot_namespace, std::shared_ptr<YOLO> yolo_) : OffboardControl_Base(ardupilot_namespace),
		ardupilot_namespace_copy_{ardupilot_namespace},
		_yolo{yolo_},
		_servo_controller{std::make_shared<ServoController>(ardupilot_namespace_copy_, this)},
		_inav(std::make_shared<InertialNav>(ardupilot_namespace_copy_, this)),
		_motors(std::make_shared<Motors>(ardupilot_namespace_copy_, this)),
		_pose_control(std::make_shared<PosControl>(ardupilot_namespace_copy_, this)),
		_camera_gimbal(std::make_shared<CameraGimbal>(ardupilot_namespace_copy_, this)),
		mypid(),
		state_machine_(*this)  // 显式初始化 state_machine_
	{
		// Declare and get parameters
		this->declare_parameter("sim_mode", false);
		this->get_parameter("sim_mode", sim_mode_);
		RCLCPP_INFO(this->get_logger(), "sim_mode: %s", sim_mode_ ? "true" : "false");
		this->declare_parameter("mode_switch", false);
		this->get_parameter("mode_switch", debug_mode_);
		RCLCPP_INFO(this->get_logger(), "mode_switch: %s", debug_mode_ ? "true" : "false");

		// RCLCPP_INFO(this->get_logger(), "Starting Offboard Control example with PX4 services");
		RCLCPP_INFO(this->get_logger(), "开始使用APM服务的离线控制 OffboardControl");
		RCLCPP_INFO(this->get_logger(), "初始化 OffboardControl， ardupilot_namespace: %s", ardupilot_namespace.c_str());
		mypid.readPIDParameters("pos_config.yaml","mypid");

		// 读取罗盘数据
		read_configs("OffboardControl.yaml");
		// RCLCPP_INFO_STREAM(geometry_msgs::msg::PoseStampedthis->get_logger(), "Waiting for " << px4_namespace << "vehicle_command service");
		InertialNav::position.x() = DEFAULT_X_POS;
		Motors::home_position.x() = DEFAULT_X_POS;
		// rclcpp::Rate rate(1s);
		
		// 发布当前状态 
		state_publisher_ = this->create_publisher<std_msgs::msg::Int32>("current_state", 10);

		while (!mode_switch_client_->wait_for_service(std::chrono::seconds(1)))
		{
			if (!rclcpp::ok() || is_equal(get_x_pos(), DEFAULT_X_POS))
			{
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. 停止");
				return;
			}
			RCLCPP_INFO(this->get_logger(), "模式切换服务未准备好, 正在等待...");
			// rate.sleep();
		}

		timestamp_init = get_cur_time();
		timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));
	}

	float get_x_pos(void)
	{
		// return local_frame.x();
		return InertialNav::position.x();
	}
	float get_y_pos(void)
	{
		// return local_frame.y();
		return InertialNav::position.y();
	}
	float get_z_pos(void)
	{
		// return local_frame.z();
		return InertialNav::position.z();
	}
	Vector3f get_pos_3f(void)
	{
		// return local_frame;
		return InertialNav::position;
	}
	Vector4f get_pos_4f(void)
	{
		return {InertialNav::position.x(), InertialNav::position.y(), InertialNav::position.z(), get_yaw()};
	}
	float get_x_vel(void)
	{
		// return local_frame.x();
		return InertialNav::velocity.x();
	}
	float get_y_vel(void)
	{
		// return local_frame.y();
		return InertialNav::velocity.y();
	}
	float get_z_vel(void)
	{
		// return local_frame.z();
		return InertialNav::velocity.z();
	}
	float get_yaw_vel(void)
	{
		// return local_frame.z();
		return InertialNav::velocity.w();
	}
	Vector3f get_vel_3f(void)
	{
		// return local_frame;
		return {InertialNav::velocity.x(), InertialNav::velocity.y(), InertialNav::velocity.z()};
	}
	Vector4f get_vel_4f(void)
	{
		return InertialNav::velocity;
	}
	// float get_n_yaw(){
	// 	Eigen::Quaterniond quaternion;// 四元数
	// 	quaternion.w() = InertialNav::orientation.y()aw;
	// 	quaternion.x()() = InertialNav::orientation.x();
	// 	quaternion.y()() = InertialNav::orientation.y();
	// 	quaternion.z()() = InertialNav::orientation.z();
	// 	return quaternion.toRotationMatrix().eulerAngles(2, 1, 0).reverse()(2);
	// }
	// #define YAW_TOLERANCE 0.1
	float get_yaw(void)
	{
		float w = InertialNav::orientation.w();
		float x = InertialNav::orientation.x();
		float y = InertialNav::orientation.y();
		float z = InertialNav::orientation.z();
		// 计算欧拉角
		float yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
		// float pitch = asin(2.0 * (w * y - z * x));
		// float roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
		return yaw; // 弧度制
	}
	float get_yaw_eigen() {
    // 直接从四元数提取偏航角
    return InertialNav::orientation.toRotationMatrix().eulerAngles(2, 1, 0)[0];
	}
	Vector3f get_gps(void)
	{
		return InertialNav::gps;
	}
	float get_lat(void)
	{
		return InertialNav::gps.x();
	}
	float get_lon(void)
	{
		return InertialNav::gps.y();
	}
	float get_alt(void)
	{
		return InertialNav::gps.z();
	}
	float get_rangefinder_distance()
	{
		return InertialNav::rangefinder_height;
	}
	bool get_armed(void)
	{
		return Motors::armed;
	}
	bool get_connected(void)
	{
		return Motors::connected;
	}
	bool get_guided(void)
	{
		return Motors::guided;
	}
	std::string get_mode(void)
	{
		return Motors::mode;
	}
	std::string get_system_status(void)
	{
		return Motors::system_status;
	}
	float get_z_home_pos()
	{
		return Motors::home_position.z();
	}
	// 获取当前时间
	double get_cur_time() {
		auto now = this->get_clock()->now();
		return now.seconds() - timestamp_init;  // 直接计算时间差并转为秒
	}

	// 顺时针旋转
	template <typename T>
	void rotate_global2stand(T in_x,T in_y, T &out_x, T &out_y) {
		rotate_angle(in_x, in_y, headingangle_compass);
		out_x = in_x;
		out_y = in_y;
	}

	template <typename T>
	void rotate_2start(T in_x, T in_y, T &out_x, T &out_y) {
		rotate_angle(in_x, in_y, -start.w());
		out_x = in_x;
		out_y = in_y;
	}
	
	template <typename T>
	void rotate_2local(T in_x, T in_y, T &out_x, T &out_y) {
		rotate_angle(in_x, in_y, -get_yaw());
		out_x = in_x;
		out_y = in_y;
	}

	int save_log(bool finish = false)
	{
		static bool is_first = true;
		static auto start_time = std::chrono::steady_clock::now();
		static std::ofstream outfile;
		if (is_first)
		{
			outfile = std::ofstream("log.csv");
			if (!outfile.is_open()) {
				std::cerr << "Failed to open file for writing" << std::endl;
				return 1;
			}
			is_first = false;
		}
		auto current_time = std::chrono::steady_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
		double pos_x = get_x_pos();
		double pos_y = get_y_pos();
		double pos_z = get_z_pos();
		outfile << duration.count() << "," << pos_x << "," << pos_y << "," << pos_z << std::endl;
		if (finish)
		{
			outfile.close();
			std::cout << "Log saved to log.csv" << std::endl;
		}
		return 0;
	}

private:
	bool sim_mode_ = false; // 是否为仿真模式
	bool debug_mode_ = false; // 是否手动切换状态
	std::string ardupilot_namespace_copy_;
	std::shared_ptr<YOLO> _yolo;
	std::shared_ptr<ServoController> _servo_controller;
	std::shared_ptr<InertialNav> _inav;
	std::shared_ptr<Motors> _motors;
	std::shared_ptr<PosControl> _pose_control;
	std::shared_ptr<CameraGimbal> _camera_gimbal;
	
	// --- 定义的PID函数---
	MYPID mypid;
	// ------------------
  

	class StateMachine {
	public:
		explicit StateMachine(OffboardControl& parent) 
			: parent_(parent), current_state_(FlyState::init) {}

		template<FlyState... States>
		void process_states();

		void add_dynamic_task(std::function<void()> task);
		void execute_dynamic_tasks();
		void transition_to(FlyState new_state);
	private:
		OffboardControl& parent_;
		FlyState current_state_;
		FlyState previous_state_;
		std::vector<std::function<void()>> dynamic_tasks_;
		std::mutex task_mutex_;

		template<FlyState S>
		void handle_state();

    // 声明 OffboardControl 为友元类
		friend class OffboardControl;
	};

	StateMachine state_machine_;



	float default_yaw = 0.0; // 默认偏转角 = 450 - headingangle_compass 角度
	// headingangle_compass为罗盘读数 角度制
	float headingangle_compass = M_PI_2; // 默认罗盘读数，待读取

	// 投弹区域巡航属性
	const double shot_length = 7.0; //x方向，左右方向 
	const double shot_width = 5.0; //y方向，前后方向 符合直角坐标系
	// const double shot_halt = 4.0;

	// 侦查区域巡航属性
	const double see_length = 6.0;
	const double see_width = 5.0;
	// const double see_halt = 3.0;

	// 定义投弹侦察点位
	float dx_shot, dy_shot;
	float dx_see, dy_see;
	float shot_halt;
	float see_halt;
	// 坐标待旋转
	float tx_shot;
	float ty_shot;
	float tx_see;
	float ty_see;

	// 定义航点
	vector<Vector2f> surround_shot_points{
		{0.0, 1.0}, // 开始
		{-0.16667, 0.66667}, 
		{-0.16667, 0.33333}, 
		{0.0, 0.0}, 
		{0.16667, 0.33333}, 
		{0.16667, 0.66667}, 
		{0.0, 1.0}, 
		{-0.33333, 0.9}, 
		{-0.33333, 0.1}, 
		{0.33333, 0.1}, 
		{0.33333, 0.9}, 
		{0.0, 0.9}, 
		{0.0, 0.5}
	};
	
	vector<Vector2f> surround_see_points{
		{0.0, 1.0}, 
		{-0.5, 0.0}, 
		{-0.5, 1.0}, 
		{0.5, 0.0}, 
		{0.5, 1.0}, 
		{0.0, 0.0}
	};

	class GlobalFrame
	{
	public:
		float lat;
		float lon;
		float alt;
	};

	double timestamp_init = 0;

	GlobalFrame start_global{0, 0, 0};

	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_publisher_;
  void publish_current_state();

	rclcpp::TimerBase::SharedPtr timer_;

	void set_pose();
	void set_gps();
	void set_velocity();
	void set_altitude();
	void set_state();
	void set_home_position();

	void timer_callback(void);
	void FlyState_init(void);

	void read_configs(const std::string &filename)
	{
		YAML::Node config = Readyaml::readYAML(filename);
		headingangle_compass = config["headingangle_compass"].as<float>();
		// 1. 航向角转换：指南针角度 → 数学标准角度（东为0°，逆时针）
		default_yaw = fmod(90.0 - headingangle_compass + 360.0, 360.0);
		headingangle_compass = fmod(360.0 - headingangle_compass, 360.0);
		RCLCPP_INFO(this->get_logger(), "读取罗盘角度: %f，默认旋转角：%f", headingangle_compass, default_yaw);
		headingangle_compass = headingangle_compass * M_PI / 180.0; // 弧度制
		default_yaw = default_yaw * M_PI / 180.0; // 弧度制
		dx_shot = config["dx_shot"].as<float>();
		dy_shot = config["dy_shot"].as<float>();
		dx_see = config["dx_see"].as<float>(); 
		dy_see = config["dy_see"].as<float>();
		shot_halt = config["shot_halt"].as<float>();
		see_halt = config["see_halt"].as<float>();
		RCLCPP_INFO(this->get_logger(), "读取投弹区起点坐标: dx_shot: %f, dy_shot: %f shot_halt: %f", dx_shot, dy_shot, shot_halt);
		RCLCPP_INFO(this->get_logger(), "读取侦查区起点坐标: dx_see: %f, dy_see: %f see_halt: %f", dx_see, dy_see, see_halt);
		
		tx_shot = dx_shot;
		ty_shot = dy_shot - 0.5;
		tx_see = dx_see;
		ty_see = dy_see;
	}
	// control.cpp
	Timer state_timer_;         // 通用计时器1
	bool waypoint_goto_next(double x, double y, double length, double width, double halt, vector<Vector2f> &way_points, double time, int *count = nullptr, const std::string &description = "");
	// bool surround_shot_goto_next(double x, double y, double length, double width);
	// bool surround_see(double x, double y, double length, double width);
	bool Doland();
	void PID_rtl(double now_x, double now_y, double now_z, double target_x, double target_y, bool &is_land);

	bool catch_target(bool &result, enum YOLO::TARGET_TYPE target);
	bool surrounding_shot_area(void);
	bool surrounding_scout_area(void);
	void send_local_setpoint_command(float x, float y, float z, float yaw);
	bool local_setpoint_command(float x, float y, float z, float yaw, double accuracy);
	bool trajectory_setpoint(float x, float y, float z, float yaw, double accuracy = DEFAULT_ACCURACY);
	bool trajectory_setpoint_world(float x, float y, float z, float yaw, PID::Defaults defaults, double accuracy = DEFAULT_ACCURACY);
	bool trajectory_setpoint_world(float x, float y, float z, float yaw, double accuracy = DEFAULT_ACCURACY);
	bool publish_setpoint_world(float x, float y, float z, float yaw, double accuracy = DEFAULT_ACCURACY);
	void send_velocity_command(float x, float y, float z, float yaw);
	bool send_velocity_command_with_time(float x, float y, float z, float yaw, double time);
	bool trajectory_circle(float a, float b, float height, float dt = 0.05, float yaw = 0);
	bool trajectory_generator_world(double speed_factor, std::array<double, 3> q_goal);
	bool trajectory_generator(double speed_factor, std::array<double, 3> q_goal);
	bool trajectory_generator_world_points(double speed_factor, const std::vector<std::array<double, 3>> &data, int data_length, bool init = false);
};
#endif // OFFBOARDCONTROL_H