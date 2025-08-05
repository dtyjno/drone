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
#include <yaml-cpp/yaml.h>
#include<fstream>
// #include <Eigen/Eigen>
#include "math.h"
// #include "Vector3.h"
// #include "Vector4.h"
// #define DEFAULT_ACCURACY 0.3 //+-0.3m
// #define DEFAULT_ACCURACY_YAW 1 //+-1°
// #define DEFAULT_HEADING 90.00//角度

#include "std_msgs/msg/int32.hpp"


using namespace std::chrono_literals;

#include "OffboardControl_Base.h"
#include "StateMachine.h"

#include <fstream>
#include <iostream>

#include "CameraGimbal.h"
#include "clustering.h"

#include "utils.h" // 包含自定义的工具函数

class OffboardControl : public OffboardControl_Base
{
	friend class StateMachine;
	
public:
	OffboardControl(const std::string ardupilot_namespace, std::shared_ptr<YOLO> yolo_) : OffboardControl_Base(ardupilot_namespace),
		ardupilot_namespace_copy_{ardupilot_namespace},
		_yolo{yolo_},
		_servo_controller{std::make_shared<ServoController>(ardupilot_namespace_copy_, this)},
		_inav(std::make_shared<InertialNav>(ardupilot_namespace_copy_, this)),
		_motors(std::make_shared<Motors>(ardupilot_namespace_copy_, this)),
		_pose_control(std::make_shared<PosControl>(ardupilot_namespace_copy_, this, _inav)),
		_camera_gimbal(std::make_shared<CameraGimbal>(ardupilot_namespace_copy_, this)),
		mypid(),
		state_machine_(this)  // 显式初始化 state_machine_
	{

		if (print_info_) {
			state_machine_.transition_to(FlyState::Print_Info);
		}

		// RCLCPP_INFO(this->get_logger(), "Starting Offboard Control example with PX4 services");
		RCLCPP_INFO(this->get_logger(), "开始使用APM服务的离线控制 OffboardControl");
		RCLCPP_INFO(this->get_logger(), "初始化 OffboardControl， ardupilot_namespace: %s", ardupilot_namespace.c_str());
		mypid.readPIDParameters("pos_config.yaml","mypid");

		// 读取罗盘数据
		read_configs("OffboardControl.yaml");
		// RCLCPP_INFO_STREAM(geometry_msgs::msg::PoseStampedthis->get_logger(), "Waiting for " << px4_namespace << "vehicle_command service");
		_inav->position.x() = DEFAULT_POS;
		_inav->position.z() = DEFAULT_POS;
		_motors->home_position.x() = DEFAULT_POS;
		// rclcpp::Rate rate(1s);
		
		// 发布当前状态 
		state_publisher_ = this->create_publisher<std_msgs::msg::Int32>("current_state", 10);

		if (!debug_mode_){
			while (!mode_switch_client_->wait_for_service(std::chrono::seconds(2)))
			{
				if (!rclcpp::ok())
				{
					RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. 停止");
					return;
				}
				RCLCPP_INFO(this->get_logger(), "模式切换服务未准备好, 正在等待...");
				// rate.sleep();
			}
			while (!_motors->get_set_home_client()->wait_for_service(std::chrono::seconds(2)))
			{
				if (!rclcpp::ok())
				{
					RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. 停止");
					return;
				}
				RCLCPP_INFO(this->get_logger(), "设置home位置服务未准备好, 正在等待...");
				// rate.sleep();
			}
		}
		
		timestamp_init = get_cur_time();
		_motors->switch_mode("GUIDED");
		timer_ = this->create_wall_timer(wait_time, std::bind(&OffboardControl::timer_callback, this));
		#ifdef PAL_STATISTIC_VISIBILITY
		stats_publisher_ = this->create_publisher<pal_statistics_msgs::msg::Statistics>("/statistics", 10);
		stats_timer_ = this->create_wall_timer(wait_time,std::bind(&OffboardControl::publish_statistics, this));
		#endif
	}

	float get_x_pos(void)
	{
		// return local_frame.x();
		return _inav->position.x();
	}
	float get_y_pos(void)
	{
		// return local_frame.y();
		return _inav->position.y();
	}
	float get_z_pos(void)
	{
		// return local_frame.z();
		return _inav->position.z() == DEFAULT_POS ? _inav->rangefinder_height : _inav->position.z();
	}
	Vector3f get_pos_3f(void)
	{
		// return local_frame;
		return _inav->position;
	}
	Vector4f get_pos_4f(void)
	{
		return {_inav->position.x(), _inav->position.y(), _inav->position.z(), get_yaw()};
	}
	float get_x_vel(void)
	{
		// return local_frame.x();
		return _inav->velocity.x();
	}
	float get_y_vel(void)
	{
		// return local_frame.y();
		return _inav->velocity.y();
	}
	float get_z_vel(void)
	{
		// return local_frame.z();
		return _inav->velocity.z();
	}
	float get_yaw_vel(void)
	{
		// return local_frame.z();
		return _inav->velocity.w();
	}
	Vector3f get_vel_3f(void)
	{
		// return local_frame;
		return {_inav->velocity.x(), _inav->velocity.y(), _inav->velocity.z()};
	}
	Vector4f get_vel_4f(void)
	{
		return _inav->velocity;
	}
	// float get_n_yaw(){
	// 	Eigen::Quaterniond quaternion;// 四元数
	// 	quaternion.w() = _inav->orientation.y()aw;
	// 	quaternion.x()() = _inav->orientation.x();
	// 	quaternion.y()() = _inav->orientation.y();
	// 	quaternion.z()() = _inav->orientation.z();
	// 	return quaternion.toRotationMatrix().eulerAngles(2, 1, 0).reverse()(2);
	// }
	// #define YAW_TOLERANCE 0.1
	// 角度标准化到 [-π, π]
	float normalize_angle(float angle){
		while (angle > M_PI) angle -= 2.0f * M_PI;
		while (angle < -M_PI) angle += 2.0f * M_PI;
		return angle;
	};
	float get_yaw(void)
	{
		return _inav->get_yaw();
	}
	float get_world_yaw(void)
	{
		return fmod(M_PI_2 - _inav->get_yaw() + 2 * M_PI, 2 * M_PI); // 将偏航角转换为世界坐标系下的角度
	}

	void get_euler(float &roll, float &pitch, float &yaw) {
		// 获取四元数分量
		float w = _inav->orientation.w();
		float x = _inav->orientation.x();
		float y = _inav->orientation.y();
		float z = _inav->orientation.z();
		
		// // 验证四元数有效性
		// float norm = sqrt(w*w + x*x + y*y + z*z);
		// if (fabs(norm - 1.0f) > 0.1f) {
		// 	RCLCPP_WARN(this->get_logger(), "四元数异常！norm=%.6f", norm);
		// 	// 标准化四元数
		// 	if (norm > 0.001f) {
		// 		w /= norm; x /= norm; y /= norm; z /= norm;
		// 	} else {
		// 		w = 1.0f; x = y = z = 0.0f; // 默认无旋转
		// 	}
		// }
		
		// 使用稳定的欧拉角计算方法
		// Roll (X轴旋转)
		float sinr_cosp = 2.0f * (w * x + y * z);
		float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
		roll = atan2(sinr_cosp, cosr_cosp);
		
		// Pitch (Y轴旋转) - 处理万向锁
		float sinp = 2.0f * (w * y - z * x);
		if (fabs(sinp) >= 1.0f) {
			pitch = copysign(M_PI / 2.0f, sinp);
		} else {
			pitch = asin(sinp);
		}
		
		// Yaw (Z轴旋转)
		float siny_cosp = 2.0f * (w * z + x * y);
		float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
		yaw = atan2(siny_cosp, cosy_cosp);
		
		roll = normalize_angle(roll);
		pitch = normalize_angle(pitch);
		yaw = normalize_angle(yaw);
		
		// 调试输出四元数信息
		// static int debug_count = 0;
		// if (++debug_count % 100 == 0) { // 每10秒输出一次
		// 	RCLCPP_INFO(this->get_logger(), 
		// 				"四元数调试: w=%.3f x=%.3f y=%.3f z=%.3f norm=%.6f", 
		// 				w, x, y, z, norm);
		// }
	}
	float get_yaw_eigen() {
    // 直接从四元数提取偏航角
    return _inav->orientation.toRotationMatrix().eulerAngles(2, 1, 0)[0];
	}
	Vector3f get_gps(void)
	{
		return _inav->gps;
	}
	float get_lat(void)
	{
		return _inav->gps.x();
	}
	float get_lon(void)
	{
		return _inav->gps.y();
	}
	float get_alt(void)
	{
		return _inav->gps.z();
	}
	float get_rangefinder_distance()
	{
		return _inav->rangefinder_height;
	}
	bool get_armed(void)
	{
		return _motors->armed;
	}
	bool get_connected(void)
	{
		return _motors->connected;
	}
	bool get_guided(void)
	{
		return _motors->guided;
	}
	std::string get_mode(void)
	{
		return _motors->mode;
	}
	std::string get_system_status(void)
	{
		return _motors->system_status;
	}
	float get_x_home_pos()
	{
		return _motors->home_position.x();
	}
	float get_y_home_pos()
	{
		return _motors->home_position.y();
	}	
	float get_z_home_pos()
	{
		return _motors->home_position.z();
	}
	// 获取当前时间
	double get_cur_time() {
		auto now = this->get_clock()->now();
		return now.seconds() - timestamp_init;  // 直接计算时间差并转为秒
	}

	// 顺时针旋转
	template <typename T>
	void rotate_global2stand(T in_x,T in_y, T &out_x, T &out_y) {
		rotate_angle(in_x, in_y, -headingangle_compass);
		out_x = in_x;
		out_y = in_y;
	}

	template <typename T>
	void rotate_stand2global(T in_x, T in_y, T &out_x, T &out_y) {
		rotate_angle(in_x, in_y, headingangle_compass);
		out_x = in_x;
		out_y = in_y;
	}

	template <typename T>
	void rotate_world2start(T in_x, T in_y, T &out_x, T &out_y) {
		rotate_angle(in_x, in_y, start.w());
		out_x = in_x;
		out_y = in_y;
	}
	
	template <typename T>
	void rotate_world2local(T in_x, T in_y, T &out_x, T &out_y) {
		// std::cout << "rotate_2local yaw: " << - (M_PI_2 - get_yaw()) << std::endl;
		rotate_angle(in_x, in_y, get_world_yaw()); // 使用 get_world_yaw() 获取世界坐标系下的偏航角
		out_x = in_x;
		out_y = in_y;
	}

	template <typename T>
	void rotate_local2world(T in_x, T in_y, T &out_x, T &out_y) {
		// std::cout << "rotate_2world yaw: " << get_world_yaw() << std::endl;
		rotate_angle(in_x, in_y, -get_world_yaw()); // 使用 get_world_yaw() 获取世界坐标系下的偏航角
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

#ifdef PAL_STATISTIC_VISIBILITY
	rclcpp::Publisher<pal_statistics_msgs::msg::Statistics>::SharedPtr stats_publisher_;
	rclcpp::TimerBase::SharedPtr stats_timer_;
	void publish_statistics(){
		if (!stats_publisher_) {
			RCLCPP_ERROR(this->get_logger(), "stats_publisher_ is not initialized.");
			return;
		}
		std::vector<pal_statistics_msgs::msg::Statistic> statistics;
		auto msg = pal_statistics_msgs::msg::Statistics();
		msg.header.stamp = this->get_clock()->now();
		msg.header.frame_id = "base_link";
		_pose_control->publish_statistic(statistics);
		this->publish_statistic(statistics);
		msg.statistics = statistics;
		this->stats_publisher_->publish(msg);
	}
	void publish_statistic(std::vector<pal_statistics_msgs::msg::Statistic> &statistics){
		auto current_state_stat = pal_statistics_msgs::msg::Statistic();
		current_state_stat.name ="Offboard_State";
		current_state_stat.value = static_cast<int>(state_machine_.get_current_state());
		statistics.push_back(current_state_stat);	
		
		auto yolo_x_circle_raw_stat = pal_statistics_msgs::msg::Statistic();
		yolo_x_circle_raw_stat.name ="yolo_x_circle_raw";
		yolo_x_circle_raw_stat.value = _yolo->get_raw_x(YOLO::TARGET_TYPE::CIRCLE);
		statistics.push_back(yolo_x_circle_raw_stat);	

		auto yolo_y_circle_raw_stat = pal_statistics_msgs::msg::Statistic();
		yolo_y_circle_raw_stat.name ="yolo_y_circle_raw";
		yolo_y_circle_raw_stat.value = _yolo->get_raw_y(YOLO::TARGET_TYPE::CIRCLE);
		statistics.push_back(yolo_y_circle_raw_stat);	

		auto yolo_x_circle_stat = pal_statistics_msgs::msg::Statistic();
		yolo_x_circle_stat.name ="yolo_x_circle";
		yolo_x_circle_stat.value = _yolo->get_x(YOLO::TARGET_TYPE::CIRCLE);
		statistics.push_back(yolo_x_circle_stat);	

		auto yolo_y_circle_stat = pal_statistics_msgs::msg::Statistic();
		yolo_y_circle_stat.name ="yolo_y_circle";
		yolo_y_circle_stat.value = _yolo->get_y(YOLO::TARGET_TYPE::CIRCLE);
		statistics.push_back(yolo_y_circle_stat);	

		auto yolo_x_h_raw_stat = pal_statistics_msgs::msg::Statistic();
		yolo_x_h_raw_stat.name ="yolo_x_h_raw";
		yolo_x_h_raw_stat.value = _yolo->get_raw_x(YOLO::TARGET_TYPE::H);
		statistics.push_back(yolo_x_h_raw_stat);	

		auto yolo_y_h_raw_stat = pal_statistics_msgs::msg::Statistic();
		yolo_y_h_raw_stat.name ="yolo_y_h_raw";
		yolo_y_h_raw_stat.value = _yolo->get_raw_y(YOLO::TARGET_TYPE::H);
		statistics.push_back(yolo_y_h_raw_stat);

		auto yolo_x_h_stat = pal_statistics_msgs::msg::Statistic();
		yolo_x_h_stat.name ="yolo_x_h";
		yolo_x_h_stat.value = _yolo->get_x(YOLO::TARGET_TYPE::H);
		statistics.push_back(yolo_x_h_stat);	

		auto yolo_y_h_stat = pal_statistics_msgs::msg::Statistic();
		yolo_y_h_stat.name ="yolo_y_h";
		yolo_y_h_stat.value = _yolo->get_y(YOLO::TARGET_TYPE::H);
		statistics.push_back(yolo_y_h_stat);

		auto tar_x_stat = pal_statistics_msgs::msg::Statistic();
		tar_x_stat.name = "tar_x_stat";
		tar_x_stat.value = target1.has_value() ? target1.value().x() : 0.0;
		statistics.push_back(tar_x_stat);

		auto tar_y_stat = pal_statistics_msgs::msg::Statistic();
		tar_y_stat.name = "tar_y_stat";
		tar_y_stat.value = target1.has_value() ? target1.value().y() : 0.0;
		statistics.push_back(tar_y_stat);

		for(size_t i = 0; i < 3; ++i) {
			auto point_stat = pal_statistics_msgs::msg::Statistic();
			point_stat.name = "surround_shot_point_" + std::to_string(i);
			point_stat.value = surround_shot_points[i].x();
			statistics.push_back(point_stat);
			point_stat.value = surround_shot_points[i].y();
			statistics.push_back(point_stat);
		}
	}
#endif
	std::optional<Vector3d> target1;
private:
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
	
	// 状态机
	StateMachine state_machine_;
  
	enum class DoshotState
	{
		doshot_init,
		doshot_scout,
		doshot_shot,
		doshot_wait,
		doshot_end
	} doshot_state_ = DoshotState::doshot_init;


	float default_yaw = 0.0; // 默认偏转角 = 450 - headingangle_compass 角度
	// headingangle_compass为罗盘读数 角度制
	float headingangle_compass = M_PI_2; // 默认罗盘读数，待读取

	// 投弹区域巡航属性
	const float shot_length_max = 8.0; //x方向，左右方向 
	const float shot_length = 7.0; //x方向，左右方向 
	const float shot_width_max = 5.0; //y方向向前
	const float shot_width = 5.0; //y方向，前后方向 符合直角坐标系
	// const float shot_halt = 4.0;

	// 侦查区域巡航属性
	const float see_length = 6.6;
	const float see_width = 4.8;
	// const double see_halt = 3.0;

	// 定义投弹侦察点位 原始数据
	float dx_shot, dy_shot;
	float dx_see, dy_see;
	float shot_halt;
	float shot_halt_low; // 投弹区预测目标低高度巡航
	float see_halt;
	// 坐标待旋转处理后坐标
	float tx_shot;
	float ty_shot;
	float tx_see;
	float ty_see;

	float bucket_height = 0.3; // 桶高度
	Vector3d drone_to_camera;

	float servo_open_position;
	float servo_close_position;

	vector<Circles> cal_center;

	// 定义航点
	vector<Vector2f> surround_shot_points{
		// {0.0, 1.0}, // 开始
		{0.16667, 0.66667}, 
		{-0.33333, 0.1}, 
		{0.16667, 0.33333},
		{-0.16667, 0.66667}, 
		{-0.16667, 0.33333}, 
		{0.0, 0.0},  
		{0.0, 1.0}, 
		{-0.33333, 0.9}, 
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
		// {0.0, 0.0}
	};

	class GlobalFrame
	{
	public:
		float lat;
		float lon;
		float alt;
	};

	double timestamp_init = 0;

	// GlobalFrame start_global{0, 0, 0};

	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_publisher_;

  void publish_current_state();

	rclcpp::TimerBase::SharedPtr timer_;

	std::chrono::milliseconds wait_time{50}; // 定时器间隔

	float get_wait_time() const {
		return wait_time.count() / 1000.0; // 返回秒数
	}

	// void set_pose();
	// void set_gps();
	// void set_velocity();
	// void set_altitude();
	// void set_state();
	// void set_home_position();

	void timer_callback(void);
	void FlyState_init(void);

	void read_configs(const std::string &filename)
	{
		YAML::Node config = Readyaml::readYAML(filename);
		headingangle_compass = config["headingangle_compass"].as<float>();
		// 1. 航向角转换：指南针角度 → 数学标准角度（东为0°，逆时针）
		// default_yaw = fmod(90.0 - headingangle_compass + 720.0, 360.0); // 确保角度在0到360度之间
		default_yaw = fmod(headingangle_compass + 360.0, 360.0); // 确保角度在0到360度之间
		RCLCPP_INFO(this->get_logger(), "读取罗盘角度: %f，默认旋转角：%f", headingangle_compass, default_yaw);
		headingangle_compass = headingangle_compass * M_PI / 180.0; // 弧度制
		default_yaw = M_PI/2 - default_yaw * M_PI / 180.0; // 弧度制
		dx_shot = config["dx_shot"].as<float>();
		dy_shot = config["dy_shot"].as<float>();
		dx_see = config["dx_see"].as<float>(); 
		dy_see = config["dy_see"].as<float>();
		shot_halt = config["shot_halt"].as<float>();
		shot_halt_low = config["shot_halt_low"].as<float>();
		see_halt = config["see_halt"].as<float>();
		drone_to_camera[0] = config["drone_to_camera_x"].as<float>();
		drone_to_camera[1] = config["drone_to_camera_y"].as<float>();
		drone_to_camera[2] = config["drone_to_camera_z"].as<float>();
		servo_open_position = config["servo_open_position"].as<float>();
		servo_close_position = config["servo_close_position"].as<float>();

		RCLCPP_INFO(this->get_logger(), "读取投弹区起点坐标: dx_shot: %f, dy_shot: %f shot_halt: %f", dx_shot, dy_shot, shot_halt);
		RCLCPP_INFO(this->get_logger(), "读取侦查区起点坐标: dx_see: %f, dy_see: %f see_halt: %f", dx_see, dy_see, see_halt);
		
		// tx_shot = dx_shot;
		// ty_shot = dy_shot;
		// tx_see = dx_see;
		// ty_see = dy_see;
	}
	// control.cpp
	Timer waypoint_timer_;         // 航点计时器
	Timer state_timer_;		       // 状态计时器
	bool is_first_run_=true;
	bool waypoint_goto_next(float x, float y, float length, float width, float halt, vector<Vector2f> &way_points, float time, int *count = nullptr, const std::string &description = "");
	// bool surround_shot_goto_next(double x, double y, double length, double width);
	// bool surround_see(double x, double y, double length, double width);
	bool catch_target(PID::Defaults defaults, enum YOLO::TARGET_TYPE target, float tar_x, float tar_y, float tar_z, float tar_yaw, float accuracy);
	bool Doland();
	// void PID_rtl(double now_x, double now_y, double now_z, double target_x, double target_y, bool &is_land);
	bool Doshot(int shot_count, bool &shot_flag);
	bool autotune(bool &result, enum YOLO::TARGET_TYPE target);
	bool surrounding_shot_area(void);
	bool surrounding_scout_area(void);
	void send_local_setpoint_command(float x, float y, float z, float yaw);
	bool local_setpoint_command(float x, float y, float z, float yaw, double accuracy);
	bool trajectory_setpoint(float x, float y, float z, float yaw, double accuracy = DEFAULT_ACCURACY);
	// bool trajectory_setpoint_world(float x, float y, float z, float yaw, PID::Defaults defaults, double accuracy = DEFAULT_ACCURACY);
	bool trajectory_setpoint_world(float x, float y, float z, float yaw, double accuracy = DEFAULT_ACCURACY);
	bool publish_setpoint_world(float x, float y, float z, float yaw, double accuracy = DEFAULT_ACCURACY);
	void send_velocity_command(float x, float y, float z, float yaw);
	bool send_velocity_command_with_time(float x, float y, float z, float yaw, double time);
	bool trajectory_circle(float a, float b, float height, float dt = 0.05, float yaw = 0);
	bool trajectory_generator_world(double speed_factor, std::array<double, 3> q_goal);
	bool trajectory_generator(double speed_factor, std::array<double, 3> q_goal);
	bool trajectory_generator_world_points(double speed_factor, const std::vector<std::array<double, 3>> &data, int data_length, Vector3f max_speed_xy, Vector3f max_accel_xy, float tar_yaw = 0.0);
};
#endif // OFFBOARDCONTROL_H
