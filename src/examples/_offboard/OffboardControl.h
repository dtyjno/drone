// OffboardControl.h
#ifndef OFFBOARDCONTROL_H // 如果OFFBOARDCONTROL_H没有被定义
#define OFFBOARDCONTROL_H // 定义OFFBOARDCONTROL_H

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

enum class FlyState
{
	init,
	// request,
	takeoff,
	goto_shot_area,
	findtarget,
	goto_scout_area,
	scout,
	land,
	end,
	// 以下为老代码实现
	Goto_shotpoint, 
	Doshot,
	Goto_scoutpoint,
	Surround_see,
	Doland,
	//
	Print_Info,
} ;

// 将当前状态发布到currentstate 1=circle:shot/sco 2=h:land
inline int fly_state_to_int(FlyState state) {
  switch (state) {
    case FlyState::init: return 1;
    case FlyState::takeoff: return 1;
    case FlyState::goto_shot_area: return 1;
    case FlyState::findtarget: return 0;
    case FlyState::goto_scout_area: return 1;
    case FlyState::scout: return 3;
    case FlyState::land: return 4;
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

#include <chrono>

class Timer {
public:
    Timer() 
        : start_time_(std::chrono::steady_clock::now()), 
          allow_single_reset_(true) {}

		Timer(std::function<void()> callback) 
				: start_time_(std::chrono::steady_clock::now()), 
				allow_single_reset_(true) {
					callback();
				}
		
		Timer(std::function<void()> callback, bool allow_single_reset) 
		: start_time_(std::chrono::steady_clock::now()), 
			allow_single_reset_(allow_single_reset) {
				callback();
			}
		
		/// @brief  将 start_time_ 初始化为默认时间点： (elapsed()当前时间-start开始时间 > 任意时间)=true
		Timer(bool trigger_once)
		{
			if (trigger_once)
			{
				start_time_ = std::chrono::steady_clock::time_point(); // 默认时间点
			}
			else
			{
				start_time_ = std::chrono::steady_clock::now();
			}
		}
			
    /// @brief 无条件重置计时器
    void reset() {
        start_time_ = std::chrono::steady_clock::now();
    }

    /// @brief 单次重置（仅在允许状态下生效）
    void reset_once() {
        if (allow_single_reset_) {
            start_time_ = std::chrono::steady_clock::now();
            allow_single_reset_ = false;
        }
    }

    void reset_once(std::function<void()> callback) {
		if (allow_single_reset_) {
					callback();
					start_time_ = std::chrono::steady_clock::now();
					allow_single_reset_ = false;
			}
		}
    /// @brief 获取自计时开始经过的时间（秒）
    double elapsed() const {
        const auto end_time = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(end_time - start_time_).count();
    }

    /// @brief 启用单次重置功能
    void enable_single_reset() {
				start_time_ = std::chrono::steady_clock::now();
        allow_single_reset_ = true;
    }

		// 标记当前时间
		void set_timepoint(){
			time_point_ = std::chrono::steady_clock::now();
		}
		// 距上次标记经过时间（秒）
		double get_timepoint_elapsed(){
			const auto end_time = std::chrono::steady_clock::now();
			return std::chrono::duration<double>(end_time - time_point_).count();
		}

		void set_start_time_to_default(){
			// 将计时器的开始时间设置为默认时间点
			start_time_ = std::chrono::steady_clock::time_point();
		}
private:
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point time_point_;
		bool allow_single_reset_;
};

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
		state_machine_(*this)  // 显式初始化 state_machine_
	{
		// RCLCPP_INFO(this->get_logger(), "Starting Offboard Control example with PX4 services");
		RCLCPP_INFO(this->get_logger(), "开始使用APM服务的离线控制 OffboardControl");
		RCLCPP_INFO(this->get_logger(), "初始化 OffboardControl， ardupilot_namespace: %s", ardupilot_namespace.c_str());
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

	// 先旋转到飞机坐标系当前机头朝向角度
	template <typename T>
	void rotate2yaw(T x,T y,T &x_t, T &y_t) {
		const T cs = cos(get_yaw() - start.w());
		const T sn = sin(get_yaw() - start.w());
		T rx = x * cs - y * sn;
		T ry = x * sn + y * cs;
		x_t = rx;
		y_t = ry;
	}

	// 所有位置控制均旋转到世界坐标系的默认机头朝向
	template <typename T>
	void rotate2global(T &x,T &y) {
		// 顺时针旋转180度
		const T cs = cos(default_yaw + M_PI_2);
		const T sn = sin(default_yaw + M_PI_2);
		T rx = x * cs - y * sn;
		T ry = x * sn + y * cs;
		x = rx;
		y = ry;
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
	std::string ardupilot_namespace_copy_;
	std::shared_ptr<YOLO> _yolo;
	std::shared_ptr<ServoController> _servo_controller;
	std::shared_ptr<InertialNav> _inav;
	std::shared_ptr<Motors> _motors;
	std::shared_ptr<PosControl> _pose_control;

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
		{0.0, 0.0}, // 首次执行默认触发
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
		// {0.0, 1.5}, 
		// {1.5, 1}
		{1.0, 0.0}
	};
	vector<Vector2f> surround_see_points{
		{0.0, 0.0}, // 首次执行默认触发
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
	// uint8_t service_result_;
	// bool service_done_;

	// '''定义一个全局变量'''
	// float _rngfnd_distance;
	double timestamp_init = 0;
	// float heading=0;
	// const float default_heading=default_yaw;//初始偏转角

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

	// Vector4f start_temp{0,0,0,0};
	// Vector4f end_temp={0,0,0,0};
	// Vector3f end={0,0,0,0};
	// static Vector4f start;

	GlobalFrame start_global{0, 0, 0};
	// GlobalFrame start_global_temp{0,0,0};
	// GlobalFrame end_global_temp={0,0,0};
	// Vector3f end_global={0,0,0,0};

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

	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_publisher_;
  void publish_current_state();

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

	void read_configs(const std::string &filename)
	{
		YAML::Node config = Readyaml::readYAML(filename);
		headingangle_compass = config["headingangle_compass"].as<float>();
		// 1. 航向角转换：指南针角度 → 数学标准角度（东为0°，逆时针）
		default_yaw = fmod(90.0 - headingangle_compass + 360.0, 360.0);
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