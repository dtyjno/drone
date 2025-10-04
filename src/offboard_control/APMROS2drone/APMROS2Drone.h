#pragma once

#include <vector>
#include <memory>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include "../utils/Readyaml.h"
#include "../utils/math.h"
#include "../utils/utils.h"
#include "../ROS2drone/ROS2Drone.h"
#include "APMROS2PosController.h"
#include "APMROS2StatusController.h"
#include "APMROS2PosSubscriber.h"
#include "APMROS2PosPublisher.h"

// #include "../stateMachine/StateMachine.h"
#include "ROS2ServoController.h"
#include "ROS2CameraGimbal.h"
#include "ROS2YOLODetector.h"
// #include "../Yolo.h"
#include "../algorithm/clustering.h"
#include "../algorithm/pid/MYPID.h"
// #include "StatusController.h"
// #include "InertialNav.h"
// #include "PosController.h"
// #include "../CameraGimbal.h"
// #include "../ServoController.h"

// 前向声明
template <typename DroneType>
class StateMachine;

class APMROS2Drone : public ROS2Drone, public std::enable_shared_from_this<APMROS2Drone> {
	// friend class StateMachine;
public:
	static std::shared_ptr<APMROS2Drone> create(const std::string topic_namespace) {
		// Create PosSubscriber and PosPublisher
		auto node = rclcpp::Node::make_shared("offboard_control_node");
		std::shared_ptr<APMROS2PosPublisher> pos_publisher = std::make_shared<APMROS2PosPublisher>();
		std::shared_ptr<APMROS2PosSubscriber> pos_subscriber = std::make_shared<APMROS2PosSubscriber>();
		pos_publisher->CreateROS2Topics(node, topic_namespace);
		pos_subscriber->CreateROS2Topics(node, topic_namespace);
		auto sta_ctl = std::make_shared<APMROS2StatusController>();
		auto pos_ctl = std::make_shared<APMROS2PosController>(pos_subscriber, pos_publisher);
		pos_ctl->CreateROS2Topics(node, topic_namespace);
		sta_ctl->CreateROS2Topics(node, topic_namespace);
		auto instance = std::make_shared<APMROS2Drone>(topic_namespace, sta_ctl, pos_ctl, node);
		// 延迟初始化状态机，确保 shared_ptr 已经完全建立
		// instance->initializeStateMachine();
		instance->set_yolo_detector(std::make_shared<ROS2YOLODetector>(node));
		instance->set_servo_controller(std::make_shared<ROS2ServoController>(topic_namespace, node));
		instance->set_camera_gimbal(std::make_shared<ROS2CameraGimbal>(topic_namespace, node));
		pos_subscriber->add_observer(instance->get_camera().get()); // 订阅位置数据更新

		instance->read_configs("OffboardControl.yaml");
		
		return instance;
	}
	
    APMROS2Drone(const std::string& ardupilot_namespace,
                std::shared_ptr<StatusController> sta_ctl,
                std::shared_ptr<PosController> pos_ctl,
				std::shared_ptr<rclcpp::Node> node);

     ~APMROS2Drone() = default;
    
	std::shared_ptr<rclcpp::Node> get_node() {
        return node;
    }

	std::shared_ptr<APMROS2StatusController> get_status_controller() {
		return std::static_pointer_cast<APMROS2StatusController>(sta_ctl_);;
	}
	std::shared_ptr<APMROS2PosController> get_position_controller() {
		return std::static_pointer_cast<APMROS2PosController>(pos_ctl_);
	}

    // 重写 StateMachine 初始化
    // void initializeStateMachine();

	// 运行接口
	virtual void timer_callback(void);


    enum state {init, takeoff, gotoshot, shot, dropping, scout, land, end};
	
	int state_to_int(state s) {
		switch (s) {
			case init: return 1;
			case takeoff: return 1;
			case gotoshot: return 1;
			case shot: return 0;
			case dropping: return 0;
			case scout: return 1;
			case land: return 4;
			case end: return 2;
			default: return -1; // Invalid state
		}
	}

	// shot
	// int target_position_index = 0;     // 目标位置索引
	int shot_counter = 0;              // 投弹计数
	vector<size_t> shoted_cluster_ids; // 已投弹的目标索引

    // // 日志接口
    //  int save_log(bool finish = false) = 0;

	state current_state = init;

    // 发布接口
    void publish_current_state(int state);

	void set_wp_limits(PosController::Limits_t limits)
	{
		// pos_ctl->set_limits(limits);
		get_status_controller()->set_param("WPNAV_SPEED", limits.speed_max_xy * 100);
		get_status_controller()->set_param("WPNAV_SPEED_DN", limits.speed_max_z * 100);
	}

	void reset_wp_limits()
	{
		// pos_ctl->reset_limits();
		get_status_controller()->set_param("WPNAV_SPEED", get_position_controller()->limit_defaults.speed_max_xy * 100);
		get_status_controller()->set_param("WPNAV_SPEED_DN", get_position_controller()->limit_defaults.speed_max_z * 100);
		// RCLCPP_INFO(node->get_logger(), "Limits reset to defaults");
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
	// rclcpp::TimerBase::SharedPtr stats_timer_;
	void publish_statistics();
	void publish_statistic(std::vector<pal_statistics_msgs::msg::Statistic> &statistics);

#endif
	std::optional<Vector3d> target1;
	// --- 定义的PID函数---
	MYPID mypid;
	// ------------------
	


	float default_yaw = 0.0; // 默认偏转角 = 450 - headingangle_compass 角度
	// headingangle_compass为罗盘读数 角度制
	float headingangle_compass; // 默认罗盘读数，待读取
	float headingangle_real;

	// 投弹区域巡航属性
	const float shot_length_max = 8.0; //x方向，左右方向 
	const float shot_length = 8.0; //x方向，左右方向 
	const float shot_width_max = 5.0; //y方向向前
	const float shot_width = 5.0; //y方向，前后方向 符合直角坐标系
	// const float shot_halt = 4.0;

	// 侦查区域巡航属性
	const float see_length = 8.0;
	const float see_width = 5.0;
	// const double see_halt = 3.0;

	// 定义投弹侦察点位 原始数据
	float dx_shot, dy_shot;
	float dx_see, dy_see;
	float shot_halt;
	float shot_halt_surround; // 投弹区巡航时，周围巡航高度
	float shot_halt_low; // 投弹区预测目标低高度巡航
	float see_halt;
	bool shot_big_target; // 是否投放大目标
	// 坐标待旋转处理后坐标
	float tx_shot;
	float ty_shot;
	float tx_see;
	float ty_see;

	float bucket_height = 0.3; // 桶高度

	float servo_open_position;
	float servo_close_position;

	vector<Circles> cal_center;

	// 定义航点
	vector<Vector2f> surround_shot_points{
		// {0.0, 1.0}, // 开始
		{0.33333, 0.9}, 
		{0.0, 0.9}, 
		{0.0, 0.5},
		{0.16667, 0.66667}, 
		{-0.33333, 0.1}, 
		{0.16667, 0.33333},
		{-0.16667, 0.66667}, 
		{-0.16667, 0.33333}, 
		{0.0, 0.0},  
		{0.0, 1.0}, 
		{-0.33333, 0.9}, 
		{0.33333, 0.1}
	};
	
	vector<Vector2f> surround_see_points{
		{0.0, 1.0}, 
		{-0.5, 0.0}, 
		{-0.5, 1.0}, 
		{0.5, 0.0}, 
		{0.5, 1.0}, 
		// {0.0, 0.0}
	};

	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_publisher_;

	void FlyState_init(void);

	Timer waypoint_timer_;         // 航点计时器
	Timer state_timer_;		       // 状态计时器
	bool is_first_run_=true;

	// control.cpp
	bool waypoint_goto_next(float x, float y, float length, float width, float halt, vector<Vector2f> &way_points, float time, int *count = nullptr, const std::string &description = "");
	// bool surround_shot_goto_next(double x, double y, double length, double width);
	// bool surround_see(double x, double y, double length, double width);
	bool catch_target(PID::Defaults defaults, enum YOLO_TARGET_TYPE target, float tar_x, float tar_y, float tar_z, float tar_yaw, float accuracy);
	bool Doland();
	// void PID_rtl(double now_x, double now_y, double now_z, double target_x, double target_y, bool &is_land);
	bool Doshot(int shot_count, bool &shot_flag);
	bool autotune(bool &result, enum YOLO_TARGET_TYPE target);
	bool surrounding_shot_area(void);
	bool surrounding_scout_area(void);
	void terminal_control();
	void calculate_target_position();
	std::vector<Circles> Target_Samples;// 全局变量，存储目标样本点

	void read_configs(const std::string &filename)
	{
		YAML::Node config = Readyaml::readYAML(filename);
		try {
			std::cout << "读取配置文件: " << filename << std::endl;
			
			dx_shot = config["dx_shot"].as<float>();
			dy_shot = config["dy_shot"].as<float>();
			dx_see = config["dx_see"].as<float>(); 
			dy_see = config["dy_see"].as<float>();
			shot_halt = config["shot_halt"].as<float>();
			shot_halt_surround = config["shot_halt_surround"].as<float>();
			shot_halt_low = config["shot_halt_low"].as<float>();
			see_halt = config["see_halt"].as<float>();

			shot_big_target = config["shot_big_target"].as<bool>(true);

			RCLCPP_INFO(node->get_logger(), "读取投弹区起点坐标: dx_shot: %f, dy_shot: %f shot_halt: %f", dx_shot, dy_shot, shot_halt);
			RCLCPP_INFO(node->get_logger(), "读取侦查区起点坐标: dx_see: %f, dy_see: %f see_halt: %f", dx_see, dy_see, see_halt);
		} catch (const YAML::Exception &e) {
			RCLCPP_ERROR(node->get_logger(), "读取配置文件时发生错误: %s", e.what());
			return;
		}
		// tx_shot = dx_shot;
		// ty_shot = dy_shot;
		// tx_see = dx_see;
		// ty_see = dy_see;
	}

    // 访问接口
    void accept(std::shared_ptr<TaskBase> visitor);

protected:
	// StateMachine<APMROS2Drone>& state_machine_;
// private:
// 	std::shared_ptr<YOLODetector> yolo_detector;
// 	std::shared_ptr<ServoController> _servo_controller;
// 	// std::shared_ptr<Motors> sta_ctl.
// 	// std::shared_ptr<InertialNav> pos_ctl;
// 	// std::shared_ptr<PosControl> pos_ctl;
// 	std::shared_ptr<CameraGimbal> _camera_gimbal;

};