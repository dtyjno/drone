#pragma once

#include <vector>
#include <memory>
#include <chrono>

// 前向声明
class DeviceVisitor;
class TaskBase;
class ROS2StatusController;
class ROS2PosController;

#include "../utils/Readyaml.h"
#include "../utils/math.h"
#include "../utils/utils.h"
#include "StatusController.h"
#include "PosController.h"
#include "PosSubscriber.h"
#include "PosPublisher.h"
// module
#include "CameraGimbal.h"
#include "ServoController.h"
#include "YOLODetector.h"

class AbstractDrone {
public:
    // 默认构造函数
    AbstractDrone()
	{
		debug_mode_ = true;
		if (!debug_mode_)
	        std::cout << "AbstractDrone: Starting Drone example" << std::endl;
        read_default_yaw_configs("Drone.yaml");
		auto pos_subscriber = std::make_shared<PosSubscriber>();
		auto pos_publisher = std::make_shared<PosPublisher>(pos_subscriber, get_wait_time());
		if (!debug_mode_)
			std::cout << "AbstractDrone: Setting up controllers" << std::endl;
		sta_ctl_ = std::make_shared<StatusController>();
		pos_ctl_ = std::make_shared<PosController>(pos_subscriber, pos_publisher);
		pos_ctl_->set_dt(get_wait_time()); // 设置控制器的时间间隔
		set_camera_gimbal(std::make_shared<CameraGimbal>());
		set_servo_controller(std::make_shared<ServoController>());
		set_yolo_detector(std::make_shared<YOLODetector>());
		pos_subscriber->set_position(Vector3f{0.0, 0.0, 2.0}); // 初始位置设为(0,0,2)
    }

    AbstractDrone(std::shared_ptr<StatusController> sta_ctl, std::shared_ptr<PosController> pos_ctl)
        : sta_ctl_(sta_ctl),
          pos_ctl_(pos_ctl)
	{
		pos_ctl_->set_dt(get_wait_time()); // 设置控制器的时间间隔
        std::cout << "AbstractDrone: Starting Drone example and setting up controllers" << std::endl;
        read_default_yaw_configs("OffboardControl.yaml");
    }
    
    // 初始化方法，用于在构造后设置控制器
    void initialize_controllers(std::shared_ptr<StatusController> sta_ctl, std::shared_ptr<PosController> pos_ctl) {
        std::cout << "AbstractDrone: Setting up controllers" << std::endl;
		pos_ctl_->set_dt(get_wait_time()); // 设置控制器的时间间隔
        sta_ctl_ = sta_ctl;
        pos_ctl_ = pos_ctl;
    }
    
    ~AbstractDrone() = default;

    // 位置稳定后设置初始位置
    void init_startposition() {
        start = {get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()}; 
    }

	void timer_callback_update() {
		// 更新相机位置和方向
		if (debug_mode_ && get_position_controller() && get_position_controller()->get_pos_data()) { // 调试模式下，强制设置飞机位置
			get_position_controller()->get_pos_data()->set_position(Vector3f(0.0, 0.0, 1.2)); // 设置飞机初始位置为(0,0,1.2)
		}
		float roll, pitch, yaw;
		get_euler(roll, pitch, yaw);
		if (debug_mode_ && get_status_controller()) {
			roll = 0.0f;
			pitch = 0.0f;
			// std::cout << "ENU 0E->N yaw" << get_yaw() << "NED 0N->E world_yaw" << get_world_yaw() << std::endl; // 默认飞机方向为正东 yaw=0,world_yaw=90 ,北 yaw 90 world_yaw 0.0
		}
		_camera_gimbal->set_parent_position(Vector3d(get_x_pos(), get_y_pos(), get_z_pos()));
		_camera_gimbal->set_camera_relative_rotation(Vector3d(0, 0, 0)); // 相机相对飞机的旋转，roll=0, pitch=0 (垂直向下), yaw=0
		// std::cout << "ENU 0E->N yaw" << get_yaw() << "NED 0N->E world_yaw" << get_world_yaw() << "headingangle_compass: " << headingangle_compass << std::endl; // 默认飞机方向为正东 世界方向 yaw=0,world_yaw=90 ,北 yaw 90 world_yaw 0.0
		_camera_gimbal->set_parent_rotation(Vector3d(-pitch, roll, get_world_yaw()));
		if (debug_mode_) {
			std::cout << "相机位置: (" << _camera_gimbal->get_position().transpose() << ")" << std::endl;
			std::cout << "相机旋转: (" << _camera_gimbal->get_parent_rotation().transpose() << ")" << std::endl;
		}
	}

	// 获取YOLO检测器
	std::shared_ptr<YOLODetectorInterface> get_yolo_detector() {
		return yolo_detector;
	}
	void set_yolo_detector(std::shared_ptr<YOLODetectorInterface> yolo_detector)
	{
		this->yolo_detector = yolo_detector;
	}
	// 获取舵机控制器
	std::shared_ptr<ServoControllerInterface> get_servo_controller() {
		return _servo_controller;
	}
	void set_servo_controller(std::shared_ptr<ServoControllerInterface> servo_controller)
	{
		this->_servo_controller = servo_controller;
	}
	// 获取云台控制器
	std::shared_ptr<CameraInterface> get_camera() {
		return _camera_gimbal;
	}
	void set_camera(std::shared_ptr<CameraInterface> camera_gimbal)
	{
		this->_camera_gimbal = camera_gimbal;
	}
	std::shared_ptr<CameraGimbalInterface> get_camera_gimbal() {
		auto camera_gimbal = dynamic_pointer_cast<CameraGimbalInterface>(_camera_gimbal);
		if (!camera_gimbal) {
			throw std::runtime_error("CameraGimbalInterface is not set or invalid.");
		}
		return camera_gimbal;
	}
	void set_camera_gimbal(std::shared_ptr<CameraGimbalInterface> camera_gimbal) {
		this->_camera_gimbal = std::dynamic_pointer_cast<CameraInterface>(camera_gimbal);
	}

	std::shared_ptr<StatusController> get_status_controller() {
		return sta_ctl_;
	}
	std::shared_ptr<PosController> get_position_controller() {
		return pos_ctl_;
	}

    // 位置相关接口
    float get_x_pos() {
        return pos_ctl_->pos_data->get_position().x();
    }
    float get_y_pos() {
        return pos_ctl_->pos_data->get_position().y();
    }
    float get_z_pos() {
        return pos_ctl_->pos_data->get_position().z();
    }
    Vector3f get_pos_3f() {
        return pos_ctl_->pos_data->get_position();
    }
    Vector4f get_pos_4f() {
        return {pos_ctl_->pos_data->get_position().x(), pos_ctl_->pos_data->get_position().y(), pos_ctl_->pos_data->get_position().z(), get_yaw()};
    }

    // 速度相关接口
    float get_x_vel() {
        return pos_ctl_->pos_data->get_velocity().x();
    }
    float get_y_vel() {
        return pos_ctl_->pos_data->get_velocity().y();
    }
    float get_z_vel() {
        return pos_ctl_->pos_data->get_velocity().z();
    }
    float get_yaw_vel() {
        return pos_ctl_->pos_data->get_velocity_yaw();
    }
    Vector3f get_vel_3f() {
        return {pos_ctl_->pos_data->get_velocity().x(), pos_ctl_->pos_data->get_velocity().y(), pos_ctl_->pos_data->get_velocity().z()};
    }
    Vector4f get_vel_4f() {
        return {pos_ctl_->pos_data->get_velocity().x(), pos_ctl_->pos_data->get_velocity().y(), pos_ctl_->pos_data->get_velocity().z(), pos_ctl_->pos_data->get_velocity_yaw()};
    }

    // 姿态相关接口

	// 角度标准化到 [-π, π]
	float normalize_angle(float angle){
		while (angle > M_PI) angle -= 2.0f * M_PI;
		while (angle < -M_PI) angle += 2.0f * M_PI;
		return angle;
	};
	
	void get_euler(float &roll, float &pitch, float &yaw) {
		// 获取四元数分量
		float w = pos_ctl_->pos_data->get_orientation().w();
		float x = pos_ctl_->pos_data->get_orientation().x();
		float y = pos_ctl_->pos_data->get_orientation().y();
		float z = pos_ctl_->pos_data->get_orientation().z();

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

    float get_yaw() {
        return pos_ctl_->pos_data->get_yaw();
    }
    float get_world_yaw() {
        return fmod(M_PI_2 - pos_ctl_->pos_data->get_yaw() + 2 * M_PI, 2 * M_PI); // 将偏航角转换为世界坐标系下的角度
    }
    float get_yaw_eigen() {
        return pos_ctl_->pos_data->get_orientation().toRotationMatrix().eulerAngles(2, 1, 0)[0];
    }
	float get_default_yaw() {
		return default_yaw;
	}
	float get_default_world_yaw() {
		return headingangle_compass;
	}

    // GPS相关接口
    Vector3f get_gps(void){
        return pos_ctl_->pos_data->get_gps();
    }
	float get_lat(void){
        return pos_ctl_->pos_data->get_gps().x();
    }
	float get_lon(void){
        return pos_ctl_->pos_data->get_gps().y();
    }
	float get_alt(void){
        return pos_ctl_->pos_data->get_gps().z();
    }
    // 传感器接口
	float get_rangefinder_distance(){
        return pos_ctl_->pos_data->get_rangefinder_height();
    }

    // 状态相关接口
	bool get_armed(void){
        return sta_ctl_->armed;
    }
	bool get_connected(void){
        return sta_ctl_->connected;
    }
	bool get_guided(void){
        return sta_ctl_->guided;
    }
	std::string get_mode(void){
        return sta_ctl_->get_mode();
    }
	uint8_t get_system_status(void){
        return sta_ctl_->get_system_status_uint8_t();
    }

    // Home位置接口
    float get_x_home_pos(void){
        return sta_ctl_->home_position.x() > 10000 || sta_ctl_->home_position.x() < -10000 ? 0 : sta_ctl_->home_position.x();
    }
	float get_y_home_pos(void){
        return sta_ctl_->home_position.y();
    }
	float get_z_home_pos(void){
        return sta_ctl_->home_position.z();
    }
    
    // 坐标变换接口

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
	void rotate_realglobal2stand(T in_x,T in_y, T &out_x, T &out_y) {
		rotate_angle(in_x, in_y, -headingangle_real);
		out_x = in_x;
		out_y = in_y;
	}

	template <typename T>
	void rotate_realstand2global(T in_x, T in_y, T &out_x, T &out_y) {
		rotate_angle(in_x, in_y, headingangle_real);
		out_x = in_x;
		out_y = in_y;
	}

	template <typename T>
	void rotate_world2start(T in_x, T in_y, T &out_x, T &out_y) {
		rotate_angle(in_x, in_y, -start.w());
		out_x = in_x;
		out_y = in_y;
	}
	
	template <typename T>
	void rotate_world2local(T in_x, T in_y, T &out_x, T &out_y) {
		rotate_angle(in_x, in_y, -get_world_yaw()); // 使用 get_world_yaw() 获取世界坐标系下的偏航角
		out_x = in_x;
		out_y = in_y;
	}

	template <typename T>
	void rotate_local2world(T in_x, T in_y, T &out_x, T &out_y) {
		// std::cout << "rotate_2world yaw: " << get_world_yaw() << std::endl;
		rotate_angle(in_x, in_y, get_world_yaw()); // 使用 get_world_yaw() 获取世界坐标系下的偏航角
		out_x = in_x;
		out_y = in_y;
	}

    // // 日志接口


    // 计时器相关接口
	double get_cur_time() {
		return 0.0;  // 返回当前时间戳
	};
	double get_start_time() {
		return start_time;  // 返回开始时间
	}
	void set_start_time(double time) {
		start_time = time;  // 设置开始时间
	}

	float get_wait_time() const {
		return wait_time.count() / 1000.0; // 返回秒数
	}
    // 运动控制接口
	bool is_equal_start_target_xy(float x, float y, double accuracy = 0.5);
	bool is_equal_local_target_xy(float x, float y, double accuracy = 0.5);
    virtual void send_start_setpoint_command(float x, float y, float z, float yaw);
	void send_local_setpoint_command(float x, float y, float z, float yaw);
    void send_world_setpoint_command(float x, float y, float z, float yaw);
    bool local_setpoint_command(float x, float y, float z, float yaw, double accuracy);
    bool trajectory_setpoint(float x, float y, float z, float yaw, double accuracy = 0.5);
    bool trajectory_setpoint_world(float x, float y, float z, float yaw, double accuracy = 0.5);
    bool publish_setpoint_world(float x, float y, float z, float yaw, double accuracy = 0.5);
    void send_velocity_command(float x, float y, float z, float yaw);
    bool send_velocity_command_with_time(float x, float y, float z, float yaw, double time);
    bool trajectory_circle(float a, float b, float height, float dt = 0.05, float yaw = 0);
    bool trajectory_generator_world(double speed_factor, std::array<double, 3> q_goal);
    bool trajectory_generator(double speed_factor, std::array<double, 3> q_goal);
    bool trajectory_generator_world_points(double speed_factor, 
                                                  const std::vector<std::array<double, 3>> &data, 
                                                  int data_length, Vector3f max_speed_xy, 
                                                  Vector3f max_accel_xy, float tar_yaw = 0.0);
		// 使用输出方法
    template<typename ... Args>
    void log_info(const Args&... args) {
        std::ostringstream oss;
        (oss << ... << args);
        std::cout << "[INFO] " << oss.str() << std::endl;
    }
    template<typename ... Args>
    void log_warn(const Args&... args) {
        std::ostringstream oss;
        (oss << ... << args);
        std::cout << "[WARN] " << oss.str() << std::endl;
    }
    template<typename ... Args>
    void log_error(const Args&... args) {
        std::ostringstream oss;
        (oss << ... << args);
        std::cerr << "[ERROR] " << oss.str() << std::endl;
    }
    template<typename ... Args>
    void log_debug(const Args&... args) {
        std::ostringstream oss;
        (oss << ... << args);
        std::cout << "[DEBUG] " << oss.str() << std::endl;
    }
    template<typename ... Args>
    void log_debug_throttle(const std::chrono::milliseconds& wait_time, const Args&... args) {
        std::ostringstream oss;
        (oss << ... << args);
        std::cout << "[DEBUG_THROTTLE] " << oss.str() << " (interval: " << wait_time.count() << "ms)" << std::endl;
    }
    template<typename ... Args>
    void log_info_throttle(const std::chrono::milliseconds& wait_time, const Args&... args) {
        std::ostringstream oss;
        (oss << ... << args);
        std::cout << "[INFO_THROTTLE] " << oss.str() << " (interval: " << wait_time.count() << "ms)" << std::endl;
    }
    template<typename ... Args>
    void log_warn_throttle(const std::chrono::milliseconds& wait_time, const Args&... args) {
        std::ostringstream oss;
        (oss << ... << args);
        std::cout << "[WARN_THROTTLE] " << oss.str() << " (interval: " << wait_time.count() << "ms)" << std::endl;
    }
    template<typename ... Args>
    void log_error_throttle(const std::chrono::milliseconds& wait_time, const Args&... args) {
        std::ostringstream oss;
        (oss << ... << args);
        std::cerr << "[ERROR_THROTTLE] " << oss.str() << " (interval: " << wait_time.count() << "ms)" << std::endl;
    }
	std::chrono::milliseconds wait_time{50}; // 定时器间隔

	void accept(std::shared_ptr<TaskBase> visitor);

	// 参数接口
	bool sim_mode_ = false; // 是否为仿真模式
	bool debug_mode_ = false; // 是否验证状态
	bool print_info_ = false; // 是否打印信息
	bool fast_mode_ = false; // 是否快速模式
	// 配置读取接口
	void read_default_yaw_configs(const std::string &filename)
	{
		if (!debug_mode_)
			std::cout << "读取配置文件: " << filename << std::endl;
		 // 读取配置文件
		YAML::Node config = Readyaml::readYAML(filename);
		try {
			headingangle_compass = config["headingangle_compass"].as<float>(180.0); // 默认罗盘角度 ENU
			headingangle_real = config["headingangle_real"].as<float>(headingangle_compass);
			// 1. 航向角转换：指南针角度 → 数学标准角度（东为0°，逆时针）
			// default_yaw = fmod(90.0 - headingangle_compass + 720.0, 360.0); // 确保角度在0到360度之间
			default_yaw = fmod(headingangle_compass + 360.0, 360.0); // 确保角度在0到360度之间
			if (!debug_mode_)
				std::cout << "读取默认偏航角: " << default_yaw << "，默认旋转角：" << default_yaw << "，实际方向角：" << headingangle_real << std::endl;
			headingangle_compass = headingangle_compass * M_PI / 180.0; // 弧度制
			default_yaw = M_PI/2 - default_yaw * M_PI / 180.0; // 弧度制 NED
			headingangle_real = headingangle_real * M_PI / 180.0; // 弧度制
			
		} catch (const YAML::Exception &e) {
            std::cerr << "读取配置文件时发生错误: " << e.what() << std::endl;
			return;
		}
		// tx_shot = dx_shot;
		// ty_shot = dy_shot;
		// tx_see = dx_see;
		// ty_see = dy_see;
	}
protected:
    // headingangle_compass为罗盘读数 角度制
	float headingangle_compass; // 默认罗盘读数，待读取
	float headingangle_real;

    float default_yaw = 0.0; // 默认偏转角 = 450 - headingangle_compass 角度
    
    static Vector4f start;

	double timestamp_init = 0; // 初始化时间戳
	double start_time = 0; // 任务开始时间



	std::shared_ptr<StatusController> sta_ctl_;
    std::shared_ptr<PosController> pos_ctl_;

	// module
	std::shared_ptr<CameraInterface> _camera_gimbal = nullptr;
	std::shared_ptr<ServoControllerInterface> _servo_controller = nullptr;
	std::shared_ptr<YOLODetectorInterface> yolo_detector = nullptr;

};