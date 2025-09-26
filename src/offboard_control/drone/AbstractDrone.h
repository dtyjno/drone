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

class AbstractDrone {
public:
    // 默认构造函数
    AbstractDrone()
	{
        std::cout << "AbstractDrone: Starting Drone example" << std::endl;
        read_default_yaw_configs("Drone.yaml");
    }

    AbstractDrone(std::shared_ptr<StatusController> sta_ctl, std::shared_ptr<PosController> pos_ctl)
        : sta_ctl_(sta_ctl),
          pos_ctl_(pos_ctl)
	{
        std::cout << "AbstractDrone: Starting Drone example and setting up controllers" << std::endl;
        read_default_yaw_configs("Drone.yaml");
    }
    
    // 初始化方法，用于在构造后设置控制器
    void initialize_controllers(std::shared_ptr<StatusController> sta_ctl, std::shared_ptr<PosController> pos_ctl) {
        std::cout << "AbstractDrone: Setting up controllers" << std::endl;
        sta_ctl_ = sta_ctl;
        pos_ctl_ = pos_ctl;
    }
    
    virtual ~AbstractDrone() = default;

    // 位置稳定后设置初始位置
    void init_startposition() {
        start = {get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()}; 
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
    // virtual float get_rangefinder_distance() = 0;

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

    // // 日志接口
    // virtual int save_log(bool finish = false) = 0;


    // 计时器相关接口
	virtual double get_cur_time() = 0;
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
    void send_start_setpoint_command(float x, float y, float z, float yaw);
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
    void log_info(const std::string& format, ...);
    void log_warn(const std::string& format, ...);
    void log_error(const std::string& format, ...);
    void log_debug(const std::string& format, ...);
	void log_debug_throttle(const std::chrono::milliseconds& wait_time, const std::string& format, ...);
	void log_info_throttle(const std::chrono::milliseconds& wait_time, const std::string& format, ...);
	void log_warn_throttle(const std::chrono::milliseconds& wait_time, const std::string& format, ...);
	void log_error_throttle(const std::chrono::milliseconds& wait_time, const std::string& format, ...);

	std::chrono::milliseconds wait_time{50}; // 定时器间隔

	virtual void accept(std::shared_ptr<TaskBase> visitor);

protected:
    // headingangle_compass为罗盘读数 角度制
	float headingangle_compass; // 默认罗盘读数，待读取
	float headingangle_real;

    float default_yaw = 0.0; // 默认偏转角 = 450 - headingangle_compass 角度
    
    static Vector4f start;

	double timestamp_init = 0; // 初始化时间戳
	double start_time = 0; // 任务开始时间

	// 配置读取接口
	void read_default_yaw_configs(const std::string &filename)
	{
		std::cout << "读取配置文件: " << filename << std::endl;
		 // 读取配置文件
		YAML::Node config = Readyaml::readYAML(filename);
		try {
			headingangle_compass = config["headingangle_compass"].as<float>(180.0); // 默认罗盘角度
			headingangle_real = config["headingangle_real"].as<float>(headingangle_compass);
			// 1. 航向角转换：指南针角度 → 数学标准角度（东为0°，逆时针）
			// default_yaw = fmod(90.0 - headingangle_compass + 720.0, 360.0); // 确保角度在0到360度之间
			default_yaw = fmod(headingangle_compass + 360.0, 360.0); // 确保角度在0到360度之间
            std::cout << "读取默认偏航角: " << default_yaw << "，默认旋转角：" << default_yaw << "，实际方向角：" << headingangle_real << std::endl;
			headingangle_compass = headingangle_compass * M_PI / 180.0; // 弧度制
			default_yaw = M_PI/2 - default_yaw * M_PI / 180.0; // 弧度制
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
	std::shared_ptr<StatusController> sta_ctl_;
    std::shared_ptr<PosController> pos_ctl_;

};