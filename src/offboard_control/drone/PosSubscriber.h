#pragma once

#include "../utils/math.h"
#include "PosSubscriberInterface.h"
#include "PosDataObserverInterface.h"

#define DEFAULT_POS INFINITY

class PosSubscriber : public PosSubscriberInterface, public PosDataObservable{
public:
     ~PosSubscriber() = default;

	Vector3f get_position(){
		return position;
	}

	void set_position(Vector3f pos){
		position = pos;
	}

	Vector3f get_velocity(){
		return {velocity.x(),velocity.y(),velocity.z()};
	}
	void set_velocity(Vector3f vel){
		velocity = {vel.x(),vel.y(),vel.z(),0.0};
	}
	// Add method for yaw velocity
	float get_velocity_yaw() {
		return velocity.w();
	}
	
	void set_velocity_yaw(float yaw_vel) {
		velocity.w() = yaw_vel;
	}
	Quaternionf get_orientation(){
		return orientation;
	}

	Vector3f get_gps(){
		return gps;
	}
	void set_gps(Vector3f g){
		gps = g;
	}

	float get_altitude(){
		return altitude;
	}
	void set_altitude(float alt){
		altitude = alt;
	}
	Vector3f get_linear_acceleration(){
		return linear_acceleration;
	}
	void set_linear_acceleration(Vector3f la){
		linear_acceleration = la;
	}
	Vector3f get_angular_velocity(){
		return angular_velocity;
	}
	void set_angular_velocity(Vector3f av){
		angular_velocity = av;
	}
	float get_rangefinder_height(){
		return rangefinder_height;
	}
	void set_rangefinder_height(float rf){
		rangefinder_height = rf;
	}
	float get_roll(){
		return roll;
	}
	float get_pitch(){
		return pitch;
	}
    float get_yaw(void)
	{
		// 计算欧拉角
		float yaw = atan2(2.0 * (orientation.w() * orientation.z() 
			+ orientation.x() * orientation.y()), 1.0 - 2.0 * 
			(orientation.y() * orientation.y() + orientation.z() * orientation.z()));

		// 角度标准化到 [-π, π]
		// auto normalize_angle = [](float angle) -> float {
		// 	while (angle > M_PI) angle -= 2.0f * M_PI;
		// 	while (angle < -M_PI) angle += 2.0f * M_PI;
		// 	return angle;
		// };
		return yaw;
	}
	void set_yaw(float y){
		yaw = y;
	}

	// 角度标准化到 [-π, π]
	float normalize_angle(float angle){
		while (angle > M_PI) angle -= 2.0f * M_PI;
		while (angle < -M_PI) angle += 2.0f * M_PI;
		return angle;
	};
	void calculate_euler(float &roll, float &pitch, float &yaw) {
		// 获取四元数分量
		float w = orientation.w();
		float x = orientation.x();
		float y = orientation.y();
		float z = orientation.z();

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
	void get_euler(float &r, float &p, float &y){
		r = roll;
		p = pitch;
		y = yaw;
	}
	void set_euler(float r, float p, float y){
		roll = r;
		pitch = p;
		yaw = y;
	}


    // 位置相关接口
    Vector3f position = {DEFAULT_POS, 0.0, 2.0}; // 默认位置设为无穷大，表示未初始化
	Vector4f velocity = {0.0, 0.0, 0.0, 0.0};
	Quaternionf orientation = {1.0, 0.0, 0.0, 0.0};
	Vector3f gps = {DEFAULT_POS, DEFAULT_POS, DEFAULT_POS};
	float altitude = DEFAULT_POS;
	Vector3f linear_acceleration = {0.0, 0.0, 0.0};
	Vector3f angular_velocity = {0.0, 0.0, 0.0};
	float rangefinder_height = DEFAULT_POS;
	float roll = 0.0, pitch = 0.0, yaw = 0.0; // 欧拉角

};