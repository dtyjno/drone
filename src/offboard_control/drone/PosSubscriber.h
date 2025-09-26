#pragma once

#include "../utils/math.h"

#define DEFAULT_POS INFINITY

class PosSubscriber {
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
	void set_rpy(float r, float p, float y){
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