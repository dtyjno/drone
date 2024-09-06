#ifndef POSECONTROL_H
#define POSECONTROL_H

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/global_position_target.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>

#include "rclcpp/rclcpp.hpp"
#include "Vector3.h"
#include "Vector4.h"
#include "OffboardControl_Base.h"
#include "PID.h"


//
 # define POSCONTROL_Z_P                    0.6f    // vertical velocity controller P gain default
 # define POSCONTROL_Z_I                    0.0f    // vertical velocity controller I gain default
 # define POSCONTROL_Z_D                    0.0f    // vertical velocity controller D gain default
 # define POSCONTROL_Z_IMAX                 1.0f // vertical velocity controller IMAX gain default

 # define POSCONTROL_Z_FILT_P_HZ            0.3f    // vertical velocity controller input filter
 # define POSCONTROL_Z_FILT_D_HZ            0.01f    // vertical velocity controller input filter for D
 
 # define POSCONTROL_XY_P                   0.8f    // horizontal velocity controller P gain default 0.5
 # define POSCONTROL_XY_I                   0.0f    // horizontal velocity controller I gain default 0.2
 # define POSCONTROL_XY_D                   0.0f    // horizontal velocity controller D gain default 0.1
 # define POSCONTROL_XY_IMAX                1.0f // horizontal velocity controller IMAX gain default

 # define POSCONTROL_VEL_XY_MAX                    2.0f    // horizontal acceleration controller max acceleration default
 # define POSCONTROL_VEL_Z_MAX                     2.0f    // vertical acceleration controller max acceleration default
 # define POSCONTROL_VEL_YAW_MAX                   0.3f   // yaw acceleration controller max acceleration default

 # define DEFAULT_ACCURACY                          0.01f   // default accuracy for position control
 # define DEFAULT_YAW_ACCURACY                      0.1f   // default accuracy for yaw control


// 串级PID控制器
 # define POSCONTROL_POS_Z_P                    1.3f//1.0f    // vertical position controller P gain default
 # define POSCONTROL_VEL_Z_P                    1.0f//5.0f    // vertical velocity controller P gain default
 # define POSCONTROL_VEL_Z_IMAX                 1000.0f // vertical velocity controller IMAX gain default
 # define POSCONTROL_VEL_Z_FILT_HZ              5.0f    // vertical velocity controller input filter
 # define POSCONTROL_VEL_Z_FILT_D_HZ            5.0f    // vertical velocity controller input filter for D
 # define POSCONTROL_ACC_Z_P                    0.5f    // vertical acceleration controller P gain default
 # define POSCONTROL_ACC_Z_I                    0.5f//1.0f    // vertical acceleration controller I gain default
 # define POSCONTROL_ACC_Z_D                    0.0f    // vertical acceleration controller D gain default
 # define POSCONTROL_ACC_Z_IMAX                 800     // vertical acceleration controller IMAX gain default
 # define POSCONTROL_ACC_Z_FILT_HZ              20.0f   // vertical acceleration controller input filter default
 # define POSCONTROL_ACC_Z_DT                   0.0025f // vertical acceleration controller dt default
 # define POSCONTROL_POS_XY_P                   1.3f//1.0f    // horizontal position controller P gain default
 # define POSCONTROL_VEL_XY_P                   0.75f//2.0f    // horizontal velocity controller P gain default
 # define POSCONTROL_VEL_XY_I                   0.40f//1.0f    // horizontal velocity controller I gain default
 # define POSCONTROL_VEL_XY_D                   0.25f//0.5f    // horizontal velocity controller D gain default
 # define POSCONTROL_VEL_XY_IMAX                1000.0f // horizontal velocity controller IMAX gain default
 # define POSCONTROL_VEL_XY_FILT_HZ             5.0f    // horizontal velocity controller input filter
 # define POSCONTROL_VEL_XY_FILT_D_HZ           5.0f    // horizontal velocity controller input filter for D

 # define POSCONTROL_ACC_XY_MAX                 0.3f    
 # define POSCONTROL_ACC_Z_MAX                  0.3f 


 
class PoseControl{
public:
    PoseControl(const std::string ardupilot_namespace,OffboardControl_Base* node) : node(node)
	{
		this->ardupilot_namespace = ardupilot_namespace;
		// RCLCPP_INFO(node->get_logger(), "Starting Pose Control example");
		//global_gps_publisher_{this->create_publisher<ardupilot_msgs::msg::GlobalPosition>(ardupilot_namespace+"cmd_gps_pose", 5)},
		twist_stamped_publisher_=node->create_publisher<geometry_msgs::msg::TwistStamped>(ardupilot_namespace+"setpoint_velocity/cmd_vel", 5);
        //  * /mavros/setpoint_position/global [geographic_msgs/msg/GeoPoseStamped] 1 subscriber
		// global_gps_publisher_=this->create_publisher<geographic_msgs::msg::GeoPoseStamped>(ardupilot_namespace+"setpoint_position/global", 5);
		// * /mavros/setpoint_position/local [geometry_msgs/msg/PoseStamped] 1 subscriber
		local_setpoint_publisher_=node->create_publisher<geometry_msgs::msg::PoseStamped>(ardupilot_namespace+"setpoint_position/local", 5);
        // * /mavros/setpoint_raw/local [mavros_msgs/msg/PositionTarget] 1 subscriber
        setpoint_raw_local_publisher_=node->create_publisher<mavros_msgs::msg::PositionTarget>(ardupilot_namespace+"setpoint_raw/local", 5);
        //  * /mavros/setpoint_raw/global [mavros_msgs/msg/GlobalPositionTarget] 1 subscriber
		setpoint_raw_global_publisher_=node->create_publisher<mavros_msgs::msg::GlobalPositionTarget>(ardupilot_namespace+"setpoint_raw/global", 5);
		// /mavros/setpoint_trajectory/local [trajectory_msgs/msg/MultiDOFJointTrajectory] 1 subscriber
		// trajectory_publisher_=this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(ardupilot_namespace+"setpoint_trajectory/local", 5);
		setpoint_accel_publisher_=node->create_publisher<geometry_msgs::msg::Vector3Stamped>(ardupilot_namespace+"setpoint_accel/accel", 5);
		setpoint_raw_attitude_publisher_=node->create_publisher<mavros_msgs::msg::AttitudeTarget>(ardupilot_namespace+"setpoint_raw/attitude", 5);
		pid_x = PID(
			POSCONTROL_XY_P,
			POSCONTROL_XY_I,
			POSCONTROL_XY_D,
			0,
			0,
			POSCONTROL_XY_IMAX,
			0
		);
		pid_y = PID(
			POSCONTROL_XY_P,
			POSCONTROL_XY_I,
			POSCONTROL_XY_D,
			0,
			0,
			POSCONTROL_XY_IMAX,
			0
		);
		pid_z = PID(
			POSCONTROL_Z_P,
			POSCONTROL_Z_I,
			POSCONTROL_Z_D,
			0,
			0,
			POSCONTROL_Z_IMAX,
			0
		);
		pid_yaw = PID(
			POSCONTROL_Z_FILT_P_HZ,
			0,
			POSCONTROL_Z_FILT_D_HZ,
			0,
			0,
			0,
			0	
		);
		pid_px = PID(
			POSCONTROL_POS_XY_P,
			0,
			0
		);
		pid_py = PID(
			POSCONTROL_POS_XY_P,
			0,
			0
		);
		pid_pz = PID(
			POSCONTROL_POS_Z_P,
			0,
			0
		);
		pid_vx = PID(
			POSCONTROL_VEL_XY_P,
			POSCONTROL_VEL_XY_I,
			POSCONTROL_VEL_XY_D,
			0,
			0,
			POSCONTROL_VEL_XY_IMAX,
			0
		);
		pid_vy = PID(
			POSCONTROL_VEL_XY_P,
			POSCONTROL_VEL_XY_I,
			POSCONTROL_VEL_XY_D,
			0,
			0,
			POSCONTROL_VEL_XY_IMAX,
			0
		);
		pid_vz = PID(
			POSCONTROL_VEL_Z_P,
			0,
			0
		);
    }

	OffboardControl_Base* node;
	void publish_setpoint_raw(Vector4f p, Vector4f v);
    void publish_setpoint_raw_global(double latitude, double longitude, double altitude, double yaw);
    void send_local_setpoint_command(double x, double y, double z,double yaw);
    void send_velocity_command_world(double linear_x, double linear_y, double linear_z, double angular_z);
	void send_velocity_command_world(Vector4f v);
	void send_velocity_command(Vector4f v);
	bool send_velocity_command_with_time(Vector4f v, double time);
	void send_accel_command(Vector4f v);
	// void setYawRate(float yaw_rate);
	bool publish_setpoint_world(Vector4f now, Vector4f target, double accuracy = DEFAULT_ACCURACY, double yaw_accuracy = DEFAULT_YAW_ACCURACY);
	bool trajectory_setpoint(Vector4f pos_now,Vector4f pos_target,double accuracy = DEFAULT_ACCURACY,double yaw_accuracy = DEFAULT_YAW_ACCURACY);
	bool trajectory_setpoint_world(Vector4f pos_now,Vector4f pos_target,double accuracy = DEFAULT_ACCURACY,double yaw_accuracy = DEFAULT_YAW_ACCURACY);
	bool trajectory_setpoint(Vector4f pos_now,Vector4f pos_target,PID::Defaults defaults,double accuracy = DEFAULT_ACCURACY,double yaw_accuracy = DEFAULT_YAW_ACCURACY);


private:
	std::string ardupilot_namespace;

	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_publisher_;
	// rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr global_gps_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_setpoint_publisher_;
	rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_raw_local_publisher_;
	rclcpp::Publisher<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr setpoint_raw_global_publisher_;
	// rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr trajectory_publisher_;
	// /mavros/setpoint_accel/accel (geometry_msgs/Vector3Stamped)
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr setpoint_accel_publisher_;
	// /mavros/setpoint_raw/attitude mavros_msgs/msg/AttitudeTarget
	rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr setpoint_raw_attitude_publisher_;
	
	//
	PID pid_x;
	PID pid_y;
	PID pid_z;
	PID pid_yaw;
	// 串级PID控制器
	PID pid_px;
	PID pid_py;
	PID pid_pz;
	PID pid_vx;
	PID pid_vy;
	PID pid_vz;
	float max_speed_xy = POSCONTROL_VEL_XY_MAX;
	float max_speed_z = POSCONTROL_VEL_Z_MAX;
	float max_speed_yaw = POSCONTROL_VEL_YAW_MAX;
	float max_accel_xy = POSCONTROL_ACC_XY_MAX;
	float max_accel_z = POSCONTROL_ACC_Z_MAX;
	// float max_speed_yaw = POSCONTROL_VEL_YAW_MAX;
	float default_accuracy = DEFAULT_ACCURACY;
	float default_yaw_accuracy = DEFAULT_YAW_ACCURACY;
	float default_yaw = DEFAULT_YAW;
	float dt = 1;
	float dt_pid_p_v = 1;
	Vector3f input_pos_xyz(Vector3f now, Vector3f target);
	Vector4f input_pos_xyz_yaw(Vector4f now, Vector4f target);
	Vector4f input_pos_vel_1_xyz_yaw(Vector4f now, Vector4f target);
	Vector4f input_pos_vel_xyz_yaw(Vector4f now, Vector4f target);


	Vector4f _pos_target;
	Vector4f _pos_desired;
};
#endif  // POSECONTROL_H