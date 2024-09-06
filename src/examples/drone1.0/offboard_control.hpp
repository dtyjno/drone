#ifndef OFFBOARD_CONTROL_H
#define OFFBOARD_CONTROL_H
#endif

#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol_local.hpp>
#include <mavros_msgs/srv/command_tol.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/altitude.hpp>

#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/global_position_target.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <mavros_msgs/msg/state.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <string>
#include <cmath>



//  #define GET_GPS
#define PI 3.14159
//#define default_heading -0.044156
//#define default_heading -60  //yaw=-2.53
//M_PI dhaeding: 0.000000
#define DEFAULT_ACCURACY 0.3 //+-0.3m
#define DEFAULT_ACCURACY_YAW 1 //+-1°
#define DEfault_HEADING 60.00//角度制
//using namespace std::chrono;
// struct State {
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//   Eigen::Vector3d position;
//   Eigen::Vector3d velocity;
//   Eigen::Vector4d attitude;
// };
using namespace std::chrono_literals;

#include <iostream>
#include <thread>


#include <Eigen/Dense>

class OffboardControl : public rclcpp::Node {
public:
	OffboardControl(const std::string ardupilot_namespace) :
		Node("offboard_control_srv"),
		state_{State::init},
		fly_state_{FlyState::init},
		service_result_{0},
		service_done_{false},
		
		//global_gps_publisher_{this->create_publisher<ardupilot_msgs::msg::GlobalPosition>(ardupilot_namespace+"cmd_gps_pose", 5)},
		twist_stamped_publisher_{this->create_publisher<geometry_msgs::msg::TwistStamped>(ardupilot_namespace+"setpoint_velocity/cmd_vel", 5)},
		//  * /mavros/setpoint_position/global [geographic_msgs/msg/GeoPoseStamped] 1 subscriber
		global_gps_publisher_{this->create_publisher<geographic_msgs::msg::GeoPoseStamped>(ardupilot_namespace+"setpoint_position/global", 5)},
		// * /mavros/setpoint_position/local [geometry_msgs/msg/PoseStamped] 1 subscriber
		local_setpoint_publisher_{this->create_publisher<geometry_msgs::msg::PoseStamped>(ardupilot_namespace+"setpoint_position/local", 5)},
		// * /mavros/setpoint_raw/local [mavros_msgs/msg/PositionTarget] 1 subscriber
		/*# A Pose with reference coordinate frame and timestamp
		std_msgs/Header header
		Pose pose*/
		setpoint_raw_local_publisher_(this->create_publisher<mavros_msgs::msg::PositionTarget>(ardupilot_namespace+"setpoint_raw/local", 5)),
		//  * /mavros/setpoint_raw/global [mavros_msgs/msg/GlobalPositionTarget] 1 subscriber
		setpoint_raw_global_publisher_(this->create_publisher<mavros_msgs::msg::GlobalPositionTarget>(ardupilot_namespace+"setpoint_raw/global", 5)),
		// /mavros/setpoint_trajectory/local [trajectory_msgs/msg/MultiDOFJointTrajectory] 1 subscriber
		trajectory_publisher_{this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(ardupilot_namespace+"setpoint_trajectory/local", 5)},
		
		
		arm_motors_client_{this->create_client<mavros_msgs::srv::CommandBool>(ardupilot_namespace+"cmd/arming")},
		mode_switch_client_{this->create_client<mavros_msgs::srv::SetMode>(ardupilot_namespace+"set_mode")}
		
	{
		ardupilot_namespace_copy_ = ardupilot_namespace;
		// 质量服务配置
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		// 声明回调组,实例化回调组，类型为：可重入的
		rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_= this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
		// Each of these callback groups is basically a thread
    	// Everything assigned to one of them gets bundled into the same thread
		auto sub_opt = rclcpp::SubscriptionOptions();
    	sub_opt.callback_group = callback_group_subscriber_;
		//ros2 topic echo /mavros/local_position/pose geometry_msgs/msg/PoseStamped
		pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(ardupilot_namespace_copy_+"local_position/pose", qos,
		std::bind(&OffboardControl::pose_callback, this, std::placeholders::_1),sub_opt);
		//ros2 topic echo /mavros/global_position/global sensor_msgs/msg/NavSatFix
		gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(ardupilot_namespace_copy_+"global_position/global", qos,
		std::bind(&OffboardControl::gps_callback, this, std::placeholders::_1),sub_opt);
		velocity_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(ardupilot_namespace_copy_+"local_position/velocity_local", qos,
		std::bind(&OffboardControl::velocity_callback, this, std::placeholders::_1),sub_opt);
		altitude_subscription_ = this->create_subscription<mavros_msgs::msg::Altitude>(ardupilot_namespace_copy_+"altitude", qos,
        std::bind(&OffboardControl::altitude_callback, this, std::placeholders::_1),sub_opt);
		//订阅IMU信息
		// * /mavros/imu/data [sensor_msgs/msg/Imu] 1 publisher
		//ros2 topic echo /mavros/imu/data
		
		//查询系统状态
		// * /mavros/state [mavros_msgs/msg/State] 1 publisher
		//ros2 topic echo /mavros/state
		state_subscription_ = this->create_subscription<mavros_msgs::msg::State>(ardupilot_namespace_copy_+"state", qos,
		 std::bind(&OffboardControl::state_callback, this, std::placeholders::_1));

		RCLCPP_INFO(this->get_logger(), "Starting Offboard Control example with PX4 services");
		//RCLCPP_INFO_STREAM(geometry_msgs::msg::PoseStampedthis->get_logger(), "Waiting for " << px4_namespace << "vehicle_command service");
		while (!mode_switch_client_->wait_for_service(std::chrono::seconds(2))) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
				return;
			}
			RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  		}
		// std::thread t1(DaemonThread::run);
		// t1.detach();
		timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));
	}

    void switch_mode(std::string mode);
	void arm_motors(bool arm);
	double quaternion_to_yaw(double x, double y, double z, double w);
	void yaw_to_quaternion(double yaw, double* x, double* y, double* z, double* w);
private:
	class LocalFrame{
	public:
		double x;
		double y;
		double z;
		double yaw;
	};
	class GlobalFrame{
	public:
		double lat;
		double lon;
		double alt;
		double yaw;
	};

	// '''定义一个全局变量'''
	double _rngfnd_distance;
	double timestamp0;
	double heading=0;
	const double default_heading=DEfault_HEADING;//初始偏转角
	class Location{
	public:
		LocalFrame local_frame;
		GlobalFrame global_frame;
	};
	Eigen::Quaterniond quaternion;// 四元数
	Eigen::Vector3d euler;// 欧拉角
	Location location;
	geometry_msgs::msg::PoseStamped pose_{};
	geometry_msgs::msg::TwistStamped velocity_{};
	sensor_msgs::msg::NavSatFix global_gps_{};
	mavros_msgs::msg::State drone_state_{};

	LocalFrame start{0,0,0,0};
	LocalFrame start_temp{0,0,0,0};
	LocalFrame end_temp={0,0,0,0};
	//LocalFrame end={0,0,0,0};

	GlobalFrame start_global{0,0,0,0};
	GlobalFrame start_temp_global{0,0,0,0};
	GlobalFrame end_temp_global={0,0,0,0};
	//LocalFrame end_global={0,0,0,0};

		


	double PrintYaw(void);
	//main.cpp
	enum class State{
		init,
		send_geo_grigin,
		wait_for_stable_offboard_mode,
		arm_requested,
		takeoff,
		autotune_mode,
		
	} state_;
	//起飞
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
	std::string ardupilot_namespace_copy_;
	uint8_t service_result_;
	bool service_done_;
	bool arm_done_;

	double k=0.002;//控制vx和vy
	// 初始化PID控制器
	double dt=0.1;
	double kp = 0.55;  // 比例参数
	double ki = 0.10;  // 积分参数
	double kd = 0.125;  // 微分参数
	double kp_yaw = 0.20;  // 比例参数
	double ki_yaw = 0.04;  // 积分参数
	double kd_yaw = 0.04;  // 微分参数
	double max_vx=1; //前后方向最大速度
	double max_vy=1; //左右方向最大速度
	double max_vz=1; //上下方向最大速度
	double max_yaw=10; //最大角速度(°/s)

	GlobalFrame shot_area_start{};
	GlobalFrame scout_area_start{};



	rclcpp::TimerBase::SharedPtr timer_;
	
	//sensor_msgs::msg::NavSatFix global_gps_start{};
	//ardupilot_msgs::msg::GlobalPosition global_gps_start{};

	//rclcpp::Publisher<ardupilot_msgs::msg::GlobalPosition>::SharedPtr global_gps_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_publisher_;
	rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr global_gps_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_setpoint_publisher_;
	rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_raw_local_publisher_;
	rclcpp::Publisher<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr setpoint_raw_global_publisher_;
	rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr trajectory_publisher_;
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
	rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscription_;
	rclcpp::Subscription<mavros_msgs::msg::Altitude>::SharedPtr altitude_subscription_;
	rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscription_;
 	rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_motors_client_;
	rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_switch_client_;
	
	
	void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
	void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
	void altitude_callback(const mavros_msgs::msg::Altitude::SharedPtr msg);
	void state_callback(const mavros_msgs::msg::State::SharedPtr msg);

	void switch_to_guided_mode();
	void switch_to_flip_mode();
	void switch_to_rtl_mode();
	// void switch_to_auto_mode();
	//void publish_global_gps(double latitude, double longitude, double altitude);
	
	void command_takeoff_or_land_local(std::string mode);
	void command_takeoff_or_land(std::string mode);
	
	void trajectory_setpoint_global(double x,double y ,double z ,double yaw);
	void trajectory_setpoint_takeoff(double x,double y ,double z ,double yaw);
	void trajectory_setpoint(double x,double y ,double z ,double yaw, double accuracy=DEFAULT_ACCURACY);
	void trajectory_setpoint_start(double x,double y ,double z ,double yaw,double accuracy=DEFAULT_ACCURACY);
	bool alt_hold(double vx,double vy ,double z ,double yaw,double time,double accuracy=DEFAULT_ACCURACY);
	void publish_trajectory_setpoint(double x,double y ,double z ,double yaw);
	bool publish_trajectory_setpoint_z(double *x,double accuracy=DEFAULT_ACCURACY);
	bool publish_trajectory_setpoint_yaw(double *yaw,double accuracy=DEFAULT_ACCURACY_YAW);
	bool send_velocity_command_with_time(double linear_x, double linear_y, double linear_z, double angular_z, double time);
	void send_velocity_command(double linear_x, double linear_y, double linear_z, double angular_z);
	void publish_trajectory(double x, double y, double z, double yaw);
	
	void send_gps_setpoint_command(double latitude, double longitude, double altitude);
	void send_local_setpoint_command(double x, double y, double z, double yaw);
	void publish_setpoint_raw_local(double x, double y, double z, double yaw);
	void publish_setpoint_raw_global(double latitude, double longitude, double altitude, double yaw);
	
	bool set_time(double time);

	bool surrounding_shot_area(void);
	bool surrounding_scout_area(void);

	void set_target_point(std::string mode,double x=0,double y=0,double z=0,double yaw=0);
	// void set_start_temp_point(double x,double y,double z,double yaw);
	// void set_end_temp_point(double x,double y,double z,double yaw);
	// void set_drone_target_point_local(double x,double y,double z,double yaw);
	// void set_start_point_local(double x,double y,double z,double yaw);
	// void set_world_point_local(double x,double y,double z,double yaw);
	bool at_check_point(double accuracy=DEFAULT_ACCURACY);	

	void get_target_location(double* x, double* y,double default_heading=0);
	inline void get_target_location_global(double &lat,double &lon,double &alt,double default_heading=0);

	void timer_callback(void);
};