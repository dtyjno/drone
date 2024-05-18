#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol_local.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
//#include <ardupilot_msgs/msg/global_position.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/altitude.hpp>

#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/global_position_target.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

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
#define DEfault_HEADING 0.000000//角度制
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
// #include <Eigen/Dense>
//   std::vector<Eigen::Vector3d> vehicle_position_history_;
//   std::vector<geometry_msgs::msg::PoseStamped> posehistory_vector_;
//   std::vector<geometry_msgs::msg::PoseStamped> referencehistory_vector_;
//   Eigen::Vector3d vehicle_position_{Eigen::Vector3d::Zero()};
//   Eigen::Vector3d vehicle_velocity_{Eigen::Vector3d::Zero()};
//   Eigen::Vector4d vehicle_attitude_{Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)};
//   Eigen::Vector3d last_planning_position_{Eigen::Vector3d::Zero()};
//   Eigen::Vector3d previous_start_position_{Eigen::Vector3d::Zero()};
//   Eigen::Vector3d previous_return_start_position_{Eigen::Vector3d::Zero()};
//   Eigen::Vector3d mission_loiter_center_{Eigen::Vector3d::Zero()};
// void TerrainPlanner::mavLocalPoseCallback(const geometry_msgs::msg::PoseStamped &msg) {
//   vehicle_attitude_(0) = msg.pose.orientation.w;
//   vehicle_attitude_(1) = msg.pose.orientation.x;
//   vehicle_attitude_(2) = msg.pose.orientation.y;
//   vehicle_attitude_(3) = msg.pose.orientation.z;
// }

// class DaemonThread {
// public:
// 	static void run() {
// 		for (int i = 0; i < 600; i++) {
// 			std::cout << i << std::endl;
// 			std::this_thread::sleep_for(std::chrono::seconds(1));
// 		}
// 	}
// };

#include <Eigen/Dense>
Eigen::Quaterniond quaternion_from_euler(const Eigen::Vector3d &euler)
{
	// YPR is ZYX axes
	return Eigen::Quaterniond(Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()) *
				  Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
				  Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX()));
}
Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q)
{
	// YPR is ZYX axes
	return q.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
}
void quaternion_to_euler(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw)
{
	const auto euler = quaternion_to_euler(q);
	roll = euler.x();
	pitch = euler.y();
	yaw = euler.z();
}

#include <GeographicLib/Geodesic.hpp>//https://geographiclib.sourceforge.io/C++/doc/
#include <GeographicLib/LocalCartesian.hpp>
// 该函数将地理坐标（经度、纬度、高度）转换为（flu）坐标。
// 函数接受七个参数：lat、lon 和 alt 是要转换的地理坐标；lat0、lon0 和 alt0 是原点的地理坐标；x、y 和 z 是输出的 NED 坐标，它们是通过引用传递的，所以函数会直接修改它们的值。
void geodetic_to_flu(double lat, double lon, double alt, double lat0, double lon0, double alt0, double &x, double &y, double &z);
void geodetic_to_flu(double lat, double lon, double alt, double lat0, double lon0, double alt0, double &x, double &y, double &z)
{
	try {
		GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
		GeographicLib::LocalCartesian proj(lat0, lon0, alt0, earth);
		proj.Forward(lat, lon, alt, x, y, z);
	}
	catch (const std::exception& e) {
		std::cerr << "Caught exception: " << e.what() << "\n";
	}
}
// 这是一个使用GeographicLib库实现的函数，
// 它将一个地理坐标（纬度、经度、高度，即LLH）加上一个在本地坐标系（前/左/上，即FLU）中的偏移量，然后返回新的地理坐标。
// lat、lon 和 alt 是输入的地理坐标；offset_f、offset_l 和 offset_u 是在本地坐标系中的偏移量；lat_new、lon_new 和 alt_new 是输出的地理坐标。
void add_flu_offset_to_llh(double lat, double lon, double alt, double offset_f, double offset_l, double offset_u, double &lat_new, double &lon_new, double &alt_new){
    try {
        GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
        GeographicLib::LocalCartesian proj(lat, lon, alt, earth);
        double x, y, z;
        proj.Forward(lat, lon, alt, x, y, z);
        x += offset_f;
        y += offset_l;
        z += offset_u;
        proj.Reverse(x, y, z, lat_new, lon_new, alt_new);
    }
    catch (const std::exception& e) {
        std::cerr << "Caught exception: " << e.what() << "\n";
    }
}
// 	double PrintYaw(void){
// 		quaternion.w() = pose_.pose.orientation.w;
// 		quaternion.x() = pose_.pose.orientation.x;
// 		quaternion.y() = pose_.pose.orientation.y;
// 		quaternion.z() = pose_.pose.orientation.z;
// 		euler = quaternion_to_euler(quaternion);
// 		heading = euler(2); //弧度制
// 		// std::cout << "heading: " << heading/PI*180 << std::endl;
// 		return heading/PI*180; //角度制
// 	}
	
// };
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
		pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(ardupilot_namespace_copy_+"local_position/pose", qos,
		std::bind(&OffboardControl::pose_callback, this, std::placeholders::_1),sub_opt);
		gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(ardupilot_namespace_copy_+"global_position/global", qos,
		std::bind(&OffboardControl::gps_callback, this, std::placeholders::_1),sub_opt);
		velocity_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(ardupilot_namespace_copy_+"local_position/velocity_local", qos,
		std::bind(&OffboardControl::velocity_callback, this, std::placeholders::_1),sub_opt);
		altitude_subscription_ = this->create_subscription<mavros_msgs::msg::Altitude>(ardupilot_namespace_copy_+"altitude", qos,
        std::bind(&OffboardControl::altitude_callback, this, std::placeholders::_1),sub_opt);
		// /mavros/home_position/home
		///mavros/state [mavros_msgs/msg/State] 1 publisher
		// state_subscription_ = this->create_subscription<mavros_msgs::msg::State>(ardupilot_namespace_copy_+"state", qos,
		// std::bind(&OffboardControl::state_callback, this, std::placeholders::_1));

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
	
	LocalFrame start{0,0,0,0};
	LocalFrame start_temp{0,0,0,0};
	LocalFrame end_temp={0,0,0,0};
	//LocalFrame end={0,0,0,0};

	GlobalFrame start_global{0,0,0,0};
	GlobalFrame start_temp_global{0,0,0,0};
	GlobalFrame end_temp_global={0,0,0,0};
	//LocalFrame end_global={0,0,0,0};

		


	double PrintYaw(void){
		quaternion.w() = pose_.pose.orientation.w;
		quaternion.x() = pose_.pose.orientation.x;
		quaternion.y() = pose_.pose.orientation.y;
		quaternion.z() = pose_.pose.orientation.z;
		euler = quaternion_to_euler(quaternion);
		heading = euler(2); //弧度制
		// std::cout << "heading: " << heading/PI*180 << std::endl;
		return heading/PI*180; //角度制
	}
	
	enum class State{
		init,
		send_geo_grigin,
		wait_for_stable_offboard_mode,
		arm_requested,
		takeoff,
		autotune_mode,
		
	} state_;
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
 
 	rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_motors_client_;
	rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_switch_client_;
	
	
	void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
	void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
	void altitude_callback(const mavros_msgs::msg::Altitude::SharedPtr msg);

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

	bool surrending_shot_area(void);
	bool surrending_scout_area(void);

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

void OffboardControl::switch_to_guided_mode(){
	RCLCPP_INFO(this->get_logger(), "requesting switch to GUIDED mode");
	OffboardControl::switch_mode("GUIDED");
}

// void OffboardControl::switch_to_flip_mode(){
// 	RCLCPP_INFO(this->get_logger(), "requesting switch to FLIP mode");
// 	OffboardControl::switch_mode("FLIP");
// }

void OffboardControl::switch_to_rtl_mode(){
	RCLCPP_INFO(this->get_logger(), "requesting switch to RTL mode");
	OffboardControl::switch_mode("RTL");
}

/*
# set FCU mode
#
# Known custom modes listed here:
# http://wiki.ros.org/mavros/CustomModes

# basic modes from MAV_MODE
uint8 MAV_MODE_PREFLIGHT		= 0
uint8 MAV_MODE_STABILIZE_DISARMED	= 80
uint8 MAV_MODE_STABILIZE_ARMED		= 208
uint8 MAV_MODE_MANUAL_DISARMED		= 64
uint8 MAV_MODE_MANUAL_ARMED		= 192
uint8 MAV_MODE_GUIDED_DISARMED		= 88
uint8 MAV_MODE_GUIDED_ARMED		= 216
uint8 MAV_MODE_AUTO_DISARMED		= 92
uint8 MAV_MODE_AUTO_ARMED		= 220
uint8 MAV_MODE_TEST_DISARMED		= 66
uint8 MAV_MODE_TEST_ARMED		= 194

uint8 base_mode		# filled by MAV_MODE enum value or 0 if custom_mode != ''
string custom_mode	# string mode representation or integer
---
bool mode_sent		# Mode known/parsed correctly and SET_MODE are sent

// Auto Pilot Modes enumeration
enum class Number {
    STABILIZE =     0,  // manual airframe angle with manual throttle
    ACRO =          1,  // manual body-frame angular rate with manual throttle
    ALT_HOLD =      2,  // manual airframe angle with automatic throttle
    AUTO =          3,  // fully automatic waypoint control using mission commands
    GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    LOITER =        5,  // automatic horizontal acceleration with automatic throttle
    RTL =           6,  // automatic return to launching point
    CIRCLE =        7,  // automatic circular flight with automatic throttle
    LAND =          9,  // automatic landing with horizontal position control
    DRIFT =        11,  // semi-automous position, yaw and throttle control
    SPORT =        13,  // manual earth-frame angular rate control with manual throttle
    FLIP =         14,  // automatically flip the vehicle on the roll axis
    AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
    POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
    BRAKE =        17,  // full-brake using inertial/GPS system, no pilot input
    THROW =        18,  // throw to launch mode using inertial/GPS system, no pilot input
    AVOID_ADSB =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
    GUIDED_NOGPS = 20,  // guided mode but only accepts attitude and altitude
    SMART_RTL =    21,  // SMART_RTL returns to home by retracing its steps
    FLOWHOLD  =    22,  // FLOWHOLD holds position with optical flow without rangefinder
    FOLLOW    =    23,  // follow attempts to follow another vehicle or ground station
    ZIGZAG    =    24,  // ZIGZAG mode is able to fly in a zigzag manner with predefined point A and point B
    SYSTEMID  =    25,  // System ID mode produces automated system identification signals in the controllers
    AUTOROTATE =   26,  // Autonomous autorotation
    NEW_MODE =     27,  // your new flight mode
};
*/
void OffboardControl::switch_mode(std::string mode)
{
	auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
	request->base_mode = 0;
	request->custom_mode = mode;

	service_done_ = false;

	RCLCPP_INFO(this->get_logger(), "Command send");
	auto result_future = mode_switch_client_->async_send_request(request,
		[this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
			auto status = future.wait_for(0.5s);
			if (status == std::future_status::ready) {
				auto reply = future.get()->mode_sent;
				RCLCPP_INFO(this->get_logger(), "Mode switch: %s", reply ? "success" : "failed");
				if (reply == 1) {
					// Code to execute if the future is successful
					service_done_ = true;
				}
				else {
					// Code to execute if the future is unsuccessful
					service_done_ = false;
					RCLCPP_ERROR(this->get_logger(), "Failed to call service /ap/mode_switch");
				}
			} else {
				// Wait for the result.
				RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
			}
		});
}

// void OffboardControl::switch_to_auto_mode(){
// 	RCLCPP_INFO(this->get_logger(), "requesting switch to mode");
// 	OffboardControl::switch_mode("AUTO");
// }


// arm or disarm motors
// arm= arm: true, disarm: false
void OffboardControl::arm_motors(bool arm)
{
  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = arm;

  while (!arm_motors_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
	RCLCPP_INFO(this->get_logger(), "Command send");
	auto result_future = arm_motors_client_->async_send_request(request,
		[this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
			auto status = future.wait_for(0.5s);
			if (status == std::future_status::ready) {
				auto reply = future.get()->success;
				RCLCPP_INFO(this->get_logger(), "Arm motors: %s", reply ? "success" : "failed");
				if (reply) {
					// Code to execute if the future is successful
					arm_done_ = true;
				}
				else {
					// Code to execute if the future is unsuccessful
					// arm_done_ = false;
					RCLCPP_ERROR(this->get_logger(), ("Failed to call service " + ardupilot_namespace_copy_ + "cmd/arming").c_str());
				}
			} else {
				// Wait for the result.
				RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
			}
		});
}
// # Common type for switch commands

// bool value
// ---
// bool success
// uint8 result

// 笛卡尔坐标系起飞或降落命令（无效）
void OffboardControl::command_takeoff_or_land_local(std::string mode)
{
	std::string mode_str(mode);
	if(mode=="TAKEOFF"){
	auto takeoff_client = this->create_client<mavros_msgs::srv::CommandTOLLocal>(ardupilot_namespace_copy_+"cmd/takeoff_local");

	auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOLLocal::Request>();
	takeoff_request->min_pitch = 0.0;
	takeoff_request->offset = 0.0;
	takeoff_request->rate = 1.0;
	takeoff_request->yaw = 0.0;
	takeoff_request->position.x = 0.0;
	takeoff_request->position.y = 0.0;
	takeoff_request->position.z = 0.0;
	RCLCPP_INFO(this->get_logger(), "Take Off Local Command send");
	auto takeoff_result_future = takeoff_client->async_send_request(takeoff_request);
	} else if(mode=="LAND"){
	auto land_client = this->create_client<mavros_msgs::srv::CommandTOLLocal>(ardupilot_namespace_copy_+"cmd/land_local");

	auto land_request = std::make_shared<mavros_msgs::srv::CommandTOLLocal::Request>();
	land_request->min_pitch = 0.0;
	land_request->offset = 0.0;
	land_request->rate = 1.0;
	land_request->yaw = 0.0;
	land_request->position.x = 0.0;
	land_request->position.y = 0.0;
	land_request->position.z = 0.0;
	RCLCPP_INFO(this->get_logger(), "Land Local Command send");
	auto land_result_future = land_client->async_send_request(land_request);
	}
	/*
	service_done_ = false;
		[this](rclcpp::Client<mavros_msgs::srv::CommandTOLLocal>::SharedFuture future) {
			auto status = future.wait_for(0.5s);
			if (status == std::future_status::ready) {
				auto reply = future.get()->success;
				RCLCPP_INFO(this->get_logger(), "Mode switch: %s", reply ? "success" : "failed");
				if (reply == 1) {
					// Code to execute if the future is successful
					service_done_ = true;
				}
				else {
					// Code to execute if the future is unsuccessful
					//service_done_ = false;
					RCLCPP_ERROR(this->get_logger(), "Failed to call service /ap/mode_switch");
				}
			} else {
				// Wait for the result.
				RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
			}
		});
	*/
}
// #Common type for LOCAL Take Off and Landing

// double32 min_pitch		# used by takeoff
// double32 offset    		# used by land (landing position accuracy)
// double32 rate			# speed of takeoff/land in m/s
// double32 yaw			# in radians
// geometry_msgs/Vector3 position 	#(x,y,z) in meters
// ---
// bool success
// uint8 result

// # Common type for GLOBAL Take Off and Landing
void OffboardControl::command_takeoff_or_land(std::string mode)
{
	//RCLCPP_ERROR(this->get_logger(), ("Failed to call service "+ardupilot_namespace_copy_+"cmd/takeoff").c_str());
	service_done_ = false;
	std::string mode_str(mode);
	if(mode=="TAKEOFF"){
	auto takeoff_client = this->create_client<mavros_msgs::srv::CommandTOL>(ardupilot_namespace_copy_+"cmd/takeoff");

	auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
	takeoff_request->min_pitch = 0.0;
	takeoff_request->yaw = 0.0;
	takeoff_request->latitude = 0.0;
	takeoff_request->longitude = 0.0;
	takeoff_request->altitude = 6.0;
	
	auto takeoff_result_future = takeoff_client->async_send_request(takeoff_request,
		[this](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
			auto status = future.wait_for(0.5s);
			if (status == std::future_status::ready) {
				auto reply = future.get()->success;
				RCLCPP_INFO(this->get_logger(), "TakeOff: %s", reply ? "success" : "failed");
				if (reply == 1) {
					// Code to execute if the future is successful
					service_done_ = true;
				}
				else {
					// Code to execute if the future is unsuccessful
					RCLCPP_ERROR(this->get_logger(), ("Failed to call service "+ardupilot_namespace_copy_+"cmd/takeoff").c_str());
				}
			} else {
				// Wait for the result.
				RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
			}
		});
	} else if(mode=="LAND"){
	auto land_client = this->create_client<mavros_msgs::srv::CommandTOL>(ardupilot_namespace_copy_+"cmd/land");
	
	auto land_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
	land_request->yaw = 0.0;
	land_request->latitude = 0.0;
	land_request->longitude = 0.0;
	land_request->altitude = 0.0;

	auto land_result_future = land_client->async_send_request(land_request,
		[this](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
			auto status = future.wait_for(0.5s);
			if (status == std::future_status::ready) {
				auto reply = future.get()->success;
				RCLCPP_INFO(this->get_logger(), "Land: %s", reply ? "success" : "failed");
				if (reply == 1) {
					// Code to execute if the future is successful
					service_done_ = true;
				}
				else {
					// Code to execute if the future is unsuccessful
					RCLCPP_ERROR(this->get_logger(), ("Failed to call service "+ardupilot_namespace_copy_+"cmd/land").c_str());
				}
			} else {
				// Wait for the result.
				RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
			}
		});
	}
}
/*
接收位置数据
std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
std::cout << "RECEIVED POSITION DATA"   << std::endl;
std::cout << "============================="   << std::endl;
std::cout << "stamp:" << std::to_string(msg->header.stamp.sec) << "." << std::to_string(msg->header.stamp.nanosec) << std::endl;
std::cout << "frame_id:" << msg->header.frame_id << std::endl;
std::cout << "position x: " << msg->pose.position.x << std::endl;
std::cout << "position y: " << msg->pose.position.y  << std::endl;
std::cout << "position z: " << msg->pose.position.z  << std::endl;
std::cout << "orientation: x: " << msg->pose.orientation.x << std::endl;
std::cout << "orientation: y: " << msg->pose.orientation.y << std::endl;
std::cout << "orientation: z: " << msg->pose.orientation.z << std::endl;
std::cout << "orientation: w: " << msg->pose.orientation.w << std::endl;
*/
void OffboardControl::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
{
	pose_ = *msg;
	location.local_frame.x = msg->pose.position.x;
	location.local_frame.y = msg->pose.position.y;
	location.local_frame.z = msg->pose.position.z;
	// quaternion.w() = msg->pose.orientation.w;
	// quaternion.x() = msg->pose.orientation.x;
	// quaternion.y() = msg->pose.orientation.y;
	// quaternion.z() = msg->pose.orientation.z;
	// location.local_frame.x = loc.pose.position.x;
	// location.local_frame.y = loc.pose.position.y;
	// location.local_frame.z = loc.pose.position.z;
	// quaternion.w() = loc.pose.orientation.w;
	// quaternion.x() = loc.pose.orientation.x;
	// quaternion.y() = loc.pose.orientation.y;
	// quaternion.z() = loc.pose.orientation.z;
	// euler = quaternion_to_euler(quaternion);
	// location.global_frame.lat = loc.latitude;
	// location.global_frame.lon = loc.longitude;
	// location.global_frame.alt = loc.altitude;
	
	// RCLCPP_INFO(this->get_logger(),"position x: %lf", pose_.pose.position.x);
	// std::cout << "position x: " << msg->pose.position.x << std::endl;
	// std::cout << "position y: " << msg->pose.position.y  << std::endl;
	// std::cout << "position z: " << msg->pose.position.z  << std::endl;
	//RCLCPP_INFO(this->get_logger(), "Received yaw: %f", quaternion_to_yaw(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w));
}
// 接收GPS数据
// RCLCPP_INFO(this->get_logger(), "Received GPS data");
// RCLCPP_INFO(this->get_logger(), "Header frame id: %s", (*msg).header.frame_id.c_str());
// RCLCPP_INFO(this->get_logger(), "Header stamp: %d.%d", (*msg).header.stamp.sec, (*msg).header.stamp.nanosec);
// RCLCPP_INFO(this->get_logger(), "Latitude: %f", (*msg).latitude);
// RCLCPP_INFO(this->get_logger(), "Longitude: %f", (*msg).longitude);
// RCLCPP_INFO(this->get_logger(), "Altitude: %f", (*msg).altitude);
void OffboardControl::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) 
{
	global_gps_ = *msg;
	location.global_frame.lat = msg->latitude;
	location.global_frame.lon = msg->longitude;
	location.global_frame.alt = msg->altitude;
	#ifdef GET_GPS
	RCLCPP_INFO(this->get_logger(), "Latitude: %f", (*msg).latitude);
	RCLCPP_INFO(this->get_logger(), "Longitude: %f", (*msg).longitude);
	RCLCPP_INFO(this->get_logger(), "Altitude: %f", (*msg).altitude);
	#endif
}
// 接收速度数据
void OffboardControl::velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) 
{
	velocity_ = *msg;
	// RCLCPP_INFO(this->get_logger(), "Received velocity data");
	// RCLCPP_INFO(this->get_logger(), "Linear x: %f", msg->twist.linear.x);
	// RCLCPP_INFO(this->get_logger(), "Linear y: %f", msg->twist.linear.y);
	// RCLCPP_INFO(this->get_logger(), "Linear z: %f", msg->twist.linear.z);
	// RCLCPP_INFO(this->get_logger(), "Angular x: %f", msg->twist.angular.x);
	// RCLCPP_INFO(this->get_logger(), "Angular y: %f", msg->twist.angular.y);
	// RCLCPP_INFO(this->get_logger(), "Angular z: %f", msg->twist.angular.z);
}
// 接收高度数据=0
void OffboardControl::altitude_callback(const mavros_msgs::msg::Altitude::SharedPtr msg) 
{
	// this->location.global_frame.alt = msg->amsl;


	// RCLCPP_INFO(this->get_logger(), "Received altitude data");
	// RCLCPP_INFO(this->get_logger(), "Monotonic: %f", msg->monotonic);
	// RCLCPP_INFO(this->get_logger(), "Amsl: %f", msg->amsl);
	// RCLCPP_INFO(this->get_logger(), "Local: %f", msg->local);
	// RCLCPP_INFO(this->get_logger(), "Relative: %f", msg->relative);
	// RCLCPP_INFO(this->get_logger(), "Terrain: %f", msg->terrain);
	// RCLCPP_INFO(this->get_logger(), "Bottom clear: %f", msg->bottom_clearance);
}

// 环绕射击区域
bool OffboardControl::surrending_shot_area(void){
	static enum class SurState{
		init,
		set_point_x,
		set_point_y,
		fly_to_target_x,
		fly_to_target_y,
		end
	} sur_state_;
	static uint64_t time_find_start;
	/*bool result=false;
	if(vehicle_local_position_msg.x>10 || vehicle_local_position_msg.x<-10 || vehicle_local_position_msg.y>10 || vehicle_local_position_msg.y<-10){
		result=true;
	}*/
	//uint64_t time_now = this->get_clock()->now().nanoseconds() / 1000;
	//AtCheckPoint();
	switch (sur_state_)
	{
	case SurState::init:
		time_find_start = this->get_clock()->now().nanoseconds() / 1000;
		static double fx=1,fy=2*fx;
		sur_state_=SurState::set_point_x;
		break;
	case SurState::set_point_x:
		sur_state_=SurState::fly_to_target_x;
		break;
	case SurState::set_point_y:
		sur_state_=SurState::fly_to_target_y;
		break;
	case SurState::fly_to_target_x:
		trajectory_setpoint(fx,0,0,0);
		if(at_check_point()){
			fx=(fx>0)?(-fx-1):(-fx+1);
			sur_state_=SurState::set_point_y;
		}
		if((this->get_clock()->now().nanoseconds() / 1000-time_find_start)>12000000){//>12s
			sur_state_=SurState::end;
		}
		break;
	case SurState::fly_to_target_y:
		trajectory_setpoint(0,fy,0,0);
		if(at_check_point()){
			sur_state_=SurState::set_point_x;
			fy=(fy>0)?(-fy-1):(-fy+1);
		}
		if((this->get_clock()->now().nanoseconds() / 1000-time_find_start)>12000000){//>12s
			sur_state_=SurState::end;
		}
		break;
	case SurState::end:
		sur_state_=SurState::init;
		fx=1,fy=2*fx;
		return true;
		break;
	default:
		break;
	}
	return false;
}
// 环绕侦察区域
bool OffboardControl::surrending_scout_area(void){
	static enum class ScoState{
		init,
		first_path,
		second_path,
		third_path,
		forth_path,
		fifth_path,
		sixth_path,
		end
	} sco_state_;
	static uint64_t time_find_start;
	/*bool result=false;
	if(vehicle_local_position_msg.x>10 || vehicle_local_position_msg.x<-10 || vehicle_local_position_msg.y>10 || vehicle_local_position_msg.y<-10){
		result=true;
	}*/
	//uint64_t time_now = this->get_clock()->now().nanoseconds() / 1000;
	//AtCheckPoint();
	switch (sco_state_)
	{
	case ScoState::init:
		time_find_start = this->get_clock()->now().nanoseconds() / 1000;
		sco_state_=ScoState::first_path;
		break;
	case ScoState::first_path:
		trajectory_setpoint(2,0,0,-60);
		if(at_check_point()){
			sco_state_=ScoState::second_path;
		}
		if((this->get_clock()->now().nanoseconds() / 1000-time_find_start)>12000000){//>12s
			sco_state_=ScoState::end;
		}
		break;
	case ScoState::second_path:
		trajectory_setpoint(-2,2.5,0,-60);
		if(at_check_point()){
			sco_state_=ScoState::third_path;
		}
		if((this->get_clock()->now().nanoseconds() / 1000-time_find_start)>12000000){//>12s
			sco_state_=ScoState::end;
		}
		break;
	case ScoState::third_path:
		trajectory_setpoint(2,0,0,-60);
		if(at_check_point()){
			sco_state_=ScoState::forth_path;
		}
		if((this->get_clock()->now().nanoseconds() / 1000-time_find_start)>12000000){//>12s
			sco_state_=ScoState::end;
		}
		break;
	case ScoState::forth_path:
		trajectory_setpoint(-2,2.5,0,-60);
		if(at_check_point()){
			sco_state_=ScoState::fifth_path;
		}
		if((this->get_clock()->now().nanoseconds() / 1000-time_find_start)>12000000){//>12s
			sco_state_=ScoState::end;
		}
		break;
	case ScoState::fifth_path:
		trajectory_setpoint(2,0,0,-60);
		if(at_check_point()){
			sco_state_=ScoState::sixth_path;
		}
		if((this->get_clock()->now().nanoseconds() / 1000-time_find_start)>12000000){//>12s
			sco_state_=ScoState::end;
		}
		break;
	case ScoState::sixth_path:
		trajectory_setpoint(-1,-2.5,0,-90);
		if(at_check_point()){
			sco_state_=ScoState::end;
		}
		if((this->get_clock()->now().nanoseconds() / 1000-time_find_start)>12000000){//>12s
			sco_state_=ScoState::end;
		}
		break;
	case ScoState::end:
		sco_state_=ScoState::init;
		RCLCPP_INFO(this->get_logger(), "end scouting: %f", (this->get_clock()->now().nanoseconds() / 1000-time_find_start)/1000000.0);
		return true;
		break;
	default:
		break;
	}
	return false;
}
// 发布全局GPS位置
// 输入相对移动距离
// x:m y:m z:m yaw:°
void OffboardControl::trajectory_setpoint_global(double x,double y ,double z ,double yaw){
	if(global_gps_.header.stamp.sec == 0){
		RCLCPP_INFO(this->get_logger(), "No pose data received yet");
		return;
	}
	get_target_location_global(x,y,z,yaw);
	// RCLCPP_INFO(this->get_logger(),"trajectory_setpoint_global: et:%f %f %f",end_temp.x, end_temp.y, end_temp.z);
	publish_setpoint_raw_global(x,y,z,yaw);
}

















// 飞行到指定位置（相对于世界坐标系）
// x:前后方向位置(m) y:左右方向位置(m) z:高度(m) yaw:偏航角(°）
void OffboardControl::trajectory_setpoint_takeoff(double x,double y ,double z ,double yaw){
	set_target_point("world",start_temp.x+x,start_temp.y+y,z,yaw);
	RCLCPP_INFO(this->get_logger(),"trajectory_setpoint_takeoff: et:%f %f %f",end_temp.x, end_temp.y, end_temp.z); 
	if(pose_.header.stamp.sec == 0){
		RCLCPP_INFO(this->get_logger(), "No pose data received yet");
		return;
	}
	publish_trajectory_setpoint(end_temp.x,end_temp.y,z,yaw);
}
// 飞行到指定位置（相对于当前位置）
// x:前后方向位置(m) y:左右方向位置(m) z:高度(m) yaw:偏航角(°) accuracy=DEFAULT_ACCURACY:精度(m)
void OffboardControl::trajectory_setpoint(double x,double y ,double z ,double yaw,double accuracy){
	static bool first=true;
	if(first){
		get_target_location( &x, &y);
		RCLCPP_INFO(this->get_logger(),"trajectory_setpoint: x:%f y:%f",x,y);
		set_target_point("base_link",x,y,z,yaw);
		first=false;
	}
	if(pose_.header.stamp.sec == 0){
		RCLCPP_INFO(this->get_logger(), "No pose data received yet");
		return;
	}
	if(at_check_point(accuracy)){
		RCLCPP_INFO(this->get_logger(), "at_check_point");
		first=true;
		return;
	}
	publish_trajectory_setpoint(end_temp.x,end_temp.y,end_temp.z,end_temp.yaw);
}
// 飞行到指定位置（相对于起飞点）
// x:前后方向位置(m) y:左右方向位置(m) z:高度(m) yaw:偏航角(°) accuracy=DEFAULT_ACCURACY:精度(m)
void OffboardControl::trajectory_setpoint_start(double x,double y ,double z ,double yaw,double accuracy){
	static bool first=true;
	if(first){
	get_target_location( &x, &y);
	RCLCPP_INFO(this->get_logger(),"trajectory_setpoint_start: x:%f y:%f",x,y);
	set_target_point("start",x,y,z,yaw);
	first=false;
	}
	if(pose_.header.stamp.sec == 0){
		RCLCPP_INFO(this->get_logger(), "No pose data received yet");
		return;
	}
	if(at_check_point(accuracy)){
		RCLCPP_INFO(this->get_logger(), "at_check_point");
		first=true;
		return;
	}
	publish_trajectory_setpoint(end_temp.x,end_temp.y,end_temp.z,end_temp.yaw);
}
// 定高悬停//定角度悬停
// vx=0:前后方向速度(m/s) vy=0:左右方向速度(m/s) z:高度(m) yaw:偏航角(°) time:持续时间(s) accuracy=DEFAULT_ACCURACY:精度(m) 
// 返回值：是否到达规定时间
bool OffboardControl::alt_hold(double vx,double vy ,double z ,double yaw,double time,double accuracy){
	static bool first=true;
	static double vx1=0,vy1=0;
	if(first){
	get_target_location(&vx, &vy);
	vx1=vx;
	vy1=vy;
	RCLCPP_INFO(this->get_logger(),"trajectory_setpoint_start: vx:%f vy:%f",vx,vy);
	start_temp=end_temp;
	set_target_point("base_link",0,0,z,yaw);
	first=false;
	}
	if(pose_.header.stamp.sec == 0){
		RCLCPP_INFO(this->get_logger(), "No pose data received yet");
		return false;
	}
	z=end_temp.z+z;
	yaw=end_temp.yaw+yaw;
	if(
		!set_time(time)
	){
		if(
			//publish_trajectory_setpoint_yaw(&yaw)&
			publish_trajectory_setpoint_z(&z,accuracy)
		){
			RCLCPP_INFO(this->get_logger(), "at_check_point");
			return false;
		}else{
			RCLCPP_INFO(this->get_logger(), "alt_hold: z=%f,yaw=%f", z,yaw);
		send_velocity_command(vx1, vy1, z, yaw);
		return false;
		}
	}else{
		first=true;
		return true;
	}
}
// PID控制_本地坐标系
// x/y/z/yaw位置PID控制
// x:目标位置(m) y:~ z:~ yaw:目标角度(°) //at_check_point(accuracy:精度(m)默认)
// 返回值：是否到达目标位置
void OffboardControl::publish_trajectory_setpoint(double x,double y ,double z ,double yaw){
    //RCLCPP_INFO(this->get_logger(), "Publishing setpoint: x=%f, y=%f, z=%f, yaw=%f", x, y, z, yaw);
	//static double previous_error_x = x - pose_.pose.position.x;
    //static double previous_error_y = y - pose_.pose.position.y;
    //static double previous_error_z = z - pose_.pose.position.z;
    //static double previous_error_yaw = yaw - quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);//-pi-pi

    static double integral_x = 0;
    static double integral_y = 0;
    static double integral_z = 0;
    static double integral_yaw = 0;

	double error_x = x - pose_.pose.position.x;
	double error_y = y - pose_.pose.position.y;
	double error_z = z - pose_.pose.position.z;
	double error_yaw = yaw - quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);
	
	const static int n = 10;
	static double integral_[n][4] = {0}; // 0:x, 1:y, 2:z, 3:yaw
	static int i = 0;
	integral_[i][0] = error_x * dt;
	integral_[i][1] = error_y * dt;
	integral_[i][2] = error_z * dt;
	integral_[i][3] = error_yaw * dt;
	integral_x += error_x * dt;
	integral_y += error_y * dt;
	integral_z += error_z * dt;
    integral_yaw += error_yaw * dt;
	integral_x -= integral_[(i+1)%n][0];
	integral_y -= integral_[(i+1)%n][1];
	integral_z -= integral_[(i+1)%n][2];
	integral_yaw -= integral_[(i+1)%n][3];
	i = (i+1)%n;
	
    //double derivative_x = (error_x - previous_error_x) / dt;
    //double derivative_y = (error_y - previous_error_y) / dt;
    //double derivative_z = (error_z - previous_error_z) / dt;
    //double derivative_yaw = (error_yaw - previous_error_yaw) / dt;

    double output_x = kp * error_x + ki * integral_x + kd * velocity_.twist.linear.x;
    double output_y = kp * error_y + ki * integral_y + kd * velocity_.twist.linear.y;
    double output_z = kp * error_z + ki * integral_z + kd * velocity_.twist.linear.z;
    double output_yaw = kp_yaw * error_yaw + ki_yaw * integral_yaw + kd_yaw * velocity_.twist.angular.z;

    //previous_error_x = error_x;
    //previous_error_y = error_y;
    //previous_error_z = error_z;
    //previous_error_yaw = error_yaw;

	if (output_x > max_vx) output_x = max_vx;
	if (output_x < -max_vx) output_x = -max_vx;
	if (output_y > max_vy) output_y = max_vy;
	if (output_y < -max_vy) output_y = -max_vy;
	if (output_z > max_vz) output_z = max_vz;
	if (output_z < -max_vz) output_z = -max_vz;
	if (output_yaw > 180) output_yaw = -360+output_yaw;
	if (output_yaw < -180) output_yaw = 360+output_yaw;
	if (output_yaw > max_yaw) output_yaw = max_yaw;
	if (output_yaw < -max_yaw) output_yaw = -max_yaw;

	RCLCPP_INFO(this->get_logger(), "publish_trajectory_setpoint: output: x=%f, y=%f, z=%f, yaw=%f", output_x, output_y, output_z, output_yaw);
    send_velocity_command(output_x, output_y, output_z, output_yaw);
	if(at_check_point()){
		//RCLCPP_INFO(this->get_logger(), "at_check_point");
		//previous_error_x = 0;
		//previous_error_y = 0;
		//previous_error_z = 0;
		//previous_error_yaw = 0;
		integral_x = 0;
		integral_y = 0;
		integral_z = 0;
		integral_yaw = 0;
	}
}
// x/y/z位置PID控制_本地坐标系
// x:目标位置(m) accuracy:精度(m)默认
// 返回值：是否到达目标位置
bool OffboardControl::publish_trajectory_setpoint_z(double *z,double accuracy){
    static double integral_z = 0;

	double error_z = *z - pose_.pose.position.z;
	RCLCPP_INFO(this->get_logger(), "publish_trajectory_setpoint: error_z=%f", error_z);
	const static int n = 10;
	static double integral_[n] = {0}; 
	static int i = 0;
	integral_[i] = error_z * dt;
	integral_z += error_z * dt;
	integral_z -= integral_[(i+1)%n];
	i = (i+1)%n;
	RCLCPP_INFO(this->get_logger(), "publish_trajectory_setpoint: integral_z=%f", integral_z);
    double output_z = kp * error_z + ki * integral_z + kd * velocity_.twist.linear.z;

	if (output_z > max_vz) output_z = max_vz;
	if (output_z < -max_vz) output_z = -max_vz;

	RCLCPP_INFO(this->get_logger(), "publish_trajectory_setpoint: output: output_z=%f", output_z);
	RCLCPP_INFO(this->get_logger(), "publish_trajectory_setpoint: output: z=%f", *z);

	if(abs(*z - end_temp.z) <=accuracy){
		integral_z = 0;
		*z = output_z;
		return true;
	}
	*z = output_z;
	return false;
}
// 旋转角度PID控制_本地坐标系
// yaw:目标角度（度） accuracy:精度（度）默认为
// 返回值：是否到达目标角度
bool OffboardControl::publish_trajectory_setpoint_yaw(double *yaw,double accuracy){
    static double integral_yaw = 0;

	double error_yaw = *yaw - quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);
	
	const static int n = 10;
	static double integral_[n] = {0}; // 0:x, 1:y, 2:z, 3:yaw
	static int i = 0;
	integral_[i] = error_yaw * dt;
    integral_yaw += error_yaw * dt;
	integral_yaw -= integral_[(i+1)%n];
	i = (i+1)%n;
	
    double output_yaw = kp_yaw * error_yaw + ki_yaw * integral_yaw + kd_yaw * velocity_.twist.angular.z;

	if (output_yaw > 180) output_yaw = -360+output_yaw;
	if (output_yaw < -180) output_yaw = 360+output_yaw;
	if (output_yaw > max_yaw) output_yaw = max_yaw;
	if (output_yaw < -max_yaw) output_yaw = -max_yaw;

	RCLCPP_INFO(this->get_logger(), "publish_trajectory_setpoint: output_yaw=%f", output_yaw);
	RCLCPP_INFO(this->get_logger(), "publish_trajectory_setpoint: yaw=%f", *yaw);
	RCLCPP_INFO(this->get_logger(), "publish_trajectory_setpoint: pose_yaw=%f", quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w));
	if(abs(*yaw - quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w)) <=accuracy){
		integral_yaw = 0;
		*yaw = output_yaw;
		return true;
	}
	*yaw = output_yaw;
	return false;
}
//time(s), linear_x, linear_y, linear_z, angular_z(jiaoduzhi/s)
bool OffboardControl::set_time(double time){
	static bool first=true;
	static double find_start;
	if(first){
		find_start = this->get_clock()->now().nanoseconds() / 1000;
		first=false;
	}
	RCLCPP_INFO(this->get_logger(), "set_time: %f", (this->get_clock()->now().nanoseconds() / 1000-find_start)/1000000.0);
	RCLCPP_INFO(this->get_logger(), "set_time: time:%f", time);
	if((this->get_clock()->now().nanoseconds() / 1000-find_start)>1000000*time){
		first=true;
		return true;
	}
	else{
		return false;
  	}
}
// 发布定时速度控制指令
// linear_x, linear_y, linear_z 为速度(m/s), angular_z为角速度(度/s)
// send_velocity_command_with_time(3, 0, 0, 0, 3);//前进3m/s,3s后停止
bool OffboardControl::send_velocity_command_with_time(double linear_x, double linear_y, double linear_z, double angular_z,double time){
	static bool first=true;
	static double find_start;
	get_target_location(&linear_x, &linear_y);
	if(first){
		set_target_point("start_temp");
		////set_start_temp_point(pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z, quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w));
		find_start = this->get_clock()->now().nanoseconds() / 1000;
		first=false;
	}
	geometry_msgs::msg::TwistStamped msg;
	if((this->get_clock()->now().nanoseconds() / 1000-find_start)>1000000*time){
		send_velocity_command(0, 0, 0, 0);
		// 或者
		// msg.twist.linear.x = 0;
		// msg.twist.linear.y = 0;
		// msg.twist.linear.z = 0;
		// msg.twist.angular.z = 0;
		// msg.header.stamp = this->now();
		// msg.header.frame_id = "base_link";
		// twist_stamped_publisher_->publish(msg);
		set_target_point("end_temp");
		////set_end_temp_point(pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z, quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w));
		first=true;
		return true;
	}
	else{
		send_velocity_command(linear_x, linear_y, linear_z, angular_z);
		// 或者
		// msg.twist.linear.x = linear_x;
		// msg.twist.linear.y = linear_y;
		// msg.twist.linear.z = linear_z;
		// msg.twist.angular.z = angular_z/2/PI;
		// msg.header.stamp = this->now();
		// msg.header.frame_id = "base_link";
		// twist_stamped_publisher_->publish(msg);
		return false;
  	}
}
// 发布速度控制指令
// linear_x, linear_y, linear_z 为速度(m/s), angular_z为角速度(度/s)
//
// ros2 topic pub /mavros/setpoint_velocity/cmd_vel geometry_msgs/msg/TwistStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}'
void OffboardControl::send_velocity_command(double linear_x, double linear_y, double linear_z, double angular_z)
{
  geometry_msgs::msg::TwistStamped msg;
  msg.twist.linear.x = linear_x;
  msg.twist.linear.y = linear_y;
  msg.twist.linear.z = linear_z;
  msg.twist.angular.z = angular_z/2/PI;
  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";
  twist_stamped_publisher_->publish(msg);
}
// 发布全局位置控制指令(无效)
// send_gps_setpoint_command(latitude, longitude, altitude);
// 
// ros2 topic pub /mavros/setpoint_position/global geographic_msgs/msg/GeoPoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, pose: {position: {latitude: 0.0, longitude: 0.0, altitude: 0.0}}}'
// publishing #6: geographic_msgs.msg.GeoPoseStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id='base_link'), pose=geographic_msgs.msg.GeoPose(position=geographic_msgs.msg.GeoPoint(latitude=0.0, longitude=0.0, altitude=0.0), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)))

void OffboardControl::send_gps_setpoint_command(double latitude, double longitude, double altitude){
	geographic_msgs::msg::GeoPoseStamped msg;
  	msg.pose.position.latitude = latitude;
	msg.pose.position.longitude = longitude;
	msg.pose.position.altitude = altitude;
	msg.header.stamp = this->now();
	msg.header.frame_id = "base_link";
	global_gps_publisher_->publish(msg);
}
// 发布本地位置控制指令
// send_local_setpoint_command(x, y, z, yaw(未完成)); 飞行到
// 飞行到相对于世界坐标系的(x,y,z)位置
void OffboardControl::send_local_setpoint_command(double x, double y, double z,double yaw){
	geometry_msgs::msg::PoseStamped msg;
	yaw = yaw/2/PI;
  	msg.pose.position.x = x;
	msg.pose.position.y = y;
	msg.pose.position.z = z;
	msg.pose.orientation.x = 0;
	msg.pose.orientation.y = 0;
	msg.pose.orientation.z = 0;
	msg.pose.orientation.w = 0;
	msg.header.stamp = this->now();
	msg.header.frame_id = "base_link";
	local_setpoint_publisher_->publish(msg);
}
// 设置本地目标点(无效)
// publish_setpoint_raw_local(x, y, z, yaw);
// 
// ros2 topic pub /mavros/setpoint_raw/local mavros_msgs/msg/PositionTarget '{header: "auto", position: {x: 1.0, y: 2.0, z: 3.0}, yaw: 0.0}'
// publishing #15: mavros_msgs.msg.PositionTarget(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1713970314, nanosec=464943418), frame_id=''), coordinate_frame=0, type_mask=0, position=geometry_msgs.msg.Point(x=1.0, y=2.0, z=3.0), velocity=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), acceleration_or_force=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), yaw=0.0, yaw_rate=0.0)

void OffboardControl::publish_setpoint_raw_local(double x, double y, double z, double yaw) {
    mavros_msgs::msg::PositionTarget msg;
    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;
	msg.yaw = yaw;
    //msg.yaw = tf2::getYaw(pose.pose.orientation);
	msg.velocity.x = max_vx;
	msg.velocity.y = max_vy;
	msg.velocity.z = max_vz;
	msg.yaw_rate = max_yaw;
	//msg.acceleration_or_force.x = 0;
	//msg.acceleration_or_force.y = 0;
	//msg.acceleration_or_force.z = 0;
	msg.header.stamp = this->now();
	msg.header.frame_id = "base_link";
    setpoint_raw_local_publisher_->publish(msg);
}
// 发布全局位置控制指令
// publish_setpoint_raw_global(latitude, longitude, altitude);
// ros2 topic pub /mavros/setpoint_raw/global mavros_msgs/msg/GlobalPositionTarget '{header: "auto", pose: {position: {x: 1.0, y: 2.0, z: 3.0}}}'
// ros2 topic pub /mavros/setpoint_raw/global mavros_msgs/msg/GlobalPositionTarget '{header: "auto", latitude: 1.0, longitude: 2.0, altitude: 3.0}'
//
//publishing : mavros_msgs.msg.GlobalPositionTarget(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1713969082, nanosec=321245648), frame_id=''), coordinate_frame=0, type_mask=0, latitude=1.0, longitude=2.0, altitude=3.0, velocity=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), acceleration_or_force=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), yaw=0.0, yaw_rate=0.0)

//ros2 topic pub /mavros/setpoint_raw/global mavros_msgs/msg/GlobalPositionTarget '{header: "auto", latitude: -35.363262, longitude: 149.165237, altitude: 700.788637}'


void OffboardControl::publish_setpoint_raw_global(double latitude, double longitude, double altitude, double yaw){
	mavros_msgs::msg::GlobalPositionTarget msg;
	msg.coordinate_frame = mavros_msgs::msg::GlobalPositionTarget::FRAME_GLOBAL_INT;
	//mavros_msgs::msg::GlobalPositionTarget::FRAME_GLOBAL_INT;
	msg.latitude = latitude;
	msg.longitude = longitude;
	msg.altitude = altitude;
	msg.yaw = yaw;
	msg.velocity.x = 0;
	msg.velocity.y = 0;
	msg.velocity.z = 0;
	msg.yaw_rate = 0;
	msg.acceleration_or_force.x = 0;
	msg.acceleration_or_force.y = 0;
	msg.acceleration_or_force.z = 0;
	
	msg.header.stamp = this->now();
	msg.header.frame_id = "";
	setpoint_raw_global_publisher_->publish(msg);
}
// 发布位置控制指令
// send_setpoint_command(x, y, z, yaw);
// 无效
void OffboardControl::publish_trajectory(double x, double y, double z, double yaw)
{
    trajectory_msgs::msg::MultiDOFJointTrajectory msg;
	yaw=yaw/2/PI;
    msg.header.stamp = this->now();
	msg.header.frame_id = "base_link";
	msg.joint_names.push_back("base_link");
	msg.points.resize(1);
	msg.points[0].transforms.resize(1);
	msg.points[0].transforms[0].translation.x = x;
	msg.points[0].transforms[0].translation.y = y;
	msg.points[0].transforms[0].translation.z = z;
	msg.points[0].transforms[0].rotation.x = 0;
	msg.points[0].transforms[0].rotation.y = 0;
	msg.points[0].transforms[0].rotation.z = 0;
	msg.points[0].transforms[0].rotation.w = 0;
	msg.points[0].time_from_start = rclcpp::Duration(1, 0);
    trajectory_publisher_->publish(msg);
}

#define PRE_TEST //测试设定位置是否更改，用于循环中，不影响结果
//设置目标点_local
//mode: base_link, start, world, start_temp, end_temp
//x,y,z: 相对于当前位置的偏移量，默认为0m
//yaw: 相对于当前位置的偏转角，默认为0度
//mode为base_link时，设置相对于机体坐标系的目标点，默认不改变位置
//mode为start时，设置相对于起飞点的目标点，默认不改变
//mode为world时，设置绝对坐标系的目标点，默认不改变
//mode为start_temp时，设置相对于结束位置的临时目标点，默认不改变
//mode为end_temp时，设置相对于出发位置的临时目标点，默认为当前位置
//mode为end时，设置相对于降落点的目标点，默认不改变
//set_target_point("base_link",0,0,5,0);//set_target_point("base_link");
//set_target_point("start",0,0,5,0);//set_target_point("start");
//set_target_point("world",0,0,5,0);//set_target_point("world");
//set_target_point("start_temp",0,0,5,0);//set_target_point("start_temp");
//set_target_point("end_temp",0,0,5,0);//set_target_point("end_temp");
//set_target_point("end",0,0,5,0);//set_target_point("end");

void OffboardControl::set_target_point(std::string mode,double x,double y,double z,double yaw){
	#ifdef PRE_TEST
	static double x_pre;
	static double y_pre;
	static double z_pre;
	//static double yaw_pre;
	if(x!=x_pre || y!=y_pre || z!=z_pre){
		x_pre=x;
		y_pre=y;
		z_pre=z;
		//yaw_pre=yaw;
		//mode_pre=mode;
	#endif
		if(mode=="base_link"){
			start_temp=end_temp;
			end_temp.x=start_temp.x+x;
			end_temp.y=start_temp.y+y;
			end_temp.z=start_temp.z+z;
			end_temp.yaw=start_temp.yaw+yaw;
			RCLCPP_INFO(this->get_logger(),"base_link:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw);
		}else if(mode=="start"){
			start_temp=end_temp;
			end_temp.x=start.x+x;
			end_temp.y=start.y+y;
			end_temp.z=start.z+z;
			end_temp.yaw=start.yaw+yaw;
			RCLCPP_INFO(this->get_logger(),"start:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z,end_temp.yaw);
		}else if(mode=="world"){
			start_temp=end_temp;
			end_temp.x=x;
			end_temp.y=y;
			end_temp.z=z;
			end_temp.yaw=yaw;
			RCLCPP_INFO(this->get_logger(),"world:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw);
		}else if(mode=="start_temp"){
			if(x==0.0){start_temp.x=pose_.pose.position.x;}else{start_temp.x=x;}
			if(y==0.0){start_temp.y=pose_.pose.position.y;}else{start_temp.y=y;}
			if(z==0.0){start_temp.z=pose_.pose.position.z;}else{start_temp.z=z;}
			if(yaw==0.0){start_temp.yaw=quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);}else{start_temp.yaw=yaw;}
			RCLCPP_INFO(this->get_logger(),"start_temp:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw);
		}else if(mode=="end_temp"){
			if(x==0.0){end_temp.x=pose_.pose.position.x;}else{end_temp.x=x;}
			if(y==0.0){end_temp.y=pose_.pose.position.y;}else{end_temp.y=y;}
			if(z==0.0){end_temp.z=pose_.pose.position.z;}else{end_temp.z=z;}
			if(yaw==0.0){end_temp.yaw=quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);}else{end_temp.yaw=yaw;}

			RCLCPP_INFO(this->get_logger(),"end_temp:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw);
		}else{
			RCLCPP_ERROR(this->get_logger(), "No such mode");
		}
	#ifdef PRE_TEST
	}else{
		RCLCPP_INFO(this->get_logger(),"set_target_point:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw); 
		return;
	}
	#endif
}

// template <typename T>
// class AtCheckPoint{
// public:
// 	AtCheckPoint(T t):t_(t){}
// 	bool operator()(const T& t){
// 		return t_->at_check_point();
// 	}
// private:
// 	T t_;
// };

// template <typename T>
// void OffboardControl::set_target_point(std::string mode,double x,double y,double z,double yaw,bool(*at_check_point)(const T&)){
// 	#ifdef PRE_TEST
// 	static double x_pre;
// 	static double y_pre;
// 	static double z_pre;
// 	//static double yaw_pre;
// 	if(x!=x_pre || y!=y_pre || z!=z_pre){
// 		x_pre=x;
// 		y_pre=y;
// 		z_pre=z;
// 		//yaw_pre=yaw;
// 		//mode_pre=mode;
// 	#endif
// 		if(mode=="base_link"){
// 			start_temp=end_temp;
// 			end_temp.x=start_temp.x+x;
// 			end_temp.y=start_temp.y+y;
// 			end_temp.z=start_temp.z+z;
// 			end_temp.yaw=start_temp.yaw+yaw;
// 			RCLCPP_INFO(this->get_logger(),"base_link:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw);
// 		}else if(mode=="start"){
// 			start_temp=end_temp;
// 			end_temp.x=start.x+x;
// 			end_temp.y=start.y+y;
// 			end_temp.z=start.z+z;
// 			end_temp.yaw=start.yaw+yaw;
// 			RCLCPP_INFO(this->get_logger(),"start:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z,end_temp.yaw);
// 		}else if(mode=="world"){
// 			start_temp=end_temp;
// 			end_temp.x=x;
// 			end_temp.y=y;
// 			end_temp.z=z;
// 			end_temp.yaw=yaw;
// 			RCLCPP_INFO(this->get_logger(),"world:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw);
// 		}else if(mode=="start_temp"){
// 			if(x==0.0){start_temp.x=pose_.pose.position.x;}else{start_temp.x=x;}
// 			if(y==0.0){start_temp.y=pose_.pose.position.y;}else{start_temp.y=y;}
// 			if(z==0.0){start_temp.z=pose_.pose.position.z;}else{start_temp.z=z;}
// 			if(yaw==0.0){start_temp.yaw=quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);}else{start_temp.yaw=yaw;}
// 			RCLCPP_INFO(this->get_logger(),"start_temp:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw);
// 		}else if(mode=="end_temp"){
// 			if(x==0.0){end_temp.x=pose_.pose.position.x;}else{end_temp.x=x;}
// 			if(y==0.0){end_temp.y=pose_.pose.position.y;}else{end_temp.y=y;}
// 			if(z==0.0){end_temp.z=pose_.pose.position.z;}else{end_temp.z=z;}
// 			if(yaw==0.0){end_temp.yaw=quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);}else{end_temp.yaw=yaw;}

// 			RCLCPP_INFO(this->get_logger(),"end_temp:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw);
// 		}else{
// 			RCLCPP_ERROR(this->get_logger(), "No such mode");
// 		}
// 	#ifdef PRE_TEST
// 	}else{
// 		RCLCPP_INFO(this->get_logger(),"set_target_point:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw); 
// 		return;
// 	}
// 	#endif
// }

// void OffboardControl::set_target_point_global(std::string mode,double x,double y,double z,double yaw){
// 	#ifdef PRE_TEST
// 	static double x_pre;
// 	static double y_pre;
// 	static double z_pre;
// 	//static double yaw_pre;
// 	if(x!=x_pre || y!=y_pre || z!=z_pre){
// 		x_pre=x;
// 		y_pre=y;
// 		z_pre=z;
// 		//yaw_pre=yaw;
// 		//mode_pre=mode;
// 	#endif
// 		if(mode=="base_link"){
// 			start_temp_global=end_temp_global;
// 			end_temp_global.x=start_temp_global.x+x;
// 			end_temp_global.y=start_temp_global.y+y;
// 			end_temp_global.z=start_temp_global.z+z;
// 			end_temp_global.yaw=start_temp_global.yaw+yaw;
// 			RCLCPP_INFO(this->get_logger(),"base_link:%f %f %f %f",end_temp_global.x, end_temp_global.y, end_temp_global.z, end_temp_global.yaw);
// 		}else if(mode=="start"){
// 			start_temp_global=end_temp_global;
// 			end_temp_global.x=start_global.x+x;
// 			end_temp_global.y=start_global.y+y;
// 			end_temp_global.z=start_global.z+z;
// 			end_temp_global.yaw=start_global.yaw+yaw;
// 			RCLCPP_INFO(this->get_logger(),"start_global:%f %f %f %f",end_temp_global.x, end_temp_global.y, end_temp_global.z,end_temp_global.yaw);
// 		}else if(mode=="world"){
// 			start_temp_global=end_temp_global;
// 			end_temp_global.x=x;
// 			end_temp_global.y=y;
// 			end_temp_global.z=z;
// 			end_temp_global.yaw=yaw;
// 			RCLCPP_INFO(this->get_logger(),"world:%f %f %f %f",end_temp_global.x, end_temp_global.y, end_temp_global.z, end_temp_global.yaw);
// 		}else if(mode=="start_temp_global"){
// 			if(x==0.0){start_temp_global.x=pose_.pose.position.x;}else{start_temp_global.x=x;}
// 			if(y==0.0){start_temp_global.y=pose_.pose.position.y;}else{start_temp_global.y=y;}
// 			if(z==0.0){start_temp_global.z=pose_.pose.position.z;}else{start_temp_global.z=z;}
// 			if(yaw==0.0){start_temp_global.yaw=quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);}else{start_temp_global.yaw=yaw;}
// 			RCLCPP_INFO(this->get_logger(),"start_temp_global:%f %f %f %f",end_temp_global.x, end_temp_global.y, end_temp_global.z, end_temp_global.yaw);
// 		}else if(mode=="end_temp_global"){
// 			if(x==0.0){end_temp_global.x=pose_.pose.position.x;}else{end_temp_global.x=x;}
// 			if(y==0.0){end_temp_global.y=pose_.pose.position.y;}else{end_temp_global.y=y;}
// 			if(z==0.0){end_temp_global.z=pose_.pose.position.z;}else{end_temp_global.z=z;}
// 			if(yaw==0.0){end_temp_global.yaw=quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);}else{end_temp_global.yaw=yaw;}

// 			RCLCPP_INFO(this->get_logger(),"end_temp_global:%f %f %f %f",end_temp_global.x, end_temp_global.y, end_temp_global.z, end_temp_global.yaw);
// 		}else{
// 			RCLCPP_ERROR(this->get_logger(), "No such mode");
// 		}
// 	#ifdef PRE_TEST
// 	}else{
// 		RCLCPP_INFO(this->get_logger(),"set_target_point:%f %f %f %f",end_temp_global.x, end_temp_global.y, end_temp_global.z, end_temp_global.yaw); 
// 		return;
// 	}
// 	#endif
// }

//确认到达目标点_local
//accuracy: 精度（默认为DEFAULT_ACCURACY）
//返回值：到达目标点返回true，否则返回false
bool OffboardControl::at_check_point(double accuracy){
	////////RCLCPP_INFO(this->get_logger(),"et:%f %f %f",end_temp.x, end_temp.y, end_temp.z); 
	////////RCLCPP_INFO(this->get_logger(),"vl:%f %f %f",pose_.pose.position.x-end_temp.x, pose_.pose.position.y - end_temp.y, pose_.pose.position.z - end_temp.z); 
	RCLCPP_INFO(this->get_logger()," p: %f %f %f %f",pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z, quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w));
	RCLCPP_INFO(this->get_logger()," e: %f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw);
	if(
		abs(pose_.pose.position.x - end_temp.x) <=accuracy && 
		abs(pose_.pose.position.y - end_temp.y) <=accuracy && 
		abs(pose_.pose.position.z - end_temp.z) <=accuracy 
		&& abs(quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w) - end_temp.yaw) <= DEFAULT_ACCURACY_YAW
		){
		
		return true;
	}
	else{
		return false;
	}	
}
// void OffboardControl::set_target_point(std::string mode,double x,double y,double z,double yaw){
// 	static double x_pre;
// 	static double y_pre;
// 	static double z_pre;
// 	//static double yaw_pre;
// 	if(x!=x_pre || y!=y_pre || z!=z_pre){
// 		x_pre=x;
// 		y_pre=y;
// 		z_pre=z;
// 		//yaw_pre=yaw;
// 		//mode_pre=mode;
// 		if(mode=="base_link"){
// 			set_drone_target_point_local(x,y,z,yaw);
// 			RCLCPP_INFO(this->get_logger(),"base_link:et:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw);
// 		}
// 		else if(mode=="start"){
// 			set_start_point_local(x,y,z,yaw);
// 			RCLCPP_INFO(this->get_logger(),"start:et:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z,end_temp.yaw);
// 		}
// 		else if(mode=="world"){
// 			set_world_point_local(x,y,z,yaw);
// 			RCLCPP_INFO(this->get_logger(),"world:et:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw);
// 		}
// 		else{
// 			RCLCPP_ERROR(this->get_logger(), "No such mode");
// 		}
// 	}else{
// 		RCLCPP_INFO(this->get_logger(),"set_target_point: et:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw); 
// 		return;
// 	}
// }
// void OffboardControl::set_start_temp_point(double x,double y,double z,double yaw){
// 	start_temp.x=x;
// 	start_temp.y=y;
// 	start_temp.z=z;
// 	start_temp.yaw=yaw;
// 	//RCLCPP_INFO(this->get_logger(),"set:et:%f %f %f",end_temp.x, end_temp.y, end_temp.z); 
// }
// void OffboardControl::set_end_temp_point(double x=0.0,double y=0.0,double z=0.0,double yaw=0.0){
//     if(x==0.0){end_temp.x=pose_.pose.position.x;}else{end_temp.x=x;}
//     if(y==0.0){end_temp.y=pose_.pose.position.y;}else{end_temp.y=y;}
//     if(z==0.0){end_temp.z=pose_.pose.position.z;}else{end_temp.z=z;}
//     if(yaw==0.0){end_temp.yaw=quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);}else{end_temp.yaw=yaw;}
// 	RCLCPP_INFO(this->get_logger(),"set:et:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw); 
// }

// void OffboardControl::set_drone_target_point_local(double x,double y,double z,double yaw){
// 	start_temp=end_temp;
// 	end_temp.x=start_temp.x+x;
// 	end_temp.y=start_temp.y+y;
// 	end_temp.z=start_temp.z+z;
// 	end_temp.yaw=start_temp.yaw+yaw;
	
// 	RCLCPP_INFO(this->get_logger(),"set:drone_et:%f %f %f",end_temp.x, end_temp.y, end_temp.z); 
// }

// void OffboardControl::set_start_point_local(double x,double y,double z,double yaw){
// 	start_temp=end_temp;
// 	end_temp.x=start.x+x;
// 	end_temp.y=start.y+y;
// 	end_temp.z=start.z+z;
// 	end_temp.yaw=start.yaw+yaw;
	
// 	//RCLCPP_INFO(this->get_logger(),"set:et:%f %f %f",end_temp.x, end_temp.y, end_temp.z); 
// }

// void OffboardControl::set_world_point_local(double x,double y,double z,double yaw){
// 	start_temp=end_temp;
// 	end_temp.x=x;
// 	end_temp.y=y;
// 	end_temp.z=z;
// 	end_temp.yaw=yaw;
	
// 	//RCLCPP_INFO(this->get_logger(),"set:et:%f %f %f",end_temp.x, end_temp.y, end_temp.z); 
// }

// bool OffboardControl::at_check_point(double accuracy){
// 	////////RCLCPP_INFO(this->get_logger(),"et:%f %f %f",end_temp.x, end_temp.y, end_temp.z); 
// 	////////RCLCPP_INFO(this->get_logger(),"vl:%f %f %f",pose_.pose.position.x-end_temp.x, pose_.pose.position.y - end_temp.y, pose_.pose.position.z - end_temp.z); 
// 	RCLCPP_INFO(this->get_logger()," p: %f %f %f %f",pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z, quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w));
// 	RCLCPP_INFO(this->get_logger()," e: %f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw);
// 	if(
// 		abs(pose_.pose.position.x - end_temp.x) <=accuracy && 
// 		abs(pose_.pose.position.y - end_temp.y) <=accuracy && 
// 		abs(pose_.pose.position.z - end_temp.z) <=accuracy 
// 		&& abs(quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w) - end_temp.yaw) <= DEFAULT_ACCURACY_YAW
// 		){
		
// 		return true;
// 	}
// 	else{
// 		return false;
// 	}	
// }

double OffboardControl::quaternion_to_yaw(double x, double y, double z, double w) {
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return yaw/PI*180;
}
void OffboardControl::yaw_to_quaternion(double yaw, double* x, double* y, double* z, double* w) {
	*x = 0.0;
	*y = 0.0;
	*z = sin(yaw / 2);
	*w = cos(yaw / 2);
}
// #x,y为目标点相对于当前位置的偏移量
void OffboardControl::get_target_location( double* x, double* y,double delta_heading) {
    // Assuming that 'heading' is a member variable of the class representing the current heading
	RCLCPP_INFO(get_logger(), "default_heading: %f", default_heading);
	double yaw = heading + default_heading + delta_heading;
	if (yaw >= 360) {
		heading -= 360;
	}
	RCLCPP_INFO(get_logger(), "yaw: %f", yaw);
    // Convert yaw to radians
    double yaw_rad = yaw / 180 * PI;
	RCLCPP_INFO(get_logger(), "yaw_rad: %f", yaw_rad);
    // Assuming that 'distance' is the distance you want to move in the direction of 'yaw'
    //double distance = sqrt(*x**x+*y**y); // replace with your actual distance
	//RCLCPP_INFO(get_logger(), "distance: %f", distance);
    // Calculate the new position
	double x1=*x;
	double y1=*y;
    *x = x1 * cos(yaw_rad)-y1 * sin(yaw_rad);//
    *y = x1 * sin(yaw_rad)+y1 * cos(yaw_rad);//
	RCLCPP_INFO(get_logger(), "cos(yaw_rad): %f, sin(yaw_rad): %f", cos(yaw_rad), sin(yaw_rad));
	RCLCPP_INFO(get_logger(), "ldx: %f, ldy: %f", *x, *y);
}
// #将相对坐标系下的坐标 x:m,y:m,z:m,heading:degree
// 转换为全局坐标系下的经纬度坐标
//get_target_location(default_heading, );
//ros2 topic pub /mavros/setpoint_raw/global mavros_msgs/msg/GlobalPositionTarget '{header: "auto", latitude: -35.363262, longitude: 149.165237, altitude: 700.788637}'
void OffboardControl::get_target_location_global(double &x,double &y,double &z,double delta_heading) {
    double yaw = heading+default_heading+delta_heading;

    if(heading >= 360){
        heading -= 360;
	}
	std::cout << "heading: " << yaw << std::endl;
    
    double yaw_rad = yaw/180*PI;

	double x1=x;
	double y1=y;
	//double z1=z;
	// #change in lat and lon  
    double d_x = (x1 * cos(yaw_rad)-y1 * sin(yaw_rad)) ;//* 0.0000093;//
    double d_y = (x1 * sin(yaw_rad)+y1 * cos(yaw_rad)) ;//* 0.000009;//
	double d_z =  z ;//* 0.3048;
	//std::cout << "x: " << x << " y: " << y << " z: " << z << std::endl;
	//std::cout << "lat: " << location.global_frame.lat << " lon: " << location.global_frame.lon << " alt: " << location.global_frame.alt << std::endl;

	add_flu_offset_to_llh(location.global_frame.lat, location.global_frame.lon, location.global_frame.alt,d_x, d_y, d_z,  x, y, z);;

	// std::cout << "lat: " << x << " lon: " << y << " alt: " << z << std::endl;
	// xn: 0, yn: 100, zn: 10
	// lat_new: 0.000904368, lon_new: 0, alt_new: 10.0008
}
// #将相对坐标系下的坐标 x:m,y:m,z:m,heading:degree
// 转换为全局坐标系下的经纬度坐标
//get_target_location(default_heading
// OffboardControl::GlobalFrame OffboardControl::get_target_location(double dheading,double x,OffboardControl &offboard_control){
//     double heading = offboard_control.heading;
//     double alt = offboard_control.location.global_frame.alt + 5;
//     heading += dheading;
//     if (heading >= 360){
//         heading -= 360;
// 	}
    
//     double heading_radians = heading * M_PI / 180;

//     //change in lat and lon    
//     double dlon = x * sin(heading_radians) * 0.0000093;
//     double dlat = x * cos(heading_radians) * 0.000009;
    
//     // print("dlat: ", dlat, "dlon: ", dlon)

//     GlobalFrame target_location = {offboard_control.location.global_frame.lat + dlat, offboard_control.location.global_frame.lon + dlon, alt};
    
//     return target_location;
// }

void OffboardControl::timer_callback(void){
	static uint8_t num_of_steps = 0;
	// RCLCPP_INFO(this->get_logger(), "timer_callback");
	// RCLCPP_INFO(this->get_logger(), "arm_done: %d", arm_done_);
	//switch_to_autotune_mode();
	// offboard_control_mode needs to be paired with trajectory_setpoint
	//publish_offboard_control_mode();
	//trajectory_setpoint_global(0, 0, 100, 0);
	//PrintYaw();
	//RCLCPP_INFO(this->get_logger(), "yaw: %lf", quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w));
	GlobalFrame a{0,0,10,0}; get_target_location_global(a.lat, a.lon, a.alt, a.yaw);
	std::cout << "alt: " << location.global_frame.alt << " lon " << location.global_frame.lon << " alt: " << location.global_frame.alt  << std::endl;

	RCLCPP_INFO(this->get_logger(), "global_gps: %lf %lf %lf", location.global_frame.lat, location.global_frame.lon, location.global_frame.alt);
	switch (fly_state_)
	{
	case FlyState::init:
		rclcpp::sleep_for(1s);
		if(pose_.header.stamp.sec == 0 || global_gps_.header.stamp.sec == 0){
			RCLCPP_INFO(this->get_logger(), "No pose data received yet");
			break;
		}
		timestamp0 = this->get_clock()->now().nanoseconds() / 1000;
		RCLCPP_INFO(this->get_logger(), "timestamp0= %f ,\ntimestamp-timestamp0=%f", timestamp0, this->get_clock()->now().nanoseconds() - timestamp0);
		start = {pose_.pose.position.x,pose_.pose.position.y,pose_.pose.position.z,quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w)};
		start_global = {global_gps_.latitude, global_gps_.longitude, global_gps_.altitude, 0};
		end_temp=start;
		heading=start.yaw;
		RCLCPP_INFO(this->get_logger(), "yaw: %f", heading);
		// shot_area_start = get_target_location_global(0, 0);
		// scout_area_start = get_target_location_global(0, 30);
		
		//RCLCPP_INFO(this->get_logger()," start: x: %f  y: %f  z: %f", start.x, start.y,  start.z);
		fly_state_ = FlyState::takeoff;
		break;
	// case FlyState::request:
	// 	if (arm_done_){
	// 		fly_state_ = FlyState::takeoff;
	// 	}
	// 	break;
	case FlyState::takeoff:
		//RCLCPP_INFO(this->get_logger(), "takeoff start");
		// command_takeoff_or_land("TAKEOFF");
		// rclcpp::sleep_for(1s);
		//RCLCPP_INFO(this->get_logger(), "pose_.pose.position.z-start.z=%lf", pose_.pose.position.z-start.z);
		//if (true){//pose_.pose.position.z>1+start.z){
		//if(arm_done_){	
			command_takeoff_or_land("TAKEOFF");
			
		if (pose_.pose.position.z>2.5+start.z){
		// if (false){
			RCLCPP_INFO(this->get_logger(), "goto_shot_area start, totaltime=%fs", (this->get_clock()->now().nanoseconds() / 1000- timestamp0)/1000000.0);
			fly_state_ = FlyState::goto_shot_area;
		}
		else{
			send_velocity_command(0, 0, 0.5, 0);

			// publish_setpoint_raw_global(global_gps_.latitude, global_gps_.longitude,global_gps_.altitude, 0);
			// trajectory_setpoint_global(0 , 0, 0, 0);
		}
		// 	if (false){//pose_.pose.position.z>2.5+start.z){
		// 		RCLCPP_INFO(this->get_logger(), "goto_shot_area start, totaltime=%fs", (this->get_clock()->now().nanoseconds() / 1000- timestamp0)/1000000.0);
		// 		fly_state_ = FlyState::goto_shot_area;
		// 	}
		// 	else{
		// 		//send_velocity_command(0, 0, 0.5, 0);
		// 		//send_local_setpoint_command(0, 0, 10,0);
		// 		//publish_setpoint_raw_global(global_gps_.latitude, global_gps_.longitude,global_gps_.altitude+10);
		// 		//publish_setpoint_raw_local(0, 0, 10, 0);
		// 		//publish_setpoint_raw_global(global_gps_.latitude, global_gps_.longitude, global_gps_.altitude+10, 0);
		// 		//publish_setpoint_raw_global(0, 1, 2, 0);
		// 		//publish_trajectory(0, 0, 10, 0);
				
		// 		publish_setpoint_raw_global(location.global_frame.lat,location.global_frame.lon, location.global_frame.alt+100, location.global_frame.yaw);
		// 	}
			rclcpp::sleep_for(2s);
		//}
		break;
	case FlyState::goto_shot_area:
		//trajectory_setpoint(53, 10, 0, 0);
		trajectory_setpoint_start(53, 10, 5, 0);
		if (at_check_point()){
			fly_state_ = FlyState::findtarget;
			RCLCPP_INFO(this->get_logger(), "findtarget start, totaltime=%fs", (this->get_clock()->now().nanoseconds() / 1000- timestamp0)/1000000.0);
		}
		else{
			
		}
		break;
	case FlyState::findtarget:
		if(surrending_shot_area()){
			fly_state_ = FlyState::goto_scout_area;
			RCLCPP_INFO(this->get_logger(), "findtarget done,goto_scout_area start totaltime=%fs", (this->get_clock()->now().nanoseconds() / 1000- timestamp0)/1000000.0);
		}
		break;
	case FlyState::goto_scout_area:
		trajectory_setpoint_start(53+30, 0, 5, 0);

		if(at_check_point()){
			fly_state_ = FlyState::scout;
			RCLCPP_INFO(this->get_logger(), "goto_scout_area done,scout start totaltime=%fs", (this->get_clock()->now().nanoseconds() / 1000- timestamp0)/1000000.0);
		}
		break;
	case FlyState::scout:
		if(surrending_scout_area()){
			fly_state_ = FlyState::land;
			RCLCPP_INFO(this->get_logger(), "scout done,land start totaltime=%fs", (this->get_clock()->now().nanoseconds() / 1000- timestamp0)/1000000.0);	
			switch_to_rtl_mode();
			rclcpp::sleep_for(10s);
		}
		break;
	case FlyState::land:
		
		switch_to_guided_mode();
		trajectory_setpoint_start(0, 0, 1, 0);
		if(at_check_point(0.1)){
			send_velocity_command(0, 0, 0, 0);
			command_takeoff_or_land("LAND");
			fly_state_ = FlyState::end;
			RCLCPP_INFO(this->get_logger(), "land done,end start totaltime=%fs", (this->get_clock()->now().nanoseconds() / 1000- timestamp0)/1000000.0);
		}
		break;
	case FlyState::end:
		RCLCPP_INFO(this->get_logger(), "end done, totaltime=%fs", (this->get_clock()->now().nanoseconds() / 1000- timestamp0)/1000000.0);
		command_takeoff_or_land("LAND");
		rclcpp::sleep_for(3s);
		//rclcpp::shutdown();
		break;
	default:
		break;
	}

	
	//publish_trajectory_setpoint(0, 0, 10, 0);
	switch (state_)
	{
	case State::init :
		RCLCPP_INFO(this->get_logger(), "Entered guided mode");
			//global_gps_publisher_->publish(global_gps_start);
			

		
		state_ = State::send_geo_grigin;
		break;
	case State::send_geo_grigin :
		
			//global_gps_start=global_gps_;
			//global_gps_start=global_gps_;
			switch_to_guided_mode();
			state_ = State::wait_for_stable_offboard_mode;				
		
		break;
	case State::wait_for_stable_offboard_mode :
		if (++num_of_steps>10){
			arm_motors(true);
			state_ = State::arm_requested;
		}
		break;
	case State::arm_requested :
		if(arm_done_){
			
			//RCLCPP_INFO(this->get_logger(), "vehicle is armed");
			//switch_to_auto_mode();

			// state_ = State::takeoff;
		}
		else{
			rclcpp::sleep_for(1s);
			arm_motors(true);
			//service_done_ = false;
		}
		break;
	case State::takeoff:
		//arm_done_ = true;
		// rclcpp::sleep_for(1s);
			//RCLCPP_INFO(this->get_logger(), "vehicle is start");
			
			//arm_motors(true);
			
			//publish_global_gps(global_gps_.pose.position.latitude, global_gps_.pose.position.longitude, 600.0);
			
			//arm_motors(true);
			command_takeoff_or_land("TAKEOFF");
			state_ = State::autotune_mode;

		break;
	case State::autotune_mode:

	default:
		break;
	}
	//RCLCPP_INFO(this->get_logger(), "State: %d", vehicle.pose_.header.stamp.sec);
	//RCLCPP_INFO(this->get_logger(), "State: %d", vehicle.pose_.header.stamp.nanosec);
}


class Yolo : public rclcpp::Node{
public:
	Yolo()
	: Node("yolo")
	{
		// sub_ = this->create_subscription<sensor_msgs::msg::Image>(
		// 	"/camera/image_raw", 10, std::bind(&Yolo::image_callback, this, std::placeholders::_1));
	}

};


int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

	rclcpp::executors::MultiThreadedExecutor executor;
	auto node = std::make_shared<OffboardControl>("/mavros/");
	auto node1 = std::make_shared<Yolo>();
	// auto pubnode = std::make_shared<PublisherNode>();
	/* 运行节点，并检测退出信号*/
	executor.add_node(node);
	executor.add_node(node1);
	executor.spin();
	// rclcpp::spin(std::make_shared<OffboardControl>("/mavros/"));
	rclcpp::shutdown();
	return 0;
}


