#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol_local.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
//#include <ardupilot_msgs/msg/global_position.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <string>
#include <cmath>

#define PI 3.14
//#define delta_heading -0.044156
//#define delta_heading -60  //yaw=-2.53
//M_PI dhaeding: 0.000000
#define DEFAULT_ACCURACY 0.3 //+-0.3m
#define DEFAULT_ACCURACY_YAW 1 //+-1°
#define DELTA_HEADING 0
//using namespace std::chrono;
using namespace std::chrono_literals;

#include <iostream>
#include <thread>

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl(std::string ardupilot_namespace) :
		Node("offboard_control_srv"),
		state_{State::init},
		fly_state_{FlyState::init},
		service_result_{0},
		service_done_{false},
		//global_gps_publisher_{this->create_publisher<ardupilot_msgs::msg::GlobalPosition>(ardupilot_namespace_+"cmd_gps_pose", 5)},
		twist_stamped_publisher_{this->create_publisher<geometry_msgs::msg::TwistStamped>(ardupilot_namespace+"setpoint_velocity/cmd_vel", 5)},
		arm_motors_client_{this->create_client<mavros_msgs::srv::CommandBool>(ardupilot_namespace+"cmd/arming")},
		mode_switch_client_{this->create_client<mavros_msgs::srv::SetMode>(ardupilot_namespace+"set_mode")}
		
	{
		ardupilot_namespace_ = ardupilot_namespace;
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		
		pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(ardupilot_namespace_+"local_position/pose", qos,
		std::bind(&OffboardControl::pose_callback, this, std::placeholders::_1));
		gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(ardupilot_namespace_+"global_position/global", qos,
		std::bind(&OffboardControl::gps_callback, this, std::placeholders::_1));
		velocity_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(ardupilot_namespace_+"local_position/velocity_local", qos,
		std::bind(&OffboardControl::velocity_callback, this, std::placeholders::_1));

		RCLCPP_INFO(this->get_logger(), "Starting Offboard Control example with PX4 services");
		//RCLCPP_INFO_STREAM(geometry_msgs::msg::PoseStampedthis->get_logger(), "Waiting for " << px4_namespace << "vehicle_command service");
		while (!mode_switch_client_->wait_for_service(std::chrono::seconds(2))) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
				return;
			}
			RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  		}
		timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));
	}

  void switch_mode(std::string mode);
	void arm_motors(bool arm);
	 double quaternion_to_yaw(double x, double y, double z, double w);
	 void yaw_to_quaternion(double yaw, double* x, double* y, double* z, double* w);
private:
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
		request,
		takeoff,
		goto_shot_area,
		findtarget,
		goto_scout_area,
		scout,
		land,
		end
	} fly_state_;
	std::string ardupilot_namespace_;
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
	double max_vx=0.05; //前后方向最大速度
	double max_vy=0.05; //左右方向最大速度
	double max_vz=0.05; //上下方向最大速度
	double max_yaw=1; //最大角速度(°/s)

	float delta_heading=DELTA_HEADING;//-2;

	double timestamp0;
	struct Point{
		public:
		float x;
		float y;
		float z;
		float yaw;
	};
	Point start = {0,0,0,0};
	Point start_temp= {0,0,0,0};
	Point end_temp= {0,0,0,0};
	Point end= {0,0,0,0};
	float heading;

	rclcpp::TimerBase::SharedPtr timer_;

	geometry_msgs::msg::PoseStamped pose_{};
	sensor_msgs::msg::NavSatFix global_gps_{};
	geometry_msgs::msg::TwistStamped velocity_{};
	//sensor_msgs::msg::NavSatFix global_gps_start{};
	//ardupilot_msgs::msg::GlobalPosition global_gps_start{};

	//rclcpp::Publisher<ardupilot_msgs::msg::GlobalPosition>::SharedPtr global_gps_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_publisher_;
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
	
	rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_motors_client_;
	rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_switch_client_;
	rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscription_;
	
	void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
	void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

	void switch_to_guided_mode();
	void switch_to_flip_mode();
	void switch_to_rtl_mode();
	// void switch_to_auto_mode();
	//void publish_global_gps(double latitude, double longitude, double altitude);
	
	void command_takeoff_or_land_local(std::string mode);
	void command_takeoff_or_land(std::string mode);
	
	void trajectory_setpoint_takeoff(float x,float y ,float z ,float yaw);
	void trajectory_setpoint(float x,float y ,float z ,float yaw, float accuracy=DEFAULT_ACCURACY);
	void trajectory_setpoint_start(float x,float y ,float z ,float yaw,float accuracy=DEFAULT_ACCURACY);
	bool alt_hold(float vx,float vy ,float z ,float yaw,float time,float accuracy=DEFAULT_ACCURACY);
	void publish_trajectory_setpoint(float x,float y ,float z ,float yaw);
	bool publish_trajectory_setpoint_z(float *x,float accuracy=DEFAULT_ACCURACY);
	bool publish_trajectory_setpoint_yaw(float *yaw,float accuracy=DEFAULT_ACCURACY_YAW);
	bool send_velocity_command_with_time(float linear_x, float linear_y, float linear_z, float angular_z, double time);
	void send_velocity_command(double linear_x, double linear_y, double linear_z, double angular_z);
	bool set_time(float time);

	bool surrending_shot_area(void);
	bool surrending_scout_area(void);

	void set_target_point(std::string mode,float x,float y,float z,float yaw);
	void set_start_temp_point(float x,float y,float z,float yaw);
	void set_end_temp_point(float x,float y,float z,float yaw);
	void set_drone_target_point_local(float x,float y,float z,float yaw);
	void set_start_point_local(float x,float y,float z,float yaw);
	void set_world_point_local(float x,float y,float z,float yaw);
	bool at_check_point(float accuracy=DEFAULT_ACCURACY);	

	void get_target_location(float delta_heading, float* x, float* y);

	void timer_callback(void);
};

void OffboardControl::switch_to_guided_mode(){
	RCLCPP_INFO(this->get_logger(), "requesting switch to GUIDED mode");
	OffboardControl::switch_mode("GUIDED");
}

void OffboardControl::switch_to_flip_mode(){
	RCLCPP_INFO(this->get_logger(), "requesting switch to FLIP mode");
	OffboardControl::switch_mode("FLIP");
}

void OffboardControl::switch_to_rtl_mode(){
	RCLCPP_INFO(this->get_logger(), "requesting switch to RTL mode");
	OffboardControl::switch_mode("RTL");
}

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
					arm_done_ = false;
					RCLCPP_ERROR(this->get_logger(), ("Failed to call service " + ardupilot_namespace_ + "cmd/arming").c_str());
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

void OffboardControl::command_takeoff_or_land_local(std::string mode)
{
	std::string mode_str(mode);
	if(mode=="TAKEOFF"){
	auto takeoff_client = this->create_client<mavros_msgs::srv::CommandTOLLocal>(ardupilot_namespace_+"cmd/takeoff_local");

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
	auto land_client = this->create_client<mavros_msgs::srv::CommandTOLLocal>(ardupilot_namespace_+"cmd/land_local");

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

// float32 min_pitch		# used by takeoff
// float32 offset    		# used by land (landing position accuracy)
// float32 rate			# speed of takeoff/land in m/s
// float32 yaw			# in radians
// geometry_msgs/Vector3 position 	#(x,y,z) in meters
// ---
// bool success
// uint8 result
void OffboardControl::command_takeoff_or_land(std::string mode)
{
	//RCLCPP_ERROR(this->get_logger(), ("Failed to call service "+ardupilot_namespace_+"cmd/takeoff").c_str());
	service_done_ = false;
	std::string mode_str(mode);
	if(mode=="TAKEOFF"){
	auto takeoff_client = this->create_client<mavros_msgs::srv::CommandTOL>(ardupilot_namespace_+"cmd/takeoff");

	auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
	takeoff_request->min_pitch = 0.0;
	takeoff_request->yaw = 0.0;
	takeoff_request->latitude = 0.0;
	takeoff_request->longitude = 0.0;
	takeoff_request->altitude = 2.0;
	
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
					RCLCPP_ERROR(this->get_logger(), ("Failed to call service "+ardupilot_namespace_+"cmd/takeoff").c_str());
				}
			} else {
				// Wait for the result.
				RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
			}
		});
	} else if(mode=="LAND"){
	auto land_client = this->create_client<mavros_msgs::srv::CommandTOL>(ardupilot_namespace_+"cmd/land");
	
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
					RCLCPP_ERROR(this->get_logger(), ("Failed to call service "+ardupilot_namespace_+"cmd/land").c_str());
				}
			} else {
				// Wait for the result.
				RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
			}
		});
	}
}

void OffboardControl::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
{
	pose_ = *msg;
	// std::cout << "position x: " << msg->pose.position.x << std::endl;
	// std::cout << "position y: " << msg->pose.position.y  << std::endl;
	// std::cout << "position z: " << msg->pose.position.z  << std::endl;
	//RCLCPP_INFO(this->get_logger(), "Received yaw: %f", quaternion_to_yaw(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w));
}
/*
std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
std::cout << "RECEIVED SENSOR COMBINED DATA"   << std::endl;
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
void OffboardControl::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) 
{
	global_gps_ = *msg;
	
	// RCLCPP_INFO(this->get_logger(), "Received GPS data");
	// RCLCPP_INFO(this->get_logger(), "Latitude: %f", global_gps_.latitude);
	// RCLCPP_INFO(this->get_logger(), "Longitude: %f", global_gps_.longitude);
	// RCLCPP_INFO(this->get_logger(), "Altitude: %f", global_gps_.altitude);
	
}
void OffboardControl::velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) 
{
	velocity_= *msg;
	// RCLCPP_INFO(this->get_logger(), "Received velocity data");
	// RCLCPP_INFO(this->get_logger(), "Linear x: %f", msg->twist.linear.x);
	// RCLCPP_INFO(this->get_logger(), "Linear y: %f", msg->twist.linear.y);
	// RCLCPP_INFO(this->get_logger(), "Linear z: %f", msg->twist.linear.z);
	// RCLCPP_INFO(this->get_logger(), "Angular z: %f", msg->twist.angular.z);
}
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
		static float fx=1,fy=2*fx;
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
void OffboardControl::trajectory_setpoint_takeoff(float x,float y ,float z ,float yaw){
	set_target_point("world",start_temp.x+x,start_temp.y+y,z,yaw);
	RCLCPP_INFO(this->get_logger(),"trajectory_setpoint_takeoff: et:%f %f %f",end_temp.x, end_temp.y, end_temp.z); 
	if(pose_.header.stamp.sec == 0){
		RCLCPP_INFO(this->get_logger(), "No pose data received yet");
		return;
	}
	publish_trajectory_setpoint(end_temp.x,end_temp.y,z,yaw);
}
void OffboardControl::trajectory_setpoint(float x,float y ,float z ,float yaw,float accuracy){
	static bool first=true;
	if(first){
		get_target_location(delta_heading, &x, &y);
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
void OffboardControl::trajectory_setpoint_start(float x,float y ,float z ,float yaw,float accuracy){
	static bool first=true;
	if(first){
	get_target_location(delta_heading, &x, &y);
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
// void OffboardControl::alt_hold(float vx,float vy ,float z ,float yaw,float time,float accuracy){
// task finish return true;
bool OffboardControl::alt_hold(float vx,float vy ,float z ,float yaw,float time,float accuracy){
	static bool first=true;
	static float vx1=0,vy1=0;
	if(first){
	get_target_location(delta_heading, &vx, &vy);
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
			publish_trajectory_setpoint_z(&z,accuracy)&
			publish_trajectory_setpoint_yaw(&yaw)
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
void OffboardControl::publish_trajectory_setpoint(float x,float y ,float z ,float yaw){
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
bool OffboardControl::publish_trajectory_setpoint_z(float *z,float accuracy){
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
bool OffboardControl::publish_trajectory_setpoint_yaw(float *yaw,float accuracy){
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
bool OffboardControl::set_time(float time){
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
bool OffboardControl::send_velocity_command_with_time(float linear_x, float linear_y, float linear_z, float angular_z,double time){
	static bool first=true;
	static double find_start;
	get_target_location(delta_heading, &linear_x, &linear_y);
	if(first){
		set_start_temp_point(pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z, quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w));
		find_start = this->get_clock()->now().nanoseconds() / 1000;
		first=false;
	}
	geometry_msgs::msg::TwistStamped msg;
	if((this->get_clock()->now().nanoseconds() / 1000-find_start)>1000000*time){
		msg.twist.linear.x = 0;
		msg.twist.linear.y = 0;
		msg.twist.linear.z = 0;
		msg.twist.angular.z = 0;
		msg.header.stamp = this->now();
		msg.header.frame_id = "base_link";
		twist_stamped_publisher_->publish(msg);
		set_end_temp_point(pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z, quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w));
		first=true;
		return true;
	}
	else{
		msg.twist.linear.x = linear_x;
		msg.twist.linear.y = linear_y;
		msg.twist.linear.z = linear_z;
		msg.twist.angular.z = angular_z/2/PI;
		msg.header.stamp = this->now();
		msg.header.frame_id = "base_link";
		twist_stamped_publisher_->publish(msg);
		return false;
  	}
}
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
void OffboardControl::set_target_point(std::string mode,float x,float y,float z,float yaw){
	static float x_pre;
	static float y_pre;
	static float z_pre;
	//static float yaw_pre;
	if(x!=x_pre || y!=y_pre || z!=z_pre){
		x_pre=x;
		y_pre=y;
		z_pre=z;
		//yaw_pre=yaw;
		//mode_pre=mode;
		if(mode=="base_link"){
			set_drone_target_point_local(x,y,z,yaw);
			RCLCPP_INFO(this->get_logger(),"base_link:et:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw);
		}
		else if(mode=="start"){
			set_start_point_local(x,y,z,yaw);
			RCLCPP_INFO(this->get_logger(),"start:et:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z,end_temp.yaw);
		}
		else if(mode=="world"){
			set_world_point_local(x,y,z,yaw);
			RCLCPP_INFO(this->get_logger(),"world:et:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw);
		}
		else{
			RCLCPP_ERROR(this->get_logger(), "No such mode");
		}
	}else{
		RCLCPP_INFO(this->get_logger(),"set_target_point: et:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw); 
		return;
	}
}
void OffboardControl::set_start_temp_point(float x,float y,float z,float yaw){
	start_temp.x=x;
	start_temp.y=y;
	start_temp.z=z;
	start_temp.yaw=yaw;
	//RCLCPP_INFO(this->get_logger(),"set:et:%f %f %f",end_temp.x, end_temp.y, end_temp.z); 
}
void OffboardControl::set_end_temp_point(float x=0.0,float y=0.0,float z=0.0,float yaw=0.0){
    if(x==0.0){end_temp.x=pose_.pose.position.x;}else{end_temp.x=x;}
    if(y==0.0){end_temp.y=pose_.pose.position.y;}else{end_temp.y=y;}
    if(z==0.0){end_temp.z=pose_.pose.position.z;}else{end_temp.z=z;}
    if(yaw==0.0){end_temp.yaw=quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);}else{end_temp.yaw=yaw;}
	RCLCPP_INFO(this->get_logger(),"set:et:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw); 
}

void OffboardControl::set_drone_target_point_local(float x,float y,float z,float yaw){
	start_temp=end_temp;
	end_temp.x=start_temp.x+x;
	end_temp.y=start_temp.y+y;
	end_temp.z=start_temp.z+z;
	end_temp.yaw=start_temp.yaw+yaw;
	
	RCLCPP_INFO(this->get_logger(),"set:drone_et:%f %f %f",end_temp.x, end_temp.y, end_temp.z); 
}

void OffboardControl::set_start_point_local(float x,float y,float z,float yaw){
	start_temp=end_temp;
	end_temp.x=start.x+x;
	end_temp.y=start.y+y;
	end_temp.z=start.z+z;
	end_temp.yaw=start.yaw+yaw;
	
	//RCLCPP_INFO(this->get_logger(),"set:et:%f %f %f",end_temp.x, end_temp.y, end_temp.z); 
}

void OffboardControl::set_world_point_local(float x,float y,float z,float yaw){
	start_temp=end_temp;
	end_temp.x=x;
	end_temp.y=y;
	end_temp.z=z;
	end_temp.yaw=yaw;
	
	//RCLCPP_INFO(this->get_logger(),"set:et:%f %f %f",end_temp.x, end_temp.y, end_temp.z); 
}

bool OffboardControl::at_check_point(float accuracy){
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
void OffboardControl::get_target_location(float delta_heading, float* x, float* y) {
    // Assuming that 'heading' is a member variable of the class representing the current heading
	RCLCPP_INFO(get_logger(), "dheading: %f", delta_heading);
	float yaw = heading + delta_heading;
	if (yaw >= 2*180) {
		heading -= 2*180;
	}
	RCLCPP_INFO(get_logger(), "yaw: %f", yaw);
    // Convert yaw to radians
    double yaw_rad = yaw / 180 * PI;
	RCLCPP_INFO(get_logger(), "yaw_rad: %f", yaw_rad);
    // Assuming that 'distance' is the distance you want to move in the direction of 'yaw'
    //float distance = sqrt(*x**x+*y**y); // replace with your actual distance
	//RCLCPP_INFO(get_logger(), "distance: %f", distance);
    // Calculate the new position
	float x1=*x;
	float y1=*y;
    *x = x1 * cos(yaw_rad)-y1 * sin(yaw_rad);//
    *y = x1 * sin(yaw_rad)+y1 * cos(yaw_rad);//
	RCLCPP_INFO(get_logger(), "cos(yaw_rad): %f, sin(yaw_rad): %f", cos(yaw_rad), sin(yaw_rad));
	RCLCPP_INFO(get_logger(), "ldx: %f, ldy: %f", *x, *y);
}
void OffboardControl::timer_callback(void){
	static uint8_t num_of_steps = 0;
	//switch_to_autotune_mode();
	// offboard_control_mode needs to be paired with trajectory_setpoint
	//publish_offboard_control_mode();
	switch (fly_state_)
	{
	case FlyState::init:
		if(pose_.header.stamp.sec == 0){
			RCLCPP_INFO(this->get_logger(), "No pose data received yet");
			break;
		}
		
		timestamp0 = this->get_clock()->now().nanoseconds() / 1000;
		RCLCPP_INFO(this->get_logger(), "timestamp0= %f ,\ntimestamp-timestamp0=%f", timestamp0, this->get_clock()->now().nanoseconds() - timestamp0);
		start.x= pose_.pose.position.x;
		start.y= pose_.pose.position.y;
		start.z= pose_.pose.position.z;
		start.yaw=quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);
		start_temp=start;
		heading=start.yaw;
		RCLCPP_INFO(this->get_logger(), "yaw: %f", heading);
		//RCLCPP_INFO(this->get_logger()," start: x: %f  y: %f  z: %f", start.x, start.y,  start.z);
		fly_state_ = FlyState::request;
		break;
	case FlyState::request:
		if (arm_done_){
			fly_state_ = FlyState::takeoff;
		}
		break;
	case FlyState::takeoff:
		//RCLCPP_INFO(this->get_logger(), "takeoff start");
		if (pose_.pose.position.z>2){
			start_temp=end_temp;
			set_end_temp_point();
			fly_state_ = FlyState::goto_shot_area;
			RCLCPP_INFO(this->get_logger(), "takeoff done,goto_shot_area start, totaltime=%fs", (this->get_clock()->now().nanoseconds() / 1000- timestamp0)/1000000.0);
		}
		else if(pose_.pose.position.z>1){
			send_velocity_command(0, 0, 0.1, 0);
		}else{
			command_takeoff_or_land("TAKEOFF");
			rclcpp::sleep_for(1s);
		}
		break;
	case FlyState::goto_shot_area:
		if (send_velocity_command_with_time(0.1, 0, 0, 0, 10)){
			fly_state_ = FlyState::goto_scout_area;
			RCLCPP_INFO(this->get_logger(), "findtarget start, totaltime=%fs", (this->get_clock()->now().nanoseconds() / 1000- timestamp0)/1000000.0);
		}
		else{
			
		}
		break;
	case FlyState::goto_scout_area:
		if (send_velocity_command_with_time(0, -0.1, 0, 0, 10)){
		//if(alt_hold(0, -0.1, 0, 90, 10)){
			switch_to_rtl_mode();
			rclcpp::sleep_for(10s);
			fly_state_ = FlyState::land;
			RCLCPP_INFO(this->get_logger(), "goto_scout_area done,scout start totaltime=%fs", (this->get_clock()->now().nanoseconds() / 1000- timestamp0)/1000000.0);
		}
		break;
	case FlyState::land:
		switch_to_guided_mode();
		command_takeoff_or_land("LAND");
		fly_state_ = FlyState::end;
		RCLCPP_INFO(this->get_logger(), "land done,end start totaltime=%fs", (this->get_clock()->now().nanoseconds() / 1000- timestamp0)/1000000.0);
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
			num_of_steps = 0;
			arm_motors(true);
			state_ = State::arm_requested;
		}
		break;
	case State::arm_requested :
		if(arm_done_){
			
			//RCLCPP_INFO(this->get_logger(), "vehicle is armed");
			//switch_to_auto_mode();
			state_ = State::takeoff;
		}
		else{
			rclcpp::sleep_for(1s);
			arm_motors(true);
			//service_done_ = false;
		}
		break;
	case State::takeoff:
		rclcpp::sleep_for(1s);
			//RCLCPP_INFO(this->get_logger(), "vehicle is start");
			
			//arm_motors(true);
			
			//publish_global_gps(global_gps_.pose.position.latitude, global_gps_.pose.position.longitude, 600.0);
			
			//arm_motors(true);
			//command_takeoff_or_land("TAKEOFF");
			state_ = State::autotune_mode;

		break;
	case State::autotune_mode:

	default:
		break;
	}
	//RCLCPP_INFO(this->get_logger(), "State: %d", pose_.header.stamp.sec);
	//RCLCPP_INFO(this->get_logger(), "State: %d", pose_.header.stamp.nanosec);
}







int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>("/mavros/"));

	rclcpp::shutdown();
	return 0;
}


