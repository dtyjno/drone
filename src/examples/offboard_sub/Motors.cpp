#include "Motors.h"
#include <chrono>
#include "math.h"
using namespace std::chrono_literals;


bool Motors::armed;
bool Motors::connected;
bool Motors::guided;
std::string Motors::mode;
std::string Motors::system_status;
Vector3f Motors::home_position;//{FLT_MIN,0,0};
Vector3f Motors::home_position_global;
Vector4f Motors::home_quaternion;// 四元数

// 起飞 local_frame_z: 当前高度(m) takeoff_altitude=5 起飞高度(m)
// 1. 设置家的位置
// 2. 进入GUIDED模式
// 3. 等待稳定的offboard模式
// 4. 请求解锁
// 5. 起飞
// 6. 若未解锁则回到第3步
bool Motors::takeoff(float local_frame_z,float takeoff_altitude){
	static bool is_takeoff = false;
	static uint8_t num_of_steps = 0;
	switch (state_)
	{
	case State::init :
		set_home_position();
		state_ = State::send_geo_grigin;
		break;
	case State::send_geo_grigin :
		RCLCPP_INFO(node->get_logger(), "Entered guided mode");
		switch_mode("GUIDED");
		state_ = State::wait_for_stable_offboard_mode;				
		
		break;
	case State::wait_for_stable_offboard_mode :
		if (++num_of_steps>10){
			rclcpp::sleep_for(50ms);
			arm_motors(true);
			state_ = State::arm_requested;
		}
		break;
	case State::arm_requested :
		if(!armed){//_arm_done
			rclcpp::sleep_for(200ms);
			arm_motors(true);
			command_takeoff_or_land("TAKEOFF");
		}
		else{
			//RCLCPP_INFO(this->get_logger(), "vehicle is armed");
			state_ = State::takeoff;
		}
		break;
	case State::takeoff:
		//RCLCPP_INFO(this->get_logger(), "vehicle is start");
		//arm_motors(true);			
		if(local_frame_z - home_position.z < 2){
			command_takeoff_or_land("TAKEOFF",takeoff_altitude);
			rclcpp::sleep_for(300ms);
			
		}else{
			is_takeoff = true;
			//RCLCPP_INFO(this->get_logger(), "takeoff done");
			state_ = State::autotune_mode;
		}
		break;
	case State::autotune_mode:
		if(!armed){
			is_takeoff = false;
			num_of_steps = 0;
			RCLCPP_INFO(node->get_logger(), "vehicle is not armed");
			state_ = State::wait_for_stable_offboard_mode;
		}
		break;
	default:
		break;
	}
	return is_takeoff;
}


void Motors::state_callback(const mavros_msgs::msg::State::SharedPtr msg) 
{
	armed = msg->armed;
	connected = msg->connected;
	guided = msg->guided;
	mode = msg->mode;
	system_status = msg->system_status;
	// node->set_state();
}

// 接收home位置数据
void Motors::home_position_callback(const mavros_msgs::msg::HomePosition::SharedPtr msg) 
{
	home_position = {
		static_cast<float>(msg->position.x),
		static_cast<float>(msg->position.y),
		static_cast<float>(msg->position.z)
	};
	home_position_global = {
		static_cast<float>(msg->geo.latitude),
		static_cast<float>(msg->geo.longitude),
		static_cast<float>(msg->geo.altitude)
	};
	home_quaternion = {
		static_cast<float>(msg->orientation.x),
		static_cast<float>(msg->orientation.y),
		static_cast<float>(msg->orientation.z),
		static_cast<float>(msg->orientation.w)
	}; // 四元数
	// node->set_home_position();
	// home_position.x = msg->position.x;
	// home_position.y = msg->position.y;
	// home_position.z = msg->position.z;
    // home_quaternion.w 
	// RCLCPP_INFO(node->get_logger(), "Received home position data");
	// RCLCPP_INFO(node->get_logger(), "Latitude: %f", msg->geo.latitude);
	// RCLCPP_INFO(node->get_logger(), "Longitude: %f", msg->geo.longitude);
	// RCLCPP_INFO(node->get_logger(), "Altitude: %f", msg->geo.altitude);
	// RCLCPP_INFO(node->get_logger(), "X: %f", msg->position.x);
	// RCLCPP_INFO(node->get_logger(), "Y: %f", msg->position.y);
	// RCLCPP_INFO(node->get_logger(), "Z: %f", msg->position.z);
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
void Motors::switch_mode(std::string mode)
{
	auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
	request->base_mode = 0;
	request->custom_mode = mode;

	// service_done_ = false;

	RCLCPP_INFO(node->get_logger(), "Command send");
	OffboardControl_Base* node = this->node;
	auto result_future = mode_switch_client_->async_send_request(request,
		[node](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
			auto status = future.wait_for(0.5s);
			if (status == std::future_status::ready) {
				auto reply = future.get()->mode_sent;
				RCLCPP_INFO(node->get_logger(), "Mode switch: %s", reply ? "success" : "failed");
				if (reply == 1) {
					// Code to execute if the future is successful
					// service_done_ = true;
				}
				else {
					// Code to execute if the future is unsuccessful
					// service_done_ = false;
					RCLCPP_ERROR(node->get_logger(), "Failed to call service /ap/mode_switch");
				}
			} else {
				// Wait for the result.
				RCLCPP_INFO(node->get_logger(), "Service In-Progress...");
			}
		});
}

// void Motors::switch_to_auto_mode(){
// 	RCLCPP_INFO(node->get_logger(), "requesting switch to mode");
// 	Motors::switch_mode("AUTO");
// }


// arm or disarm motors
// arm= arm: true, disarm: false
void Motors::arm_motors(bool arm)
{
  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = arm;

  while (!arm_motors_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }
	RCLCPP_INFO(node->get_logger(), "arm command send");
    
    OffboardControl_Base* node = this->node;
	auto result_future = arm_motors_client_->async_send_request(request,
		[node,this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
			auto status = future.wait_for(0.5s);
			if (status == std::future_status::ready) {
				auto reply = future.get()->success;
				RCLCPP_INFO(node->get_logger(), "Arm motors: %s", reply ? "success" : "failed");
				if (reply) {
					// Code to execute if the future is successful
					_arm_done = true;
				}
				else {
					// Code to execute if the future is unsuccessful
					_arm_done = false;
					RCLCPP_ERROR(node->get_logger(), ("Failed to call service " + ardupilot_namespace + "cmd/arming").c_str());
				}
			} else {
				// Wait for the result.
				RCLCPP_INFO(node->get_logger(), "Service In-Progress...");
			}
		});
}

// 设置无人机家的位置
// /mavros/cmd/set_home [mavros_msgs/srv/CommandHome]
void Motors::set_home_position(double lat, double lon, double alt)
{
	auto request = std::make_shared<mavros_msgs::srv::CommandHome::Request>();
	request->current_gps = false;
	request->latitude = lat;
	request->longitude = lon;
	request->altitude = alt;

	while (!set_home_client_->wait_for_service(std::chrono::seconds(1))) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
			return;
		}
		RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
	}
	RCLCPP_INFO(node->get_logger(), "set home command send");
    OffboardControl_Base* node = this->node;
	auto result_future = set_home_client_->async_send_request(request,
		[node,this](rclcpp::Client<mavros_msgs::srv::CommandHome>::SharedFuture future) {
			auto status = future.wait_for(0.5s);
			if (status == std::future_status::ready) {
				auto reply = future.get()->success;
				RCLCPP_INFO(node->get_logger(), "Set Home Position: %s", reply ? "success" : "failed");
				if (reply) {
					// Code to execute if the future is successful
				}
				else {
					// Code to execute if the future is unsuccessful
					RCLCPP_ERROR(node->get_logger(), ("Failed to call service " + ardupilot_namespace + "cmd/set_home").c_str());
				}
			} else {
				// Wait for the result.
				RCLCPP_INFO(node->get_logger(), "Service In-Progress...");
			}
		});
}

void Motors::set_home_position()
{
	auto request = std::make_shared<mavros_msgs::srv::CommandHome::Request>();
	request->current_gps = true;
	while (!set_home_client_->wait_for_service(std::chrono::seconds(1))) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
			return;
		}
		RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
	}
	RCLCPP_INFO(node->get_logger(), "set home command send");
    OffboardControl_Base* node = this->node;
	auto result_future = set_home_client_->async_send_request(request,
		[node,this](rclcpp::Client<mavros_msgs::srv::CommandHome>::SharedFuture future) {
			auto status = future.wait_for(0.5s);
			if (status == std::future_status::ready) {
				auto reply = future.get()->success;
				RCLCPP_INFO(node->get_logger(), "Set Home Position: %s", reply ? "success" : "failed");
				if (reply) {
					// Code to execute if the future is successful
				}
				else {
					// Code to execute if the future is unsuccessful
					RCLCPP_ERROR(node->get_logger(), ("Failed to call service " + ardupilot_namespace + "cmd/set_home").c_str());
				}
			} else {
				// Wait for the result.
				RCLCPP_INFO(node->get_logger(), "Service In-Progress...");
			}
		});
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

// #### Common type for GLOBAL Take Off and Landing
void Motors::command_takeoff_or_land(std::string mode, double altitude)
{
	//RCLCPP_ERROR(this->get_logger(), ("Failed to call service "+ardupilot_namespace+"cmd/takeoff").c_str());
	// service_done_ = false;
	std::string mode_str(mode);
    OffboardControl_Base* node = this->node;

	if(mode=="TAKEOFF"){
	auto takeoff_client = node->create_client<mavros_msgs::srv::CommandTOL>(ardupilot_namespace+"cmd/takeoff");

	auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
	takeoff_request->min_pitch = 0.0;
	takeoff_request->yaw = 0.0;
	takeoff_request->latitude = 0.0;
	takeoff_request->longitude = 0.0;
	takeoff_request->altitude = altitude;
	
	auto takeoff_result_future = takeoff_client->async_send_request(takeoff_request,
		[this,node](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
			auto status = future.wait_for(0.5s);
			if (status == std::future_status::ready) {
				auto reply = future.get()->success;
				RCLCPP_INFO(node->get_logger(), "TakeOff: %s", reply ? "success" : "failed");
				if (reply == 1) {
					// Code to execute if the future is successful
					// service_done_ = true;
				}
				else {
					// Code to execute if the future is unsuccessful
					RCLCPP_ERROR(node->get_logger(), ("Failed to call service "+ardupilot_namespace+"cmd/takeoff").c_str());
				}
			} else {
				// Wait for the result.
				RCLCPP_INFO(node->get_logger(), "Service In-Progress...");
			}
		});
	} else if(mode=="LAND"){
	auto land_client = node->create_client<mavros_msgs::srv::CommandTOL>(ardupilot_namespace+"cmd/land");
	
	auto land_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
	land_request->yaw = 0.0;
	land_request->latitude = 0.0;
	land_request->longitude = 0.0;
	land_request->altitude = 0.0;

	auto land_result_future = land_client->async_send_request(land_request,
		[this,node](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
			auto status = future.wait_for(0.5s);
			if (status == std::future_status::ready) {
				auto reply = future.get()->success;
				RCLCPP_INFO(node->get_logger(), "Land: %s", reply ? "success" : "failed");
				if (reply == 1) {
					// Code to execute if the future is successful
					// service_done_ = true;
				}
				else {
					// Code to execute if the future is unsuccessful
					RCLCPP_ERROR(node->get_logger(), ("Failed to call service "+ardupilot_namespace+"cmd/land").c_str());
				}
			} else {
				// Wait for the result.
				RCLCPP_INFO(node->get_logger(), "Service In-Progress...");
			}
		});
	}
}