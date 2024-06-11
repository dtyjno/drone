#include "offboard_control.hpp"

void OffboardControl::switch_to_guided_mode(){
	RCLCPP_INFO(this->get_logger(), "requesting switch to GUIDED mode");
	OffboardControl::switch_mode("GUIDED");
}

void OffboardControl::switch_to_takeoff(){
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
	RCLCPP_INFO(this->get_logger(), "Received yaw: %f", quaternion_to_yaw(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w));
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
	 RCLCPP_INFO(this->get_logger(), "Angular z: %f", msg->twist.angular.z);
}