#include "offboard_control.hpp"

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
	RCLCPP_INFO(this->get_logger(), "arm command send");
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
	msg->amsl;

	// RCLCPP_INFO(this->get_logger(), "Received altitude data");
	// RCLCPP_INFO(this->get_logger(), "Monotonic: %f", msg->monotonic);
	// RCLCPP_INFO(this->get_logger(), "Amsl: %f", msg->amsl);
	// RCLCPP_INFO(this->get_logger(), "Local: %f", msg->local);
	// RCLCPP_INFO(this->get_logger(), "Relative: %f", msg->relative);
	// RCLCPP_INFO(this->get_logger(), "Terrain: %f", msg->terrain);
	// RCLCPP_INFO(this->get_logger(), "Bottom clear: %f", msg->bottom_clearance);
}
//
void OffboardControl::state_callback(const mavros_msgs::msg::State::SharedPtr msg) 
{
	drone_state_ = *msg;
	// RCLCPP_INFO(this->get_logger(), "Received state data");
	// RCLCPP_INFO(this->get_logger(), "Mode: %s", msg->mode.c_str());
	// RCLCPP_INFO(this->get_logger(), "Armed: %d", msg->armed);
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
// ros2 topic pub /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, pose: {position: {x: 0.0, y: 0.0, z: 5.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}'
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
