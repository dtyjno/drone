#include "OffboardControl.h"

void OffboardControl::timer_callback(void)
{
	// std::cout << "--------timer_callback----------" << std::endl;
	// std::cout << get_x_pos() <<" yaw: "<< get_n_yaw() <<" "<<get_armed()<< std::endl;
	// RCLCPP_INFO(this->get_logger(),"cur_time: %f , start_time: %f ,yaw: %f",get_cur_time(), timestamp_init, get_yaw());
	// RCLCPP_INFO(this->get_logger(),"yolo_x: %f , yolo_y: %f ",_yolo->get_x( YOLO::TARGET_TYPE::CIRCLE),_yolo->get_y( YOLO::TARGET_TYPE::CIRCLE));

	bool is_takeoff = _motors->takeoff(get_z_pos());

	switch (fly_state_)
	{
	case FlyState::init:
		FlyState_init();
		break;
	case FlyState::takeoff:
		if (is_takeoff)
		{
			if (get_z_pos() > 2.5 + get_z_home_pos())
			{
				RCLCPP_INFO(this->get_logger(), "goto_shot_area start, totaltime=%fs", get_cur_time());
				fly_state_ = FlyState::goto_shot_area;
			}
			else
			{
				// 设定速度
				_pose_control->send_velocity_command_world(0, 0, 0.5, 0);
			}
		}
		else
		{
			// RCLCPP_INFO(this->get_logger(), "Takeoff failed");
		}
		break;
	case FlyState::goto_shot_area:
		// if(publish_setpoint_world(30, 0, 5, 0)){
		// //trajectory_setpoint(53, 10, 0, 0);

		// static enum { point1,
		// 			  point2,
		// 			  point3 } gsa_ps = point1;
		// switch (gsa_ps)
		// {
		// case point1:
		// 	if (trajectory_setpoint_world(0, 0, 5, 0))
		// 	{
		// 		gsa_ps = point2;
		// 	}
		// 	break;
		// case point2:
		// 	save_log();
		// 	if (trajectory_setpoint_world(10, 0, 5, 0, 0.00001))
		// 	{
		// 		// if(trajectory_generator_world(2, {10 ,0 ,5})){
		// 		gsa_ps = point3;
		// 	}
		// 	break;
		// case point3:
		// 	save_log();
		// 	if (trajectory_setpoint_world(10, 0, 5, 0, 0.00001))
		// 	{
		// 		// fly_state_ = FlyState::shot;
		// 		RCLCPP_INFO(this->get_logger(), "goto_shot_area done,shot start totaltime=%fs", get_cur_time());
		// 	}
		// 	// _pose_control->send_velocity_command_world(0, 0, 0, 0);
		// 	break;
		// default:
		// 	break;
		// }

		if (trajectory_setpoint_world(30, 0, 5, 0))
		{
			// if(trajectory_generator_world(0.9, {10 ,10 ,5})){
			// if(trajectory_generator_world_points(
			// 	0.9,
			// 	{
			// 		{10 ,10 ,5},
			// 		{10, -10, 5},
			// 		{-10,10,5}
			// 	},
			// 	3
			// )){
			fly_state_ = FlyState::findtarget;
			RCLCPP_INFO(this->get_logger(), "findtarget start, totaltime=%fs", get_cur_time());
		}
		else
		{
		}
		break;
	case FlyState::findtarget:
		if (surrounding_shot_area())
		{
			// if(trajectory_circle(0.6,1.0,5,0.08)){
			fly_state_ = FlyState::goto_scout_area;
			RCLCPP_INFO(this->get_logger(), "findtarget done,goto_scout_area start totaltime=%fs", get_cur_time());
		}
		break;
	case FlyState::goto_scout_area:
		if (trajectory_setpoint_world(58, 0, 5, 0))
		{
			fly_state_ = FlyState::scout;
			RCLCPP_INFO(this->get_logger(), "goto_scout_area done,scout start totaltime=%fs", get_cur_time());
		}
		break;
	case FlyState::scout:
		if (surrounding_scout_area())
		{
			fly_state_ = FlyState::land;
			RCLCPP_INFO(this->get_logger(), "scout done,land start totaltime=%fs", get_cur_time());
			_motors->switch_mode("RTL");
			rclcpp::sleep_for(15s);
		}
		break;
	case FlyState::land:
		_motors->switch_mode("LAND");
		if (trajectory_setpoint_world(0, 0, 2, 0))
		{
			_pose_control->send_velocity_command_world(0, 0, 0, 0);
			_motors->command_takeoff_or_land("LAND");
			fly_state_ = FlyState::end;
			RCLCPP_INFO(this->get_logger(), "land done,end start totaltime=%fs", get_cur_time());
		}
		break;
	case FlyState::end:
		RCLCPP_INFO(this->get_logger(), "end done, totaltime=%fs", get_cur_time());
		_motors->command_takeoff_or_land("LAND");
		rclcpp::sleep_for(3s);
		// rclcpp::shutdown();
		break;
	default:
		break;
	}
}

void OffboardControl::FlyState_init()
{
	rclcpp::sleep_for(1s);
	if (is_equal(get_x_pos(), DEFAULT_X_POS))
	{
		RCLCPP_INFO(this->get_logger(), "No pose data received yet");
		return;
	}
	// set_home_position(location.global_frame.lat,location.global_frame.lon,location.global_frame.alt);
	timestamp_init = get_cur_time();
	// RCLCPP_INFO(this->get_logger(), "timestamp_init= %f ,\ntimestamp_init-timestamp_init=%f", timestamp_init, this->get_clock()->now().nanoseconds() - timestamp_init);
	start = {get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()}; // 扩展卡尔曼滤波器（EKF3）已经为IMU（惯性测量单元）0和IMU1设置了起点。
	start_global = {get_lat(), get_lon(), get_alt()};			// AP: Field Elevation Set: 0m  当前位置的地面高度为0米，这对于高度控制和避免地面碰撞非常重要。
	RCLCPP_INFO(this->get_logger(), "yaw: %f", get_yaw());
	fly_state_ = FlyState::takeoff;
}

// //override the set_pose function
// void OffboardControl::set_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
// 	local_frame.x = msg->pose.position.x;
// 	local_frame.y = msg->pose.position.y;
// 	local_frame.z = msg->pose.position.z;
// 	// position.x = msg->pose.position.x;
// 	// position.y = msg->pose.position.y;
// 	// position.z = msg->pose.position.z;
// 	quaternion.w() = msg->pose.orientation.w;
// 	quaternion.x() = msg->pose.orientation.x;
// 	quaternion.y() = msg->pose.orientation.y;
// 	quaternion.z() = msg->pose.orientation.z;
// }
// //override the set_gps function
// void OffboardControl::set_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg){
// 	global_frame.lat = msg->latitude;
// 	global_frame.lon = msg->longitude;
// 	global_frame.alt = msg->altitude;
// }
// //override the set_velocity function
// void OffboardControl::set_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg){
// 	velocity.x = msg->twist.linear.x;
// 	velocity.y = msg->twist.linear.y;
// 	velocity.z = msg->twist.linear.z;
// 	velocity.yaw = msg->twist.angular.z;
// }
// //override the set_altitude function
// void OffboardControl::set_altitude(const mavros_msgs::msg::Altitude::SharedPtr msg){
// 	// altitude = msg->monotonic;
// 	(void)msg;
// }
// //override the set_state function
// void OffboardControl::set_state(const mavros_msgs::msg::State::SharedPtr msg){

//     armed = msg->armed;
//     connected = msg->connected;
//     guided = msg->guided;
//     mode = msg->mode;
//     system_status = msg->system_status;
// }
// //override the set_home_position function
// void OffboardControl::set_home_position(const mavros_msgs::msg::HomePosition::SharedPtr msg){

// 	home_position.x = msg->position.x;
// 	home_position.y = msg->position.y;
// 	home_position.z = msg->position.z;
//     home_position_global.x = msg->geo.latitude;
//     home_position_global.y = msg->geo.longitude;
//     home_position_global.z = msg->geo.altitude;
// 	home_quaternion.w() = msg->orientation.w;
// 	home_quaternion.x() = msg->orientation.x;
// 	home_quaternion.y() = msg->orientation.y;
// 	home_quaternion.z() = msg->orientation.z;
// }

// //override the set_pose function
// void OffboardControl::set_pose(){
// 	Vector3f pos = InertialNav::position;
// 	Vector4f ori = InertialNav::orientation;
// 	local_frame.x = pos.x;
// 	local_frame.y = pos.y;
// 	local_frame.z = pos.z;
// 	// local_frame = pos;
// 	quaternion.w() = ori.yaw;
// 	quaternion.x() = ori.x;
// 	quaternion.y() = ori.y;
// 	quaternion.z() = ori.z;
// }
// //override the set_gps function
// void OffboardControl::set_gps(){
// 	Vector3f gps = InertialNav::gps;
// 	global_frame.lat = gps.x;
// 	global_frame.lon = gps.y;
// 	global_frame.alt = gps.z;
// }
// //override the set_velocity function
// void OffboardControl::set_velocity(){
// 	Vector4f vel = InertialNav::velocity;
// 	velocity.x = vel.x;
// 	velocity.y = vel.y;
// 	velocity.z = vel.z;
// 	velocity.yaw = vel.yaw;
// }
// //override the set_altitude function
// void OffboardControl::set_altitude(){
// 	// altitude = msg->monotonic;
// }
// //override the set_state function
// void OffboardControl::set_state(){
//     armed = Motors::armed;
//     connected = Motors::connected;
//     guided = Motors::guided;
//     mode = Motors::mode;
//     system_status = Motors::system_status;
// }
// //override the set_home_position function
// void OffboardControl::set_home_position(){
// 	Vector3f pos = Motors::home_position;
// 	Vector3f pos_global = Motors::home_position_global;
// 	Vector4f ori = Motors::home_quaternion;
// 	home_position.x = pos.x;
// 	home_position.y = pos.y;
// 	home_position.z = pos.z;
// 	home_position_global.x = pos_global.x;
// 	home_position_global.y = pos_global.y;
// 	home_position_global.z = pos_global.z;
// 	home_quaternion.w() = ori.yaw;
// 	home_quaternion.x() = ori.x;
// 	home_quaternion.y() = ori.y;
// 	home_quaternion.z() = ori.z;
// }