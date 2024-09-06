#include <rclcpp/rclcpp.hpp>
#include "offboard_control.hpp"
#include "tools.hpp"

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>("/mavros/"));
	rclcpp::shutdown();
	return 0;
}
OffboardControl::OffboardControl(std::string ardupilot_namespace) :
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
void OffboardControl::timer_callback(void){
	static uint8_t num_of_steps = 0;
	//switch_to_autotune_mode();
	// offboard_control_mode needs to be paired with trajectory_setpoint
	//publish_offboard_control_mode();
	
	
	//publish_trajectory_setpoint(0, 0, 10, 0);
	
    switch (fly_state_)
	{
	case FlyState::init:
		if(pose_.header.stamp.sec == 0){
			RCLCPP_INFO(this->get_logger(), "No pose data received yet");
			break;
		}
		
		timestamp0 = this->get_clock()->now().nanoseconds() / 1000;
		RCLCPP_INFO(this->get_logger(), "timestamp-timestamp0=%f", this->get_clock()->now().nanoseconds() - timestamp0);
		start.x= pose_.pose.position.x;
		start.y= pose_.pose.position.y;
		start.z= pose_.pose.position.z;
		start.yaw=quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);
		end_temp=start;
		heading=start.yaw;
		RCLCPP_INFO(this->get_logger(), "target_heading: %f", heading);
		RCLCPP_INFO(this->get_logger()," current location: x: %f  y: %f  z: %f", start.x, start.y,  start.z);
		//subscribe to yolo detection
        fly_state_ = FlyState::request;
		break;
	case FlyState::request:
		if (arm_done_){
            RCLCPP_INFO(this->get_logger(), "Taking off");
			fly_state_ = FlyState::takeoff;
		}
		break;
	case FlyState::takeoff:
        //takeoff and leave tekeoff area
		if (pose_.pose.position.z>5){
			RCLCPP_INFO(this->get_logger(), "goto_shot_area start, totaltime=%fs", (this->get_clock()->now().nanoseconds() / 1000- timestamp0)/1000000.0);
			fly_state_ = FlyState::goto_shot_area;
		}else if(pose_.pose.position.z>0.5){
			trajectory_setpoint_takeoff(0, 0, 5, 0);
		}
		else{
			command_takeoff_or_land("TAKEOFF");//height=6m
			rclcpp::sleep_for(2s);
		}
		break;
	case FlyState::goto_shot_area:
    ////////////////////////////////
		trajectory_setpoint(53, 0, 0, 0);
		if (at_check_point()){
			fly_state_ = FlyState::findtarget;
            RCLCPP_INFO(this->get_logger(), "开始投弹部分");
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
			RCLCPP_INFO(this->get_logger(), "RTL open!");
            switch_to_rtl_mode();
			rclcpp::sleep_for(10s);
            RCLCPP_INFO(this->get_logger(), "gogogo");
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

    switch (state_)
	{
	case State::init :
		RCLCPP_INFO(this->get_logger(), "Arming motors...");
			//global_gps_publisher_->publish(global_gps_start);
		state_ = State::send_geo_grigin;
		break;
	case State::send_geo_grigin :
		RCLCPP_INFO(this->get_logger(), "Vehicle mode: GUIDED");
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
			RCLCPP_INFO(this->get_logger(), "Vehicle armed");
			//switch_to_auto_mode();
			state_ = State::takeoff;
		}
		else{
            RCLCPP_INFO(this->get_logger(), "Waiting for arming...");
			rclcpp::sleep_for(1s);
			arm_motors(true);
			//service_done_ = false;
		}
		break;
	case State::takeoff:
		break;
	default:
		break;
	}
	//RCLCPP_INFO(this->get_logger(), "State: %d", pose_.header.stamp.sec);
	//RCLCPP_INFO(this->get_logger(), "State: %d", pose_.header.stamp.nanosec);
	
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
