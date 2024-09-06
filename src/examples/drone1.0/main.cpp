#include "offboard_control.hpp"
#include "yolo.hpp"

void OffboardControl::timer_callback(void){
	static uint8_t num_of_steps = 0;
	static bool is_takeoff = false;
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
		RCLCPP_INFO(this->get_logger(), "arm_done_: %d", arm_done_);
		//RCLCPP_INFO(this->get_logger(), "takeoff start");
		// command_takeoff_or_land("TAKEOFF");
		// rclcpp::sleep_for(1s);
		//RCLCPP_INFO(this->get_logger(), "pose_.pose.position.z-start.z=%lf", pose_.pose.position.z-start.z);
		//if (true){//pose_.pose.position.z>1+start.z){
		//if(arm_done_){
		if (is_takeoff){	
		//command_takeoff_or_land("TAKEOFF");	
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
			//rclcpp::sleep_for(3s);
		}
		break;
	case FlyState::goto_shot_area:
		//trajectory_setpoint(53, 10, 0, 0);
		trajectory_setpoint_start(30, 0, 5, 0);
		if (at_check_point()){
			fly_state_ = FlyState::findtarget;
			RCLCPP_INFO(this->get_logger(), "findtarget start, totaltime=%fs", (this->get_clock()->now().nanoseconds() / 1000- timestamp0)/1000000.0);
		}
		else{
			
		}
		break;
	case FlyState::findtarget:
		if(surrounding_shot_area()){
			fly_state_ = FlyState::goto_scout_area;
			RCLCPP_INFO(this->get_logger(), "findtarget done,goto_scout_area start totaltime=%fs", (this->get_clock()->now().nanoseconds() / 1000- timestamp0)/1000000.0);
		}
		break;
	case FlyState::goto_scout_area:
		trajectory_setpoint_start(53, 0, 5, 0);

		if(at_check_point()){
			fly_state_ = FlyState::scout;
			RCLCPP_INFO(this->get_logger(), "goto_scout_area done,scout start totaltime=%fs", (this->get_clock()->now().nanoseconds() / 1000- timestamp0)/1000000.0);
		}
		break;
	case FlyState::scout:
		if(surrounding_scout_area()){
			fly_state_ = FlyState::land;
			RCLCPP_INFO(this->get_logger(), "scout done,land start totaltime=%fs", (this->get_clock()->now().nanoseconds() / 1000- timestamp0)/1000000.0);	
			switch_to_rtl_mode();
			rclcpp::sleep_for(10s);
		}
		break;
	case FlyState::land:
		
		switch_to_guided_mode();
		trajectory_setpoint_start(0, 0, 2, 0);
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
		if(arm_done_==true){
			
			//RCLCPP_INFO(this->get_logger(), "vehicle is armed");
			//switch_to_auto_mode();
			state_ = State::takeoff;
		}
		else{
			rclcpp::sleep_for(1s);
			arm_motors(true);
			//service_done_ = false;command
		}
		break;
	case State::takeoff:
		// rclcpp::sleep_for(1s);
			//RCLCPP_INFO(this->get_logger(), "vehicle is start");
			
			//arm_motors(true);
			
			//publish_global_gps(global_gps_.pose.position.latitude, global_gps_.pose.position.longitude, 600.0);
			
			//arm_motors(true)//;
			if(pose_.pose.position.z-start.z < 2){
				command_takeoff_or_land("TAKEOFF");
				
			}else{
				//RCLCPP_INFO(this->get_logger(), "takeoff done");
				state_ = State::autotune_mode;
			}
		break;
	case State::autotune_mode:
		is_takeoff = true;
		if(!drone_state_.armed){
			RCLCPP_INFO(this->get_logger(), "vehicle is not armed");
			state_ = State::wait_for_stable_offboard_mode;
		}
	default:
		break;
	}
}


// 环绕射击区域
bool OffboardControl::surrounding_shot_area(void){
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
bool OffboardControl::surrounding_scout_area(void){
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



int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

	rclcpp::executors::MultiThreadedExecutor executor;
	auto node = std::make_shared<OffboardControl>("/mavros/");
	auto node1 = std::make_shared<YOLO>();
	// auto pubnode = std::make_shared<PublisherNode>();
	/* 运行节点，并检测退出信号*/
	executor.add_node(node);
	executor.add_node(node1);
	executor.spin();
	// rclcpp::spin(std::make_shared<OffboardControl>("/mavros/"));
	rclcpp::shutdown();
	return 0;
}