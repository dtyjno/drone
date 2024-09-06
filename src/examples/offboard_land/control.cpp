#include "OffboardControl.h"
#include "PID.h"
#include "math.h"

// 抵达桶上方
// if(识别到桶=catch_target_bucket（到达正上方）){[if(到达正上方==true){...}}
bool OffboardControl::catch_target_bucket(bool &result){
	if(_yolo->get_x()==0 && _yolo->get_y()==0){
		return false;
	}
	static enum class CatchState{
		init,
		fly_to_target,
		end
	} catch_state_;
	static double time_find_start = 0;
	(void)time_find_start;
	static float set_z=1;
	static float set_yaw=0;
	//bool result=false;
	//if(vehicle_local_position_msg.x>10 || vehicle_local_position_msg.x<-10 || vehicle_local_position_msg.y>10 || vehicle_local_position_msg.y<-10){
	//	result=true;
	//}
	//double time_now = get_cur_time();
	//AtCheckPoint();
	switch (catch_state_)
	{
		case CatchState::init:{
			time_find_start = get_cur_time();
			set_z = 1;
			set_yaw = get_yaw();
			catch_state_=CatchState::fly_to_target;
			break;
		}
		case CatchState::fly_to_target:{
			float now_x = _yolo->get_x();
			float now_y = - _yolo->get_y();
			float tar_x = SET_CAP_FRAME_WIDTH/2 ;// /10
			float tar_y = - SET_CAP_FRAME_HEIGHT/2 ;// /3
			rotate_xy(now_x,now_y,get_yaw());
			rotate_xy(tar_x,tar_y,get_yaw());
			RCLCPP_INFO(this->get_logger(), "catch_target_bucket: now_x: %f, now_y: %f, tar_x: %f, tar_y: %f", now_x, now_y, tar_x, tar_y);
			RCLCPP_INFO(this->get_logger(), "catch_target_bucket: now_x: %f, now_y: %f, tar_x: %f, tar_y: %f", now_x/SET_CAP_FRAME_WIDTH, now_y/SET_CAP_FRAME_HEIGHT,(tar_x)/SET_CAP_FRAME_WIDTH,(tar_y)/SET_CAP_FRAME_HEIGHT);
			PID::Defaults defaults;
            
            // bool trajectory_setpoint(Vector4f pos_now,Vector4f pos_target,PID::Defaults defaults,double accuracy = DEFAULT_ACCURACY);
            // 假设Vector4f可以这样构造，并且PID::Defaults有一个名为p的公共成员变量
            if(_pose_control->trajectory_setpoint(
                Vector4f{now_x/SET_CAP_FRAME_WIDTH, now_y/SET_CAP_FRAME_HEIGHT, get_z_pos(), get_yaw()},
                Vector4f{(tar_x)/SET_CAP_FRAME_WIDTH, (tar_y)/SET_CAP_FRAME_HEIGHT, set_z, set_yaw},
                // PID::Defaults{.p = 0.5,.i = 0.1,.d = 0.1,.imax = 10,._use_vel = false}  //c++20
                PID::Defaults{0.5,0.1,0.1,0,0,10},
				5
            )){
				RCLCPP_INFO(this->get_logger(), "Arrive, 投弹");
				rclcpp::sleep_for(std::chrono::seconds(5));
				catch_state_=CatchState::end;
			}
			if(get_cur_time()-time_find_start>12){//>12s
				catch_state_=CatchState::end;
			}
			break;
		}
		case CatchState::end:{
			catch_state_=CatchState::init;
			result = true;
			return true;
			break;
		default:
			break;
		}
	}
	result = false;
	return true;
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
	static double time_find_start = get_cur_time();
	if((get_cur_time()-time_find_start)>12){//>12s
		return true;
	}
	RCLCPP_INFO(this->get_logger(),"cur_time: %f , start_time: %f",get_cur_time(),time_find_start);
	// std::cout<<"cur time: "<<get_cur_time()<<" find_start: "<<time_find_start<<std::endl;
	bool arrive;
	if(catch_target_bucket(arrive)){
		if(arrive){
			return true;
		}
		return false;
	}
	/*bool result=false;
	if(vehicle_local_position_msg.x>10 || vehicle_local_position_msg.x<-10 || vehicle_local_position_msg.y>10 || vehicle_local_position_msg.y<-10){
		result=true;
	}*/
	//double time_now = get_cur_time();
	//AtCheckPoint();

	switch (sur_state_)
	{
	case SurState::init:
		time_find_start = get_cur_time();
		static double fx=1,fy=2;
		sur_state_=SurState::set_point_x;
		break;
	case SurState::set_point_x:
		sur_state_=SurState::fly_to_target_x;
		break;
	case SurState::set_point_y:
		sur_state_=SurState::fly_to_target_y;
		break;
	case SurState::fly_to_target_x:
		if(trajectory_setpoint(fx,0,0,0)){
			fx=(fx>0)?(-fx-1):(-fx+1);
			sur_state_=SurState::set_point_y;
		}
		break;
	case SurState::fly_to_target_y:
		if(trajectory_setpoint(0,fy,0,0)){
			sur_state_=SurState::set_point_x;
			fy=(fy>0)?(-fy-1):(-fy+1);
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
	static double time_find_start = 0;
	/*bool result=false;
	if(vehicle_local_position_msg.x>10 || vehicle_local_position_msg.x<-10 || vehicle_local_position_msg.y>10 || vehicle_local_position_msg.y<-10){
		result=true;
	}*/
	//double time_now = get_cur_time();
	//AtCheckPoint();
	switch (sco_state_)
	{
	case ScoState::init:
		time_find_start = get_cur_time();
		sco_state_=ScoState::first_path;
		break;
	case ScoState::first_path:
		if(trajectory_setpoint(2,0,0,-60)){
			sco_state_=ScoState::second_path;
		}
		if((get_cur_time()-time_find_start)>12){//>12s
			sco_state_=ScoState::end;
		}
		break;
	case ScoState::second_path:
		if(trajectory_setpoint(-2,2.5,0,-60)){
			sco_state_=ScoState::third_path;
		}
		if((get_cur_time()-time_find_start)>12){//>12s
			sco_state_=ScoState::end;
		}
		break;
	case ScoState::third_path:
		if(trajectory_setpoint(2,0,0,-60)){
			sco_state_=ScoState::forth_path;
		}
		if((get_cur_time()-time_find_start)>12){//>12s
			sco_state_=ScoState::end;
		}
		break;
	case ScoState::forth_path:
		if(trajectory_setpoint(-2,2.5,0,-60)){
			sco_state_=ScoState::fifth_path;
		}
		if((get_cur_time()-time_find_start)>12){//>12s
			sco_state_=ScoState::end;
		}
		break;
	case ScoState::fifth_path:
		if(trajectory_setpoint(2,0,0,-60)){
			sco_state_=ScoState::sixth_path;
		}
		if((get_cur_time()-time_find_start)>12){//>12s
			sco_state_=ScoState::end;
		}
		break;
	case ScoState::sixth_path:
		if(trajectory_setpoint(-1,-2.5,0,-90)){
			sco_state_=ScoState::end;
		}
		if((get_cur_time()-time_find_start)>12){//>12s
			sco_state_=ScoState::end;
		}
		break;
	case ScoState::end:
		sco_state_=ScoState::init;
		RCLCPP_INFO(this->get_logger(), "end scouting: %f", (get_cur_time()-time_find_start)/1000000.0);
		return true;
		break;
	default:
		break;
	}
	return false;
}

bool OffboardControl::trajectory_setpoint(float x,float y,float z,float yaw,double accuracy){
    return _pose_control->trajectory_setpoint(
        Vector4f{get_x_pos(),get_y_pos(),get_z_pos(),get_yaw()},
        Vector4f{x,y,z,static_cast<float>(yaw)},
        accuracy
    );
}
bool OffboardControl::trajectory_setpoint(float x,float y,float z,float yaw,PID::Defaults defaults,double accuracy){
    return _pose_control->trajectory_setpoint(
        Vector4f{get_x_pos(),get_y_pos(),get_z_pos(),get_yaw()},
        Vector4f{x,y,z,static_cast<float>(yaw)},
		defaults,
        accuracy
    );
}
bool OffboardControl::trajectory_setpoint_world(float x,float y,float z,float yaw,double accuracy){
    return _pose_control->trajectory_setpoint_world(
        Vector4f{get_x_pos(),get_y_pos(),get_z_pos(),get_yaw()},
        Vector4f{x,y,z,static_cast<float>(yaw)},
        accuracy
    );
}

bool OffboardControl::publish_setpoint_world(float x,float y,float z,float yaw, double accuracy){
    return _pose_control->publish_setpoint_world(
        Vector4f{get_x_pos(),get_y_pos(),get_z_pos(),get_yaw()},
        Vector4f{x,y,z,static_cast<float>(yaw)},
		accuracy
	);
}