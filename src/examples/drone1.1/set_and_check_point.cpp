#include "offboard_control.hpp"


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
void OffboardControl::set_end_temp_point(float x,float y,float z,float yaw){
	end_temp.x=x;
	end_temp.y=y;
	end_temp.z=z;
	end_temp.yaw=yaw;
	//RCLCPP_INFO(this->get_logger(),"set:et:%f %f %f",end_temp.x, end_temp.y, end_temp.z); 
}

void OffboardControl::set_drone_target_point_local(float x,float y,float z,float yaw){
	start_temp=end_temp;
	end_temp.x=start_temp.x+x;
	end_temp.y=start_temp.y+y;
	end_temp.z=start_temp.z+z;
	end_temp.yaw=start_temp.yaw+yaw;
	
	//RCLCPP_INFO(this->get_logger(),"set:et:%f %f %f",end_temp.x, end_temp.y, end_temp.z); 
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
		&& abs(quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w) - end_temp.yaw) <= 5
		){
		
		return true;
	}
	else{
		return false;
	}	
}