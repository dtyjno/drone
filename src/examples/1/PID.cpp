#include <rclcpp/rclcpp.hpp>

//if(publish_trajectory_setpoint(location.local_frame.x,location.local_frame.y,location.local_frame.z,start_x,start_y,start_z){
//	return true;
//}

// if(catch_target_bucket(arrive)){
// 		if(arrive){
// 			return true;
// 		}
// 		return false;
// 	}

class OffboardControl : public rclcpp::Node {
public:
	OffboardControl():
    	Node("offboard_control_srv")
        {

        }
private:

	double k=0.002;//控制vx和vy
	// 初始化PID控制器
	double dt=0.1;
	static const double kp = 0.41;  // 比例参数
	static const double ki = 0.06;  // 积分参数
	static const double kd = 0.28;  // 微分参数
	double kp_yaw = 0.20;  // 比例参数
	double ki_yaw = 0.04;  // 积分参数
	double kd_yaw = 0.04;  // 微分参数
	double max_vx=2; //前后方向最大速度
	double max_vy=2; //左右方向最大速度
	double max_vz=1; //上下方向最大速度
	double max_yaw=10; //最大角速度(°/s)

    bool publish_trajectory_setpoint(double n_x,double n_y ,double n_z ,double t_x,double t_y ,double t_z  , double accuracy = 0.5 ,double kp_xy = kp ,double ki_xy = ki,double kd_xy = kd){
};


// 抵达桶上方
// if(识别到桶=catch_target_bucket（到达正上方）){[if(到达正上方==true){...}}
bool OffboardControl::catch_target_bucket(bool &result){
	//识别到桶圆心坐标
	if(yolo_->get_x()==0 && yolo_->get_y()==0){
		return false;
	}
	static enum class CatchState{
		init,
		fly_to_target,
		end
	} catch_state_;
	static uint64_t time_find_start = 0;
	(void)time_find_start;
	static double set_z=10;
	static double set_yaw=0;
	//bool result=false;
	//if(vehicle_local_position_msg.x>10 || vehicle_local_position_msg.x<-10 || vehicle_local_position_msg.y>10 || vehicle_local_position_msg.y<-10){
	//	result=true;
	//}
	//uint64_t time_now = this->get_clock()->now().nanoseconds() / 1000;
	//AtCheckPoint();
	switch (catch_state_)
	{
		case CatchState::init:{
			time_find_start = this->get_clock()->now().nanoseconds() / 1000;
			set_z = 1;
			set_yaw = location.local_frame.yaw;
			catch_state_=CatchState::fly_to_target;
			break;
		}
		case CatchState::fly_to_target:{
			double now_x = yolo_->get_x();
			double now_y = -yolo_->get_y();
			//SET_CAP_FRAME_WIDTH 获取到的图像宽度（像素）
			double tar_x = SET_CAP_FRAME_WIDTH/2 ;// /10
			//SET_CAP_FRAME_HEIGHT 获取到的图像高度（像素）
			double tar_y = -SET_CAP_FRAME_HEIGHT/2 ;// /3
			//坐标旋转默认角度+当前偏航角
			get_target_location(&now_x,&now_y,location.local_frame.yaw);
			get_target_location(&tar_x,&tar_y,location.local_frame.yaw);
			RCLCPP_INFO(this->get_logger(), "catch_target_bucket: now_x: %f, now_y: %f, tar_x: %f, tar_y: %f", now_x, now_y, tar_x, tar_y);
			RCLCPP_INFO(this->get_logger(), "catch_target_bucket: now_x: %f, now_y: %f, tar_x: %f, tar_y: %f", now_x/SET_CAP_FRAME_WIDTH, now_y/SET_CAP_FRAME_HEIGHT,(tar_x)/SET_CAP_FRAME_WIDTH,(tar_y)/SET_CAP_FRAME_HEIGHT);
			if(publish_trajectory_setpoint(
				now_x/SET_CAP_FRAME_WIDTH ,now_y/SET_CAP_FRAME_HEIGHT,location.local_frame.z,location.local_frame.yaw,
				(tar_x)/SET_CAP_FRAME_WIDTH,(tar_y)/SET_CAP_FRAME_HEIGHT,set_z,set_yaw,10,
				0.5,0.01,0.01
			)){
				RCLCPP_INFO(this->get_logger(), "Arive, 投弹");
				rclcpp::sleep_for(std::chrono::seconds(5));
				catch_state_=CatchState::end;
			}
			if((this->get_clock()->now().nanoseconds() / 1000-time_find_start)>12000000){//>12s
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

// PID控制
// n_x:当前位置x(m) n_y:当前位置y(m) n_z:当前位置z(m) n_yaw:当前偏航角(°) t_x:目标位置x(m) t_y:目标位置y(m) t_z:目标位置z(m) t_yaw:目标偏航角(°) kp:比例系数 ki:积分系数 kd:微分系数
// 返回值：是否到达目标位置
bool OffboardControl::publish_trajectory_setpoint(double n_x,double n_y ,double n_z ,double t_x,double t_y ,double t_z  , double accuracy,double kp_xy ,double ki_xy ,double kd_xy){
    //RCLCPP_INFO(this->get_logger(), "Publishing setpoint: x=%f, y=%f, z=%f, yaw=%f", x, y, z, yaw);
	static double previous_error_x = t_x - n_x;
    static double previous_error_y = t_y - n_y;
    static double previous_error_z = t_z - n_z;

    static double integral_x = 0;
    static double integral_y = 0;
    static double integral_z = 0;

	double error_x = t_x - n_x;
	double error_y = t_y - n_y;
	double error_z = t_z - n_z;
 
	const static int n = 10;
	const static double integral_limit = 0.1;
	static double integral_[n][4] = {0}; // 0:x, 1:y, 2:z, 3:yaw
	static int i = 0;
	// 移除旧的积分值
	integral_x -= integral_[i][0];
	integral_y -= integral_[i][1];
	integral_z -= integral_[i][2];
	// 添加新的积分值
	integral_[i][0] = error_x * dt;
	integral_[i][1] = error_y * dt;
	integral_[i][2] = error_z * dt;
	// 更新积分，并引入积分限幅
	integral_x = std::min(std::max(integral_x + error_x * dt, -integral_limit), integral_limit);
	integral_y = std::min(std::max(integral_y + error_y * dt, -integral_limit), integral_limit);
	integral_z = std::min(std::max(integral_z + error_z * dt, -integral_limit), integral_limit);
	i = (i+1)%n;


	double velocity_x = (previous_error_x - error_x) / dt;
	double velocity_y = (previous_error_y - error_y) / dt;
	double velocity_z = (previous_error_z - error_z) / dt;

	//double derivative_x = (error_x - previous_error_x) / dt;
	//double derivative_y = (error_y - previous_error_y) / dt;
	//double derivative_z = (error_z - previous_error_z) / dt;
	//double derivative_yaw = (error_yaw - previous_error_yaw) / dt;	

	double output_x = kp_xy * error_x + ki_xy * integral_x + kd_xy * velocity_x;
	double output_y = kp_xy * error_y + ki_xy * integral_y + kd_xy * velocity_y;
	double output_z = kp * error_z + ki * integral_z + kd * velocity_z;

	previous_error_x = error_x;
	previous_error_y = error_y;
	previous_error_z = error_z;

	/*
	if(abs(output_x)>max_vx||abs(output_y)>max_vy){
		double rx= abs(output_x)/max_vx;
		double ry= abs(output_y)/max_vy;
		if(rx>ry){
			output_x*=rx;
			output_y*=rx;
		}else{
			output_x*=ry;
			output_y*=ry;
		}
	}*/
	double output_xy_d = sqrt(pow(output_x,2)+pow(output_y,2));
	//double output_xy_d = output_x + output_y;
	const double max_vxy = sqrt(pow(max_vx,2)+pow(max_vy,2));
	//const double max_vxy = max_vx + max_vy;
	if(output_xy_d>max_vxy){
		double r= max_vxy/output_xy_d;
		output_x*=r;
		output_y*=r;
	}
	if (output_x > max_vx) output_x = max_vx;
	if (output_x < -max_vx) output_x = -max_vx;
	if (output_y > max_vy) output_y = max_vy;
	if (output_y < -max_vy) output_y = -max_vy;
	if (output_z > max_vz) output_z = max_vz;
	if (output_z < -max_vz) output_z = -max_vz;
\	RCLCPP_INFO(this->get_logger(), "publish_trajectory_setpoint: error: x=%f, y=%f, z=%f, yaw=", error_x, error_y, error_z);
	RCLCPP_INFO(this->get_logger(), "publish_trajectory_setpoint: integral: x=%f, y=%f, z=%f, yaw=", integral_x, integral_y, integral_z);
	RCLCPP_INFO(this->get_logger(), "publish_trajectory_setpoint: velocity: x=%f, y=%f, z=%f, yaw=", velocity_x, velocity_y, velocity_z);

	RCLCPP_INFO(this->get_logger(), "publish_trajectory_setpoint: output: x=%f, y=%f, z=%f, yaw=", output_x, output_y, output_z);
    send_velocity_command(output_x, output_y, output_z, 0);
	// if(at_check_point()){
 	// 	//RCLCPP_INFO(this->get_logger(), "at_check_point");
	// 	previous_error_x = 0;
	// 	previous_error_y = 0;
	// 	previous_error_z = 0;
	// 	previous_error_yaw = 0;
	// 	integral_x = 0;
	// 	integral_y = 0;
	// 	integral_z = 0;
	// 	integral_yaw = 0;
	// }
    if(abs(n_x-t_x)<accuracy && abs(n_x-t_x)<accuracy && abs(n_x-t_x)<accuracy){
 		//RCLCPP_INFO(this->get_logger(), "at_check_point");
		previous_error_x = 0;
		previous_error_y = 0;
		previous_error_z = 0;
		integral_x = 0;
		integral_y = 0;
		integral_z = 0;
        return true;
	}
    return false;
}