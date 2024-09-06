#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

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
	double kp = 0.41;  // 比例参数
	double ki = 0.06;  // 积分参数
	double kd = 0.28;  // 微分参数
	double kp_yaw = 0.20;  // 比例参数
	double ki_yaw = 0.04;  // 积分参数
	double kd_yaw = 0.04;  // 微分参数
	double max_vx=2; //前后方向最大速度
	double max_vy=2; //左右方向最大速度
	double max_vz=1; //上下方向最大速度
	double max_yaw=10; //最大角速度(°/s)

    void PidRTL(double x,double y,double frtl);
    bool publish_trajectory_setpoint_z(double *z,double accuracy);
};

int main(int argc, char *argv[]) {
    int frtl = 0;
    ? timestamp0 = this->get_clock()->now().nanoseconds() / 1000 ; 
    while (true)
    {
        float x = yolo_->get_x();
        float y = yolo_->get_y();
        if(x!=0 && y!=0){
            double dx = x;
            double dy = y;
            PidRTL(dx,dy,frtl);
            //frtl=get_value(1)
        }
        else if(this->get_clock()->now().nanoseconds() / 1000 - timestamp0 > 290){
            command_takeoff_or_land("LAND");
            rclcpp::sleep_for(10s);
            arm_motors(false);
            break;
        }
    }
    std::cout << (this->get_clock()->now().nanoseconds() / 1000 - timestamp0) << std::endl;
		
}

void OffboardControl::PidRTL(double x,double y,double frtl){
	// PID控制
	double Kp = 0.5;  // 比例系数 0.5/0.47
	double Ki = 0.1;  // 积分系数 0.1
	double Kd = 0.02;  // 微分系数
	double target_X = 69;  // 目标X轴坐标/70
	double current_X = 0;  // 当前X轴坐标
	double target_Y = 51;  // 目标Y轴坐标/53
	double current_Y = 0;  // 当前Y轴坐标
	double error_priorX = 0;  // 上一次误差
	double integralX = 0;  // 积分
	double derivativeX = 0;  // 微分
	double error_priorY = 0;  // 上一次误差
	double integralY = 0;  // 积分
	double derivativeY = 0;  // 微分
	current_X=x/640*140; // 测量当前X坐标
	current_Y=y/480*105; // 测量当前Y坐标
	if (current_X<=67 || current_X>=71){
		// 计算误差
		double errorX = target_X - current_X;
		// 计算积分
		integralX = integralX + errorX;
		// 计算微分
		derivativeX = errorX - error_priorX;
		// 计算控制量
		double controlX = Kp * errorX + Ki * integralX + Kd * derivativeX;
		// 更新上一次误差
		error_priorX = errorX;
		// 应用控制量到无人机 
		double vy=controlX*-0.01;
		send_velocity_command(0,vy,0,0);
	}
	if (current_Y<=50 || current_Y>=52){
		// 计算误差
		double errorY = target_Y - current_Y;
		// 计算积分
		integralY = integralY + errorY;
		// 计算微分
		derivativeY = errorY - error_priorY;
		// 计算控制量
		double controlY = Kp * errorY + Ki * integralY + Kd * derivativeY;
		// 更新上一次误差
		error_priorY = errorY;
		// 应用控制量到无人机
		double vx=controlY*0.01;
		send_velocity_command(vx,0,0,0);
	}
	if (location.local_frame.z>=3.5){
		double z = 3.2;			
		publish_trajectory_setpoint_z(&z,0.1);
		send_velocity_command(0, 0, z, 0);
	}
	if (current_X>=67 && current_X<=71 && current_Y>=50 && current_Y<=52){// and vehicle.location.global_relative_frame.alt < 2.0):
		frtl=frtl+1;
		if(frtl>= 288){
			send_velocity_command(0,0,0.8,0);
			rclcpp::sleep_for(std::chrono::seconds(3));
			command_takeoff_or_land("LAND");
			rclcpp::sleep_for(std::chrono::seconds(5));
			arm_motors(false);
		}
	}else if(location.local_frame.z<=1.5){
		send_velocity_command(0,0,0,0);
		rclcpp::sleep_for(std::chrono::seconds(3));
		command_takeoff_or_land("LAND");
		rclcpp::sleep_for(std::chrono::seconds(10));
		arm_motors(false);
	}
}


// x/y/z位置PID控制_本地坐标系
// x:目标位置(m) accuracy:精度(m)默认
// 返回值：是否到达目标位置
bool OffboardControl::publish_trajectory_setpoint_z(double *z,double accuracy){
    static double integral_z = 0;

	double error_z = *z - pose_.pose.position.z;
	RCLCPP_INFO(this->get_logger(), "publish_trajectory_setpoint: error_z=%f", error_z);
	const static int n = 20;
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