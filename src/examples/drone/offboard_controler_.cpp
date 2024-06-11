#include "offboard_control.hpp"

#include <Eigen/Dense>

//四元数欧拉角转换
Eigen::Quaterniond quaternion_from_euler(const Eigen::Vector3d &euler)
{
	// YPR is ZYX axes
	return Eigen::Quaterniond(Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()) *
				  Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
				  Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX()));
}
Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q)
{
	// YPR is ZYX axes
	return q.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
}
void quaternion_to_euler(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw)
{
	const auto euler = quaternion_to_euler(q);
	roll = euler.x();
	pitch = euler.y();
	yaw = euler.z();
}
double OffboardControl::PrintYaw(void){
		quaternion.w() = pose_.pose.orientation.w;
		quaternion.x() = pose_.pose.orientation.x;
		quaternion.y() = pose_.pose.orientation.y;
		quaternion.z() = pose_.pose.orientation.z;
		euler = quaternion_to_euler(quaternion);
		heading = euler(2); //弧度制
		// std::cout << "heading: " << heading/PI*180 << std::endl;
		return heading/PI*180; //角度制
	}
#include <GeographicLib/Geodesic.hpp>//https://geographiclib.sourceforge.io/C++/doc/
#include <GeographicLib/LocalCartesian.hpp>
// 该函数将地理坐标（经度、纬度、高度）转换为（flu）坐标。
// 函数接受七个参数：lat、lon 和 alt 是要转换的地理坐标；lat0、lon0 和 alt0 是原点的地理坐标；x、y 和 z 是输出的 NED 坐标，它们是通过引用传递的，所以函数会直接修改它们的值。
void geodetic_to_flu(double lat, double lon, double alt, double lat0, double lon0, double alt0, double &x, double &y, double &z);
void geodetic_to_flu(double lat, double lon, double alt, double lat0, double lon0, double alt0, double &x, double &y, double &z)
{
	try {
		GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
		GeographicLib::LocalCartesian proj(lat0, lon0, alt0, earth);
		proj.Forward(lat, lon, alt, x, y, z);
	}
	catch (const std::exception& e) {
		std::cerr << "Caught exception: " << e.what() << "\n";
	}
}
// 这是一个使用GeographicLib库实现的函数，
// 它将一个地理坐标（纬度、经度、高度，即LLH）加上一个在本地坐标系（前/左/上，即FLU）中的偏移量，然后返回新的地理坐标。
// lat、lon 和 alt 是输入的地理坐标；offset_f、offset_l 和 offset_u 是在本地坐标系中的偏移量；lat_new、lon_new 和 alt_new 是输出的地理坐标。
void add_flu_offset_to_llh(double lat, double lon, double alt, double offset_f, double offset_l, double offset_u, double &lat_new, double &lon_new, double &alt_new){
    try {
        GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
        GeographicLib::LocalCartesian proj(lat, lon, alt, earth);
        double x, y, z;
        proj.Forward(lat, lon, alt, x, y, z);
        x += offset_f;
        y += offset_l;
        z += offset_u;
        proj.Reverse(x, y, z, lat_new, lon_new, alt_new);
    }
    catch (const std::exception& e) {
        std::cerr << "Caught exception: " << e.what() << "\n";
    }
}



// 飞行到指定位置（相对于世界坐标系）
// x:前后方向位置(m) y:左右方向位置(m) z:高度(m) yaw:偏航角(°）
void OffboardControl::trajectory_setpoint_takeoff(double x,double y ,double z ,double yaw){
	set_target_point("world",start_temp.x+x,start_temp.y+y,z,yaw);
	RCLCPP_INFO(this->get_logger(),"trajectory_setpoint_takeoff: et:%f %f %f",end_temp.x, end_temp.y, end_temp.z); 
	if(pose_.header.stamp.sec == 0){
		RCLCPP_INFO(this->get_logger(), "No pose data received yet");
		return;
	}
	publish_trajectory_setpoint(end_temp.x,end_temp.y,z,yaw);
}
// 飞行到指定位置（相对于当前位置）
// x:前后方向位置(m) y:左右方向位置(m) z:高度(m) yaw:偏航角(°) accuracy=DEFAULT_ACCURACY:精度(m)
void OffboardControl::trajectory_setpoint(double x,double y ,double z ,double yaw,double accuracy){
	static bool first=true;
	if(first){
		get_target_location( &x, &y);
		RCLCPP_INFO(this->get_logger(),"trajectory_setpoint: x:%f y:%f",x,y);
		set_target_point("base_link",x,y,z,yaw);
		first=false;
	}
	if(pose_.header.stamp.sec == 0){
		RCLCPP_INFO(this->get_logger(), "No pose data received yet");
		return;
	}
	if(at_check_point(accuracy)){
		RCLCPP_INFO(this->get_logger(), "at_check_point");
		first=true;
		return;
	}
	publish_trajectory_setpoint(end_temp.x,end_temp.y,end_temp.z,end_temp.yaw);
}
// 飞行到指定位置（相对于起飞点）
// x:前后方向位置(m) y:左右方向位置(m) z:高度(m) yaw:偏航角(°) accuracy=DEFAULT_ACCURACY:精度(m)
void OffboardControl::trajectory_setpoint_start(double x,double y ,double z ,double yaw,double accuracy){
	static bool first=true;
	if(first){
	get_target_location( &x, &y);
	RCLCPP_INFO(this->get_logger(),"trajectory_setpoint_start: x:%f y:%f",x,y);
	set_target_point("start",x,y,z,yaw);
	first=false;
	}
	if(pose_.header.stamp.sec == 0){
		RCLCPP_INFO(this->get_logger(), "No pose data received yet");
		return;
	}
	if(at_check_point(accuracy)){
		RCLCPP_INFO(this->get_logger(), "at_check_point");
		first=true;
		return;
	}
	publish_trajectory_setpoint(end_temp.x,end_temp.y,end_temp.z,end_temp.yaw);
}
// 定高悬停//定角度悬停
// vx=0:前后方向速度(m/s) vy=0:左右方向速度(m/s) z:高度(m) yaw:偏航角(°) time:持续时间(s) accuracy=DEFAULT_ACCURACY:精度(m) 
// 返回值：是否到达规定时间
bool OffboardControl::alt_hold(double vx,double vy ,double z ,double yaw,double time,double accuracy){
	static bool first=true;
	static double vx1=0,vy1=0;
	if(first){
	get_target_location(&vx, &vy);
	vx1=vx;
	vy1=vy;
	RCLCPP_INFO(this->get_logger(),"trajectory_setpoint_start: vx:%f vy:%f",vx,vy);
	start_temp=end_temp;
	set_target_point("base_link",0,0,z,yaw);
	first=false;
	}
	if(pose_.header.stamp.sec == 0){
		RCLCPP_INFO(this->get_logger(), "No pose data received yet");
		return false;
	}
	z=end_temp.z+z;
	yaw=end_temp.yaw+yaw;
	if(
		!set_time(time)
	){
		if(
			//publish_trajectory_setpoint_yaw(&yaw)&
			publish_trajectory_setpoint_z(&z,accuracy)
		){
			RCLCPP_INFO(this->get_logger(), "at_check_point");
			return false;
		}else{
			RCLCPP_INFO(this->get_logger(), "alt_hold: z=%f,yaw=%f", z,yaw);
		send_velocity_command(vx1, vy1, z, yaw);
		return false;
		}
	}else{
		first=true;
		return true;
	}
}
// PID控制_本地坐标系
// x/y/z/yaw位置PID控制
// x:目标位置(m) y:~ z:~ yaw:目标角度(°) //at_check_point(accuracy:精度(m)默认)
// 返回值：是否到达目标位置
void OffboardControl::publish_trajectory_setpoint(double x,double y ,double z ,double yaw){
    //RCLCPP_INFO(this->get_logger(), "Publishing setpoint: x=%f, y=%f, z=%f, yaw=%f", x, y, z, yaw);
	//static double previous_error_x = x - pose_.pose.position.x;
    //static double previous_error_y = y - pose_.pose.position.y;
    //static double previous_error_z = z - pose_.pose.position.z;
    //static double previous_error_yaw = yaw - quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);//-pi-pi

    static double integral_x = 0;
    static double integral_y = 0;
    static double integral_z = 0;
    static double integral_yaw = 0;

	double error_x = x - pose_.pose.position.x;
	double error_y = y - pose_.pose.position.y;
	double error_z = z - pose_.pose.position.z;
	double error_yaw = yaw - quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);
	
	const static int n = 10;
	static double integral_[n][4] = {0}; // 0:x, 1:y, 2:z, 3:yaw
	static int i = 0;
	integral_[i][0] = error_x * dt;
	integral_[i][1] = error_y * dt;
	integral_[i][2] = error_z * dt;
	integral_[i][3] = error_yaw * dt;
	integral_x += error_x * dt;
	integral_y += error_y * dt;
	integral_z += error_z * dt;
    integral_yaw += error_yaw * dt;
	integral_x -= integral_[(i+1)%n][0];
	integral_y -= integral_[(i+1)%n][1];
	integral_z -= integral_[(i+1)%n][2];
	integral_yaw -= integral_[(i+1)%n][3];
	i = (i+1)%n;
	
    //double derivative_x = (error_x - previous_error_x) / dt;
    //double derivative_y = (error_y - previous_error_y) / dt;
    //double derivative_z = (error_z - previous_error_z) / dt;
    //double derivative_yaw = (error_yaw - previous_error_yaw) / dt;	

    double output_x = kp * error_x + ki * integral_x + kd * velocity_.twist.linear.x;
    double output_y = kp * error_y + ki * integral_y + kd * velocity_.twist.linear.y;
    double output_z = kp * error_z + ki * integral_z + kd * velocity_.twist.linear.z;
    double output_yaw = kp_yaw * error_yaw + ki_yaw * integral_yaw + kd_yaw * velocity_.twist.angular.z;

    //previous_error_x = error_x;
    //previous_error_y = error_y;
    //previous_error_z = error_z;
    //previous_error_yaw = error_yaw;
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
	if(output_xy_d>pow(max_vx,2)+pow(max_vy,2)){
		double r= sqrt(output_xy_d)/(sqrt(pow(output_x,2)+pow(output_y,2)));
		output_x*=r;
		output_y*=r;
	}
	if (output_x > max_vx) output_x = max_vx;
	if (output_x < -max_vx) output_x = -max_vx;
	if (output_y > max_vy) output_y = max_vy;
	if (output_y < -max_vy) output_y = -max_vy;
	if (output_z > max_vz) output_z = max_vz;
	if (output_z < -max_vz) output_z = -max_vz;
	if (output_yaw > 180) output_yaw = -360+output_yaw;
	if (output_yaw < -180) output_yaw = 360+output_yaw;
	if (output_yaw > max_yaw) output_yaw = max_yaw;
	if (output_yaw < -max_yaw) output_yaw = -max_yaw;

	RCLCPP_INFO(this->get_logger(), "publish_trajectory_setpoint: output: x=%f, y=%f, z=%f, yaw=%f", output_x, output_y, output_z, output_yaw);
    send_velocity_command(output_x, output_y, output_z, output_yaw);
	if(at_check_point()){
		//RCLCPP_INFO(this->get_logger(), "at_check_point");
		//previous_error_x = 0;
		//previous_error_y = 0;
		//previous_error_z = 0;
		//previous_error_yaw = 0;
		integral_x = 0;
		integral_y = 0;
		integral_z = 0;
		integral_yaw = 0;
	}
}
// x/y/z位置PID控制_本地坐标系
// x:目标位置(m) accuracy:精度(m)默认
// 返回值：是否到达目标位置
bool OffboardControl::publish_trajectory_setpoint_z(double *z,double accuracy){
    static double integral_z = 0;

	double error_z = *z - pose_.pose.position.z;
	RCLCPP_INFO(this->get_logger(), "publish_trajectory_setpoint: error_z=%f", error_z);
	const static int n = 10;
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
// 旋转角度PID控制_本地坐标系
// yaw:目标角度（度） accuracy:精度（度）默认为
// 返回值：是否到达目标角度
bool OffboardControl::publish_trajectory_setpoint_yaw(double *yaw,double accuracy){
    static double integral_yaw = 0;

	double error_yaw = *yaw - quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);
	
	const static int n = 10;
	static double integral_[n] = {0}; // 0:x, 1:y, 2:z, 3:yaw
	static int i = 0;
	integral_[i] = error_yaw * dt;
    integral_yaw += error_yaw * dt;
	integral_yaw -= integral_[(i+1)%n];
	i = (i+1)%n;
	
    double output_yaw = kp_yaw * error_yaw + ki_yaw * integral_yaw + kd_yaw * velocity_.twist.angular.z;

	if (output_yaw > 180) output_yaw = -360+output_yaw;
	if (output_yaw < -180) output_yaw = 360+output_yaw;
	if (output_yaw > max_yaw) output_yaw = max_yaw;
	if (output_yaw < -max_yaw) output_yaw = -max_yaw;

	RCLCPP_INFO(this->get_logger(), "publish_trajectory_setpoint: output_yaw=%f", output_yaw);
	RCLCPP_INFO(this->get_logger(), "publish_trajectory_setpoint: yaw=%f", *yaw);
	RCLCPP_INFO(this->get_logger(), "publish_trajectory_setpoint: pose_yaw=%f", quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w));
	if(abs(*yaw - quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w)) <=accuracy){
		integral_yaw = 0;
		*yaw = output_yaw;
		return true;
	}
	*yaw = output_yaw;
	return false;
}
//time(s), linear_x, linear_y, linear_z, angular_z(jiaoduzhi/s)
bool OffboardControl::set_time(double time){
	static bool first=true;
	static double find_start;
	if(first){
		find_start = this->get_clock()->now().nanoseconds() / 1000;
		first=false;
	}
	RCLCPP_INFO(this->get_logger(), "set_time: %f", (this->get_clock()->now().nanoseconds() / 1000-find_start)/1000000.0);
	RCLCPP_INFO(this->get_logger(), "set_time: time:%f", time);
	if((this->get_clock()->now().nanoseconds() / 1000-find_start)>1000000*time){
		first=true;
		return true;
	}
	else{
		return false;
  	}
}
// 发布定时速度控制指令
// linear_x, linear_y, linear_z 为速度(m/s), angular_z为角速度(度/s)
// send_velocity_command_with_time(3, 0, 0, 0, 3);//前进3m/s,3s后停止
bool OffboardControl::send_velocity_command_with_time(double linear_x, double linear_y, double linear_z, double angular_z,double time){
	static bool first=true;
	static double find_start;
	get_target_location(&linear_x, &linear_y);
	if(first){
		set_target_point("start_temp");
		////set_start_temp_point(pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z, quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w));
		find_start = this->get_clock()->now().nanoseconds() / 1000;
		first=false;
	}
	geometry_msgs::msg::TwistStamped msg;
	if((this->get_clock()->now().nanoseconds() / 1000-find_start)>1000000*time){
		send_velocity_command(0, 0, 0, 0);
		// 或者
		// msg.twist.linear.x = 0;
		// msg.twist.linear.y = 0;
		// msg.twist.linear.z = 0;
		// msg.twist.angular.z = 0;
		// msg.header.stamp = this->now();
		// msg.header.frame_id = "base_link";
		// twist_stamped_publisher_->publish(msg);
		set_target_point("end_temp");
		////set_end_temp_point(pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z, quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w));
		first=true;
		return true;
	}
	else{
		send_velocity_command(linear_x, linear_y, linear_z, angular_z);
		// 或者
		// msg.twist.linear.x = linear_x;
		// msg.twist.linear.y = linear_y;
		// msg.twist.linear.z = linear_z;
		// msg.twist.angular.z = angular_z/2/PI;
		// msg.header.stamp = this->now();
		// msg.header.frame_id = "base_link";
		// twist_stamped_publisher_->publish(msg);
		return false;
  	}
}

#define PRE_TEST //测试设定位置是否更改，用于循环中，不影响结果
//设置目标点_local
//mode: base_link, start, world, start_temp, end_temp
//x,y,z: 相对于当前位置的偏移量，默认为0m
//yaw: 相对于当前位置的偏转角，默认为0度
//mode为base_link时，设置相对于机体坐标系的目标点，默认不改变位置
//mode为start时，设置相对于起飞点的目标点，默认不改变
//mode为world时，设置绝对坐标系的目标点，默认不改变
//mode为start_temp时，设置相对于结束位置的临时目标点，默认不改变
//mode为end_temp时，设置相对于出发位置的临时目标点，默认为当前位置
//mode为end时，设置相对于降落点的目标点，默认不改变
//set_target_point("base_link",0,0,5,0);//set_target_point("base_link");
//set_target_point("start",0,0,5,0);//set_target_point("start");
//set_target_point("world",0,0,5,0);//set_target_point("world");
//set_target_point("start_temp",0,0,5,0);//set_target_point("start_temp");
//set_target_point("end_temp",0,0,5,0);//set_target_point("end_temp");
//set_target_point("end",0,0,5,0);//set_target_point("end");

void OffboardControl::set_target_point(std::string mode,double x,double y,double z,double yaw){
	#ifdef PRE_TEST
	static double x_pre;
	static double y_pre;
	static double z_pre;
	//static double yaw_pre;
	if(x!=x_pre || y!=y_pre || z!=z_pre){
		x_pre=x;
		y_pre=y;
		z_pre=z;
		//yaw_pre=yaw;
		//mode_pre=mode;
	#endif
		if(mode=="base_link"){
			start_temp=end_temp;
			end_temp.x=start_temp.x+x;
			end_temp.y=start_temp.y+y;
			end_temp.z=start_temp.z+z;
			end_temp.yaw=start_temp.yaw+yaw;
			RCLCPP_INFO(this->get_logger(),"base_link:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw);
		}else if(mode=="start"){
			start_temp=end_temp;
			end_temp.x=start.x+x;
			end_temp.y=start.y+y;
			end_temp.z=start.z+z;
			end_temp.yaw=start.yaw+yaw;
			RCLCPP_INFO(this->get_logger(),"start:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z,end_temp.yaw);
		}else if(mode=="world"){
			start_temp=end_temp;
			end_temp.x=x;
			end_temp.y=y;
			end_temp.z=z;
			end_temp.yaw=yaw;
			RCLCPP_INFO(this->get_logger(),"world:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw);
		}else if(mode=="start_temp"){
			if(x==0.0){start_temp.x=pose_.pose.position.x;}else{start_temp.x=x;}
			if(y==0.0){start_temp.y=pose_.pose.position.y;}else{start_temp.y=y;}
			if(z==0.0){start_temp.z=pose_.pose.position.z;}else{start_temp.z=z;}
			if(yaw==0.0){start_temp.yaw=quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);}else{start_temp.yaw=yaw;}
			RCLCPP_INFO(this->get_logger(),"start_temp:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw);
		}else if(mode=="end_temp"){
			if(x==0.0){end_temp.x=pose_.pose.position.x;}else{end_temp.x=x;}
			if(y==0.0){end_temp.y=pose_.pose.position.y;}else{end_temp.y=y;}
			if(z==0.0){end_temp.z=pose_.pose.position.z;}else{end_temp.z=z;}
			if(yaw==0.0){end_temp.yaw=quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);}else{end_temp.yaw=yaw;}

			RCLCPP_INFO(this->get_logger(),"end_temp:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw);
		}else{
			RCLCPP_ERROR(this->get_logger(), "No such mode");
		}
	#ifdef PRE_TEST
	}else{
		RCLCPP_INFO(this->get_logger(),"set_target_point:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw); 
		return;
	}
	#endif
}

// template <typename T>
// class AtCheckPoint{
// public:
// 	AtCheckPoint(T t):t_(t){}
// 	bool operator()(const T& t){
// 		return t_->at_check_point();
// 	}
// private:
// 	T t_;
// };

// template <typename T>
// void OffboardControl::set_target_point(std::string mode,double x,double y,double z,double yaw,bool(*at_check_point)(const T&)){
// 	#ifdef PRE_TEST
// 	static double x_pre;
// 	static double y_pre;
// 	static double z_pre;
// 	//static double yaw_pre;
// 	if(x!=x_pre || y!=y_pre || z!=z_pre){
// 		x_pre=x;
// 		y_pre=y;
// 		z_pre=z;
// 		//yaw_pre=yaw;
// 		//mode_pre=mode;
// 	#endif
// 		if(mode=="base_link"){
// 			start_temp=end_temp;
// 			end_temp.x=start_temp.x+x;
// 			end_temp.y=start_temp.y+y;
// 			end_temp.z=start_temp.z+z;
// 			end_temp.yaw=start_temp.yaw+yaw;
// 			RCLCPP_INFO(this->get_logger(),"base_link:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw);
// 		}else if(mode=="start"){
// 			start_temp=end_temp;
// 			end_temp.x=start.x+x;
// 			end_temp.y=start.y+y;
// 			end_temp.z=start.z+z;
// 			end_temp.yaw=start.yaw+yaw;
// 			RCLCPP_INFO(this->get_logger(),"start:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z,end_temp.yaw);
// 		}else if(mode=="world"){
// 			start_temp=end_temp;
// 			end_temp.x=x;
// 			end_temp.y=y;
// 			end_temp.z=z;
// 			end_temp.yaw=yaw;
// 			RCLCPP_INFO(this->get_logger(),"world:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw);
// 		}else if(mode=="start_temp"){
// 			if(x==0.0){start_temp.x=pose_.pose.position.x;}else{start_temp.x=x;}
// 			if(y==0.0){start_temp.y=pose_.pose.position.y;}else{start_temp.y=y;}
// 			if(z==0.0){start_temp.z=pose_.pose.position.z;}else{start_temp.z=z;}
// 			if(yaw==0.0){start_temp.yaw=quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);}else{start_temp.yaw=yaw;}
// 			RCLCPP_INFO(this->get_logger(),"start_temp:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw);
// 		}else if(mode=="end_temp"){
// 			if(x==0.0){end_temp.x=pose_.pose.position.x;}else{end_temp.x=x;}
// 			if(y==0.0){end_temp.y=pose_.pose.position.y;}else{end_temp.y=y;}
// 			if(z==0.0){end_temp.z=pose_.pose.position.z;}else{end_temp.z=z;}
// 			if(yaw==0.0){end_temp.yaw=quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);}else{end_temp.yaw=yaw;}

// 			RCLCPP_INFO(this->get_logger(),"end_temp:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw);
// 		}else{
// 			RCLCPP_ERROR(this->get_logger(), "No such mode");
// 		}
// 	#ifdef PRE_TEST
// 	}else{
// 		RCLCPP_INFO(this->get_logger(),"set_target_point:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw); 
// 		return;
// 	}
// 	#endif
// }

// void OffboardControl::set_target_point_global(std::string mode,double x,double y,double z,double yaw){
// 	#ifdef PRE_TEST
// 	static double x_pre;
// 	static double y_pre;
// 	static double z_pre;
// 	//static double yaw_pre;
// 	if(x!=x_pre || y!=y_pre || z!=z_pre){
// 		x_pre=x;
// 		y_pre=y;
// 		z_pre=z;
// 		//yaw_pre=yaw;
// 		//mode_pre=mode;
// 	#endif
// 		if(mode=="base_link"){
// 			start_temp_global=end_temp_global;
// 			end_temp_global.x=start_temp_global.x+x;
// 			end_temp_global.y=start_temp_global.y+y;
// 			end_temp_global.z=start_temp_global.z+z;
// 			end_temp_global.yaw=start_temp_global.yaw+yaw;
// 			RCLCPP_INFO(this->get_logger(),"base_link:%f %f %f %f",end_temp_global.x, end_temp_global.y, end_temp_global.z, end_temp_global.yaw);
// 		}else if(mode=="start"){
// 			start_temp_global=end_temp_global;
// 			end_temp_global.x=start_global.x+x;
// 			end_temp_global.y=start_global.y+y;
// 			end_temp_global.z=start_global.z+z;
// 			end_temp_global.yaw=start_global.yaw+yaw;
// 			RCLCPP_INFO(this->get_logger(),"start_global:%f %f %f %f",end_temp_global.x, end_temp_global.y, end_temp_global.z,end_temp_global.yaw);
// 		}else if(mode=="world"){
// 			start_temp_global=end_temp_global;
// 			end_temp_global.x=x;
// 			end_temp_global.y=y;
// 			end_temp_global.z=z;
// 			end_temp_global.yaw=yaw;
// 			RCLCPP_INFO(this->get_logger(),"world:%f %f %f %f",end_temp_global.x, end_temp_global.y, end_temp_global.z, end_temp_global.yaw);
// 		}else if(mode=="start_temp_global"){
// 			if(x==0.0){start_temp_global.x=pose_.pose.position.x;}else{start_temp_global.x=x;}
// 			if(y==0.0){start_temp_global.y=pose_.pose.position.y;}else{start_temp_global.y=y;}
// 			if(z==0.0){start_temp_global.z=pose_.pose.position.z;}else{start_temp_global.z=z;}
// 			if(yaw==0.0){start_temp_global.yaw=quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);}else{start_temp_global.yaw=yaw;}
// 			RCLCPP_INFO(this->get_logger(),"start_temp_global:%f %f %f %f",end_temp_global.x, end_temp_global.y, end_temp_global.z, end_temp_global.yaw);
// 		}else if(mode=="end_temp_global"){
// 			if(x==0.0){end_temp_global.x=pose_.pose.position.x;}else{end_temp_global.x=x;}
// 			if(y==0.0){end_temp_global.y=pose_.pose.position.y;}else{end_temp_global.y=y;}
// 			if(z==0.0){end_temp_global.z=pose_.pose.position.z;}else{end_temp_global.z=z;}
// 			if(yaw==0.0){end_temp_global.yaw=quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);}else{end_temp_global.yaw=yaw;}

// 			RCLCPP_INFO(this->get_logger(),"end_temp_global:%f %f %f %f",end_temp_global.x, end_temp_global.y, end_temp_global.z, end_temp_global.yaw);
// 		}else{
// 			RCLCPP_ERROR(this->get_logger(), "No such mode");
// 		}
// 	#ifdef PRE_TEST
// 	}else{
// 		RCLCPP_INFO(this->get_logger(),"set_target_point:%f %f %f %f",end_temp_global.x, end_temp_global.y, end_temp_global.z, end_temp_global.yaw); 
// 		return;
// 	}
// 	#endif
// }

//确认到达目标点_local
//accuracy: 精度（默认为DEFAULT_ACCURACY）
//返回值：到达目标点返回true，否则返回false
bool OffboardControl::at_check_point(double accuracy){
	////////RCLCPP_INFO(this->get_logger(),"et:%f %f %f",end_temp.x, end_temp.y, end_temp.z); 
	////////RCLCPP_INFO(this->get_logger(),"vl:%f %f %f",pose_.pose.position.x-end_temp.x, pose_.pose.position.y - end_temp.y, pose_.pose.position.z - end_temp.z); 
	RCLCPP_INFO(this->get_logger()," p: %f %f %f %f",pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z, quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w));
	RCLCPP_INFO(this->get_logger()," e: %f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw);
	if(
		abs(pose_.pose.position.x - end_temp.x) <=accuracy && 
		abs(pose_.pose.position.y - end_temp.y) <=accuracy && 
		abs(pose_.pose.position.z - end_temp.z) <=accuracy 
		&& abs(quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w) - end_temp.yaw) <= DEFAULT_ACCURACY_YAW
		){
		
		return true;
	}
	else{
		return false;
	}	
}
// void OffboardControl::set_target_point(std::string mode,double x,double y,double z,double yaw){
// 	static double x_pre;
// 	static double y_pre;
// 	static double z_pre;
// 	//static double yaw_pre;
// 	if(x!=x_pre || y!=y_pre || z!=z_pre){
// 		x_pre=x;
// 		y_pre=y;
// 		z_pre=z;
// 		//yaw_pre=yaw;
// 		//mode_pre=mode;
// 		if(mode=="base_link"){
// 			set_drone_target_point_local(x,y,z,yaw);
// 			RCLCPP_INFO(this->get_logger(),"base_link:et:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw);
// 		}
// 		else if(mode=="start"){
// 			set_start_point_local(x,y,z,yaw);
// 			RCLCPP_INFO(this->get_logger(),"start:et:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z,end_temp.yaw);
// 		}
// 		else if(mode=="world"){
// 			set_world_point_local(x,y,z,yaw);
// 			RCLCPP_INFO(this->get_logger(),"world:et:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw);
// 		}
// 		else{
// 			RCLCPP_ERROR(this->get_logger(), "No such mode");
// 		}
// 	}else{
// 		RCLCPP_INFO(this->get_logger(),"set_target_point: et:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw); 
// 		return;
// 	}
// }
// void OffboardControl::set_start_temp_point(double x,double y,double z,double yaw){
// 	start_temp.x=x;
// 	start_temp.y=y;
// 	start_temp.z=z;
// 	start_temp.yaw=yaw;
// 	//RCLCPP_INFO(this->get_logger(),"set:et:%f %f %f",end_temp.x, end_temp.y, end_temp.z); 
// }
// void OffboardControl::set_end_temp_point(double x=0.0,double y=0.0,double z=0.0,double yaw=0.0){
//     if(x==0.0){end_temp.x=pose_.pose.position.x;}else{end_temp.x=x;}
//     if(y==0.0){end_temp.y=pose_.pose.position.y;}else{end_temp.y=y;}
//     if(z==0.0){end_temp.z=pose_.pose.position.z;}else{end_temp.z=z;}
//     if(yaw==0.0){end_temp.yaw=quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);}else{end_temp.yaw=yaw;}
// 	RCLCPP_INFO(this->get_logger(),"set:et:%f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw); 
// }

// void OffboardControl::set_drone_target_point_local(double x,double y,double z,double yaw){
// 	start_temp=end_temp;
// 	end_temp.x=start_temp.x+x;
// 	end_temp.y=start_temp.y+y;
// 	end_temp.z=start_temp.z+z;
// 	end_temp.yaw=start_temp.yaw+yaw;
	
// 	RCLCPP_INFO(this->get_logger(),"set:drone_et:%f %f %f",end_temp.x, end_temp.y, end_temp.z); 
// }

// void OffboardControl::set_start_point_local(double x,double y,double z,double yaw){
// 	start_temp=end_temp;
// 	end_temp.x=start.x+x;
// 	end_temp.y=start.y+y;
// 	end_temp.z=start.z+z;
// 	end_temp.yaw=start.yaw+yaw;
	
// 	//RCLCPP_INFO(this->get_logger(),"set:et:%f %f %f",end_temp.x, end_temp.y, end_temp.z); 
// }

// void OffboardControl::set_world_point_local(double x,double y,double z,double yaw){
// 	start_temp=end_temp;
// 	end_temp.x=x;
// 	end_temp.y=y;
// 	end_temp.z=z;
// 	end_temp.yaw=yaw;
	
// 	//RCLCPP_INFO(this->get_logger(),"set:et:%f %f %f",end_temp.x, end_temp.y, end_temp.z); 
// }

// bool OffboardControl::at_check_point(double accuracy){
// 	////////RCLCPP_INFO(this->get_logger(),"et:%f %f %f",end_temp.x, end_temp.y, end_temp.z); 
// 	////////RCLCPP_INFO(this->get_logger(),"vl:%f %f %f",pose_.pose.position.x-end_temp.x, pose_.pose.position.y - end_temp.y, pose_.pose.position.z - end_temp.z); 
// 	RCLCPP_INFO(this->get_logger()," p: %f %f %f %f",pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z, quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w));
// 	RCLCPP_INFO(this->get_logger()," e: %f %f %f %f",end_temp.x, end_temp.y, end_temp.z, end_temp.yaw);
// 	if(
// 		abs(pose_.pose.position.x - end_temp.x) <=accuracy && 
// 		abs(pose_.pose.position.y - end_temp.y) <=accuracy && 
// 		abs(pose_.pose.position.z - end_temp.z) <=accuracy 
// 		&& abs(quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w) - end_temp.yaw) <= DEFAULT_ACCURACY_YAW
// 		){
		
// 		return true;
// 	}
// 	else{
// 		return false;
// 	}	
// }

//欧拉角转四元数

//quaternion_from_euler(x,y,z,w)
double OffboardControl::quaternion_to_yaw(double x, double y, double z, double w) {
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return yaw/PI*180;
}

//quaternion_to_euler()
void OffboardControl::yaw_to_quaternion(double yaw, double* x, double* y, double* z, double* w) {
	*x = 0.0;
	*y = 0.0;
	*z = sin(yaw / 2);
	*w = cos(yaw / 2);
}
// #x,y为目标点相对于当前位置的偏移量
void OffboardControl::get_target_location( double* x, double* y,double delta_heading) {
    // Assuming that 'heading' is a member variable of the class representing the current heading
	RCLCPP_INFO(get_logger(), "default_heading: %f", default_heading);
	double yaw = heading + default_heading + delta_heading;
	if (yaw >= 360) {
		heading -= 360;
	}
	RCLCPP_INFO(get_logger(), "yaw: %f", yaw);
    // Convert yaw to radians
    double yaw_rad = yaw / 180 * PI;
	RCLCPP_INFO(get_logger(), "yaw_rad: %f", yaw_rad);
    // Assuming that 'distance' is the distance you want to move in the direction of 'yaw'
    //double distance = sqrt(*x**x+*y**y); // replace with your actual distance
	//RCLCPP_INFO(get_logger(), "distance: %f", distance);
    // Calculate the new position
	double x1=*x;
	double y1=*y;
    *x = x1 * cos(yaw_rad)-y1 * sin(yaw_rad);//
    *y = x1 * sin(yaw_rad)+y1 * cos(yaw_rad);//
	RCLCPP_INFO(get_logger(), "cos(yaw_rad): %f, sin(yaw_rad): %f", cos(yaw_rad), sin(yaw_rad));
	RCLCPP_INFO(get_logger(), "ldx: %f, ldy: %f", *x, *y);
}
// #将相对坐标系下的坐标 x:m,y:m,z:m,heading:degree
// 转换为全局坐标系下的经纬度坐标
//get_target_location(default_heading, );
//ros2 topic pub /mavros/setpoint_raw/global mavros_msgs/msg/GlobalPositionTarget '{header: "auto", latitude: -35.363262, longitude: 149.165237, altitude: 700.788637}'
void OffboardControl::get_target_location_global(double &x,double &y,double &z,double delta_heading) {
    double yaw = heading+default_heading+delta_heading;

    if(heading >= 360){
        heading -= 360;
	}
	std::cout << "heading: " << yaw << std::endl;
    
    double yaw_rad = yaw/180*PI;

	double x1=x;
	double y1=y;
	//double z1=z;
	// #change in lat and lon  
    double d_x = (x1 * cos(yaw_rad)-y1 * sin(yaw_rad)) ;//* 0.0000093;//
    double d_y = (x1 * sin(yaw_rad)+y1 * cos(yaw_rad)) ;//* 0.000009;//
	double d_z =  z ;//* 0.3048;
	//std::cout << "x: " << x << " y: " << y << " z: " << z << std::endl;
	//std::cout << "lat: " << location.global_frame.lat << " lon: " << location.global_frame.lon << " alt: " << location.global_frame.alt << std::endl;

	add_flu_offset_to_llh(location.global_frame.lat, location.global_frame.lon, location.global_frame.alt,d_x, d_y, d_z,  x, y, z);;

	// std::cout << "lat: " << x << " lon: " << y << " alt: " << z << std::endl;
	// xn: 0, yn: 100, zn: 10
	// lat_new: 0.000904368, lon_new: 0, alt_new: 10.0008
}
// #将相对坐标系下的坐标 x:m,y:m,z:m,heading:degree
// 转换为全局坐标系下的经纬度坐标
//get_target_location(default_heading
// OffboardControl::GlobalFrame OffboardControl::get_target_location(double dheading,double x,OffboardControl &offboard_control){
//     double heading = offboard_control.heading;
//     double alt = offboard_control.location.global_frame.alt + 5;
//     heading += dheading;
//     if (heading >= 360){
//         heading -= 360;
// 	}
    
//     double heading_radians = heading * M_PI / 180;

//     //change in lat and lon    
//     double dlon = x * sin(heading_radians) * 0.0000093;
//     double dlat = x * cos(heading_radians) * 0.000009;
    
//     // print("dlat: ", dlat, "dlon: ", dlon)

//     GlobalFrame target_location = {offboard_control.location.global_frame.lat + dlat, offboard_control.location.global_frame.lon + dlon, alt};
    
//     return target_location;
// }

// 发布全局GPS位置
// 输入相对移动距离
// x:m y:m z:m yaw:°
void OffboardControl::trajectory_setpoint_global(double x,double y ,double z ,double yaw){
	if(global_gps_.header.stamp.sec == 0){
		RCLCPP_INFO(this->get_logger(), "No pose data received yet");
		return;
	}
	get_target_location_global(x,y,z,yaw);
	// RCLCPP_INFO(this->get_logger(),"trajectory_setpoint_global: et:%f %f %f",end_temp.x, end_temp.y, end_temp.z);
	publish_setpoint_raw_global(x,y,z,yaw);
}

