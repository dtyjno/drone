#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol_local.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/command_home.hpp>


//#include <ardupilot_msgs/msg/global_position.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/altitude.hpp>
#include <mavros_msgs/msg/home_position.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/global_position_target.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <mavros_msgs/msg/state.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <string>
#include <cmath>



//  #define GET_GPS
#define PI 3.14159
//#define default_heading -0.044156
//#define default_heading -60  //yaw=-2.53
//M_PI dhaeding: 0.000000
#define DEFAULT_ACCURACY 0.3 //+-0.3m
#define DEFAULT_ACCURACY_YAW 1 //+-1°
#define DEFAULT_HEADING 90.00//角度制
//using namespace std::chrono;
// struct State {
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//   Eigen::Vector3d position;
//   Eigen::Vector3d velocity;
//   Eigen::Vector4d attitude;
// };
using namespace std::chrono_literals;

#include <iostream>
#include <thread>
// #include <Eigen/Dense>
//   std::vector<Eigen::Vector3d> vehicle_position_history_;
//   std::vector<geometry_msgs::msg::PoseStamped> posehistory_vector_;
//   std::vector<geometry_msgs::msg::PoseStamped> referencehistory_vector_;
//   Eigen::Vector3d vehicle_position_{Eigen::Vector3d::Zero()};
//   Eigen::Vector3d vehicle_velocity_{Eigen::Vector3d::Zero()};
//   Eigen::Vector4d vehicle_attitude_{Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)};
//   Eigen::Vector3d last_planning_position_{Eigen::Vector3d::Zero()};
//   Eigen::Vector3d previous_start_position_{Eigen::Vector3d::Zero()};
//   Eigen::Vector3d previous_return_start_position_{Eigen::Vector3d::Zero()};
//   Eigen::Vector3d mission_loiter_center_{Eigen::Vector3d::Zero()};
// void TerrainPlanner::mavLocalPoseCallback(const geometry_msgs::msg::PoseStamped &msg) {
//   vehicle_attitude_(0) = msg.pose.orientation.w;
//   vehicle_attitude_(1) = msg.pose.orientation.x;
//   vehicle_attitude_(2) = msg.pose.orientation.y;
//   vehicle_attitude_(3) = msg.pose.orientation.z;
// }

// class DaemonThread {
// public:
// 	static void run() {
// 		for (int i = 0; i < 600; i++) {
// 			std::cout << i << std::endl;
// 			std::this_thread::sleep_for(std::chrono::seconds(1));
// 		}
// 	}
// };


#include <Eigen/Dense>
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
// 	double PrintYaw(void){
// 		quaternion.w() = pose_.pose.orientation.w;
// 		quaternion.x() = pose_.pose.orientation.x;
// 		quaternion.y() = pose_.pose.orientation.y;
// 		quaternion.z() = pose_.pose.orientation.z;
// 		euler = quaternion_to_euler(quaternion);
// 		heading = euler(2); //弧度制
// 		// std::cout << "heading: " << heading/PI*180 << std::endl;
// 		return heading/PI*180; //角度制
// 	}
	
// };

// #include <vision_msgs/msg/detection2_d_array.hpp>
// // #include "sensor_msgs/msg/image.hpp"
// // #include <opencv2/opencv.hpp>
// // #include "cv_bridge/cv_bridge.h"
// // using namespace Eigen;
// // using namespace std;
// // using namespace cv;
// // using namespace cv::dnn;
// class YOLO : public rclcpp::Node{
// public:
// 	YOLO()
// 	: Node("YOLO")
// 	{
// 		// 声明回调组,实例化回调组，类型为：可重入的
// 		rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_= this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
// 		// Each of these callback groups is basically a thread
//     	// Everything assigned to one of them gets bundled into the same thread
// 		auto sub_opt = rclcpp::SubscriptionOptions();
//     	sub_opt.callback_group = callback_group_subscriber_;
// 		//打印thread_id
// 		//std::cout<<std::to_string(std::hash<std::thread::id>()(std::this_thread::get_id())).c_str()<<std::endl;
// 		yolo_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
// 		"yolo_result", 50, std::bind(&YOLO::yolo_timer_callback, this, std::placeholders::_1),sub_opt);


// 		// publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);

//         // timer_ = this->create_wall_timer(
//         //     std::chrono::milliseconds(100), 
//         //     std::bind(&YOLO::timer_callback, this)
//         // );
// 		// cap.open(0, CAP_V4L2);
//         // cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
//         // cap.set(CAP_PROP_FRAME_WIDTH, 640);//图像的宽
//         // cap.set(CAP_PROP_FRAME_HEIGHT, 480);//图像的高
// 	}
// 	int get_thread_id(){
// 		return std::hash<std::thread::id>()(std::this_thread::get_id());
// 	}
// 	float get_x(){
// 		return x;
// 	}
// 	float get_y(){
// 		return y;
// 	}
// 	float get_width(){
// 		return width;
// 	}
// 	float get_height(){
// 		return height;
// 	}
// private:
// 	// rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
// 	// rclcpp::TimerBase::SharedPtr timer_;
//     // VideoCapture cap;
// 	// void timer_callback()
//     // {
//     //     Mat frame;
//     //     cap >> frame;
//     //     if (frame.rows > 0 && frame.cols > 0)
//     //     {
//     //         auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
//     //         publisher_->publish(*img_msg);
//     //     }
//     //     RCLCPP_INFO(this->get_logger(), "Publishing video frame");
//     // }

// 	rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr yolo_sub_;
// 	/*
// 	ros2 topic pub /yolo_result vision_msgs/msg/Detection2DArray '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "string"}, detections: [{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "string"}, results: [{id: 0, score: 0.0, x: 0.0, y: 0.0, width: 0.0, height: 0.0, roi: {x_offset: 0, y_offset: 0, height: 0, width: 0}, mask_array: [0, 0, 0], mask: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: "string"}, image: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: "string"}, height: 0, width: 0, encoding: "string", is_bigendian: 0, step: 0, data: [0, 0, 0]}}]}]}'
// 	*/
// 	float x;
// 	float y;
// 	float width;
// 	float height;
// 	void yolo_timer_callback(const vision_msgs::msg::Detection2DArray msg){
// 		std::cout<<"-----------------------YOLO——START----------------------"<<std::endl;
// 		for (const auto& detection : msg.detections) {
// 			auto bbox = detection.bbox;
// 			x = bbox.center.position.x;
// 			y = bbox.center.position.y;
// 			width = bbox.size_x;
// 			height = bbox.size_y;
// 			// 这里的x和y是边界框中心的坐标，width和height是边界框的大小
// 			// 你可以在这里添加你的代码来处理这些坐标
// 			RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, width: %f, height: %f", x, y, width, height);
// 		}
// 		std::cout<<"------------------------YOLO——END-----------------------"<<std::endl;
// 	}
// };
#define SET_CAP_FRAME_WIDTH 640
#define SET_CAP_FRAME_HEIGHT 480

#include "ros2_interfaces/msg/coord.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
using namespace Eigen;
using namespace std;
using namespace cv;
using namespace cv::dnn;
class YOLO : public rclcpp::Node
{
public:
    YOLO() : Node("image_pub")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&YOLO::timer_callback, this)
        );

        subscriber_ = this->create_subscription<ros2_interfaces::msg::Coord>(
            "coord", 
            10, 
            std::bind(&YOLO::coord_callback, this, std::placeholders::_1)
        );
        cap.open(0, CAP_V4L2);
        cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
        cap.set(CAP_PROP_FRAME_WIDTH, SET_CAP_FRAME_HEIGHT);//图像的宽
        cap.set(CAP_PROP_FRAME_HEIGHT, SET_CAP_FRAME_WIDTH);//图像的高
    }
	float get_x(){
		return x;
	}
	float get_y(){
		return y;
	}
	float get_width(){
		return 0;
	}
	float get_height(){
		return 0;
	}
private:
	float x;
	float y;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Subscription<ros2_interfaces::msg::Coord>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    VideoCapture cap;

    void timer_callback()
    {
        Mat frame;
        cap >> frame;
        if (frame.rows > 0 && frame.cols > 0)
        {
            auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            publisher_->publish(*img_msg);
        }
        //RCLCPP_INFO(this->get_logger(), "Publishing video frame");
    }

    void coord_callback(const ros2_interfaces::msg::Coord::SharedPtr msg)
    {
         x = msg->x;
         y = msg->y;
        int flag = msg->flag_servo;
		(void)flag;
        //RCLCPP_INFO(this->get_logger(), "收到坐标(%f, %f), flag_servo = %d", x, y, flag);
    }
    
};

#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/msg/command_code.hpp>
class ServoController : public rclcpp::Node {
public:
    ServoController() : Node("servo_controller") {
        client_ = this->create_client<mavros_msgs::srv::CommandLong>("mavros/cmd/command");
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
    }

    void set_servo(int servo_number, float position) {
        auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
        request->command = mavros_msgs::msg::CommandCode::DO_SET_SERVO;
        request->param1 = servo_number;
        request->param2 = position;

        auto result_future = client_->async_send_request(request);

        // Wait for the result
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Servo set successfully");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service set_servo");
        }
    }

private:
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr client_;
};


class OffboardControl : public rclcpp::Node {
public:
	OffboardControl(const std::string ardupilot_namespace,std::shared_ptr<YOLO> yolo_,std::shared_ptr<ServoController> servo_controller_) :
		Node("offboard_control_srv"),
		state_{State::init},
		fly_state_{FlyState::init},
		service_result_{0},
		service_done_{false},
		
		//global_gps_publisher_{this->create_publisher<ardupilot_msgs::msg::GlobalPosition>(ardupilot_namespace+"cmd_gps_pose", 5)},
		twist_stamped_publisher_{this->create_publisher<geometry_msgs::msg::TwistStamped>(ardupilot_namespace+"setpoint_velocity/cmd_vel", 5)},
		//  * /mavros/setpoint_position/global [geographic_msgs/msg/GeoPoseStamped] 1 subscriber
		global_gps_publisher_{this->create_publisher<geographic_msgs::msg::GeoPoseStamped>(ardupilot_namespace+"setpoint_position/global", 5)},
		// * /mavros/setpoint_position/local [geometry_msgs/msg/PoseStamped] 1 subscriber
		local_setpoint_publisher_{this->create_publisher<geometry_msgs::msg::PoseStamped>(ardupilot_namespace+"setpoint_position/local", 5)},
		// * /mavros/setpoint_raw/local [mavros_msgs/msg/PositionTarget] 1 subscriber
		/*# A Pose with reference coordinate frame and timestamp
		std_msgs/Header header
		Pose pose*/
		setpoint_raw_local_publisher_(this->create_publisher<mavros_msgs::msg::PositionTarget>(ardupilot_namespace+"setpoint_raw/local", 5)),
		//  * /mavros/setpoint_raw/global [mavros_msgs/msg/GlobalPositionTarget] 1 subscriber
		setpoint_raw_global_publisher_(this->create_publisher<mavros_msgs::msg::GlobalPositionTarget>(ardupilot_namespace+"setpoint_raw/global", 5)),
		// /mavros/setpoint_trajectory/local [trajectory_msgs/msg/MultiDOFJointTrajectory] 1 subscriber
		trajectory_publisher_{this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(ardupilot_namespace+"setpoint_trajectory/local", 5)},
		
		// // * /mavros/home_position/set [mavros_msgs/msg/HomePosition] 1 subscriber
		// // ros2 topic pub /mavros/home_position/set mavros_msgs/msg/HomePosition '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, geo: {latitude: 0.0, longitude: 0.0, altitude: 0.0}, position: {x: 1.0, y: 1.0, z: 1.0}}'
		// home_position_publisher_{this->create_publisher<mavros_msgs::msg::HomePosition>(ardupilot_namespace+"home_position/set", 5)},
		
		arm_motors_client_{this->create_client<mavros_msgs::srv::CommandBool>(ardupilot_namespace+"cmd/arming")},
		mode_switch_client_{this->create_client<mavros_msgs::srv::SetMode>(ardupilot_namespace+"set_mode")},
		set_home_client_ {this->create_client<mavros_msgs::srv::CommandHome>(ardupilot_namespace+"cmd/set_home")}
		
	{
		ardupilot_namespace_copy_ = ardupilot_namespace;
		this->yolo_ = yolo_;
		this->servo_controller_ = servo_controller_;
		// 质量服务配置
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		// 声明回调组,实例化回调组，类型为：可重入的
		rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_= this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
		// Each of these callback groups is basically a thread
    	// Everything assigned to one of them gets bundled into the same thread
		auto sub_opt = rclcpp::SubscriptionOptions();
    	sub_opt.callback_group = callback_group_subscriber_;
		//ros2 topic echo /mavros/local_position/pose geometry_msgs/msg/PoseStamped
		pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(ardupilot_namespace_copy_+"local_position/pose", qos,
		std::bind(&OffboardControl::pose_callback, this, std::placeholders::_1),sub_opt);
		//ros2 topic echo /mavros/global_position/global sensor_msgs/msg/NavSatFix
		gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(ardupilot_namespace_copy_+"global_position/global", qos,
		std::bind(&OffboardControl::gps_callback, this, std::placeholders::_1),sub_opt);
		velocity_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(ardupilot_namespace_copy_+"local_position/velocity_local", qos,
		std::bind(&OffboardControl::velocity_callback, this, std::placeholders::_1),sub_opt);
		altitude_subscription_ = this->create_subscription<mavros_msgs::msg::Altitude>(ardupilot_namespace_copy_+"altitude", qos,
        std::bind(&OffboardControl::altitude_callback, this, std::placeholders::_1),sub_opt);
		//订阅IMU信息
		// * /mavros/imu/data [sensor_msgs/msg/Imu] 1 publisher
		//ros2 topic echo /mavros/imu/data
		
		//查询系统状态
		// * /mavros/state [mavros_msgs/msg/State] 1 publisher
		//ros2 topic echo /mavros/state
		state_subscription_ = this->create_subscription<mavros_msgs::msg::State>(ardupilot_namespace_copy_+"state", qos,
		 std::bind(&OffboardControl::state_callback, this, std::placeholders::_1));
		
		// * /mavros/home_position/home [mavros_msgs/msg/HomePosition] 1 subscriber
		// ros2 topic pub /mavros/home_position/home mavros_msgs/msg/HomePosition '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, geo: {latitude: 0.0, longitude: 0.0, altitude: 0.0}, position: {x: 0.0, y: 0.0, z: 0.0}}'
		home_position_subscription_ = this->create_subscription<mavros_msgs::msg::HomePosition>(ardupilot_namespace_copy_+"home_position/home", qos,
		std::bind(&OffboardControl::home_position_callback, this, std::placeholders::_1),sub_opt);

		//yolo
		//yolo_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
		//"yolo_result", 10, std::bind(&OffboardControl::yolo_timer_callback, this, std::placeholders::_1),sub_opt);

		RCLCPP_INFO(this->get_logger(), "Starting Offboard Control example with PX4 services");
		//RCLCPP_INFO_STREAM(geometry_msgs::msg::PoseStampedthis->get_logger(), "Waiting for " << px4_namespace << "vehicle_command service");
		while (!mode_switch_client_->wait_for_service(std::chrono::seconds(2))) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
				return;
			}
			RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  		}
		// std::thread t1(DaemonThread::run);
		// t1.detach();
		timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));
	}

    void switch_mode(std::string mode);
	void arm_motors(bool arm);
	void set_home_position(double lat, double lon, double alt);
	double quaternion_to_yaw(double x, double y, double z, double w);
	void yaw_to_quaternion(double yaw, double* x, double* y, double* z, double* w);
private:
	std::shared_ptr<YOLO> yolo_;
	std::shared_ptr<ServoController> servo_controller_;
	class LocalFrame{
	public:
		double x;
		double y;
		double z;
		double yaw;
	};
	class GlobalFrame{
	public:
		double lat;
		double lon;
		double alt;
		double yaw;
	};

	// '''定义一个全局变量'''
	double _rngfnd_distance;
	double timestamp0;
	double heading=0;
	const double default_heading=DEFAULT_HEADING;//初始偏转角
	class Location{
	public:
		LocalFrame local_frame;
		GlobalFrame global_frame;
	};
	Eigen::Quaterniond quaternion;// 四元数
	Eigen::Vector3d euler;// 欧拉角
	Location location;
	geometry_msgs::msg::PoseStamped pose_{};
	geometry_msgs::msg::TwistStamped velocity_{};
	sensor_msgs::msg::NavSatFix global_gps_{};
	mavros_msgs::msg::State drone_state_{};

	LocalFrame start{0,0,0,0};
	LocalFrame start_temp{0,0,0,0};
	LocalFrame end_temp={0,0,0,0};
	//LocalFrame end={0,0,0,0};

	LocalFrame home_position{0,0,0,0};

	GlobalFrame start_global{0,0,0,0};
	GlobalFrame start_global_temp{0,0,0,0};
	GlobalFrame end_global_temp={0,0,0,0};
	//LocalFrame end_global={0,0,0,0};

		


	double PrintYaw(void){
		quaternion.w() = pose_.pose.orientation.w;
		quaternion.x() = pose_.pose.orientation.x;
		quaternion.y() = pose_.pose.orientation.y;
		quaternion.z() = pose_.pose.orientation.z;
		euler = quaternion_to_euler(quaternion);
		heading = euler(2); //弧度制
		// std::cout << "heading: " << heading/PI*180 << std::endl;
		return heading/PI*180; //角度制
	}
	
	enum class State{
		init,
		send_geo_grigin,
		wait_for_stable_offboard_mode,
		arm_requested,
		takeoff,
		autotune_mode,
		
	} state_;
	enum class FlyState{
		init,
		//request,
		takeoff,
		goto_shot_area,
		findtarget,
		goto_scout_area,
		scout,
		land,
		end
	} fly_state_;
	std::string ardupilot_namespace_copy_;
	uint8_t service_result_;
	bool service_done_;
	bool arm_done_;

	// double _D_max = 2; //位置环最大速度
	// double _D = 1; //位置环比例系数

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

	GlobalFrame shot_area_start{};
	GlobalFrame scout_area_start{};



	rclcpp::TimerBase::SharedPtr timer_;
	
	//sensor_msgs::msg::NavSatFix global_gps_start{};
	//ardupilot_msgs::msg::GlobalPosition global_gps_start{};

	//rclcpp::Publisher<ardupilot_msgs::msg::GlobalPosition>::SharedPtr global_gps_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_publisher_;
	rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr global_gps_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_setpoint_publisher_;
	rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_raw_local_publisher_;
	rclcpp::Publisher<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr setpoint_raw_global_publisher_;
	rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr trajectory_publisher_;
	// rclcpp::Publisher<mavros_msgs::msg::HomePosition>::SharedPtr home_position_publisher_;
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
	rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscription_;
	rclcpp::Subscription<mavros_msgs::msg::Altitude>::SharedPtr altitude_subscription_;
	rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscription_;
	rclcpp::Subscription<mavros_msgs::msg::HomePosition>::SharedPtr home_position_subscription_;
 	rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_motors_client_;
	rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_switch_client_;
	rclcpp::Client<mavros_msgs::srv::CommandHome>::SharedPtr set_home_client_;
	
	void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
	void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
	void altitude_callback(const mavros_msgs::msg::Altitude::SharedPtr msg);
	void state_callback(const mavros_msgs::msg::State::SharedPtr msg);
	void home_position_callback(const mavros_msgs::msg::HomePosition::SharedPtr msg);

	void switch_to_guided_mode();
	void switch_to_flip_mode();
	void switch_to_rtl_mode();
	// void switch_to_auto_mode();
	//void publish_global_gps(double latitude, double longitude, double altitude);
	
	void command_takeoff_or_land_local(std::string mode);
	void command_takeoff_or_land(std::string mode);

	void trajectory_setpoint_global(double x,double y ,double z , double lat, double lon, double alt,double yaw);
	void trajectory_setpoint_takeoff(double x,double y ,double z ,double yaw);
	void trajectory_setpoint(double x,double y ,double z ,double yaw, double accuracy=DEFAULT_ACCURACY);
	void trajectory_setpoint_start(double x,double y ,double z ,double yaw,double accuracy=DEFAULT_ACCURACY);
	bool alt_hold(double vx,double vy ,double z ,double yaw,double time,double accuracy=DEFAULT_ACCURACY);
	void publish_trajectory_setpoint(double n_x,double n_y ,double n_z ,double n_yaw,double t_x,double t_y ,double t_z ,double t_yaw, double kp_xy,double ki_xy,double kd_xy);
	void publish_trajectory_setpoint(double x,double y ,double z ,double yaw);
	bool publish_trajectory_setpoint_z(double *x,double accuracy=DEFAULT_ACCURACY);
	bool publish_trajectory_setpoint_yaw(double *yaw,double accuracy=DEFAULT_ACCURACY_YAW);
	bool send_velocity_command_with_time(double linear_x, double linear_y, double linear_z, double angular_z, double time);
	void send_velocity_command(double linear_x, double linear_y, double linear_z, double angular_z);
	void publish_trajectory(double x, double y, double z, double yaw);
	
	void send_gps_setpoint_command(double latitude, double longitude, double altitude);
	void send_local_setpoint_command(double x, double y, double z, double yaw);
	void publish_setpoint_raw_local(double x, double y, double z, double yaw);
	void publish_setpoint_raw_global(double latitude, double longitude, double altitude, double yaw);
	// void set_home_position(double x, double y, double z);
	void PidRTL(double x,double y,double frtl);
	
	bool set_time(double time);

	bool surrounding_shot_area(void);
	bool surrounding_scout_area(void);
	bool catch_target_bucket(bool &result);

	void set_target_point(std::string mode,double x=0,double y=0,double z=0,double yaw=0);
	void set_target_point_global(std::string mode,double lat=0,double lon=0,double alt=0,double yaw=0);
	// void set_start_temp_point(double x,double y,double z,double yaw);
	// void set_end_temp_point(double x,double y,double z,double yaw);
	// void set_drone_target_point_local(double x,double y,double z,double yaw);
	// void set_start_point_local(double x,double y,double z,double yaw);
	// void set_world_point_local(double x,double y,double z,double yaw);
	bool at_check_point(double accuracy=DEFAULT_ACCURACY);	
	bool at_check_point(double now_x, double now_y,double target_x, double target_y, double accuracy=DEFAULT_ACCURACY);


	void get_target_location(double* x, double* y,double delta_heading=0);
	inline void get_target_location_global(double &x,double &y,double &z,double lat,double lon,double alt,double delta_heading=0);

	void timer_callback(void);

	//rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr yolo_sub_;
	/*
	ros2 topic pub /yolo_result vision_msgs/msg/Detection2DArray '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "string"}, detections: [{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "string"}, results: [{id: 0, score: 0.0, x: 0.0, y: 0.0, width: 0.0, height: 0.0, roi: {x_offset: 0, y_offset: 0, height: 0, width: 0}, mask_array: [0, 0, 0], mask: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: "string"}, image: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: "string"}, height: 0, width: 0, encoding: "string", is_bigendian: 0, step: 0, data: [0, 0, 0]}}]}]}'
	*/
/*
	void yolo_timer_callback(const vision_msgs::msg::Detection2DArray msg){
		std::cout<<"YOLO"<<std::endl;
		for (const auto& detection : msg.detections) {
			auto bbox = detection.bbox;
			float x = bbox.center.position.x;
			float y = bbox.center.position.y;
			float width = bbox.size_x;
			float height = bbox.size_y;
			// 这里的x和y是边界框中心的坐标，width和height是边界框的大小
			// 你可以在这里添加你的代码来处理这些坐标
			RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, width: %f, height: %f", x, y, width, height);
		}
	}*/

	// bool PID_P_Position_Control(LocalFrame end,double vp = kp,double vi = ki,double vd = kd,double dp = p);
};

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

// 设置无人机家的位置
// /mavros/cmd/set_home [mavros_msgs/srv/CommandHome]
void OffboardControl::set_home_position(double lat, double lon, double alt)
{
	auto request = std::make_shared<mavros_msgs::srv::CommandHome::Request>();
	request->current_gps = false;
	request->latitude = lat;
	request->longitude = lon;
	request->altitude = alt;

	while (!set_home_client_->wait_for_service(std::chrono::seconds(1))) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
			return;
		}
		RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
	}
	RCLCPP_INFO(this->get_logger(), "set home command send");
	auto result_future = set_home_client_->async_send_request(request,
		[this](rclcpp::Client<mavros_msgs::srv::CommandHome>::SharedFuture future) {
			auto status = future.wait_for(0.5s);
			if (status == std::future_status::ready) {
				auto reply = future.get()->success;
				RCLCPP_INFO(this->get_logger(), "Set Home Position: %s", reply ? "success" : "failed");
				if (reply) {
					// Code to execute if the future is successful
				}
				else {
					// Code to execute if the future is unsuccessful
					RCLCPP_ERROR(this->get_logger(), ("Failed to call service " + ardupilot_namespace_copy_ + "cmd/set_home").c_str());
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
// 接收home位置数据
void OffboardControl::home_position_callback(const mavros_msgs::msg::HomePosition::SharedPtr msg) 
{
	home_position.x = msg->position.x;
	home_position.y = msg->position.y;
	home_position.z = msg->position.z;
	// RCLCPP_INFO(this->get_logger(), "Received home position data");
	// RCLCPP_INFO(this->get_logger(), "Latitude: %f", msg->geo.latitude);
	// RCLCPP_INFO(this->get_logger(), "Longitude: %f", msg->geo.longitude);
	// RCLCPP_INFO(this->get_logger(), "Altitude: %f", msg->geo.altitude);
	// RCLCPP_INFO(this->get_logger(), "X: %f", msg->position.x);
	// RCLCPP_INFO(this->get_logger(), "Y: %f", msg->position.y);
	// RCLCPP_INFO(this->get_logger(), "Z: %f", msg->position.z);
}



// 抵达桶上方
// if(识别到桶=catch_target_bucket（到达正上方）){[if(到达正上方==true){...}}
bool OffboardControl::catch_target_bucket(bool &result){
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
			double tar_x = SET_CAP_FRAME_WIDTH/2 ;// /10
			double tar_y = -SET_CAP_FRAME_HEIGHT/2 ;// /3
			get_target_location(&now_x,&now_y,location.local_frame.yaw);
			get_target_location(&tar_x,&tar_y,location.local_frame.yaw);
			RCLCPP_INFO(this->get_logger(), "catch_target_bucket: now_x: %f, now_y: %f, tar_x: %f, tar_y: %f", now_x, now_y, tar_x, tar_y);
			RCLCPP_INFO(this->get_logger(), "catch_target_bucket: now_x: %f, now_y: %f, tar_x: %f, tar_y: %f", now_x/SET_CAP_FRAME_WIDTH, now_y/SET_CAP_FRAME_HEIGHT,(tar_x)/SET_CAP_FRAME_WIDTH,(tar_y)/SET_CAP_FRAME_HEIGHT);
			publish_trajectory_setpoint(
				now_x/SET_CAP_FRAME_WIDTH ,now_y/SET_CAP_FRAME_HEIGHT,location.local_frame.z,location.local_frame.yaw,
				(tar_x)/SET_CAP_FRAME_WIDTH,(tar_y)/SET_CAP_FRAME_HEIGHT,set_z,set_yaw,
				0.5,0.01,0.01
			);
			if(at_check_point(now_x,now_y,tar_x,tar_y,10)){
				RCLCPP_INFO(this->get_logger(), "Arrive, 投弹");
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
	static uint64_t time_find_start = this->get_clock()->now().nanoseconds() / 1000;
	if((this->get_clock()->now().nanoseconds() / 1000-time_find_start)>31000000){//>12s
		return true;
	}
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
		// if((this->get_clock()->now().nanoseconds() / 1000-time_find_start)>12000000){//>12s
		// 	sur_state_=SurState::end;
		// }
		break;
	case SurState::fly_to_target_y:
		trajectory_setpoint(0,fy,0,0);
		if(at_check_point()){
			sur_state_=SurState::set_point_x;
			fy=(fy>0)?(-fy-1):(-fy+1);
		}
		// if((this->get_clock()->now().nanoseconds() / 1000-time_find_start)>12000000){//>12s
		// 	sur_state_=SurState::end;
		// }
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
	static uint64_t time_find_start = 0;
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

// 发布全局GPS位置
// 输入相对移动距离
// x:m y:m z:m yaw:°
// lat lon alt:开始位置
void OffboardControl::trajectory_setpoint_global(double x,double y ,double z,double lat,double lon,double alt, double yaw){
	if(global_gps_.header.stamp.sec == 0){
		RCLCPP_INFO(this->get_logger(), "No pose data received yet");
		return;
	}
	double z1 = z;
	get_target_location_global(x,y,z,lat,lon,alt,yaw);
	RCLCPP_INFO(this->get_logger(),"trajectory_setpoint_global: et:%f %f %f", x, y, z1);
	publish_setpoint_raw_global(x, y, z1, yaw);
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

// //
// while True:
//     x=get_value(2)
//     y=get_value(3)
//     if x!=0 and y!=0: 
//         dx=x
//         dy=y
//         PidRTL(dx,dy,frtl,vehicle)
//         frtl=get_value(1)
//     elif time.time()-time0>290:
//         vehicle.mode = VehicleMode("LAND")
//         time.sleep(10)
//         vehicle.armed = False
//         break
// print((time.time()-time0))
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

// PID控制
// n_x:当前位置x(m) n_y:当前位置y(m) n_z:当前位置z(m) n_yaw:当前偏航角(°) t_x:目标位置x(m) t_y:目标位置y(m) t_z:目标位置z(m) t_yaw:目标偏航角(°) kp:比例系数 ki:积分系数 kd:微分系数
// 返回值：是否到达目标位置
void OffboardControl::publish_trajectory_setpoint(double n_x,double n_y ,double n_z ,double n_yaw,double t_x,double t_y ,double t_z ,double t_yaw ,double kp_xy ,double ki_xy ,double kd_xy){
    //RCLCPP_INFO(this->get_logger(), "Publishing setpoint: x=%f, y=%f, z=%f, yaw=%f", x, y, z, yaw);
	static double previous_error_x = t_x - n_x;
    static double previous_error_y = t_y - n_y;
    static double previous_error_z = t_z - n_z;
    static double previous_error_yaw = t_yaw - n_yaw;

    static double integral_x = 0;
    static double integral_y = 0;
    static double integral_z = 0;
    static double integral_yaw = 0;

	double error_x = t_x - n_x;
	double error_y = t_y - n_y;
	double error_z = t_z - n_z;
	double error_yaw = t_yaw - n_yaw;
 
	const static int n = 10;
	const static double integral_limit = 0.1;
	static double integral_[n][4] = {0}; // 0:x, 1:y, 2:z, 3:yaw
	static int i = 0;
	// 移除旧的积分值
	integral_x -= integral_[i][0];
	integral_y -= integral_[i][1];
	integral_z -= integral_[i][2];
	integral_yaw -= integral_[i][3];
	// 添加新的积分值
	integral_[i][0] = error_x * dt;
	integral_[i][1] = error_y * dt;
	integral_[i][2] = error_z * dt;
	integral_[i][3] = error_yaw * dt;
	// 更新积分，并引入积分限幅
	integral_x = std::min(std::max(integral_x + error_x * dt, -integral_limit), integral_limit);
	integral_y = std::min(std::max(integral_y + error_y * dt, -integral_limit), integral_limit);
	integral_z = std::min(std::max(integral_z + error_z * dt, -integral_limit), integral_limit);
	integral_yaw = std::min(std::max(integral_yaw + error_yaw * dt, -integral_limit), integral_limit);
	i = (i+1)%n;


	double velocity_x = (previous_error_x - error_x) / dt;
	double velocity_y = (previous_error_y - error_y) / dt;
	double velocity_z = (previous_error_z - error_z) / dt;
	double velocity_yaw = (previous_error_yaw - error_yaw) / dt;

	//double derivative_x = (error_x - previous_error_x) / dt;
	//double derivative_y = (error_y - previous_error_y) / dt;
	//double derivative_z = (error_z - previous_error_z) / dt;
	//double derivative_yaw = (error_yaw - previous_error_yaw) / dt;	

	double output_x = kp_xy * error_x + ki_xy * integral_x + kd_xy * velocity_x;
	double output_y = kp_xy * error_y + ki_xy * integral_y + kd_xy * velocity_y;
	double output_z = kp * error_z + ki * integral_z + kd * velocity_z;
	double output_yaw = kp_yaw * error_yaw + ki_yaw * integral_yaw + kd_yaw * velocity_yaw;

	previous_error_x = error_x;
	previous_error_y = error_y;
	previous_error_z = error_z;
	previous_error_yaw = error_yaw;

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
	if (output_yaw > 180) output_yaw = -360+output_yaw;
	if (output_yaw < -180) output_yaw = 360+output_yaw;
	if (output_yaw > max_yaw) output_yaw = max_yaw;
	if (output_yaw < -max_yaw) output_yaw = -max_yaw;
	RCLCPP_INFO(this->get_logger(), "publish_trajectory_setpoint(自定义): error: x=%f, y=%f, z=%f, yaw=%f", error_x, error_y, error_z, error_yaw);
	RCLCPP_INFO(this->get_logger(), "publish_trajectory_setpoint(自定义): integral: x=%f, y=%f, z=%f, yaw=%f", integral_x, integral_y, integral_z, integral_yaw);
	RCLCPP_INFO(this->get_logger(), "publish_trajectory_setpoint(自定义): velocity: x=%f, y=%f, z=%f, yaw=%f", velocity_x, velocity_y, velocity_z, velocity_yaw);

	RCLCPP_INFO(this->get_logger(), "publish_trajectory_setpoint(自定义): output: x=%f, y=%f, z=%f, yaw=%f", output_x, output_y, output_z, output_yaw);
    send_velocity_command(output_x, output_y, output_z, output_yaw);
	if(at_check_point()){
 		//RCLCPP_INFO(this->get_logger(), "at_check_point");
		previous_error_x = 0;
		previous_error_y = 0;
		previous_error_z = 0;
		previous_error_yaw = 0;
		integral_x = 0;
		integral_y = 0;
		integral_z = 0;
		integral_yaw = 0;
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
	
	const static int n = 20;
	static double integral_[n][4] = {0}; // 0:x, 1:y, 2:z, 3:yaw
	static int i = 0;
	integral_x -= integral_[i][0];
	integral_y -= integral_[i][1];
	integral_z -= integral_[i][2];
	integral_yaw -= integral_[i][3];
	integral_[i][0] = error_x * dt;
	integral_[i][1] = error_y * dt;
	integral_[i][2] = error_z * dt;
	integral_[i][3] = error_yaw * dt;
	integral_x += error_x * dt;
	integral_y += error_y * dt;
	integral_z += error_z * dt;
    integral_yaw += error_yaw * dt;
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
// 旋转角度PID控制_本地坐标系
// yaw:目标角度（度） accuracy:精度（度）默认为
// 返回值：是否到达目标角度
bool OffboardControl::publish_trajectory_setpoint_yaw(double *yaw,double accuracy){
    static double integral_yaw = 0;

	double error_yaw = *yaw - quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);
	
	const static int n = 20;
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
// send_local_setpoint_command(x, y, z, yaw); 飞行到
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
	RCLCPP_INFO(this->get_logger(), "Publishing setpoint: latitude=%f, longitude=%f, altitude=%f, yaw=%f", latitude, longitude, altitude, yaw);
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


// // 发布家位置控制指令(无效)
// // * /mavros/home_position/set [mavros_msgs/msg/HomePosition] 1 subscriber
// // ros2 topic pub /mavros/home_position/set mavros_msgs/msg/HomePosition '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, geo: {latitude: 0.0, longitude: 0.0, altitude: 0.0}, position: {x: 1.0, y: 1.0, z: 1.0}}'
// void OffboardControl::set_home_position(double x,double y,double z)
// {
// 	mavros_msgs::msg::HomePosition msg;
// 	msg.position.x = x;
// 	msg.position.y = y;
// 	msg.position.z = z;
// 	msg.header.stamp = this->now();
// 	msg.header.frame_id = "base_link";
// 	home_position_publisher_->publish(msg);
// }



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

//设置目标点_global
//mode: base_link, start, world, start_temp, end_temp
//lat,lon,alt: 绝对坐标系的目标点，默认不改变
//yaw: 绝对坐标系的目标点，默认不改变
//set_target_point_global("base_link",0,0,5,0);//set_target_point_global("base_link");
//set_target_point_global("start",0,0,5,0);//set_target_point_global("start");
//set_target_point_global("world",0,0,5,0);//set_target_point_global("world");
//set_target_point_global("start_temp",0,0,5,0);//set_target_point_global("start_temp");
//set_target_point_global("end_temp",0,0,5,0);//set_target_point_global("end_temp");
// void OffboardControl::set_target_point_global(std::string mode,double lat,double lon,double alt,double yaw){
// 	#ifdef PRE_TEST
// 	static double lat_pre;
// 	static double lon_pre;
// 	static double alt_pre;
// 	//static double yaw_pre;
// 	if(lat!=lat_pre || lon!=lon_pre || alt!=alt_pre){
// 		lat_pre=lat;
// 		lon_pre=lon;
// 		alt_pre=alt;
// 		//yaw_pre=yaw;
// 		//mode_pre=mode;
// 	#endif
// 		if(mode=="base_link"){
// 			start_global_temp=end_global_temp;
// 			end_global_temp.lat=start_global_temp.lat+lat;
// 			end_global_temp.lon=start_global_temp.lon+lon;
// 			end_global_temp.alt=start_global_temp.alt+alt;
// 			end_global_temp.yaw=start_global_temp.yaw+yaw;
// 			RCLCPP_INFO(this->get_logger(),"base_link:%f %f %f %f",end_global_temp.lat, end_global_temp.lon, end_global_temp.alt, end_global_temp.yaw);
// 		}else if(mode=="start"){
// 			start_global_temp=end_global_temp;
// 			end_global_temp.lat=start_global.lat+lat;
// 			end_global_temp.lon=start_global.lon+lon;
// 			end_global_temp.alt=start_global.alt+alt;
// 			end_global_temp.yaw=start_global.yaw+yaw;
// 			RCLCPP_INFO(this->get_logger(),"start:%f %f %f %f",end_global_temp.lat, end_global_temp.lon, end_global_temp.alt,end_global_temp.yaw);
// 		}else if(mode=="world"){
// 			start_global_temp=end_global_temp;
// 			end_global_temp.lat=lat;
// 			end_global_temp.lon=lon;
// 			end_global_temp.alt=alt;
// 			end_global_temp.yaw=yaw;
// 			RCLCPP_INFO(this->get_logger(),"world:%f %f %f %f",end_global_temp.lat, end_global_temp.lon, end_global_temp.alt, end_global_temp.yaw);
// 		}else if(mode=="start_global_temp"){
// 			if(lat==0.0){start_global_temp.lat=location.global_frame.lat;}else{start_global_temp.lat=lat;}
// 			if(lon==0.0){start_global_temp.lon=location.global_frame.lon;}else{start_global_temp.lon=lon;}
// 			if(alt==0.0){start_global_temp.alt=location.global_frame.alt;}else{start_global_temp.alt=alt;}
// 			if(yaw==0.0){start_global_temp.yaw=location.global_frame.yaw;}else{start_global_temp.yaw=yaw;}
// 			RCLCPP_INFO(this->get_logger(),"start_global_temp:%f %f %f %f",end_global_temp.lat, end_global_temp.lon, end_global_temp.alt, end_global_temp.yaw);
// 		}else if(mode=="end_global_temp"){
// 			if(lat==0.0){end_global_temp.lat=location.global_frame.lat;}else{end_global_temp.lat=lat;}
// 			if(lon==0.0){end_global_temp.lon=location.global_frame.lon;}else{end_global_temp.lon=lon;}
// 			if(alt==0.0){end_global_temp.alt=location.global_frame.alt;}else{end_global_temp.alt=alt;}
// 			if(yaw==0.0){end_global_temp.yaw=location.global_frame.yaw;}else{end_global_temp.yaw=yaw;}

// 			RCLCPP_INFO(this->get_logger(),"end_global_temp:%f %f %f %f",end_global_temp.lat, end_global_temp.lon, end_global_temp.alt, end_global_temp.yaw);
// 		}else{
// 			RCLCPP_ERROR(this->get_logger(), "No such mode");
// 		}
// 	#ifdef PRE_TEST
// 	}else{
// 		RCLCPP_INFO(this->get_logger(),"set_target_point_global:%f %f %f %f",end_global_temp.lat, end_global_temp.lon, end_global_temp.alt, end_global_temp.yaw); 
// 		return;
// 	}
// 	#endif
// }

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
//确认到达目标点
//now_x, now_y, 当前位置, target_x, target_y: 目标点位置, accuracy: 精度（默认为DEFAULT_ACCURACY）
//返回值：到达目标点返回true，否则返回false
bool OffboardControl::at_check_point(double now_x, double now_y,double target_x, double target_y, double accuracy){
	////////RCLCPP_INFO(this->get_logger(),"et:%f %f %f",end_temp.x, end_temp.y, end_temp.z); 
	////////RCLCPP_INFO(this->get_logger(),"vl:%f %f %f",pose_.pose.position.x-end_temp.x, pose_.pose.position.y - end_temp.y, pose_.pose.position.z - end_temp.z); 
	RCLCPP_INFO(this->get_logger()," now: %f %f", now_x, now_y);
	RCLCPP_INFO(this->get_logger()," tar: %f %f", target_x, target_y);
	if(
		abs(now_x - target_x) <=accuracy && 
		abs(now_y - target_y) <=accuracy
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

double OffboardControl::quaternion_to_yaw(double x, double y, double z, double w) {
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return yaw/PI*180;
}
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
// #将相对坐标系下的坐标 x:m,y:m,z:m,lat，lon，alt:出发位置heading:degree
// 转换为全局坐标系下的经纬度坐标
//get_target_location(default_heading, );
//ros2 topic pub /mavros/setpoint_raw/global mavros_msgs/msg/GlobalPositionTarget '{header: "auto", latitude: -35.363262, longitude: 149.165237, altitude: 700.788637}'
void OffboardControl::get_target_location_global(double &x,double &y,double &z,double lat,double lon,double alt,double delta_heading) {
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

	add_flu_offset_to_llh(lat, lon, alt,d_x, d_y, d_z,  x, y, z);;

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

void OffboardControl::timer_callback(void){
	static uint8_t num_of_steps = 0;
	static bool is_takeoff = false;

// int frtl=0;

	// RCLCPP_INFO(this->get_logger(), "timer_callback");
	// RCLCPP_INFO(this->get_logger(), "arm_done: %d", arm_done_);
	//switch_to_autotune_mode();
	// offboard_control_mode needs to be paired with trajectory_setpoint
	//publish_offboard_control_mode();
	//trajectory_setpoint_global(0, 0, 100, 0);
	//PrintYaw();
	//RCLCPP_INFO(this->get_logger(), "yaw: %lf", quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w));
	//GlobalFrame a{0,0,10,0}; get_target_location_global(a.lat, a.lon, a.alt, a.yaw);
	// std::cout << "al: " << location.global_frame.lat << " lon " << location.global_frame.lon << " alt: " << location.global_frame.alt  << std::endl;
	std::cout << "home_x: " << home_position.x << " home_y: " << home_position.y << " home_z: " << home_position.z << std::endl;
	// set_home_position(location.global_frame.lat,location.global_frame.lon,0);

	std::cout << yolo_->get_x() << " " << yolo_->get_y() << " " << std::endl;
	RCLCPP_INFO(this->get_logger(), "global_gps: %lf %lf %lf", location.global_frame.lat, location.global_frame.lon, location.global_frame.alt);
	switch (fly_state_)
	{
	case FlyState::init:
		rclcpp::sleep_for(1s);
		if(pose_.header.stamp.sec == 0 || global_gps_.header.stamp.sec == 0){
			RCLCPP_INFO(this->get_logger(), "No pose data received yet");
			break;
		}
		set_home_position(location.global_frame.lat,location.global_frame.lon,location.global_frame.alt);
		timestamp0 = this->get_clock()->now().nanoseconds() / 1000;
		RCLCPP_INFO(this->get_logger(), "timestamp0= %f ,\ntimestamp-timestamp0=%f", timestamp0, this->get_clock()->now().nanoseconds() - timestamp0);
		start = {pose_.pose.position.x,pose_.pose.position.y,pose_.pose.position.z,quaternion_to_yaw(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w)}; // 扩展卡尔曼滤波器（EKF3）已经为IMU（惯性测量单元）0和IMU1设置了起点。
		start_global = {location.global_frame.lat, location.global_frame.lon, 0, 0};//AP: Field Elevation Set: 0m  当前位置的地面高度为0米，这对于高度控制和避免地面碰撞非常重要。
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
			//if (false){
				RCLCPP_INFO(this->get_logger(), "goto_shot_area start, totaltime=%fs", (this->get_clock()->now().nanoseconds() / 1000- timestamp0)/1000000.0);
				fly_state_ = FlyState::goto_shot_area;
			}
			else{
				// 设定速度
				send_velocity_command(0, 0, 0.5, 0);

				// 飞行到指定坐标
				//publish_setpoint_raw_global(59.659859999999995, 31.7103000000003,20, 0);
				
				// ros2 topic pub /mavros/setpoint_raw/global mavros_msgs/msg/GlobalPositionTarget '{header: "auto", latitude: -359.659859999999995, longitude: 31.7103000000003, altitude7 30}'
				// 飞行到指定经纬度
				// publish_setpoint_raw_global(start_global.lat+0.0001, start_global.lon+0.0001, start_global.alt+10, 0);
				// std::cout<<"lat:"<<start_global.lat<<" lon:"<< start_global.lon<<" alt:"<< start_global.alt<<std::endl;
				
				// 本地坐标使用gps
				// static bool first = true;
				// static GlobalFrame temp_location;
				// if(first){
				// 	temp_location = {location.global_frame.lat, location.global_frame.lon, location.global_frame.alt, 0};
				// 	first = false;
				// }
				// trajectory_setpoint_global(10, 10, 10, temp_location.lat, temp_location.lon, temp_location.alt, 0);
			
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
		// while (true)
		// {
		// 	float x = yolo_->get_x();
		// 	float y = yolo_->get_y();
		// 	if(x!=0 && y!=0){
		// 		double dx = x;
		// 		double dy = y;
		// 		PidRTL(dx,dy,frtl);
		// 		//frtl=get_value(1)
		// 	}
		// 	else if(this->get_clock()->now().nanoseconds() / 1000 - timestamp0 > 290){
		// 		command_takeoff_or_land("LAND");
		// 		rclcpp::sleep_for(10s);
		// 		arm_motors(false);
		// 		break;
		// 	}
		// }
		// std::cout << (this->get_clock()->now().nanoseconds() / 1000 - timestamp0) << std::endl;
		


		if(surrounding_shot_area()){
			fly_state_ = FlyState::goto_scout_area;
			RCLCPP_INFO(this->get_logger(), "findtarget done,goto_scout_area start totaltime=%fs", (this->get_clock()->now().nanoseconds() / 1000- timestamp0)/1000000.0);
		}
		break;
	case FlyState::goto_scout_area:
		trajectory_setpoint_start(58, 0, 5, 0);

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
				rclcpp::sleep_for(1s);
				
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


    //node->set_servo(12, 1050);出来
    //node->set_servo(12, 1800);缩回去
    // Example usage: set servo number 12 to position 1500


int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

	rclcpp::executors::MultiThreadedExecutor executor;

	auto yolo_ = std::make_shared<YOLO>();
	auto servocountroller_ = std::make_shared<ServoController>();
	auto node = std::make_shared<OffboardControl>("/mavros/",yolo_,servocountroller_);
	
	// auto pubnode = std::make_shared<PublisherNode>();
	/* 运行节点，并检测退出信号*/
	executor.add_node(yolo_);

	executor.add_node(node);
	executor.add_node(servocountroller_);

	executor.spin();

	rclcpp::shutdown();
	return 0;
}

/*
Subscribed topics:
### * /mavros/adsb/send [mavros_msgs/msg/ADSBVehicle] 1 subscriber
 这个类型可能包含了一些关于航空器的信息，如位置、速度等。

 # The location and information of an ADSB vehicle
#
# https://mavlink.io/en/messages/common.html#ADSB_VEHICLE

# [[[cog:
# import mavros_cog
# mavros_cog.idl_decl_enum('ADSB_ALTITUDE_TYPE', 'ALT_')
# mavros_cog.idl_decl_enum('ADSB_EMITTER_TYPE', 'EMITTER_')
# mavros_cog.idl_decl_enum('ADSB_FLAGS', 'FLAG_', 16)
# ]]]
# ADSB_ALTITUDE_TYPE
uint8 ALT_PRESSURE_QNH = 0               # Altitude reported from a Baro source using QNH reference
uint8 ALT_GEOMETRIC = 1                  # Altitude reported from a GNSS source
# ADSB_EMITTER_TYPE
uint8 EMITTER_NO_INFO = 0
uint8 EMITTER_LIGHT = 1
uint8 EMITTER_SMALL = 2
uint8 EMITTER_LARGE = 3
uint8 EMITTER_HIGH_VORTEX_LARGE = 4
uint8 EMITTER_HEAVY = 5
uint8 EMITTER_HIGHLY_MANUV = 6
uint8 EMITTER_ROTOCRAFT = 7
uint8 EMITTER_UNASSIGNED = 8
uint8 EMITTER_GLIDER = 9
uint8 EMITTER_LIGHTER_AIR = 10
uint8 EMITTER_PARACHUTE = 11
uint8 EMITTER_ULTRA_LIGHT = 12
uint8 EMITTER_UNASSIGNED2 = 13
uint8 EMITTER_UAV = 14
uint8 EMITTER_SPACE = 15
uint8 EMITTER_UNASSGINED3 = 16
uint8 EMITTER_EMERGENCY_SURFACE = 17
uint8 EMITTER_SERVICE_SURFACE = 18
uint8 EMITTER_POINT_OBSTACLE = 19
# ADSB_FLAGS
uint16 FLAG_VALID_COORDS = 1
uint16 FLAG_VALID_ALTITUDE = 2
uint16 FLAG_VALID_HEADING = 4
uint16 FLAG_VALID_VELOCITY = 8
uint16 FLAG_VALID_CALLSIGN = 16
uint16 FLAG_VALID_SQUAWK = 32
uint16 FLAG_SIMULATED = 64
uint16 FLAG_VERTICAL_VELOCITY_VALID = 128
uint16 FLAG_BARO_VALID = 256
uint16 FLAG_SOURCE_UAT = 32768
# [[[end]]] (checksum: a34f2a081739921b6e3e443ed0516d8d)

std_msgs/Header header

uint32 icao_address
string callsign

float64 latitude
float64 longitude
float32 altitude 	# AMSL

 ros2 topic pub /mavros/adsb/send mavros_msgs/msg/ADSBVehicle '{icao_address: 0, lat: 0, lon: 0, altitude: 0, heading: 0, hor_velocity: 0, ver_velocity: 0, flags: 0, squawk: 0, altitude_type: 0, callsign: "", emitter_type: 0, tslc: 0, metype: 0, mesub: 0, heading_valid: False, hor_velocity_valid: False, ver_velocity_valid: False, callsign_valid: False, emitter_type_valid: False, tslc_valid: False, metype_valid: False, mesub_valid: False, squawk_valid: False, ident_valid: False, baro_valid: False, geo_valid: False, category: 0, callsign_str: ""}'
 
 ---

### * /mavros/cellular_status/status [mavros_msgs/msg/CellularStatus] 1 subscriber
 这个类型可能包含了一些关于无线通信状态的信息，如信号强度、网络类型等。
 #Follows https://mavlink.io/en/messages/common.html#CELLULAR_STATUS specification
uint8 status
uint8 failure_reason
uint8 type
uint8 quality
uint16 mcc
uint16 mnc
uint16 lac

 ros2 topic pub /mavros/cellular_status/status mavros_msgs/msg/CellularStatus '{status: 0, failure_reason: 0, quality: 0, mcc: 0, mnc: 0, lac: 0}'

---

### * /mavros/companion_process/status [mavros_msgs/msg/CompanionProcessStatus] 1 subscriber
这个类型可能包含了一些关于伴随进程状态的信息，如进程ID、状态等。
# Mavros message: COMPANIONPROCESSSTATUS

std_msgs/Header header

uint8 state			# See enum COMPANION_PROCESS_STATE
uint8 component		# See enum MAV_COMPONENT

uint8 MAV_STATE_UNINIT = 0
uint8 MAV_STATE_BOOT = 1
uint8 MAV_STATE_CALIBRATING = 2
uint8 MAV_STATE_STANDBY = 3
uint8 MAV_STATE_ACTIVE = 4
uint8 MAV_STATE_CRITICAL = 5
uint8 MAV_STATE_EMERGENCY = 6
uint8 MAV_STATE_POWEROFF = 7
uint8 MAV_STATE_FLIGHT_TERMINATION = 8

uint8 MAV_COMP_ID_OBSTACLE_AVOIDANCE = 196
uint8 MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY = 197

ros2 topic pub /mavros/companion_process/status mavros_msgs/msg/CompanionProcessStatus '{state: 0, component: 0}'

---

 * /mavros/fake_gps/mocap/pose [geometry_msgs/msg/PoseStamped] 1 subscriber
 ros2 topic pub /mavros/fake_gps/mocap/pose geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}'
mavros 异常退出
 * /mavros/global_position/global [sensor_msgs/msg/NavSatFix] 1 subscriber
 ros2 topic pub /mavros/global_position/global sensor_msgs/msg/NavSatFix '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, status: {status: 0, service: 0}, latitude: 0.0, longitude: 0.0, altitude: 0.0, position_covariance: [0.0]}'
 
 * /mavros/global_position/gp_origin [geographic_msgs/msg/GeoPointStamped] 1 subscriber
 ros2 topic pub /mavros/global_position/gp_origin geographic_msgs/msg/GeoPointStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "", }, position: {latitude: 0.0, longitude: 0.0, altitude: 0.0}}'
 
 * /mavros/global_position/set_gp_origin [geographic_msgs/msg/GeoPointStamped] 1 subscriber
  ros2 topic pub /mavros/global_position/set_gp_origin geographic_msgs/msg/GeoPointStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, position: {latitude: 0.0, longitude: 0.0, altitude: 0.0}}'
 
 * /mavros/gps_input/gps_input [mavros_msgs/msg/GPSINPUT] 1 subscriber
  #ros2 topic pub /mavros/gps_input/gps_input mavros_msgs/msg/GPSINPUT '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, time_usec: 0, fix_type: 0, lat: 0.0, lon: 0.0, alt: 0.0, eph: 0.0, epv: 0.0, vel: 0.0, vn: 0.0, ve: 0.0, vd: 0.0, cog: 0.0, satellites_visible: 0, alt_ellipsoid: 0.0, h_acc: 0.0, v_acc: 0.0, vel_acc: 0.0, hdg_acc: 0.0, yaw: 0.0, yaw_offset: 0.0}'
 
 * /mavros/gps_rtk/send_rtcm [mavros_msgs/msg/RTCM] 1 subscriber
  ros2 topic pub /mavros/gps_rtk/send_rtcm mavros_msgs/msg/RTCM '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, data: [0]}'
 

 * /mavros/home_position/home [mavros_msgs/msg/HomePosition] 1 subscriber
  ros2 topic pub /mavros/home_position/home mavros_msgs/msg/HomePosition '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, geo: {latitude: 0.0, longitude: 0.0, altitude: 0.0}, position: {x: 0.0, y: 0.0, z: 0.0}}'
 
 * /mavros/home_position/set [mavros_msgs/msg/HomePosition] 1 subscriber
  ros2 topic pub /mavros/home_position/set mavros_msgs/msg/HomePosition '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, geo: {latitude: 0.0, longitude: 0.0, altitude: 0.0}, position: {x: 1.0, y: 1.0, z: 1.0}}'
 

 * /mavros/landing_target/pose [geometry_msgs/msg/PoseStamped] 1 subscriber
  ros2 topic pub /mavros/landing_target/pose geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}'
 
 * /mavros/local_position/pose [geometry_msgs/msg/PoseStamped] 1 subscriber
  ros2 topic pub /mavros/local_position/pose geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}'
 
 * /mavros/manual_control/send [mavros_msgs/msg/ManualControl] 1 subscriber
 # ros2 topic pub /mavros/manual_control/send mavros_msgs/msg/ManualControl '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, x: 0.0, y: 0.0, z: 0.0, r: 0.0, buttons: 0, target: 0}'
 
 * /mavros/mocap/pose [geometry_msgs/msg/PoseStamped] 1 subscriber
  ros2 topic pub /mavros/mocap/pose geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}'
 
 * /mavros/mocap/tf [geometry_msgs/msg/TransformStamped] 1 subscriber
  ros2 topic pub /mavros/mocap/pose geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}'
 
 * /mavros/mount_control/command [mavros_msgs/msg/MountControl] 1 subscriber
 # ros2 topic pub /mavros/mount_control/command mavros_msgs/msg/MountControl '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, target_system: 0, target_component: 0, input_a: 0.0, input_b: 0.0, input_c: 0.0, save_position: False}'
 
 * /mavros/obstacle/send [sensor_msgs/msg/LaserScan] 1 subscriber
  ros2 topic pub /mavros/obstacle/send sensor_msgs/msg/LaserScan '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, angle_min: 0.0, angle_max: 0.0, angle_increment: 0.0, time_increment: 0.0, scan_time: 0.0, range_min: 0.0, range_max: 0.0, ranges: [0.0], intensities: [0.0]}'
 
 * /mavros/odometry/out [nav_msgs/msg/Odometry] 1 subscriber
  ros2 topic pub /mavros/odometry/out nav_msgs/msg/Odometry '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, child_frame_id: "", pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}, covariance: [0.0], twist: {twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, covariance: [0.0]}}}'
 
 * /mavros/onboard_computer/status [mavros_msgs/msg/OnboardComputerStatus] 1 subscriber
  #ros2 topic pub /mavros/onboard_computer/status mavros_msgs/msg/OnboardComputerStatus '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, uptime: 0, ram_usage: 0.0, ram_total: 0.0, disk_usage: 0.0, disk_total: 0.0, cpu_freq: 0.0, cpu_load: 0.0, temperature_board: 0.0, temperature_core: [0.0], voltage_master: 0.0, voltage_usb: 0.0, drop_rate_comm: 0.0, errors_comm: 0, errors_count1: 0, errors_count2: 0, errors_count3: 0, errors_count4: 0, errors_count5: 0, errors_count6: 0, errors_count7: 0, errors_count8: 0, errors_count9: 0, errors_count10: 0, errors_count11: 0, errors_count12: 0, errors_count13: 0, errors_count14: 0, errors_count15: 0, errors_count16: 0, errors_count17: 0, errors_count18: 0, errors_count19: 0, errors_count20: 0, errors_count21: 0, errors_count22: 0, errors_count23: 0, errors_count24: 0, errors_count25: 0, errors_count26: 0, errors_count27: 0, errors_count28: 0, errors_count29: 0, errors_count30: 0, errors_count31: 0, errors_count32: 0, errors_count33: 0, errors_count34: 0, errors_count35: 0, errors_count36: 0, errors_count37: 0, errors_count38: 0, errors_count39: 0, errors_count40: 0, errors_count41: 0, errors_count42: 0, errors_count43: 0, errors_count44: 0, errors_count45: 0, errors_count46: 0, errors_count47: 0, errors_count48: 0, errors_count49: 0, errors_count50: 0, errors_count51: 0, errors_count52: 0, errors_count53: 0, errors_count54: 0, errors_count55: 0, errors_count56: 0, errors_count57: 0, errors_count58: 0, errors_count59: 0, errors_count60: 0, errors_count61: 0, errors_count62: 0, errors_count63: 0, errors_count64: 0, errors_count65: 0, errors_count66: 0, errors_count67: 0, errors_count68: 0, errors_count69: 0, errors_count70: 0, errors_count71: 0, errors_count72: 0, errors_count73: 0, errors_count74: 0, errors_count75: 0, errors_count76: 0, errors_count77: 0, errors_count78: 0, errors_count79: 0, errors_count80: 0, errors_count81: 0, errors_count82: 0, errors_count83: 0, errors_count84: 0, errors_count85: 0, errors_count86: 0, errors_count87: 0, errors_count88: 0, errors_count89: 0, errors_count90: 0, errors_count91: 0, errors_count92: 0, errors_count93: 0, errors_count94: 0, errors_count95: 0, errors_count96: 0, errors_count97: 0, errors_count98: 0, errors_count99: 0, errors_count100: 0, errors_count101: 0, errors_count102: 0, errors_count103: 0, errors_count104: 0, errors_count105: 0, errors_count106: 0, errors_count107: 0, errors_count108: 0, errors_count109: 0, errors_count110: 0, errors_count111: 0, errors_count112: 0, errors_count113: 0, errors_count114: 0, errors_count115: 0, errors_count116: 0, errors_count117: 0, errors_count118: 0, errors_count119: 0, errors_count120: 0, errors_count121: 0, errors_count122: 0, errors_count123: 0, errors_count124: 0, errors_count125: 0, errors_count126: 0, errors_count127: 0, errors_count128: 0, errors_count129: 0, errors_count130: 0, errors_count131: 0, errors_count132: 0, errors_count133: 0, errors_count134: 0, errors_count135: 0, errors_count136: 0, errors_count137: 0, errors_count138: 0, errors_count139: 0, errors_count140: 0, errors_count141: 0, errors_count142: 0, errors_count143: 0, errors_count144: 0, errors_count145: 0, errors_count146: 0, errors_count147: 0, errors_count148: 0, errors_count149: 0, errors_count150: 0, errors_count151: 0, errors_count152: 0, errors_count153: 0, errors_count154: 0, errors_count155: 0, errors_count156: 0, errors_count157: 0, errors_count158: 0, errors_count159: 0, errors_count160: 0, errors_count161: 0, errors_count162: 0, errors_count163: 0, errors_count164: 0, errors_count165: 0, errors_count166: 0, errors_count167: 0, errors_count168: 0, errors_count169: 0, errors_count170: 0, errors_count171: 0, errors_count172: 0, errors_count173: 0, errors_count174: 0, errors_count175: 0, errors_count176: 0, errors_count177: 0, errors_count178: 0, errors_count179: 0, errors_count180: 0, errors_count181: 0, errors_count182: 0, errors_count183: 0, errors_count184: 0, errors_count185: 0, errors_count186: 0, errors_count187: 0, errors_count188: 0, errors_count189: 0, errors_count190: 0, errors_count191: 0, errors_count192: 0, errors_count193 ... (length 1)
 
 * /mavros/optical_flow/raw/send [mavros_msgs/msg/OpticalFlow] 1 subscriber
  #ros2 topic pub /mavros/optical_flow/raw/send mavros_msgs/msg/OpticalFlow '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, integration_time_us: 0, time_delta_distance_us: 0, frame_count_since_last_readout: 0, pixel_flow_x_integral: 0.0, pixel_flow_y_integral: 0.0, gyro_x_rate_integral: 0.0, gyro_y_rate_integral: 0.0, gyro_z_rate_integral: 0.0, ground_distance_m: 0.0, integration_timespan: 0.0, time_since_last_sonar_update: 0.0, frame_count_since_last_readout: 0, gyro_temperature: 0.0, quality: 0, max_flow_rate: 0.0, min_ground_distance: 0.0, max_ground_distance: 0.0}'
 
 * /mavros/play_tune [mavros_msgs/msg/PlayTuneV2] 1 subscriber
  #ros2 topic pub /mavros/play_tune mavros_msgs/msg/PlayTuneV2 '{tune: "", tempo: 0.0, frequency: [0], duration: [0.0], silence: 0.0}'
 
 * /mavros/rangefinder_sub [sensor_msgs/msg/Range] 1 subscriber
  ros2 topic pub /mavros/rangefinder_sub sensor_msgs/msg/Range '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, radiation_type: 0, field_of_view: 0.0, min_range: 0.0, max_range: 0.0, range: 0.0}'
 
 * /mavros/rc/override [mavros_msgs/msg/OverrideRCIn] 1 subscriber
  #ros2 topic pub /mavros/rc/override mavros_msgs/msg/OverrideRCIn '{channels: [0, 0, 0, 0, 0, 0, 0, 0]}'
 
 * /mavros/setpoint_accel/accel [geometry_msgs/msg/Vector3Stamped] 1 subscriber
  ros2 topic pub /mavros/setpoint_accel/accel geometry_msgs/msg/Vector3Stamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, vector: {x: 0.0, y: 0.0, z: 0.0}}'
 
 * /mavros/setpoint_attitude/cmd_vel [geometry_msgs/msg/TwistStamped] 1 subscriber
  ros2 topic pub /mavros/setpoint_attitude/cmd_vel geometry_msgs/msg/TwistStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, twist: {linear: {x: 0.0, y: 0.0, z: 5.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}'

/////////////////////////////////////////// 
这行代码是在描述一个ROS（Robot Operating System）主题的信息。在ROS中，主题是节点之间进行通信的一种方式。一个节点可以发布消息到一个主题，其他节点可以订阅这个主题来接收这些消息。

在这个例子中，`/mavros/setpoint_attitude/thrust`是主题的名称。这个主题可能是用于发送关于推力设定点的消息。

`mavros_msgs/msg/Thrust`是这个主题的消息类型。这表明发送到这个主题的消息应该是`Thrust`类型，这个类型可能包含了一些关于推力设定点的信息。

`1 subscriber`表明当前有一个节点订阅了这个主题。这意味着当一个消息被发布到这个主题时，这个订阅者节点会接收到这个消息。
 
 ---
  no link 
 * /mavros/setpoint_attitude/thrust [mavros_msgs/msg/Thrust] 1 subscriber
  ros2 topic pub /mavros/setpoint_attitude/thrust mavros_msgs/msg/Thrust '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, thrust: 0.0}'
 
 * /mavros/setpoint_position/global [geographic_msgs/msg/GeoPoseStamped] 1 subscriber
  ros2 topic pub /mavros/setpoint_position/global geographic_msgs/msg/GeoPoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, pose: {position: {latitude: 0.0, longitude: 0.0, altitude: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}'
 * /mavros/setpoint_position/global_to_local [geographic_msgs/msg/GeoPoseStamped] 1 subscriber
  ros2 topic pub /mavros/setpoint_position/global_to_local geographic_msgs/msg/GeoPoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, pose: {position: {latitude: 0.0, longitude: 0.0, altitude: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}'
 * /mavros/setpoint_position/local [geometry_msgs/msg/PoseStamped] 1 subscriber
  ros2 topic pub /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}'
 * /mavros/setpoint_raw/attitude [mavros_msgs/msg/AttitudeTarget] 1 subscriber
  ros2 topic pub /mavros/setpoint_raw/attitude mavros_msgs/msg/AttitudeTarget '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, type_mask: 0, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}, body_rate: {x: 0.0, y: 0.0, z: 0.0}, thrust: 0.0}'
 * /mavros/setpoint_raw/global [mavros_msgs/msg/GlobalPositionTarget] 1 subscriber
  ros2 topic pub /mavros/setpoint_raw/global mavros_msgs/msg/GlobalPositionTarget '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, coordinate_frame: 0, type_mask: 0, lat_lon: {latitude: 0.0, longitude: 0.0}, alt: 0.0, vx: 0.0, vy: 0.0, vz: 0.0, afx: 0.0, afy: 0.0, afz: 0.0, yaw: 0.0, yaw_rate: 0.0}'
 * /mavros/setpoint_raw/local [mavros_msgs/msg/PositionTarget] 1 subscriber
  ros2 topic pub /mavros/setpoint_raw/local mavros_msgs/msg/PositionTarget '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, coordinate_frame: 0, type_mask: 0, position: {x: 0.0, y: 0.0, z: 0.0}, velocity: {x: 0.0, y: 0.0, z: 0.0}, acceleration_or_force: {x: 0.0, y: 0.0, z: 0.0}, yaw: 0.0, yaw_rate: 0.0}'
 * /mavros/setpoint_trajectory/local [trajectory_msgs/msg/MultiDOFJointTrajectory] 1 subscriber
  ros2 topic pub /mavros/setpoint_trajectory/local trajectory_msgs/msg/MultiDOFJointTrajectory '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, joint_names: [], points: [{transforms: [{translation: {x: 0.0, y: 0.0, z: 0.0}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}], velocities: [{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}], accelerations: [{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}], time_from_start: {sec: 0, nanosec: 0}}]}'
 * /mavros/setpoint_velocity/cmd_vel [geometry_msgs/msg/TwistStamped] 1 subscriber
  ros2 topic pub /mavros/setpoint_velocity/cmd_vel geometry_msgs/msg/TwistStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}'
 * /mavros/setpoint_velocity/cmd_vel_unstamped [geometry_msgs/msg/Twist] 1 subscriber
  ros2 topic pub /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
 * /mavros/statustext/send [mavros_msgs/msg/StatusText] 1 subscriber
  ros2 topic pub /mavros/statustext/send mavros_msgs/msg/StatusText '{severity: 0, text: ""}'
 * /mavros/trajectory/generated [mavros_msgs/msg/Trajectory] 1 subscriber
  ros2 topic pub /mavros/trajectory/generated mavros_msgs/msg/Trajectory '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, point_1: {time_from_start: {sec: 0, nanosec: 0}, transforms: [{translation: {x: 0.0, y: 0.0, z: 0.0}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}], velocities: [{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}], accelerations: [{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}]}}'
 * /mavros/trajectory/path [nav_msgs/msg/Path] 1 subscriber
  ros2 topic pub /mavros/trajectory/path nav_msgs/msg/Path '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, poses: [{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}]}]'
 * /mavros/tunnel/in [mavros_msgs/msg/Tunnel] 1 subscriber
  ros2 topic pub /mavros/tunnel/in mavros_msgs/msg/Tunnel '{payload: [0]}'
 * /mavros/vision_pose/pose [geometry_msgs/msg/PoseStamped] 1 subscriber
  ros2 topic pub /mavros/vision_pose/pose geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}'
 * /mavros/vision_pose/pose_cov [geometry_msgs/msg/PoseWithCovarianceStamped] 1 subscriber
  ros2 topic pub /mavros/vision_pose/pose_cov geometry_msgs/msg/PoseWithCovarianceStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}, covariance: [0.0]}'
 * /mavros/vision_speed/speed_twist [geometry_msgs/msg/TwistStamped] 1 subscriber
  ros2 topic pub /mavros/vision_speed/speed_twist geometry_msgs/msg/TwistStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}'
 * /mavros/vision_speed/speed_twist_cov [geometry_msgs/msg/TwistWithCovarianceStamped
  ros2 topic pub /mavros/vision_speed/speed_twist_cov geometry_msgs/msg/TwistWithCovarianceStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, twist: {twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, covariance: [0.0]}'
 * */
