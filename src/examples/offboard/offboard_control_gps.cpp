#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol_local.hpp>
#include <mavros_msgs/srv/command_tol.hpp>

//#include <ardupilot_msgs/msg/global_position.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/altitude.hpp>

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
#define DEfault_HEADING 60.00//角度制
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
	OffboardControl(const std::string ardupilot_namespace,std::shared_ptr<ServoController> servo_controller_) :
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
		
		
		arm_motors_client_{this->create_client<mavros_msgs::srv::CommandBool>(ardupilot_namespace+"cmd/arming")},
		mode_switch_client_{this->create_client<mavros_msgs::srv::SetMode>(ardupilot_namespace+"set_mode")}
		
	{
		ardupilot_namespace_copy_ = ardupilot_namespace;

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
		
		//ros2 topic echo /mavros/global_position/global sensor_msgs/msg/NavSatFix
		gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(ardupilot_namespace_copy_+"global_position/global", qos,
		std::bind(&OffboardControl::gps_callback, this, std::placeholders::_1),sub_opt);
		
		
		//订阅IMU信息
		// * /mavros/imu/data [sensor_msgs/msg/Imu] 1 publisher
		//ros2 topic echo /mavros/imu/data
		
		//查询系统状态
		// * /mavros/state [mavros_msgs/msg/State] 1 publisher
		//ros2 topic echo /mavros/state
		state_subscription_ = this->create_subscription<mavros_msgs::msg::State>(ardupilot_namespace_copy_+"state", qos,
		 std::bind(&OffboardControl::state_callback, this, std::placeholders::_1));
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
	double quaternion_to_yaw(double x, double y, double z, double w);
	void yaw_to_quaternion(double yaw, double* x, double* y, double* z, double* w);
private:
	//std::shared_ptr<YOLO> yolo_;
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
	const double default_heading=DEfault_HEADING;//初始偏转角
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

	double k=0.002;//控制vx和vy
	// 初始化PID控制器
	double dt=0.1;
	double kp = 0.41;  // 比例参数
	double ki = 0.06;  // 积分参数
	double kd = 0.28;  // 微分参数
	double kp_yaw = 0.20;  // 比例参数
	double ki_yaw = 0.04;  // 积分参数
	double kd_yaw = 0.04;  // 微分参数
	double max_vx=1; //前后方向最大速度
	double max_vy=1; //左右方向最大速度
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
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
	rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscription_;
	rclcpp::Subscription<mavros_msgs::msg::Altitude>::SharedPtr altitude_subscription_;
	rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscription_;
 	rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_motors_client_;
	rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_switch_client_;
	
	
	void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
	void state_callback(const mavros_msgs::msg::State::SharedPtr msg);

	void switch_to_guided_mode();
	void switch_to_flip_mode();
	void switch_to_rtl_mode();

	void command_takeoff_or_land(std::string mode);

	void trajectory_setpoint_global(double x,double y ,double z , double lat, double lon, double alt,double yaw);
	
	void publish_setpoint_raw_global(double latitude, double longitude, double altitude, double yaw);
	
	void set_target_point_global(std::string mode,double lat=0,double lon=0,double alt=0,double yaw=0);

	inline void get_target_location_global(double &x,double &y,double &z,double lat,double lon,double alt,double default_heading=0);

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
void OffboardControl::state_callback(const mavros_msgs::msg::State::SharedPtr msg) 
{
	drone_state_ = *msg;
	// RCLCPP_INFO(this->get_logger(), "Received state data");
	// RCLCPP_INFO(this->get_logger(), "Mode: %s", msg->mode.c_str());
	// RCLCPP_INFO(this->get_logger(), "Armed: %d", msg->armed);
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
void OffboardControl::timer_callback(void){
	static uint8_t num_of_steps = 0;
	static bool is_takeoff = false;
	std::cout << "alt: " << location.global_frame.lat << " lon " << location.global_frame.lon << " alt: " << location.global_frame.alt  << std::endl;
	
	RCLCPP_INFO(this->get_logger(), "global_gps: %lf %lf %lf", location.global_frame.lat, location.global_frame.lon, location.global_frame.alt);
	switch (fly_state_)
	{
	case FlyState::init:
		rclcpp::sleep_for(1s);
		if(global_gps_.header.stamp.sec == 0){
			RCLCPP_INFO(this->get_logger(), "No pose data received yet");
			break;
		}
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
	case FlyState::takeoff:
		RCLCPP_INFO(this->get_logger(), "arm_done_: %d", arm_done_);
		if (is_takeoff){	
			//if (pose_.pose.position.z>2.5+start.z){
			if (false){
				RCLCPP_INFO(this->get_logger(), "goto_shot_area start, totaltime=%fs", (this->get_clock()->now().nanoseconds() / 1000- timestamp0)/1000000.0);
				fly_state_ = FlyState::goto_shot_area;
			}
			else{
				// 设定速度
				// send_velocity_command(0, 0, 0.5, 0);

				// 飞行到指定坐标
				//publish_setpoint_raw_global(59.659859999999995, 31.7103000000003,20, 0);
				
				// ros2 topic pub /mavros/setpoint_raw/global mavros_msgs/msg/GlobalPositionTarget '{header: "auto", latitude: -359.659859999999995, longitude: 31.7103000000003, altitude7 30}'
				// 飞行到指定经纬度
				publish_setpoint_raw_global(start_global.lat, start_global.lon, start_global.alt+10, 0);
				std::cout<<"lat:"<<start_global.lat<<" lon:"<< start_global.lon<<" alt:"<< start_global.alt<<std::endl;
				
				// 本地坐标使用gps
				// static bool first = true;
				// static GlobalFrame temp_location;
				// if(first){
				// 	temp_location = {location.global_frame.lat, location.global_frame.lon, location.global_frame.alt, 0};
				// 	first = false;
				// }
				// trajectory_setpoint_global(10, 10, 10, temp_location.lat, temp_location.lon, temp_location.alt, 0);
			
			}
		}
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

	auto servocountroller_ = std::make_shared<ServoController>();
	auto node = std::make_shared<OffboardControl>("/mavros/",servocountroller_);
	
	// auto pubnode = std::make_shared<PublisherNode>();
	/* 运行节点，并检测退出信号*/

	executor.add_node(node);
	executor.add_node(servocountroller_);

	executor.spin();

	rclcpp::shutdown();
	return 0;
}
