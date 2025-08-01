#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/command_home.hpp>
#include <mavros_msgs/msg/home_position.hpp>
#include <mavros_msgs/msg/state.hpp>


#include "rclcpp/rclcpp.hpp"
// #include "OffboardControl.h"
#include "math.h"
#include "OffboardControl_Base.h"

class Motors{
public:
    Motors(const std::string ardupilot_namespace,OffboardControl_Base* node) : node(node)
	{
		this->ardupilot_namespace = ardupilot_namespace;
		state_ = State::init;

		// RCLCPP_INFO(node->get_logger(), "Starting Motors Example");
        // 质量服务配置
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		// 声明回调组,实例化回调组，类型为：可重入的
		rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_= node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
		// Each of these callback groups is basically a thread
    	// Everything assigned to one of them gets bundled into the same thread
		auto sub_opt = rclcpp::SubscriptionOptions();
    	sub_opt.callback_group = callback_group_subscriber_;

        arm_motors_client_ = node->create_client<mavros_msgs::srv::CommandBool>(ardupilot_namespace+"cmd/arming");
	    // mode_switch_client_ = node->create_client<mavros_msgs::srv::SetMode>(ardupilot_namespace+"set_mode");
	    mode_switch_client_ = node->mode_switch_client_;
        set_home_client_ = node->create_client<mavros_msgs::srv::CommandHome>(ardupilot_namespace+"cmd/set_home");
        takeoff_client_ = node->create_client<mavros_msgs::srv::CommandTOL>(ardupilot_namespace+"cmd/takeoff");
        land_client_ = node->create_client<mavros_msgs::srv::CommandTOL>(ardupilot_namespace+"cmd/land");
        
        state_subscription_ = node->create_subscription<mavros_msgs::msg::State>(ardupilot_namespace+"state", qos,
		    std::bind(&Motors::state_callback, this, std::placeholders::_1),sub_opt);
		// * /mavros/home_position/home [mavros_msgs/msg/HomePosition] 1 subscriber
		home_position_subscription_ = node->create_subscription<mavros_msgs::msg::HomePosition>(ardupilot_namespace+"home_position/home", qos,
		    std::bind(&Motors::home_position_callback, this, std::placeholders::_1),sub_opt);

    }

    void state_callback(const mavros_msgs::msg::State::SharedPtr msg);
    void home_position_callback(const mavros_msgs::msg::HomePosition::SharedPtr msg);

	// "GUIDED"/"RTL"
    void switch_mode(std::string mode);
    // void switch_to_guided_mode();
	// void switch_to_flip_mode();
	// void switch_to_rtl_mode();
    
	void arm_motors(bool arm);
	void set_home_position(float lat, float lon, float alt, float yaw = 0.0f);
	void set_home_position(float yaw = 0.0f); // 使用当前位置

	auto get_set_home_client() {
		return set_home_client_;
	}

	//"TAKEOFF" or "LAND"
	void command_takeoff_or_land(std::string mode, float altitude = 5.0f, float yaw = 0.0f);

	bool takeoff(float local_frame_z ,float takeoff_altitude = 5.0f, float yaw = 0.0f);

	bool takeoff_command = false;
	bool _arm_done; //arm_motor's reply
	enum class State{
		init,
		wait_for_takeoff_command,
		wait_for_stable_offboard_mode,
		arm_requested,
		takeoff,
		autotune_mode,
		
	} state_;
	// State: GUIDED, Armed: 0, Connected: 1, Guided: 1, System Status: 
	// State: GUIDED, Armed: 1, Connected: 1, Guided: 1, System Status:
	// State: RTL, Armed: 1, Connected: 1, Guided: 1, System Status:  
	// State: LAND, Armed: 1, Connected: 1, Guided: 1, System Status:
	bool is_takeoff = false;
	bool armed;
	bool connected;
	bool guided;
	std::string mode;
	std::string system_status;
	Vector3f home_position;//{FLT_MIN,0,0};
	Vector3f home_position_global;
	Quaternionf home_quaternion;// 四元数
private:
	OffboardControl_Base* node;
	std::string ardupilot_namespace;
	// rclcpp::Node::SharedPtr node;


	rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscription_;
	rclcpp::Subscription<mavros_msgs::msg::HomePosition>::SharedPtr home_position_subscription_;
 	rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_motors_client_;
	rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_switch_client_;
	rclcpp::Client<mavros_msgs::srv::CommandHome>::SharedPtr set_home_client_;
	rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
	rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
// 	系统时间
// 用于时间同步。
// Time:
// /mavros/time_reference (sensor_msgs/TimeReference)
    

};
