#include "OffboardControl.h"
/*
    北360|0偏航角            
		  y+                       |y+
西       x+ 东  <=> ---- x-____|______x+  0
270       90                   |
                               |y+
	  南180                      90

 世界坐标系(东北天)    飞机坐标系(base_link)
 -> g2l_location
    aircraft_deg = 90 - world_deg
    aircraft_deg %= 360.0

*/

void OffboardControl::timer_callback(void)
{
	// 发布当前状态
	publish_current_state();
	// RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "当前时间：%f", get_cur_time());
	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "收到坐标c(%f, %f) h(%f ,%f), flag_servo = %d yaw=%f",
		_yolo->get_x(YOLO::TARGET_TYPE::CIRCLE), _yolo->get_y(YOLO::TARGET_TYPE::CIRCLE),
		_yolo->get_x(YOLO::TARGET_TYPE::H), _yolo->get_y(YOLO::TARGET_TYPE::H),
		_yolo->get_servo_flag(), get_yaw());
	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "当前飞机位置 x: %f y: %f z: %f yaw: %f",
		get_x_pos(), get_y_pos(), get_z_pos(), get_yaw());
	
	CameraParams camera1;
	camera1.position = Vector3d(get_x_pos(), get_y_pos(), get_z_pos());
	camera1.rotation = Vector3d(0, -M_PI/2, 0);  // 俯仰角-90度(向下看)
	camera1.fx = 360;
	camera1.fy = 360;
	camera1.cx = 320;
	camera1.cy = 240;
	camera1.width = 640;
	camera1.height = 480;
 	Vector2d image_point1(
		_yolo->get_x(YOLO::TARGET_TYPE::CIRCLE),
		_yolo->get_y(YOLO::TARGET_TYPE::CIRCLE)
	);
	// std::cout << "Image width: " << camera1.width << ", height: " << camera1.height << std::endl;
	// std::cout << "Image point 1: " << image_point1.transpose() << std::endl;
	// std::cout << "Camera 1 position: " << camera1.position.transpose() << std::endl;
  	auto target1 = calculateWorldPosition(image_point1, camera1, 0.0, 0.0);
	if (target1) {
			std::cout << "Example 1 - Target position: " << std::endl;
			std::cout << *target1 << std::endl;
	}

	if (_motors->mode == "LAND" && state_machine_.get_current_state() != FlyState::end)
	{
		RCLCPP_INFO(this->get_logger(), "飞行结束，进入结束状态");
		state_machine_.transition_to(FlyState::end);
		return;
	}

	state_machine_.execute_dynamic_tasks();
	state_machine_.process_states<
		FlyState::init,
		FlyState::takeoff,

		FlyState::end,
		// 
		FlyState::Goto_shotpoint,
		FlyState::Doshot,
		FlyState::Goto_scoutpoint,
		FlyState::Surround_see,
		FlyState::Doland,
		//
		FlyState::Termial_Control,
		FlyState::Print_Info,
		FlyState::Reflush_config,
		FlyState::MYPID

	>();
}


void OffboardControl::FlyState_init()
{
		// 读取到配置文件中的投弹区和侦查区坐标
		// headingangle_compass为罗盘读数
    // angle为四元数的角度，
    // dx,dy为飞机坐标系下的横纵坐标
    // x,y为global坐标下的，x正指向正东，y正指向正北
		// float tx_shot = dx_shot - 0.5;
		// float ty_shot = dy_shot;
		// float tx_see = dx_see;
		// float ty_see = dy_see;
		// 旋转到飞机坐标系当前机头朝向角度
		float x_shot, y_shot, x_see, y_see;
		rotate_global2stand(tx_shot, ty_shot, x_shot, y_shot);
		rotate_global2stand(tx_see, ty_see, x_see, y_see);
		RCLCPP_INFO(this->get_logger(), "默认方向下角度：%f", default_yaw);
		RCLCPP_INFO(this->get_logger(), "起始点投弹区起点 x: %f   y: %f    angle: %f", x_shot, y_shot, default_yaw);
    RCLCPP_INFO(this->get_logger(), "起始点侦查起点 x: %f   y: %f    angle: %f", x_see, y_see, default_yaw);
		rotate_2start(tx_shot, ty_shot, x_shot, y_shot);
		rotate_2start(tx_see, ty_see, x_see, y_see);
		RCLCPP_INFO(this->get_logger(), "飞机坐标投弹区起点 x: %f   y: %f    angle: %f", x_shot, y_shot, default_yaw);
    RCLCPP_INFO(this->get_logger(), "飞机坐标侦查起点 x: %f   y: %f    angle: %f", x_see, y_see, default_yaw);
		rotate_2local(tx_shot, ty_shot, x_shot, y_shot);
		rotate_2local(tx_see, ty_see, x_see, y_see);
		RCLCPP_INFO(this->get_logger(), "当前朝向投弹区起点 x: %f   y: %f    angle: %f", x_shot, y_shot, default_yaw);
    RCLCPP_INFO(this->get_logger(), "当前朝向侦查起点 x: %f   y: %f    angle: %f", x_see, y_see, default_yaw);
		
	_camera_gimbal->set_gimbal(
		-90.0, 0.0, 0.0
	);
	 
	// RCLCPP_INFO(this->get_logger(), "开始初始化舵机");
	// 初始化舵机操作 等待舵机初始化
	// while (!_servo_controller.client_->wait_for_service(std::chrono::seconds(1)))
	// {
	// 		if (!rclcpp::ok())
	// 		{
	// 				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the Servo service. Exiting.");
	// 				return;
	// 		}
	// 		RCLCPP_INFO(this->get_logger(), "Servo Service not available, waiting again...");
	// }
  // servo_controller(12, 1050);
  // RCLCPP_INFO(this->get_logger(), "结束初始化舵机");

	// rclcpp::sleep_for(1s);
	// if (is_equal(get_x_pos(), DEFAULT_X_POS))
	// {
	// 	// THROTTLE表示节流的意思，以下代码节流时间间隔为 500 毫秒（即 5 秒）．
	// 	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "没有获取到位置数据，等待GPS信号...");
	// 	return;
	// }
	// 飞控的扩展卡尔曼滤波器（EKF3）已经为IMU（惯性测量单元）0和IMU1设置了起点。
	start = {get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()}; 
	// 飞控日志 AP: Field Elevation Set: 0m 设定当前位置的地面高度为0米，这对于高度控制和避免地面碰撞非常重要。
	start_global = {get_lat(), get_lon(), get_alt()};			
	RCLCPP_INFO(this->get_logger(), "初始旋转角: %f", get_yaw());
}

// 发布状态
void OffboardControl::publish_current_state()
{
  // auto message = std_msgs::msg::String();
	auto message = std_msgs::msg::Int32();
  // message.data = "Current state: " + std::to_string(static_cast<int>(state_machine_.current_state_));
	message.data = fly_state_to_int(state_machine_.get_current_state());
  state_publisher_->publish(message);
}