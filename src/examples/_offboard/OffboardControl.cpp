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

	// 这里是定时器回调函数的实现

	state_machine_.execute_dynamic_tasks();
	state_machine_.process_states<
		FlyState::init,
		FlyState::takeoff,

		// FlyState::end,
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
	if (is_equal(get_x_pos(), DEFAULT_X_POS))
	{
		// THROTTLE表示节流的意思，以下代码节流时间间隔为 500 毫秒（即 5 秒）．
		RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "没有获取到位置数据，等待GPS信号...");
		return;
	}
	// 飞控的扩展卡尔曼滤波器（EKF3）已经为IMU（惯性测量单元）0和IMU1设置了起点。
	start = {get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()}; 
	// 飞控日志 AP: Field Elevation Set: 0m 设定当前位置的地面高度为0米，这对于高度控制和避免地面碰撞非常重要。
	start_global = {get_lat(), get_lon(), get_alt()};			
	RCLCPP_INFO(this->get_logger(), "初始旋转角: %f", get_yaw());
}

// 具体状态处理实现
template<>
void OffboardControl::StateMachine::handle_state<FlyState::init>() {
	if (current_state_ == FlyState::init) {
		parent_.FlyState_init();
		RCLCPP_INFO_ONCE(parent_.get_logger(), "初始化完成");
		transition_to(FlyState::takeoff);
	}
}

template<>
void OffboardControl::StateMachine::handle_state<FlyState::takeoff>() {
	if (current_state_ == FlyState::takeoff){
		RCLCPP_INFO_ONCE(parent_.get_logger(), "开始起飞");

		if (parent_._motors->takeoff(parent_.get_z_pos())) {
				RCLCPP_INFO_ONCE(parent_.get_logger(), "起飞成功");
				transition_to(FlyState::Goto_shotpoint);
		} else {
				// RCLCPP_INFO(parent_.get_logger(), "起飞失败");
		}
	}
}

template<>
void OffboardControl::StateMachine::handle_state<FlyState::end>() {
    if (current_state_ == FlyState::end) {
        RCLCPP_INFO_ONCE(parent_.get_logger(), "任务结束");
        // 如果需要，可以在这里添加清理或退出逻辑
        rclcpp::shutdown();  // 停止 ROS 2 节点
    }
}

template<>
void OffboardControl::StateMachine::handle_state<FlyState::Goto_shotpoint>() {
	if (current_state_ == FlyState::Goto_shotpoint) {
		RCLCPP_INFO_ONCE(parent_.get_logger(), "开始前往投弹区起点");
		float x_shot, y_shot;
		parent_.rotate_global2stand(parent_.tx_shot, parent_.ty_shot, x_shot, y_shot);
		if(parent_.state_timer_.elapsed() > 12)
		{
			parent_.state_timer_.set_start_time_to_default();
			RCLCPP_INFO(parent_.get_logger(), "到达投弹区起点");
			transition_to(FlyState::Doshot);
		} else {
			parent_.send_local_setpoint_command(
				x_shot, y_shot, parent_.shot_halt, parent_.default_yaw
			);
			RCLCPP_INFO_THROTTLE(parent_.get_logger(), *parent_.get_clock(), 3000, "前往投弹区中...%f", parent_.state_timer_.elapsed());
		}
	}
}


template<>
void OffboardControl::StateMachine::handle_state<FlyState::Doshot>() {
	if (current_state_ == FlyState::Doshot) {
		RCLCPP_INFO_ONCE(parent_.get_logger(), "执行投弹任务Doshot");
		static Timer doshot_start = Timer();  // 全程计时器
		static bool arrive = false;   // 投弹结束标志
		static int counter = 0; // 航点计数器
		if (doshot_start.elapsed() > 60) // 超时 60 秒
		{
			RCLCPP_INFO(parent_.get_logger(), "超时");
			arrive = true;
		} else if (parent_.catch_target(arrive, YOLO::TARGET_TYPE::CIRCLE)
		){// (在高度高于1.6m前提下)判断是否找到目标，找到目标改变arrive值为true，如未找到目标
			// 距离执行上一个航点时长超过3秒时，定义投弹区的宽度和长度，顺序发布航点
			// 判断是否遍历完投弹区
			//RCLCPP_INFO(parent_.get_logger(), "寻找完毕，投弹!!投弹!!");
		// } else if(parent_.get_rangefinder_distance() < 2.0){ // 距离地面高度小于2米
		// 	parent_.send_velocity_command(
		// 		0.0, 0.0, 0.5, 0.0
		// 	); // 发送速度命令
		} else if (parent_.waypoint_goto_next(
								parent_.tx_shot, parent_.ty_shot, parent_.shot_length, parent_.shot_width, 
								parent_.shot_halt, parent_.surround_shot_points, 3.0, &counter, "投弹区")
		) {
			RCLCPP_INFO(parent_.get_logger(), "投弹区航点计数器：%d", counter);
		}
		// 接受投弹信号
		// if (get_servo_flag() == true) // 找到目标
		// {
		// 	RCLCPP_INFO(parent_.get_logger(), "找到目标");
		// 	arrive = true;
		// }
		if (arrive) // 执行投弹命令
		{
			RCLCPP_INFO(parent_.get_logger(), "投弹!!投弹!!，总用时：%f", doshot_start.elapsed());
			// 设置舵机位置
			parent_._servo_controller->set_servo(9, 1800);
			parent_._servo_controller->set_servo(10, 1800);
			doshot_start.set_start_time_to_default();
			counter = 0;
			arrive = false;
			RCLCPP_INFO(parent_.get_logger(), "投弹完成，3s后前往侦查区域");
			rclcpp::sleep_for(3s);
			// rclcpp::sleep_for(std::chrono::seconds(2));
			transition_to(FlyState::Goto_scoutpoint);
		}
		return;
	}
}

template<>
void OffboardControl::StateMachine::handle_state<FlyState::Goto_scoutpoint>() {
	if (current_state_ == FlyState::Goto_scoutpoint) {
		RCLCPP_INFO_ONCE(parent_.get_logger(), "开始前往侦查起点");
		float x_see, y_see;
		parent_.rotate_global2stand(parent_.tx_see, parent_.ty_see, x_see, y_see);
		if(parent_.state_timer_.elapsed() > 10)
		{
			parent_.state_timer_.set_start_time_to_default();
			RCLCPP_INFO(parent_.get_logger(), "到达侦查区起点");
			transition_to(FlyState::Surround_see);
		} else {
			parent_.send_local_setpoint_command(
				x_see, y_see, parent_.see_halt, parent_.default_yaw
			);
			RCLCPP_INFO_THROTTLE(parent_.get_logger(), *parent_.get_clock(), 3000, "前往侦查区中...%f", parent_.state_timer_.elapsed());
		}
	}
}

template<>
void OffboardControl::StateMachine::handle_state<FlyState::Surround_see>() {
	if (current_state_ == FlyState::Surround_see) {
		static int counter = 0; // 航点计数器
		if (parent_.waypoint_goto_next(
			parent_.tx_see, parent_.ty_see, parent_.see_length, parent_.see_width, 
			parent_.see_halt, parent_.surround_see_points, 4.0, &counter, "侦查区"))
		{
			RCLCPP_INFO_ONCE(parent_.get_logger(), "侦查完毕");
			counter = 0;
			parent_._motors->switch_mode("RTL");
			rclcpp::sleep_for(std::chrono::seconds(11));
			parent_._motors->switch_mode("GUIDED");
			transition_to(FlyState::Doland);
		}
	}
}

template<>
void OffboardControl::StateMachine::handle_state<FlyState::Doland>() {
	if (current_state_ == FlyState::Doland) {
		if(parent_.Doland()){
			parent_._motors->switch_mode("LAND");
			transition_to(FlyState::end);
		}
	}
}

template<>
void OffboardControl::StateMachine::handle_state<FlyState::Print_Info>() {
	if (current_state_ == FlyState::Print_Info
	) {
		RCLCPP_INFO_THROTTLE(parent_.get_logger(), *parent_.get_clock(), 500, "--------timer_callback----------");
		RCLCPP_INFO_THROTTLE(parent_.get_logger(), *parent_.get_clock(), 500, "x:   %f, y:  %f, z: %f", parent_.get_x_pos(), parent_.get_y_pos(), parent_.get_z_pos());
		RCLCPP_INFO_THROTTLE(parent_.get_logger(), *parent_.get_clock(), 500, "vx:  %f, vy: %f, vz: %f", parent_.get_x_vel(), parent_.get_y_vel(), parent_.get_z_vel());
		RCLCPP_INFO_THROTTLE(parent_.get_logger(), *parent_.get_clock(), 500, "yaw: %f", parent_.get_yaw());
		RCLCPP_INFO_THROTTLE(parent_.get_logger(), *parent_.get_clock(), 500, "yaw_e: %f", parent_.get_yaw_eigen());
		RCLCPP_INFO_THROTTLE(parent_.get_logger(), *parent_.get_clock(), 500, "yaw_vel: %f", parent_.get_yaw_vel());
		RCLCPP_INFO_THROTTLE(parent_.get_logger(), *parent_.get_clock(), 500, "lat: %f, lon: %f, alt: %f", parent_.get_lat(), parent_.get_lon(), parent_.get_alt());
		RCLCPP_INFO_THROTTLE(parent_.get_logger(), *parent_.get_clock(), 500, "rangefinder_distance:  %f", parent_.get_rangefinder_distance());
		RCLCPP_INFO_THROTTLE(parent_.get_logger(), *parent_.get_clock(), 500, "armed:     %d", parent_.get_armed());
		RCLCPP_INFO_THROTTLE(parent_.get_logger(), *parent_.get_clock(), 500, "connected: %d", parent_.get_connected());
		RCLCPP_INFO_THROTTLE(parent_.get_logger(), *parent_.get_clock(), 500, "guided:    %d", parent_.get_guided());
		RCLCPP_INFO_THROTTLE(parent_.get_logger(), *parent_.get_clock(), 500, "mode:      %s", parent_.get_mode().c_str());
		RCLCPP_INFO_THROTTLE(parent_.get_logger(), *parent_.get_clock(), 500, "system_status:  %s", parent_.get_system_status().c_str());
		RCLCPP_INFO_THROTTLE(parent_.get_logger(), *parent_.get_clock(), 500, "z_home_pos:     %f", parent_.get_z_home_pos());
		auto now = parent_.get_clock()->now();
		RCLCPP_INFO_THROTTLE(parent_.get_logger(), *parent_.get_clock(), 500, "Current time: %f s,%ld nanos", now.seconds(), now.nanoseconds());
	}
}

template<>
 void OffboardControl::StateMachine::handle_state<FlyState::MYPID>() {
 	if (current_state_ == FlyState::MYPID
 	) {
 		parent_.mypid.readPIDParameters("pos_config.yaml","mypid");
 		parent_.dx_shot = parent_.mypid.read_goal("OffboardControl.yaml","dx_shot");
 	    parent_.dy_shot = parent_.mypid.read_goal("OffboardControl.yaml","dy_shot");
 		parent_.shot_halt = parent_.mypid.read_goal("OffboardControl.yaml","shot_halt");
 		printf("已进入MYPID状态,当前kp ki kd参数分别为：%lf %lf %lf,输出限制为： %lf,积分限制为： %lf\n",parent_.mypid.kp_,parent_.mypid.ki_,parent_.mypid.kd_,parent_.mypid.output_limit_,parent_.mypid.integral_limit);
 
 		parent_.mypid.velocity_x = parent_.get_x_vel();
         parent_.mypid.velocity_y = parent_.get_y_vel();
 		parent_.mypid.velocity_z = parent_.get_z_vel();
 
 		printf("当前速度分别为：vx: %lf vy: %lf vz:%lf\n",parent_.mypid.velocity_x,parent_.mypid.velocity_y,parent_.mypid.velocity_z);
 		printf("当前位置为（ %lf , %lf , %lf ）\n",parent_.get_x_pos(),parent_.get_y_pos(),parent_.get_z_pos());
 		printf("目标位置为 （ %lf , %lf , %lf ）\n",parent_.dx_shot,parent_.dy_shot,parent_.shot_halt);
 		printf("PID输出分别为：（ %lf , %lf ,%lf）\n",parent_.mypid.compute(parent_.dx_shot,parent_.get_x_pos(),0.01),parent_.mypid.compute(parent_.dy_shot,parent_.get_y_pos(),0.01),
                               parent_.mypid.compute(parent_.shot_halt,parent_.get_z_pos(),0.01));
 		
 		parent_.mypid.Mypid(parent_.dx_shot,parent_.dy_shot,parent_.shot_halt,parent_.get_x_pos(),parent_.get_y_pos(),parent_.get_z_pos(),0.01);
 		parent_.send_velocity_command(parent_.mypid.velocity_x,parent_.mypid.velocity_y,parent_.mypid.velocity_z,0);
 		//transition_to(FlyState::end);
 	}
 }


template<>
void OffboardControl::StateMachine::handle_state<FlyState::Reflush_config>() {
	if (current_state_ == FlyState::Reflush_config) {
		RCLCPP_INFO_ONCE(parent_.get_logger(), "开始刷新配置");
		// 读取配置文件
		parent_.read_configs("OffboardControl.yaml");
		parent_._pose_control->pid_x_defaults = PID::readPIDParameters("pos_config.yaml","pos_x");
		parent_._pose_control->pid_y_defaults = PID::readPIDParameters("pos_config.yaml","pos_y");
		parent_._pose_control->pid_z_defaults = PID::readPIDParameters("pos_config.yaml","pos_z");
		parent_._pose_control->pid_yaw_defaults = PID::readPIDParameters("pos_config.yaml","pos_yaw");
		parent_._pose_control->pid_px_defaults = PID::readPIDParameters("pos_config.yaml","pos_px");
		parent_._pose_control->pid_py_defaults = PID::readPIDParameters("pos_config.yaml","pos_py");
		parent_._pose_control->pid_pz_defaults = PID::readPIDParameters("pos_config.yaml","pos_pz");
		parent_._pose_control->pid_vx_defaults = PID::readPIDParameters("pos_config.yaml","pos_vx");
		parent_._pose_control->pid_vy_defaults = PID::readPIDParameters("pos_config.yaml","pos_vy");
		parent_._pose_control->pid_vz_defaults = PID::readPIDParameters("pos_config.yaml","pos_vz");
		parent_._pose_control->limit_defaults = parent_._pose_control->readLimits("pos_config.yaml","limits");
		// 重新设置PID参数
		parent_._pose_control->reset_pid();
		parent_._pose_control->set_limits(parent_._pose_control->limit_defaults);

		RCLCPP_INFO_ONCE(parent_.get_logger(), "配置刷新完成");
		transition_to(previous_state_); // 切换回上一个状态
	}
}

// 始终开启
template<>
void OffboardControl::StateMachine::handle_state<FlyState::Termial_Control>() {
  char key = 0;
	static std::string input = "";

  if (_kbhit()) // 检查是否有按键输入
	{
		key = _getch(); // 获取按键输入
		// 特殊状态处理（起飞解锁前）
		if (parent_._motors->state_ == (Motors::State::wait_for_takeoff_command)){
			if (key == '\n') // 检查是否按下回车键
			{
				parent_._motors->takeoff_command = true; // 设置起飞命令
			}
			else if (key == 'q') // 检查是否按下q键
			{
				RCLCPP_INFO(parent_.get_logger(), "退出程序");
				transition_to(FlyState::end); // 切换到结束状态
			}
			else if (key != 0)
			{
				RCLCPP_INFO(parent_.get_logger(), "无效输入，请按回车键解锁无人机或按q键退出程序");
			}
			return;
		}
    // 处理多字符输入
    if (key == '\n' || key == '\r') { // 按下回车，尝试解析命令
      std::string upperInput = input;
      std::transform(upperInput.begin(), upperInput.end(), upperInput.begin(), ::toupper);
      auto it = FlyStateMap.find(upperInput);
      if (it != FlyStateMap.end()) {
        transition_to(it->second);
        RCLCPP_INFO(parent_.get_logger(), "切换到状态: %s", upperInput.c_str());
      } else {
        RCLCPP_INFO(parent_.get_logger(), "无效指令: %s", input.c_str());
      }
      input.clear();
    } else if (key == '\b' && !input.empty()) { // 处理退格
      input.pop_back();
		} else if (key == '\t') { // Tab键自动补全
			std::string upperInput = input;
			std::transform(upperInput.begin(), upperInput.end(), upperInput.begin(), ::toupper);
			std::vector<std::string> candidates;
			for (const auto& kv : FlyStateMap) {
					if (kv.first.find(upperInput) == 0) { // 前缀匹配
							candidates.push_back(kv.first);
					}
			}
			if (candidates.size() == 1) {
					input = candidates[0];
					RCLCPP_INFO(parent_.get_logger(), "自动补全: %s", input.c_str());
			} else if (candidates.size() > 1) {
					std::string msg = "可选项: ";
					for (const auto& s : candidates) msg += s + " ";
					RCLCPP_INFO(parent_.get_logger(), "%s", msg.c_str());
			}
    } else if (key != 0) {
      input += key; // 将按键添加到输入字符串中
    }
	}
}

// 状态机动态任务管理
void OffboardControl::StateMachine::add_dynamic_task(std::function<void()> task) {
	std::lock_guard<std::mutex> lock(task_mutex_);
	dynamic_tasks_.emplace_back(std::move(task));
}

void OffboardControl::StateMachine::execute_dynamic_tasks() {
	std::vector<std::function<void()>> tasks;
	{
		std::lock_guard<std::mutex> lock(task_mutex_);
		tasks.swap(dynamic_tasks_);
	}
	for(const auto& task : tasks) {
		task();
	}
}

// 模板展开状态处理
template<FlyState... States>
void OffboardControl::StateMachine::process_states() {
    ((handle_state<States>()), ...);
}

// 状态转移
void OffboardControl::StateMachine::transition_to(FlyState new_state) {
	RCLCPP_INFO(parent_.get_logger(), "状态转换: %d -> %d", 
						 static_cast<int>(current_state_), 
						 static_cast<int>(new_state));

	parent_.state_timer_.reset(); // 重置状态计时器

	if (new_state == current_state_) {
		RCLCPP_INFO(parent_.get_logger(), "状态未改变，保持当前状态: %d", static_cast<int>(current_state_));
		return;
	}
	previous_state_ = current_state_;
	current_state_ = new_state;
}


// 发布状态
void OffboardControl::publish_current_state()
{
  // auto message = std_msgs::msg::String();
	auto message = std_msgs::msg::Int32();
  // message.data = "Current state: " + std::to_string(static_cast<int>(state_machine_.current_state_));
	message.data = fly_state_to_int(state_machine_.current_state_);
  state_publisher_->publish(message);
}