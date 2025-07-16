#include "StateMachine.h"
#include "OffboardControl.h"

// 具体状态处理实现
template<>
void StateMachine::handle_state<FlyState::init>() {
	if (current_state_ == FlyState::init) {
		owner_->FlyState_init();
		RCLCPP_INFO_ONCE(owner_->get_logger(), "初始化完成");
		transition_to(FlyState::takeoff);
	}
}

template<>
void StateMachine::handle_state<FlyState::takeoff>() {
	if (current_state_ == FlyState::takeoff){
		RCLCPP_INFO_ONCE(owner_->get_logger(), "开始起飞");

		if (owner_->_motors->takeoff(owner_->get_z_pos())) {
				RCLCPP_INFO_ONCE(owner_->get_logger(), "起飞成功");
				transition_to(FlyState::Goto_shotpoint);
		} else {
				// RCLCPP_INFO(owner_->get_logger(), "起飞失败");
		}
	}
}

template<>
void StateMachine::handle_state<FlyState::end>() {
    if (current_state_ == FlyState::end) {
        RCLCPP_INFO_ONCE(owner_->get_logger(), "任务结束");
        // 如果需要，可以在这里添加清理或退出逻辑
        rclcpp::shutdown();  // 停止 ROS 2 节点
    }
}

template<>
void StateMachine::handle_state<FlyState::Goto_shotpoint>() {
	if (current_state_ == FlyState::Goto_shotpoint) {
		RCLCPP_INFO_ONCE(owner_->get_logger(), "开始前往投弹区起点");
		float x_shot, y_shot;
		owner_->rotate_global2stand(owner_->dx_shot, owner_->dy_shot, x_shot, y_shot);
		if(owner_->state_timer_.elapsed() > 12)
		{
			owner_->state_timer_.set_start_time_to_default();
			RCLCPP_INFO(owner_->get_logger(), "到达投弹区起点");
			transition_to(FlyState::Doshot);
		} else {
			owner_->send_local_setpoint_command(
				x_shot, y_shot, owner_->shot_halt, 0
			);
			RCLCPP_INFO_THROTTLE(owner_->get_logger(), *owner_->get_clock(), 3000, "前往投弹区中...%f", owner_->state_timer_.elapsed());
		}
	}
}


template<>
void StateMachine::handle_state<FlyState::Doshot>() {
	if (current_state_ == FlyState::Doshot) {
		RCLCPP_INFO_ONCE(owner_->get_logger(), "执行投弹任务Doshot");
		static Timer doshot_start = Timer();  // 全程计时器
		static int counter = 0, pre_counter; // 航点计数器
		static double doshot_halt_end_time; // 记录结束时间
		static int shot_counter = 1; // 投弹计数器
		static vector<array<double, 3>> surround_shot_scout_points;

		bool timeout = doshot_start.elapsed() > 100; // 超时 60 秒

		if (timeout) // 超时 60 秒
		{
			RCLCPP_INFO(owner_->get_logger(), "超时");
			owner_->doshot_state_ = owner_->DoshotState::doshot_end; // 设置投弹状态为结束
		} 
		while(true){
			switch (owner_->doshot_state_)  // 根据投弹状态执行不同的操作
			{
			case owner_->DoshotState::doshot_init: // 初始化投弹状态
				{
					RCLCPP_INFO(owner_->get_logger(), "开始投弹任务");
					surround_shot_scout_points = {
						{owner_->dx_shot + 2.4, owner_->dy_shot + 1.3, 4},
						{owner_->dx_shot + 2.4, owner_->dy_shot + 3.7, 4},
						{owner_->dx_shot - 2.4, owner_->dy_shot + 3.7, 4},
						{owner_->dx_shot - 2.4, owner_->dy_shot + 1.3, 4},
					};
					owner_->doshot_state_ = owner_->DoshotState::doshot_scout; // 设置投弹状态为侦查
					doshot_start.reset(); // 重置计时器
					counter = 0; // 重置计数器
					shot_counter = 1; // 重置投弹计数器
				}
				continue;
			case owner_->DoshotState::doshot_scout: // 侦查投弹区
				RCLCPP_INFO_ONCE(owner_->get_logger(), "开始侦查投弹区");
				if (!surround_shot_scout_points.empty()) {
					if (owner_->trajectory_generator_world_points(
						1, surround_shot_scout_points, surround_shot_scout_points.size(),
						{1.7, 1.7, 1.7}, {0.15, 0.15, 0.15} // 设置最大速度和加速度
					)) {
					// if (owner_->waypoint_goto_next(
					// 	owner_->dx_shot, owner_->dy_shot, owner_->shot_length, owner_->shot_width, 
					// 	owner_->shot_halt, surround_shot_scout_points, 4.0, &counter, "侦查投弹区"))
					// {
						
						owner_->doshot_state_ = owner_->DoshotState::doshot_halt; // 设置投弹状态为侦查完成
						owner_->state_timer_.reset();
					}
				} else {
					RCLCPP_WARN(owner_->get_logger(), "surround_shot_scout_points为空，跳转到doshot_init");
					owner_->doshot_state_ = owner_->DoshotState::doshot_halt;
				}
				break;
			case owner_->DoshotState::doshot_halt: // 投弹
				// RCLCPP_INFO(owner_->get_logger(), "收到坐标c(%f, %f) h(%f ,%f), flag_servo = %d",
				// 	owner_->_yolo->get_x(YOLO::TARGET_TYPE::CIRCLE), owner_->_yolo->get_y(YOLO::TARGET_TYPE::CIRCLE),
				// 	owner_->_yolo->get_x(YOLO::TARGET_TYPE::H), owner_->_yolo->get_y(YOLO::TARGET_TYPE::H),
				// 	owner_->_yolo->get_servo_flag());
				if (owner_->Doshot(shot_counter)) { // 如果到达投弹点
					// RCLCPP_INFO(owner_->get_logger(), "寻找完毕，投弹!!投弹!!");
					RCLCPP_INFO(owner_->get_logger(), "投弹!!投弹!!，总用时：%f", doshot_start.elapsed());
					RCLCPP_INFO(owner_->get_logger(), "Arrive, 投弹 等待5秒");
					// 设置舵机位置
					// owner_->_servo_controller->set_servo(10 + shot_counter, 1864); // 舵机闭合
					owner_->doshot_state_ = owner_->DoshotState::doshot_end; // 设置投弹状态为结束
					doshot_halt_end_time = owner_->get_cur_time(); // 记录结束时间
				} else 
				if (!owner_->_yolo->is_get_target(YOLO::TARGET_TYPE::CIRCLE)) { // 如果没有找到投弹目标
					pre_counter = counter; // 记录上一次的计数器值
					owner_->waypoint_goto_next(
						owner_->dx_shot, owner_->dy_shot, owner_->shot_length, owner_->shot_width, 
						owner_->shot_halt, owner_->surround_shot_points, 3, &counter, "投弹区");
					// RCLCPP_INFO(owner_->get_logger(), "投弹区航点计数器：%d", counter);
				} else { // 如果找到投弹目标但未到达目标上方
					counter = pre_counter; // 恢复上一次的计数器值
					owner_->state_timer_.reset(); // 重置航点计时器
				}
				break;
			case owner_->DoshotState::doshot_end: // 侦查投弹区
				// RCLCPP_INFO(owner_->get_logger(), "投弹!!投弹!!，总用时：%f", doshot_start.elapsed());
				// RCLCPP_INFO(owner_->get_logger(), "Arrive, 投弹 等待5秒");
				// 设置舵机位置+
				owner_->_servo_controller->set_servo(10 + shot_counter, 1200);
				if(shot_counter <= 1 && !timeout) // 投弹计数器小于1，再次执行投弹
				{
					if (owner_->get_cur_time() - doshot_halt_end_time < 0.5) { // 等待1秒
						owner_->_pose_control->send_velocity_command_world(0, 0, 0, 0); // 停止飞行
						RCLCPP_INFO(owner_->get_logger(), "等待0.5秒，准备投弹");
					}
					owner_->waypoint_goto_next(
						owner_->dx_shot, owner_->dy_shot, owner_->shot_length, owner_->shot_width, 
						owner_->shot_halt, owner_->surround_shot_points, owner_->shot_halt, &counter, "投弹区");
					if (owner_->get_cur_time() - doshot_halt_end_time < 5.0 || counter == pre_counter) {   // 非阻塞等待至第5秒或抵达下一个航点
						break;
					}
					RCLCPP_INFO(owner_->get_logger(), "投弹完成，继续投弹 shot_counter=%d", shot_counter);
					shot_counter++;
					owner_->doshot_state_ = owner_->DoshotState::doshot_halt;
					owner_->state_timer_.reset(); // 重置航点计时器
					continue; // 继续投弹
				}
				owner_->_servo_controller->set_servo(11, 1864);
				owner_->_servo_controller->set_servo(12, 1864);
				// 重置状态
				owner_->doshot_state_ = owner_->DoshotState::doshot_init; // 重置投弹状态
				doshot_start.set_start_time_to_default();
				RCLCPP_INFO(owner_->get_logger(), "投弹完成，1s后前往侦查区域");
				rclcpp::sleep_for(1s);
				transition_to(FlyState::Goto_scoutpoint);
				break;
			default:
				break;
			}
			break; // 跳出 while 循环
		}
	}
	return; // 结束函数
}

template<>
void StateMachine::handle_state<FlyState::Goto_scoutpoint>() {
	if (current_state_ == FlyState::Goto_scoutpoint) {
		RCLCPP_INFO_ONCE(owner_->get_logger(), "开始前往侦查起点");
		float x_see, y_see;
		owner_->rotate_global2stand(owner_->dx_see, owner_->dy_see, x_see, y_see);
		if(owner_->state_timer_.elapsed() > 10)
		{
			owner_->state_timer_.set_start_time_to_default();
			RCLCPP_INFO(owner_->get_logger(), "到达侦查区起点");
			transition_to(FlyState::Surround_see);
		} else {
			owner_->send_local_setpoint_command(
				x_see, y_see, owner_->see_halt, 0
			);
			RCLCPP_INFO_THROTTLE(owner_->get_logger(), *owner_->get_clock(), 3000, "前往侦查区中...%f", owner_->state_timer_.elapsed());
		}
	}
}

template<>
void StateMachine::handle_state<FlyState::Surround_see>() {
	if (current_state_ == FlyState::Surround_see) {
		static int counter = 0; // 航点计数器
		if (owner_->waypoint_goto_next(
			owner_->dx_see, owner_->dy_see, owner_->see_length, owner_->see_width, 
			owner_->see_halt, owner_->surround_see_points, 3.5, &counter, "侦查区"))
		{
			RCLCPP_INFO_ONCE(owner_->get_logger(), "侦查完毕");
			counter = 0;
			owner_->_motors->switch_mode("RTL");
			rclcpp::sleep_for(std::chrono::seconds(17));
			owner_->_motors->switch_mode("GUIDED");
			transition_to(FlyState::Doland);
		}
	}
}

template<>
void StateMachine::handle_state<FlyState::Doland>() {
	if (current_state_ == FlyState::Doland) {
		if(owner_->Doland()){
			owner_->_motors->switch_mode("LAND");
			transition_to(FlyState::end);
		}
	}
}

template<>
void StateMachine::handle_state<FlyState::Print_Info>() {
	if (current_state_ == FlyState::Print_Info
	) {
		static unsigned short print_count = 0;
		print_count++; // 先递增
		print_count = print_count % 3; // 然后取模
		if (print_count != 0) {
			return; // 每10次打印一次
		}
		std::stringstream ss;
		// 打印当前状态信息
		ss << "--------timer_callback----------" << std::endl;
		ss << "px:  " << std::setw(10) << owner_->get_x_pos() << ", py: " << std::setw(10) << owner_->get_y_pos() << ", pz: " << std::setw(10) << owner_->get_z_pos() << std::endl;
		ss << "vx:  " << std::setw(10) << owner_->get_x_vel() << ", vy: " << std::setw(10) << owner_->get_y_vel() << ", vz: " << std::setw(10) << owner_->get_z_vel() << std::endl;
		ss << "yaw: " << owner_->get_yaw() << std::endl;
		ss << "yaw_e: " << owner_->get_yaw_eigen() << std::endl;
		ss << "yaw_vel: " << owner_->get_yaw_vel() << std::endl;
		ss << "lat: " << owner_->get_lat() << ", lon: " << owner_->get_lon() << ", alt: " << owner_->get_alt() << std::endl;
		ss << "rangefinder_distance:  " << owner_->get_rangefinder_distance() << std::endl;
		ss << "armed:     " << owner_->get_armed() << std::endl;
		ss << "connected: " << owner_->get_connected() << std::endl;
		ss << "guided:	" << owner_->get_guided() << std::endl;
		ss << "mode:	  " << owner_->get_mode() << std::endl;
		ss << "system_status:  " << owner_->get_system_status() << std::endl;
		ss << "z_home_pos:     " << owner_->get_z_home_pos() << std::endl;
		ss << "running_time: " << owner_->get_cur_time() << std::endl;
		RCLCPP_INFO_STREAM(owner_->get_logger(), ss.str());
		ss.str(""); // 清空字符串流
		ss.clear(); // 清除状态标志
	}
}

template<>
 void StateMachine::handle_state<FlyState::MYPID>() {
 	if (current_state_ == FlyState::MYPID
 	) {
 		owner_->mypid.readPIDParameters("pos_config.yaml","mypid");
 		owner_->dx_shot = owner_->mypid.read_goal("OffboardControl.yaml","dx_shot");
 	    owner_->dy_shot = owner_->mypid.read_goal("OffboardControl.yaml","dy_shot");
 		owner_->shot_halt = owner_->mypid.read_goal("OffboardControl.yaml","shot_halt");
 		printf("已进入MYPID状态,当前kp ki kd参数分别为：%lf %lf %lf,输出限制为： %lf,积分限制为： %lf\n",owner_->mypid.kp_,owner_->mypid.ki_,owner_->mypid.kd_,owner_->mypid.output_limit_,owner_->mypid.integral_limit);
 
 		owner_->mypid.velocity_x = owner_->get_x_vel();
         owner_->mypid.velocity_y = owner_->get_y_vel();
 		owner_->mypid.velocity_z = owner_->get_z_vel();
 
 		printf("当前速度分别为：vx: %lf vy: %lf vz:%lf\n",owner_->mypid.velocity_x,owner_->mypid.velocity_y,owner_->mypid.velocity_z);
 		printf("当前位置为（ %lf , %lf , %lf ）\n",owner_->get_x_pos(),owner_->get_y_pos(),owner_->get_z_pos());
 		printf("目标位置为 （ %lf , %lf , %lf ）\n",owner_->dx_shot,owner_->dy_shot,owner_->shot_halt);
 		printf("PID输出分别为：（ %lf , %lf ,%lf）\n",owner_->mypid.compute(owner_->dx_shot,owner_->get_x_pos(),0.01),owner_->mypid.compute(owner_->dy_shot,owner_->get_y_pos(),0.01),
                               owner_->mypid.compute(owner_->shot_halt,owner_->get_z_pos(),0.01));
 		
 		owner_->mypid.Mypid(owner_->dx_shot,owner_->dy_shot,owner_->shot_halt,owner_->get_x_pos(),owner_->get_y_pos(),owner_->get_z_pos(),0.01);
 		owner_->send_velocity_command(owner_->mypid.velocity_x,owner_->mypid.velocity_y,owner_->mypid.velocity_z,0);
 		//transition_to(FlyState::end);
 	}
 }


template<>
void StateMachine::handle_state<FlyState::Reflush_config>() {
	if (current_state_ == FlyState::Reflush_config) {
		RCLCPP_INFO_ONCE(owner_->get_logger(), "开始刷新配置");
		// 读取配置文件
		owner_->read_configs("OffboardControl.yaml");
		owner_->_pose_control->pid_x_defaults = PID::readPIDParameters("pos_config.yaml","pos_x");
		owner_->_pose_control->pid_y_defaults = PID::readPIDParameters("pos_config.yaml","pos_y");
		owner_->_pose_control->pid_z_defaults = PID::readPIDParameters("pos_config.yaml","pos_z");
		owner_->_pose_control->pid_yaw_defaults = PID::readPIDParameters("pos_config.yaml","pos_yaw");
		owner_->_pose_control->pid_px_defaults = PID::readPIDParameters("pos_config.yaml","pos_px");
		owner_->_pose_control->pid_py_defaults = PID::readPIDParameters("pos_config.yaml","pos_py");
		owner_->_pose_control->pid_pz_defaults = PID::readPIDParameters("pos_config.yaml","pos_pz");
		owner_->_pose_control->pid_vx_defaults = PID::readPIDParameters("pos_config.yaml","pos_vx");
		owner_->_pose_control->pid_vy_defaults = PID::readPIDParameters("pos_config.yaml","pos_vy");
		owner_->_pose_control->pid_vz_defaults = PID::readPIDParameters("pos_config.yaml","pos_vz");
		owner_->_pose_control->limit_defaults = owner_->_pose_control->readLimits("pos_config.yaml","limits");
		// 重新设置PID参数
		owner_->_pose_control->reset_pid();
		owner_->_pose_control->set_limits(owner_->_pose_control->limit_defaults);

		RCLCPP_INFO_ONCE(owner_->get_logger(), "配置刷新完成");
		transition_to(previous_state_); // 切换回上一个状态
	}
}

// 始终开启
template<>
void StateMachine::handle_state<FlyState::Termial_Control>() {
  char key = 0;
	static std::string input = "";

  if (_kbhit()) // 检查是否有按键输入
	{
		key = _getch(); // 获取按键输入
		// 特殊状态处理（起飞解锁前）
		if (owner_->_motors->state_ == (Motors::State::wait_for_takeoff_command)){
			if (key == '\n') // 检查是否按下回车键
			{
				owner_->_motors->takeoff_command = true; // 设置起飞命令
			}
			else if (key == 'q') // 检查是否按下q键
			{
				RCLCPP_INFO(owner_->get_logger(), "退出程序");
				transition_to(FlyState::end); // 切换到结束状态
			}
			else if (key != 0)
			{
				RCLCPP_INFO(owner_->get_logger(), "无效输入，请按回车键解锁无人机或按q键退出程序");
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
        RCLCPP_INFO(owner_->get_logger(), "切换到状态: %s", upperInput.c_str());
      } else {
        RCLCPP_INFO(owner_->get_logger(), "无效指令: %s", input.c_str());
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
					RCLCPP_INFO(owner_->get_logger(), "自动补全: %s", input.c_str());
			} else if (candidates.size() > 1) {
					std::string msg = "可选项: ";
					for (const auto& s : candidates) msg += s + " ";
					RCLCPP_INFO(owner_->get_logger(), "%s", msg.c_str());
			}
    } else if (key != 0) {
      input += key; // 将按键添加到输入字符串中
    }
	}
}

// transition_to函数实现
void StateMachine::transition_to(FlyState new_state) {
	RCLCPP_INFO(owner_->get_logger(), "状态转换: %d -> %d", 
						 static_cast<int>(current_state_), 
						 static_cast<int>(new_state));

	owner_->state_timer_.reset(); // 重置状态计时器
	if (new_state == current_state_) {
		RCLCPP_INFO(owner_->get_logger(), "状态未改变，保持当前状态: %d", static_cast<int>(current_state_));
		return;
	}
	previous_state_ = current_state_;
	current_state_ = new_state;
}
