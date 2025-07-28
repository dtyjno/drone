#include "StateMachine.h"
#include "OffboardControl.h"

// 具体状态处理实现
template<>
void StateMachine::handle_state<FlyState::init>() {
	if (current_state_ == FlyState::init) {
		owner_->FlyState_init();
		RCLCPP_INFO_ONCE(owner_->get_logger(), "初始化完成");
		if (owner_->debug_mode_) {
			RCLCPP_INFO_ONCE(owner_->get_logger(), "测试模式下, 不进行起飞");
			transition_to(FlyState::Goto_shotpoint);
			return;
		}
		transition_to(FlyState::takeoff);
	}
}

template<>
void StateMachine::handle_state<FlyState::takeoff>() {
	if (current_state_ == FlyState::takeoff){
		RCLCPP_INFO_ONCE(owner_->get_logger(), "开始起飞");

		if (owner_->_motors->takeoff(owner_->get_z_pos(), 5.0f, owner_->get_yaw())) {
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
        RCLCPP_INFO_ONCE(owner_->get_logger(), "任务结束, 运行时间: %f 秒", owner_->get_cur_time());
        // 如果需要，可以在这里添加清理或退出逻辑
        rclcpp::shutdown();  // 停止 ROS 2 节点
    }
}

template<>
void StateMachine::handle_state<FlyState::Goto_shotpoint>() {
	if (current_state_ == FlyState::Goto_shotpoint) {
		RCLCPP_INFO_ONCE(owner_->get_logger(), "开始前往投弹区起点");
		float x_shot, y_shot;
		owner_->rotate_global2stand(owner_->dx_shot, owner_->dy_shot + owner_->shot_width_max / 2, x_shot, y_shot);
		if(owner_->waypoint_timer_.elapsed() > 12)
		{
			owner_->waypoint_timer_.set_start_time_to_default();
			RCLCPP_INFO(owner_->get_logger(), "到达投弹区起点");
			transition_to(FlyState::Doshot);
		} else {
			owner_->send_local_setpoint_command(
				x_shot, y_shot, owner_->shot_halt, 0
			);
			RCLCPP_INFO_THROTTLE(owner_->get_logger(), *owner_->get_clock(), 3000, "(THROTTLE 3s)前往投弹区中...%f", owner_->waypoint_timer_.elapsed());
		}
	}
}


template<>
void StateMachine::handle_state<FlyState::Doshot>() {
	if (current_state_ == FlyState::Doshot) {
		if (owner_->is_first_run_){ 
			owner_->doshot_state_ = owner_->DoshotState::doshot_init; // 设置投弹状态为初始化
			owner_->is_first_run_ = false; // 重置第一次运行标志
		}
		RCLCPP_INFO_ONCE(owner_->get_logger(), "执行投弹任务Doshot");
		static int counter = 0, pre_counter; // 航点计数器
		static float pre_time = 0.0f; // 上次航点时间
		static double doshot_halt_end_time; // 记录结束时间
		static int shot_counter = 1; // 投弹计数器
		static float max_accurate; // 聚类目标投弹最大距离
		static bool shot_flag = false; // 投弹标志

		// static vector<array<double, 3>> surround_shot_scout_points;

		if (owner_->state_timer_.elapsed() > 100 && owner_->doshot_state_ != owner_->DoshotState::doshot_end) // 超时 100 秒
		{
			doshot_halt_end_time = owner_->get_cur_time(); // 记录结束时间
			RCLCPP_INFO(owner_->get_logger(), "超时");
			owner_->doshot_state_ = owner_->DoshotState::doshot_end; // 设置投弹状态为结束
		}
		if ((static_cast<int>(owner_->cal_center.size()) > counter && counter != pre_counter) || owner_->doshot_state_ == owner_->DoshotState::doshot_wait || owner_->doshot_state_ == owner_->DoshotState::doshot_init) {
			// max_accurate = owner_->cal_center[counter].diameters / 2; // 更新最大距离
			max_accurate = 0.05; // 更新最大距离
			// RCLCPP_INFO(owner_->get_logger(), "更新最大距离为: %f", max_accurate);
		}
		while(true){
			switch (owner_->doshot_state_)  // 根据投弹状态执行不同的操作
			{
			case owner_->DoshotState::doshot_init: // 初始化投弹状态
				{
					RCLCPP_INFO(owner_->get_logger(), "开始投弹任务");
					// surround_shot_scout_points = {
					// 	{owner_->dx_shot + 2.4, owner_->dy_shot + 1.3, 4.5},
					// 	{owner_->dx_shot + 2.4, owner_->dy_shot + 3.7, 4.5},
					// 	{owner_->dx_shot - 2.4, owner_->dy_shot + 3.7, 4.5},
					// 	{owner_->dx_shot - 2.4, owner_->dy_shot + 1.3, 4.5},
					// };
					owner_->doshot_state_ = owner_->DoshotState::doshot_shot; // 设置投弹状态为侦查
					pre_counter = 0;
					counter = 0; // 重置计数器
					shot_counter = 1; // 重置投弹计数器

					owner_->waypoint_timer_.reset();

				}
				continue;
			// case owner_->DoshotState::doshot_scout: // 侦查投弹区
			// 	RCLCPP_INFO_ONCE(owner_->get_logger(), "开始侦查投弹区");
			// 	if (!surround_shot_scout_points.empty()) {
			// 		if (owner_->trajectory_generator_world_points(
			// 			1, surround_shot_scout_points, surround_shot_scout_points.size(),
			// 			{1.6, 1.6, 1.6}, {0.14, 0.14, 0.14} // 设置最大速度和加速度
			// 		)) {
			// 		// if (owner_->waypoint_goto_next(
			// 		// 	owner_->dx_shot, owner_->dy_shot, owner_->shot_length, owner_->shot_width, 
			// 		// 	owner_->shot_halt, surround_shot_scout_points, 4.0, &counter, "侦查投弹区"))
			// 		// {
						
			// 			owner_->doshot_state_ = owner_->DoshotState::doshot_shot; // 设置投弹状态为侦查完成
			// 			owner_->waypoint_timer_.reset();
			// 		}
			// 	} else {
			// 		RCLCPP_WARN(owner_->get_logger(), "surround_shot_scout_points为空，跳转到doshot_init");
			// 		owner_->doshot_state_ = owner_->DoshotState::doshot_shot;
			// 	}
			// 	break;
			case owner_->DoshotState::doshot_shot: // 投弹
				// RCLCPP_INFO(owner_->get_logger(), "收到坐标c(%f, %f) h(%f ,%f), flag_servo = %d",
				// 	owner_->_yolo->get_x(YOLO::TARGET_TYPE::CIRCLE), owner_->_yolo->get_y(YOLO::TARGET_TYPE::CIRCLE),
				// 	owner_->_yolo->get_x(YOLO::TARGET_TYPE::H), owner_->_yolo->get_y(YOLO::TARGET_TYPE::H),
				// 	owner_->_yolo->get_servo_flag());
				RCLCPP_INFO_THROTTLE(owner_->get_logger(), *owner_->get_clock(), 1000, "handle_state<Doshot>:(THROTTLE 1s) counter=%d shot_counter=%d x:%f, y:%f max:%f", counter, shot_counter,
					abs(owner_->_yolo->get_x(YOLO::TARGET_TYPE::CIRCLE) - owner_->_yolo->get_cap_frame_width()/2), abs(owner_->_yolo->get_y(YOLO::TARGET_TYPE::CIRCLE) - owner_->_yolo->get_cap_frame_height()/2), max_accurate);
				if (!shot_flag && static_cast<size_t>(counter) < owner_->cal_center.size() && (
						owner_->waypoint_timer_.elapsed() < 4.5 || (
							owner_->waypoint_timer_.elapsed() < 10.0 && (
							abs(owner_->get_x_pos() - owner_->cal_center[counter].point.x()) > max_accurate && 
							abs(owner_->get_y_pos() - owner_->cal_center[counter].point.y()) > max_accurate
							)
						)
					)
				) {
					double tx, ty;
					owner_->rotate_stand2global(owner_->cal_center[counter].point.x(), owner_->cal_center[counter].point.y(), tx, ty);
					if (tx < owner_->dx_shot - owner_->shot_length_max / 2 + 1.0 || tx > owner_->dx_shot + owner_->shot_length_max / 2 - 1.0 ||
						ty < owner_->dy_shot - 1.5 || ty > owner_->dy_shot + owner_->shot_width_max + 1.5) {
						RCLCPP_WARN(owner_->get_logger(), "侦查点坐标异常，跳过: %d, x: %f, y: %f", counter, tx, ty);
						counter++;
						pre_counter = counter;
						continue; // 跳过无效坐标
					}
					pre_counter = counter; // 记录上一次的计数器值
					pre_time = owner_->get_cur_time(); // 记录上一次的时间
					owner_->send_local_setpoint_command(
						owner_->cal_center[counter].point.x(),
						owner_->cal_center[counter].point.y(),
						owner_->shot_halt_low, 0
					);
					RCLCPP_INFO_THROTTLE(owner_->get_logger(), *owner_->get_clock(), 500, "handle_state<Doshot>:(THROTTLE 0.5s) 已经确认直径为%f的%d号桶，位置为（%f,%f）, 执行航点时间%f秒，x轴偏差%f, y轴偏差%f，最大距离为%f",
						owner_->cal_center[counter].diameters,
						counter, owner_->cal_center[counter].point.x(), owner_->cal_center[counter].point.y(),
						owner_->waypoint_timer_.elapsed(),
						abs(owner_->get_x_pos() - owner_->cal_center[counter].point.x()),
						abs(owner_->get_y_pos() - owner_->cal_center[counter].point.y()),
						max_accurate);
				} else if (!shot_flag && owner_->fast_mode_) { // 快速投弹
					RCLCPP_INFO(owner_->get_logger(), "fast_mode_ is true, 投弹");
					owner_->_servo_controller->set_servo(10 + shot_counter, 1864);
					owner_->waypoint_timer_.reset();
					owner_->doshot_state_ = owner_->DoshotState::doshot_wait; // 设置投弹状态为等待
					doshot_halt_end_time = owner_->get_cur_time(); // 记录结束时间
					continue; // 直接跳到下一个状态;
				} else if (!shot_flag && !owner_->_yolo->is_get_target(YOLO::TARGET_TYPE::CIRCLE)) { // 未找到圆
					pre_counter = counter; // 记录上一次的计数器值
					pre_time = owner_->get_cur_time(); // 记录上一次的时间
					owner_->waypoint_goto_next(
						owner_->dx_shot, owner_->dy_shot, owner_->shot_length, owner_->shot_width, 
						owner_->shot_halt, owner_->surround_shot_points, 3, &counter, "投弹区");
				} else if (owner_->Doshot(shot_counter, shot_flag)) { // 如果到达投弹点
					// RCLCPP_INFO(owner_->get_logger(), "寻找完毕，投弹!!投弹!!");
					RCLCPP_INFO(owner_->get_logger(), "已经锁定%d号桶，坐标为（%f,%f）", shot_counter, owner_->_yolo->get_x(YOLO::TARGET_TYPE::CIRCLE), owner_->_yolo->get_y(YOLO::TARGET_TYPE::CIRCLE));
					RCLCPP_INFO(owner_->get_logger(), "投弹!!投弹!!，总用时：%f", owner_->state_timer_.elapsed());
					owner_->doshot_state_ = owner_->DoshotState::doshot_wait; // 设置投弹状态为结束
					doshot_halt_end_time = owner_->get_cur_time(); // 记录结束时间
					continue; // 继续执行下一次循环
				} else { // 如果找到投弹目标但未到达目标上方
					max_accurate = 5;
					counter = pre_counter; // 恢复上一次的计数器值
					owner_->waypoint_timer_.set_start_time_to_time_point(pre_time); // 重置航点计时器
				}
				break;
			case owner_->DoshotState::doshot_wait: // 等待再次投弹
				if(shot_counter <= 1) // 投弹计数器小于1，再次执行投弹
				{
					if (static_cast<size_t>(counter) >= owner_->cal_center.size()){
						owner_->waypoint_goto_next(
							owner_->dx_shot, owner_->dy_shot, owner_->shot_length, owner_->shot_width, 
							owner_->shot_halt, owner_->surround_shot_points, owner_->shot_halt, &counter, "投弹区");
						if (owner_->get_cur_time() - doshot_halt_end_time < 5.0 || counter == pre_counter + 1) {   // 非阻塞等待至第5秒或抵达下一个航点
							break;
						}
					} else {
						if (owner_->get_cur_time() - doshot_halt_end_time < 2.0){
							break; // 等待2秒
						}
						counter++; // 增加计数器
						pre_counter = counter; // 更新上一次计数器值
					}
					RCLCPP_INFO(owner_->get_logger(), "投弹完成，继续投弹 shot_counter=%d", shot_counter);
					shot_counter++;
					owner_->doshot_state_ = owner_->DoshotState::doshot_shot; // 设置投弹状态为投弹
					owner_->waypoint_timer_.reset(); // 重置航点计时器
					doshot_halt_end_time = owner_->get_cur_time(); // 记录结束时间
					continue; // 继续投弹
				} else {
					owner_->doshot_state_ = owner_->DoshotState::doshot_end; // 设置投弹状态为结束
					continue; // 继续执行下一次循环
				}
				break;
			case owner_->DoshotState::doshot_end: // 侦查投弹区
				if (owner_->get_cur_time() - doshot_halt_end_time < 2.0) {
					if (owner_->get_cur_time() - doshot_halt_end_time < owner_->get_wait_time()) {
						RCLCPP_INFO_THROTTLE(owner_->get_logger(), *owner_->get_clock(), 1000, "投弹完成，等待2秒后前往侦查区域");
						owner_->_servo_controller->set_servo(11, 1864);			// owner_->_servo_controller->set_servo(11, 1200);
						owner_->_servo_controller->set_servo(12, 1864);				// owner_->_servo_controller->set_servo(12, 1200);
					}
					break; // 等待2秒
				}
				// 重置状态
				owner_->doshot_state_ = owner_->DoshotState::doshot_init; // 重置投弹状态
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
		if(owner_->waypoint_timer_.elapsed() > 10)
		{
			owner_->waypoint_timer_.set_start_time_to_default();
			RCLCPP_INFO(owner_->get_logger(), "到达侦查区起点");
			transition_to(FlyState::Surround_see);
		} else {
			owner_->send_local_setpoint_command(
				x_see, y_see, owner_->see_halt, 0
			);
			RCLCPP_INFO_THROTTLE(owner_->get_logger(), *owner_->get_clock(), 3000, "(THROTTLE 3s)前往侦查区中...%f", owner_->waypoint_timer_.elapsed());
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
			// owner_->_motors->switch_mode("RTL");
			// rclcpp::sleep_for(std::chrono::seconds(17));
			// owner_->_motors->switch_mode("GUIDED");
			transition_to(FlyState::Doland);
		}
	}
}

template<>
void StateMachine::handle_state<FlyState::Doland>() {
	if (current_state_ == FlyState::Doland) {
		static enum class DolandState {
			doland_init, // 降落初始化
			doland_wait, // 等待降落
			doland_landing, // 降落中
			doland_end // 降落结束
		} doland_state = DolandState::doland_init; // 降落状态
		if (owner_->is_first_run_) {
			doland_state = DolandState::doland_init; // 重置降落状态
			owner_->is_first_run_ = false; // 重置第一次运行标志
		}
		while (true){
			switch (doland_state) {
			case DolandState::doland_init: // 降落初始化
				RCLCPP_INFO(owner_->get_logger(), "开始降落");
				owner_->_motors->switch_mode("RTL");
				doland_state = DolandState::doland_wait; // 切换到等待降落状态
				continue; // 继续执行下一次循环;
			case DolandState::doland_wait: // 等待降落
				if (owner_->state_timer_.elapsed() > 17.5) { // 如果等待超过17.5秒
					RCLCPP_INFO(owner_->get_logger(), "等待降落超过17.5秒，开始降落");
					owner_->_motors->switch_mode("GUIDED");
					doland_state = DolandState::doland_landing; // 切换到降落中状态
					continue; // 继续执行下一次循环;
				} else {
					RCLCPP_INFO_THROTTLE(owner_->get_logger(), *owner_->get_clock(), 3000, "(THROTTLE 3s)等待降落中...%f", owner_->state_timer_.elapsed());
				}
				break;
			case DolandState::doland_landing: // 降落中
				if(owner_->Doland()){
					doland_state = DolandState::doland_end; // 切换到降落结束状态
					continue; // 继续执行下一次循环;
				}
				break; // 继续执行下一次循环;
			case DolandState::doland_end: // 降落结束
				RCLCPP_INFO(owner_->get_logger(), "降落完成");
				owner_->_motors->switch_mode("LAND");
				transition_to(FlyState::end);
				doland_state = DolandState::doland_init; // 重置降落状态
				break; // 结束函数
			default:
				break;
			}
			break;
		}
	}
	return; // 结束函数
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
		// ss << "yaw: " << owner_->get_yaw() << std::endl;
		// ss << "yaw_e: " << owner_->get_yaw_eigen() << std::endl;
		ss << "yaw_vel: " << owner_->get_yaw_vel() << std::endl;
		float roll, pitch, yaw;
		owner_->get_euler(roll, pitch, yaw);
		ss << "roll: " << roll << ", pitch: " << pitch << ", yaw: " << yaw << std::endl;
		ss << "lat: " << owner_->get_lat() << ", lon: " << owner_->get_lon() << ", alt: " << owner_->get_alt() << std::endl;
		ss << "rangefinder_distance:  " << owner_->get_rangefinder_distance() << std::endl;
		ss << "armed:     " << owner_->get_armed() << std::endl;
		ss << "connected: " << owner_->get_connected() << std::endl;
		ss << "guided:	" << owner_->get_guided() << std::endl;
		ss << "mode:	  " << owner_->get_mode() << std::endl;
		ss << "system_status:  " << owner_->get_system_status() << std::endl;
		ss << "x_home_pos:     " << owner_->get_x_home_pos() << ", y_home_pos: " << owner_->get_y_home_pos() << ", z_home_pos: " << owner_->get_z_home_pos() << std::endl;
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

	owner_->waypoint_timer_.reset(); // 重置航点计时器
	owner_->state_timer_.reset(); // 重置状态计时器
	owner_->is_first_run_ = true; // 重置第一次运行标志
	if (new_state == current_state_) {
		RCLCPP_INFO(owner_->get_logger(), "状态未改变，保持当前状态: %d", static_cast<int>(current_state_));
		return;
	}
	previous_state_ = current_state_;
	current_state_ = new_state;
}
