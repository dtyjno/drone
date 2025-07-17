#include "OffboardControl.h"
#include "math.h"
#include "clustering.h"
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
	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "当前时间：%f", get_cur_time());
	// if(!print_info_)
	// {
		// RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "收到坐标c(%f, %f) h(%f ,%f), flag_servo = %d yaw=%f",
		// _yolo->get_x(YOLO::TARGET_TYPE::CIRCLE), _yolo->get_y(YOLO::TARGET_TYPE::CIRCLE),
		// _yolo->get_x(YOLO::TARGET_TYPE::H), _yolo->get_y(YOLO::TARGET_TYPE::H),
		// _yolo->get_servo_flag(), get_yaw());
	// }
	// RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "当前飞机位置 x: %f y: %f z: %f yaw: %f",
	// 	get_x_pos(), get_y_pos(), get_z_pos(), get_yaw());

	// 桶1（1 -31） 2 (2 -32) 3 (-1 -33)
	CameraParams camera1;
	camera1.position = Vector3d(get_x_pos(), get_y_pos(), get_z_pos());
	
	// 相机坐标系：相对于飞机机体坐标系，向前旋转90度后垂直向下
	// 飞机偏航角 + 相机相对偏航角(90度) + 俯仰角(-90度向下)
	float roll, pitch, yaw;
	get_euler(roll, pitch, yaw);	
	// RCLCPP_INFO(this->get_logger(), "飞机当前滚转角: %f 弧度 (%f°), 俯仰角: %f 弧度 (%f°), 偏航角: %f 弧度 (%f°)",
	// 	roll, roll * 180.0 / M_PI, pitch, pitch * 180.0 / M_PI, yaw, yaw * 180.0 / M_PI);
	
	// 设置相机姿态：垂直向下看（pitch = -90°）
	camera1.rotation = Vector3d(roll, pitch - M_PI/2, yaw);  // roll=0, pitch=-90°(垂直向下), yaw=0
	
	// std::cout << "相机旋转角度: roll=" << camera1.rotation[0] << " pitch=" << camera1.rotation[1] << " (" << camera1.rotation[1] * 180.0/M_PI << "°) yaw=" << camera1.rotation[2] << std::endl;
	
	// 设置正确的相机内参
	camera1.fx = 360;  // 与配置文件一致
	camera1.fy = 360;  // 与配置文件一致
	camera1.cx = 320;  // 配置文件中的cx值
	camera1.cy = 240;  // 配置文件中的cy值
	camera1.width = 640;
	camera1.height = 480;
	
 	Vector2d image_point1(
		_yolo->get_raw_x(YOLO::TARGET_TYPE::CIRCLE),
		_yolo->get_raw_y(YOLO::TARGET_TYPE::CIRCLE)
	);
	
	// 调试输出：像素坐标和相机位置
	// std::cout << "检测到的像素坐标: (" << image_point1.x() << ", " << image_point1.y() << ")" << std::endl;
	// std::cout << "飞机当前位置: (" << get_x_pos() << ", " << get_y_pos() << ", " << get_z_pos() << ")" << std::endl;
	// std::cout << "相机旋转角度: roll=" << camera1.rotation[0] << " pitch=" << camera1.rotation[1] << " yaw=" << camera1.rotation[2] << std::endl;
	// std::cout << "相机内参: fx=" << camera1.fx << " fy=" << camera1.fy << " cx=" << camera1.cx << " cy=" << camera1.cy << std::endl;
	
	if (_yolo->is_get_target(YOLO::TARGET_TYPE::CIRCLE))
	{
		this->target1 = calculateWorldPosition(image_point1, camera1, 0.0, 0.3);
		if (target1) {
			RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "(THROTTLE 0.5s) Example 1 - Target position: %f, %f, %f",
				target1->x(), target1->y(), target1->z());
		}
	}
	if( doshot_state_ == DoshotState::doshot_scout)
	{
		Points Target;
		if(target1)
		{
			Target.point = *target1;
			Target.cluster_id = 0;
            Target_Samples.push_back(Target);
		    vector<Vector3d>cal_center = Clustering(Target_Samples);
			uint8_t shot_count = 0;
			for(size_t i = 0; i < cal_center.size(); ++i)
			{
				double tx, ty;
				rotate_stand2global(cal_center[i].x(), cal_center[i].y(), tx, ty);
				if (tx < dx_shot - 5 || tx > dx_shot + 5 ||
					ty < dy_shot || ty > dy_shot + 5) {
					RCLCPP_WARN(this->get_logger(), "侦查点坐标异常，跳过: %zu, x: %f, y: %f", i, cal_center[i].x(), cal_center[i].y());
					continue; // 跳过无效坐标
				}
				RCLCPP_INFO(this->get_logger(), "侦查点坐标 %zu: x: %f, y: %f ,n_x: %f, n_y: %f", 
					i, cal_center[i].x(), cal_center[i].y(), surround_shot_points[shot_count].x(), surround_shot_points[shot_count].y());
				surround_shot_points[shot_count++] = Vector2f((tx - dx_shot) / 10, (ty - dy_shot)/ 5);
				// for (size_t j = 0; j < surround_shot_points.size(); ++j)
				// {
				// 	RCLCPP_INFO(this->get_logger(), "侦查点坐标 %zu: x: %f, y: %f ", j, surround_shot_points[j].x(), surround_shot_points[j].y());
				// }
			}
		}
    }

	if (_motors->mode == "LAND" && state_machine_.get_current_state() != FlyState::end && !print_info_)
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
	// 发布当前状态
	publish_current_state();
	// 发布目标点
	_yolo->publish_visualization_target();
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
	rotate_global2stand(dx_shot, dy_shot, tx_shot, ty_shot);
	rotate_global2stand(dx_see, dy_see, tx_see, ty_see);
	RCLCPP_INFO(this->get_logger(), "默认方向下角度：%f", default_yaw);
	RCLCPP_INFO(this->get_logger(), "罗盘方向投弹区起点 x: %f   y: %f    angle: %f", tx_shot, ty_shot, default_yaw);
    RCLCPP_INFO(this->get_logger(), "罗盘方向侦查区起点 x: %f   y: %f    angle: %f", tx_see, ty_see, default_yaw);
	rotate_2start(tx_shot, ty_shot, x_shot, y_shot);
	rotate_2start(tx_see, ty_see, x_see, y_see);
	RCLCPP_INFO(this->get_logger(), "飞机起飞方向投弹区起点 x: %f   y: %f    angle: %f", x_shot, y_shot, default_yaw);
    RCLCPP_INFO(this->get_logger(), "飞机起飞方向侦查区起点 x: %f   y: %f    angle: %f", x_see, y_see, default_yaw);
	rotate_2local(tx_shot, ty_shot, x_shot, y_shot);
	rotate_2local(tx_see, ty_see, x_see, y_see);
	RCLCPP_INFO(this->get_logger(), "当前朝向投弹区起点 x: %f   y: %f    angle: %f", x_shot, y_shot, default_yaw);
    RCLCPP_INFO(this->get_logger(), "当前朝向侦查区起点 x: %f   y: %f    angle: %f", x_see, y_see, default_yaw);
		
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


	if (is_equal(get_x_pos(), DEFAULT_X_POS) || fabs(sqrt(_inav->orientation.w()*_inav->orientation.w() + _inav->orientation.x()*_inav->orientation.x() + _inav->orientation.y()*_inav->orientation.y() + _inav->orientation.z()*_inav->orientation.z()) - 1.0f) > 0.1f)
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
	_pose_control->set_dt(0.1); // 设置执行周期（用于PID）

	// 重新设置家地址
	_motors->set_home_position(get_yaw());

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


// 指定间隔时间循环执行航点
// x，y为中心位置，length，width为航点的长宽，halt为高度，way_points为航点集合，description为航点描述
bool OffboardControl::waypoint_goto_next(float x, float y, float length, float width, float halt, vector<Vector2f> &way_points, float time, int *count, const std::string &description)
{
	static std::vector<Vector2f>::size_type surround_cnt = 0; // 修改类型
	float x_temp = 0.0, y_temp = 0.0;
	int count_n = count == nullptr? surround_cnt : *count;
	if(count!=nullptr)
		RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "(THROTTLE 1s) w_g_n,counter: %d, time=%lf", *count, waypoint_timer_.elapsed());
	x_temp = x + (length * way_points[count_n].x());
	y_temp = y + (width * way_points[count_n].y());
	if (waypoint_timer_.elapsed() > time || (is_equal(get_x_pos(), x_temp, 0.2f) && is_equal(get_y_pos(), y_temp, 0.2f))) 
	{
		if (static_cast<std::vector<Vector2f>::size_type>(count_n) >= way_points.size())
		{
				RCLCPP_INFO(this->get_logger(), "w_g_n, %s已经全部遍历", description.c_str());
				count == nullptr? surround_cnt = 0 : *count = 0;
				// waypoint_timer_.reset();
				waypoint_timer_.set_start_time_to_default();
				return true;
		} else {
			count == nullptr? surround_cnt++ : (*count)++;
			RCLCPP_INFO(this->get_logger(), "w_g_n, %s点位%d x: %lf   y: %lf, timeout=%s", description.c_str(), count_n, x_temp, y_temp, (is_equal(get_x_pos(), x_temp, 0.2f) && is_equal(get_y_pos(), y_temp, 0.2f))? "true" : "false");

			rotate_global2stand(x_temp, y_temp, x_temp, y_temp);

			send_local_setpoint_command(x_temp, y_temp, halt, 0.0); // 发送本地坐标系下的航点指令
			// RCLCPP_INFO(this->get_logger(), "前往下一点");
			waypoint_timer_.reset();
		}
	}
	return false;
}

// 接近目标点
bool OffboardControl::catch_target(PID::Defaults defaults, enum YOLO::TARGET_TYPE target, float tar_x, float tar_y, float tar_z, float tar_yaw, float accuracy){
	// RCLCPP_INFO(this->get_logger(), "--------------------\n\n读取pid参数: p: %f, i: %f, d: %f, ff: %f, dff: %f, imax: %f", defaults.p, defaults.i, defaults.d, defaults.ff, defaults.dff, defaults.imax);d_max_xy: %f, speed_max_z: %f, accel_max_x: %f, accel_max_z: %f", limits.speed_max_xy, limits.speed_max_z, limits.accel_max_xy, limits.accel_max_z);
	// 检查YOLO帧尺寸是否有效
    // if (_yolo->get_cap_frame_width() <= 0 || _yolo->get_cap_frame_height() <= 0) {
    //     RCLCPP_ERROR(this->get_logger(), "Invalid YOLO frame dimensions: width=%d, height=%d", 
    //                  _yolo->get_cap_frame_width(), _yolo->get_cap_frame_height());
    //     return false;
    // }
	// yolo返回值坐标系：x右y下（x_flip|y_flip = false），转换为飞机坐标系：x右y上
	float now_x = _yolo->get_x(target);// _yolo->get_cap_frame_width() - _yolo->get_x(target);
	float now_y = _yolo->get_cap_frame_height() - _yolo->get_y(target); // _yolo->get_y(target);
	tar_x =  tar_x;// _yolo->get_cap_frame_width() - tar_x; // 目标x坐标
	tar_y =  _yolo->get_cap_frame_height() - tar_y;// tar_y; // 目标y坐标
	// 检查坐标是否有效
    // if (!std::isfinite(now_x) || !std::isfinite(now_y) || !std::isfinite(tar_x) || !std::isfinite(tar_y)) {
    //     RCLCPP_ERROR(this->get_logger(), "Invalid coordinates detected");
    //     return false;
    // }
	rotate_xy(now_x, now_y, get_yaw() + default_yaw); // 将目标坐标旋转到世界坐标系 headingangle_compass
	rotate_xy(tar_x, tar_y, get_yaw() + default_yaw); // 将目标坐标旋转到世界坐标系
	float max_frame = std::max(_yolo->get_cap_frame_width(), _yolo->get_cap_frame_height());
	// RCLCPP_INFO(this->get_logger(), "catch_target_bucket: yaw: %f, default_yaw: %f, headingangle_compass: %f", get_yaw(), default_yaw, headingangle_compass);
	// RCLCPP_INFO(this->get_logger(), "catch_target_bucket: now_x: %f, now_y: %f, tar_x: %f, tar_y: %f", now_x, now_y, tar_x, tar_y);
	// RCLCPP_INFO(this->get_logger(), "catch_target_bucket: now_x: %f, now_y: %f, tar_x: %f, tar_y: %f", now_x / _yolo->get_cap_frame_width(), now_y / _yolo->get_cap_frame_height(), (tar_x) / _yolo->get_cap_frame_width(), (tar_y) / _yolo->get_cap_frame_height());
	// RCLCPP_INFO(this->get_logger(), "catch_target_bucket: now_z: %f, tar_z: %f, now_yaw: %f, tar_yaw: %f", get_z_pos(), tar_z, get_yaw(), tar_yaw);
	return _pose_control->trajectory_setpoint_world(
		Vector4f{tar_x / max_frame, tar_y / max_frame, get_z_pos(), get_yaw()}, // 当前坐标
		Vector4f{now_x / max_frame, now_y / max_frame, tar_z, tar_yaw + default_yaw}, // 目标坐标
		defaults,
		true,
		accuracy
	);	
}

// 抵达桶上方
// if(识别到桶=Doshot（到达正上方）){[if(到达正上方==true){...}}
bool OffboardControl::Doshot(int shot_count)
{
	static enum class CatchState {
		init,
		fly_to_target,
		end
	} catch_state_ = CatchState::init;
	static double time_find_start = 0;			 		// 开始时间
	static float _t_time = 0; 					        // 接近目标时单轮执行时间
	static float accuracy = 0.1; 						// 声明精度
	static float shot_duration = 2; 					// 稳定持续时间
	static std::vector<YOLO::Target> targets; 			// 声明目标x和y坐标
	static float tar_z = 1, tar_yaw = 0; 				// 声明目标偏航角（rad）
	bool result = false;

	// 读取PID参数
	PID::Defaults defaults;
	defaults = PID::readPIDParameters("can_config.yaml", "pid");

	while(true){
		switch (catch_state_)
		{
		case CatchState::init:
		{
			RCLCPP_INFO(this->get_logger(), "Doshot: Init");
			PID::Defaults defaults;
			defaults = PID::readPIDParameters("can_config.yaml", "pid");
			PosControl::Limits_t limits = _pose_control->readLimits("can_config.yaml", "limits");
			_pose_control->set_limits(limits);
			// 读取距离目标一定范围内退出的距离
			YAML::Node config = Readyaml::readYAML("can_config.yaml");
			accuracy = config["accuracy"].as<float>();
			tar_z = config["tar_z"].as<float>();
			std::vector<std::string> targets_str = {"_l", "_r"};
			targets.clear();
			for (size_t i = 0; i < targets_str.size(); i++)
			{
				YOLO::Target target;
				target.x = config[std::string("tar_x").append(targets_str[i])].as<float>(0.0f);
				target.y = config[std::string("tar_y").append(targets_str[i])].as<float>(0.0f);
				target.z = config[std::string("tar_z").append(targets_str[i])].as<float>(0.0f);
				target.x = (is_equal(target.x, 0.0f) ? _yolo->get_cap_frame_width() / 2 : target.x);
				target.y = (is_equal(target.y, 0.0f) ? _yolo->get_cap_frame_height() / 2 : target.y);
				target.z = (is_equal(target.z, 0.0f) ? tar_z : target.z);
				target.r = 1.0f; 
				target.g = 0.0f;
				target.b = 0.0f;
				target.radius = accuracy * std::max(_yolo->get_cap_frame_width(), _yolo->get_cap_frame_height()) * 2;
				RCLCPP_INFO(this->get_logger(), "Doshot: cap_frame_width: %d, cap_frame_height: %d, radius: %f", 
					_yolo->get_cap_frame_width(), _yolo->get_cap_frame_height(), target.radius);
				target.category = config[std::string("tar_category").append(targets_str[i])].as<float>(1.0f);
				RCLCPP_INFO(this->get_logger(), "Doshot: tar_x: %f, tar_y: %f, tar_z: %f", target.x, target.y, target.z);
				targets.push_back(target);

			}

			time_find_start = get_cur_time();
			_t_time = time_find_start;
			tar_yaw = 0;			            // 设置目标偏航角（rad）
			catch_state_ = CatchState::fly_to_target;
			continue; // 继续执行下一个case;
		}
		case CatchState::fly_to_target:
		{
			double cur_shot_time = get_cur_time();
			int shot_index = (shot_count - 1) % targets.size(); // 计算当前投弹桶的索引，shot_count从1开始计数， tar_x.size()!=0
			
			for (size_t i = 0; i < targets.size(); i++)
			{
				targets[i].r = 1.0f; // 设置所有目标颜色为红色
				targets[i].g = 0.0f;
				targets[i].b = 0.0f;
			}
			targets[shot_index].r = 0.0f; // 设置当前目标颜色为绿色
			targets[shot_index].g = 1.0f;
			targets[shot_index].b = 0.0f;
			
			// yolo未识别到桶
			if (!_yolo->is_get_target(YOLO::TARGET_TYPE::CIRCLE))
			{
				// RCLCPP_INFO(this->get_logger(), "Doshot: yolo未识别到桶，等待");
				break;
			}
			if (catch_target(
					defaults,
					YOLO::TARGET_TYPE::CIRCLE, // 目标类型
					targets[shot_index].x, targets[shot_index].y, targets[shot_index].z, tar_yaw, accuracy
				))
			{
				RCLCPP_INFO(this->get_logger(), "Doshot: Approach, Doshot, time = %fs", cur_shot_time - _t_time);
				// if(error_x<0.05 && error_y<0.05){
					// RCLCPP_INFO(this->get_logger(), "Arrive, Doshot");
					// catch_state_=CatchState::end;
				// } else 
				if(cur_shot_time - _t_time > shot_duration){ // 1.5秒
					RCLCPP_INFO(this->get_logger(), "Doshot: Approach, Doshot, time > %fs, tar_x = %f, tar_y = %f, tar_z = %f, tar_yaw = %f", 
						shot_duration, targets[shot_index].x, targets[shot_index].y, targets[shot_index].z, tar_yaw);
					catch_state_=CatchState::end;
					continue; // 直接跳到下一个状态;
				}
			}
			else
			{
				_t_time = cur_shot_time;
			}
			break;
		}
		case CatchState::end:
		{
			RCLCPP_INFO(this->get_logger(), "Doshot: end");
			// 重置所有静态变量
			catch_state_ = CatchState::init;
			_pose_control->reset_limits();
			time_find_start = 0;
			result = true;
			break;
		}
		default:
			break;
		}
		break;
	}
	_yolo->append_targets(targets); // 将目标添加到YOLO中准备发布
	return result;
}

bool OffboardControl::Doland()
{
	static Timer timer_ = Timer();
	static enum class LandState {
		init,
		land_to_target,
		end
	} land_state_ = LandState::init;
	static int surround_land = -3;
	static PID::Defaults defaults;
	static double tar_x = 0.0, tar_y = 0.0, tar_z = 1.0, scout_x = 0.0, scout_y = 0.0, scout_halt = 3.0, accuracy = 0.3;
	double x_home, y_home;
	bool result = false;
	while(true){
		switch (land_state_)
		{
		case LandState::init:{
			// 读取PID参数
			defaults = PID::readPIDParameters("land_config.yaml", "pid");
			PosControl::Limits_t limits = _pose_control->readLimits("land_config.yaml", "limits");
			_pose_control->set_limits(limits);
			YAML::Node config = Readyaml::readYAML("land_config.yaml");
			tar_x = config["tar_x"].as<double>();
			tar_y = config["tar_y"].as<double>();
			tar_z = config["tar_z"].as<double>();
			scout_halt = config["scout_halt"].as<double>();
			scout_x = config["scout_x"].as<double>();
			scout_y = config["scout_y"].as<double>();
			accuracy = config["accuracy"].as<double>();
			tar_x = (is_equal(tar_x, 0.0) ? _yolo->get_cap_frame_width() / 2 : tar_x);
			tar_y = (is_equal(tar_y, 0.0) ? _yolo->get_cap_frame_height() / 2 : tar_y);
			RCLCPP_INFO(this->get_logger(), "Doland");
			rotate_global2stand(scout_x, scout_y, x_home, y_home);
			RCLCPP_INFO(this->get_logger(), "返回降落准备点 x: %lf   y: %lf    angle: %lf", x_home, y_home, headingangle_compass);
			send_local_setpoint_command(x_home, y_home, scout_halt, 0);
			// rclcpp::sleep_for(std::chrono::seconds(6));
			rotate_global2stand(scout_x, scout_y + 0.3, x_home, y_home);
			RCLCPP_INFO(this->get_logger(), "返回降落点 x: %lf   y: %lf    angle: %lf", x_home, y_home, 0.0);
			send_local_setpoint_command(x_home, y_home, scout_halt, 0);
			timer_.reset();
			timer_.set_timepoint();
			land_state_ = LandState::land_to_target;
			continue; // 直接跳到下一个状态;
		}
		case LandState::land_to_target:{
			if (timer_.elapsed() > 19 || surround_land > 3 || get_z_pos() < 0.5) // 降落时间超过19秒，或者降落高度小于0.5米
			{
				land_state_ = LandState::end;
				continue; // 直接跳到下一个状态;
			}
			if (!_yolo->is_get_target(YOLO::TARGET_TYPE::H)) // yolo未识别到YOLO::TARGET_TYPE::H   (YOLO::TARGET_TYPE::CIRCLE)
			{
				if (timer_.get_timepoint_elapsed() > 1.5)
				{
						RCLCPP_INFO(this->get_logger(), "surround_land = %d", surround_land);
						rotate_global2stand(scout_x + static_cast<double>(surround_land) * 1.0, scout_y, x_home, y_home);
						RCLCPP_INFO(this->get_logger(), "land点 x: %lf   y: %lf    angle: %lf", x_home, y_home, 0.0);
						send_local_setpoint_command(x_home, y_home, scout_halt, 0);
						timer_.set_timepoint();
						surround_land++;
				}
			}
			else
			{
				RCLCPP_INFO(this->get_logger(), "看见H了，执行PID_rtl");
				if (catch_target(
						defaults,
						YOLO::TARGET_TYPE::H, // 目标类型
						tar_x, tar_y, tar_z, 0, accuracy
					))
				{
					RCLCPP_INFO(this->get_logger(), "到达降落点");
					land_state_ = LandState::end;
					continue; // 直接跳到下一个状态;
				}
				else
				{
					RCLCPP_INFO(this->get_logger(), "未到达降落点");
				}
			}
			break;
		}
		case LandState::end:{
			send_velocity_command_with_time(0, 0, -0.1, 0, 1);
			RCLCPP_INFO(this->get_logger(), "降落");
			land_state_ = LandState::init;
			surround_land = 0;
			timer_.set_timepoint();
			result = true;
			break;
		}
		default:
			break;
		}
		break;
	}
	return result;
}

// OffboardControl.h #define TRAIN_PID
struct PIDDataPoint
{
	double p;
	double i;
	double d;
	double error;
	double time;
} data_point;

// 抵达桶上方
// if(识别到桶=catch_target_bucket（到达正上方）){[if(到达正上方==true){...}}
bool OffboardControl::autotune(bool &result, enum YOLO::TARGET_TYPE target)
{
	static enum class CatchState {
		init,
		fly_to_target,
		end
	} catch_state_ = CatchState::init;
	static double time_find_start = 0; // 开始时间
	static float tar_z = 1;		// 声明目标高度（m）
	static float tar_yaw = 0; // 声明目标偏航角（rad）
	static float dt = 0.1;									 // 声明执行周期（s）
	static float last_time = get_cur_time(); // 声明上次执行时间
	static float accuracy = 0.1; // 声明精度

	// yolo未识别到桶
	if (is_equal(_yolo->get_x(target), (float)0) && is_equal(_yolo->get_y(target), (float)0))
	{
		RCLCPP_INFO(this->get_logger(), "Doshot: yolo未识别到桶");
		result = false;
		return false;
	}
	RCLCPP_INFO(this->get_logger(), "Doshot: x: %f, y: %f", _yolo->get_x(target), _yolo->get_y(target));

	switch (catch_state_)
	{
	case CatchState::init:
	{
		// 读取PID参数
		PID::Defaults defaults = PID::readPIDParameters("can_config.yaml", "pid_bucket");
		PosControl::Limits_t limits = _pose_control->readLimits("can_config.yaml", "limits");
		_pose_control->set_limits(limits);
		// 读取距离目标一定范围内退出的距离
		YAML::Node config = Readyaml::readYAML("can_config.yaml");
		accuracy = config["accuracy"].as<float>();

		RCLCPP_INFO(this->get_logger(), "Doshot: Init1");
		data_point.p = defaults.p;
		data_point.i = defaults.i;
		data_point.d = defaults.d;
		data_point.error = 0;
		data_point.time = 1000;
		time_find_start = get_cur_time();
		last_time = get_cur_time();
		tar_z = 1.5;								 // 设置目标高度（m）
		tar_yaw = 0;			 // 设置目标偏航角（rad）
		dt = 0.25;								 // 设置执行周期（s）
		_pose_control->set_dt(dt); // 设置执行周期（用于PID）

		RCLCPP_INFO(this->get_logger(), "Doshot: Init2");
		RCLCPP_INFO(this->get_logger(), "--------------------\n\n读取pid_bucket: p: %f, i: %f, d: %f, ff: %f, dff: %f, imax: %f", defaults.p, defaults.i, defaults.d, defaults.ff, defaults.dff, defaults.imax);
		RCLCPP_INFO(this->get_logger(), "n读取limits: speed_max_xy: %f, speed_max_z: %f, accel_max_x: %f, accel_max_z: %f", limits.speed_max_xy, limits.speed_max_z, limits.accel_max_xy, limits.accel_max_z);

		catch_state_ = CatchState::fly_to_target;
		break;
	}
	case CatchState::fly_to_target:
	{
		// 以 dt 为周期，执行一次 PID 控制
		RCLCPP_INFO(this->get_logger(), "当前时间: %f, 上次时间: %f, 执行周期: %f, 是否执行: %d", get_cur_time(), last_time, dt, get_cur_time() - last_time < dt);
		if (get_cur_time() - last_time < dt)
		{
			result = false;
			return true;
		}
		last_time = get_cur_time();

		// yolo返回值坐标系：x右y上，转换为飞机坐标系：x左y上
		float now_x = _yolo->get_cap_frame_width()-_yolo->get_x(target);
		float now_y = _yolo->get_cap_frame_height()-_yolo->get_y(target);
		float tar_x = _yolo->get_cap_frame_width()-_yolo->get_cap_frame_width() / 2; // /10
		float tar_y = _yolo->get_cap_frame_height()-_yolo->get_cap_frame_height() / 2; // /3
		rotate_xy(now_x, now_y, (get_yaw() - default_yaw));
		rotate_xy(tar_x, tar_y, (get_yaw() - default_yaw));
		RCLCPP_INFO(this->get_logger(), "Doshot: now_x: %f, now_y: %f, tar_x: %f, tar_y: %f", now_x, now_y, tar_x, tar_y);
		RCLCPP_INFO(this->get_logger(), "Doshot: now_x: %f, now_y: %f, tar_x: %f, tar_y: %f", now_x / _yolo->get_cap_frame_width(), now_y / _yolo->get_cap_frame_height(), (tar_x) / _yolo->get_cap_frame_width(), (tar_y) / _yolo->get_cap_frame_height());
		static float _t_time = get_cur_time();

		(void)accuracy;
		if (!_pose_control->auto_tune 
				(
						Vector4f{now_x / _yolo->get_cap_frame_width(), now_y / _yolo->get_cap_frame_height(), get_z_pos(), get_yaw()},
						Vector4f{(tar_x) / _yolo->get_cap_frame_width(), (tar_y) / _yolo->get_cap_frame_height(), tar_z, tar_yaw},
						(uint32_t)(dt * 1000),
						true,
						true,
						false,
						false
					)
				)
		{
			double error_x = abs(now_x / _yolo->get_cap_frame_width() - tar_x / _yolo->get_cap_frame_width());
			double error_y = abs(now_y / _yolo->get_cap_frame_height() - tar_y / _yolo->get_cap_frame_height());
			data_point.error += error_x + error_y;
		}
		else
		{
			RCLCPP_INFO(this->get_logger(), "Arrive, Doshot, time = %f", get_cur_time() - _t_time);
			data_point.time = _t_time - get_cur_time();
			_t_time = get_cur_time();
			catch_state_ = CatchState::end;
		}

		if (
				get_cur_time() - time_find_start > 1200 //>1200s
		)
		{
			catch_state_ = CatchState::end;
		}
		break;
	}
	case CatchState::end:
	{
		static double end_time = get_cur_time(); // 记录结束时间
    if (get_cur_time() - end_time < 3.0) {   // 非阻塞等待3秒
        result = false;
        return true;
    }
		RCLCPP_INFO(this->get_logger(), "error:%lf,time:%lf", data_point.error, data_point.time);
		_pose_control->reset_limits();
		RCLCPP_INFO(this->get_logger(), "Arrive, 投弹");

    // 重置所有静态变量
    catch_state_ = CatchState::init;
    time_find_start = 0;
    tar_z = 1.0;
    tar_yaw = 0;
    dt = 0.1;
    last_time = get_cur_time();

		result = true;
		return true;
		break;
	}
	default:
		break;
	}
	RCLCPP_INFO(this->get_logger(), "catch_target_bucket: end");
	result = false;
	return true;
}

void OffboardControl::send_local_setpoint_command(float x, float y, float z, float yaw){
	// yaw = fmod(M_PI / 2 - yaw + 2 * M_PI, 2 * M_PI);
	// default_yaw M_PI/2 - headingangle_compass 
	_pose_control->send_local_setpoint_command(x, y, z, default_yaw - yaw);
}

bool OffboardControl::local_setpoint_command(float x, float y, float z, float yaw, double accuracy){
	return _pose_control->local_setpoint_command(
		Vector4f{get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()},
		Vector4f{x, y, z, static_cast<float>(yaw + default_yaw)},
		accuracy);
}

bool OffboardControl::trajectory_setpoint(float x, float y, float z, float yaw, double accuracy)
{
	RCLCPP_INFO_ONCE(this->get_logger(), "trajectory_setpoint转换后目标位置：%f %f", x, y);
	return _pose_control->trajectory_setpoint(
			Vector4f{get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()},
			Vector4f{x, y, z, static_cast<float>(yaw + default_yaw)},
			accuracy);
}
bool OffboardControl::trajectory_setpoint_world(float x, float y, float z, float yaw, PID::Defaults defaults, double accuracy)
{
	return _pose_control->trajectory_setpoint_world(
			Vector4f{get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()},
			Vector4f{x, y, z, static_cast<float>(yaw + default_yaw)},
			defaults,
			accuracy);
}
bool OffboardControl::trajectory_setpoint_world(float x, float y, float z, float yaw, double accuracy)
{
	return _pose_control->trajectory_setpoint_world(
			Vector4f{get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()},
			Vector4f{x, y, z, static_cast<float>(yaw + default_yaw)},
			accuracy);
}

bool OffboardControl::publish_setpoint_world(float x, float y, float z, float yaw, double accuracy)
{
	return _pose_control->publish_setpoint_world(
			Vector4f{get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()},
			Vector4f{x, y, z, static_cast<float>(yaw + default_yaw)},
			accuracy);
}

void OffboardControl::send_velocity_command(float x, float y, float z, float yaw)
{
	return _pose_control->send_velocity_command(
			Vector4f{x, y, z, yaw});
}
bool OffboardControl::send_velocity_command_with_time(float x, float y, float z, float yaw, double time)
{
	return _pose_control->send_velocity_command_with_time(
			Vector4f{x, y, z, yaw},
			time);
}
bool OffboardControl::trajectory_circle(float a, float b, float height, float dt, float yaw)
{
	return _pose_control->trajectory_circle(
			a,
			b,
			(height - get_z_pos()),
			dt,
			yaw + default_yaw,
			default_yaw);
}
bool OffboardControl::trajectory_generator_world(double speed_factor, std::array<double, 3> q_goal)
{
	return _pose_control->trajectory_generator_world(
			speed_factor,
			q_goal);
}
// bool OffboardControl::trajectory_generator(double speed_factor, std::array<double, 3> q_goal){
// 	rotate_xy(q_goal[0], q_goal[1], default_yaw);
// 	return _pose_control->trajectory_generator_world(
// 		speed_factor,
// 		{q_goal[0]+get_x_pos(), q_goal[1]+get_y_pos(),q_goal[2]+get_z_pos()}
// 	);
// }
bool OffboardControl::trajectory_generator_world_points(double speed_factor, const std::vector<std::array<double, 3>> &data, int data_length, Vector3f max_speed_xy, Vector3f max_accel_xy, float tar_yaw)
{
	static bool first = true;
	static uint16_t data_length_;
	static uint16_t current_waypoint_index = 0;
	static bool sequence_completed = false;
	
	// 只在第一次调用或者序列完成后需要重新开始时初始化
	if (first || (sequence_completed))
	{
		data_length_ = data_length;
		current_waypoint_index = 0;
		sequence_completed = false;
		first = false;
		std::cout << "Initializing waypoint sequence: data_length=" << data_length_ << std::endl;
	}
	
	std::cout << "data.size(): " << data.size() << std::endl;
	std::cout << "data_length: " << data_length_ << std::endl;
	std::cout << "current_waypoint_index: " << current_waypoint_index << std::endl;
	
	// 检查数据有效性，防止越界访问
	if (data.empty() || current_waypoint_index >= data.size() || current_waypoint_index >= data_length_) {
		std::cout << "数据无效或已完成，返回true" << std::endl;
		sequence_completed = true;
		return true;
	}

	std::array<double, 3> q_goal = data[current_waypoint_index];
	double global_x, global_y;
	rotate_global2stand(q_goal[0], q_goal[1], global_x, global_y);
	std::cout << "waypoint " << current_waypoint_index << " q_goal: " << global_x << " " << global_y << " " << q_goal[2] << std::endl;
	
	if (_pose_control->trajectory_generator_world(
				speed_factor,
				{global_x, global_y, q_goal[2]},
				max_speed_xy,
				max_accel_xy,
				static_cast<float>(tar_yaw + default_yaw)
			)
		)
	{
		current_waypoint_index++;
		std::cout << "Waypoint " << (current_waypoint_index - 1) << " completed! Moving to waypoint " << current_waypoint_index << std::endl;
		
		// 检查是否完成所有waypoint
		if (current_waypoint_index >= data.size() || current_waypoint_index >= data_length_)
		{
			std::cout << "All waypoints completed!" << std::endl;
			sequence_completed = true;
			return true;
		}
	}
	
	return false;
}