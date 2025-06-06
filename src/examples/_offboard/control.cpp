#include "OffboardControl.h"
// #include "PID.h"
#include "math.h"


// #define AUTOTUNE

// 指定间隔时间循环执行航点
// x，y为中心位置，length，width为航点的长宽，halt为高度，way_points为航点集合，description为航点描述
bool OffboardControl::waypoint_goto_next(double x, double y, double length, double width, double halt, vector<Vector2f> &way_points, double time, int *count, const std::string &description)
{
	static Timer start_(true);         // 航点计时器，true表明实例化后发送航点
	static std::vector<Vector2f>::size_type surround_cnt = 0; // 修改类型
	double x_temp = 0.0, y_temp = 0.0;
	if(count!=nullptr)
		RCLCPP_INFO(this->get_logger(), "w_g_n,counter: %d, time=%lf", *count, start_.elapsed());
	if (start_.elapsed() > time) 
	{
		if (static_cast<std::vector<Vector2f>::size_type>(count == nullptr? surround_cnt : *count) >= way_points.size())
		{
				RCLCPP_INFO(this->get_logger(), "%s已经全部遍历", description.c_str());
				count == nullptr? surround_cnt = 0 : *count = 0;
				// start_.reset();
				start_.set_start_time_to_default();
				return true;
		} else {
			x_temp = x + (length * way_points[count == nullptr? surround_cnt : *count].x());
			y_temp = y + (width * way_points[count == nullptr? surround_cnt : *count].y());
			count == nullptr? surround_cnt++ : (*count)++;
			RCLCPP_INFO(this->get_logger(), "%s点位%zu x: %lf   y: %lf", description.c_str(), count == nullptr? surround_cnt : *count, x_temp, y_temp);
			send_local_setpoint_command(x_temp, y_temp, halt, start.w());
			// RCLCPP_INFO(this->get_logger(), "前往下一点");
			start_.reset();
		}
	}
	return false;
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
	double x_home = 0.0, y_home = 0.0;
	switch (land_state_)
	{
	case LandState::init:{
		// 读取PID参数
		defaults = PID::readPIDParameters("land_config.yaml", "pid");
		PosControl::Limits_t limits = _pose_control->readLimits("land_config.yaml", "limits");
		_pose_control->set_limits(limits);
    RCLCPP_INFO(this->get_logger(), "Doland");
		x_home = 0.0, y_home = 0.0;
		rotate2global(x_home, y_home);
    RCLCPP_INFO(this->get_logger(), "返回降落准备点 x: %lf   y: %lf    angle: %lf", x_home, y_home, headingangle_compass);
    send_local_setpoint_command(x_home, y_home, 4, headingangle_compass);
    rclcpp::sleep_for(std::chrono::seconds(6));
		x_home = 0.0, y_home = 0.3;
		rotate2global(x_home, y_home);
    RCLCPP_INFO(this->get_logger(), "返回降落点 x: %lf   y: %lf    angle: %lf", x_home, y_home, headingangle_compass);
    send_local_setpoint_command(x_home, y_home, 4, headingangle_compass);
		timer_.reset();
		timer_.set_timepoint();
		land_state_ = LandState::land_to_target;
		break;
	}
	case LandState::land_to_target:{
		if (timer_.elapsed() > 19 || surround_land > 3)
		{
			land_state_ = LandState::end;
			break;
		}
		// yolo未识别到桶
		if (is_equal(_yolo->get_x(YOLO::TARGET_TYPE::H), (float)0) && is_equal(_yolo->get_y(YOLO::TARGET_TYPE::H), (float)0))
		{
			if (timer_.get_timepoint_elapsed() > 1.5)
			{
					RCLCPP_INFO(this->get_logger(), "surround_land = %d", surround_land);
					x_home = surround_land;
					y_home = 0;
					rotate2global(x_home, y_home);
					RCLCPP_INFO(this->get_logger(), "land点 x: %lf   y: %lf    angle: %lf", x_home, y_home, headingangle_compass);
					send_local_setpoint_command(x_home, y_home, shot_halt, headingangle_compass);
					timer_.set_timepoint();
					surround_land++;
			}
		}
		else
		{
			RCLCPP_INFO(this->get_logger(), "看见H了，执行PID_rtl");
			trajectory_setpoint_world(
				_yolo->get_x(YOLO::TARGET_TYPE::H), _yolo->get_y(YOLO::TARGET_TYPE::H), get_rangefinder_distance(),	0,
				defaults, 30
			);
		}
		break;
	}
	case LandState::end:{
		send_velocity_command_with_time(0, 0, -0.1, 0, 1);
		RCLCPP_INFO(this->get_logger(), "1s后降落");
		return true;
		break;
	}
	default:
		break;
	}
	return false;
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
bool OffboardControl::catch_target(bool &result, enum YOLO::TARGET_TYPE target)
{
	static enum class CatchState {
		init,
		fly_to_target,
		end
	} catch_state_ = CatchState::init;
	static double time_find_start = 0; // 开始时间
	static float set_z = 1;		// 声明目标高度（m）
	static float set_yaw = 0; // 声明目标偏航角（rad）
	static float dt = 0.1;									 // 声明执行周期（s）
	static float last_time = get_cur_time(); // 声明上次执行时间
	static float accuracy = 0.1; // 声明精度

	// 读取PID参数
	PID::Defaults defaults = PID::readPIDParameters("can_config.yaml", "pid_bucket");
	PosControl::Limits_t limits = _pose_control->readLimits("can_config.yaml", "limits");
	_pose_control->set_limits(limits);
	// 读取距离目标一定范围内退出的距离
	YAML::Node config = Readyaml::readYAML("can_config.yaml");
	accuracy = config["accuracy"].as<float>();

	// yolo未识别到桶
	if (is_equal(_yolo->get_x(target), (float)0) && is_equal(_yolo->get_y(target), (float)0))
	{
		RCLCPP_INFO(this->get_logger(), "catch_target_bucket: yolo未识别到桶");
		result = false;
		return false;
	}
	RCLCPP_INFO(this->get_logger(), "catch_target_bucket: x: %f, y: %f", _yolo->get_x(target), _yolo->get_y(target));

	switch (catch_state_)
	{
	case CatchState::init:
	{
		RCLCPP_INFO(this->get_logger(), "catch_target_bucket: Init1");
		data_point.p = defaults.p;
		data_point.i = defaults.i;
		data_point.d = defaults.d;
		data_point.error = 0;
		data_point.time = 1000;
		time_find_start = get_cur_time();
		last_time = get_cur_time();
		set_z = 1.5;								 // 设置目标高度（m）
		set_yaw = get_yaw();			 // 设置目标偏航角（rad）
		dt = 0.25;								 // 设置执行周期（s）
		_pose_control->set_dt(dt); // 设置执行周期（用于PID）
		catch_state_ = CatchState::fly_to_target;
		RCLCPP_INFO(this->get_logger(), "catch_target_bucket: Init2");
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
		RCLCPP_INFO(this->get_logger(), "--------------------\n\n读取pid_bucket: p: %f, i: %f, d: %f, ff: %f, dff: %f, imax: %f", defaults.p, defaults.i, defaults.d, defaults.ff, defaults.dff, defaults.imax);
		RCLCPP_INFO(this->get_logger(), "n读取limits: speed_max_xy: %f, speed_max_z: %f, accel_max_x: %f, accel_max_z: %f", limits.speed_max_xy, limits.speed_max_z, limits.accel_max_xy, limits.accel_max_z);

		last_time = get_cur_time();

		// yolo返回值坐标系：x右y上，转换为飞机坐标系：x左y上
		float now_x = _yolo->get_cap_frame_width()-_yolo->get_x(target);
		float now_y = _yolo->get_cap_frame_height()-_yolo->get_y(target);
		float tar_x = _yolo->get_cap_frame_width()-_yolo->get_cap_frame_width() / 2; // /10
		float tar_y = _yolo->get_cap_frame_height()-_yolo->get_cap_frame_height() / 2; // /3
		rotate_xy(now_x, now_y, (default_yaw + get_yaw()));
		rotate_xy(tar_x, tar_y, (default_yaw + get_yaw()));
		RCLCPP_INFO(this->get_logger(), "catch_target_bucket: now_x: %f, now_y: %f, tar_x: %f, tar_y: %f", now_x, now_y, tar_x, tar_y);
		RCLCPP_INFO(this->get_logger(), "catch_target_bucket: now_x: %f, now_y: %f, tar_x: %f, tar_y: %f", now_x / _yolo->get_cap_frame_width(), now_y / _yolo->get_cap_frame_height(), (tar_x) / _yolo->get_cap_frame_width(), (tar_y) / _yolo->get_cap_frame_height());
		// PID::Defaults defaults;
		// PoseControl::Limits limits;
		// _pose_control->set_limits(PosControl::Limits_t{.speed_max_z = 0.3,.accel_max_z = 0.1});
		// _pose_control->set_limits(PosControl::Limits_t{2, 0.5, 0.3, 0.5, 0.5, 0.1});
		// bool trajectory_setpoint(Vector4f pos_now,Vector4f pos_target,PID::Defaults defaults,double accuracy = DEFAULT_ACCURACY);
		// 假设Vector4f可以这样构造，并且PID::Defaults有一个名为p的公共成员变量
		static float _t_time = get_cur_time();
#ifndef AUTOTUNE
		if (!_pose_control->trajectory_setpoint_world(
						Vector4f{now_x / _yolo->get_cap_frame_width(), now_y / _yolo->get_cap_frame_height(), get_z_pos(), get_yaw()},
						Vector4f{(tar_x) / _yolo->get_cap_frame_width(), (tar_y) / _yolo->get_cap_frame_height(), set_z, set_yaw},
						// PID::Defaults{.p = 0.5,.i = 0.1,.d = 0.1,.imax = 10,._use_vel = false}  //c++20
						// PID::Defaults{0.3,0.01,0.01,0,0,1},
						defaults,
						accuracy // accuracy
					)
				)
		{
			// RCLCPP_INFO(this->get_logger(), "Arrive, time = %f", get_cur_time()-_t_time);

			double error_x = abs(now_x / _yolo->get_cap_frame_width() - tar_x / _yolo->get_cap_frame_width());
			double error_y = abs(now_y / _yolo->get_cap_frame_height() - tar_y / _yolo->get_cap_frame_height());
			data_point.error += error_x + error_y;

			// if(error_x<0.05 && error_y<0.05){
			// 	// RCLCPP_INFO(this->get_logger(), "Arrive, catch_target_bucket");
			// 	// catch_state_=CatchState::end;
			// } else if(_t_time - get_cur_time()>3){
			// 	// RCLCPP_INFO(this->get_logger(), "Approach, catch_target_bucket");
			// 	// catch_state_=CatchState::end;
			// }
		}
		else
		{
			RCLCPP_INFO(this->get_logger(), "Arrive, catch_target_bucket, time = %f", get_cur_time() - _t_time);
			data_point.time = _t_time - get_cur_time();
			_t_time = get_cur_time();
			catch_state_ = CatchState::end;
		}
#else
		(void)accuracy;
		if (!_pose_control->auto_tune 
				(
						Vector4f{now_x / _yolo->get_cap_frame_width(), now_y / _yolo->get_cap_frame_height(), get_z_pos(), get_yaw()},
						Vector4f{(tar_x) / _yolo->get_cap_frame_width(), (tar_y) / _yolo->get_cap_frame_height(), set_z, set_yaw},
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
			RCLCPP_INFO(this->get_logger(), "Arrive, catch_target_bucket, time = %f", get_cur_time() - _t_time);
			data_point.time = _t_time - get_cur_time();
			_t_time = get_cur_time();
			catch_state_ = CatchState::end;
		}
#endif

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
		// _pose_control->set_limits(PosControl::Limits_t{});
		_pose_control->reset_limits();
		RCLCPP_INFO(this->get_logger(), "Arrive, 投弹");

    // 重置所有静态变量
    catch_state_ = CatchState::init;
    time_find_start = 0;
    set_z = 1.0;
    set_yaw = 0;
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

// 环绕射击区域
bool OffboardControl::surrounding_shot_area(void)
{
	static enum class SurState {
		init,
		set_point_x,
		set_point_y,
		fly_to_target_x,
		fly_to_target_y,
		end
	} sur_state_;
	static double time_find_start = get_cur_time();
	if ((get_cur_time() - time_find_start) > 1200)
	{ //>12s
		return true;
	}

	// RCLCPP_INFO(this->get_logger(),"cur_time: %f , start_time: %f",get_cur_time(),time_find_start);

	// std::cout<<"cur time: "<<get_cur_time()<<" find_start: "<<time_find_start<<std::endl;
	bool arrive;
	if (catch_target(arrive, YOLO::TARGET_TYPE::CIRCLE))
	{
		if (arrive)
		{
			return true;
		}
		return false;
	}
	/*bool result=false;
	if(vehicle_local_position_msg.x>10 || vehicle_local_position_msg.x<-10 || vehicle_local_position_msg.y>10 || vehicle_local_position_msg.y<-10){
		result=true;
	}*/
	// double time_now = get_cur_time();
	// AtCheckPoint();

	switch (sur_state_)
	{
	case SurState::init:
		time_find_start = get_cur_time();
		static double fx = 1, fy = 2;
		sur_state_ = SurState::set_point_x;
		break;
	case SurState::set_point_x:
		sur_state_ = SurState::fly_to_target_x;
		break;
	case SurState::set_point_y:
		sur_state_ = SurState::fly_to_target_y;
		break;
	case SurState::fly_to_target_x:
		if (trajectory_setpoint(fx, 0, 0, 0))
		{
			fx = (fx > 0) ? (-fx - 1) : (-fx + 1);
			sur_state_ = SurState::set_point_y;
		}
		break;
	case SurState::fly_to_target_y:
		if (trajectory_setpoint(0, fy, 0, 0))
		{
			sur_state_ = SurState::set_point_x;
			fy = (fy > 0) ? (-fy - 1) : (-fy + 1);
		}
		break;
	case SurState::end:
		sur_state_ = SurState::init;
		fx = 1, fy = 2 * fx;
		return true;
		break;
	default:
		break;
	}
	return false;
}
// 环绕侦察区域
bool OffboardControl::surrounding_scout_area(void)
{
	static enum class ScoState {
		init,
		first_path,
		second_path,
		third_path,
		forth_path,
		fifth_path,
		sixth_path,
		end
	} sco_state_;
	static double time_find_start = 0;
	/*bool result=false;
	if(vehicle_local_position_msg.x>10 || vehicle_local_position_msg.x<-10 || vehicle_local_position_msg.y>10 || vehicle_local_position_msg.y<-10){
		result=true;
	}*/
	// double time_now = get_cur_time();
	// AtCheckPoint();
	switch (sco_state_)
	{
	case ScoState::init:
		time_find_start = get_cur_time();
		sco_state_ = ScoState::first_path;
		break;
	case ScoState::first_path:
		if (trajectory_setpoint(2, 0, 0, -60))
		{
			sco_state_ = ScoState::second_path;
		}
		if ((get_cur_time() - time_find_start) > 12)
		{ //>12s
			sco_state_ = ScoState::end;
		}
		break;
	case ScoState::second_path:
		if (trajectory_setpoint(-2, 2.5, 0, -60))
		{
			sco_state_ = ScoState::third_path;
		}
		if ((get_cur_time() - time_find_start) > 12)
		{ //>12s
			sco_state_ = ScoState::end;
		}
		break;
	case ScoState::third_path:
		if (trajectory_setpoint(2, 0, 0, -60))
		{
			sco_state_ = ScoState::forth_path;
		}
		if ((get_cur_time() - time_find_start) > 12)
		{ //>12s
			sco_state_ = ScoState::end;
		}
		break;
	case ScoState::forth_path:
		if (trajectory_setpoint(-2, 2.5, 0, -60))
		{
			sco_state_ = ScoState::fifth_path;
		}
		if ((get_cur_time() - time_find_start) > 12)
		{ //>12s
			sco_state_ = ScoState::end;
		}
		break;
	case ScoState::fifth_path:
		if (trajectory_setpoint(2, 0, 0, -60))
		{
			sco_state_ = ScoState::sixth_path;
		}
		if ((get_cur_time() - time_find_start) > 12)
		{ //>12s
			sco_state_ = ScoState::end;
		}
		break;
	case ScoState::sixth_path:
		if (trajectory_setpoint(-1, -2.5, 0, -90))
		{
			sco_state_ = ScoState::end;
		}
		if ((get_cur_time() - time_find_start) > 12)
		{ //>12s
			sco_state_ = ScoState::end;
		}
		break;
	case ScoState::end:
		sco_state_ = ScoState::init;
		RCLCPP_INFO(this->get_logger(), "end scouting: %f", (get_cur_time() - time_find_start) / 1000000.0);
		return true;
		break;
	default:
		break;
	}
	return false;
}

void OffboardControl::send_local_setpoint_command(float x, float y, float z, float yaw){
	rotate2global(x, y);
	_pose_control->send_local_setpoint_command(x, y, z, yaw);
}

bool OffboardControl::local_setpoint_command(float x, float y, float z, float yaw, double accuracy){
	rotate2global(x, y);
	return _pose_control->local_setpoint_command(
		Vector4f{get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()},
		Vector4f{x, y, z, static_cast<float>(yaw + default_yaw)},
		accuracy);
}

bool OffboardControl::trajectory_setpoint(float x, float y, float z, float yaw, double accuracy)
{
	rotate2global(x, y);
	RCLCPP_INFO_ONCE(this->get_logger(), "trajectory_setpoint转换后目标位置：%f %f", x, y);
	return _pose_control->trajectory_setpoint(
			Vector4f{get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()},
			Vector4f{x, y, z, static_cast<float>(yaw + default_yaw)},
			accuracy);
}
bool OffboardControl::trajectory_setpoint_world(float x, float y, float z, float yaw, PID::Defaults defaults, double accuracy)
{
	rotate2global(x, y);
	return _pose_control->trajectory_setpoint_world(
			Vector4f{get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()},
			Vector4f{x, y, z, static_cast<float>(yaw + default_yaw)},
			defaults,
			accuracy);
}
bool OffboardControl::trajectory_setpoint_world(float x, float y, float z, float yaw, double accuracy)
{
	rotate2global(x, y);
	return _pose_control->trajectory_setpoint_world(
			Vector4f{get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()},
			Vector4f{x, y, z, static_cast<float>(yaw + default_yaw)},
			accuracy);
}

bool OffboardControl::publish_setpoint_world(float x, float y, float z, float yaw, double accuracy)
{
	rotate2global(x, y);
	return _pose_control->publish_setpoint_world(
			Vector4f{get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()},
			Vector4f{x, y, z, static_cast<float>(yaw + default_yaw)},
			accuracy);
}

void OffboardControl::send_velocity_command(float x, float y, float z, float yaw)
{
	rotate2global(x, y);
	return _pose_control->send_velocity_command(
			Vector4f{x, y, z, yaw});
}
bool OffboardControl::send_velocity_command_with_time(float x, float y, float z, float yaw, double time)
{
	rotate2global(x, y);
	return _pose_control->send_velocity_command_with_time(
			Vector4f{x, y, z, yaw},
			time);
}
bool OffboardControl::trajectory_circle(float a, float b, float height, float dt, float yaw)
{
	return _pose_control->trajectory_circle(
			a,
			b,
			(height - InertialNav::position.z()),
			dt,
			yaw + default_yaw,
			default_yaw);
}
bool OffboardControl::trajectory_generator_world(double speed_factor, std::array<double, 3> q_goal)
{
	rotate2global(q_goal[0], q_goal[1]);
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
bool OffboardControl::trajectory_generator_world_points(double speed_factor, const std::vector<std::array<double, 3>> &data, int data_length, bool init)
{
	static bool first = true;
	static uint16_t data_length_;
	if (first || init)
	{
		data_length_ = data_length;
		first = false;
	}
	std::cout << "data.size(): " << data.size() << std::endl;
	std::cout << "data_length: " << data_length_ << std::endl;
	// std::cout<<"data: "<<<<std::endl;

	std::array<double, 3> q_goal = data[data.size() - data_length_];
	rotate2global(q_goal[0], q_goal[1]);
	std::cout << "q_goal: " << q_goal[0] << " " << q_goal[1] << " " << q_goal[2] << std::endl;
	// Vector3f max_vel = {__MAX_FLT__, __MAX_FLT__, __MAX_FLT__};
	// Vector3f max_acc = {__MAX_FLT__, __MAX_FLT__, __MAX_FLT__};
	// if(data_length_ != data.size()){}
	// if(data_length_ != 1){}
	if (_pose_control->trajectory_generator_world(
					speed_factor,
					{q_goal[0], q_goal[1], q_goal[2]}))
	{
		data_length_--;
	}
	if (data_length_ == 0)
	{
		data_length_ = 0;
		first = true;
		return true;
	}
	return false;
}