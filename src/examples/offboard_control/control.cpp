#include "OffboardControl.h"
// #include "PID.h"
#include "math.h"


#define AUTOTUNE

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
	// 读取PID参数
	PID::Defaults defaults = PID::readPIDParameters("can_config.yaml", "pid_bucket");
	PosControl::Limits_t limits = _pose_control->readLimits("can_config.yaml", "limits");
	_pose_control->set_limits(limits);

	// yolo未识别到桶
	if (is_equal(_yolo->get_x(target), (float)0) && is_equal(_yolo->get_y(target), (float)0))
	{
		RCLCPP_INFO(this->get_logger(), "catch_target_bucket: yolo未识别到桶");
		result = false;
		return false;
	}

	static enum class CatchState {
		init,
		fly_to_target,
		end
	} catch_state_ = CatchState::init;
	static double time_find_start = 0; // 开始时间
	// (void)time_find_start;
	static float set_z = 1;		// 声明目标高度（m）
	static float set_yaw = 0; // 声明目标偏航角（rad）
	// bool result=false;
	// if(vehicle_local_position_msg.x>10 || vehicle_local_position_msg.x<-10 || vehicle_local_position_msg.y>10 || vehicle_local_position_msg.y<-10){
	//	result=true;
	// }
	// double time_now = get_cur_time();
	// AtCheckPoint();

	static float dt = 0.1;									 // 声明执行周期（s）
	static float last_time = get_cur_time(); // 声明上次执行时间
	switch (catch_state_)
	{
	case CatchState::init:
	{
		data_point.p = defaults.p;
		data_point.i = defaults.i;
		data_point.d = defaults.d;
		data_point.error = 0;
		data_point.time = 1000;
		time_find_start = get_cur_time();
		set_z = 1.5;								 // 设置目标高度（m）
		set_yaw = get_yaw();			 // 设置目标偏航角（rad）
		dt = 0.25;								 // 设置执行周期（s）
		_pose_control->set_dt(dt); // 设置执行周期（用于PID）
		catch_state_ = CatchState::fly_to_target;
		break;
	}
	case CatchState::fly_to_target:
	{
		// 以 dt 为周期，执行一次 PID 控制
		// RCLCPP_INFO(this->get_logger(), "当前时间: %f, 上次时间: %f, 执行周期: %f, 是否执行: %d", get_cur_time(), last_time, dt, get_cur_time() - last_time < dt);
		if (get_cur_time() - last_time < dt)
		{
			result = false;
			return true;
		}
		RCLCPP_INFO(this->get_logger(), "--------------------\n\n\n\n读取pid_bucket: p: %f, i: %f, d: %f, ff: %f, dff: %f, imax: %f", defaults.p, defaults.i, defaults.d, defaults.ff, defaults.dff, defaults.imax);
		RCLCPP_INFO(this->get_logger(), "n读取limits: speed_max_xy: %f, speed_max_z: %f, accel_max_x: %f, accel_max_z: %f", limits.speed_max_xy, limits.speed_max_z, limits.accel_max_xy, limits.accel_max_z);

		last_time = get_cur_time();

		// yolo返回值坐标系：x右y上，转换为飞机坐标系：x左y上
		float now_x = -_yolo->get_x(target);
		float now_y = _yolo->get_y(target);
		float tar_x = -SET_CAP_FRAME_WIDTH / 2; // /10
		float tar_y = SET_CAP_FRAME_HEIGHT / 2; // /3
		rotate_xy(now_x, now_y, (DEFAULT_YAW + get_yaw()));
		rotate_xy(tar_x, tar_y, (DEFAULT_YAW + get_yaw()));
		RCLCPP_INFO(this->get_logger(), "catch_target_bucket: now_x: %f, now_y: %f, tar_x: %f, tar_y: %f", now_x, now_y, tar_x, tar_y);
		RCLCPP_INFO(this->get_logger(), "catch_target_bucket: now_x: %f, now_y: %f, tar_x: %f, tar_y: %f", now_x / SET_CAP_FRAME_WIDTH, now_y / SET_CAP_FRAME_HEIGHT, (tar_x) / SET_CAP_FRAME_WIDTH, (tar_y) / SET_CAP_FRAME_HEIGHT);
		// PID::Defaults defaults;
		// PoseControl::Limits limits;
		// _pose_control->set_limits(PosControl::Limits_t{.speed_max_z = 0.3,.accel_max_z = 0.1});
		// _pose_control->set_limits(PosControl::Limits_t{2, 0.5, 0.3, 0.5, 0.5, 0.1});
		// bool trajectory_setpoint(Vector4f pos_now,Vector4f pos_target,PID::Defaults defaults,double accuracy = DEFAULT_ACCURACY);
		// 假设Vector4f可以这样构造，并且PID::Defaults有一个名为p的公共成员变量
		static float _t_time = get_cur_time();
#ifndef AUTOTUNE
		if (!_pose_control->trajectory_setpoint_world(
						Vector4f{now_x / SET_CAP_FRAME_WIDTH, now_y / SET_CAP_FRAME_HEIGHT, get_z_pos(), get_yaw()},
						Vector4f{(tar_x) / SET_CAP_FRAME_WIDTH, (tar_y) / SET_CAP_FRAME_HEIGHT, set_z, set_yaw},
						// PID::Defaults{.p = 0.5,.i = 0.1,.d = 0.1,.imax = 10,._use_vel = false}  //c++20
						// PID::Defaults{0.3,0.01,0.01,0,0,1},
						defaults,
						0.01 // accuracy
					)
				)
		{
			// RCLCPP_INFO(this->get_logger(), "Arrive, time = %f", get_cur_time()-_t_time);

			double error_x = abs(now_x / SET_CAP_FRAME_WIDTH - tar_x / SET_CAP_FRAME_WIDTH);
			double error_y = abs(now_y / SET_CAP_FRAME_HEIGHT - tar_y / SET_CAP_FRAME_HEIGHT);
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
		if (!_pose_control->auto_tune
				(
						Vector4f{now_x / SET_CAP_FRAME_WIDTH, now_y / SET_CAP_FRAME_HEIGHT, get_z_pos(), get_yaw()},
						Vector4f{(tar_x) / SET_CAP_FRAME_WIDTH, (tar_y) / SET_CAP_FRAME_HEIGHT, set_z, set_yaw},
						(uint32_t)(dt * 1000),
						true,
						true,
						false,
						false
					)
				)
		{
			double error_x = abs(now_x / SET_CAP_FRAME_WIDTH - tar_x / SET_CAP_FRAME_WIDTH);
			double error_y = abs(now_y / SET_CAP_FRAME_HEIGHT - tar_y / SET_CAP_FRAME_HEIGHT);
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
		RCLCPP_INFO(this->get_logger(), "error:%lf,time:%lf", data_point.error, data_point.time);
		// _pose_control->set_limits(PosControl::Limits_t{});
		_pose_control->reset_limits();
		RCLCPP_INFO(this->get_logger(), "Arrive, 投弹");

		rclcpp::sleep_for(std::chrono::seconds(3));
		catch_state_ = CatchState::init;
		result = true;
		return true;
		break;
	}
	default:
		break;
	}
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

bool OffboardControl::trajectory_setpoint(float x, float y, float z, float yaw, double accuracy)
{
	rotate_xy(x, y, DEFAULT_YAW);
	return _pose_control->trajectory_setpoint(
			Vector4f{get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()},
			Vector4f{x, y, z, static_cast<float>(yaw + DEFAULT_YAW)},
			accuracy);
}
bool OffboardControl::trajectory_setpoint_world(float x, float y, float z, float yaw, PID::Defaults defaults, double accuracy)
{
	rotate_xy(x, y, DEFAULT_YAW);
	return _pose_control->trajectory_setpoint_world(
			Vector4f{get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()},
			Vector4f{x, y, z, static_cast<float>(yaw + DEFAULT_YAW)},
			defaults,
			accuracy);
}
bool OffboardControl::trajectory_setpoint_world(float x, float y, float z, float yaw, double accuracy)
{
	rotate_xy(x, y, DEFAULT_YAW);
	return _pose_control->trajectory_setpoint_world(
			Vector4f{get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()},
			Vector4f{x, y, z, static_cast<float>(yaw + DEFAULT_YAW)},
			accuracy);
}

bool OffboardControl::publish_setpoint_world(float x, float y, float z, float yaw, double accuracy)
{
	rotate_xy(x, y, DEFAULT_YAW);
	return _pose_control->publish_setpoint_world(
			Vector4f{get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()},
			Vector4f{x, y, z, static_cast<float>(yaw + DEFAULT_YAW)},
			accuracy);
}

void OffboardControl::send_velocity_command(float x, float y, float z, float yaw)
{
	rotate_xy(x, y, DEFAULT_YAW);
	return _pose_control->send_velocity_command(
			Vector4f{x, y, z, yaw});
}
bool OffboardControl::send_velocity_command_with_time(float x, float y, float z, float yaw, double time)
{
	rotate_xy(x, y, DEFAULT_YAW);
	return _pose_control->send_velocity_command_with_time(
			Vector4f{x, y, z, yaw},
			time);
}
bool OffboardControl::trajectory_circle(float a, float b, float height, float dt, float yaw)
{
	return _pose_control->trajectory_circle(
			a,
			b,
			(height - InertialNav::position.z),
			dt,
			yaw + DEFAULT_YAW,
			DEFAULT_YAW);
}
bool OffboardControl::trajectory_generator_world(double speed_factor, std::array<double, 3> q_goal)
{
	rotate_xy(q_goal[0], q_goal[1], DEFAULT_YAW);
	return _pose_control->trajectory_generator_world(
			speed_factor,
			q_goal);
}
// bool OffboardControl::trajectory_generator(double speed_factor, std::array<double, 3> q_goal){
// 	rotate_xy(q_goal[0], q_goal[1], DEFAULT_YAW);
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
	rotate_xy(q_goal[0], q_goal[1], DEFAULT_YAW);
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