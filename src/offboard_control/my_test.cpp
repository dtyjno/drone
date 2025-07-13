#include "StateMachine.h"
#include "OffboardControl.h"
#inlcude "clustering.h"

auto target1 = calculateWorldPosition(image_point1, camera1, 0.0, 0.3);
*target1; //桶坐标变量
target1.x();//调取坐标
using Vector3d = Eigen::Matrix<double, 3, 1, Eigen::ColMajor>;
int trg_num = 0;//记录投弹次数

template<>
void StateMachine::handle_state<FlyState::Doshot>() 
{   
	if (current_state_ == FlyState::Doshot) 
    {
		RCLCPP_INFO_ONCE(owner_->get_logger(), "执行投弹任务Doshot");
		static Timer doshot_start = Timer();  // 全程计时器
		static int counter = 0; // 航点计数器
		static bool arrive = false; // 是否到达投弹区
		static vector<array<double, 3>> surround_shot_scout_points;

		if (doshot_start.elapsed() > 60) // 超时 60 秒
		{
			RCLCPP_INFO(owner_->get_logger(), "超时");
			owner_->doshot_state_ = owner_->DoshotState::doshot_end; // 设置投弹状态为结束
		} 
		switch (owner_->doshot_state_)  // 根据投弹状态执行不同的操作
		{
		case owner_->DoshotState::doshot_init: // 初始化投弹状态
			{
				RCLCPP_INFO(owner_->get_logger(), "开始投弹任务");
				surround_shot_scout_points = 
                {
					{owner_->tx_shot + 2.27, owner_->ty_shot + 1.73, 3},
					{owner_->tx_shot + 2.27, owner_->ty_shot + 3.27, 3},
					{owner_->tx_shot - 2.27, owner_->ty_shot + 3.27, 3},
					{owner_->tx_shot - 2.27, owner_->ty_shot + 1.73, 3},
					{owner_->tx_shot, owner_->ty_shot + 1.73, 3},
				};
				owner_->doshot_state_ = owner_->DoshotState::doshot_scout; // 设置投弹状态为侦查
				doshot_start.reset(); // 重置计时器
				counter = 0; // 重置计数器
				arrive = false; // 重置到达状态
			}
			break;
		case owner_->DoshotState::doshot_scout: // 侦查投弹区
			RCLCPP_INFO_ONCE(owner_->get_logger(), "开始侦查投弹区");
			if (!surround_shot_scout_points.empty()) 
            {
				if (owner_->trajectory_generator_world_points(
					1, surround_shot_scout_points, surround_shot_scout_points.size(), true
				)) 
                {
					owner_->state_timer_.reset();
					owner_->doshot_state_ = owner_->DoshotState::doshot_halt; // 设置投弹状态为侦查完成
				}
			} 
            else 
            {
				RCLCPP_WARN(owner_->get_logger(), "surround_shot_scout_points为空，跳转到doshot_init");
				owner_->doshot_state_ = owner_->DoshotState::doshot_halt;
			}
			break;
		case owner_->DoshotState::doshot_halt: // 侦查投弹区
			if (owner_->catch_target(arrive, YOLO::TARGET_TYPE::CIRCLE)
			)
            {  // (在高度高于1.6m前提下)判断是否找到目标，找到目标改变arrive值为true，如未找到目标
			    // 距离执行上一个航点时长超过3秒时，定义投弹区的宽度和长度，顺序发布航点
				// 判断是否遍历完投弹区
				//RCLCPP_INFO(owner_->get_logger(), "寻找完毕，投弹!!投弹!!")
			} 
            else if (owner_->waypoint_goto_next(
									owner_->tx_shot, owner_->ty_shot, owner_->shot_length, owner_->shot_width, 
									owner_->shot_halt, owner_->surround_shot_points, 3.0, &counter, "投弹区")
			) 
            {
				RCLCPP_INFO(owner_->get_logger(), "投弹区航点计数器：%d", counter);
			}
			if (arrive)
            {
				owner_->doshot_state_ = owner_->DoshotState::doshot_end; // 设置投弹状态为结束
			}
			break;
		case owner_->DoshotState::doshot_end: // 投弹进程
			RCLCPP_INFO(owner_->get_logger(), "投弹!!投弹!!，总用时：%f", doshot_start.elapsed());
			// 设置舵机位置
			owner_->_servo_controller->set_servo(11, 1800);
			owner_->_servo_controller->set_servo(12, 1800);
			// 重置状态
			owner_->doshot_state_ = owner_->DoshotState::doshot_init; // 重置投弹状态为侦查
			doshot_start.set_start_time_to_default();
			counter = 0;
			arrive = false; // 重置到达状态
			RCLCPP_INFO(owner_->get_logger(), "投弹完成，3s后前往侦查区域");
		    rclcpp::sleep_for(3s);
		    transition_to(FlyState::Goto_scoutpoint);
			break;
			
		default:
			break;
		}
		// 接受投弹信号
		// if (get_servo_flag() == true) // 找到目标
		// {
		// 	RCLCPP_INFO(owner_->get_logger(), "找到目标");
		// 	arrive = true;
		// }
	}
}

bool OffboardControl::Doland()
{
	static Timer timer_ = Timer();
	static enum class LandState 
	{
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
		rotate_global2stand(0.0, 0.0, x_home, y_home);
    RCLCPP_INFO(this->get_logger(), "返回降落准备点 x: %lf   y: %lf    angle: %lf", x_home, y_home, headingangle_compass);
    send_local_setpoint_command(x_home, y_home, 4, headingangle_compass);
    rclcpp::sleep_for(std::chrono::seconds(6));
		rotate_global2stand(0.0, 0.3, x_home, y_home);
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
					rotate_global2stand(static_cast<double>(surround_land) * 1.0, 0.0, x_home, y_home);
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

