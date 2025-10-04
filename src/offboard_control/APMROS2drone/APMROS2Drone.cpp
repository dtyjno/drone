#include "APMROS2Drone.h"

#include <rclcpp/rclcpp.hpp>
#include "../task/Task.h"
#include "../task/TaskManager.h"
// #include "../stateMachine/StateMachine.h"
#include "../task/WayPointTask.h"
#include "../task/PrintInfoTask.h"
#include "../task/SetPointTask.h"
#include "../task/RTLLandTask.h"
#include "../task/DoShotTask.h"
#include "../task/DoLandTask.h"
#include "../task/WaitTask.h"
// #include "state/states.h"

// Constructor implementation
APMROS2Drone::APMROS2Drone(const std::string& ardupilot_namespace,
	std::shared_ptr<StatusController> sta_ctl,
	std::shared_ptr<PosController> pos_ctl,
	std::shared_ptr<rclcpp::Node> node) :
    ROS2Drone(ardupilot_namespace, sta_ctl, pos_ctl, node)
    // state_machine_(StateMachine<APMROS2Drone>::getInstance())
{
	RCLCPP_INFO(node->get_logger(), "APMROS2Drone: Starting Offboard Control example");	


	// Initialize controllers
	// initialize_controllers(sta_ctl, pos_ctl);
    RCLCPP_INFO(node->get_logger(), "开始使用APM服务的离线控制 OffboardControl");
    RCLCPP_INFO(node->get_logger(), "初始化 OffboardControl， ardupilot_namespace: %s", topic_namespace.c_str());
    
    // if (print_info_) {
    //     state_machine_.transition_to(FlyState::Print_Info);
    // }

    // Initialize shared components that need shared_from_this()
    _servo_controller = std::make_shared<ROS2ServoController>(topic_namespace, node->shared_from_this());
    _camera_gimbal = std::make_shared<ROS2CameraGimbal>(topic_namespace, node->shared_from_this());
	yolo_detector = std::make_shared<ROS2YOLODetector>(node->shared_from_this());

    // 发布当前状态 
    state_publisher_ = node->create_publisher<std_msgs::msg::Int32>("current_state", 10);
    #ifdef PAL_STATISTIC_VISIBILITY
    stats_publisher_ = node->create_publisher<pal_statistics_msgs::msg::Statistics>("/statistics", 10);
    // stats_timer_ = node->create_wall_timer(wait_time, std::bind(&OffboardControl::publish_statistics, this));
    #endif

    mypid.readPIDParameters("pos_config.yaml","mypid");

	// timer_ = node->create_wall_timer(
	// 	wait_time,
	// 	std::bind(&ROS2Drone::timer_callback, this)
	// );

	// timestamp_init = get_cur_time();
}
// void APMROS2Drone::initializeStateMachine() {
//     try {
//         RCLCPP_INFO(node->get_logger(), "Starting state machine initialization...");
        
//         // Try to get shared_ptr - if this fails, skip state machine initialization
//         std::shared_ptr<APMROS2Drone> self;
//         try {
//             self = shared_from_this();
//         } catch (const std::bad_weak_ptr& e) {
//             RCLCPP_WARN(node->get_logger(), "State machine initialization skipped - object not yet managed by shared_ptr");
//             return;
//         }
        
//         RCLCPP_INFO(node->get_logger(), "Got shared_ptr successfully");
        
//         // Set state machine owner
//         state_machine_.setOwner(self);
//         RCLCPP_INFO(node->get_logger(), "StateMachine owner set successfully");
        
//         // Create state instances
//         RCLCPP_INFO(node->get_logger(), "Creating state instances...");
//         InitState::getInstance();
//         DolandState::getInstance();
//         DoshotState::getInstance();
//         GotoScoutPointState::getInstance();
//         GotoShotPointState::getInstance();
//         EndState::getInstance();
// 		TakeoffState::getInstance();
// 		SurroundSeeState::getInstance();
// 		StopState::getInstance();
// 		ReflushConfigState::getInstance();
// 		TerminalControlState::getInstance();
// 		GetTargetState::getInstance();
        
//         // // Register states manually
//         // RCLCPP_INFO(node->get_logger(), "Registering states...");
//         // state_machine_.registerState("InitState", const_cast<InitState&>(initState));
//         // state_machine_.registerState("DolandState", const_cast<DolandState&>(dolandState));
//         // state_machine_.registerState("DoshotState", const_cast<DoshotState&>(doshotState));
//         // state_machine_.registerState("GotoScoutPointState", const_cast<GotoScoutPointState&>(gotoScoutState));
//         // state_machine_.registerState("GotoShotPointState", const_cast<GotoShotPointState&>(gotoShotState));
//         // state_machine_.registerState("EndState", const_cast<EndState&>(endState));
        
//         // Set initial state
//         state_machine_.setCurrentState("InitState");
//         RCLCPP_INFO(node->get_logger(), "State machine initialized successfully with InitState");
        
//     } catch (const std::exception& e) {
//         RCLCPP_ERROR(node->get_logger(), "Failed to initialize state machine: %s", e.what());
//         // Don't throw - let the program continue without state machine
//     }
// }

/*
	北360|0偏航角 顺时针          90
		 y+                    |y+
西        x+ 东  <=> ---- x-____|______x+  0 偏航角 逆时针
270       90                   |
							   |y+
	  南180                     -90

 世界坐标系(东北天)    飞机坐标系(base_link)
 -> g2l_location
	aircraft_deg = 90 - world_deg
	aircraft_deg %= 360.0

*/

// void APMROS2Drone::timer_callback(void){
// 	static int call_count = 0;
// 	call_count++;
// 	if (call_count % 20 == 0) { // 每20次调用打印一次
// 		RCLCPP_INFO(node->get_logger(), "APMROS2Drone: Timer callback called %d times", call_count);
// 	}
// 	// 发布当前状态
// 	// publish_current_state();
// }

void APMROS2Drone::timer_callback(void)
{
	timer_callback_update();
	// accept(
	// 	SetPointTask::createTask("setpoint0")->set_config(
	// 		SetPointTask::Parameters{
	// 			dx_shot,    // target_x
	// 			dy_shot + shot_width_max / 2,    // target_y
	// 			shot_halt,    // target_z
	// 			0.0f,  // target_yaw
	// 			12.0f,    // point_time (到达点位的最长等待时间)
	// 			0.2f     // accuracy (点位到达精度)
	// 		}
	// 	)->next_task(
	// 		WayPointTask::createTask()->set_config(
	// 			WayPoints("waypoint0", std::vector<Vector4f>{
	// 				Vector4f(0.0f, 0.0f, 2.0f, 0.0f),
	// 				Vector4f(2.0f, 0.0f, 2.0f, M_PI/2)
	// 				// Vector4f(2.0f, 2.0f, 2.0f, M_PI),
	// 				// Vector4f(0.0f, 2.0f, 2.0f, -M_PI/2),
	// 				// Vector4f(0.0f, 0.0f, 2.0f, 0.0f)
	// 			}),
	// 			WayPointTask::Parameters{
	// 				0.0f,    // center_x
	// 				0.0f,    // center_y
	// 				1.0f,    // scope_length
	// 				1.0f,    // scope_width
	// 				2.0f,    // point_time (每个航点停留时间)
	// 				0.3f     // accuracy (航点到达精度)
	// 			}
	// 		)
	// 	)
	// );

	// WayPointTask::createTask()->set_config(
	// 	WayPoints("waypoint1", std::vector<Vector4f>{
	// 		Vector4f(0.0f, 0.0f, 2.0f, 0.0f),
	// 		Vector4f(2.0f, 0.0f, 2.0f, M_PI/2)
	// 		// Vector4f(2.0f, 2.0f, 2.0f, M_PI),
	// 		// Vector4f(0.0f, 2.0f, 2.0f, -M_PI/2),
	// 		// Vector4f(0.0f, 0.0f, 2.0f, 0.0f)
	// 	}),
	// 	WayPointTask::Parameters{
	// 		0.0f,    // center_x
	// 		0.0f,    // center_y
	// 		1.0f,    // scope_length
	// 		1.0f,    // scope_width
	// 		2.0f,    // point_time (每个航点停留时间)
	// 		0.3f     // accuracy (航点到达精度)
	// 	}
	// );


	// if (!WayPointTask::getTask()->is_execute_finished()) {
	// 	accept(PrintInfoTask::createTask()->next_task(WayPointTask::getTask())->next_task(PrintInfoTask::createTask("NextPrintInfo")));
	// }

	// TaskManager<APMROS2Drone>(shared_from_this())
	// 	.addTask(PrintInfoTask::createTask()->next_task(WayPointTask::getTask())->next_task(PrintInfoTask::createTask("NextPrintInfo")))
	// 	->addTask(WaitTask::createTask("Wait_2s")->set_config(2.0))
	// 	->execute();

	#ifdef PAL_STATISTIC_VISIBILITY
	publish_statistics();
	#endif

	// 桶1（1 -31） 2 (2 -32) 3 (-1 -33)
	// std::cout << get_yolo_detector()->get_x(YOLO_TARGET_TYPE::CIRCLE) << ","
	// 		  << get_yolo_detector()->get_y(YOLO_TARGET_TYPE::CIRCLE) << ","
	// 		  << get_yolo_detector()->get_x(YOLO_TARGET_TYPE::H) << ","
	// 		  << get_yolo_detector()->get_y(YOLO_TARGET_TYPE::H) << std::endl;
	// 调试输出：像素坐标和相机位置
	// std::cout << "相机当前位置: (" << _camera_gimbal->position[0] << ", " << _camera_gimbal->position[1] << ", " << _camera_gimbal->position[2] << ")" << std::endl;
	// std::cout << "相机旋转角度: roll=" << _camera_gimbal->rotation[0] << " pitch=" << _camera_gimbal->rotation[1] << " yaw=" << _camera_gimbal->rotation[2] << std::endl;
	// std::cout << "相机内参: fx=" << _camera_gimbal->fx << " fy=" << _camera_gimbal->fy << " cx=" << _camera_gimbal->cx << " cy=" << _camera_gimbal->cy << std::endl;

	// 测试目标可视化
	if (debug_mode_) {
		auto AppochTargetTask = AppochTargetTask::createTask("AppochTargetTask");
		AppochTargetTask::Parameters appoch_params;
		appoch_params.fx = _camera_gimbal->get_fx(); // 相机焦距，像素单位
		appoch_params.dynamic_target_image_callback = [this]() -> Vector2f {
			// return Vector2f(get_yolo_detector()->get_x(YOLO_TARGET_TYPE::CIRCLE),
			// 				get_yolo_detector()->get_y(YOLO_TARGET_TYPE::CIRCLE));
			return Vector2f(100, 100);
		};
		appoch_params.target_height = bucket_height; // 目标高度
		appoch_params.task_type = AppochTargetTask::Type::PID;
		AppochTargetTask->setParameters(appoch_params);
		accept(AppochTargetTask);
	}

	// 检查位置数据的有效性，防止段错误
	if (!debug_mode_ && !print_info_) {
		if (get_status_controller()->get_system_status() == StatusController::State__system_status::MAV_STATE_UNINIT) {
			RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "MAVROS待启动，等待MAVROS创建ROS节点...");
			return;
		} else if (get_status_controller()->get_system_status() == StatusController::State__system_status::MAV_STATE_BOOT) {
			RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "飞控正在启动，等待...");
			return;
		} else if (get_status_controller()->get_system_status() == StatusController::State__system_status::MAV_STATE_CALIBRATING) {
			RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "飞控正在校准，尚未准备好飞行，等待...");
			return;
		} else if (get_status_controller()->get_system_status() != StatusController::State__system_status::MAV_STATE_STANDBY &&
					get_status_controller()->get_system_status() != StatusController::State__system_status::MAV_STATE_ACTIVE) {
			RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "当前飞机状态 %d %s",
			get_status_controller()->get_system_status_uint8_t(), get_status_controller()->get_state_name().c_str());
		}
		if (!isfinite(get_x_pos()) || !isfinite(get_y_pos()) || !isfinite(get_z_pos())) {
			RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "位置数据无效，等待有效GPS信号(EKF3 IMU0/1 is using GPS)...");
			return;
		}
	}

	if (current_state == state::init || current_state ==  state::gotoshot || current_state == state::shot || current_state == state::dropping) {
		calculate_target_position();
	}

	// state_machine_.executeAllStates();
	if (print_info_) {
		accept(PrintInfoTask::createTask()->set_print_time(10000));
		return;
	} else {
		accept(PrintInfoTask::createTask()->set_print_time(3));
	}
	TaskManager<APMROS2Drone> task_manager(shared_from_this());

	SetPointTask::createTask("Goto_shotpoint")->set_config(
		SetPointTask::Parameters{
			dx_shot,    // target_x
			dy_shot + shot_width_max / 2,    // target_y
			shot_halt,    // target_z
			0.0f,  // target_yaw
			12.0f,    // point_time (到达点位的最长等待时间)
			0.10f     // accuracy (点位到达精度)
		}
	);

	auto do_shot_waypoint_task = WayPointTask::createTask("Do_shot_waypoint");
	WayPoints do_shot_waypoint = WayPoints(
		"投弹区",
		std::vector<Vector4f>{
			Vector4f( 1.5f , 4.5f, shot_halt_surround, 0.0f),
			Vector4f( 0.0f , 4.5f, shot_halt_surround, 0.0f),
			Vector4f( 0.75f, 3.4f, shot_halt_surround, 0.0f),
			Vector4f(-1.5f , 0.5f, shot_halt_surround, 0.0f),
			Vector4f( 0.75f, 1.7f, shot_halt_surround, 0.0f),
			Vector4f(-0.75f, 3.4f, shot_halt_surround, 0.0f),
			Vector4f(-0.75f, 1.7f, shot_halt_surround, 0.0f),
			Vector4f( 0.0f , 0.0f, shot_halt_surround, 0.0f),
			Vector4f( 0.0f , 1.0f, shot_halt_surround, 0.0f),
			Vector4f(-1.5f , 0.9f, shot_halt_surround, 0.0f),
			Vector4f( 1.5f , 0.5f, shot_halt_surround, 0.0f)
		}
	);
	do_shot_waypoint_task->set_config(
		do_shot_waypoint,
		WayPointTask::Parameters{
			dx_shot,    // center_x
			dy_shot,    // center_y
			// shot_length - 3.5f,    // scope_length
			// shot_width,    // scope_width
			5.0f,    // point_time (每个航点停留时间)
			0.3f     // accuracy (航点到达精度)
		}
	);

	auto do_shot_task = DoShotTask::createTask("Do_shot");
	DoShotTask::Parameters doshot_params;
	doshot_params.dynamic_target_position_callback = [this, do_shot_task]() -> AppochTargetTask::PositionTarget {
		// Vector2d drone_to_shot_rotated; // 中间变量
		// rotate_local2world(0.0, 0.10, drone_to_shot_rotated.x(), drone_to_shot_rotated.y());
		AppochTargetTask::PositionTarget pos_target;
		if (!cal_center.empty() && static_cast<size_t>(do_shot_task->get_appochtarget_task()->get_auto_target_position_index()) < cal_center.size()) {    // 确保索引在范围内
			pos_target.position.x() = cal_center[do_shot_task->get_appochtarget_task()->get_auto_target_position_index()].point.x();
			pos_target.position.y() = cal_center[do_shot_task->get_appochtarget_task()->get_auto_target_position_index()].point.y();
			pos_target.position.z() = shot_halt_low;
			pos_target.radius = cal_center[do_shot_task->get_appochtarget_task()->get_auto_target_position_index()].diameters / 2.0f;    // 目标半径为检测到的物体半径
			pos_target.index = cal_center[do_shot_task->get_appochtarget_task()->get_auto_target_position_index()].cluster_id;
			return pos_target;
		} else {
			pos_target.position = Vector3f::Zero();
			pos_target.radius = 0.0f;
			pos_target.index = -1;

			return pos_target;
		}
        // return Vector4f(1.0f, 1.0f, 1.0f, 1.0f);
	};
	
	doshot_params.dynamic_target_image_callback = [this, do_shot_task]() -> Vector2f {
		static int circle_counter = 0;
		// 优先找圆桶
		if (!get_yolo_detector()->is_get_target(YOLO_TARGET_TYPE::CIRCLE) && circle_counter * get_wait_time() <= 0.6) {
			// 没检测到圆桶，计数器累加
			circle_counter++;
			return Vector2f::Zero();
		} else if (!get_yolo_detector()->is_get_target(YOLO_TARGET_TYPE::CIRCLE)) {
			// 如果已经投过弹了，继续找填充物
			if (do_shot_task->is_shot()) {
				return Vector2f(get_yolo_detector()->get_x(YOLO_TARGET_TYPE::STUFFED),
								get_yolo_detector()->get_y(YOLO_TARGET_TYPE::STUFFED));
			} else {
				// 没有目标，返回默认值
				return Vector2f::Zero();
			}
		} else {
			circle_counter = 0;
			RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000, "检测到圆桶目标，坐标: (%.1f, %.1f)", get_yolo_detector()->get_x(YOLO_TARGET_TYPE::CIRCLE), get_yolo_detector()->get_y(YOLO_TARGET_TYPE::CIRCLE));
			return Vector2f(get_yolo_detector()->get_x(YOLO_TARGET_TYPE::CIRCLE),
							get_yolo_detector()->get_y(YOLO_TARGET_TYPE::CIRCLE));
		} 
	};
	doshot_params.device_index = shot_counter;     // 投弹计数
	doshot_params.target_height = bucket_height;
	do_shot_task->setParameters(doshot_params);    // 在首次execute任务前设置参数

	SetPointTask::createTask("Goto_Scoutpoint")->set_config(
		SetPointTask::Parameters{
			dx_see,    // target_x
			dy_see,    // target_y
			see_halt,    // target_z
			0.0f,  // target_yaw
			7.5f,    // point_time (到达点位的最长等待时间)
			0.2f     // accuracy (点位到达精度)
		}
	);

	auto do_scout_waypoint_task = WayPointTask::createTask("Do_scout_waypoint");
	WayPoints do_scout_waypoint = WayPoints(
		"侦察区",
		std::vector<Vector4f>{
			Vector4f(0.0f, 4.8f, see_halt, 0.0f),
			Vector4f(-3.0f, 4.8f, see_halt, 0.0f),
			Vector4f(-3.0f, 0.2f, see_halt, 0.0f),
			Vector4f(3.0f, 4.8f, see_halt, 0.0f),
			Vector4f(3.0f, 0.2f, see_halt, 0.0f)
		}
	);
	do_scout_waypoint_task->set_config(
		do_scout_waypoint,
		WayPointTask::Parameters{
			dx_see,    // center_x
			dy_see,    // center_y
			// see_length - 2.0f,    // scope_length
			// see_width - 0.2f,    // scope_width
			3.5f,    // point_time (每个航点最大停留时间)
			0.05f     // accuracy (航点到达精度)
		}
	);

	// static state current_state = shot;
	switch (current_state) {
		case init:
			FlyState_init();
			current_state = takeoff;
			break;
		case takeoff:
			if (get_status_controller()->takeoff(get_z_pos(), 2.0, get_yaw())) {
				current_state = gotoshot;
			}
			break;
		case gotoshot:
			// 搜索阶段的逻辑
			accept(SetPointTask::getTask("Goto_shotpoint"));
			if (SetPointTask::getTask("Goto_shotpoint")->is_execute_finished()) {
				current_state = shot;
				do_shot_task->reset();
				do_shot_waypoint_task->reset();
			}
			break;
		case shot:
			// do_shot_task->set_device_index(shot_counter);     // 将doshot_appoch的目标设置为shot_counter，do_shot_task未使用（判断shot_counter和投弹时间解决未超时投弹的重复执行）
			task_manager.addTask(WaitTask::createTask("Wait_70s")->set_config(70.0, false));
			task_manager.addTask(do_shot_task->set_task_when_no_target(do_shot_waypoint_task));
			task_manager.execute();
            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000, "投弹次数: %d, 时间: %.1f/70.0", shot_counter, WaitTask::getTask("Wait_70s")->get_timer().elapsed());
			// 接近阶段的逻辑
			if (do_shot_task->is_execute_finished() || WaitTask::getTask("Wait_70s")->get_timer().elapsed() > 70.0f) {
				RCLCPP_INFO(node->get_logger(), "投弹任务完成，进入投放阶段");
				current_state = dropping; // 到达目标上方，进入投放阶段
			}
			break;
		case dropping:
			// 投放阶段的逻辑
			shot_counter++;
			do_shot_waypoint_task->get_timer().reset();
			// 问题： Doshot参数未持久化，在execute的init状态执行时更改参数，需在execute前设置参数
			// doshot_params.device_index = shot_counter;
			if (shot_counter >= 2) {
				// 处理达到最大投弹次数的逻辑
				get_servo_controller()->set_servo(11 + shot_counter - 1, get_servo_controller()->get_servo_open_position()); // 设置舵机位置，投弹
				current_state = scout;    // 完成投放，进入等待阶段
			} else {
				// 超时重复投弹
				get_servo_controller()->set_servo(11 + shot_counter - 1, get_servo_controller()->get_servo_open_position()); // 设置舵机位置，投弹
				do_shot_task->reset();    // 重置投弹任务
				current_state = shot;     // 继续投放下一个
			}
			break;
		case scout:
			accept(WaitTask::createTask("Wait_3s_shot")->set_config(3.0, true)->next_task(SetPointTask::getTask("Goto_Scoutpoint")->next_task(do_scout_waypoint_task)));
			if (do_scout_waypoint_task->is_execute_finished()) {
				current_state = land; // 返回搜索阶段
			}
			break;
		case land:
			accept(RTLLandTask::createTask()->setLandTask(DoLandTask::createTask()));
			if (RTLLandTask::getTask()->is_execute_finished()) {
				current_state = end; // 任务结束
			}
			break;
		case end:
			// accept(EndTask::createTask());
			rclcpp::shutdown();
			// 任务结束后的逻辑
			break;
		default:
			current_state = gotoshot; // 默认回到搜索阶段
			break;
	}

	
	if (get_status_controller()->mode == "LAND" &&
		// state_machine_.getCurrentState().getName() != "InitState" &&
		// state_machine_.getCurrentState().getName() != "TakeoffState" &&
		// state_machine_.getCurrentState().getName() != "EndState" &&
		current_state != takeoff &&
		current_state != end &&
		!print_info_)
	{
		RCLCPP_INFO(node->get_logger(), "飞行结束，进入结束状态");
		// state_machine_.transitionTo("end");
		current_state = end;
		return;
	}	

	terminal_control();
	// 发布当前状态
	publish_current_state(state_to_int(current_state));
	// 发布目标点
	get_yolo_detector()->publish_visualization_target();
}


void APMROS2Drone::FlyState_init()
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
	RCLCPP_INFO(node->get_logger(), "默认方向下角度：%f", default_yaw);
	RCLCPP_INFO(node->get_logger(), "罗盘方向投弹区起点 x: %f   y: %f    angle: %f", tx_shot, ty_shot, default_yaw);
	RCLCPP_INFO(node->get_logger(), "罗盘方向侦查区起点 x: %f   y: %f    angle: %f", tx_see, ty_see, default_yaw);
	rotate_world2start(tx_shot, ty_shot, x_shot, y_shot);
	rotate_world2start(tx_see, ty_see, x_see, y_see);
	RCLCPP_INFO(node->get_logger(), "飞机起飞方向投弹区起点 x: %f   y: %f    angle: %f", x_shot, y_shot, default_yaw);
	RCLCPP_INFO(node->get_logger(), "飞机起飞方向侦查区起点 x: %f   y: %f    angle: %f", x_see, y_see, default_yaw);
	rotate_world2local(tx_shot, ty_shot, x_shot, y_shot);
	rotate_world2local(tx_see, ty_see, x_see, y_see);
	RCLCPP_INFO(node->get_logger(), "当前朝向投弹区起点 x: %f   y: %f    angle: %f", x_shot, y_shot, default_yaw);
	RCLCPP_INFO(node->get_logger(), "当前朝向侦查区起点 x: %f   y: %f    angle: %f", x_see, y_see, default_yaw);

	get_camera_gimbal()->set_gimbal(
		-90.0, 0.0, 0.0
	);
	 
	// RCLCPP_INFO(node->get_logger(), "开始初始化舵机");
	// 初始化舵机操作 等待舵机初始化
	
	// while (!_servo_controller.client_->wait_for_service(std::chrono::seconds(1)))
	// {
	// 		if (!rclcpp::ok())
	// 		{
	// 				RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the Servo service. Exiting.");
	// 				return;
	// 		}
	// 		RCLCPP_INFO(node->get_logger(), "Servo Service not available, waiting again...");
	// }

  // servo_controller(12, 1050);
  // RCLCPP_INFO(node->get_logger(), "结束初始化舵机");

	// rclcpp::sleep_for(1s);


	if (!isfinite(get_x_pos()))
	{
		// THROTTLE表示节流的意思，以下代码节流时间间隔为 500 毫秒（即 5 秒）．
		RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500, "没有获取到位置数据，等待GPS信号...");
		return;
	}
	
	// 飞控的扩展卡尔曼滤波器（EKF3）已经为IMU（惯性测量单元）0和IMU1设置了起点。
	start = {get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()}; 
	// 飞控日志 AP: Field Elevation Set: 0m 设定当前位置的地面高度为0米，这对于高度控制和避免地面碰撞非常重要。
	// start_global = {get_lat(), get_lon(), get_alt()};		
	
	RCLCPP_INFO(node->get_logger(), "初始旋转角: %f", get_yaw());
	get_position_controller()->set_dt(get_wait_time()); // 设置执行周期（用于PID）

	// 重新设置家地址
	if (get_status_controller()->get_system_status() != StatusController::State__system_status::MAV_STATE_ACTIVE && get_z_pos() < 0.5) {
		get_status_controller()->set_home_position(get_yaw());
	} else {
		RCLCPP_WARN(node->get_logger(), "飞机已经起飞，无法重新设置家地址");
	}
	get_status_controller()->switch_mode("GUIDED");

	if (!debug_mode_)
		reset_wp_limits();
}

#ifdef PAL_STATISTIC_VISIBILITY
void APMROS2Drone::publish_statistics(){
	if (!stats_publisher_) {
        std::cerr << "[ERROR] stats_publisher_ is null in publish_statistics()" << std::endl;
        return;
	}
    if (!node) {
        RCLCPP_ERROR(rclcpp::get_logger("APMROS2Drone"), "ROS2 node is null!");
        return;
    }
	std::vector<pal_statistics_msgs::msg::Statistic> statistics;
	auto msg = pal_statistics_msgs::msg::Statistics();
	msg.header.stamp = node->get_clock()->now();
	msg.header.frame_id = "base_link";
	if (get_position_controller()) {
		static_pointer_cast<APMROS2PosController>(get_position_controller())->publish_statistic(statistics);
	}
	this->publish_statistic(statistics);
	msg.statistics = statistics;
	try {
		this->stats_publisher_->publish(msg);
	} catch (const std::exception &e) {
		RCLCPP_ERROR(rclcpp::get_logger("APMROS2Drone"), "Failed to publish statistics: %s", e.what());
	}
}

void APMROS2Drone::publish_statistic(std::vector<pal_statistics_msgs::msg::Statistic> &statistics){
	// auto current_state_stat = pal_statistics_msgs::msg::Statistic();
	// current_state_stat.name ="Offboard_State";
	// // current_state_stat.value = state_machine_.get_current_state().getIndex();
	// current_state_stat.value = state_machine_.getCurrentState().getIndex();
	// statistics.push_back(current_state_stat);
	
	auto yolo_x_circle_raw_stat = pal_statistics_msgs::msg::Statistic();
	yolo_x_circle_raw_stat.name ="yolo_x_circle_raw";
	yolo_x_circle_raw_stat.value = yolo_detector->get_raw_x(YOLO_TARGET_TYPE::CIRCLE);
	statistics.push_back(yolo_x_circle_raw_stat);

	auto yolo_y_circle_raw_stat = pal_statistics_msgs::msg::Statistic();
	yolo_y_circle_raw_stat.name ="yolo_y_circle_raw";
	yolo_y_circle_raw_stat.value = yolo_detector->get_raw_y(YOLO_TARGET_TYPE::CIRCLE);
	statistics.push_back(yolo_y_circle_raw_stat);

	auto yolo_x_circle_stat = pal_statistics_msgs::msg::Statistic();
	yolo_x_circle_stat.name ="yolo_x_circle";
	yolo_x_circle_stat.value = yolo_detector->get_x(YOLO_TARGET_TYPE::CIRCLE);
	statistics.push_back(yolo_x_circle_stat);

	auto yolo_y_circle_stat = pal_statistics_msgs::msg::Statistic();
	yolo_y_circle_stat.name ="yolo_y_circle";
	yolo_y_circle_stat.value = yolo_detector->get_y(YOLO_TARGET_TYPE::CIRCLE);
	statistics.push_back(yolo_y_circle_stat);

	auto yolo_x_h_raw_stat = pal_statistics_msgs::msg::Statistic();
	yolo_x_h_raw_stat.name ="yolo_x_h_raw";
	yolo_x_h_raw_stat.value = yolo_detector->get_raw_x(YOLO_TARGET_TYPE::H);
	statistics.push_back(yolo_x_h_raw_stat);

	auto yolo_y_h_raw_stat = pal_statistics_msgs::msg::Statistic();
	yolo_y_h_raw_stat.name ="yolo_y_h_raw";
	yolo_y_h_raw_stat.value = yolo_detector->get_raw_y(YOLO_TARGET_TYPE::H);
	statistics.push_back(yolo_y_h_raw_stat);

	auto yolo_x_h_stat = pal_statistics_msgs::msg::Statistic();
	yolo_x_h_stat.name ="yolo_x_h";
	yolo_x_h_stat.value = yolo_detector->get_x(YOLO_TARGET_TYPE::H);
	statistics.push_back(yolo_x_h_stat);

	auto yolo_y_h_stat = pal_statistics_msgs::msg::Statistic();
	yolo_y_h_stat.name ="yolo_y_h";
	yolo_y_h_stat.value = yolo_detector->get_y(YOLO_TARGET_TYPE::H);
	statistics.push_back(yolo_y_h_stat);

	auto tar_x_stat = pal_statistics_msgs::msg::Statistic();
	tar_x_stat.name = "tar_x_stat";
	tar_x_stat.value = target1.has_value() ? target1.value().x() : 0.0;
	statistics.push_back(tar_x_stat);

	auto tar_y_stat = pal_statistics_msgs::msg::Statistic();
	tar_y_stat.name = "tar_y_stat";
	tar_y_stat.value = target1.has_value() ? target1.value().y() : 0.0;
	statistics.push_back(tar_y_stat);

	for(size_t i = 0; i < 3; ++i) {
		auto point_stat = pal_statistics_msgs::msg::Statistic();
		point_stat.name = "surround_shot_point_" + std::to_string(i);
		point_stat.value = surround_shot_points[i].x();
		statistics.push_back(point_stat);
		point_stat.value = surround_shot_points[i].y();
		statistics.push_back(point_stat);
	}
}
#endif

void APMROS2Drone::accept(std::shared_ptr<TaskBase> visitor) {
	// std::cout << "APMROS2Drone accepting visitor." << std::endl;
	std::shared_ptr<TaskBase> final_visitor = visitor->final_task();
	// std::cout << "Final task to execute: " << final_visitor->get_string() << std::endl;
	final_visitor->visit(std::shared_ptr<APMROS2Drone>(this, [](APMROS2Drone*){})); 
	// visitor->final_task()->visit(std::shared_ptr<APMROS2Drone>(this, [](APMROS2Drone*){}));
}

// // 指定间隔时间循环执行航点
// // x，y为中心位置，length，width为航点的长宽，halt为高度，way_points为航点集合，description为航点描述
// bool APMROS2Drone::waypoint_goto_next(float x, float y, float length, float width, float halt, vector<Vector2f> &way_points, float time, int *count, const std::string &description)
// {
// 	static std::vector<Vector2f>::size_type surround_cnt = 0; // 修改类型
// 	float x_temp = 0.0, y_temp = 0.0;
// 	int count_n = count == nullptr? surround_cnt : *count;
// 	if(count!=nullptr)
// 		RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "(T 1s) w_g_n,counter: %d, time=%lf", *count, waypoint_timer_.elapsed());
// 	x_temp = x + (length * way_points[count_n].x());
// 	y_temp = y + (width * way_points[count_n].y());
// 	if (waypoint_timer_.elapsed() > time || (is_equal(get_x_pos(), x_temp, 0.2f) && is_equal(get_y_pos(), y_temp, 0.2f))) 
// 	{
// 		if (static_cast<std::vector<Vector2f>::size_type>(count_n) >= way_points.size())
// 		{
// 				RCLCPP_INFO(node->get_logger(), "w_g_n, %s已经全部遍历", description.c_str());
// 				count == nullptr? surround_cnt = 0 : *count = 0;
// 				// waypoint_timer_.reset();
// 				waypoint_timer_.set_start_time_to_default();
// 				return true;
// 		} else {
// 			count == nullptr? surround_cnt++ : (*count)++;
// 			RCLCPP_INFO(node->get_logger(), "w_g_n, %s点位%d x: %lf   y: %lf, timeout=%s", description.c_str(), count_n, x_temp, y_temp, (is_equal(get_x_pos(), x_temp, 0.2f) && is_equal(get_y_pos(), y_temp, 0.2f))? "true" : "false");

// 			rotate_global2stand(x_temp, y_temp, x_temp, y_temp);

// 			send_start_setpoint_command(x_temp, y_temp, halt, 0.0); // 发送本地坐标系下的航点指令
// 			// RCLCPP_INFO(node->get_logger(), "前往下一点");
// 			waypoint_timer_.reset();
// 		}
// 	}
// 	return false;
// }

// // 接近目标点
// bool APMROS2Drone::catch_target(PID::Defaults defaults, enum YOLO_TARGET_TYPE target, float tar_x, float tar_y, float tar_z, float tar_yaw, float accuracy){
// 	// RCLCPP_INFO(node->get_logger(), "--------------------\n\n读取pid参数: p: %f, i: %f, d: %f, ff: %f, dff: %f, imax: %f", defaults.p, defaults.i, defaults.d, defaults.ff, defaults.dff, defaults.imax);d_max_xy: %f, speed_max_z: %f, accel_max_x: %f, accel_max_z: %f", limits.speed_max_xy, limits.speed_max_z, limits.accel_max_xy, limits.accel_max_z);
// 	// 检查YOLO帧尺寸是否有效
// 	// if (get_yolo_detector()->get_cap_frame_width() <= 0 || get_yolo_detector()->get_cap_frame_height() <= 0) {
// 	//     RCLCPP_ERROR(node->get_logger(), "Invalid YOLO frame dimensions: width=%d, height=%d", 
// 	//                  get_yolo_detector()->get_cap_frame_width(), get_yolo_detector()->get_cap_frame_height());
// 	//     return false;
// 	// }
// 	// yolo返回值坐标系：x右y下（x_flip|y_flip = false），转换为飞机坐标系：x右y上
// 	float now_x = get_yolo_detector()->get_x(target);// get_yolo_detector()->get_cap_frame_width() - get_yolo_detector()->get_x(target);
// 	float now_y = get_yolo_detector()->get_cap_frame_height() - get_yolo_detector()->get_y(target); // get_yolo_detector()->get_y(target);
// 	tar_x =  tar_x;// get_yolo_detector()->get_cap_frame_width() - tar_x; // 目标x坐标
// 	tar_y =  get_yolo_detector()->get_cap_frame_height() - tar_y;// tar_y; // 目标y坐标
// 	// 检查坐标是否有效
// 	// if (!std::isfinite(now_x) || !std::isfinite(now_y) || !std::isfinite(tar_x) || !std::isfinite(tar_y)) {
// 	//     RCLCPP_ERROR(node->get_logger(), "Invalid coordinates detected");
// 	//     return false;
// 	// }
// 	rotate_xy(now_x, now_y, -get_world_yaw()); // 将目标坐标旋转到世界坐标系 headingangle_compass
// 	rotate_xy(tar_x, tar_y, -get_world_yaw()); // 将目标坐标旋转到世界坐标系 headingangle_compass
// 	float max_frame = std::max(get_yolo_detector()->get_cap_frame_width(), get_yolo_detector()->get_cap_frame_height());
// 	// RCLCPP_INFO(node->get_logger(), "catch_target_bucket: yaw: %f, default_yaw: %f, headingangle_compass: %f", get_yaw(), default_yaw, headingangle_compass);
// 	// RCLCPP_INFO(node->get_logger(), "catch_target_bucket: now_x: %f, now_y: %f, tar_x: %f, tar_y: %f", now_x, now_y, tar_x, tar_y);
// 	// RCLCPP_INFO(node->get_logger(), "catch_target_bucket: now_x: %f, now_y: %f, tar_x: %f, tar_y: %f", now_x / get_yolo_detector()->get_cap_frame_width(), now_y / get_yolo_detector()->get_cap_frame_height(), (tar_x) / get_yolo_detector()->get_cap_frame_width(), (tar_y) / get_yolo_detector()->get_cap_frame_height());
// 	// RCLCPP_INFO(node->get_logger(), "catch_target_bucket: now_z: %f, tar_z: %f, now_yaw: %f, tar_yaw: %f", get_z_pos(), tar_z, get_yaw(), tar_yaw);
// 	// RCLCPP_INFO(node->get_logger(), "catch_target: accuracy: %f, max_frame: %f", accuracy, max_frame);

// 	// bool trajectory_setpoint_world(Vector4f pos_now, Vector4f pos_target, PID::Defaults defaults, double accuracy, double yaw_accuracy, bool calculate_or_get_vel, float vel_x = DEFAULT_VELOCITY, float vel_y = DEFAULT_VELOCITY);
// 	get_position_controller()->trajectory_setpoint_world(
// 		Vector4f{tar_x / max_frame, tar_y / max_frame, get_z_pos(), get_yaw()}, // 当前坐标        get_world_yaw()  // 当前坐标
// 		Vector4f{now_x / max_frame, now_y / max_frame, tar_z, tar_yaw + default_yaw}, // 目标坐标  tar_yaw
// 		defaults,
// 		0.0,               				// 精度
// 		0.0 			 				// 偏航精度
// 		// true             				// 是否不使用飞机速度计算
// 		//	get_yolo_detector()->get_velocity_x(target) / max_frame, 	// 飞机速度
// 		//	get_yolo_detector()->get_velocity_y(target) / max_frame  	// 飞机速度
// 	);
// 	if (abs(now_x - tar_x) <= accuracy && abs(now_y - tar_y) <= accuracy)
// 	{
// 		RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "(T 1s) catch_target_bucket: 到达目标点, x_err: %f像素, y_err: %f像素", abs(now_x - tar_x), abs(now_y - tar_y));
// 		return true;
// 	}
// 	return false;
// }

// // 抵达桶上方
// // if(识别到桶=Doshot（到达正上方）){[if(到达正上方==true){...}}
// bool APMROS2Drone::Doshot(int shot_count, bool &shot_flag)
// {
// 	static enum class CatchState {
// 		init,
// 		fly_to_target,
// 		end
// 	} catch_state_ = CatchState::init;
// 	// static double time_find_start = 0;		 		// 开始时间
// 	// static float _t_time = 0; 					        // 接近目标时单轮执行时间
// 	static float find_duration = 0; 					        // 接近目标时执行时间
// 	static float radius = 0.1; 						    // 声明精度
// 	static float accuracy = 0.1;					    // 声明准确度
// 	static float shot_duration = 2; 					// 稳定持续时间
// 	static float shot_wait = 0.5; 						// 投弹后稳定时间
// 	static std::vector<TargetData> targets; 			// 声明目标x和y坐标
// 	static float tar_z = 1, tar_yaw = 0; 				// 声明目标偏航角（rad）
// 	// static bool shot_flag = false; 						// 投弹标志
// 	static std::vector<Vector3f> shot_point;		// 声明投弹点坐标
// 	const std::vector<std::string> targets_str = {"_r", "_l"};
// 	bool result = false;

// 	// 读取PID参数
// 	PID::Defaults defaults;
// 	defaults = PID::readPIDParameters("can_config.yaml", "pid");

// 	while(true){
// 		switch (catch_state_)
// 		{
// 		case CatchState::init:
// 		{
// 			get_position_controller()->reset_pid(); // 重置PID参数与配置
// 			RCLCPP_INFO(node->get_logger(), "Doshot: Init");
// 			PID::Defaults defaults;
// 			defaults = PID::readPIDParameters("can_config.yaml", "pid");
// 			PosController::Limits_t limits = get_position_controller()->readLimits("can_config.yaml", "limits");
// 			get_position_controller()->set_limits(limits);
// 			// 读取距离目标一定范围内退出的距离
// 			YAML::Node config = Readyaml::readYAML("can_config.yaml");
// 			radius = config["radius"].as<float>();
// 			accuracy = config["accuracy"].as<float>();
// 			shot_duration = config["shot_duration"].as<float>();
// 			shot_wait = config["shot_wait"].as<float>();
// 			tar_z = config["tar_z"].as<float>();
// 			// shot_point = {{0.045, 0.0, 0.10}, {-0.045, 0.0, 0.10}};
// 			shot_point.clear();
// 			targets.clear();
// 			for (size_t i = 0; i < targets_str.size(); i++)
// 			{
// 				float adjusted_x = config[std::string("shot_target_x").append(targets_str[i])].as<float>(0.0f) - drone_to_camera[0];
// 				float adjusted_y = config[std::string("shot_target_y").append(targets_str[i])].as<float>(0.0f) - drone_to_camera[1];
// 				float adjusted_z = config[std::string("shot_target_z").append(targets_str[i])].as<float>(0.0f) - drone_to_camera[2];
// 				shot_point.push_back(Vector3f(adjusted_x, adjusted_y, adjusted_z)); // 调整高度 待旋转
// 				std::cout << "shot_point_x" << targets_str[i] << ": " << shot_point[i].x() << std::endl;
// 				std::cout << "shot_point_y" << targets_str[i] << ": " << shot_point[i].y() << std::endl;
// 				std::cout << "shot_point_z" << targets_str[i] << ": " << shot_point[i].z() << std::endl;
// 			}
// 			for (size_t i = 0; i < targets_str.size(); i++)
// 			{
// 				TargetData target;
// 				target.category = std::string("circle").append(targets_str[i]);
// 				target.x = config[std::string("tar_x").append(targets_str[i])].as<float>(0.0f);
// 				target.y = config[std::string("tar_y").append(targets_str[i])].as<float>(0.0f);
// 				target.z = config[std::string("tar_z").append(targets_str[i])].as<float>(0.0f);
// 				target.x = (is_equal(target.x, 0.0f) ? get_yolo_detector()->get_cap_frame_width() / 2 : target.x);
// 				target.y = (is_equal(target.y, 0.0f) ? get_yolo_detector()->get_cap_frame_height() / 2 : target.y);
// 				target.z = (is_equal(target.z, 0.0f) ? tar_z : target.z);
// 				target.r = 1.0f; 
// 				target.g = 0.0f;
// 				target.b = 0.0f;
// 				target.fx = _camera_gimbal->fx;
// 				target.radius = radius;
// 				RCLCPP_INFO(node->get_logger(), "Doshot: tar_x: %f, tar_y: %f, tar_z: %f", target.x, target.y, target.z);
// 				targets.push_back(target);
// 			}

// 			RCLCPP_INFO(node->get_logger(), "Doshot: cap_frame_width: %d, cap_frame_height: %d, radius: %f, accuracy: %f, shot_duration: %f, shot_wait: %f", 
// 				get_yolo_detector()->get_cap_frame_width(), get_yolo_detector()->get_cap_frame_height(), radius, accuracy, shot_duration, shot_wait);

// 			// time_find_start = get_cur_time();
// 			// _t_time = time_find_start;
// 			find_duration = 0.0f; // 重置查找持续时间
// 			tar_yaw = 0;		            // 设置目标偏航角（rad）
// 			shot_flag = false;  // 重置投弹标志
// 			catch_state_ = CatchState::fly_to_target;
// 			continue; // 继续执行下一个case;
// 		}
// 		case CatchState::fly_to_target:
// 		{
// 			// double cur_shot_time = get_cur_time();
			
// 			// 检查targets数组是否为空
// 			if (targets.empty()) {
// 				RCLCPP_ERROR(node->get_logger(), "Doshot: targets array is empty!");
// 				catch_state_ = CatchState::end;
// 				continue;
// 			}
			
// 			int shot_index = (shot_count - 1) % targets.size(); // 计算当前投弹桶的索引，shot_count从1开始计数， tar_x.size()!=0
			
// 			// 验证索引有效性
// 			if (shot_index < 0 || shot_index >= static_cast<int>(targets.size()) || 
// 				shot_index >= static_cast<int>(shot_point.size())) {
// 				RCLCPP_ERROR(node->get_logger(), "Doshot: Invalid shot_index: %d, targets.size(): %zu, shot_point.size(): %zu", 
// 							shot_index, targets.size(), shot_point.size());
// 				catch_state_ = CatchState::end;
// 				continue;
// 			}
			
// 			for (size_t i = 0; i < targets.size(); i++)
// 			{
// 				targets[i].r = 1.0f; // 设置所有目标颜色为红色
// 				targets[i].g = 0.0f;
// 				targets[i].b = 0.0f;
// 				targets[i].relative_z = _camera_gimbal->get_position().z() - bucket_height; // 设置目标的高度为相机高度
// 			}


// 			// TargetData temp_target = targets[shot_index];

// 			// // 显示目标位置
// 			// if (static_cast<int>(cal_center.size()) > shot_index){
// 			// 	Vector3d world_point(cal_center[shot_index].point.x(), 
// 			// 						cal_center[shot_index].point.y(), 
// 			// 						cal_center[shot_index].point.z());
// 			// 	auto shot_center_opt = _camera_gimbal->worldToPixel(world_point);
// 			// 	if (shot_center_opt.has_value()) {
// 			// 		Vector2d shot_center = shot_center_opt.value();
// 			// 		temp_target.x = shot_center.x();
// 			// 		temp_target.y = shot_center.y();
// 			// 		temp_target.category = std::string("circle").append("_w2p");
// 			// 		get_yolo_detector()->append_target(temp_target);
// 			// 	}
// 			// }

// 			// Vector2d input_pixel(targets[shot_index].x, targets[shot_index].y);
// 			// auto mapped_center_opt = _camera_gimbal->mapPixelToVerticalDownCamera(input_pixel, bucket_height);
// 			// if (mapped_center_opt.has_value()) {
// 			// 	Vector2d shot_center = mapped_center_opt.value();
// 			// 	temp_target.x = shot_center.x();
// 			// 	temp_target.y = shot_center.y();
// 			// 	temp_target.category = std::string("circle").append("_p2p");
// 			// 	get_yolo_detector()->append_target(temp_target);
// 			// }

// 			// 确保shot_index在shot_point范围内
// 			std::vector<TargetData> t2p_targets;
// 			if (!targets.empty()) { // 确保targets不为空
// 				for(size_t i = 0; i < shot_point.size(); i++)
// 				{
// 					TargetData t2p_target = targets[0];
// 					if (i < static_cast<size_t>(shot_point.size())) {
// 						float rotated_x, rotated_y;
// 						// std::cout << "Doshot: shot_point[" << i << "]: " << shot_point[i].transpose() << std::endl;
// 						rotate_local2world(shot_point[i].x(), shot_point[i].y(), rotated_x, rotated_y);
// 						// std::cout << "Doshot: rotated_x: " << rotated_x << ", rotated_y: " << rotated_y << std::endl;
// 						Vector3d world_point_target(
// 							_camera_gimbal->get_position().x() + rotated_x,
// 							_camera_gimbal->get_position().y() + rotated_y,
// 							bucket_height + shot_point[i].z()
// 						);
// 						// std::cout << "Doshot: camera_gimbal position: " << _camera_gimbal->get_position().transpose() << " shot_point: " << shot_point[i].transpose() << " world_point_target: " << world_point_target.transpose() << std::endl;
// 						auto output_pixel_opt = _camera_gimbal->worldToPixel(world_point_target);
// 						// std::cout <<output_pixel_opt.has_value() << std::endl;
// 						if (output_pixel_opt.has_value()) {
// 							Vector2d output_pixel = output_pixel_opt.value();
// 							t2p_target.x = output_pixel.x();
// 							t2p_target.y = output_pixel.y();
// 							t2p_target.category = std::string("circle").append("_t2p").append(targets_str[i]);
// 							if (!debug_mode_) {
// 								uint8_t nearest_circle_index = 0;
// 								double min_distance = std::numeric_limits<double>::max();
// 								for (size_t j = 0; j < cal_center.size(); j++) {
// 									double new_min_distance = 
// 										std::pow(cal_center[j].point.x() - get_x_pos(), 2) +
// 										std::pow(cal_center[j].point.y() - get_y_pos(), 2);
// 									if (new_min_distance < min_distance) {
// 										min_distance = new_min_distance;
// 										nearest_circle_index = j;
// 									}
// 								}
// 								// 如果cal_center为空，设置默认半径，防止段错误
// 								if (cal_center.empty()){
// 									t2p_target.radius = 0.08;
// 								} else {
// 									t2p_target.radius = cal_center[nearest_circle_index].diameters / 2.0f * accuracy; // 设置目标半径为像素半径的百分比
// 								}
// 								// } else if (static_cast<int>(cal_center.size()) > shot_index){
// 								// 	t2p_target.radius = cal_center[shot_index].diameters / 2.0f * accuracy; // 设置目标半径为像素半径的百分比
// 								// }
// 								t2p_targets.push_back(t2p_target);
// 							} else { // 发布所有的目标
// 								t2p_target.radius = 0.15 / 2.0f * accuracy; // 设置目标半径为像素半径的百分比
// 								t2p_targets.push_back(t2p_target);
// 								t2p_target.radius = 0.20 / 2.0f * accuracy; // 设置目标半径为像素半径的百分比
// 								t2p_targets.push_back(t2p_target);
// 								t2p_target.radius = 0.25 / 2.0f * accuracy; // 设置目标半径为像素半径的百分比
// 								t2p_targets.push_back(t2p_target);
									
							
// 							}
// 						}
// 					}
// 				}
// 			}

// 			TargetData shot_index_target; // 当前投弹目标
// 			if(shot_index < static_cast<int>(t2p_targets.size()) && !t2p_targets.empty()){
// 				shot_index_target = t2p_targets[shot_index];
// 				t2p_targets.erase(t2p_targets.begin() + shot_index); // 移除当前投弹目标，避免重复添加
// 			}
// 			else if (shot_index < static_cast<int>(targets.size()) && !targets.empty()) {
// 				RCLCPP_INFO(node->get_logger(), "Doshot: 找不到目标在图像的映射，shot_index: %d, t2p_targets.size(): %zu, targets.size(): %zu", shot_index, t2p_targets.size(), targets.size());
// 				shot_index_target = targets[shot_index];
// 				// targets.erase(targets.begin() + shot_index); // 移除当前投弹目标，避免重复添加
// 			}
// 			else {
// 				// 如果没有有效目标，使用默认值
// 				shot_index_target = targets[0];
// 			}
// 			// 设置当前目标颜色为黄色
// 			shot_index_target.r = 1.0f;
// 			shot_index_target.g = 1.0f;
// 			shot_index_target.b = 0.0f;
// 			// std::cout << "Doshot: shot_index_target: " << shot_index_target.x << ", " << shot_index_target.y << ", " << shot_index_target.z  << ", " << shot_index_target.radius << std::endl;

// 			// 未投弹且无目标
// 			if (!shot_flag && !get_yolo_detector()->is_get_target(YOLO_TARGET_TYPE::CIRCLE))
// 			{
// 				RCLCPP_INFO(node->get_logger(), "Doshot: yolo未识别到桶，等待");
// 			} 
// 			// 已经投弹且无可选目标
// 			else if (shot_flag && !get_yolo_detector()->is_get_target(YOLO_TARGET_TYPE::STUFFED) && !get_yolo_detector()->is_get_target(YOLO_TARGET_TYPE::CIRCLE)){
// 				RCLCPP_INFO(node->get_logger(), "Doshot: 已投弹，yolo未识别到桶，原地等待");
// 				send_velocity_command(0, 0, 0, 0); // 停止飞行
// 			}
// 			// 接近目标
// 			else if (catch_target(
// 					defaults,
// 					YOLO_TARGET_TYPE::CIRCLE, // 目标类型
// 					// targets[shot_index].x, 
// 					// targets[shot_index].y, 
// 					shot_index_target.x,
// 					shot_index_target.y,
// 					shot_index_target.z, 
// 					tar_yaw, 
// 					shot_index_target.caculate_pixel_radius() // 目标精度
// 				))
// 			{
// 				if (!shot_flag) // 未投弹时接近目标时间累加
// 					find_duration += get_wait_time(); // 累加查找持续时间
// 				shot_index_target.r = 0.0f; // 设置当前目标颜色为绿色
// 				shot_index_target.g = 1.0f;
// 				shot_index_target.b = 0.0f;
// 				RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 100, "(T 0.1s) Doshot: Approach, Doshot, time = %fs", find_duration);
// 				// if(error_x<0.05 && error_y<0.05){
// 					// RCLCPP_INFO(node->get_logger(), "Arrive, Doshot");
// 					// catch_state_=CatchState::end;
// 				// } else 
// 				if(!shot_flag && find_duration >= shot_duration){ 
// 					RCLCPP_INFO(node->get_logger(), "Doshot: Approach, 投弹, time > %fs, tar_x = %f, tar_y = %f, tar_z = %f, tar_yaw = %f", 
// 						shot_duration, shot_index_target.x, shot_index_target.y, shot_index_target.z, tar_yaw);
// 					shot_flag = true; // 设置投弹标志
// 					_servo_controller->set_servo(11 + shot_index, servo_open_position); // 设置舵机位置，投弹
// 				} 
// 				else if (shot_flag) // 已投弹，shot_wait时间内继续等待
// 				{
// 					if (find_duration <= shot_duration + get_wait_time()) // 投弹后周期 重复投弹一次
// 					{
// 						RCLCPP_INFO(node->get_logger(), "Doshot: Arrive, 再次投弹, wait, time = %fs", find_duration - shot_duration);
// 						_servo_controller->set_servo(11 + shot_index, servo_open_position); // 重复投弹
// 					} else {
// 						RCLCPP_INFO(node->get_logger(), "Doshot: Arrive, 等待, wait, time = %fs", find_duration - shot_duration);
// 					}
// 				}			
// 			}
// 			// 未投弹且有目标且未接近目标
// 			else if (!shot_flag)
// 			{
// 				find_duration = 0.0f; // 重置查找持续时间
// 			}

// 			// 执行投弹命令后，如果查找到持续时间大于于投弹持续时间+等待时间
// 			if (shot_flag) 
// 			{
// 				find_duration += get_wait_time(); // 投弹后必定累加查找持续时间
// 				if (find_duration >= shot_duration + shot_wait) {
// 					RCLCPP_INFO(node->get_logger(), "Doshot: 投弹后等待, find_duration_time = %fs", find_duration);
// 					catch_state_ = CatchState::end;
// 					continue; // 直接跳到下一个状态;
// 				}
// 			}
// 			// get_yolo_detector()->append_targets(targets); // 将目标添加到YOLO中准备发布
// 			get_yolo_detector()->append_target(shot_index_target); // 将当前投弹目标添加到YOLO中准备发布
// 			get_yolo_detector()->append_targets(t2p_targets); // 将投弹到拍摄目标添加到YOLO中准备发布
// 			break;
// 		}
// 		case CatchState::end:
// 		{
// 			RCLCPP_INFO(node->get_logger(), "Doshot: end");
// 			// 重置所有静态变量
// 			catch_state_ = CatchState::init;
// 			get_position_controller()->reset_limits();
// 			// time_find_start = 0;
// 			result = true;
// 			break;
// 		}
// 		default:
// 			break;
// 		}
// 		break;
// 	}
// 	return result;
// }

// bool APMROS2Drone::Doland()
// {
// 	static Timer timer_ = Timer();
// 	static enum class LandState {
// 		init,
// 		land_to_target,
// 		end
// 	} land_state_ = LandState::init;
// 	static int surround_land = -3;
// 	static PID::Defaults defaults;
// 	static TargetData target;
// 	static double scout_x = 0.0, scout_y = 0.0, scout_halt = 3.0, accuracy = 0.3;
// 	double x_home, y_home;
// 	bool result = false;
// 	while(true){
// 		switch (land_state_)
// 		{
// 		case LandState::init:{
// 			get_position_controller()->reset_pid(); // 重置PID参数与配置
// 			// 读取PID参数
// 			defaults = PID::readPIDParameters("land_config.yaml", "pid");
// 			PosController::Limits_t limits = get_position_controller()->readLimits("land_config.yaml", "limits");
// 			get_position_controller()->set_limits(limits);
// 			YAML::Node config = Readyaml::readYAML("land_config.yaml");
// 			scout_halt = config["scout_halt"].as<double>();
// 			scout_x = config["scout_x"].as<double>();
// 			scout_y = config["scout_y"].as<double>();
// 			accuracy = config["accuracy"].as<double>();
// 			target.category = std::string("h_target");
// 			target.x = config["tar_x"].as<double>(0.0f);
// 			target.y = config["tar_y"].as<double>(0.0f);
// 			target.z = config["tar_z"].as<double>(0.0f);
// 			target.x = (is_equal(target.x, 0.0f) ? get_yolo_detector()->get_cap_frame_width() / 2 : target.x);
// 			target.y = (is_equal(target.y, 0.0f) ? get_yolo_detector()->get_cap_frame_height() / 2 : target.y);
// 			target.z = (is_equal(target.z, 0.0f) ? target.z : target.z);
// 			target.r = 0.0f; 
// 			target.g = 0.0f;
// 			target.b = 1.0f;
// 			target.fx = _camera_gimbal->fx;
// 			target.radius = accuracy;
// 			RCLCPP_INFO(node->get_logger(), "Doland");
// 			// rotate_global2stand(scout_x, scout_y, x_home, y_home);
// 			// RCLCPP_INFO(node->get_logger(), "返回降落准备点 x: %lf   y: %lf    angle: %lf", x_home, y_home, headingangle_compass);
// 			// rclcpp::sleep_for(std::chrono::seconds(6));
// 			rotate_global2stand(scout_x, scout_y + 0.3, x_home, y_home);
// 			RCLCPP_INFO(node->get_logger(), "返回降落点 x: %lf   y: %lf    angle: %lf", x_home, y_home, 0.0);
// 			send_start_setpoint_command(x_home, y_home, scout_halt, 0);
// 			timer_.reset();
// 			timer_.set_timepoint();
// 			land_state_ = LandState::land_to_target;
// 			continue; // 直接跳到下一个状态;
// 		}
// 		case LandState::land_to_target:{
// 			if (timer_.elapsed() > 19 || surround_land > 3 || get_z_pos() < target.z + 0.1) // 降落时间超过39秒，或者降落高度小于目标高度
// 			{
// 				land_state_ = LandState::end;
// 				continue; // 直接跳到下一个状态;
// 			}
			
// 			TargetData t2p_target = target;
// 			// double rotated_x, rotated_y;
// 			// rotate_local2world(drone_to_camera.x(), drone_to_camera.y(), rotated_x, rotated_y);
// 			// Vector3d world_point_target(
// 			// 	_camera_gimbal->get_position().x() - rotated_x,
// 			// 	_camera_gimbal->get_position().y() - rotated_y,
// 			// 	0.0f
// 			// );
// 			// auto output_pixel_opt = _camera_gimbal->worldToPixel(world_point_target);
// 			// if (output_pixel_opt.has_value()) {
// 			// 	Vector2d output_pixel = output_pixel_opt.value();
// 			// 	t2p_target.r = 0.0f; 
// 			// 	t2p_target.g = 0.0f;
// 			// 	t2p_target.b = 1.0f;
// 			// 	t2p_target.x = output_pixel.x();
// 			// 	t2p_target.y = output_pixel.y();
// 			// 	t2p_target.category = std::string("h").append("_t2p");
// 			// 	t2p_target.radius = accuracy;
// 			// } else {
// 				// RCLCPP_ERROR(node->get_logger(), "Doland: worldToPixel failed, using read target position");
// 			t2p_target.x = target.x;
// 			t2p_target.y = target.y;
// 			// }

// 			if (!get_yolo_detector()->is_get_target(YOLO_TARGET_TYPE::H)) // yolo未识别到YOLODetector::TARGET_TYPE::H   (YOLO_TARGET_TYPE::CIRCLE)
// 			{
// 				if (timer_.get_timepoint_elapsed() > 2.0)
// 				{
// 						RCLCPP_INFO(node->get_logger(), "Doland: surround_land = %d", surround_land);
// 						rotate_global2stand(scout_x + static_cast<double>(surround_land) * 1.0, scout_y, x_home, y_home);
// 						RCLCPP_INFO(node->get_logger(), "Doland: land点 x: %lf   y: %lf    angle: %lf", x_home, y_home, 0.0); // 开始执行程序+x_home位置
// 						send_start_setpoint_command(x_home, y_home, scout_halt, 0);
// 						timer_.set_timepoint();
// 						surround_land++;
// 				}
// 			}
// 			else
// 			{
// 				RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "(T 1s) Doland: 看见H了，执行Doland");
// 				if (catch_target(
// 						defaults,
// 						YOLO_TARGET_TYPE::H, // 目标类型
// 						t2p_target.x,
// 						t2p_target.y,
// 						t2p_target.z, 
// 						0, 
// 						t2p_target.caculate_pixel_radius() // 目标精度
// 					))
// 				{
// 					RCLCPP_INFO(node->get_logger(), "Doland: 到达降落点");
// 					land_state_ = LandState::end;
// 					continue; // 直接跳到下一个状态;
// 				}
// 				else
// 				{
// 					RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "(T 1s) Doland: 未到达降落点");
// 					// RCLCPP_INFO(node->get_logger(), "Doland: 未到达降落点");
// 				}
// 			}
// 			get_yolo_detector()->append_target(t2p_target); // 将目标添加到YOLO中准备发布
// 			break;
// 		}
// 		case LandState::end:{
// 			send_velocity_command_with_time(0, 0, -0.2, 0, 1);
// 			RCLCPP_INFO(node->get_logger(), "Doland: 降落");
// 			land_state_ = LandState::init;
// 			surround_land = 0;
// 			timer_.set_timepoint();
// 			result = true;
// 			break;
// 		}
// 		default:
// 			break;
// 		}
// 		break;
// 	}
// 	return result;
// }

// // APMROS2Drone.h #define TRAIN_PID
// struct PIDDataPoint
// {
// 	double p;
// 	double i;
// 	double d;
// 	double error;
// 	double time;
// } data_point;

// // 抵达桶上方
// // if(识别到桶=catch_target_bucket（到达正上方）){[if(到达正上方==true){...}}
// bool APMROS2Drone::autotune(bool &result, enum YOLO_TARGET_TYPE target)
// {
// 	static enum class CatchState {
// 		init,
// 		fly_to_target,
// 		end
// 	} catch_state_ = CatchState::init;
// 	static double time_find_start = 0; // 开始时间
// 	static float tar_z = 1;	// 声明目标高度（m）
// 	static float tar_yaw = 0; // 声明目标偏航角（rad）
// 	static float dt = 0.1;								 // 声明执行周期（s）
// 	static float last_time = get_cur_time(); // 声明上次执行时间
// 	static float accuracy = 0.1; // 声明精度

// 	// yolo未识别到桶
// 	if (is_equal(get_yolo_detector()->get_x(target), (float)0) && is_equal(get_yolo_detector()->get_y(target), (float)0))
// 	{
// 		RCLCPP_INFO(node->get_logger(), "Doshot: yolo未识别到桶");
// 		result = false;
// 		return false;
// 	}
// 	RCLCPP_INFO(node->get_logger(), "Doshot: x: %f, y: %f", get_yolo_detector()->get_x(target), get_yolo_detector()->get_y(target));

// 	switch (catch_state_)
// 	{
// 	case CatchState::init:
// 	{
// 		// 读取PID参数
// 		PID::Defaults defaults = PID::readPIDParameters("can_config.yaml", "pid_bucket");
// 		PosController::Limits_t limits = get_position_controller()->readLimits("can_config.yaml", "limits");
// 		get_position_controller()->set_limits(limits);
// 		// 读取距离目标一定范围内退出的距离
// 		YAML::Node config = Readyaml::readYAML("can_config.yaml");
// 		accuracy = config["accuracy"].as<float>();

// 		RCLCPP_INFO(node->get_logger(), "Doshot: Init1");
// 		data_point.p = defaults.p;
// 		data_point.i = defaults.i;
// 		data_point.d = defaults.d;
// 		data_point.error = 0;
// 		data_point.time = 1000;
// 		time_find_start = get_cur_time();
// 		last_time = get_cur_time();
// 		tar_z = 1.5;							 // 设置目标高度（m）
// 		tar_yaw = 0;		 // 设置目标偏航角（rad）
// 		dt = 0.25;							 // 设置执行周期（s）
// 		get_position_controller()->set_dt(dt); // 设置执行周期（用于PID）

// 		RCLCPP_INFO(node->get_logger(), "Doshot: Init2");
// 		RCLCPP_INFO(node->get_logger(), "--------------------\n\n读取pid_bucket: p: %f, i: %f, d: %f, ff: %f, dff: %f, imax: %f", defaults.p, defaults.i, defaults.d, defaults.ff, defaults.dff, defaults.imax);
// 		RCLCPP_INFO(node->get_logger(), "n读取limits: speed_max_xy: %f, speed_max_z: %f, accel_max_x: %f, accel_max_z: %f", limits.speed_max_xy, limits.speed_max_z, limits.accel_max_xy, limits.accel_max_z);

// 		catch_state_ = CatchState::fly_to_target;
// 		break;
// 	}
// 	case CatchState::fly_to_target:
// 	{
// 		// 以 dt 为周期，执行一次 PID 控制
// 		RCLCPP_INFO(node->get_logger(), "当前时间: %f, 上次时间: %f, 执行周期: %f, 是否执行: %d", get_cur_time(), last_time, dt, get_cur_time() - last_time < dt);
// 		if (get_cur_time() - last_time < dt)
// 		{
// 			result = false;
// 			return true;
// 		}
// 		last_time = get_cur_time();

// 		// yolo返回值坐标系：x右y上，转换为飞机坐标系：x左y上
// 		float now_x = get_yolo_detector()->get_cap_frame_width()-get_yolo_detector()->get_x(target);
// 		float now_y = get_yolo_detector()->get_cap_frame_height()-get_yolo_detector()->get_y(target);
// 		float tar_x = get_yolo_detector()->get_cap_frame_width()-get_yolo_detector()->get_cap_frame_width() / 2; // /10
// 		float tar_y = get_yolo_detector()->get_cap_frame_height()-get_yolo_detector()->get_cap_frame_height() / 2; // /3
// 		rotate_xy(now_x, now_y, (get_yaw() - default_yaw));
// 		rotate_xy(tar_x, tar_y, (get_yaw() - default_yaw));
// 		RCLCPP_INFO(node->get_logger(), "Doshot: now_x: %f, now_y: %f, tar_x: %f, tar_y: %f", now_x, now_y, tar_x, tar_y);
// 		RCLCPP_INFO(node->get_logger(), "Doshot: now_x: %f, now_y: %f, tar_x: %f, tar_y: %f", now_x / get_yolo_detector()->get_cap_frame_width(), now_y / get_yolo_detector()->get_cap_frame_height(), (tar_x) / get_yolo_detector()->get_cap_frame_width(), (tar_y) / get_yolo_detector()->get_cap_frame_height());
// 		static float _t_time = get_cur_time();

// 		(void)accuracy;
// 		if (!get_position_controller()->auto_tune 
// 				(
// 						Vector4f{now_x / get_yolo_detector()->get_cap_frame_width(), now_y / get_yolo_detector()->get_cap_frame_height(), get_z_pos(), get_yaw()},
// 						Vector4f{(tar_x) / get_yolo_detector()->get_cap_frame_width(), (tar_y) / get_yolo_detector()->get_cap_frame_height(), tar_z, tar_yaw},
// 						(uint32_t)(dt * 1000),
// 						true,
// 						true,
// 						false,
// 						false
// 					)
// 				)
// 		{
// 			double error_x = abs(now_x / get_yolo_detector()->get_cap_frame_width() - tar_x / get_yolo_detector()->get_cap_frame_width());
// 			double error_y = abs(now_y / get_yolo_detector()->get_cap_frame_height() - tar_y / get_yolo_detector()->get_cap_frame_height());
// 			data_point.error += error_x + error_y;
// 		}
// 		else
// 		{
// 			RCLCPP_INFO(node->get_logger(), "Arrive, Doshot, time = %f", get_cur_time() - _t_time);
// 			data_point.time = _t_time - get_cur_time();
// 			_t_time = get_cur_time();
// 			catch_state_ = CatchState::end;
// 		}

// 		if (
// 				get_cur_time() - time_find_start > 1200 //>1200s
// 		)
// 		{
// 			catch_state_ = CatchState::end;
// 		}
// 		break;
// 	}
// 	case CatchState::end:
// 	{
// 		static double end_time = get_cur_time(); // 记录结束时间
// 	if (get_cur_time() - end_time < 3.0) {   // 非阻塞等待3秒
// 		result = false;
// 		return true;
// 	}
// 		RCLCPP_INFO(node->get_logger(), "error:%lf,time:%lf", data_point.error, data_point.time);
// 		get_position_controller()->reset_limits();
// 		RCLCPP_INFO(node->get_logger(), "Arrive, 投弹");

// 	// 重置所有静态变量
// 	catch_state_ = CatchState::init;
// 	time_find_start = 0;
// 	tar_z = 1.0;
// 	tar_yaw = 0;
// 	dt = 0.1;
// 	last_time = get_cur_time();

// 		result = true;
// 		return true;
// 		break;
// 	}
// 	default:
// 		break;
// 	}
// 	RCLCPP_INFO(node->get_logger(), "catch_target_bucket: end");
// 	result = false;
// 	return true;
// }


// 发布状态
void APMROS2Drone::publish_current_state(int state)
{
  // auto message = std_msgs::msg::String();
	auto message = std_msgs::msg::Int32();
  // message.data = "Current state: " + std::to_string(static_cast<int>(state_machine_.current_state_));
	message.data = state;// state_machine_.getCurrentState().getPublishIndex();
  state_publisher_->publish(message);
}

void APMROS2Drone::terminal_control()  {
    RCLCPP_INFO_ONCE(rclcpp::get_logger("TerminalControlState"), "执行终端控制状态");
    // 终端控制逻辑可以在这里实现  char key = 0;
    static std::string input = "";

    if (_kbhit()) // 检查是否有按键输入
    {
        char key = _getch(); // 获取按键输入
        // auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
        // 特殊状态处理（起飞解锁前）
        auto apm_sta_ctl = std::static_pointer_cast<APMROS2StatusController>(get_status_controller());
        if (apm_sta_ctl && apm_sta_ctl->state_ == APMROS2StatusController::TakeoffState::wait_for_takeoff_command){
            if (key == '\n') // 检查是否按下回车键
            {
                apm_sta_ctl->takeoff_command = true; // 设置起飞命令
            }
            else if (key == 'q') // 检查是否按下q键
            {
                RCLCPP_INFO(get_node()->get_logger(), "退出程序");
                // stateMachine.transitionTo(EndState::getInstance()); // 切换到结束状态
            }
            else if (key != 0)
            {
                RCLCPP_INFO(get_node()->get_logger(), "无效输入，请按回车键解锁无人机或按q键退出程序");
            }
            return;
        }
        // // 处理多字符输入
        // if (key == '\n' || key == '\r') { // 按下回车，尝试解析命令
        // std::string upperInput = input;
        // std::transform(upperInput.begin(), upperInput.end(), upperInput.begin(), ::toupper);
        // const auto& allStates = stateMachine.getAllStates();
        // auto it = allStates.find(upperInput);
        // if (it != allStates.end()) {
        //     stateMachine.transitionTo(it->second->getName());
        //     RCLCPP_INFO(get_node()->get_logger(), "切换到状态: %s", upperInput.c_str());
        // } else {
        //     RCLCPP_INFO(get_node()->get_logger(), "无效指令: %s", input.c_str());
        // }
        // input.clear();
        // } else if (key == '\b' && !input.empty()) { // 处理退格
        // input.pop_back();
        //     } else if (key == '\t') { // Tab键自动补全
        //         std::string upperInput = input;
        //         std::transform(upperInput.begin(), upperInput.end(), upperInput.begin(), ::toupper);
        //         std::vector<std::string> candidates;
        //         auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
        //         const auto& allStates = stateMachine.getAllStates();
        //         for (const auto& kv : allStates) {
        //                 if (kv.first.find(upperInput) == 0) { // 前缀匹配
        //                         candidates.push_back(kv.first);
        //                 }
        //         }
        //         if (candidates.size() == 1) {
        //                 input = candidates[0];
        //                 RCLCPP_INFO(get_node()->get_logger(), "自动补全: %s", input.c_str());
        //         } else if (candidates.size() > 1) {
        //                 std::string msg = "可选项: ";
        //                 for (const auto& s : candidates) msg += s + " ";
        //                 RCLCPP_INFO(get_node()->get_logger(), "%s", msg.c_str());
        //         }
        // } else if (key != 0) {
        // input += key; // 将按键添加到输入字符串中
        // }
    }
}

void APMROS2Drone::calculate_target_position()
{
// detection2d_array_sub_ = node->create_subscription<vision_msgs::msg::Detection2DArray>(
//             "detection2d_array", 10,
//             std::bind(&ROS2YOLODetector::detection2d_array_callback, this, std::placeholders::_1)
//         );
	// // 调试输出：像素坐标和相机位置
	// std::cout << "相机当前位置: (" << get_camera()->get_position().x() << ", " << get_camera()->get_position().y() << ", " << get_camera()->get_position().z() << ")" << std::endl;
	// std::cout << "相机旋转角度: roll=" << get_camera()->get_camera_relative_rotation().x() << " pitch=" << get_camera()->get_camera_relative_rotation().y() << " yaw=" << get_camera()->get_camera_relative_rotation().z() << std::endl;
	// std::cout << "相机内参: fx=" << get_camera()->get_fx() << " fy=" << get_camera()->get_fy() << " cx=" << get_camera()->get_cx() << " cy=" << get_camera()->get_cy() << std::endl;

	std::vector<vision_msgs::msg::BoundingBox2D> raw_circles = get_yolo_detector()->get_raw_targets(YOLO_TARGET_TYPE::CIRCLE);
	if (debug_mode_) {
		vision_msgs::msg::BoundingBox2D debug_circle;
		debug_circle.center.position.x = 665;
		debug_circle.center.position.y = 470;
		debug_circle.size_x = 10;
		debug_circle.size_y = 10;
		raw_circles.push_back(debug_circle);
	}
	for (const auto& circle : raw_circles) 
	{
		// std::cout << "检测到的像素坐标: (" << circle.center.position.x << ", " << circle.center.position.y << ")" << std::endl;
		
		this->target1 = get_camera()->pixelToWorldPosition(
			Vector2d(circle.center.position.x, circle.center.position.y), 
			bucket_height // 桶顶高度
		);
		double avg_size = (circle.size_x + circle.size_y) / 2.0;
		double distance_to_bucket = get_camera()->get_position().z() - bucket_height;
		double diameter = 0.0;
		if (distance_to_bucket > 0.01) { // 防止除零
			diameter = get_camera()->calculateRealDiameter(avg_size, distance_to_bucket);
		}
		if (target1.has_value()) {
			// RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "(T 1s) Example 1 - Target position: %f, %f, %f. Diameter: %f",
			// 	target1->x(), target1->y(), target1->z(), diameter);
			// RCLCPP_INFO(get_node()->get_logger(), "Example 1 - Target position: %f, %f, %f. Diameter: %f",
			// 	target1->x(), target1->y(), target1->z(), diameter);
			Circles circle;
			circle.point = Vector3d(target1->x(), target1->y(), target1->z());
			circle.cluster_id = 0;
			circle.original_index = 0;
			circle.diameters = diameter;
			Target_Samples.push_back(circle);
		}
		else {
			RCLCPP_WARN(get_node()->get_logger(), "Example 1 - 无效的目标位置");
		}
	}
	if(!Target_Samples.empty() && (current_state == init || current_state == shot || current_state == gotoshot))
	{
		this->cal_center = Clustering(Target_Samples);
		if (!cal_center.empty()) {
			std::ostringstream ss;
			ss << "(T 2s) \n";
			for (size_t i = 0; i < cal_center.size(); ++i) {
				ss << "侦查点坐标 " << i << ": x: " << cal_center[i].point.x() 
					<< ", y: " << cal_center[i].point.y() 
					<< ", n_x: " << surround_shot_points[i].x() 
					<< ", n_y: " << surround_shot_points[i].y() 
					<< ", d: " << cal_center[i].diameters << "\n";
			}
			RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000, "%s", ss.str().c_str());
			// RCLCPP_INFO_STREAM(get_node()->get_logger(), ss.str());
			// cal_center[0].diameters = 0.25;
			// if (cal_center.size() > 1)
			// {
			// 	cal_center[1].diameters = 0.20;
			// }
			// if (cal_center.size() > 2)
			// {
			// 	cal_center[2].diameters = 0.15;
			// }
		}
		uint8_t shot_count = 0;
		std::stringstream ss;
		ss << "(T 2s) \n";
		for (size_t i = 0; i < shoted_cluster_ids.size(); i++) {
			size_t index = static_cast<size_t>(-1);
			for (size_t j = 0; j < cal_center.size(); j++) {
				if (static_cast<size_t>(cal_center[j].cluster_id) == shoted_cluster_ids[i]) {
					index = j;
					break;
				}
			}
			if (index == static_cast<size_t>(-1)) {
				RCLCPP_WARN(get_node()->get_logger(), "侦查目标已投弹，但在当前目标列表中未找到对应的cluster_id: %ld", shoted_cluster_ids[i]);
				continue;
			}
			ss << "侦查目标已投弹: " << i << ", x: " << cal_center[index].point.x() 
				<< ", y: " << cal_center[index].point.y() << "\n";
			// cal_center.erase(cal_center.begin() + index);  // 不去除，保留已投弹目标
		}
		for(size_t i = 0; i < cal_center.size(); ++i)
		{
			double tx, ty;
			rotate_stand2global(cal_center[i].point.x() - get_x_home_pos(), cal_center[i].point.y() - get_y_home_pos(), tx, ty);
			if (tx < dx_shot - shot_length_max / 2 - 1.5 || tx > dx_shot + shot_length_max / 2 + 1.5 ||
				ty < dy_shot - 1.5 || ty > dy_shot + shot_width_max + 1.5) {
				// RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000, "(T 2s)侦查点坐标异常，跳过: %zu, x: %f, y: %f, tx: %f, ty: %f", i, cal_center[i].point.x(), cal_center[i].point.y(), tx, ty);
				// RCLCPP_WARN(get_node()->get_logger(), "侦查点坐标异常，跳过: %zu, x: %f, y: %f, tx: %f, ty: %f", i, cal_center[i].point.x(), cal_center[i].point.y(), tx, ty);
				ss << "侦查点坐标异常，跳过: " << i << ", x: " << cal_center[i].point.x() 
					<< ", y: " << cal_center[i].point.y() 
					<< ", tx: " << tx 
					<< ", ty: " << ty << "\n";
				cal_center.erase(cal_center.begin() + i);
				i--;    // 调整索引以适应删除后的数组
				continue;
			}
			// 从大到小排列
			sort(cal_center.begin(), cal_center.end(), [this](const Circles& a, const Circles& b) {
				return this->shot_big_target ? a.diameters > b.diameters : a.diameters < b.diameters;
			});

			surround_shot_points[shot_count] = Vector2f((tx - dx_shot) / shot_length_max, (ty - dy_shot) / shot_width_max);
			ss << "侦查点坐标 " << i << ": x: " << cal_center[i].point.x() 
				<< ", y: " << cal_center[i].point.y() 
				<< ", n_x: " << surround_shot_points[shot_count].x() 
				<< ", n_y: " << surround_shot_points[shot_count].y() 
				<< ", d: " << cal_center[i].diameters << ", cluster_id: " << cal_center[i].cluster_id << "\n";
		}
		RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000, "%s", ss.str().c_str());
	}
}