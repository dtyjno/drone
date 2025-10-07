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
#include "../task/DoLandTask.h"
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
		appoch_params.dynamic_target_image_callback = [this]() -> AppochTargetTask::ImageTargetData {
			// return Vector2f(get_yolo_detector()->get_x(YOLO_TARGET_TYPE::CIRCLE),
			// 				get_yolo_detector()->get_y(YOLO_TARGET_TYPE::CIRCLE));
			AppochTargetTask::ImageTargetData data;
			data.has_target = true;
			data.data = Vector2f(100, 100);
			return data;
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
			Vector4f( 0.0f , 4.5f, shot_halt_surround, 0.0f),
			Vector4f(-0.75f, 3.4f, shot_halt_surround, 0.0f),
			Vector4f(-0.75f, 1.7f, shot_halt_surround, 0.0f),
			Vector4f( 0.0f , 1.0f, shot_halt_surround, 0.0f),
			Vector4f( 0.75f, 1.7f, shot_halt_surround, 0.0f),
			Vector4f( 0.75f, 3.4f, shot_halt_surround, 0.0f),
			Vector4f( 0.0f , 4.5f, shot_halt_surround, 0.0f),
			Vector4f( 1.5f , 4.5f, shot_halt_surround, 0.0f),
			Vector4f(-1.5f , 3.4f, shot_halt_surround, 0.0f),
			Vector4f( 1.5f , 1.7f, shot_halt_surround, 0.0f),
			Vector4f(-1.5f , 0.5f, shot_halt_surround, 0.0f)
		}
	);
	do_shot_waypoint_task->set_config(
		do_shot_waypoint,
		WayPointTask::Parameters{
			dx_shot,    // center_x
			dy_shot,    // center_y
			// shot_length - 3.5f,    // scope_length
			// shot_width,    // scope_width
			4.0f,    // point_time (每个航点停留时间)
			0.2f     // accuracy (航点到达精度)
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
	
	doshot_params.dynamic_target_image_callback = [this, do_shot_task]() -> AppochTargetTask::ImageTargetData {
		static int circle_counter = 0;
		float wait_time = 2.0;
		AppochTargetTask::ImageTargetData result;
		// 命中则不等待
		// if (!do_shot_task->miss_flag) {
		// 	wait_time = 0;
		// }
		// 优先找圆桶
		// std::cout << "寻找圆桶目标 circle_counter: " << circle_counter << std::endl;
		if (!get_yolo_detector()->is_get_target(YOLO_TARGET_TYPE::CIRCLE) && circle_counter * get_wait_time() <= wait_time) {
			// 没检测到圆桶，计数器累加
			circle_counter++;
			// return Vector2f::Zero();
			result.data = Vector2f(get_yolo_detector()->get_x(YOLO_TARGET_TYPE::CIRCLE),
				get_yolo_detector()->get_y(YOLO_TARGET_TYPE::CIRCLE));
			result.has_target = false;
		} else if (!get_yolo_detector()->is_get_target(YOLO_TARGET_TYPE::CIRCLE)) {
			// // 如果已经投过弹了，继续找填充物
			// if (do_shot_task->is_shot()) {
			// 	circle_counter = 0;
			// 	return Vector2f(get_yolo_detector()->get_x(YOLO_TARGET_TYPE::STUFFED),
			// 					get_yolo_detector()->get_y(YOLO_TARGET_TYPE::STUFFED));
			// } else {
			// 没有目标，返回默认值
			result.data = Vector2f::Zero();
			result.has_target = true;
			// return Vector2f::Zero();
			// }
		} else {
			wait_time = 0.7;
			circle_counter = 0;
			RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000, "检测到圆桶目标，坐标: (%.1f, %.1f)", get_yolo_detector()->get_x(YOLO_TARGET_TYPE::CIRCLE), get_yolo_detector()->get_y(YOLO_TARGET_TYPE::CIRCLE));
			// return Vector2f(get_yolo_detector()->get_x(YOLO_TARGET_TYPE::CIRCLE),
			// 				get_yolo_detector()->get_y(YOLO_TARGET_TYPE::CIRCLE));
			result.data = Vector2f(get_yolo_detector()->get_x(YOLO_TARGET_TYPE::CIRCLE),
				get_yolo_detector()->get_y(YOLO_TARGET_TYPE::CIRCLE));
			result.has_target = true;
		}
		return result;
		// // 走航点时出现pid位置界限需要规范
		// return Vector2f(get_yolo_detector()->get_x(YOLO_TARGET_TYPE::CIRCLE),
		//  				get_yolo_detector()->get_y(YOLO_TARGET_TYPE::CIRCLE));
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
			Vector4f(0.0f, 4.5f, see_halt, 0.0f),
			Vector4f(-3.0f, 0.2f, see_halt, 0.0f),
			Vector4f(-3.0f, 4.5f, see_halt, 0.0f),
			Vector4f(3.0f, 0.2f, see_halt, 0.0f),
			Vector4f(3.0f, 4.5f, see_halt, 0.0f)
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
			0.10f     // accuracy (航点到达精度)
		}
	);
	auto do_land_task = DoLandTask::createTask("Do_land");
	DoLandTask::Parameters doland_params;
	doland_params.fx = get_camera()->get_fx();
	doland_params.dynamic_target_image_callback = [this]() -> AppochTargetTask::ImageTargetData {
		static float h_x = 0.0f, h_y = 0.0f;
		AppochTargetTask::ImageTargetData result;
		// std::cout <<  "dynamic_target_image_callback" << Vector2f(get_yolo_detector()->get_x(YOLO_TARGET_TYPE::H),
		//  				get_yolo_detector()->get_y(YOLO_TARGET_TYPE::H)).transpose() << std::endl;
		h_x = get_yolo_detector()->get_x(YOLO_TARGET_TYPE::H);
		h_y = get_yolo_detector()->get_y(YOLO_TARGET_TYPE::H);
		if (get_yolo_detector()->is_get_target(YOLO_TARGET_TYPE::H)) {
			// RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000, "检测到降落桩目标，坐标: (%.1f, %.1f)", h_x, h_y);
			result.data = Vector2f(h_x, h_y);
			result.has_target = false;
		} else {
			result.data = Vector2f(get_yolo_detector()->get_x(YOLO_TARGET_TYPE::H),
							get_yolo_detector()->get_y(YOLO_TARGET_TYPE::H));
			result.has_target = true;
		}
		return result;
	};
	do_land_task->setParameters(doland_params);

	// static state current_state =setParameters shot;
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
			accept(WaitTask::createTask("Wait_3s_shot")->set_config(3.0, true)->next_task(SetPointTask::getTask("Goto_Scoutpoint"))->next_task(do_scout_waypoint_task));
			if (do_scout_waypoint_task->is_execute_finished()) {
				current_state = land; // 返回搜索阶段
			}
			break;
		case land:
			accept(RTLLandTask::createTask()->setLandTask(do_land_task));
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
