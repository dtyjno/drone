#include "DoshotState.h"
#include "../APMROS2Drone.h"
#include "../../stateMachine/StateMachine.h"
#include "GotoScoutPointState.h"

void DoshotState::executeImpl()  {
    // 投弹逻辑可以在这里实现
    auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
    if (stateMachine.getCurrentState() == DoshotState::getInstance()) {
        RCLCPP_INFO(rclcpp::get_logger("DoshotState"), "执行投弹状态");
        if (owner_->is_first_run_){ 
            doshot_state_ = DoshotStateAttribute::doshot_init; // 设置投弹状态为初始化
            owner_->is_first_run_ = false; // 重置第一次运行标志
        }
        RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "执行投弹任务Doshot");
        static int counter = 0, pre_counter; // 航点计数器
        static uint8_t circle_counter = 0; 
        static float pre_time = 0.0f; // 上次航点时间
        static double doshot_halt_end_time; // 记录结束时间
        static int shot_counter = 1; // 投弹计数器
        static float max_accurate; // 聚类目标投弹最大距离
        static bool shot_flag = false; // 投弹标志
        // Vector2d drone_to_camera_rotated; // 中间变量

        // static vector<array<double, 3>> surround_shot_scout_points;

        RCLCPP_INFO_THROTTLE(owner_->get_node()->get_logger(), *owner_->get_node()->get_clock(), 5000, "(THROTTLE 5s)投弹任务执行时间 %f", owner_->state_timer_.elapsed());

        if (owner_->state_timer_.elapsed() > 70 && doshot_state_ != DoshotStateAttribute::doshot_end) // 超时 100 秒
        {
            doshot_halt_end_time = owner_->get_cur_time(); // 记录结束时间
            RCLCPP_INFO(owner_->get_node()->get_logger(), "超时");
            doshot_state_ = DoshotStateAttribute::doshot_end; // 设置投弹状态为结束
        }
        if ((static_cast<int>(owner_->cal_center.size()) > counter && counter != pre_counter) || doshot_state_ == DoshotStateAttribute::doshot_wait || doshot_state_ == DoshotStateAttribute::doshot_init) {
            // max_accurate = owner_->cal_center[counter].diameters / 2; // 更新最大距离
            max_accurate = 0.05; // 更新最大距离
            // RCLCPP_INFO(owner_->get_node()->get_logger(), "更新最大距离为: %f", max_accurate);
        }
        while(true){
            switch (doshot_state_)  // 根据投弹状态执行不同的操作
            {
            case DoshotStateAttribute::doshot_init: // 初始化投弹状态
                {
                    RCLCPP_INFO(owner_->get_node()->get_logger(), "开始投弹任务");
                    assert(owner_ != nullptr);
                    // owner_->rotate_world2local(owner_->drone_to_camera.x(), owner_->drone_to_camera.y(), drone_to_camera_rotated.x(), drone_to_camera_rotated.y());
                    // surround_shot_scout_points = {
                    // 	{owner_->dx_shot + 2.4, owner_->dy_shot + 1.3, 4.5},
                    // 	{owner_->dx_shot + 2.4, owner_->dy_shot + 3.7, 4.5},
                    // 	{owner_->dx_shot - 2.4, owner_->dy_shot + 3.7, 4.5},
                    // 	{owner_->dx_shot - 2.4, owner_->dy_shot + 1.3, 4.5},
                    // };
                    doshot_state_ = DoshotStateAttribute::doshot_shot; // 设置投弹状态为侦查
                    pre_counter = 0;
                    counter = 0; // 重置计数器
                    shot_counter = 1; // 重置投弹计数器

                    owner_->waypoint_timer_.reset();

                    // PosControl::Limits_t limits = owner_->pos_ctl->get_limits_defaults();
                    // limits.speed_max_xy = 1.6; // 设置最大速度为1.6 m/s
                    // limits.speed_max_z = 0.5;
                    // owner_->set_wp_limits(limits);

                }
                continue;
            // case DoshotStateAttribute::doshot_scout: // 侦查投弹区
            // 	RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "开始侦查投弹区");
            // 	if (!surround_shot_scout_points.empty()) {
            // 		if (owner_->trajectory_generator_world_points(
            // 			1, surround_shot_scout_points, surround_shot_scout_points.size(),
            // 			{1.6, 1.6, 1.6}, {0.14, 0.14, 0.14} // 设置最大速度和加速度
            // 		)) {
            // 		// if (owner_->waypoint_goto_next(
            // 		// 	owner_->dx_shot, owner_->dy_shot, owner_->shot_length, owner_->shot_width, 
            // 		// 	owner_->shot_halt, surround_shot_scout_points, 4.0, &counter, "侦查投弹区"))
            // 		// {
                        
            // 			doshot_state_ = DoshotStateAttribute::doshot_shot; // 设置投弹状态为侦查完成
            // 			owner_->waypoint_timer_.reset();
            // 		}
            // 	} else {
            // 		RCLCPP_WARN(owner_->get_node()->get_logger(), "surround_shot_scout_points为空，跳转到doshot_init");
            // 		doshot_state_ = DoshotStateAttribute::doshot_shot;
            // 	}
            // 	break;
            case DoshotStateAttribute::doshot_shot: // 投弹
                // RCLCPP_INFO(owner_->get_node()->get_logger(), "收到坐标c(%f, %f) h(%f ,%f), flag_servo = %d",
                // 	owner_->get_yolo_detector()->get_x(YOLO_TARGET_TYPE::CIRCLE), owner_->get_yolo_detector()->get_y(YOLO_TARGET_TYPE::CIRCLE),
                // 	owner_->get_yolo_detector()->get_x(YOLO_TARGET_TYPE::H), owner_->get_yolo_detector()->get_y(YOLO_TARGET_TYPE::H),
                // 	owner_->get_yolo_detector()->get_servo_flag());
                // RCLCPP_INFO(owner_->get_node()->get_logger(), "handle_state<Doshot>: counter=%d shot_flag=%s waypoint_timer_.elapsed=%lf owner_->cal_center.size()=%ld", counter, shot_flag?"true":"false", owner_->waypoint_timer_.elapsed(), owner_->cal_center.size());
            // 显示当前目标
            {
                TargetData temp_target;
                if (static_cast<int>(owner_->cal_center.size()) > counter) {
                    Vector3d world_point(owner_->cal_center[counter].point.x(), 
                                        owner_->cal_center[counter].point.y(), 
                                        owner_->bucket_height);
                    auto shot_center_opt = owner_->get_camera_gimbal()->worldToPixel(world_point);
                    if (shot_center_opt.has_value()) {
                        Vector2d shot_center = shot_center_opt.value();
                        temp_target.x = shot_center.x();
                        temp_target.y = shot_center.y();
                        temp_target.fx = owner_->get_camera_gimbal()->fx;
                        temp_target.radius = owner_->cal_center[counter].diameters / 2.0;
                        temp_target.category = std::string("circle").append("_w2p");
                        temp_target.relative_z = owner_->get_camera_gimbal()->get_position().z() - world_point.z(); // 设置目标的高度为相机高度
                        owner_->get_yolo_detector()->append_target(temp_target);
                    }
                }
            }

                // 处理投弹逻辑
                RCLCPP_INFO_THROTTLE(owner_->get_node()->get_logger(), *owner_->get_node()->get_clock(), 1000, "handle_state<Doshot>:(THROTTLE 1s) counter=%d shot_counter=%d x:%f, y:%f max:%f", counter, shot_counter,
                    abs(owner_->get_yolo_detector()->get_x(YOLO_TARGET_TYPE::CIRCLE) - owner_->get_yolo_detector()->get_cap_frame_width()/2), abs(owner_->get_yolo_detector()->get_y(YOLO_TARGET_TYPE::CIRCLE) - owner_->get_yolo_detector()->get_cap_frame_height()/2), max_accurate);
                if (!shot_flag && static_cast<size_t>(counter) < owner_->cal_center.size() && (
                        owner_->waypoint_timer_.elapsed() < 6.0 || ( // 至少稳定6秒
                            owner_->waypoint_timer_.elapsed() < 10.0 && ( // 如果小于10秒，且当前无人机位置偏差大于最大距离
                            abs(owner_->get_x_pos() - (owner_->cal_center[counter].point.x())) > max_accurate && 
                            abs(owner_->get_y_pos() - (owner_->cal_center[counter].point.y())) > max_accurate
                            )
                        )
                    )
                ) {
                    Vector2d drone_to_shot_rotated; // 中间变量
                    owner_->rotate_local2world(0.0, 0.10, drone_to_shot_rotated.x(), drone_to_shot_rotated.y());
                    // double tx, ty;
                    Vector2d cal_center_target = {owner_->cal_center[counter].point.x() + drone_to_shot_rotated.x(), owner_->cal_center[counter].point.y() + drone_to_shot_rotated.y()}; // 投弹点适当偏后
                    // owner_->rotate_stand2global(cal_center_target.x(), cal_center_target.y(), tx, ty);
                    // if (tx < owner_->dx_shot - owner_->shot_length_max / 2 - 1.5 || tx > owner_->dx_shot + owner_->shot_length_max / 2 + 1.5 ||
                    // 	ty < owner_->dy_shot - 1.5 || ty > owner_->dy_shot + owner_->shot_width_max + 1.5) {
                    // 	RCLCPP_WARN(owner_->get_node()->get_logger(), "侦查点坐标异常，跳过: %d, x: %f, y: %f", counter, tx, ty);
                    // 	counter++;
                    // 	pre_counter = counter;
                    // 	continue; // 跳过无效坐标
                    // }
                    pre_counter = counter; // 记录上一次的计数器值
                    pre_time = owner_->waypoint_timer_.elapsed(); // 记录上一次的时间
                    owner_->send_world_setpoint_command(
                        cal_center_target.x(),
                        cal_center_target.y(),
                        owner_->shot_halt_low, 0 // local yaw=0
                    );
                    RCLCPP_INFO_THROTTLE(owner_->get_node()->get_logger(), *owner_->get_node()->get_clock(), 500, "handle_state<Doshot>:(THROTTLE 0.5s) 已经确认直径为%f的%d号桶，位置为（%f,%f）, 执行航点时间%f秒，x轴偏差%f, y轴偏差%f，最大距离为%f",
                        owner_->cal_center[counter].diameters,
                        counter, cal_center_target.x(), owner_->cal_center[counter].point.y(),
                        owner_->waypoint_timer_.elapsed(),
                        abs(owner_->get_x_pos() - cal_center_target.x()),
                        abs(owner_->get_y_pos() - cal_center_target.y()),
                        max_accurate);
                } else if (!shot_flag && owner_->fast_mode_) { // 快速投弹
                    RCLCPP_INFO(owner_->get_node()->get_logger(), "fast_mode_ is true, 投弹");
                    owner_->get_servo_controller()->set_servo(10 + shot_counter, owner_->servo_open_position);
                    owner_->waypoint_timer_.reset();
                    doshot_state_ = DoshotStateAttribute::doshot_wait; // 设置投弹状态为等待
                    doshot_halt_end_time = owner_->get_cur_time(); // 记录结束时间
                    continue; // 直接跳到下一个状态;
                } else if (!shot_flag && (!owner_->get_yolo_detector()->is_get_target(YOLO_TARGET_TYPE::CIRCLE) ? circle_counter >= 12 : false)) { // 未找到圆，前往目标过程中最多允许连续n次(n * owner_->get_wait_time()秒)未识别出目标的情况，使用最近一次采集到的位置数据
                    pre_counter = counter; // 记录上一次的计数器值
                    pre_time = owner_->waypoint_timer_.elapsed(); // 记录上一次的时间
                    // owner_->reset_wp_limits(); // 恢复默认速度限制
                    owner_->waypoint_goto_next(
                        owner_->dx_shot, owner_->dy_shot, owner_->shot_length - 3.5, owner_->shot_width, 
                        owner_->shot_halt_surround, owner_->surround_shot_points, 5, &counter, "投弹区"); // 进入投弹区,距离左右边界各1.5m
                } else if (owner_->Doshot(shot_counter, shot_flag)) { // 如果到达投弹点
                    // RCLCPP_INFO(owner_->get_node()->get_logger(), "寻找完毕，投弹!!投弹!!");
                    RCLCPP_INFO(owner_->get_node()->get_logger(), "已经锁定%d号桶，坐标为（%f,%f）", shot_counter, owner_->get_yolo_detector()->get_x(YOLO_TARGET_TYPE::CIRCLE), owner_->get_yolo_detector()->get_y(YOLO_TARGET_TYPE::CIRCLE));
                    RCLCPP_INFO(owner_->get_node()->get_logger(), "投弹!!投弹!!，总用时：%f", owner_->state_timer_.elapsed());
                    doshot_state_ = DoshotStateAttribute::doshot_wait; // 设置投弹状态为结束
                    doshot_halt_end_time = owner_->get_cur_time(); // 记录结束时间
                    continue; // 继续执行下一次循环
                } else { // 如果找到投弹目标但未到达目标上方
                    if (owner_->get_yolo_detector()->is_get_target(YOLO_TARGET_TYPE::CIRCLE)) {
                        circle_counter = 0; // 重置计数器
                    } else {
                        circle_counter++; // 增加计数器
                    }
                    max_accurate = 5;
                    counter = pre_counter; // 恢复上一次的计数器值
                    owner_->waypoint_timer_.set_start_time_to_time_point(pre_time); // 重置航点计时器
                }
                break;
            case DoshotStateAttribute::doshot_wait: // 等待再次投弹
                if(shot_counter <= 1) // 投弹次数小于等于1，再次执行投弹
                {
                    if (static_cast<size_t>(counter) >= owner_->cal_center.size()){
                        // owner_->waypoint_goto_next(
                        // 	owner_->dx_shot, owner_->dy_shot, owner_->shot_length, owner_->shot_width, 
                        // 	owner_->shot_halt, owner_->surround_shot_points, owner_->shot_halt, &counter, "投弹区");
                        // if (owner_->get_cur_time() - doshot_halt_end_time < 5.0 || counter == pre_counter + 1) {   // 非阻塞等待至第5秒或抵达下一个航点
                        // 	break;
                        // }
                    } else {
                        // owner_->send_world_setpoint_command(
                        // 	owner_->cal_center[counter + 1].point.x(),
                        // 	owner_->cal_center[counter + 1].point.y(),
                        // 	owner_->shot_halt_low, 0
                        // );
                        // if (owner_->get_cur_time() - doshot_halt_end_time < 2.0){
                        // 	break; // 等待2秒
                        // }
                        counter++; // 增加计数器
                        pre_counter = counter; // 更新上一次计数器值
                    }
                    RCLCPP_INFO(owner_->get_node()->get_logger(), "投弹完成，继续投弹 shot_counter=%d counter=%d", shot_counter, counter);
                    shot_counter++;
                    doshot_state_ = DoshotStateAttribute::doshot_shot; // 设置投弹状态为投弹
                    owner_->waypoint_timer_.reset(); // 重置航点计时器
                    pre_time = owner_->waypoint_timer_.elapsed(); // 记录上一次的时间
                    doshot_halt_end_time = owner_->get_cur_time(); // 记录结束时间
                    continue; // 继续投弹
                } else {
                    doshot_state_ = DoshotStateAttribute::doshot_end; // 设置投弹状态为结束
                    continue; // 继续执行下一次循环
                }
                break;
            case DoshotStateAttribute::doshot_end: // 侦查投弹区
                if (owner_->get_cur_time() - doshot_halt_end_time < 2.0) {
                    if (owner_->get_cur_time() - doshot_halt_end_time < owner_->get_wait_time()) {
                        RCLCPP_INFO_THROTTLE(owner_->get_node()->get_logger(), *owner_->get_node()->get_clock(), 1000, "投弹完成，等待2秒后前往侦查区域");
                        // owner_->reset_wp_limits();
                        owner_->get_servo_controller()->set_servo(11, owner_->get_servo_controller()->get_servo_open_position());			// owner_->get_servo_controller()->set_servo(11, owner_->servo_close_position);
                        owner_->get_servo_controller()->set_servo(12, owner_->get_servo_controller()->get_servo_open_position());				// owner_->get_servo_controller()->set_servo(12, owner_->servo_close_position);
                    }
                    break; // 等待2秒
                }
                // 重置状态
                doshot_state_ = DoshotStateAttribute::doshot_init; // 重置投弹状态
                stateMachine.transitionTo(GotoScoutPointState::getInstance());
                break;
            default:
                break;
            }
            break; // 跳出 while 循环
        }
    }
    return; // 结束函数
}
