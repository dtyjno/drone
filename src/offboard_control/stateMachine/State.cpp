#include "State.h"
#include "StateMachine.h"
#include <rclcpp/rclcpp.hpp>
#include "../APMROS2drone/APMROS2Drone.h"

// 静态成员定义 - 移除不再需要的static成员

template<typename T>
int State<T>::index_counter_ = 0;

// 显式模板实例化 - 为APMROS2Drone类型实例化静态成员
template int State<APMROS2Drone>::index_counter_;

// 预定义的状态实例
// namespace States {

//     // 创建具体的状态类
//     class InitState : public State<APMROS2Drone> {
//     public:
//         static const InitState& getInstance() {
//             static InitState instance;
//             return instance;
//         }
        
//     protected:
//         void executeImpl()  override {
//             RCLCPP_INFO(rclcpp::get_logger("InitState"), "执行初始化状态");
//             // 初始化逻辑可以在这里实现
//             auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
//             if (stateMachine.getCurrentState() == InitState::getInstance()) {
//                 owner_->FlyState_init();
//                 RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "初始化完成");
//                 if (owner_->debug_mode_) {
//                     RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "测试模式下, 不进行起飞");
//                     stateMachine.transitionTo(GotoShotPointState::getInstance());
//                     return;
//                 }
//                 stateMachine.transitionTo(TakeoffState::getInstance());
//             }
//         }
        
//     private:
//         InitState() : State("init", 1) {}
//         friend class State;
//     };
    
//     class TakeoffState : public State<APMROS2Drone> {
//     public:
//         static const TakeoffState& getInstance() {
//             static TakeoffState instance;
//             return instance;
//         }
        
//     protected:
//         void executeImpl()  override {
//             RCLCPP_INFO(rclcpp::get_logger("TakeoffState"), "执行起飞状态");
//             // 起飞逻辑可以在这里实现
//             auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
//             if (stateMachine.getCurrentState() == TakeoffState::getInstance()) {
//                 RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "开始起飞");

//                 if (owner_->sta_ctl->takeoff(owner_->get_z_pos(), 2.0, owner_->get_yaw())) {
//                         RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "起飞成功");
//                         stateMachine.transitionTo(GotoShotPointState::getInstance());
//                 } else {
//                         // RCLCPP_INFO(owner_->get_node()->get_logger(), "起飞失败");
//                 }
//             }
//         }
        
//     private:
//         TakeoffState() : State("takeoff", 1) {}
//         friend class State;
//     };
    
//     class EndState : public State<APMROS2Drone> {
//     public:
//         static const EndState& getInstance() {
//             static EndState instance;
//             return instance;
//         }
        
//     protected:
//         void executeImpl()  override {
//             RCLCPP_INFO(rclcpp::get_logger("EndState"), "执行结束状态");
//             // 结束逻辑可以在这里实现
//             auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
//             if (stateMachine.getCurrentState() == EndState::getInstance()) {
//                 RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "任务开始时间: %f 秒", owner_->get_start_time());
//                 RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "任务结束, 运行时间: %f 秒", owner_->get_cur_time());
//                 RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "任务结束, 任务运行时间: %f 秒", owner_->get_cur_time() - owner_->get_start_time());
//                 if (owner_->state_timer_.elapsed() < 3) {
//                     return; // 等待 3 秒后结束
//                 }
//                 // 如果需要，可以在这里添加清理或退出逻辑
//                 rclcpp::shutdown();  // 停止 ROS 2 节点
//                 owner_.reset(); // 释放 shared_ptr 引用
//             }
//         }
        
//     private:
//         EndState() : State("end", 2) {}
//         friend class State;
//     };

//     class StopState : public State<APMROS2Drone> {
//     public:
//         static const StopState& getInstance() {
//             static StopState instance;
//             return instance;
//         }
        
//     protected:
//         void executeImpl()  override {
//             RCLCPP_INFO(rclcpp::get_logger("StopState"), "执行停止状态");
//             // 停止逻辑可以在这里实现
//             auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
//             if (stateMachine.getCurrentState() == StopState::getInstance()) {
//                 RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "停止状态");
//                 // 停止飞行器的所有运动
//                 owner_->send_velocity_command(0.0, 0.0, 0.0, 0.0);
//                 // 可以在这里添加其他停止逻辑
//             }
//         }
        
//     private:
//         StopState() : State("stop", 1) {}
//         friend class State;
//     };

//     class GotoShotPointState : public State<APMROS2Drone> {
//     public:
//         static const GotoShotPointState& getInstance() {
//             static GotoShotPointState instance;
//             return instance;
//         }
        
//     protected:
//         void executeImpl()  override {
//             RCLCPP_INFO(rclcpp::get_logger("GotoShotPointState"), "执行前往投弹点状态");
//             // 前往投弹点逻辑可以在这里实现
//             auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
//             if (stateMachine.getCurrentState() == GotoShotPointState::getInstance()) {
//                 RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "开始前往投弹区起点");
//                 float x_shot, y_shot;
//                 owner_->rotate_global2stand(owner_->dx_shot, owner_->dy_shot + owner_->shot_width_max / 2, x_shot, y_shot);
//                 if(owner_->waypoint_timer_.elapsed() > 12)
//                 {
//                     owner_->waypoint_timer_.set_start_time_to_default();
//                     RCLCPP_INFO(owner_->get_node()->get_logger(), "到达投弹区起点");
//                     stateMachine.transitionTo(DoshotState::getInstance());
//                 } else {
//                     owner_->send_start_setpoint_command(
//                         x_shot, y_shot, owner_->shot_halt, 0
//                     );
//                     RCLCPP_INFO_THROTTLE(owner_->get_node()->get_logger(), *owner_->get_node()->get_clock(), 3000, "(THROTTLE 3s)前往投弹区中...%f", owner_->waypoint_timer_.elapsed());
//                 }
//             }
//         }
        
//     private:
//         GotoShotPointState() : State("goto_shotpoint", 1) {}
//         friend class State;
//     };

//     class DoshotState : public State<APMROS2Drone> {
//     public:
//         static const DoshotState& getInstance() {
//             static DoshotState instance;
//             return instance;
//         }
        
//     protected:
//         void executeImpl()  override {
//             RCLCPP_INFO(rclcpp::get_logger("DoshotState"), "执行投弹状态");
//             // 投弹逻辑可以在这里实现
//             auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
//             if (stateMachine.getCurrentState() == DoshotState::getInstance()) {
//                 if (owner_->is_first_run_){ 
//                     owner_->doshot_state_ = owner_->DoshotState::doshot_init; // 设置投弹状态为初始化
//                     owner_->is_first_run_ = false; // 重置第一次运行标志
//                 }
//                 RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "执行投弹任务Doshot");
//                 static int counter = 0, pre_counter; // 航点计数器
//                 static uint8_t circle_counter = 0; 
//                 static float pre_time = 0.0f; // 上次航点时间
//                 static double doshot_halt_end_time; // 记录结束时间
//                 static int shot_counter = 1; // 投弹计数器
//                 static float max_accurate; // 聚类目标投弹最大距离
//                 static bool shot_flag = false; // 投弹标志
//                 // Vector2d drone_to_camera_rotated; // 中间变量

//                 // static vector<array<double, 3>> surround_shot_scout_points;

//                 RCLCPP_INFO_THROTTLE(owner_->get_node()->get_logger(), *owner_->get_node()->get_clock(), 5000, "(THROTTLE 5s)投弹任务执行时间 %f", owner_->state_timer_.elapsed());

//                 if (owner_->state_timer_.elapsed() > 70 && owner_->doshot_state_ != owner_->DoshotState::doshot_end) // 超时 100 秒
//                 {
//                     doshot_halt_end_time = owner_->get_cur_time(); // 记录结束时间
//                     RCLCPP_INFO(owner_->get_node()->get_logger(), "超时");
//                     owner_->doshot_state_ = owner_->DoshotState::doshot_end; // 设置投弹状态为结束
//                 }
//                 if ((static_cast<int>(owner_->cal_center.size()) > counter && counter != pre_counter) || owner_->doshot_state_ == owner_->DoshotState::doshot_wait || owner_->doshot_state_ == owner_->DoshotState::doshot_init) {
//                     // max_accurate = owner_->cal_center[counter].diameters / 2; // 更新最大距离
//                     max_accurate = 0.05; // 更新最大距离
//                     // RCLCPP_INFO(owner_->get_node()->get_logger(), "更新最大距离为: %f", max_accurate);
//                 }
//                 while(true){
//                     switch (owner_->doshot_state_)  // 根据投弹状态执行不同的操作
//                     {
//                     case owner_->DoshotState::doshot_init: // 初始化投弹状态
//                         {
//                             RCLCPP_INFO(owner_->get_node()->get_logger(), "开始投弹任务");
//                             assert(owner_ != nullptr);
//                             // owner_->rotate_world2local(owner_->drone_to_camera.x(), owner_->drone_to_camera.y(), drone_to_camera_rotated.x(), drone_to_camera_rotated.y());
//                             // surround_shot_scout_points = {
//                             // 	{owner_->dx_shot + 2.4, owner_->dy_shot + 1.3, 4.5},
//                             // 	{owner_->dx_shot + 2.4, owner_->dy_shot + 3.7, 4.5},
//                             // 	{owner_->dx_shot - 2.4, owner_->dy_shot + 3.7, 4.5},
//                             // 	{owner_->dx_shot - 2.4, owner_->dy_shot + 1.3, 4.5},
//                             // };
//                             owner_->doshot_state_ = owner_->DoshotState::doshot_shot; // 设置投弹状态为侦查
//                             pre_counter = 0;
//                             counter = 0; // 重置计数器
//                             shot_counter = 1; // 重置投弹计数器

//                             owner_->waypoint_timer_.reset();

//                             // PosControl::Limits_t limits = owner_->pos_ctl->get_limits_defaults();
//                             // limits.speed_max_xy = 1.6; // 设置最大速度为1.6 m/s
//                             // limits.speed_max_z = 0.5;
//                             // owner_->set_wp_limits(limits);

//                         }
//                         continue;
//                     // case owner_->DoshotState::doshot_scout: // 侦查投弹区
//                     // 	RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "开始侦查投弹区");
//                     // 	if (!surround_shot_scout_points.empty()) {
//                     // 		if (owner_->trajectory_generator_world_points(
//                     // 			1, surround_shot_scout_points, surround_shot_scout_points.size(),
//                     // 			{1.6, 1.6, 1.6}, {0.14, 0.14, 0.14} // 设置最大速度和加速度
//                     // 		)) {
//                     // 		// if (owner_->waypoint_goto_next(
//                     // 		// 	owner_->dx_shot, owner_->dy_shot, owner_->shot_length, owner_->shot_width, 
//                     // 		// 	owner_->shot_halt, surround_shot_scout_points, 4.0, &counter, "侦查投弹区"))
//                     // 		// {
                                
//                     // 			owner_->doshot_state_ = owner_->DoshotState::doshot_shot; // 设置投弹状态为侦查完成
//                     // 			owner_->waypoint_timer_.reset();
//                     // 		}
//                     // 	} else {
//                     // 		RCLCPP_WARN(owner_->get_node()->get_logger(), "surround_shot_scout_points为空，跳转到doshot_init");
//                     // 		owner_->doshot_state_ = owner_->DoshotState::doshot_shot;
//                     // 	}
//                     // 	break;
//                     case owner_->DoshotState::doshot_shot: // 投弹
//                         // RCLCPP_INFO(owner_->get_node()->get_logger(), "收到坐标c(%f, %f) h(%f ,%f), flag_servo = %d",
//                         // 	owner_->get_yolo_detector()->get_x(YOLODetector::TARGET_TYPE::CIRCLE), owner_->get_yolo_detector()->get_y(YOLODetector::TARGET_TYPE::CIRCLE),
//                         // 	owner_->get_yolo_detector()->get_x(YOLODetector::TARGET_TYPE::H), owner_->get_yolo_detector()->get_y(YOLODetector::TARGET_TYPE::H),
//                         // 	owner_->get_yolo_detector()->get_servo_flag());
//                         // RCLCPP_INFO(owner_->get_node()->get_logger(), "handle_state<Doshot>: counter=%d shot_flag=%s waypoint_timer_.elapsed=%lf owner_->cal_center.size()=%ld", counter, shot_flag?"true":"false", owner_->waypoint_timer_.elapsed(), owner_->cal_center.size());
//                     // 显示当前目标
//                     {
//                         YOLODetector::Target temp_target;
//                         if (static_cast<int>(owner_->cal_center.size()) > counter) {
//                             Vector3d world_point(owner_->cal_center[counter].point.x(), 
//                                                 owner_->cal_center[counter].point.y(), 
//                                                 owner_->bucket_height);
//                             auto shot_center_opt = owner_->_camera_gimbal->worldToPixel(world_point);
//                             if (shot_center_opt.has_value()) {
//                                 Vector2d shot_center = shot_center_opt.value();
//                                 temp_target.x = shot_center.x();
//                                 temp_target.y = shot_center.y();
//                                 temp_target.fx = owner_->_camera_gimbal->fx;
//                                 temp_target.radius = owner_->cal_center[counter].diameters / 2.0;
//                                 temp_target.category = std::string("circle").append("_w2p");
//                                 temp_target.relative_z = owner_->_camera_gimbal->get_position().z() - world_point.z(); // 设置目标的高度为相机高度
//                                 owner_->get_yolo_detector()->append_target(temp_target);
//                             }
//                         }
//                     }

//                         // 处理投弹逻辑
//                         RCLCPP_INFO_THROTTLE(owner_->get_node()->get_logger(), *owner_->get_node()->get_clock(), 1000, "handle_state<Doshot>:(THROTTLE 1s) counter=%d shot_counter=%d x:%f, y:%f max:%f", counter, shot_counter,
//                             abs(owner_->get_yolo_detector()->get_x(YOLODetector::TARGET_TYPE::CIRCLE) - owner_->get_yolo_detector()->get_cap_frame_width()/2), abs(owner_->get_yolo_detector()->get_y(YOLODetector::TARGET_TYPE::CIRCLE) - owner_->get_yolo_detector()->get_cap_frame_height()/2), max_accurate);
//                         if (!shot_flag && static_cast<size_t>(counter) < owner_->cal_center.size() && (
//                                 owner_->waypoint_timer_.elapsed() < 6.0 || ( // 至少稳定6秒
//                                     owner_->waypoint_timer_.elapsed() < 10.0 && ( // 如果小于10秒，且当前无人机位置偏差大于最大距离
//                                     abs(owner_->get_x_pos() - (owner_->cal_center[counter].point.x())) > max_accurate && 
//                                     abs(owner_->get_y_pos() - (owner_->cal_center[counter].point.y())) > max_accurate
//                                     )
//                                 )
//                             )
//                         ) {
//                             Vector2d drone_to_shot_rotated; // 中间变量
//                             owner_->rotate_local2world(0.0, 0.10, drone_to_shot_rotated.x(), drone_to_shot_rotated.y());
//                             // double tx, ty;
//                             Vector2d cal_center_target = {owner_->cal_center[counter].point.x() + drone_to_shot_rotated.x(), owner_->cal_center[counter].point.y() + drone_to_shot_rotated.y()}; // 投弹点适当偏后
//                             // owner_->rotate_stand2global(cal_center_target.x(), cal_center_target.y(), tx, ty);
//                             // if (tx < owner_->dx_shot - owner_->shot_length_max / 2 - 1.5 || tx > owner_->dx_shot + owner_->shot_length_max / 2 + 1.5 ||
//                             // 	ty < owner_->dy_shot - 1.5 || ty > owner_->dy_shot + owner_->shot_width_max + 1.5) {
//                             // 	RCLCPP_WARN(owner_->get_node()->get_logger(), "侦查点坐标异常，跳过: %d, x: %f, y: %f", counter, tx, ty);
//                             // 	counter++;
//                             // 	pre_counter = counter;
//                             // 	continue; // 跳过无效坐标
//                             // }
//                             pre_counter = counter; // 记录上一次的计数器值
//                             pre_time = owner_->waypoint_timer_.elapsed(); // 记录上一次的时间
//                             owner_->send_world_setpoint_command(
//                                 cal_center_target.x(),
//                                 cal_center_target.y(),
//                                 owner_->shot_halt_low, 0 // local yaw=0
//                             );
//                             RCLCPP_INFO_THROTTLE(owner_->get_node()->get_logger(), *owner_->get_node()->get_clock(), 500, "handle_state<Doshot>:(THROTTLE 0.5s) 已经确认直径为%f的%d号桶，位置为（%f,%f）, 执行航点时间%f秒，x轴偏差%f, y轴偏差%f，最大距离为%f",
//                                 owner_->cal_center[counter].diameters,
//                                 counter, cal_center_target.x(), owner_->cal_center[counter].point.y(),
//                                 owner_->waypoint_timer_.elapsed(),
//                                 abs(owner_->get_x_pos() - cal_center_target.x()),
//                                 abs(owner_->get_y_pos() - cal_center_target.y()),
//                                 max_accurate);
//                         } else if (!shot_flag && owner_->fast_mode_) { // 快速投弹
//                             RCLCPP_INFO(owner_->get_node()->get_logger(), "fast_mode_ is true, 投弹");
//                             owner_->_servo_controller->set_servo(10 + shot_counter, owner_->servo_open_position);
//                             owner_->waypoint_timer_.reset();
//                             owner_->doshot_state_ = owner_->DoshotState::doshot_wait; // 设置投弹状态为等待
//                             doshot_halt_end_time = owner_->get_cur_time(); // 记录结束时间
//                             continue; // 直接跳到下一个状态;
//                         } else if (!shot_flag && (!owner_->get_yolo_detector()->is_get_target(YOLODetector::TARGET_TYPE::CIRCLE) ? circle_counter >= 12 : false)) { // 未找到圆，前往目标过程中最多允许连续n次(n * owner_->get_wait_time()秒)未识别出目标的情况，使用最近一次采集到的位置数据
//                             pre_counter = counter; // 记录上一次的计数器值
//                             pre_time = owner_->waypoint_timer_.elapsed(); // 记录上一次的时间
//                             // owner_->reset_wp_limits(); // 恢复默认速度限制
//                             owner_->waypoint_goto_next(
//                                 owner_->dx_shot, owner_->dy_shot, owner_->shot_length - 3.5, owner_->shot_width, 
//                                 owner_->shot_halt_surround, owner_->surround_shot_points, 5, &counter, "投弹区"); // 进入投弹区,距离左右边界各1.5m
//                         } else if (owner_->Doshot(shot_counter, shot_flag)) { // 如果到达投弹点
//                             // RCLCPP_INFO(owner_->get_node()->get_logger(), "寻找完毕，投弹!!投弹!!");
//                             RCLCPP_INFO(owner_->get_node()->get_logger(), "已经锁定%d号桶，坐标为（%f,%f）", shot_counter, owner_->get_yolo_detector()->get_x(YOLODetector::TARGET_TYPE::CIRCLE), owner_->get_yolo_detector()->get_y(YOLODetector::TARGET_TYPE::CIRCLE));
//                             RCLCPP_INFO(owner_->get_node()->get_logger(), "投弹!!投弹!!，总用时：%f", owner_->state_timer_.elapsed());
//                             owner_->doshot_state_ = owner_->DoshotState::doshot_wait; // 设置投弹状态为结束
//                             doshot_halt_end_time = owner_->get_cur_time(); // 记录结束时间
//                             continue; // 继续执行下一次循环
//                         } else { // 如果找到投弹目标但未到达目标上方
//                             if (owner_->get_yolo_detector()->is_get_target(YOLODetector::TARGET_TYPE::CIRCLE)) {
//                                 circle_counter = 0; // 重置计数器
//                             } else {
//                                 circle_counter++; // 增加计数器
//                             }
//                             max_accurate = 5;
//                             counter = pre_counter; // 恢复上一次的计数器值
//                             owner_->waypoint_timer_.set_start_time_to_time_point(pre_time); // 重置航点计时器
//                         }
//                         break;
//                     case owner_->DoshotState::doshot_wait: // 等待再次投弹
//                         if(shot_counter <= 1) // 投弹次数小于等于1，再次执行投弹
//                         {
//                             if (static_cast<size_t>(counter) >= owner_->cal_center.size()){
//                                 // owner_->waypoint_goto_next(
//                                 // 	owner_->dx_shot, owner_->dy_shot, owner_->shot_length, owner_->shot_width, 
//                                 // 	owner_->shot_halt, owner_->surround_shot_points, owner_->shot_halt, &counter, "投弹区");
//                                 // if (owner_->get_cur_time() - doshot_halt_end_time < 5.0 || counter == pre_counter + 1) {   // 非阻塞等待至第5秒或抵达下一个航点
//                                 // 	break;
//                                 // }
//                             } else {
//                                 // owner_->send_world_setpoint_command(
//                                 // 	owner_->cal_center[counter + 1].point.x(),
//                                 // 	owner_->cal_center[counter + 1].point.y(),
//                                 // 	owner_->shot_halt_low, 0
//                                 // );
//                                 // if (owner_->get_cur_time() - doshot_halt_end_time < 2.0){
//                                 // 	break; // 等待2秒
//                                 // }
//                                 counter++; // 增加计数器
//                                 pre_counter = counter; // 更新上一次计数器值
//                             }
//                             RCLCPP_INFO(owner_->get_node()->get_logger(), "投弹完成，继续投弹 shot_counter=%d counter=%d", shot_counter, counter);
//                             shot_counter++;
//                             owner_->doshot_state_ = owner_->DoshotState::doshot_shot; // 设置投弹状态为投弹
//                             owner_->waypoint_timer_.reset(); // 重置航点计时器
//                             pre_time = owner_->waypoint_timer_.elapsed(); // 记录上一次的时间
//                             doshot_halt_end_time = owner_->get_cur_time(); // 记录结束时间
//                             continue; // 继续投弹
//                         } else {
//                             owner_->doshot_state_ = owner_->DoshotState::doshot_end; // 设置投弹状态为结束
//                             continue; // 继续执行下一次循环
//                         }
//                         break;
//                     case owner_->DoshotState::doshot_end: // 侦查投弹区
//                         if (owner_->get_cur_time() - doshot_halt_end_time < 2.0) {
//                             if (owner_->get_cur_time() - doshot_halt_end_time < owner_->get_wait_time()) {
//                                 RCLCPP_INFO_THROTTLE(owner_->get_node()->get_logger(), *owner_->get_node()->get_clock(), 1000, "投弹完成，等待2秒后前往侦查区域");
//                                 // owner_->reset_wp_limits();
//                                 owner_->_servo_controller->set_servo(11, ServoController::servo_open_position);			// owner_->_servo_controller->set_servo(11, owner_->servo_close_position);
//                                 owner_->_servo_controller->set_servo(12, ServoController::servo_open_position);				// owner_->_servo_controller->set_servo(12, owner_->servo_close_position);
//                             }
//                             break; // 等待2秒
//                         }
//                         // 重置状态
//                         owner_->doshot_state_ = owner_->DoshotState::doshot_init; // 重置投弹状态
//                         stateMachine.transitionTo(GotoScoutPointState::getInstance());
//                         break;
//                     default:
//                         break;
//                     }
//                     break; // 跳出 while 循环
//                 }
//             }
//             return; // 结束函数
//         }
        
//     private:
//         DoshotState() : State("doshot", 0) {}
//         friend class State;
//     };
    
//     class GotoScoutPointState : public State<APMROS2Drone> {
//     public:
//         static const GotoScoutPointState& getInstance() {
//             static GotoScoutPointState instance;
//             return instance;
//         }
        
//     protected:
//         void executeImpl()  override {
//             RCLCPP_INFO(rclcpp::get_logger("GotoScoutPointState"), "执行前往侦查点状态");
//             // 前往侦查点逻辑可以在这里实现
//             auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
//             if (stateMachine.getCurrentState() == GotoScoutPointState::getInstance()) {
//                 RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "开始前往侦查起点");
//                 float x_see, y_see;
//                 owner_->rotate_global2stand(owner_->dx_see, owner_->dy_see, x_see, y_see);
//                 if(owner_->waypoint_timer_.elapsed() > 7.5)
//                 {
//                     owner_->waypoint_timer_.set_start_time_to_default();
//                     RCLCPP_INFO(owner_->get_node()->get_logger(), "到达侦查区起点");
//                     stateMachine.transitionTo(SurroundSeeState::getInstance());
//                 } else {
//                     owner_->send_start_setpoint_command(
//                         x_see, y_see, owner_->see_halt, 0
//                     );
//                     RCLCPP_INFO_THROTTLE(owner_->get_node()->get_logger(), *owner_->get_node()->get_clock(), 3000, "(THROTTLE 3s)前往侦查区中...%f", owner_->waypoint_timer_.elapsed());
//                 }
//             }
//         }
        
//     private:
//         GotoScoutPointState() : State("goto_scoutpoint", 1) {}
//         friend class State;
//     };

//     class SurroundSeeState : public State<APMROS2Drone> {
//     public:
//         static const SurroundSeeState& getInstance() {
//             static SurroundSeeState instance;
//             return instance;
//         }
        
//     protected:
//         void executeImpl()  override {
//             RCLCPP_INFO(rclcpp::get_logger("SurroundSeeState"), "执行环绕侦查状态");
//             // 环绕侦查逻辑可以在这里实现
//             auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
//             if (stateMachine.getCurrentState() == SurroundSeeState::getInstance()) {
//                 static int counter = 0; // 航点计数器
//                 if (owner_->waypoint_goto_next(
//                     owner_->dx_see, owner_->dy_see, owner_->see_length - 2.0, owner_->see_width - 0.2, 
//                     owner_->see_halt, owner_->surround_see_points, 3.5, &counter, "侦查区"))
//                 {
//                     RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "侦查完毕");
//                     counter = 0;
//                     // owner_->sta_ctl->switch_mode("RTL");
//                     // rclcpp::sleep_for(std::chrono::seconds(17));
//                     // owner_->sta_ctl->switch_mode("GUIDED");
//                     stateMachine.transitionTo(DolandState::getInstance());
//                 }
//             }
//         }
        
//     private:
//         SurroundSeeState() : State("surround_see", 3) {}
//         friend class State;
//     };
    
//     class DolandState : public State<APMROS2Drone> {
//     public:
//         static const DolandState& getInstance() {
//             static DolandState instance;
//             return instance;
//         }
        
//     protected:
//         void executeImpl()  override {
//             RCLCPP_INFO(rclcpp::get_logger("DolandState"), "执行降落状态");
//             // 降落逻辑可以在这里实现
//             auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
//             if (stateMachine.getCurrentState() == DolandState::getInstance()) {
//                 static enum class DolandState {
//                     doland_init, // 降落初始化
//                     doland_wait, // 等待降落
//                     doland_landing, // 降落中
//                     doland_end // 降落结束
//                 } doland_state = DolandState::doland_init; // 降落状态
//                 if (owner_->is_first_run_) {
//                     doland_state = DolandState::doland_init; // 重置降落状态
//                     owner_->is_first_run_ = false; // 重置第一次运行标志
//                 }
//                 while (true){
//                     switch (doland_state) {
//                     case DolandState::doland_init: // 降落初始化
//                         RCLCPP_INFO(owner_->get_node()->get_logger(), "开始降落");
//                         owner_->sta_ctl->switch_mode("RTL");
//                         doland_state = DolandState::doland_wait; // 切换到等待降落状态
//                         continue; // 继续执行下一次循环;
//                     case DolandState::doland_wait: // 等待降落
//                         if (owner_->state_timer_.elapsed() > 18) { // 如果等待超过18秒
//                             RCLCPP_INFO(owner_->get_node()->get_logger(), "等待降落超过18秒，开始降落");
//                             owner_->sta_ctl->switch_mode("GUIDED");
//                             doland_state = DolandState::doland_landing; // 切换到降落中状态
//                             continue; // 继续执行下一次循环;
//                         } else {
//                             RCLCPP_INFO_THROTTLE(owner_->get_node()->get_logger(), *owner_->get_node()->get_clock(), 3000, "(THROTTLE 3s)等待降落中...%f", owner_->state_timer_.elapsed());
//                         }
//                         break;
//                     case DolandState::doland_landing: // 降落中
//                         if(owner_->Doland()){
//                             doland_state = DolandState::doland_end; // 切换到降落结束状态
//                             continue; // 继续执行下一次循环;
//                         }
//                         break; // 继续执行下一次循环;
//                     case DolandState::doland_end: // 降落结束
//                         RCLCPP_INFO(owner_->get_node()->get_logger(), "降落完成");
//                         owner_->sta_ctl->switch_mode("LAND");
//                         stateMachine.transitionTo(EndState::getInstance());
//                         doland_state = DolandState::doland_init; // 重置降落状态
//                         break; // 结束函数
//                     default:
//                         break;
//                     }
//                     break;
//                 }
//             }
//             return; // 结束函数
//         }
        
//     private:
//         DolandState() : State("doland", 4) {}
//         friend class State;
//     };

//     class MypidState : public State<APMROS2Drone> {
//     public:
//         static const MypidState& getInstance() {
//             static MypidState instance;
//             return instance;
//         }
        
//     protected:
//         void executeImpl()  override {
//             // RCLCPP_INFO(rclcpp::get_logger("MypidState"), "执行PID调节状态");
//             // // PID调节逻辑可以在这里实现
//             // auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
//             // if (stateMachine.getCurrentState() == FlyState::MYPID
//             // ) {
//             //     owner_->mypid.readPIDParameters("pos_config.yaml","mypid");
//             //     owner_->dx_shot = owner_->mypid.read_goal("OffboardControl.yaml","dx_shot");
//             //     owner_->dy_shot = owner_->mypid.read_goal("OffboardControl.yaml","dy_shot");
//             //     owner_->shot_halt = owner_->mypid.read_goal("OffboardControl.yaml","shot_halt");
//             //     printf("已进入MYPID状态,当前kp ki kd参数分别为：%lf %lf %lf,输出限制为： %lf,积分限制为： %lf\n",owner_->mypid.kp_,owner_->mypid.ki_,owner_->mypid.kd_,owner_->mypid.output_limit_,owner_->mypid.integral_limit);
        
//             //     owner_->mypid.velocity_x = owner_->get_x_vel();
//             //     owner_->mypid.velocity_y = owner_->get_y_vel();
//             //     owner_->mypid.velocity_z = owner_->get_z_vel();
        
//             //     printf("当前速度分别为：vx: %lf vy: %lf vz:%lf\n",owner_->mypid.velocity_x,owner_->mypid.velocity_y,owner_->mypid.velocity_z);
//             //     printf("当前位置为（ %lf , %lf , %lf ）\n",owner_->get_x_pos(),owner_->get_y_pos(),owner_->get_z_pos());
//             //     printf("目标位置为 （ %lf , %lf , %lf ）\n",owner_->dx_shot,owner_->dy_shot,owner_->shot_halt);
//             //     printf("PID输出分别为：（ %lf , %lf ,%lf）\n",owner_->mypid.compute(owner_->dx_shot,owner_->get_x_pos(),0.01),owner_->mypid.compute(owner_->dy_shot,owner_->get_y_pos(),0.01),
//             //                         owner_->mypid.compute(owner_->shot_halt,owner_->get_z_pos(),0.01));
                
//             //     owner_->mypid.Mypid(owner_->dx_shot,owner_->dy_shot,owner_->shot_halt,owner_->get_x_pos(),owner_->get_y_pos(),owner_->get_z_pos(),0.01);
//             //     owner_->send_velocity_command(owner_->mypid.velocity_x,owner_->mypid.velocity_y,owner_->mypid.velocity_z,0);
//             //     //stateMachine.transitionTo(FlyState::end);
//             // }
//         }
        
//     private:
//         MypidState() : State("mypid", 1) {}
//         friend class State;
//     };
    
//     class PrintInfoState : public State<APMROS2Drone> {
//     public:
//         static const PrintInfoState& getInstance() {
//             static PrintInfoState instance;
//             return instance;
//         }
        
//     protected:
//         void executeImpl()  override {
//             RCLCPP_INFO(rclcpp::get_logger("PrintInfoState"), "执行信息打印状态");
//             // 信息打印逻辑可以在这里实现
//             auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
//             if (stateMachine.getCurrentStateName() == State::PrintInfoState::getInstance().getName() {
//             ) {
//                 static unsigned short print_count = 0;
//                 print_count++; // 先递增
//                 print_count = print_count % 3; // 然后取模
//                 if (print_count != 0) {
//                     return; // 每10次打印一次
//                 }
//                 std::stringstream ss;
//                 // 打印当前状态信息
//                 ss << "--------timer_callback----------" << std::endl;
//                 ss << "px:  " << std::setw(10) << owner_->get_x_pos() << ", py: " << std::setw(10) << owner_->get_y_pos() << ", pz: " << std::setw(10) << owner_->get_z_pos() << std::endl;
//                 ss << "vx:  " << std::setw(10) << owner_->get_x_vel() << ", vy: " << std::setw(10) << owner_->get_y_vel() << ", vz: " << std::setw(10) << owner_->get_z_vel() << std::endl;
//                 // ss << "yaw: " << owner_->get_yaw() << std::endl;
//                 // ss << "yaw_e: " << owner_->get_yaw_eigen() << std::endl;
//                 ss << "yaw_vel: " << owner_->get_yaw_vel() << std::endl;
//                 float roll, pitch, yaw;
//                 owner_->get_euler(roll, pitch, yaw);
//                 ss << "roll: " << roll << ", pitch: " << pitch << ", yaw: " << yaw << std::endl;
//                 ss << "lat: " << owner_->get_lat() << ", lon: " << owner_->get_lon() << ", alt: " << owner_->get_alt() << std::endl;
//                 ss << "rangefinder_distance:  " << owner_->get_rangefinder_distance() << std::endl;
//                 ss << "armed:     " << owner_->get_armed() << std::endl;
//                 ss << "connected: " << owner_->get_connected() << std::endl;
//                 ss << "guided:	" << owner_->get_guided() << std::endl;
//                 ss << "mode:	  " << owner_->get_mode() << std::endl;
//                 ss << "system_status:  " << owner_->get_system_status() << std::endl;
//                 ss << "x_home_pos:     " << owner_->get_x_home_pos() << ", y_home_pos: " << owner_->get_y_home_pos() << ", z_home_pos: " << owner_->get_z_home_pos() << std::endl;
//                 ss << "running_time: " << owner_->get_cur_time() << std::endl;
//                 RCLCPP_INFO_STREAM(owner_->get_node()->get_logger(), ss.str());
//                 ss.str(""); // 清空字符串流
//                 ss.clear(); // 清除状态标志
//             }
//         }
        
//     private:
//         PrintInfoState() : State("print_info", 1) {}
//         friend class State;
//     };
    
//     class TerminalControlState : public State<APMROS2Drone> {
//     public:
//         static const TerminalControlState& getInstance() {
//             static TerminalControlState instance;
//             return instance;
//         }
        
//     protected:
//         void executeImpl()  override {
//             RCLCPP_INFO(rclcpp::get_logger("TerminalControlState"), "执行终端控制状态");
//             // 终端控制逻辑可以在这里实现  char key = 0;
//             static std::string input = "";

//             if (_kbhit()) // 检查是否有按键输入
//             {
//                 key = _getch(); // 获取按键输入
//                 // 特殊状态处理（起飞解锁前）
//                 auto apm_sta_ctl = std::dynamic_pointer_cast<APMROS2StatusController>(owner_->sta_ctl);
//                 if (apm_sta_ctl && apm_sta_ctl->state_ == APMROS2StatusController::TakeoffState::wait_for_takeoff_command){
//                     if (key == '\n') // 检查是否按下回车键
//                     {
//                         apm_sta_ctl->takeoff_command = true; // 设置起飞命令
//                     }
//                     else if (key == 'q') // 检查是否按下q键
//                     {
//                         RCLCPP_INFO(owner_->get_node()->get_logger(), "退出程序");
//                         stateMachine.transitionTo(EndState::getInstance()); // 切换到结束状态
//                     }
//                     else if (key != 0)
//                     {
//                         RCLCPP_INFO(owner_->get_node()->get_logger(), "无效输入，请按回车键解锁无人机或按q键退出程序");
//                     }
//                     return;
//                 }
//                 // 处理多字符输入
//                 if (key == '\n' || key == '\r') { // 按下回车，尝试解析命令
//                 std::string upperInput = input;
//                 std::transform(upperInput.begin(), upperInput.end(), upperInput.begin(), ::toupper);
//                 auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
//                 const auto& allStates = stateMachine.getAllStates();
//                 auto it = allStates.find(upperInput);
//                 if (it != allStates.end()) {
//                     stateMachine.transitionTo(it->second);
//                     RCLCPP_INFO(owner_->get_node()->get_logger(), "切换到状态: %s", upperInput.c_str());
//                 } else {
//                     RCLCPP_INFO(owner_->get_node()->get_logger(), "无效指令: %s", input.c_str());
//                 }
//                 input.clear();
//                 } else if (key == '\b' && !input.empty()) { // 处理退格
//                 input.pop_back();
//                     } else if (key == '\t') { // Tab键自动补全
//                         std::string upperInput = input;
//                         std::transform(upperInput.begin(), upperInput.end(), upperInput.begin(), ::toupper);
//                         std::vector<std::string> candidates;
//                         auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
//                         const auto& allStates = stateMachine.getAllStates();
//                         for (const auto& kv : allStates) {
//                                 if (kv.first.find(upperInput) == 0) { // 前缀匹配
//                                         candidates.push_back(kv.first);
//                                 }
//                         }
//                         if (candidates.size() == 1) {
//                                 input = candidates[0];
//                                 RCLCPP_INFO(owner_->get_node()->get_logger(), "自动补全: %s", input.c_str());
//                         } else if (candidates.size() > 1) {
//                                 std::string msg = "可选项: ";
//                                 for (const auto& s : candidates) msg += s + " ";
//                                 RCLCPP_INFO(owner_->get_node()->get_logger(), "%s", msg.c_str());
//                         }
//                 } else if (key != 0) {
//                 input += key; // 将按键添加到输入字符串中
//                 }
//             }
//         }
        
//     private:
//         TerminalControlState() : State("terminal_control", 1) {}
//         friend class State;
//     };
    
//     class ReflushConfigState : public State<APMROS2Drone> {
//     public:
//         static const ReflushConfigState& getInstance() {
//             static ReflushConfigState instance;
//             return instance;
//         }
        
//     protected:
//         void executeImpl()  override {
//             RCLCPP_INFO(rclcpp::get_logger("ReflushConfigState"), "执行配置刷新状态");
//             // 配置刷新逻辑可以在这里实现onfig
//             auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
//             if (stateMachine.getCurrentState() == ReflushConfigState::getInstance()) {
//                 RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "开始刷新配置");
//                 // 读取配置文件
//                 owner_->read_configs("OffboardControl.yaml");
//                 owner_->pos_ctl->pid_x_defaults = PID::readPIDParameters("pos_config.yaml","pos_x");
//                 owner_->pos_ctl->pid_y_defaults = PID::readPIDParameters("pos_config.yaml","pos_y");
//                 owner_->pos_ctl->pid_z_defaults = PID::readPIDParameters("pos_config.yaml","pos_z");
//                 owner_->pos_ctl->pid_yaw_defaults = PID::readPIDParameters("pos_config.yaml","pos_yaw");
//                 owner_->pos_ctl->pid_px_defaults = PID::readPIDParameters("pos_config.yaml","pos_px");
//                 owner_->pos_ctl->pid_py_defaults = PID::readPIDParameters("pos_config.yaml","pos_py");
//                 owner_->pos_ctl->pid_pz_defaults = PID::readPIDParameters("pos_config.yaml","pos_pz");
//                 owner_->pos_ctl->pid_vx_defaults = PID::readPIDParameters("pos_config.yaml","pos_vx");
//                 owner_->pos_ctl->pid_vy_defaults = PID::readPIDParameters("pos_config.yaml","pos_vy");
//                 owner_->pos_ctl->pid_vz_defaults = PID::readPIDParameters("pos_config.yaml","pos_vz");
//                 owner_->pos_ctl->limit_defaults = owner_->pos_ctl->readLimits("pos_config.yaml","limits");
//                 // 重新设置PID参数
//                 owner_->pos_ctl->reset_pid();
//                 owner_->pos_ctl->set_limits(owner_->pos_ctl->limit_defaults);
//                 owner_->reset_wp_limits(); // 重置航点速度限制

//                 RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "配置刷新完成");
//                 auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
//                 if (!stateMachine.getPreviousStateName().empty()) {
//                     const auto& previousState = stateMachine.getState(stateMachine.getPreviousStateName());
//                     stateMachine.transitionTo(previousState); // 切换回上一个状态
//                 }
//             }
//         }
        
//     private:
//         ReflushConfigState() : State("reflush_config", 1) {}
//         friend class State;
//     };

//     class LandToStartState : public State<APMROS2Drone> {
//     public:
//         static const LandToStartState& getInstance() {
//             static LandToStartState instance;
//             return instance;
//         }
        
//     protected:
//         void executeImpl()  override {
//             RCLCPP_INFO(rclcpp::get_logger("LandToStartState"), "执行返回起点降落状态");
//             // 返回起点降落逻辑可以在这里实现
//         }
        
//     private:
//         LandToStartState() : State("land_to_start", 1) {}
//         friend class State;
//     };
// // }

// // // 状态工厂函数
// // namespace StateFactory {
// //     const State& getInitState() {
// //         return States::InitState::getInstance();
// //     }
    
// //     const State& getTakeoffState() {
// //         return States::TakeoffState::getInstance();
// //     }
    
// //     const State& getEndState() {
// //         return States::EndState::getInstance();
// //     }
    
// //     const State& getStopState() {
// //         return States::StopState::getInstance();
// //     }
    
// //     const State& getGotoShotPointState() {
// //         return States::GotoShotPointState::getInstance();
// //     }
    
// //     const State& getDoshotState() {
// //         return States::DoshotState::getInstance();
// //     }
    
// //     const State& getGotoScoutPointState() {
// //         return States::GotoScoutPointState::getInstance();
// //     }
    
// //     const State& getSurroundSeeState() {
// //         return States::SurroundSeeState::getInstance();
// //     }
    
// //     const State& getDolandState() {
// //         return States::DolandState::getInstance();
// //     }
    
// //     const State& getMypidState() {
// //         return States::MypidState::getInstance();
// //     }
    
// //     const State& getPrintInfoState() {
// //         return States::PrintInfoState::getInstance();
// //     }
    
// //     const State& getTerminalControlState() {
// //         return States::TerminalControlState::getInstance();
// //     }
    
// //     const State& getReflushConfigState() {
// //         return States::ReflushConfigState::getInstance();
// //     }
    
// //     const State& getLandToStartState() {
// //         return States::LandToStartState::getInstance();
// //     }
// // }
