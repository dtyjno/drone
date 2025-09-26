#include "StopState.h"
#include "../APMROS2Drone.h"
#include "../../stateMachine/StateMachine.h"

void StopState::executeImpl()  {
    // 停止逻辑可以在这里实现
    auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
    if (stateMachine.getCurrentState() == StopState::getInstance()) {
        RCLCPP_INFO_ONCE(rclcpp::get_logger("StopState"), "执行停止状态");
        RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "停止状态");
        // 停止飞行器的所有运动
        owner_->send_velocity_command(0.0, 0.0, 0.0, 0.0);
        // 可以在这里添加其他停止逻辑
    }
}
