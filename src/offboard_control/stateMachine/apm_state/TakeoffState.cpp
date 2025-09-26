#include "TakeoffState.h"
#include "../APMROS2Drone.h"
#include "../../stateMachine/StateMachine.h"
#include "GotoShotPointState.h"

void TakeoffState::executeImpl()  {
    // 起飞逻辑可以在这里实现
    auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
    if (stateMachine.getCurrentState() == TakeoffState::getInstance()) {
        RCLCPP_INFO_ONCE(rclcpp::get_logger("TakeoffState"), "执行起飞状态");
        RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "开始起飞");

        if (owner_->sta_ctl->takeoff(owner_->get_z_pos(), 2.0, owner_->get_yaw())) {
                RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "起飞成功");
                stateMachine.transitionTo(GotoShotPointState::getInstance());
        } else {
                // RCLCPP_INFO(owner_->get_node()->get_logger(), "起飞失败");
        }
    }
}

