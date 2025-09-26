#include "InitState.h"
#include "../APMROS2Drone.h"
#include "../../stateMachine/StateMachine.h"
#include "GotoShotPointState.h"
#include "TakeoffState.h"

void InitState::executeImpl()  {
    // 初始化逻辑可以在这里实现
    auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
    if (stateMachine.getCurrentState() == InitState::getInstance()) {
        RCLCPP_INFO_ONCE(rclcpp::get_logger("InitState"), "执行初始化状态");
        owner_->FlyState_init();
        RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "初始化完成");
        if (owner_->debug_mode_) {
            RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "测试模式下, 不进行起飞");
            transition_to(GotoShotPointState::getInstance());
            return;
        }
        transition_to(TakeoffState::getInstance());
    }
}

