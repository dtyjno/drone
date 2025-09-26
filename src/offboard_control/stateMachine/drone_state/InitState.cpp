#include "InitState.h"
#include "../AbstractDrone.h"
#include "../../stateMachine/StateMachine.h"


void AbstractDroneInitState::executeImpl()  {
    RCLCPP_INFO(rclcpp::get_logger("AbstractDroneInitState"), "执行初始化状态");
    // 初始化逻辑可以在这里实现
    auto& stateMachine = StateMachine<AbstractDrone>::getInstance();
    if (stateMachine.getCurrentStateName() == "AbstractDroneInitState") {
        std::cout << "AbstractDroneInitState::executeImpl()" << std::endl;
        // owner_->FlyState_init();
        // RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "初始化完成");
        // if (owner_->debug_mode_) {
        //     RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "测试模式下, 不进行起飞");
        //     transition_to(GotoShotPointState::getInstance());
        //     return;
        // }
        // transition_to(TakeoffState::getInstance());
    }
}

