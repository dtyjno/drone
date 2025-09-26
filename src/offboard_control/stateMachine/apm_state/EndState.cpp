#include "EndState.h"
#include "../APMROS2Drone.h"
#include "../../stateMachine/StateMachine.h"

void EndState::executeImpl()  {
    // 结束逻辑可以在这里实现
    auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
    if (stateMachine.getCurrentState() == EndState::getInstance()) {
        RCLCPP_INFO(rclcpp::get_logger("EndState"), "执行结束状态");
        RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "任务开始时间: %f 秒", owner_->get_start_time());
        RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "任务结束, 运行时间: %f 秒", owner_->get_cur_time());
        RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "任务结束, 任务运行时间: %f 秒", owner_->get_cur_time() - owner_->get_start_time());
        if (owner_->state_timer_.elapsed() < 3) {
            return; // 等待 3 秒后结束
        }
        // 如果需要，可以在这里添加清理或退出逻辑
        rclcpp::shutdown();  // 停止 ROS 2 节点
        owner_.reset(); // 释放 shared_ptr 引用
    }
}
    