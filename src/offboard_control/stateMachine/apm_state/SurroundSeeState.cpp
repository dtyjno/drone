#include "SurroundSeeState.h"
#include "../APMROS2Drone.h"
#include "../../stateMachine/StateMachine.h"
#include "DolandState.h"

void SurroundSeeState::executeImpl()  {
    // 环绕侦查逻辑可以在这里实现
    auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
    if (stateMachine.getCurrentState() == SurroundSeeState::getInstance()) {
        RCLCPP_INFO_ONCE(rclcpp::get_logger("SurroundSeeState"), "执行环绕侦查状态");
        static int counter = 0; // 航点计数器
        if (owner_->waypoint_goto_next(
            owner_->dx_see, owner_->dy_see, owner_->see_length - 2.0, owner_->see_width - 0.2, 
            owner_->see_halt, owner_->surround_see_points, 3.5, &counter, "侦查区"))
        {
            RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "侦查完毕");
            counter = 0;
            // owner_->sta_ctl->switch_mode("RTL");
            // rclcpp::sleep_for(std::chrono::seconds(17));
            // owner_->sta_ctl->switch_mode("GUIDED");
            stateMachine.transitionTo(DolandState::getInstance());
        }
    }
}