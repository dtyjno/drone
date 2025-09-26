#include "DolandState.h"
#include "../APMROS2Drone.h"
#include "../../stateMachine/StateMachine.h"
#include "EndState.h"

void DolandState::executeImpl()  {
    // 降落逻辑可以在这里实现
    auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
    if (stateMachine.getCurrentState() == DolandState::getInstance()) {
        RCLCPP_INFO(rclcpp::get_logger("DolandState"), "执行降落状态");
        static enum class DolandState {
            doland_init, // 降落初始化
            doland_wait, // 等待降落
            doland_landing, // 降落中
            doland_end // 降落结束
        } doland_state = DolandState::doland_init; // 降落状态
        if (owner_->is_first_run_) {
            doland_state = DolandState::doland_init; // 重置降落状态
            owner_->is_first_run_ = false; // 重置第一次运行标志
        }
        while (true){
            switch (doland_state) {
            case DolandState::doland_init: // 降落初始化
                RCLCPP_INFO(owner_->get_node()->get_logger(), "开始降落");
                owner_->sta_ctl->switch_mode("RTL");
                doland_state = DolandState::doland_wait; // 切换到等待降落状态
                continue; // 继续执行下一次循环;
            case DolandState::doland_wait: // 等待降落
                if (owner_->state_timer_.elapsed() > 18) { // 如果等待超过18秒
                    RCLCPP_INFO(owner_->get_node()->get_logger(), "等待降落超过18秒，开始降落");
                    owner_->sta_ctl->switch_mode("GUIDED");
                    doland_state = DolandState::doland_landing; // 切换到降落中状态
                    continue; // 继续执行下一次循环;
                } else {
                    RCLCPP_INFO_THROTTLE(owner_->get_node()->get_logger(), *owner_->get_node()->get_clock(), 3000, "(THROTTLE 3s)等待降落中...%f", owner_->state_timer_.elapsed());
                }
                break;
            case DolandState::doland_landing: // 降落中
                if(owner_->Doland()){
                    doland_state = DolandState::doland_end; // 切换到降落结束状态
                    continue; // 继续执行下一次循环;
                }
                break; // 继续执行下一次循环;
            case DolandState::doland_end: // 降落结束
                RCLCPP_INFO(owner_->get_node()->get_logger(), "降落完成");
                owner_->sta_ctl->switch_mode("LAND");
                stateMachine.transitionTo(EndState::getInstance());
                doland_state = DolandState::doland_init; // 重置降落状态
                break; // 结束函数
            default:
                break;
            }
            break;
        }
    }
    return; // 结束函数
}
