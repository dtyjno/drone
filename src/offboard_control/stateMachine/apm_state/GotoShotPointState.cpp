#include "GotoShotPointState.h"
#include "../APMROS2Drone.h"
#include "../../stateMachine/StateMachine.h"
#include "DoshotState.h"

void GotoShotPointState::executeImpl()  {
    // 前往投弹点逻辑可以在这里实现
    auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
    if (stateMachine.getCurrentState() == GotoShotPointState::getInstance()) {
        RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "开始前往投弹区起点");
        float x_shot, y_shot;
        owner_->rotate_global2stand(owner_->dx_shot, owner_->dy_shot + owner_->shot_width_max / 2, x_shot, y_shot);
        if(owner_->waypoint_timer_.elapsed() > 12)
        {
            owner_->waypoint_timer_.set_start_time_to_default();
            RCLCPP_INFO(owner_->get_node()->get_logger(), "到达投弹区起点");
            stateMachine.transitionTo(DoshotState::getInstance());
        } else {
            owner_->send_start_setpoint_command(
                x_shot, y_shot, owner_->shot_halt, 0
            );
            RCLCPP_INFO_THROTTLE(owner_->get_node()->get_logger(), *owner_->get_node()->get_clock(), 3000, "(THROTTLE 3s)前往投弹区中...%f", owner_->waypoint_timer_.elapsed());
        }
    }
}