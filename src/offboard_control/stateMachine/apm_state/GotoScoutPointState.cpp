#include "GotoScoutPointState.h"
#include "../APMROS2Drone.h"
#include "../../stateMachine/StateMachine.h"
#include "SurroundSeeState.h"

void GotoScoutPointState::executeImpl()  {
    // 前往侦查点逻辑可以在这里实现
    auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
    if (stateMachine.getCurrentState() == GotoScoutPointState::getInstance()) {
        RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "开始前往侦查起点");
        float x_see, y_see;
        owner_->rotate_global2stand(owner_->dx_see, owner_->dy_see, x_see, y_see);
        if(owner_->waypoint_timer_.elapsed() > 7.5)
        {
            owner_->waypoint_timer_.set_start_time_to_default();
            RCLCPP_INFO(owner_->get_node()->get_logger(), "到达侦查区起点");
            stateMachine.transitionTo(SurroundSeeState::getInstance());
        } else {
            owner_->send_start_setpoint_command(
                x_see, y_see, owner_->see_halt, 0
            );
            RCLCPP_INFO_THROTTLE(owner_->get_node()->get_logger(), *owner_->get_node()->get_clock(), 3000, "(THROTTLE 3s)前往侦查区中...%f", owner_->waypoint_timer_.elapsed());
        }
    }
}
