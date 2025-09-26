#include "ReflushConfigState.h"
#include "../APMROS2Drone.h"
#include "../../stateMachine/StateMachine.h"

void ReflushConfigState::executeImpl()  {
    // 配置刷新逻辑可以在这里实现onfig
    auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
    if (stateMachine.getCurrentState() == ReflushConfigState::getInstance()) {
        RCLCPP_INFO_ONCE(rclcpp::get_logger("ReflushConfigState"), "执行配置刷新状态");
        // RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "开始刷新配置");
        // 读取配置文件
        owner_->read_configs("OffboardControl.yaml");
        owner_->pos_ctl->pid_x_defaults = PID::readPIDParameters("pos_config.yaml","pos_x");
        owner_->pos_ctl->pid_y_defaults = PID::readPIDParameters("pos_config.yaml","pos_y");
        owner_->pos_ctl->pid_z_defaults = PID::readPIDParameters("pos_config.yaml","pos_z");
        owner_->pos_ctl->pid_yaw_defaults = PID::readPIDParameters("pos_config.yaml","pos_yaw");
        owner_->pos_ctl->pid_px_defaults = PID::readPIDParameters("pos_config.yaml","pos_px");
        owner_->pos_ctl->pid_py_defaults = PID::readPIDParameters("pos_config.yaml","pos_py");
        owner_->pos_ctl->pid_pz_defaults = PID::readPIDParameters("pos_config.yaml","pos_pz");
        owner_->pos_ctl->pid_vx_defaults = PID::readPIDParameters("pos_config.yaml","pos_vx");
        owner_->pos_ctl->pid_vy_defaults = PID::readPIDParameters("pos_config.yaml","pos_vy");
        owner_->pos_ctl->pid_vz_defaults = PID::readPIDParameters("pos_config.yaml","pos_vz");
        owner_->pos_ctl->limit_defaults = owner_->pos_ctl->readLimits("pos_config.yaml","limits");
        // 重新设置PID参数
        owner_->pos_ctl->reset_pid();
        owner_->pos_ctl->set_limits(owner_->pos_ctl->limit_defaults);
        owner_->reset_wp_limits(); // 重置航点速度限制

        RCLCPP_INFO_ONCE(owner_->get_node()->get_logger(), "配置刷新完成");
        auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
        if (!stateMachine.getPreviousStateName().empty()) {
            const auto& previousState = stateMachine.getState(stateMachine.getPreviousStateName());
            stateMachine.transitionTo(previousState); // 切换回上一个状态
        }
    }
}
