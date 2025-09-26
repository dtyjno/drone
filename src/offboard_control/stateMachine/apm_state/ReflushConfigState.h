#pragma once

#include "../../stateMachine/State.h"

class APMROS2Drone;


class ReflushConfigState : public State<APMROS2Drone> {
public:
    static const ReflushConfigState& getInstance() {
        RCLCPP_INFO_ONCE(rclcpp::get_logger("ReflushConfigState"), "获取配置刷新状态实例");
        static ReflushConfigState instance;
        return instance;
    }
    
protected:
    void executeImpl() override;
    
private:
    ReflushConfigState() : State("reflush_config", 1) {}
    friend class State;
};
