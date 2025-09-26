#pragma once

#include "../../stateMachine/State.h"

class APMROS2Drone;

class StopState : public State<APMROS2Drone> {
public:
    static const StopState& getInstance() {
        RCLCPP_INFO_ONCE(rclcpp::get_logger("StopState"), "获取停止状态实例");
        static StopState instance;
        return instance;
    }
    
protected:
    void executeImpl() override;
    
private:
    StopState() : State("stop", 1) {}
    friend class State;
};
