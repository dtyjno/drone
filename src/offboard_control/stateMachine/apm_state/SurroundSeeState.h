#pragma once

#include "../../stateMachine/State.h"

class APMROS2Drone;

class SurroundSeeState : public State<APMROS2Drone> {
public:
    static const SurroundSeeState& getInstance() {
        RCLCPP_INFO_ONCE(rclcpp::get_logger("SurroundSeeState"), "获取环绕侦查状态实例");
        static SurroundSeeState instance;
        return instance;
    }
    
protected:
    void executeImpl() override;
    
private:
    SurroundSeeState() : State("surround_see", 3) {}
    friend class State;
};