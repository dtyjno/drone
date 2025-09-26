#pragma once

#include "../../stateMachine/State.h"

class APMROS2Drone;

class TakeoffState : public State<APMROS2Drone> {
public:
    static const TakeoffState& getInstance() {
        RCLCPP_INFO_ONCE(rclcpp::get_logger("TakeoffState"), "获取起飞状态实例");
        static TakeoffState instance;
        return instance;
    }
    
protected:
    void executeImpl() override;
    
private:
    TakeoffState() : State("TakeoffState", 1) {}
    friend class State;
};
