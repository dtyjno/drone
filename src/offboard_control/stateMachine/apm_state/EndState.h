#pragma once

#include "../../stateMachine/State.h"

class APMROS2Drone;

class EndState : public State<APMROS2Drone> {
public:
    static const EndState& getInstance() {
        RCLCPP_INFO_ONCE(rclcpp::get_logger("EndState"), "获取结束状态实例");
        static EndState instance;
        return instance;
    }
    
protected:
    void executeImpl() override;
private:
    EndState() : State("EndState", 2) {}
    friend class State;
};
