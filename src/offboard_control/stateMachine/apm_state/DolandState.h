#pragma once
#include "../../stateMachine/State.h"

class APMROS2Drone;

class DolandState : public State<APMROS2Drone> {
public:
    static const DolandState& getInstance() {
        RCLCPP_INFO_ONCE(rclcpp::get_logger("DolandState"), "获取降落状态实例");
        static DolandState instance;
        return instance;
    }
    
protected:
    void executeImpl() override;
private:
    DolandState() : State("DolandState", 4) {}
    friend class State;
};
