#pragma once

#include "../../stateMachine/State.h"

class APMROS2Drone;

class GotoScoutPointState : public State<APMROS2Drone> {
public:
    static const GotoScoutPointState& getInstance() {
        RCLCPP_INFO_ONCE(rclcpp::get_logger("GotoScoutPointState"), "获取前往侦查点状态实例");
        static GotoScoutPointState instance;
        return instance;
    }
    
protected:
    void executeImpl() override;
    
private:
    GotoScoutPointState() : State("goto_scoutpoint", 1) {}
    friend class State;
};
