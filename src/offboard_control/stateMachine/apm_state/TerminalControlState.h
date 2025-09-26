#pragma once

#include "../../stateMachine/State.h"

class APMROS2Drone;

class TerminalControlState : public State<APMROS2Drone> {
public:
    static const TerminalControlState& getInstance() {
        RCLCPP_INFO_ONCE(rclcpp::get_logger("TerminalControlState"), "获取终端控制状态实例");
        static TerminalControlState instance;
        return instance;
    }
    
protected:
    void executeImpl() override;
private:
    TerminalControlState() : State("terminal_control", 1) {}
    friend class State<APMROS2Drone>;
};
