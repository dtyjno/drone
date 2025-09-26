#pragma once

#include "../../stateMachine/State.h"

class APMROS2Drone;

class GotoShotPointState : public State<APMROS2Drone> {
public:
    static const GotoShotPointState& getInstance() {
        RCLCPP_INFO_ONCE(rclcpp::get_logger("GotoShotPointState"), "获取前往投弹点状态实例");
        static GotoShotPointState instance;
        return instance;
    }
    
protected:
    void executeImpl() override;
    
private:
    GotoShotPointState() : State("GotoShotPointState", 1) {}
    friend class State;
};
