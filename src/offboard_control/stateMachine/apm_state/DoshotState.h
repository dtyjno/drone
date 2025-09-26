#pragma once

#include "../../stateMachine/State.h"

class APMROS2Drone;

class DoshotState : public State<APMROS2Drone> {
public:
    static const DoshotState& getInstance() {
		RCLCPP_INFO_ONCE(rclcpp::get_logger("DoshotState"), "获取投弹状态实例");
        static DoshotState instance;
        return instance;
    }
    
	enum class DoshotStateAttribute
	{
		doshot_init,
		doshot_scout,
		doshot_shot,
		doshot_wait,
		doshot_end
	} doshot_state_;

protected:
    void executeImpl() override;
private:
    DoshotState() : State("DoshotState", 0) {}
    friend class State;
};
