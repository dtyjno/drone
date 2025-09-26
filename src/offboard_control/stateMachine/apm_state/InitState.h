#pragma once

#include "../../stateMachine/State.h"

class APMROS2Drone;

// 预定义的状态实例
// namespace States {
    // 创建具体的状态类
    class InitState : public State<APMROS2Drone> {
    public:
        static const InitState& getInstance() {
            RCLCPP_INFO_ONCE(rclcpp::get_logger("InitState"), "获取初始化状态实例");
            static InitState instance;
            return instance;
        }
        
    protected:
        void executeImpl() override;
        
    private:
        InitState() : State("InitState", 1) {}
        friend class State;
    };
// }