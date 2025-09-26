#pragma once

#include "../../stateMachine/State.h"

class AbstractDrone;

// 预定义的状态实例
    // 创建具体的状态类
    class AbstractDroneInitState : public State<AbstractDrone> {
    public:
        static const AbstractDroneInitState& getInstance() {
            static AbstractDroneInitState instance;
            return instance;
        }
        
    protected:
        void executeImpl() override;
        
    private:
        AbstractDroneInitState() : State("AbstractDroneInitState", 1) {}
        friend class State;
    };
