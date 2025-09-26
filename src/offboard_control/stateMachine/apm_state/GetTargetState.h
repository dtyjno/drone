#pragma once

#include "../../stateMachine/State.h"
#include "algorithm/clustering.h"

class APMROS2Drone;

// 预定义的状态实例
// namespace States {
// 创建具体的状态类
class GetTargetState : public State<APMROS2Drone> {
public:
    static const GetTargetState& getInstance() {
        RCLCPP_INFO_ONCE(rclcpp::get_logger("GetTargetState"), "获取初始化状态实例");
        static GetTargetState instance;
        return instance;
    }
    
    // 计算目标位置
    std::vector<Circles> Target_Samples;// 全局变量，存储目标样本点
    
protected:
    void executeImpl() override;
    
private:
    GetTargetState() : State("GetTargetState", 1) {}
    friend class State;
};
// }