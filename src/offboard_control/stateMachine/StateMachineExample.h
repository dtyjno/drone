#pragma once
#include "StateMachine.h"
#include "State.h"
#include "../APMROS2drone/APMROS2Drone.h"

// 状态机使用示例类
class StateMachineExample {
public:
    StateMachineExample(std::shared_ptr<APMROS2Drone> drone) : drone_(drone) {
        // 获取状态机实例
        auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
        
        // 设置拥有者对象
        stateMachine.setOwner(drone);
        
        // 设置状态改变回调函数
        stateMachine.setStateChangeCallback(
            [this](const State<APMROS2Drone>& oldState, const State<APMROS2Drone>& newState) {
                onStateChanged(oldState, newState);
            }
        );
        
        // 启动状态机
        stateMachine.start();
        
        RCLCPP_INFO(drone->node->get_logger(), "StateMachine initialized with %zu states", 
                   stateMachine.getStateCount());
    }
    
    ~StateMachineExample() {
        auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
        stateMachine.stop();
    }
    
    // 启动状态机执行
    void startExecution() {
        auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
        
        if (!stateMachine.isRunning()) {
            RCLCPP_WARN(drone_->node->get_logger(), "StateMachine is not running!");
            return;
        }
        
        // 设置初始状态为InitState
        try {
            const auto& initState = stateMachine.getState("init");
            stateMachine.setCurrentState(initState);
            RCLCPP_INFO(drone_->node->get_logger(), "Set initial state to: %s", 
                       initState.getName().c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(drone_->node->get_logger(), "Failed to set initial state: %s", e.what());
        }
    }
    
    // 执行当前状态
    void executeCurrentState() {
        auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
        
        if (!stateMachine.isRunning()) {
            return;
        }
        
        try {
            auto currentState = stateMachine.getCurrentState();
            currentState.execute();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(drone_->node->get_logger(), "Error executing current state: %s", e.what());
        }
    }
    
    // 异步执行当前状态
    std::future<void> executeCurrentStateAsync() {
        auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
        
        if (!stateMachine.isRunning()) {
            // 返回一个已完成的future
            std::promise<void> promise;
            promise.set_value();
            return promise.get_future();
        }
        
        try {
            auto currentState = stateMachine.getCurrentState();
            return currentState.executeAsync();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(drone_->node->get_logger(), "Error executing current state async: %s", e.what());
            std::promise<void> promise;
            promise.set_exception(std::current_exception());
            return promise.get_future();
        }
    }
    
    // 转换到指定状态
    void transitionToState(const std::string& stateName) {
        auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
        
        if (!stateMachine.isRunning()) {
            RCLCPP_WARN(drone_->node->get_logger(), "StateMachine is not running!");
            return;
        }
        
        try {
            stateMachine.transitionTo(stateName);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(drone_->node->get_logger(), "Failed to transition to state '%s': %s", 
                        stateName.c_str(), e.what());
        }
    }
    
    // 获取当前状态信息
    std::string getCurrentStateName() {
        auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
        
        try {
            auto currentState = stateMachine.getCurrentState();
            return currentState.getName();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(drone_->node->get_logger(), "Error getting current state: %s", e.what());
            return "unknown";
        }
    }
    
    // 检查状态是否存在
    bool hasState(const std::string& stateName) {
        auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
        return stateMachine.hasState(stateName);
    }
    
    // 获取所有状态信息
    void printAllStates() {
        auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
        const auto& allStates = stateMachine.getAllStates();
        
        RCLCPP_INFO(drone_->node->get_logger(), "Available states (%zu total):", allStates.size());
        for (const auto& [name, state] : allStates) {
            RCLCPP_INFO(drone_->node->get_logger(), "  - %s (index: %d, publish_index: %d, executions: %d)", 
                       name.c_str(), state.getIndex(), state.getPublishIndex(), state.getExecutionCount());
        }
    }
    
    // 状态改变回调函数
    void onStateChanged(const State<APMROS2Drone>& oldState, const State<APMROS2Drone>& newState) {
        RCLCPP_INFO(drone_->node->get_logger(), 
                   "State changed: %s -> %s", 
                   oldState.getName().c_str(), 
                   newState.getName().c_str());
        
        // 在状态改变时可以执行一些通用逻辑
        // 例如：记录日志、重置计时器、清理资源等
    }
    
    // 批量执行所有状态（测试用）
    void executeAllStatesTest() {
        auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
        
        RCLCPP_INFO(drone_->node->get_logger(), "Executing all states for testing...");
        
        try {
            stateMachine.executeAllStates();
            RCLCPP_INFO(drone_->node->get_logger(), "All states executed successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(drone_->node->get_logger(), "Error executing all states: %s", e.what());
        }
    }
    
    // 异步批量执行所有状态
    void executeAllStatesAsync() {
        auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
        
        RCLCPP_INFO(drone_->node->get_logger(), "Executing all states asynchronously...");
        
        try {
            stateMachine.executeAllStatesAsync();
            RCLCPP_INFO(drone_->node->get_logger(), "All states execution started asynchronously");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(drone_->node->get_logger(), "Error executing all states async: %s", e.what());
        }
    }
    
    // 获取状态机状态
    bool isStateMachineRunning() {
        auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
        return stateMachine.isRunning();
    }
    
    // 停止状态机
    void stopStateMachine() {
        auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
        stateMachine.stop();
        RCLCPP_INFO(drone_->node->get_logger(), "StateMachine stopped");
    }
    
private:
    std::shared_ptr<APMROS2Drone> drone_;
};
