#pragma once
#include <memory>
#include <vector>
#include <functional>
#include <map>
#include <mutex>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <thread>
#include <queue>
#include <condition_variable>
#include <future>
// #include "State.h"

// 前向声明
template<typename T>
class State;

// 状态机类，管理所有状态的执行和转换
template<typename T>
class StateMachine {
public:
    // 单例模式
    static StateMachine& getInstance() {
        static StateMachine instance;
        return instance;
    }

    // 删除拷贝构造函数和赋值操作符
    StateMachine(StateMachine&) = delete;
    StateMachine& operator=(StateMachine&) = delete;

    ~StateMachine() {
        stop();
        RCLCPP_INFO(rclcpp::get_logger("StateMachine"), "StateMachine instance destroyed");
    }

    // 注册状态
    void registerState(const std::string& name, State<T>& state);

    // 获取状态
    State<T>& getState(const std::string& name) {
        auto it = all_states_map_.find(name);
        if (it != all_states_map_.end()) {
            return *(it->second);
        }
        throw std::runtime_error("State not found: " + name);
    }

    // 获取所有状态
    std::map<std::string, State<T>*>& getAllStates() {
        return all_states_map_;
    }

    // 同步执行所有状态
    void executeAllStates() {
        
        RCLCPP_DEBUG(rclcpp::get_logger("StateMachine"), 
                    "Executing all states synchronously (%zu states)", 
                    all_states_map_.size());
        
        // 将所有状态的执行任务提交到线程池
        for (auto& kv : all_states_map_) {
            kv.second->execute();
        }
        
        RCLCPP_DEBUG(rclcpp::get_logger("StateMachine"), 
                    "All states execution completed");
    }

    // 执行单个状态
    void executeState(const std::string& name) {
        auto& state = getState(name);
        state.execute();
    }

    // 获取当前状态名称
    std::string getCurrentStateName() {
        return current_state_name_;
    }

    // 获取上一个状态名称
    std::string getPreviousStateName() {
        return previous_state_name_;
    }

    // 检查是否有当前状态
    bool hasCurrentState() {
        return !current_state_name_.empty();
    }

    // 获取当前状态
    State<T>& getCurrentState() {
        if (current_state_name_.empty()) {
            throw std::runtime_error("No current state set");
        }
        return getState(current_state_name_);
    }

    // 获取上一个状态
    State<T>& getPreviousState() {
        if (previous_state_name_.empty()) {
            throw std::runtime_error("No previous state available");
        }
        return getState(previous_state_name_);
    }

    // 设置当前状态
    void setCurrentState(const State<T>& state) {        
        RCLCPP_INFO(rclcpp::get_logger("StateMachine"), 
                   "State transition: %s -> %s", 
                   current_state_name_.c_str(), 
                   state.getName().c_str());

        if (state.getName() == current_state_name_) {
            RCLCPP_INFO(rclcpp::get_logger("StateMachine"), 
                       "State unchanged, maintaining current state: %s", 
                       current_state_name_.c_str());
            return;
        }

        previous_state_name_ = current_state_name_;
        current_state_name_ = state.getName();
        is_first_run_ = true;
        
        // 通知状态改变监听器
        // if (!previous_state_name_.empty()) {
        //     auto& previousState = getState(previous_state_name_);
        // }
    }


    void setCurrentState(const std::string& name) {
        auto& state = getState(name);
        setCurrentState(state);
    }

    // 状态转换
    void transitionTo(const std::string& stateName) {
        auto& newState = getState(stateName);
        setCurrentState(newState);
    }

    void transitionTo(const State<T>& state) {
        auto& newState = getState(state.getName());
        setCurrentState(newState);
    }

    // 检查是否是第一次运行
    bool isFirstRun() {
        return is_first_run_;
    }

    // 设置第一次运行标志
    void setFirstRun(bool firstRun) {
        is_first_run_ = firstRun;
    }

    // 获取状态数量
    size_t getStateCount() {
        return all_states_map_.size();
    }

    // 清空所有状态
    void clearStates() {
        all_states_map_.clear();
        RCLCPP_INFO(rclcpp::get_logger("StateMachine"), "All states cleared");
    }

    // 检查状态是否存在
    bool hasState(const std::string& name) {
        return all_states_map_.find(name) != all_states_map_.end();
    }

    // 移除状态
    void removeState(const std::string& name) {
        auto it = all_states_map_.find(name);
        if (it != all_states_map_.end()) {
            all_states_map_.erase(it);
            RCLCPP_INFO(rclcpp::get_logger("StateMachine"), 
                       "Removed state: %s", name.c_str());
        }
    }

    // 设置状态改变回调函数
    void setStateChangeCallback(std::function<void(State<T>&, State<T>&)> callback) {
        state_change_callback_ = callback;
    }

    // 启动状态机
    void start() {
        is_running_ = true;
        RCLCPP_INFO(rclcpp::get_logger("StateMachine"), "State machine started");
    }

    // 停止状态机
    void stop() {
        is_running_ = false;
        RCLCPP_INFO(rclcpp::get_logger("StateMachine"), "State machine stopped");
    }

    // 检查状态机是否运行
    bool isRunning() {
        return is_running_;
    }

    // 设置拥有者对象
    void setOwner(std::shared_ptr<T> owner) {
        owner_ = owner;
        // 设置所有状态的拥有者
        for (auto& kv : all_states_map_) {
            kv.second->setOwner(owner);
        }
        RCLCPP_INFO(rclcpp::get_logger("StateMachine"), "Owner set for StateMachine and all states");
    }

    // 获取拥有者对象
    std::shared_ptr<T> getOwner() {
        return owner_;
    }

private:
    StateMachine() : current_state_name_(""), previous_state_name_(""), is_first_run_(true), is_running_(false) {
        RCLCPP_INFO(rclcpp::get_logger("StateMachine"), "StateMachine instance created");
    }

    // 成员变量
    std::map<std::string, State<T>*> all_states_map_;
    
    std::string current_state_name_;  // 使用状态名称而不是State对象
    std::string previous_state_name_; // 使用状态名称而不是State对象
    bool is_first_run_;
    std::atomic<bool> is_running_;
    
    std::shared_ptr<T> owner_ = nullptr;
    std::function<void(State<T>&, State<T>&)> state_change_callback_;
};
