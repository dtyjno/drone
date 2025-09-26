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
#include "utils/utils.h"

// 前向声明
template<typename T>
class StateMachine;
class ThreadPool;

// using namespace std::chrono_literals;

// // 前向声明
// class OffboardControl;
// class APMROS2Drone;


// 前向声明
template<typename T>
class StateMachine;

// 状态类
template<typename T>
class State {
public:
	
	std::string getName() const {
		return name_;
	}
	
	int getIndex() const {
		return index_;
	}
	
	int getPublishIndex() const {
		return publish_index_.load();
	}
	
	void setPublishIndex(int idx) {
		publish_index_.store(idx);
	}
	
	// 比较操作符
	bool operator==(const State& other) const {
		return index_ == other.index_;
	}
	
	bool operator!=(const State& other) const {
		return index_ != other.index_;
	}

	virtual void execute() {
		// 线程安全的执行函数
		execution_count_.fetch_add(1);
		RCLCPP_DEBUG(rclcpp::get_logger("State"), 
					"Executing state: %s (execution count: %d)", 
					name_.c_str(), execution_count_.load());
		
		// Placeholder for any state-specific execution logic
		executeImpl();
	}

	// 获取执行次数（线程安全）
	int getExecutionCount() const {
		return execution_count_.load();
	}

	// 重置执行次数
	void resetExecutionCount() {
		execution_count_.store(0);
	}

	// 获取当前状态 - 通过StateMachine
	static State getCurrentState() {
		auto& stateMachine = StateMachine<T>::getInstance();
		return stateMachine.getCurrentState();
	}

	void setOwner(std::shared_ptr<T> owner) {
		this->owner_ = owner;
	}
protected:
	// 允许StateMachine访问私有成员
	friend class StateMachine<T>;
	
	std::shared_ptr<T> owner_;


	// 派生类应该重写这个方法而不是execute()
	virtual void executeImpl() {
		// 默认实现为空
	}

	void transition_to(const State& new_state) const {
		auto& stateMachine = StateMachine<T>::getInstance();

		RCLCPP_INFO(owner_->get_node()->get_logger(), "状态转换: %s -> %s",
					stateMachine.getCurrentState().getName().c_str(),
					new_state.getName().c_str());

		// reset_all_tasks(); // 重置所有任务 - TODO: Move this call to avoid circular dependency

		owner_->waypoint_timer_.reset(); // 重置航点计时器
		owner_->state_timer_.reset(); // 重置状态计时器
		// owner_->reset_wp_limits();
		owner_->is_first_run_ = true; // 重置第一次运行标志
		
		// 使用StateMachine进行状态转换
		stateMachine.setCurrentState(new_state);
	}
	
	// 构造函数 - 现在通过StateMachine注册状态
	State(const std::string& name, int publish_index = 1) 
		: name_(name), publish_index_(publish_index), execution_count_(0) {
		// 在构造时自动注册到StateMachine
		auto& stateMachine = StateMachine<T>::getInstance();
		stateMachine.registerState(name_, *this);
	}

	// 删除拷贝构造函数和赋值操作符，因为std::atomic不能被拷贝
	State(const State&) = delete;
	State& operator=(const State&) = delete;

	// 允许移动构造和赋值（如果需要的话）
	State(State&&) = default;
	State& operator=(State&&) = default;

private:
	// 静态成员变量保持，但不再直接管理状态映射
	static int index_counter_;
	
	// 实例成员变量
	int index_ = index_counter_++;
	std::string name_;
	std::atomic<int> publish_index_;
	mutable std::atomic<int> execution_count_;
};
