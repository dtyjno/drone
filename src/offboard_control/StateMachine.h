#pragma once
#include <memory>
#include <vector>
#include <functional>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include "utils.h"

using namespace std::chrono_literals;

// 前向声明
class OffboardControl;
// enum class FlyState;
// extern const std::map<std::string, FlyState> FlyStateMap;
// int fly_state_to_int(FlyState state);

enum class FlyState
{
	init,
	// request,
	takeoff,
	end,
	stop,
	Goto_shotpoint, 
	Doshot,
	Goto_scoutpoint,
	Surround_see,
	Doland,
	MYPID,
	Print_Info,
	Termial_Control, // 终端控制
	Reflush_config,
	LandToStart, // 降落到起点
} ;


const std::map<std::string, FlyState> FlyStateMap = {
	{"INIT", FlyState::init},
	// {"REQUEST", FlyState::request},
	{"TAKEOFF", FlyState::takeoff},
	{"END", FlyState::end},
	{"STOP", FlyState::stop},
	{"GOTO_SHOTPOINT", FlyState::Goto_shotpoint},
	{"DOSHOT", FlyState::Doshot},
	{"GOTO_SCOUTPOINT", FlyState::Goto_scoutpoint},
	{"SURROUND_SEE", FlyState::Surround_see},
	{"DOLAND", FlyState::Doland},
	{"MYPID", FlyState::MYPID},
	{"PRINT_INFO", FlyState::Print_Info},
	{"TERMINAL_CONTROL", FlyState::Termial_Control},
	{"REFLUSH_CONFIG", FlyState::Reflush_config},
	{"LAND_TO_START", FlyState::LandToStart},
};

// 将当前状态发布到currentstate 1=circle:shot/sco 2=h:land
inline int fly_state_to_int(FlyState state) {
  switch (state) {
    case FlyState::init: return 1;
    case FlyState::takeoff: return 1;
    case FlyState::end: return 2;
    case FlyState::Goto_shotpoint: return 1;
    case FlyState::Doshot: return 0;
    case FlyState::Goto_scoutpoint: return 1;
    case FlyState::Surround_see: return 3;
    case FlyState::Doland: return 4;
    case FlyState::Print_Info: return 1;
    default: return 1;
  }
}

class StateMachine {
public:
	explicit StateMachine(OffboardControl *owner) 
		: owner_(owner), current_state_(FlyState::init) {}

	template<FlyState... States>
	void process_states() {
		((handle_state<States>()), ...);
	}

	void add_dynamic_task(std::function<void()> task) {
		std::lock_guard<std::mutex> lock(task_mutex_);
		dynamic_tasks_.emplace_back(std::move(task));
	}

	void execute_dynamic_tasks() {
		std::vector<std::function<void()>> tasks;
		{
			std::lock_guard<std::mutex> lock(task_mutex_);
			tasks.swap(dynamic_tasks_);
		}
		for(const auto& task : tasks) {
			task();
		}
	}

	void transition_to(FlyState new_state);

	FlyState get_current_state() const { return current_state_; }
	FlyState get_previous_state() const { return previous_state_; }

private:
	OffboardControl *owner_;
	FlyState current_state_;
	FlyState previous_state_;
	std::vector<std::function<void()>> dynamic_tasks_;
	std::mutex task_mutex_;

	template<FlyState S>
	void handle_state();
};