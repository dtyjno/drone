#ifndef OFFBOARD_CONTROL__TASK__TASK_H
#define OFFBOARD_CONTROL__TASK__TASK_H

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <iostream>
#include <type_traits>
#include "TaskBase.h"
#include "../utils/Timer.h"

#include "../drone/AbstractDrone.h"
#include "../ROS2drone/ROS2Drone.h"
#include "../APMROS2drone/APMROS2Drone.h"
// Forward declarations to avoid circular dependencies
// class AbstractDrone;
// class ROS2Drone;
// class APMROS2Drone;

// Helper template for static_assert in if constexpr
template<typename> struct always_false : std::false_type {};

template<typename T>
class Task : public TaskBase {
public:
    // static std::shared_ptr<T> TASK;
    static std::map<std::string, std::shared_ptr<T>> TASKS;
    // 静态工厂方法
    static std::shared_ptr<T> createTask(const std::string& name);

    static std::shared_ptr<T> getTask(const std::string& task_name);

    std::string get_name() const override {
        return _name;
    }

    std::string get_string() const override {
        return "[" + std::to_string(index_created_) + ":" + std::to_string(index_alive_) + "] " + _name + ": " +
            get_task_state_string(task_state_);
    }

    int get_index_created() const {
        return index_created_;
    }

    int get_index_alive() const {
        return index_alive_;
    }

    Timer& get_timer() {
        return timer_;
    }

    template<typename DeviceType>
    bool execute(DeviceType device = nullptr) 
    {
        if (device == nullptr) {
            std::cerr << "Error: device is not set for task " << _name << std::endl;
            return false;
        }
        while(true){
            // if (!execute_finished_) {
            //     std::cout << "execute task: " << _name << ", sub_task: " << (_sub_task ? _sub_task->get_name() : "null") << ", state: " << get_task_state_string(task_state_) << std::endl;
            // }
            // std::cout << "_first_task " << (_first_task ? _first_task->get_name() : "null") 
            //               << ", current task: " << impl().getTask(_name)->get_name() << std::endl;
            // if (do_next_task) {
            //     device->log_info_throttle(std::chrono::milliseconds(100), "%s: (T 0.1s) Task: do_next_task=true, Moving to next task: %s", impl().getTask(_name)->get_string(), (_sub_task ? _sub_task->get_name() : "null"));
            // }
            if ((execute_finished_) && _sub_task != nullptr) {
                // do_next_task = false;
                if (_first_task == _sub_task) {
                    // std::cout << "All tasks completed." << std::endl;
                    // std::cout << (_first_task ? _first_task->get_string() : "null") << "->" <<  impl().getTask(_name)->get_string() << ": All tasks completed." << std::endl;
                    return true;
                }
                bool sub_task_result = false;
                if constexpr (std::is_same_v<DeviceType, std::shared_ptr<AbstractDrone>>) {
                    sub_task_result = _sub_task->execute_abstract(device);
                } else if constexpr (std::is_same_v<DeviceType, std::shared_ptr<ROS2Drone>>) {
                    // std::cout << "Executing sub_task: " << _sub_task->get_name() << std::endl;
                    sub_task_result = _sub_task->execute_ros2(device);
                } else if constexpr (std::is_same_v<DeviceType, std::shared_ptr<APMROS2Drone>>) {
                    // std::cout << "Executing sub_task: " << _sub_task->get_name() << std::endl;
                    sub_task_result = _sub_task->execute_apm(device);
                } else {
                    static_assert(always_false<DeviceType>::value, "Unsupported device type");
                }
                return sub_task_result;
            } else if (execute_finished_ && _sub_task == nullptr) {
                return task_result;
            }
            switch (task_state_)
            {
            case TaskState::init:
            {
                if (impl().init(device))
                {
                    task_result = false;
                    task_state_ = TaskState::running;
                    continue;
                }
                break;
            }
            case TaskState::running:
            {
                if (impl().run(device))
                {
                    task_state_ = TaskState::end;
                    continue;
                }
                break;
            }
            case TaskState::end:
            {
                if (impl().end(device))
                {
                    task_state_ = TaskState::init;
                    execute_finished_ = true;
                    // 下一个任务在下一次执行该任务后执行
                    // continue;
                }
                break;
            }
            default:
                break;
            }
            break;
        }
        return task_result;
    }

    enum class TaskState {
        init,
        running,
        end
    };
    
    TaskState task_state_ = TaskState::init;

    static std::string get_task_state_string(TaskState state) {
        switch (state) {
            case TaskState::init: return "init";
            case TaskState::running: return "running";
            case TaskState::end: return "end";
            default: return "unknown";
        }
    }

    Task(const std::string& name)
        : _name(name) {
        instance_alive_++;
        instance_created_++;
        index_created_ = instance_created_;
        index_alive_ = instance_alive_;
    }

    ~Task() {
        instance_alive_--;
    }

    // bool do_next_task = false;

    void reset() override {
        task_state_ = TaskState::init;
        execute_finished_ = false;
        task_result = false;
        // do_next_task = false;
        timer_.reset();
        next_task(impl().getTask(_name));
    }

    bool execute_abstract(std::shared_ptr<AbstractDrone> device) override {
        return execute<std::shared_ptr<AbstractDrone>>(device);
    }
    
    bool execute_ros2(std::shared_ptr<ROS2Drone> device) override {
        return execute<std::shared_ptr<ROS2Drone>>(device);
    }
    
    bool execute_apm(std::shared_ptr<APMROS2Drone> device) override {
        return execute<std::shared_ptr<APMROS2Drone>>(device);
    }
protected:
    static inline int instance_created_ = 0;
    static inline int instance_alive_ = 0;
    int index_created_;
    int index_alive_;

    Timer timer_; // 任务计时器

private:
    std::string _name;
    T& impl() { return static_cast<T&>(*this); }
};

void reset_all_tasks();

#endif // OFFBOARD_CONTROL__TASK__TASK_H