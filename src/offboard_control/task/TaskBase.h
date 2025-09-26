#ifndef OFFBOARD_CONTROL__TASK__TASK_BASE_H
#define OFFBOARD_CONTROL__TASK__TASK_BASE_H

#include <memory>
#include <string>
#include <iostream>
#include <cassert>

#include "DeviceVisitor.h"

// 前向声明
class AbstractDrone;
class ROS2Drone;
class APMROS2Drone;
class DeviceVisitor;

// 基类任务接口
class TaskBase : public DeviceVisitor, public std::enable_shared_from_this<TaskBase> {
public:
    virtual ~TaskBase() = default;
    
    virtual std::string get_name() const = 0;
    virtual std::string get_string() const = 0;
    virtual void reset() = 0;
    
    // 任务链接方法
    void set_next_task(std::shared_ptr<TaskBase> task) {
        _sub_task = task;
    }

    void set_first_task(std::shared_ptr<TaskBase> first_task) {
        _first_task = first_task;
    }
    
    bool is_execute_finished() const {
        return execute_finished_;
    }

    bool get_task_result() const {
        return task_result;
    }

    std::shared_ptr<TaskBase> next_task(std::shared_ptr<TaskBase> next_task) {
        // std::cout << "Setting next task for " << get_name() 
        //           << " to " << (next_task ? next_task->get_name() : "null") << std::endl;
        if (next_task) {
            next_task->set_next_task(_sub_task);
            next_task->set_first_task(_first_task ? _first_task : shared_from_this());
            _sub_task = next_task;
            // next_task->reset();
            return next_task;
        } else {
            return shared_from_this();
        }
    }
    
    std::shared_ptr<TaskBase> final_task() {
        if (_sub_task == nullptr) {
            return shared_from_this();
        }
        return _sub_task;
    }
    
    std::shared_ptr<DeviceVisitor> visit(std::shared_ptr<AbstractDrone> drone) override {
        this->execute_abstract(drone);
        return _sub_task;
    }

    std::shared_ptr<DeviceVisitor> visit(std::shared_ptr<ROS2Drone> drone) override {
        this->execute_ros2(drone);
        return _sub_task;
    }

    std::shared_ptr<DeviceVisitor> visit(std::shared_ptr<APMROS2Drone> drone) override {
        this->execute_apm(drone);
        return _sub_task;
    }
    
    virtual bool execute_abstract(std::shared_ptr<AbstractDrone> device) = 0;
    virtual bool execute_ros2(std::shared_ptr<ROS2Drone> device) = 0;
    virtual bool execute_apm(std::shared_ptr<APMROS2Drone> device) = 0;

protected:
    
    bool task_result = false;
    bool execute_finished_ = false;
    std::shared_ptr<TaskBase> _sub_task = nullptr;
    std::shared_ptr<TaskBase> _first_task = nullptr;
};

#endif // OFFBOARD_CONTROL__TASK__TASK_BASE_H
