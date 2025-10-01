#include "WaitTask.h"
#include "../drone/AbstractDrone.h"
#include "../ROS2drone/ROS2Drone.h"
#include "../APMROS2drone/APMROS2Drone.h"

// 定义静态成员
std::map<std::string, std::shared_ptr<WaitTask>> WaitTask::TASKS;

std::shared_ptr<WaitTask> WaitTask::createTask(const std::string& task_name) {
    if (TASKS.find(task_name) == TASKS.end()) {
        WaitTask *new_task = new WaitTask(task_name);
        TASKS[task_name] = std::shared_ptr<WaitTask>(new_task);
        std::cout << "Creating " << TASKS[task_name]->get_name() << " instance." << std::endl;
    }
    TASKS[task_name]->next_task(TASKS[task_name]);
    return TASKS[task_name];
}

std::shared_ptr<WaitTask> WaitTask::getTask(const std::string& task_name) {
    auto it = TASKS.find(task_name);
    if (it != TASKS.end()) {
        return it->second;
    }
    std::cerr << "Warning: Task " << task_name << " not found." << std::endl;
    return nullptr;
}

template<typename DeviceType>
bool WaitTask::init(DeviceType device) {
    device->log_info(get_string());
    timer_.reset();
    return true;
}

template<typename DeviceType>
bool WaitTask::run(DeviceType device) {
    // device->log_info(get_string());
    device->log_info_throttle(std::chrono::milliseconds(1000), "等待中...已等待%.2f秒，目标等待%.2f秒", timer_.elapsed(), wait_time);
    if (timer_.elapsed() >= wait_time) {
        task_result = true;
        return true;
    } else {
        return !is_blocked_;
    }
}

template<typename DeviceType>
bool WaitTask::end(DeviceType device) {
    device->log_info(get_string());
    return true;
}

// Explicit template instantiations
template bool WaitTask::init<std::shared_ptr<AbstractDrone>>(std::shared_ptr<AbstractDrone> device);
template bool WaitTask::run<std::shared_ptr<AbstractDrone>>(std::shared_ptr<AbstractDrone> device);
template bool WaitTask::end<std::shared_ptr<AbstractDrone>>(std::shared_ptr<AbstractDrone> device);

template bool WaitTask::init<std::shared_ptr<ROS2Drone>>(std::shared_ptr<ROS2Drone> device);
template bool WaitTask::run<std::shared_ptr<ROS2Drone>>(std::shared_ptr<ROS2Drone> device);
template bool WaitTask::end<std::shared_ptr<ROS2Drone>>(std::shared_ptr<ROS2Drone> device);

template bool WaitTask::init<std::shared_ptr<APMROS2Drone>>(std::shared_ptr<APMROS2Drone> device);
template bool WaitTask::run<std::shared_ptr<APMROS2Drone>>(std::shared_ptr<APMROS2Drone> device);
template bool WaitTask::end<std::shared_ptr<APMROS2Drone>>(std::shared_ptr<APMROS2Drone> device);
