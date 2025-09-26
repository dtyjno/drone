#include "BlankTask.h"
#include "../drone/AbstractDrone.h"
#include "../ROS2drone/ROS2Drone.h"
#include "../APMROS2drone/APMROS2Drone.h"

// 定义静态成员
std::map<std::string, std::shared_ptr<BlankTask>> BlankTask::TASKS;

std::shared_ptr<BlankTask> BlankTask::createTask(const std::string& task_name) {
    if (TASKS.find(task_name) == TASKS.end()) {
        BlankTask *new_task = new BlankTask(task_name);
        TASKS[task_name] = std::shared_ptr<BlankTask>(new_task);
        std::cout << "Creating " << TASKS[task_name]->get_name() << " instance." << std::endl;
    }
    TASKS[task_name]->next_task(TASKS[task_name]);
    return TASKS[task_name];
}

std::shared_ptr<BlankTask> BlankTask::getTask(const std::string& task_name) {
    auto it = TASKS.find(task_name);
    if (it != TASKS.end()) {
        return it->second;
    }
    std::cerr << "Warning: Task " << task_name << " not found." << std::endl;
    return nullptr;
}

template<typename DeviceType>
bool BlankTask::init(DeviceType device) {
    device->log_info(get_string().c_str());
    return true;
}

template<typename DeviceType>
bool BlankTask::run(DeviceType device) {
    device->log_info(get_string().c_str());
    return true;
}

template<typename DeviceType>
bool BlankTask::end(DeviceType device) {
    device->log_info(get_string().c_str());
    return true;
}

// Explicit template instantiations
template bool BlankTask::init<std::shared_ptr<AbstractDrone>>(std::shared_ptr<AbstractDrone> device);
template bool BlankTask::run<std::shared_ptr<AbstractDrone>>(std::shared_ptr<AbstractDrone> device);
template bool BlankTask::end<std::shared_ptr<AbstractDrone>>(std::shared_ptr<AbstractDrone> device);

template bool BlankTask::init<std::shared_ptr<ROS2Drone>>(std::shared_ptr<ROS2Drone> device);
template bool BlankTask::run<std::shared_ptr<ROS2Drone>>(std::shared_ptr<ROS2Drone> device);
template bool BlankTask::end<std::shared_ptr<ROS2Drone>>(std::shared_ptr<ROS2Drone> device);