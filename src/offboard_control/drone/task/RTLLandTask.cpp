#include "RTLLandTask.h"
#include "../AbstractDrone.h"
#include "../../ROS2drone/ROS2Drone.h"
#include "../../APMROS2drone/APMROS2Drone.h"
// #include "../../stateMachine/StateMachine.h"

// 定义静态成员
std::map<std::string, std::shared_ptr<RTLLandTask>> RTLLandTask::TASKS;

std::shared_ptr<RTLLandTask> RTLLandTask::createTask(const std::string& task_name) {
    if (TASKS.find(task_name) == TASKS.end()) {
        RTLLandTask *new_task = new RTLLandTask(task_name);
        TASKS[task_name] = std::shared_ptr<RTLLandTask>(new_task);
        std::cout << "Creating " << TASKS[task_name]->get_name() << " instance." << std::endl;
    }
    TASKS[task_name]->next_task(TASKS[task_name]);
    return TASKS[task_name];
}

std::shared_ptr<RTLLandTask> RTLLandTask::getTask(const std::string& task_name) {
    auto it = TASKS.find(task_name);
    if (it != TASKS.end()) {
        return it->second;
    }
    std::cerr << "Warning: Task " << task_name << " not found." << std::endl;
    return nullptr;
}

template<typename DeviceType>
bool RTLLandTask::init(DeviceType device) {
    if constexpr (!std::is_base_of_v<AbstractDrone, typename DeviceType::element_type>) {
        static_assert(std::is_base_of_v<AbstractDrone, typename DeviceType::element_type>, 
                     "RTLLandTask can only be used with devices derived from AbstractDrone");
        return false;
    } else {
        device->log_info(get_string().c_str());
        device->log_info("开始降落");
        device->get_status_controller()->switch_mode("RTL");
        return true;
    }
}

template<typename DeviceType>
bool RTLLandTask::run(DeviceType device) {
    if constexpr (!std::is_base_of_v<AbstractDrone, typename DeviceType::element_type>) {
        static_assert(std::is_base_of_v<AbstractDrone, typename DeviceType::element_type>, 
                     "RTLLandTask can only be used with devices derived from AbstractDrone");
        return false;
    } else {
        if (timer_.elapsed() < 18) { // 如果等待超过18秒
        } else if (timer_.elapsed() < 18 + device->get_wait_time()) {
            device->log_info("等待降落超过18秒，开始降落");
            device->get_status_controller()->switch_mode("GUIDED");
            land_task->reset();    // 重置计时器
        } else {
            // device->log_info(get_string().c_str());
            land_task->visit(device);
            if (device->get_mode() == "GUIDED" && land_task->get_task_result()) {
                return true;
            } else {
                device->log_info_throttle(std::chrono::milliseconds(1000), "等待降落中...%f", timer_.elapsed());
            }
        }
    }
    // device->log_info(get_string().c_str());
    return false;
}

template<typename DeviceType>
bool RTLLandTask::end(DeviceType device) {
    if constexpr (!std::is_base_of_v<AbstractDrone, typename DeviceType::element_type>) {
        static_assert(std::is_base_of_v<AbstractDrone, typename DeviceType::element_type>, 
                     "RTLLandTask can only be used with devices derived from AbstractDrone");
        return false;
    } else {
        // device->log_info(get_string().c_str());
        device->log_info("%s: 降落完成", get_string().c_str());
        device->get_status_controller()->switch_mode("LAND");
        // Note: State machine access removed - should be handled by device itself
    }
    return true;
}

// Explicit template instantiations
template bool RTLLandTask::init<std::shared_ptr<AbstractDrone>>(std::shared_ptr<AbstractDrone> device);
template bool RTLLandTask::run<std::shared_ptr<AbstractDrone>>(std::shared_ptr<AbstractDrone> device);
template bool RTLLandTask::end<std::shared_ptr<AbstractDrone>>(std::shared_ptr<AbstractDrone> device);
