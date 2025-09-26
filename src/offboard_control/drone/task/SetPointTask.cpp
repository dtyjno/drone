#include "SetPointTask.h"
#include "../AbstractDrone.h"
#include "../../ROS2drone/ROS2Drone.h"
#include "../../APMROS2drone/APMROS2Drone.h"

// 定义静态成员
std::map<std::string, std::shared_ptr<SetPointTask>> SetPointTask::TASKS;

std::shared_ptr<SetPointTask> SetPointTask::createTask(const std::string& task_name) {
    if (TASKS.find(task_name) == TASKS.end()) {
        SetPointTask *new_task = new SetPointTask(task_name);
        TASKS[task_name] = std::shared_ptr<SetPointTask>(new_task);
        std::cout << "Creating " << TASKS[task_name]->get_name() << " instance." << std::endl;
    }
    TASKS[task_name]->next_task(TASKS[task_name]);
    return TASKS[task_name];
}

std::shared_ptr<SetPointTask> SetPointTask::getTask(const std::string& task_name) {
    auto it = TASKS.find(task_name);
    if (it != TASKS.end()) {
        return it->second;
    }
    std::cerr << "Warning: Task " << task_name << " not found, creating a new one." << std::endl;
    return createTask(task_name);
}

template<typename DeviceType>
bool SetPointTask::init(DeviceType device) {
    if constexpr (!std::is_base_of_v<AbstractDrone, typename DeviceType::element_type>) {
        static_assert(std::is_base_of_v<AbstractDrone, typename DeviceType::element_type>, 
                     "SetPointTask can only be used with devices derived from AbstractDrone");
        return false;
    } else {
        device->log_info(get_string().c_str());
        // 重置计数器和计时器
        set_counter(0);
        reset_timer();
    }
    return true;
}

template<typename DeviceType>
bool SetPointTask::run(DeviceType device) {

    if constexpr (!std::is_base_of_v<AbstractDrone, typename DeviceType::element_type>) {
        static_assert(std::is_base_of_v<AbstractDrone, typename DeviceType::element_type>, 
                     "SetPointTask can only be used with devices derived from AbstractDrone");
        return false;
    } else {
        // RCLCPP_INFO_THROTTLE(device->node->get_logger(), (*device->node)->get_clock(), 1000, "(T 1s) w_g_n,counter: %d, time=%lf", parameters_.point_count, timer_.elapsed());
        device->log_info_throttle(std::chrono::milliseconds(500), "%s: time=%s", get_string().c_str(), timer_.elapsed() == std::numeric_limits<double>::max() ? "max" : std::to_string(timer_.elapsed()).c_str());
        // 判断是否到达目标点点或超时
        if (timer_.elapsed() > parameters_.point_time ||
                (is_equal(device->get_x_pos(), parameters_.target_x, parameters_.accuracy) && 
                is_equal(device->get_y_pos(), parameters_.target_y, parameters_.accuracy))
            ) 
        {
            if (get_counter() == 0)
            {
                // 更新目标点坐标
                float x_temp = parameters_.target_x;
                float y_temp = parameters_.target_y;
                float z_temp = parameters_.target_z;
                float yaw_temp = parameters_.target_yaw;
                device->log_info("%s: 点位%d x: %lf y: %lf z: %lf timeout=%s",
                    get_string().c_str(),
                    get_counter(),
                    x_temp,
                    y_temp,
                    z_temp,
                    (is_equal(device->get_x_pos(), x_temp, parameters_.accuracy) && is_equal(device->get_y_pos(), y_temp, parameters_.accuracy))? "true" : "false"
                );
                if (parameters_.frame == Parameters::Frame::START) {
                    device->send_start_setpoint_command(x_temp, y_temp, z_temp, yaw_temp); // 发送本地坐标系下的航点指令
                } else if (parameters_.frame == Parameters::Frame::LOCAL) {
                    device->send_local_setpoint_command(x_temp, y_temp, z_temp, yaw_temp); // 发送局部坐标系下的航点指令
                } else {
                    device->log_error("%s: 未知的坐标系参数", get_string().c_str());
                }
                // RCLCPP_INFO(device->node->get_logger(), "前往下一点");
                set_counter(get_counter() + 1);  // 航点自增
                timer_.reset();
            } else {
                return true;  // 任务完成
            }
        }
    }
	return false;
}

template<typename DeviceType>
bool SetPointTask::end(DeviceType device) {
    if constexpr (!std::is_base_of_v<AbstractDrone, typename DeviceType::element_type>) {
        static_assert(std::is_base_of_v<AbstractDrone, typename DeviceType::element_type>, 
                     "SetPointTask can only be used with devices derived from AbstractDrone");
        return false;
    } else {
        set_counter(0);
        reset_timer();
        device->log_info(get_string().c_str());
    }
    return true;
}

// Explicit template instantiations
template bool SetPointTask::init<std::shared_ptr<AbstractDrone>>(std::shared_ptr<AbstractDrone> device);
template bool SetPointTask::run<std::shared_ptr<AbstractDrone>>(std::shared_ptr<AbstractDrone> device);
template bool SetPointTask::end<std::shared_ptr<AbstractDrone>>(std::shared_ptr<AbstractDrone> device);
