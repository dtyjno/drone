#include "WayPointTask.h"
#include "../drone/AbstractDrone.h"

// 定义静态成员
std::map<std::string, std::shared_ptr<WayPointTask>> WayPointTask::TASKS;

std::shared_ptr<WayPointTask> WayPointTask::createTask(const std::string& task_name) {
    if (TASKS.find(task_name) == TASKS.end()) {
        WayPointTask *new_task = new WayPointTask(task_name);
        TASKS[task_name] = std::shared_ptr<WayPointTask>(new_task);
        std::cout << "Creating " << TASKS[task_name]->get_name() << " instance." << std::endl;
        TASKS[task_name]->next_task(TASKS[task_name]);
    }
    return TASKS[task_name];
}

std::shared_ptr<WayPointTask> WayPointTask::getTask(const std::string& task_name) {
    auto it = TASKS.find(task_name);
    if (it != TASKS.end()) {
        return it->second;
    }
    std::cerr << "Warning: Task " << task_name << " not found." << std::endl;
    return nullptr;
}

template<typename DeviceType>
bool WayPointTask::init(DeviceType device) {
    if constexpr (!std::is_base_of_v<AbstractDrone, typename DeviceType::element_type>) {
        static_assert(std::is_base_of_v<AbstractDrone, typename DeviceType::element_type>, 
                     "WayPointTask can only be used with devices derived from AbstractDrone");
        return false;
    } else {
        device->log_info(get_string());
        // 重置计数器和计时器
        set_counter(0);
        reset();
    }
    return true;
}

template<typename DeviceType>
bool WayPointTask::run(DeviceType device) {
    if constexpr (!std::is_base_of_v<AbstractDrone, typename DeviceType::element_type>) {
        static_assert(std::is_base_of_v<AbstractDrone, typename DeviceType::element_type>, 
                     "WayPointTask can only be used with devices derived from AbstractDrone");
        return false;
    } else {
        // RCLCPP_INFO_THROTTLE(device->node->get_logger(), (*device->node)->get_clock(), 1000, "(T 1s) w_g_n,counter: %d, time=%lf", parameters_.point_count, timer_.elapsed());
        device->log_info_throttle(std::chrono::milliseconds(500), "w_g_n,counter: ", get_counter(),", time=", timer_.elapsed() == std::numeric_limits<double>::max() ? "max" : std::to_string(timer_.elapsed()));
        // 更新目标点坐标
        float x_temp = parameters_.center_x + way_point_[get_counter()].x();
        float y_temp = parameters_.center_y + way_point_[get_counter()].y();
        float z_temp = way_point_[get_counter()].z();
        float yaw_temp = way_point_[get_counter()].w();
        // 判断是否到达当前航点或超时
        if (point_count == 0 ||     // 发布首个航点
            timer_.elapsed() > parameters_.point_time ||
            (parameters_.frame == Parameters::Frame::START && device->is_equal_start_target_xy(x_temp, y_temp, parameters_.accuracy)) ||
            (parameters_.frame == Parameters::Frame::LOCAL && device->is_equal_local_target_xy(x_temp, y_temp, parameters_.accuracy))
        )
        {
            if (static_cast<size_t>(get_counter()) >= way_point_.size())
            {
                return true;  // 任务完成
            } else {
                device->log_info(get_string(), ", ", get_string(), "点位", way_point_.get_name(),
                    " x: ", x_temp,
                    " y: ", y_temp,
                    " z: ", z_temp,
                    " yaw: ", yaw_temp,
                    " timeout: ", timer_.elapsed() > parameters_.point_time ? "true" : "false"
                );
                if (parameters_.frame == Parameters::Frame::START) {
                    device->send_start_setpoint_command(x_temp, y_temp, z_temp, yaw_temp); // 发送本地坐标系下的航点指令
                } else if (parameters_.frame == Parameters::Frame::LOCAL) {
                    device->send_local_setpoint_command(x_temp, y_temp, z_temp, yaw_temp); // 发送局部坐标系下的航点指令
                } else {
                    device->log_error("%s: 未知的坐标系参数", get_string());
                }
                // RCLCPP_INFO(device->node->get_logger(), "前往下一点");
                set_counter(get_counter() + 1);  // 航点自增
                timer_.reset();
            }
        }
    }
	return false;
}

template<typename DeviceType>
bool WayPointTask::end(DeviceType device) {
    if constexpr (!std::is_base_of_v<AbstractDrone, typename DeviceType::element_type>) {
        static_assert(std::is_base_of_v<AbstractDrone, typename DeviceType::element_type>, 
                     "WayPointTask can only be used with devices derived from AbstractDrone");
        return false;
    } else {
        device->log_info(get_string());
        device->log_info("w_g_n, %s已经全部遍历", way_point_.get_name());
        set_counter(0);
        task_result = true;
    }
    return true;
}

// Explicit template instantiations
template bool WayPointTask::init<std::shared_ptr<AbstractDrone>>(std::shared_ptr<AbstractDrone> device);
template bool WayPointTask::run<std::shared_ptr<AbstractDrone>>(std::shared_ptr<AbstractDrone> device);
template bool WayPointTask::end<std::shared_ptr<AbstractDrone>>(std::shared_ptr<AbstractDrone> device);