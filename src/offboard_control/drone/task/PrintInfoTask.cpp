#include "PrintInfoTask.h"
#include "../AbstractDrone.h"
#include "../../ROS2drone/ROS2Drone.h"
#include "../../APMROS2drone/APMROS2Drone.h"

// 定义静态成员
std::map<std::string, std::shared_ptr<PrintInfoTask>> PrintInfoTask::TASKS;

std::shared_ptr<PrintInfoTask> PrintInfoTask::createTask(const std::string& task_name) {
    if (TASKS.find(task_name) == TASKS.end()) {
        PrintInfoTask *new_task = new PrintInfoTask(task_name);
        TASKS[task_name] = std::shared_ptr<PrintInfoTask>(new_task);
        std::cout << "Creating " << TASKS[task_name]->get_name() << " instance." << std::endl;
    }
    // std::cout << "Creating next task link for " << TASKS[task_name]->get_name() << std::endl;
    TASKS[task_name]->next_task(TASKS[task_name]);
    return TASKS[task_name];
}

std::shared_ptr<PrintInfoTask> PrintInfoTask::getTask(const std::string& task_name) {
    auto it = TASKS.find(task_name);
    if (it != TASKS.end()) {
        return it->second;
    }
    std::cerr << "Warning: Task " << task_name << " not found." << std::endl;
    return nullptr;
}

template<typename DeviceType>
bool PrintInfoTask::init(DeviceType device) {
    if constexpr (!std::is_base_of_v<AbstractDrone, typename DeviceType::element_type>) {
        static_assert(std::is_base_of_v<AbstractDrone, typename DeviceType::element_type>, 
                     "PrintInfoTask can only be used with devices derived from AbstractDrone");
        return false;
    } else {
        if (device) {
            device->log_info(get_string().c_str());
        } else {
            std::cout << "No device associated with this task." << std::endl;
            return false;
        }
    }
    execute_count_ = 0;

    return true;
}

template<typename DeviceType>
bool PrintInfoTask::run(DeviceType device) {
    // RCLCPP_INFO(device->get_node()->get_logger(), get_string().c_str());
    if constexpr (!std::is_base_of_v<AbstractDrone, typename DeviceType::element_type>) {
        static_assert(std::is_base_of_v<AbstractDrone, typename DeviceType::element_type>, 
                     "PrintInfoTask can only be used with devices derived from AbstractDrone");
        return false;
    } else {
        if (device) {
            try {
                if (execute_count_++ % 10 == 0) {
                    std::stringstream ss;
                    // 打印当前状态信息
                    ss << "--------timer_callback----------" << std::endl;
                    ss << "px:  " << std::setw(10) << device->get_x_pos() << ", py: " << std::setw(10) << device->get_y_pos() << ", pz: " << std::setw(10) << device->get_z_pos() << std::endl;
                    ss << "vx:  " << std::setw(10) << device->get_x_vel() << ", vy: " << std::setw(10) << device->get_y_vel() << ", vz: " << std::setw(10) << device->get_z_vel() << std::endl;
                    // ss << "yaw: " << device->get_yaw() << std::endl;
                    // ss << "yaw_e: " << device->get_yaw_eigen() << std::endl;
                    ss << "yaw_vel: " << device->get_yaw_vel() << std::endl;
                    float roll, pitch, yaw;
                    device->get_euler(roll, pitch, yaw);
                    ss << "roll: " << roll << ", pitch: " << pitch << ", yaw: " << yaw << std::endl;
                    ss << "lat: " << device->get_lat() << ", lon: " << device->get_lon() << ", alt: " << device->get_alt() << std::endl;
                    ss << "rangefinder_distance:  " << device->get_rangefinder_distance() << std::endl;
                    ss << "armed:     " << device->get_armed() << std::endl;
                    ss << "connected: " << device->get_connected() << std::endl;
                    ss << "guided:	" << device->get_guided() << std::endl;
                    ss << "mode:	  " << device->get_mode() << std::endl;
                    ss << "system_status:  " << device->get_system_status() << std::endl;
                    ss << "x_home_pos:     " << device->get_x_home_pos() << ", y_home_pos: " << device->get_y_home_pos() << ", z_home_pos: " << device->get_z_home_pos() << std::endl;
                    ss << "running_time: " << device->get_cur_time() << std::endl;
                    // RCLCPP_INFO_STREAM(device->get_node()->get_logger(), ss.str());
                    device->log_info(ss.str().c_str());
                    ss.str(""); // 清空字符串流
                    ss.clear(); // 清除状态标志
                    // std::cout << "device Position: x=" << device->get_x_pos()
                    //           << ", y=" << device->get_y_pos()
                    //           << ", z=" << device->get_z_pos()
                    //           << ", yaw=" << device->get_yaw() << std::endl;
                }
            } catch (const std::exception& e) {
                std::cout << "Error accessing device position: " << e.what() << std::endl;
            } catch (...) {
                std::cout << "Unknown error accessing device position" << std::endl;
            }
        } else {
            std::cout << "No device associated with this task." << std::endl;
        }
        // return true; // 任务完成，进入end状态
        return execute_count_ * device->get_wait_time() >= print_time_; // 打印50次后完成任务
    }
}

template<typename DeviceType>
bool PrintInfoTask::end(DeviceType device) {
    if constexpr (!std::is_base_of_v<AbstractDrone, typename DeviceType::element_type>) {
        static_assert(std::is_base_of_v<AbstractDrone, typename DeviceType::element_type>, 
                     "PrintInfoTask can only be used with devices derived from AbstractDrone");
        return false;
    } else {
        device->log_info(get_string().c_str());
    }
    return true;
}

// Explicit template instantiations
template bool PrintInfoTask::init<std::shared_ptr<AbstractDrone>>(std::shared_ptr<AbstractDrone> device);
template bool PrintInfoTask::run<std::shared_ptr<AbstractDrone>>(std::shared_ptr<AbstractDrone> device);
template bool PrintInfoTask::end<std::shared_ptr<AbstractDrone>>(std::shared_ptr<AbstractDrone> device);
