#include "DoLandTask.h"
#include "../../drone/AbstractDrone.h"
#include "../../ROS2drone/ROS2Drone.h"
#include "../APMROS2Drone.h"
#include "../../module/YOLODetector.h"

// 定义静态成员
std::map<std::string, std::shared_ptr<DoLandTask>> DoLandTask::TASKS;

std::shared_ptr<DoLandTask> DoLandTask::createTask(const std::string& task_name) {
    if (TASKS.find(task_name) == TASKS.end()) {
        DoLandTask *new_task = new DoLandTask(task_name);
        TASKS[task_name] = std::shared_ptr<DoLandTask>(new_task);
        std::cout << "Creating " << TASKS[task_name]->get_name() << " instance." << std::endl;
    }
    TASKS[task_name]->next_task(TASKS[task_name]);
    return TASKS[task_name];
}

std::shared_ptr<DoLandTask> DoLandTask::getTask(const std::string& task_name) {
    auto it = TASKS.find(task_name);
    if (it != TASKS.end()) {
        return it->second;
    }
    std::cerr << "Warning: Task " << task_name << " not found." << std::endl;
    return nullptr;
}

template<typename DeviceType>
bool DoLandTask::init(DeviceType device) {
    // 检查类型，如果不是APMROS2Drone则返回false
    if constexpr (!std::is_same_v<DeviceType, std::shared_ptr<APMROS2Drone>>) {
        device->log_error("%s: init can only be used with APMROS2Drone", get_string().c_str());
        return false;
    } else {
        device->log_info(get_string().c_str());
        YAML::Node config = Readyaml::readYAML("land_config.yaml");
        scout_halt = config["scout_halt"].as<double>();
        scout_x = config["scout_x"].as<double>();
        scout_y = config["scout_y"].as<double>();
        target_z = config["tar_z"].as<double>();
        task = AppochTargetTask::createTask(get_name().append("_approach"));
        AppochTargetTask::Parameters task_params;
        task_params.config_file_name = parameters.config_file_name;
        task_params.config_device_name_prefix = parameters.config_device_name_suffix[0]; // Use first suffix as prefix
        task_params.config_device_name_suffix = parameters.config_device_name_suffix;
        task_params.target_type = YOLO_TARGET_TYPE::H; // YOLO::TARGET_TYPE::H
        task_params.device_index = parameters.device_index;
        task_params.fx = device->get_camera_gimbal()->fx;                                        // 相机焦距，像素单位
        task_params.dynamic_target_position_callback = parameters.dynamic_target_position_callback;         // 获取动态目标坐标的回调函数 x,y,z,r
        task_params.dynamic_target_image_callback = parameters.dynamic_target_image_callback;         // 获取动态图像目标坐标的回调函数 x,y,z,r
        task_params.target_height = parameters.target_height;                             // 目标的高度，默认为地面高度0.0m
        task_params.target_yaw = parameters.target_yaw;                                 // 目标偏航角
        task->setParameters(task_params);
        device->log_info("Doland");
        // rotate_global2stand(scout_x, scout_y, x_home, y_home);
        // device->log_info("返回降落准备点 x: %lf   y: %lf    angle: %lf", x_home, y_home, headingangle_compass);
        // rclcpp::sleep_for(std::chrono::seconds(6));
        device->rotate_global2stand(scout_x, scout_y, x_home, y_home);
        device->log_info("返回降落点 x: %lf   y: %lf    angle: %lf", x_home, y_home, 0.0);
        device->send_start_setpoint_command(x_home, y_home, scout_halt, 0);
        return true;
    }
}

template<typename DeviceType>
bool DoLandTask::run(DeviceType device) {
    // 检查类型，如果不是APMROS2Drone则返回false
    if constexpr (!std::is_same_v<DeviceType, std::shared_ptr<APMROS2Drone>>) {
        device->log_error("%s: init can only be used with APMROS2Drone", get_string().c_str());
        return false;
    } else {
        // device->log_info(get_string().c_str());
        // return true;
        if (timer_.elapsed() > 19 || surround_land > 3 || device->get_z_pos() < target_z + 0.1) // 降落时间超过39秒，或者降落高度小于目标高度
        {
            task_result = true;
            return true;
        }

        if (!device->get_yolo_detector()->is_get_target(YOLO_TARGET_TYPE::H)) // yolo未识别到YOLO::TARGET_TYPE::H   (YOLO::TARGET_TYPE::CIRCLE)
        {
            if (timer_.get_timepoint_elapsed() > 2.0)
            {
                    device->log_info("Doland: surround_land = %d", surround_land);
                    device->rotate_global2stand(scout_x + static_cast<double>(surround_land) * 1.0, scout_y, x_home, y_home);
                    device->log_info("Doland: land点 x: %lf   y: %lf   angle: %lf", x_home, y_home, 0.0); // 开始执行程序+x_home位置
                    device->send_start_setpoint_command(x_home, y_home, scout_halt, 0);
                    timer_.set_timepoint();
                    surround_land++;
            }
        }
        else
        {
            device->log_info_throttle(std::chrono::milliseconds(1000), "(T 1s) Doland: 看见H了，执行Doland");
            task->execute_apm(device);
            if (task->get_task_result())
            {
                device->log_info("Doland: 到达降落点");
                task_result = true;
                return true;  // 直接跳到下一个状态;
            }
            else
            {
                device->log_info_throttle(std::chrono::milliseconds(1000), "(T 1s) Doland: 未到达降落点");
                // device->log_info("Doland: 未到达降落点");
            }
        }
        return false;
    }
}

template<typename DeviceType>
bool DoLandTask::end(DeviceType device) {
    // 检查类型，如果不是APMROS2Drone则返回false
    if constexpr (!std::is_same_v<DeviceType, std::shared_ptr<APMROS2Drone>>) {
        device->log_error("%s: init can only be used with APMROS2Drone", get_string().c_str());
        return false;
    } else {
        device->log_info(get_string().c_str());
        device->send_velocity_command_with_time(0, 0, -0.2, 0, 1);
        device->log_info("Doland: 降落");
        surround_land = 0;
        timer_.set_timepoint();
        return true;
    }
}

// Explicit template instantiations
template bool DoLandTask::init<std::shared_ptr<AbstractDrone>>(std::shared_ptr<AbstractDrone> device);
template bool DoLandTask::run<std::shared_ptr<AbstractDrone>>(std::shared_ptr<AbstractDrone> device);
template bool DoLandTask::end<std::shared_ptr<AbstractDrone>>(std::shared_ptr<AbstractDrone> device);

template bool DoLandTask::init<std::shared_ptr<ROS2Drone>>(std::shared_ptr<ROS2Drone> device);
template bool DoLandTask::run<std::shared_ptr<ROS2Drone>>(std::shared_ptr<ROS2Drone> device);
template bool DoLandTask::end<std::shared_ptr<ROS2Drone>>(std::shared_ptr<ROS2Drone> device);

template bool DoLandTask::init<std::shared_ptr<APMROS2Drone>>(std::shared_ptr<APMROS2Drone> device);
template bool DoLandTask::run<std::shared_ptr<APMROS2Drone>>(std::shared_ptr<APMROS2Drone> device);
template bool DoLandTask::end<std::shared_ptr<APMROS2Drone>>(std::shared_ptr<APMROS2Drone> device);