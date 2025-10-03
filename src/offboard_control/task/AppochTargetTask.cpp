#include "AppochTargetTask.h"

// 定义静态成员
std::map<std::string, std::shared_ptr<AppochTargetTask>> AppochTargetTask::TASKS;

std::shared_ptr<AppochTargetTask> AppochTargetTask::createTask(const std::string& task_name) {
    if (TASKS.find(task_name) == TASKS.end()) {
        TASKS[task_name] = std::shared_ptr<AppochTargetTask>(new AppochTargetTask(task_name));
        std::cout << "Creating " << TASKS[task_name]->get_name() << " instance." << std::endl;
    }
    TASKS[task_name]->next_task(TASKS[task_name]);
    return TASKS[task_name];
}

std::shared_ptr<AppochTargetTask> AppochTargetTask::getTask(const std::string& task_name) {
    auto it = TASKS.find(task_name);
    if (it != TASKS.end()) {
        return it->second;
    }
    std::cerr << "Warning: Task " << task_name << " not found." << std::endl;
    return nullptr;
}

template<typename DeviceType>
bool AppochTargetTask::init(DeviceType device) {
    device->log_info(get_string());
    // if constexpr (!std::is_same_v<DeviceType, std::shared_ptr<APMROS2Drone>>) {
    if (device->get_camera() == nullptr || device->get_servo_controller() == nullptr || device->get_yolo_detector() == nullptr) {
        // device->log_error("%s: AppochTargetTask::init can only be used with APMROS2Drone", get_string());
        device->log_error(get_string(), ": init can only be used with APMROS2Drone");
        return false;
    } else {
        device->get_position_controller()->reset_pid();
        pid_defaults = PID::readPIDParameters(parameters.config_file_name, "pid");
        PosController::Limits_t limits = device->get_position_controller()->readLimits(parameters.config_file_name, "limits");
        device->get_position_controller()->set_limits(limits);
        // 读取距离目标一定范围内退出的距离
        YAML::Node config = Readyaml::readYAML(parameters.config_file_name);
        radius = config["radius"].as<float>();
        accuracy = config["accuracy"].as<float>();
        // tar_z = config["tar_z"].as<float>();
        this->device_position.clear();
        this->image_targets.clear();
        // 读取目标点相对于无人机机体坐标系的坐标
        for (size_t i = 0; i < parameters.config_device_name_suffix.size(); i++)
        {
            std::string key_x = parameters.config_device_name_prefix + "_x" + parameters.config_device_name_suffix[i];
            std::string key_y = parameters.config_device_name_prefix + "_y" + parameters.config_device_name_suffix[i];
            std::string key_z = parameters.config_device_name_prefix + "_z" + parameters.config_device_name_suffix[i];
            std::cout << "Reading position for target suffix: " << key_x << std::endl;
            float adjusted_x = config[key_x].as<float>();
            float adjusted_y = config[key_y].as<float>();
            float adjusted_z = config[key_z].as<float>();
            this->device_position.push_back(Vector3f(adjusted_x, adjusted_y, adjusted_z)); // 调整高度 待旋转
            device->log_info(get_string(), ": ", parameters.config_device_name_prefix, "_x", parameters.config_device_name_suffix[i], ": " , this->device_position[i].x());
            device->log_info(get_string(), ": ", parameters.config_device_name_prefix, "_y", parameters.config_device_name_suffix[i], ": ", this->device_position[i].y());
            device->log_info(get_string(), ": ", parameters.config_device_name_prefix, "_z", parameters.config_device_name_suffix[i], ": ", this->device_position[i].z());
        }
        // 初始化图像上的目标像素目标，解析目标位置在图像上的映射失败时，使用图像中心点
        for (size_t i = 0; i < parameters.config_device_name_suffix.size(); i++)
        {
            TargetData target;
            target.category = YOLODetector::enumToString(parameters.target_type).append(parameters.config_device_name_suffix[i]);
            std::string key_x = std::string("tar_x").append(parameters.config_device_name_suffix[i]);
            std::string key_y = std::string("tar_y").append(parameters.config_device_name_suffix[i]);
            std::string key_z = std::string("tar_z").append(parameters.config_device_name_suffix[i]);
            target.x = config[key_x].as<float>();
            target.y = config[key_y].as<float>();
            target.z = config[key_z].as<float>();
            // target.x = (is_equal(target.x, 0.0f) ? device->get_yolo_detector()->get_cap_frame_width() / 2 : target.x);
            // target.y = (is_equal(target.y, 0.0f) ? device->get_yolo_detector()->get_cap_frame_height() / 2 : target.y);
            // // target.z = (is_equal(target.z, 0.0f) ? tar_z : target.z);
            // target.x = device->get_yolo_detector()->get_cap_frame_width() / 2;
            // target.y = device->get_yolo_detector()->get_cap_frame_height() / 2;
            target.r = 1.0f; 
            target.g = 0.0f;
            target.b = 0.0f;
            target.fx = parameters.fx;
            target.radius = radius;
            device->log_info(get_string(), ": tar_x: ", target.x, ", tar_y: ", target.y,", tar_z: ", target.z);
            this->image_targets.push_back(target);
        }
        device->log_info(get_string(), ": cap_frame_width: ", device->get_yolo_detector()->get_cap_frame_width(), ", cap_frame_height: ", device->get_yolo_detector()->get_cap_frame_height(), ", radius: ", radius, ", accuracy: ", accuracy);

        return true;
    }
}

template<typename DeviceType>
bool AppochTargetTask::run(DeviceType device) {
    // device->log_info(get_string());
    // return true;
    // if constexpr (!std::is_same_v<DeviceType, std::shared_ptr<APMROS2Drone>>) {
    if (device->get_camera() == nullptr || device->get_servo_controller() == nullptr || device->get_yolo_detector() == nullptr) {
        device->log_error(get_string(), ": init can only be used with APMROS2Drone");
        return false;
    } else {
        // TARGET 世界坐标系
        // if (getCurrentPositionTargets() != pre_position_targets) {
        //     pre_position_targets = getCurrentPositionTargets();
        // }
        if (getCurrentPositionTargets().index != pre_position_target_index) {
            max_target_position_accurate = std::max(0.1f, getCurrentPositionTargets().radius);
            pre_position_target_index = getCurrentPositionTargets().index;
        }
        if (!getCurrentPositionTargets().position.isZero() &&
                (parameters.task_type == Type::TARGET ||
                  (parameters.task_type == Type::AUTO && (
                    target_timer.elapsed() < 6 ||      // 至少稳定6秒
                      (target_timer.elapsed() < 12 && (
                        abs(device->get_x_pos() - (getCurrentPositionTargets().position.x())) > max_target_position_accurate &&
                        abs(device->get_y_pos() - (getCurrentPositionTargets().position.y())) > max_target_position_accurate
                        )
                      )
                    )
                  )
                )
            ) {
            current_type = Type::TARGET;
            // 显示目标位置在地面的投影
            TargetData temp_target = this->image_targets[0];
            Vector3f current_targets = getCurrentPositionTargets().position;
            Vector3d world_target_point(current_targets.x(), current_targets.y(), 0.0);
            auto shot_target_opt = device->get_camera()->worldToPixel(world_target_point);
            if (shot_target_opt.has_value()) {
                Vector2d shot_center = shot_target_opt.value();
                temp_target.x = shot_center.x();
                temp_target.y = shot_center.y();
                // temp_target.z = shot_center.z();
                temp_target.radius = getCurrentPositionTargets().radius * accuracy; // 使用设定的精度作为像素半径
                temp_target.category = YOLODetector::enumToString(parameters.target_type).append("_t2p");
                device->get_yolo_detector()->append_target(temp_target);
            }

            device->log_info_throttle(std::chrono::milliseconds(1000), get_string(), ": Approaching target at (", 
                getCurrentPositionTargets().position.x(), ", ",
                getCurrentPositionTargets().position.y(), ", ",
                getCurrentPositionTargets().position.z(), ") with radius ", 
                getCurrentPositionTargets().radius);
            float target_z = getCurrentPositionTargets().position.z() > 2 ? getCurrentPositionTargets().position.z() :
                             getCurrentPositionTargets().position.z() < 1.8 && device->get_z_pos() > 2 ? 1.8 :
                             getCurrentPositionTargets().position.z() < 1.5 && device->get_z_pos() > 1.7 ? 1.5 :
                             getCurrentPositionTargets().position.z() < 1.2 && device->get_z_pos() > 1.3 ? 1.15 :
                             getCurrentPositionTargets().position.z();
            device->log_info_throttle(std::chrono::milliseconds(1000), get_string(), ": target_z: ", target_z, ", device_z: ", device->get_z_pos(), ", target: ", getCurrentPositionTargets().position.transpose());
            if (is_equal(target_z, device->get_z_pos(), 0.10f)) {
                target_z = getCurrentPositionTargets().position.z();
            }
            float rotated_x, rotated_y;  // 声明待旋转目标坐标
            device->rotate_world2local(this->device_position[parameters.device_index].x(), this->device_position[parameters.device_index].y(), rotated_x, rotated_y);
            device->send_world_setpoint_command(
                    getCurrentPositionTargets().position.x() + rotated_x,
                    getCurrentPositionTargets().position.y() + rotated_y,
                    target_z + device_position[parameters.device_index].z(), parameters.target_yaw); // 发送世界坐标系下的航点指令
            if (parameters.task_type == Type::TARGET && is_equal(getCurrentPositionTargets().position.x() + rotated_x, device->get_x_pos(), radius)
                && is_equal(getCurrentPositionTargets().position.y() + rotated_y, device->get_y_pos(), radius)
                && is_equal(getCurrentPositionTargets().position.z() + device_position[parameters.device_index].z(), device->get_z_pos(), radius)) {
                // device->log_info("Arrive, Doshot");
                task_result = true;
            } else {
                task_result = false;
            }
            return false;
        }

        // PID
        if (!getCurrentImageTargets().isZero() && (parameters.task_type == Type::PID || parameters.task_type == Type::AUTO)) {
            if (parameters.task_type == Type::AUTO && current_type == Type::TARGET) {
                auto_target_position_index++;
            }
            current_type = Type::PID;
            max_target_position_accurate = 5.0f; // PID模式下，允许更大的误差
            device->log_info_throttle(std::chrono::milliseconds(1000), get_string(), ": Approaching image target ", parameters.device_index, " at (",
                getCurrentImageTargets().x(), ", ",
                getCurrentImageTargets().y(), ")");
            // pid_defaults = PID::readPIDParameters(parameters.config_file_name, "pid");

            // 检查是否有目标
            if (parameters.config_device_name_suffix.empty() && image_targets.empty()) {
                device->log_error(get_string(), ": 目标不存在");
                return true; // 结束任务
            }
            
            // 验证设定的当前目标索引有效性 (removed < 0 check since device_index is unsigned)
            if (parameters.device_index >= image_targets.size() &&
                parameters.device_index >= device_position.size()) {
                device->log_error(get_string(), ": Invalid parameters.device_index: ", parameters.device_index, ", image_targets.size(): ", image_targets.size(), ", device_position.size(): ", device_position.size());
                return true;
            }
            
            // 重置可视化目标的颜色
            for (size_t i = 0; i < this->image_targets.size(); i++)
            {
                this->image_targets[i].r = 1.0f; // 设置所有目标颜色为红色
                this->image_targets[i].g = 0.0f;
                this->image_targets[i].b = 0.0f;
                this->image_targets[i].relative_z = device->get_camera()->get_position().z() - parameters.target_height; // 设置目标的高度为相机高度
            }


            std::vector<TargetData> t2p_targets;
            for(size_t i = 0; i < this->device_position.size(); i++)
            {
                TargetData t2p_target = image_targets[0];
                float rotated_x, rotated_y;  // 声明待旋转目标坐标
                device->rotate_world2local(this->device_position[i].x(), this->device_position[i].y(), rotated_x, rotated_y);
                if (device->debug_mode_) {
                    device->log_info(get_string(), ": 计算目标 ", i, "(", this->device_position[i].x(), ", ", this->device_position[i].y(), ") 在世界坐标系的位置: (", 
                        device->get_x_pos() + rotated_x, ", ",
                        device->get_y_pos() + rotated_y, ", ",
                        parameters.target_height, ")");         // 目标为投弹位置到地面上目标的投影
                }
                Vector3d world_point_target(
                    device->get_x_pos() + rotated_x,
                    device->get_y_pos() + rotated_y,
                    parameters.target_height
                    // -1.0
                    // device->get_target_position().z()
                );
                // 获取读取的需要接近的相对飞机目标在地面上的映射像素坐标
                auto output_pixel_opt = device->get_camera()->worldToPixel(world_point_target);
                if (output_pixel_opt.has_value()) {
                    Vector2d output_pixel = output_pixel_opt.value();
                    t2p_target.x = output_pixel.x();
                    t2p_target.y = output_pixel.y();
                    t2p_target.category = YOLODetector::enumToString(parameters.target_type).append("_d2p").append(parameters.config_device_name_suffix[i]);
                    if (!device->debug_mode_) {
                        t2p_target.radius = getCurrentPositionTargets().radius * accuracy; // 设置目标半径为像素半径的百分比
                        t2p_targets.push_back(t2p_target);
                    } else { // 发布所有的目标
                        // std::cout << "计算目标 " << i << " 在图像上的位置: (" << t2p_target.x << ", " << t2p_target.y << ")" << std::endl;
                        t2p_target.radius = 0.15 / 2.0f * accuracy; // 设置目标半径为像素半径的百分比
                        t2p_targets.push_back(t2p_target);
                        t2p_target.radius = 0.20 / 2.0f * accuracy; // 设置目标半径为像素半径的百分比
                        t2p_targets.push_back(t2p_target);
                        t2p_target.radius = 0.25 / 2.0f * accuracy; // 设置目标半径为像素半径的百分比
                        t2p_targets.push_back(t2p_target);
                    }
                }
            }
            device->get_yolo_detector()->append_targets(t2p_targets); // 将投弹到拍摄目标添加到YOLO中准备发布
            
            TargetData current_target; // 当前投弹目标
            if(parameters.device_index < t2p_targets.size() && !t2p_targets.empty()){    // 使用计算出的图像上目标
                // std::cout << "使用计算出的图像上目标, parameters.device_index: " << parameters.device_index << ", t2p_targets.size(): " << t2p_targets.size() << std::endl;
                current_target = t2p_targets[parameters.device_index];
                t2p_targets.erase(t2p_targets.begin() + parameters.device_index); // 移除当前投弹目标，避免重复添加
            } else if (parameters.device_index < image_targets.size()) {     // 使用读取的对应目标
                device->log_info(get_string(), ": 找不到目标在图像的映射，parameters.device_index: ", parameters.device_index, ", t2p_targets.size(): ", t2p_targets.size(), ", image_targets.size(): ", image_targets.size());
                current_target = image_targets[parameters.device_index];
                // targets.erase(targets.begin() + parameters.device_index); // 移除当前投弹目标，避免重复添加
            } else {  // 如果没有有效目标，使用默认值
                current_target = image_targets[0];
            }
            // 设置当前目标颜色为黄色
            current_target.r = 1.0f;
            current_target.g = 1.0f; 
            current_target.b = 0.0f;


            // PID发布速度接近目标点, 输入目标像素坐标，返回是否到达目标点

            // RCLCPP_INFO(device->get_node()->get_logger(), "--------------------\n\n读取pid参数: p: %f, i: %f, d: %f, ff: %f, dff: %f, imax: %f", defaults.p, defaults.i, defaults.d, defaults.ff, defaults.dff, defaults.imax);d_max_xy: %f, speed_max_z: %f, accel_max_x: %f, accel_max_z: %f", limits.speed_max_xy, limits.speed_max_z, limits.accel_max_xy, limits.accel_max_z);
            // 检查YOLO帧尺寸是否有效
            // if (get_cap_frame_width() <= 0 || get_cap_frame_height() <= 0) {
            //     RCLCPP_ERROR(device->get_node()->get_logger(), "Invalid YOLO frame dimensions: width=%d, height=%d", 
            //                  get_cap_frame_width(), get_cap_frame_height());
            //     return false;
            // }
            // yolo返回值坐标系：x右y下（x_flip|y_flip = false），转换为飞机坐标系：x右y上
            float now_x = getCurrentImageTargets().x();    // get_cap_frame_width() - get_x(target);
            float now_y = device->get_yolo_detector()->get_cap_frame_height() - getCurrentImageTargets().y();    // get_y(target);
            float tar_u = current_target.x;    // get_cap_frame_width() - current_target.x; // 目标x坐标
            float tar_v = device->get_yolo_detector()->get_cap_frame_height() - current_target.y;    // tar_y; // 目标y坐标
            // 检查坐标是否有效
            // if (!std::isfinite(now_x) || !std::isfinite(now_y) || !std::isfinite(current_target.x) || !std::isfinite(tar_y)) {
            //     RCLCPP_ERROR(device->get_node()->get_logger(), "Invalid coordinates detected");
            //     return false;
            // }
            rotate_xy(now_x, now_y, -device->get_world_yaw()); // 将目标坐标旋转到世界坐标系 headingangle_compass
            rotate_xy(tar_u, tar_v, -device->get_world_yaw()); // 将目标坐标旋转到世界坐标系 headingangle_compass
            float max_frame = std::max(device->get_yolo_detector()->get_cap_frame_width(), device->get_yolo_detector()->get_cap_frame_height());
            // RCLCPP_INFO(device->get_node()->get_logger(), "catch_target_bucket: yaw: %f, default_yaw: %f, headingangle_compass: %f", get_yaw(), default_yaw, headingangle_compass);
            // RCLCPP_INFO(device->get_node()->get_logger(), "catch_target_bucket: now_x: %f, now_y: %f, tar_u: %f, tar_y: %f", now_x, now_y, tar_u, tar_y);
            // RCLCPP_INFO(device->get_node()->get_logger(), "catch_target_bucket: now_x: %f, now_y: %f, tar_u: %f, tar_y: %f", now_x / get_cap_frame_width(), now_y / get_cap_frame_height(), (tar_u) / get_cap_frame_width(), (tar_y) / get_cap_frame_height());
            // RCLCPP_INFO(device->get_node()->get_logger(), "catch_target_bucket: now_z: %f, current_target.z: %f, now_yaw: %f, parameters.target_yaw: %f", get_z_pos(), current_target.z, get_yaw(), parameters.target_yaw);
            // RCLCPP_INFO(device->get_node()->get_logger(), "catch_target: current_target.caculate_pixel_radius(): %f, max_frame: %f", current_target.caculate_pixel_radius(), max_frame);

            // bool trajectory_setpoint_world(Vector4f pos_now, Vector4f pos_target, PID::Defaults defaults, double current_target.caculate_pixel_radius(), double yaw_current_target.caculate_pixel_radius(), bool calculate_or_get_vel, float vel_x = DEFAULT_VELOCITY, float vel_y = DEFAULT_VELOCITY);
            device->log_info_throttle(std::chrono::milliseconds(1000), get_string(), ": now_x: ",
                now_x, ", now_y: ",
                now_y, ", target_x: ",
                tar_u, ", target_y: ",
                tar_v, ", target_z: ",
                current_target.z);
            device->get_position_controller()->trajectory_setpoint_world(
                Vector4f{tar_u / max_frame, tar_v / max_frame, static_cast<float>(device->get_z_pos()), static_cast<float>(device->get_world_yaw())}, // 当前坐标        get_world_yaw()  // 当前坐标
                Vector4f{now_x / max_frame, now_y / max_frame, current_target.z, parameters.target_yaw + device->get_default_world_yaw()}, // 目标坐标  parameters.target_yaw
                pid_defaults,
                0.0,               				// 精度
                0.0 			 				// 偏航精度
                // true             				// 是否不使用飞机速度计算
                //	get_velocity_x(target) / max_frame, 	// 飞机速度
                //	get_velocity_y(target) / max_frame  	// 飞机速度
            );
            if ((abs(now_x - tar_u) <= current_target.caculate_pixel_radius() && abs(now_y - tar_v) <= current_target.caculate_pixel_radius()) ||
                (abs(now_x - image_targets[parameters.device_index].x) <= current_target.caculate_pixel_radius() && abs(now_y - image_targets[parameters.device_index].y) <= current_target.caculate_pixel_radius()))  // 接近目标
            {
                device->log_info_throttle(std::chrono::milliseconds(1000), get_string(), ": Arrive at image target (", now_x, ", ", now_y, ") with radius ", current_target.caculate_pixel_radius());    // return true;
                current_target.r = 0.0f; // 设置当前目标颜色为绿色
                current_target.g = 1.0f;
                current_target.b = 0.0f;
                task_result = true; // 设置任务结果为成功   
            } else {
                task_result = false; // 设置任务结果为失败
            }
            device->get_yolo_detector()->append_target(current_target); // 将当前投弹目标添加到YOLO中准备发布
            // 接近目标
            return false;
        }
        current_type = Type::NONE;

        // return true; // 没有目标，结束任务
    }
    device->log_info_throttle(std::chrono::milliseconds(1000), get_string(), ": No target found");
    return false;
}

template<typename DeviceType>
bool AppochTargetTask::end(DeviceType device) {
    // if constexpr (!std::is_same_v<DeviceType, std::shared_ptr<APMROS2Drone>>) {
    if (device->get_camera() == nullptr || device->get_servo_controller() == nullptr || device->get_yolo_detector() == nullptr) {
        device->log_error(get_string(), ": init can only be used with APMROS2Drone");
        return false;
    } else {
        device->log_info(get_string());
        device->get_position_controller()->reset_limits();
        return true;
    }
}

// Explicit template instantiations
template bool AppochTargetTask::init<std::shared_ptr<AbstractDrone>>(std::shared_ptr<AbstractDrone> device);
template bool AppochTargetTask::run<std::shared_ptr<AbstractDrone>>(std::shared_ptr<AbstractDrone> device);
template bool AppochTargetTask::end<std::shared_ptr<AbstractDrone>>(std::shared_ptr<AbstractDrone> device);

template bool AppochTargetTask::init<std::shared_ptr<ROS2Drone>>(std::shared_ptr<ROS2Drone> device);
template bool AppochTargetTask::run<std::shared_ptr<ROS2Drone>>(std::shared_ptr<ROS2Drone> device);
template bool AppochTargetTask::end<std::shared_ptr<ROS2Drone>>(std::shared_ptr<ROS2Drone> device);

template bool AppochTargetTask::init<std::shared_ptr<APMROS2Drone>>(std::shared_ptr<APMROS2Drone> device);
template bool AppochTargetTask::run<std::shared_ptr<APMROS2Drone>>(std::shared_ptr<APMROS2Drone> device);
template bool AppochTargetTask::end<std::shared_ptr<APMROS2Drone>>(std::shared_ptr<APMROS2Drone> device);