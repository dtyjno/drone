#include "DoShotTask.h"
#include "../drone/YOLODetector.h"

// 定义静态成员
std::map<std::string, std::shared_ptr<DoShotTask>> DoShotTask::TASKS;

std::shared_ptr<DoShotTask> DoShotTask::createTask(const std::string& task_name) {
    if (TASKS.find(task_name) == TASKS.end()) {
        DoShotTask *new_task = new DoShotTask(task_name);
        TASKS[task_name] = std::shared_ptr<DoShotTask>(new_task);
        std::cout << "Creating " << TASKS[task_name]->get_name() << " instance." << std::endl;
    }
    TASKS[task_name]->next_task(TASKS[task_name]);
    return TASKS[task_name];
}

std::shared_ptr<DoShotTask> DoShotTask::getTask(const std::string& task_name) {
    auto it = TASKS.find(task_name);
    if (it != TASKS.end()) {
        return it->second;
    }
    std::cerr << "Warning: Task " << task_name << " not found." << std::endl;
    return nullptr;
}

template<typename DeviceType>
bool DoShotTask::init(DeviceType device) {
    // device->log_info(get_string());
    // if constexpr (!std::is_same_v<DeviceType, std::shared_ptr<APMROS2Drone>>) {
    if (device->get_camera_gimbal() == nullptr || device->get_servo_controller() == nullptr || device->get_yolo_detector() == nullptr) {
        // device->log_error("%s: DoShotTask::init can only be used with APMROS2Drone", get_string());
        device->log_error(get_string(), ": init can only be used with APMROS2Drone");
        return false;
    } else {
        YAML::Node config = Readyaml::readYAML(parameters.config_file_name);
        shot_duration = parameters.shot_duration > 0 ? parameters.shot_duration : config["shot_duration"].as<float>();
        shot_wait = parameters.shot_wait > 0 ? parameters.shot_wait : config["shot_wait"].as<float>();
        AppochTargetTask::Parameters task_params;
        task_params.config_file_name = parameters.config_file_name;
        task_params.config_device_name_prefix = parameters.config_device_name_prefix; // Use first suffix as prefix
        task_params.config_device_name_suffix = parameters.config_device_name_suffix;
        task_params.target_type = YOLO_TARGET_TYPE::CIRCLE;                                  // 目标类型
        task_params.device_index = parameters.device_index;
        task_params.fx = device->get_camera()->get_fx();                                        // 相机焦距，像素单位
        task_params.dynamic_target_position_callback = parameters.dynamic_target_position_callback;         // 获取动态目标坐标的回调函数 x,y,z,r
        task_params.dynamic_target_image_callback = parameters.dynamic_target_image_callback;         // 获取动态图像目标坐标的回调函数 x,y,z,r
        task_params.target_height = parameters.target_height;                             // 目标的高度，默认为地面高度0.0m
        task_params.target_yaw = parameters.target_yaw;                                 // 目标偏航角
        task_params.task_type = parameters.task_type;                                   // 任务类型
        // task_params.task_type = AppochTargetTask::Type::PID;                                  // 任务类型
        task->setParameters(task_params);
        task->reset();
        find_duration = 0.0f;               // 重置查找持续时间
        shot_flag = false;                  // 重置投弹标志
        return true;
    }
}

template<typename DeviceType>
bool DoShotTask::run(DeviceType device) {
    // device->log_info(get_string());
    // return true;
    // if constexpr (!std::is_same_v<DeviceType, std::shared_ptr<APMROS2Drone>>) {
    if (device->get_camera_gimbal() == nullptr || device->get_servo_controller() == nullptr || device->get_yolo_detector() == nullptr) {
        device->log_error(get_string(), ": init can only be used with APMROS2Drone");
        return false;
    } else {
        device->log_info_throttle(std::chrono::milliseconds(1000), get_string(), ": run, index = ", parameters.device_index, "s");
        // 未投弹且无目标
        // if (!shot_flag && !device->get_yolo_detector()->is_get_target(YOLO_TARGET_TYPE::CIRCLE))
        // {
        //     device->log_info("Doshot: yolo未识别到桶，等待");
        // } 
        // 执行投弹命令后，如果查找到持续时间大于于投弹持续时间+等待时间
        if (shot_flag) 
        {
            find_duration += device->get_wait_time(); // 投弹后必定累加查找持续时间
            if (find_duration >= shot_duration + shot_wait) {
                device->log_info("Doshot: 投弹后等待已完成, find_duration_time = ", find_duration, "s");
                task_result = true;
                return true;
            }
        }
        // 已经投弹且无可选目标
        if (shot_flag && !device->get_yolo_detector()->is_get_target(YOLO_TARGET_TYPE::STUFFED) &&
                !device->get_yolo_detector()->is_get_target(YOLO_TARGET_TYPE::CIRCLE))
        {
            device->log_info("Doshot: 已投弹，yolo未识别到有东西的桶或空桶，原地等待");
            device->send_velocity_command(0, 0, 0, 0); // 停止飞行
        } else {
            task->visit(device);
            
            // 执行接近目标任务
            if ( (task->getCurrentType() == AppochTargetTask::Type::TARGET ||
                  task->getCurrentType() == AppochTargetTask::Type::PID) && 
                task->get_task_result()
                )
            {
                if (!shot_flag) // 未投弹时接近目标时间累加
                    find_duration += device->get_wait_time(); // 累加查找持续时间

                device->log_info_throttle(std::chrono::milliseconds(100), "(T 0.1s) Doshot: Approach, Doshot, time = ", find_duration,"s, type = ", task->getCurrentTypeString());
                if(!shot_flag && find_duration >= shot_duration){ 
                    device->log_info("Doshot: Approach, 投弹, time > ", shot_duration, "s");
                    if constexpr (std::is_same_v<DeviceType, std::shared_ptr<APMROS2Drone>>) {
                        device->shoted_cluster_ids.push_back(parameters.dynamic_target_position_callback().index);
                    }
                    shot_flag = true; // 设置投弹标志
                    device->get_servo_controller()->set_servo(11 + parameters.device_index, device->get_servo_controller()->get_servo_open_position()); // 设置舵机位置，投弹
                } 
                else if (shot_flag) // 已投弹，shot_wait时间内继续等待
                {
                    if (find_duration <= shot_duration + device->get_wait_time()) // 投弹后周期 重复投弹一次
                    {
                        device->log_info("Doshot: Arrive, 再次投弹, wait, time = %fs", find_duration - shot_duration);
                        device->get_servo_controller()->set_servo(11 + parameters.device_index, device->get_servo_controller()->get_servo_open_position()); // 重复投弹
                    } else {
                        device->log_info("Doshot: Arrive, 等待, wait, time = ", find_duration - shot_duration, "s");
                    }
                }			
            }
            // 未投弹且有目标且未接近目标
            else if (!shot_flag && task->getCurrentType() == AppochTargetTask::Type::NONE)   // task未执行操作，执行waypoint_task接近目标任务
            {
                if (waypoint_task != nullptr) {
                    device->log_info_throttle(std::chrono::milliseconds(100), get_string(), ": (T 0.1s) Doshot: Searching for target, time = ", find_duration, "s");
                    waypoint_task->visit(device);
                    if (waypoint_task->is_execute_finished()){
                        waypoint_task->reset();    // 重置任务以循环使用
                    }
                }
                // do_next_task = true;
                find_duration = 0.0f; // 重置查找持续时间

            }
            // return false;
        }

        return false;
    }
}

template<typename DeviceType>
bool DoShotTask::end(DeviceType device) {
    // if constexpr (!std::is_same_v<DeviceType, std::shared_ptr<APMROS2Drone>>) {
    if (device->get_camera_gimbal() == nullptr || device->get_servo_controller() == nullptr || device->get_yolo_detector() == nullptr) {
        device->log_error(get_string(), ": init can only be used with APMROS2Drone");
        return false;
    } else {
        // shot_flag = false;
        // device->log_info(get_string());
        // task_result = true;
        return true;
    }
}

// Explicit template instantiations
template bool DoShotTask::init<std::shared_ptr<AbstractDrone>>(std::shared_ptr<AbstractDrone> device);
template bool DoShotTask::run<std::shared_ptr<AbstractDrone>>(std::shared_ptr<AbstractDrone> device);
template bool DoShotTask::end<std::shared_ptr<AbstractDrone>>(std::shared_ptr<AbstractDrone> device);

template bool DoShotTask::init<std::shared_ptr<ROS2Drone>>(std::shared_ptr<ROS2Drone> device);
template bool DoShotTask::run<std::shared_ptr<ROS2Drone>>(std::shared_ptr<ROS2Drone> device);
template bool DoShotTask::end<std::shared_ptr<ROS2Drone>>(std::shared_ptr<ROS2Drone> device);

template bool DoShotTask::init<std::shared_ptr<APMROS2Drone>>(std::shared_ptr<APMROS2Drone> device);
template bool DoShotTask::run<std::shared_ptr<APMROS2Drone>>(std::shared_ptr<APMROS2Drone> device);
template bool DoShotTask::end<std::shared_ptr<APMROS2Drone>>(std::shared_ptr<APMROS2Drone> device);