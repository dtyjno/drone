#pragma once

#include <vector>
#include "Task.h"
#include "../utils/math.h"

#include "AppochTargetTask.h"

// 前向声明
// class AbstractDrone;
// class ROS2Drone;
// class APMROS2Drone;

class DoShotTask : public Task<DoShotTask>
{
public:
    static std::map<std::string, std::shared_ptr<DoShotTask>> TASKS;

    // 静态工厂方法
    static std::shared_ptr<DoShotTask> createTask(const std::string& task_name = "DoShot");
    static std::shared_ptr<DoShotTask> getTask(const std::string& task_name = "DoShot");

    std::shared_ptr<DoShotTask> set_task_when_no_target(std::shared_ptr<TaskBase> waypoint_task) {
        this->waypoint_task = waypoint_task;
        return std::static_pointer_cast<DoShotTask>(this->shared_from_this());
    }

    // 设置动态目标坐标回调函数
    void setDynamicPositionTargetCallback(std::function<Vector4f()> callback) {
        parameters.dynamic_target_position_callback = callback;   // 防止重置后init重置为参数更改
        task->setDynamicPositionTargetCallback(callback);
    }
    
    // 设置动态目标坐标回调函数
    void setDynamicImageTargetCallback(std::function<Vector2f()> callback) {
        parameters.dynamic_target_image_callback = callback;
        task->setDynamicImageTargetCallback(callback);
    }

    struct Parameters{
        std::string config_file_name = "shot_config.yaml";                      // 配置文件名
        std::string config_device_name_prefix = "shot_target";                  // 配置文件中目标前缀
        std::vector<std::string> config_device_name_suffix = {"_r", "_l"};      // 配置文件中目标后缀，决定了支持的目标数量
        // YOLODetector::TARGET_TYPE target_type;                                  // 目标类型
        size_t device_index = 1;                                                // 当前接近的设备索引
        std::function<Vector4f()> dynamic_target_position_callback;         // 获取动态准确目标坐标的回调函数 x,y,z,r
        std::function<Vector2f()> dynamic_target_image_callback;         // 获取动态图像目标坐标的回调函数 x,y,z,r
        float target_height = 0.0f;                             // 目标的高度，默认为地面高度0.0m
        float target_yaw = 0.0f;                                // 目标偏航角
        float shot_duration = -1.0f; 					    // 等待读取，稳定持续时间
        float shot_wait = -1.0f; 				     		// 等待读取，投弹后稳定时间
    };

    bool is_shot() const {
        return shot_flag;
    }

    std::shared_ptr<AppochTargetTask> get_appochtarget_task(){
        if (!task) {
            task = AppochTargetTask::createTask(get_name().append("_approach"));
        }
        return task;
    }
    

private:
    DoShotTask(std::string name) : 
        Task<DoShotTask>(name) {
        task = AppochTargetTask::createTask(get_name().append("_approach"));
    }

    // 任务执行
	float find_duration = 0; 					    // 接近目标时执行时间
    float shot_duration = 2; 					    // 等待读取，稳定持续时间
    float shot_wait = 0.5; 				     		// 等待读取，投弹后稳定时间
	bool shot_flag = false; 						// 投弹标志

    std::shared_ptr<AppochTargetTask> task;
    std::shared_ptr<TaskBase> waypoint_task = nullptr;
    Parameters parameters;
public:
    std::shared_ptr<DoShotTask> setParameters(Parameters &parameters) {
        this->parameters = parameters;
        // AppochTargetTask::Parameters task_params;
        // // task_params.config_file_name = parameters.config_file_name;
        // // task_params.config_device_name_prefix = parameters.config_device_name_prefix; // Use first suffix as prefix
        // // task_params.config_device_name_suffix = parameters.config_device_name_suffix;
        // task_params.target_type = YOLO_TARGET_TYPE::CIRCLE;                                  // 目标类型
        // task_params.device_index = parameters.device_index;
        // // task_params.fx = device->get_camera()->get_fx();                                        // 相机焦距，像素单位
        // task_params.dynamic_target_position_callback = parameters.dynamic_target_position_callback;         // 获取动态目标坐标的回调函数 x,y,z,r
        // task_params.dynamic_target_image_callback = parameters.dynamic_target_image_callback;         // 获取动态图像目标坐标的回调函数 x,y,z,r
        // task_params.target_height = parameters.target_height;                             // 目标的高度，默认为地面高度0.0m
        // task_params.target_yaw = parameters.target_yaw;                                 // 目标偏航角
        // task->setParameters(task_params);
        return std::static_pointer_cast<DoShotTask>(this->shared_from_this());
    }

    void reset() override {
        Task<DoShotTask>::reset();
        if (task) {
            task->reset();
        }
        find_duration = 0.0f;               // 重置查找持续时间
        shot_flag = false;                  // 重置投弹标志
    }
public:
    // CRTP 需要的方法 - 由基类的 impl() 调用
    template<typename DeviceType>
    bool init(DeviceType device);  // 初始化任务
    template<typename DeviceType>
    bool run(DeviceType device);   // 执行任务
    template<typename DeviceType>
    bool end(DeviceType device);   // 结束任务
};
