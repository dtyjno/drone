#pragma once

#include <vector>
#include <functional>
#include "../task/Task.h"
#include "../utils/math.h"
#include "../algorithm/pid/PID.h"  // Add this include for PID::Defaults
#include "../drone/YOLOTargetType.h"
// 前向声明
class AbstractDrone;
class ROS2Drone;
class APMROS2Drone;
class YOLODetector;

class AppochTargetTask : public Task<AppochTargetTask>
{
public:
    static std::map<std::string, std::shared_ptr<AppochTargetTask>> TASKS;

    // 静态工厂方法
    static std::shared_ptr<AppochTargetTask> createTask(const std::string& task_name = "DoShot");
    static std::shared_ptr<AppochTargetTask> getTask(const std::string& task_name = "DoShot");
public:
    enum class Type{
        AUTO,
        PID,
        TARGET,
        NONE
    } current_type = Type::NONE;
    Type getCurrentType() const {
        return current_type;
    }
    std::string getCurrentTypeString() const {
        switch (current_type) {
            case Type::PID: return "PID";
            case Type::TARGET: return "TARGET";
            case Type::NONE: return "NONE";
            default: return "UNKNOWN";
        }
    }
    void setTaskType(Type type) {
        parameters.task_type = type;
    }
    void set_device_index(size_t index) {
        parameters.device_index = index;
    }

    std::shared_ptr<AppochTargetTask> set_task_when_no_target(std::shared_ptr<TaskBase> waypoint_task) {
        this->waypoint_task = waypoint_task;
        return std::static_pointer_cast<AppochTargetTask>(this->shared_from_this());
    }
    struct PositionTarget {
        Vector3f position;       // 位置
        float radius;            // 直径
        size_t index;            // 目标索引
    };
    struct Parameters{
        std::string config_file_name = "shot_config.yaml";                      // 配置文件名
        std::string config_device_name_prefix = "shot_target";                  // 配置文件中目标前缀
        std::vector<std::string> config_device_name_suffix = {"_r", "_l"};      // 配置文件中目标后缀，决定了支持的目标数量
        YOLO_TARGET_TYPE target_type;                                  // 目标类型
        size_t device_index = 0;                                                // 当前接近的设备索引,从0开始
        float fx = 1.0f;                                        // 相机焦距，像素单位
        std::function<PositionTarget()> dynamic_target_position_callback;         // 获取动态准确目标坐标的回调函数 x,y,z,r
        std::function<Vector2f()> dynamic_target_image_callback;         // 获取动态图像目标坐标的回调函数 x,y
        float target_height = 0.0f;                             // 目标的高度，默认为地面高度0.0m
        float target_yaw = 0.0f;                                // 目标偏航角
        Type task_type = Type::AUTO;                                  // 任务类型，AUTO自动选择PID或TARGET
    };


    Timer target_timer;               // 目标检测计时器
    float max_target_position_accurate = 0.1f;  // 设置最大允许误差
    // Vector4f pre_position_targets = Vector4f::Zero(); // 上一次的位置目标
	int auto_target_position_index = 0;     // 目标位置索引
    int get_auto_target_position_index() const {
        return auto_target_position_index;
    }
    void set_auto_target_position_index(int index) {
        auto_target_position_index = index;
    }

    std::shared_ptr<TaskBase> waypoint_task = nullptr;
    void is_pid_end_use_next_target_index(bool flag) {
        pid_end_use_next_target_index = flag;
    }
private:
    AppochTargetTask(std::string name) : 
        Task<AppochTargetTask>(name) {}

    // 任务执行
	float accuracy = 0.1;						    // 声明读取的准确度
    PID::Defaults pid_defaults;                     // 声明读取的默认PID参数
    PID::Defaults pos_pid_defaults;                     // 声明读取的默认PID参数
    std::vector<Vector3f> device_position;          // 声明读取的设备需要接近目标的位置
    std::vector<YOLO_TARGET_TYPE> targets; 		// 声明读取的映射失败时目标u和v坐标
    std::vector<TargetData> image_targets;    // 声明读取的映射失败时目标u和v坐标
    float radius = 0.1; 						    // 声明读取的映射失败时使用的像素精度
    Parameters parameters;
    bool pid_end_use_next_target_index = false;                          // 投弹标志
    bool use_pos_pid = true;                                            // 使用位置PID控制
public:
    void reset() {
        Task<AppochTargetTask>::reset();
        current_type = Type::NONE;
        target_timer.reset();
        // pre_position_targets = Vector4f::Zero();
    }

    void setParameters(Parameters &parameters) {
        this->parameters = parameters;
    }

    // 设置动态目标坐标回调函数
    void setDynamicPositionTargetCallback(std::function<PositionTarget()> callback) {
        parameters.dynamic_target_position_callback = callback;
    }
    
    // 获取当前目标坐标（动态或静态）
    PositionTarget getCurrentPositionTargets() {
        if (parameters.dynamic_target_position_callback) {
            return parameters.dynamic_target_position_callback();
        }
        return {};
    }

    // 设置动态目标坐标回调函数
    void setDynamicImageTargetCallback(std::function<Vector2f()> callback) {
        parameters.dynamic_target_image_callback = callback;
    }
    
    // 获取当前目标坐标（动态或静态）
    Vector2f getCurrentImageTargets() {
        if (parameters.dynamic_target_image_callback) {
            return parameters.dynamic_target_image_callback();
        }
        return {};
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
