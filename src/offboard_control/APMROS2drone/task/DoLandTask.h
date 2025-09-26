#pragma once

#include <vector>
#include "../../task/Task.h"
#include "../../utils/utils.h"
#include "../../algorithm/pid/PID.h"
#include "AppochTargetTask.h"
// 前向声明
class AbstractDrone;
class ROS2Drone;
class APMROS2Drone;
class ROS2Drone;
class APMROS2Drone;

class DoLandTask : public Task<DoLandTask>
{
public:
    static std::map<std::string, std::shared_ptr<DoLandTask>> TASKS;

    // 静态工厂方法
    static std::shared_ptr<DoLandTask> createTask(const std::string& task_name = "DoLand");
    static std::shared_ptr<DoLandTask> getTask(const std::string& task_name = "DoLand");
    
    struct Parameters{
        std::string config_file_name = "land_config.yaml";                      // 配置文件名
        std::string config_device_name_prefix = "land_target";                  // 配置文件中目标前缀
        std::vector<std::string> config_device_name_suffix = {""};      // 配置文件中目标后缀，决定了支持的目标数量
        size_t device_index = 1;                                                // 当前接近的设备索引
        float fx = 1.0f;                                        // 相机焦距，像素单位
        std::function<Vector4f()> dynamic_target_position_callback;         // 获取动态准确目标坐标的回调函数 x,y,z,r
        std::function<Vector2f()> dynamic_target_image_callback;         // 获取动态图像目标坐标的回调函数 x,y,z,r
        float target_height = 0.0f;                             // 目标的高度，默认为地面高度0.0m
        float target_yaw = 0.0f;                                // 目标偏航角
    };
private:
    DoLandTask(std::string name) : 
        Task<DoLandTask>(name) {}

    // 任务执行
	int surround_land = -3;
    double target_z;
	double scout_x = 0.0, scout_y = 0.0, scout_halt = 3.0, accuracy = 0.3;

    double x_home, y_home;      // 转换后坐标

    std::shared_ptr<AppochTargetTask> task;
    Parameters parameters;

public:
    // CRTP 需要的方法 - 由基类的 impl() 调用
    template<typename DeviceType>
    bool init(DeviceType device);  // 初始化任务
    template<typename DeviceType>
    bool run(DeviceType device);   // 执行任务
    template<typename DeviceType>
    bool end(DeviceType device);   // 结束任务
};
