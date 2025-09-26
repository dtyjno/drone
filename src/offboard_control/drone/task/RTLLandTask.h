#pragma once

#include "../../task/Task.h"
#include "../../task/BlankTask.h"

class RTLLandTask : public Task<RTLLandTask>
{
public:
    static std::map<std::string, std::shared_ptr<RTLLandTask>> TASKS;

    // 静态工厂方法
    static std::shared_ptr<RTLLandTask> createTask(const std::string& task_name = "RTLLand");
    static std::shared_ptr<RTLLandTask> getTask(const std::string& task_name = "RTLLand");

    std::shared_ptr<RTLLandTask> setLandTask(std::shared_ptr<TaskBase> land_task){
        this->land_task = land_task;
        return std::static_pointer_cast<RTLLandTask>(shared_from_this());
    }

private:
    RTLLandTask(std::string name) : 
        Task<RTLLandTask>(name) {}

    std::shared_ptr<TaskBase> land_task = BlankTask::createTask("Blank");

public:
    // CRTP 需要的方法 - 由基类的 impl() 调用
    template<typename DeviceType>
    bool init(DeviceType device);  // 初始化任务
    template<typename DeviceType>
    bool run(DeviceType device);   // 执行任务
    template<typename DeviceType>
    bool end(DeviceType device);   // 结束任务
};
