#pragma once

#include "Task.h"

class BlankTask : public Task<BlankTask>
{
public:
    static std::map<std::string, std::shared_ptr<BlankTask>> TASKS;

    // 静态工厂方法
    static std::shared_ptr<BlankTask> createTask(const std::string& task_name = "Blank");
    static std::shared_ptr<BlankTask> getTask(const std::string& task_name = "Blank");
private:
    BlankTask(std::string name) : 
        Task<BlankTask>(name) {}

    // 任务执行计数器
    int execute_count_ = 0;

public:
    // CRTP 需要的方法 - 由基类的 impl() 调用
    template<typename DeviceType>
    bool init(DeviceType device);  // 初始化任务
    template<typename DeviceType>
    bool run(DeviceType device);   // 执行任务
    template<typename DeviceType>
    bool end(DeviceType device);   // 结束任务
};
