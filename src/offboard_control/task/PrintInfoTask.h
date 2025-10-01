#pragma once

#include "Task.h"

class PrintInfoTask : public Task<PrintInfoTask>
{
public:
    static std::map<std::string, std::shared_ptr<PrintInfoTask>> TASKS;

    // 静态工厂方法
    static std::shared_ptr<PrintInfoTask> createTask(const std::string& task_name = "PrintInfo");
    static std::shared_ptr<PrintInfoTask> getTask(const std::string& task_name = "PrintInfo");

    std::shared_ptr<PrintInfoTask> set_print_time(int seconds) {
        this->print_time_ = seconds;
        return std::static_pointer_cast<PrintInfoTask>(this->shared_from_this());
    }
private:
    PrintInfoTask(std::string name) : 
        Task<PrintInfoTask>(name) {}

    // 任务执行计数器
    int execute_count_ = 0;
    int print_time_ = 10;

public:
    // CRTP 需要的方法 - 由基类的 impl() 调用
    template<typename DeviceType>
    bool init(DeviceType device);  // 初始化任务
    template<typename DeviceType>
    bool run(DeviceType device);   // 执行任务
    template<typename DeviceType>
    bool end(DeviceType device);   // 结束任务
};

