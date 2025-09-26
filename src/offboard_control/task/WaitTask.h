#pragma once

#include "Task.h"

class WaitTask : public Task<WaitTask>
{
public:
    static std::map<std::string, std::shared_ptr<WaitTask>> TASKS;

    // 静态工厂方法
    static std::shared_ptr<WaitTask> createTask(const std::string& task_name = "Blank");
    static std::shared_ptr<WaitTask> getTask(const std::string& task_name = "Blank");
private:
    WaitTask(std::string name) : 
        Task<WaitTask>(name) {}

    // 任务执行计数器
    void set_wait_time(float s) { wait_time = s; }
    void is_blocked(bool blocked) { is_blocked_ = blocked; }
    float wait_time = 1.0f; // 等待时间，单位秒
    bool is_blocked_ = false; // 是否阻塞等待，阻塞则无限等待

public:
    // 配置方法
    std::shared_ptr<WaitTask> set_config(float wait_time_seconds, bool is_blocked = true) {
        this->wait_time = wait_time_seconds;
        this->is_blocked_ = is_blocked;
        return std::static_pointer_cast<WaitTask>(this->shared_from_this());
    }

    // CRTP 需要的方法 - 由基类的 impl() 调用
    template<typename DeviceType>
    bool init(DeviceType device);  // 初始化任务
    template<typename DeviceType>
    bool run(DeviceType device);   // 执行任务
    template<typename DeviceType>
    bool end(DeviceType device);   // 结束任务
};
