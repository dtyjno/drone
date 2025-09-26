#pragma once

#include "../../task/Task.h"
#include "../../waypoint/WayPoints.h"
#include "../../utils/utils.h"
#include <vector>

class SetPointTask : public Task<SetPointTask>
{
public:
    static std::map<std::string, std::shared_ptr<SetPointTask>> TASKS;

    // 静态工厂方法
    static std::shared_ptr<SetPointTask> createTask(const std::string& task_name = "SetPoint");
    static std::shared_ptr<SetPointTask> getTask(const std::string& task_name = "SetPoint");

    void reset() {
        Task<SetPointTask>::reset();
        point_count = 0;
        reset_timer();
    }     

    class Parameters {
    public:
        float target_x = 0.0f;      // 目标点X
        float target_y = 0.0f;      // 目标点Y
        float target_z = 2.0f;      // 目标点Z
        float target_yaw = 0.0f;    // 目标点Yaw
        float point_time = std::numeric_limits<float>::infinity();   // 每个点的时间
        float accuracy = 0.2f;     // 位置精度
        enum class Frame {
            START = 0,   // 相对读取方向起始位置
            LOCAL = 1    // 相对当前位置
        } frame = Frame::START;
    };

    std::shared_ptr<SetPointTask> set_parameters(const Parameters &params) {
        parameters_ = params;
        return std::static_pointer_cast<SetPointTask>(this->shared_from_this());
    }

    Parameters get_parameters() const {
        return parameters_;
    }

    std::shared_ptr<SetPointTask> set_config(const Parameters &params) {
        parameters_ = params;
        return std::static_pointer_cast<SetPointTask>(this->shared_from_this());
    }

    std::shared_ptr<SetPointTask> set_counter(int count) {
        point_count = count;
        return std::static_pointer_cast<SetPointTask>(this->shared_from_this());
    }

    int get_counter() const {
        return point_count;
    }
    
    void reset_timer() {
        timer_.set_start_time_to_default();
    }


private:
    SetPointTask(std::string name) : 
        Task<SetPointTask>(name) {}

    Timer timer_;         // 航点计时器
    Parameters parameters_;
    int point_count = 0;        // 当前点计数器

public:
    // CRTP 需要的方法 - 由基类的 impl() 调用
    template<typename DeviceType>
    bool init(DeviceType device);
    template<typename DeviceType>
    bool run(DeviceType device);
    template<typename DeviceType>
    bool end(DeviceType device);
};
