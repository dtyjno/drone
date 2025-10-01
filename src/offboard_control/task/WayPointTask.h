#pragma once

#include "Task.h"
#include "../waypoint/WayPoints.h"
#include "../utils/utils.h"
#include <vector>

class WayPointTask : public Task<WayPointTask>
{
public:
    static std::map<std::string, std::shared_ptr<WayPointTask>> TASKS;

    // 静态工厂方法
    static std::shared_ptr<WayPointTask> createTask(const std::string& task_name = "WayPoint");
    static std::shared_ptr<WayPointTask> getTask(const std::string& task_name = "WayPoint");

    // 设置和获取航点
    void set_waypoint(WayPoints way_point) {
        way_point_ = way_point;
    }

    WayPoints get_waypoint() {
        return way_point_;
    }

    void reset() {
        Task<WayPointTask>::reset();
        point_count = 0;
    }     

    class Parameters {
    public:
        float center_x = 0.0f;      // 中心点X
        float center_y = 0.0f;      // 中心点Y
        // float scope_length = 1.0f;  // 范围长度
        // float scope_width = 1.0f;   // 范围宽度
        // float halt_height = 2.0f;   // 悬停高度
        float point_time = 10.0f;   // 每个点的时间
        float accuracy = 0.2f;     // 位置精度
        enum class Frame {
            START = 0,   // 相对读取方向起始位置
            LOCAL = 1    // 相对当前位置
        } frame = Frame::START;
    };

    std::shared_ptr<WayPointTask> set_parameters(const Parameters &params) {
        parameters_ = params;
        return std::static_pointer_cast<WayPointTask>(this->shared_from_this());
    }

    Parameters get_parameters() const {
        return parameters_;
    }

    std::shared_ptr<WayPointTask> set_config(const WayPoints &way_point, const Parameters &params) {
        way_point_ = way_point;
        parameters_ = params;
        return std::static_pointer_cast<WayPointTask>(this->shared_from_this());
    }

    std::shared_ptr<WayPointTask> set_counter(int count) {
        point_count = count;
        return std::static_pointer_cast<WayPointTask>(this->shared_from_this());
    }

    int get_counter() const {
        return point_count;
    }


private:
    WayPointTask(std::string name) : 
        Task<WayPointTask>(name), 
        way_point_("default") {}

    WayPoints way_point_;
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
