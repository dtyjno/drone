#pragma once

#include <vector>
#include "../utils/math.h"

class WayPoints {
public:
    WayPoints(const std::string &name) : name_(name) {
        // way_points_.push_back(Vector4f::Zero());  // 确保第一个航点为零点
    }
    
    WayPoints(const std::string &name, const std::vector<Vector4f> &way_points) : name_(name) {
        // way_points_.push_back(Vector4f::Zero());  // 确保第一个航点为零点
        // 添加其他航点
        for (const auto& point : way_points) {
            way_points_.push_back(point);
        }
    }

    std::string get_name() const {
        return name_;
    }

    void add_waypoint(const Vector4f &point) {
        way_points_.push_back(point);
    }

    void clear_waypoints() {
        way_points_.clear();
        // way_points_.push_back(Vector4f::Zero());  // 清空后重新添加零点作为第一个航点
    }

    size_t size() const {
        return way_points_.size();
    }

    const Vector4f &operator [](size_t index) const {
        return way_points_[index];
    }
    
protected:
    std::string name_;
    std::vector<Vector4f> way_points_;
};