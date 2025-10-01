#pragma once

#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include "../algorithm/KalmanFilter2D.h"
#include "../utils/Readyaml.h"
#include "YOLOTargetType.h"

class YOLODetectorInterface {
public:
    virtual ~YOLODetectorInterface() = default;

    // 获取目标类型字符串
    // virtual std::string enumToString(YOLO_TARGET_TYPE t) const = 0;

    // 检查是否检测到目标
    virtual bool is_get_target(YOLO_TARGET_TYPE type) const = 0;

    // 获取原始目标框
    virtual std::vector<vision_msgs::msg::BoundingBox2D> get_raw_targets(YOLO_TARGET_TYPE type) const = 0;

    // 获取原始坐标
    virtual float get_raw_x(YOLO_TARGET_TYPE type) const = 0;
    virtual float get_raw_y(YOLO_TARGET_TYPE type) const = 0;

    // 获取滤波后坐标
    virtual float get_x(YOLO_TARGET_TYPE type) const = 0;
    virtual float get_y(YOLO_TARGET_TYPE type) const = 0;

    // 获取目标宽高
    virtual float get_width(YOLO_TARGET_TYPE type) const = 0;
    virtual float get_height(YOLO_TARGET_TYPE type) const = 0;

    // 获取速度信息
    virtual float get_velocity_x(YOLO_TARGET_TYPE type) const = 0;
    virtual float get_velocity_y(YOLO_TARGET_TYPE type) const = 0;

    // 配置读取
    virtual void read_configs(const std::string &filename) = 0;

    // 图像尺寸
    virtual int get_cap_frame_width() const = 0;
    virtual int get_cap_frame_height() const = 0;

    // 目标管理
    virtual void append_target(const TargetData &target) = 0;
    virtual void append_targets(const std::vector<TargetData> &new_targets) = 0;
    virtual void clear_targets() = 0;

    // 可视化发布
    virtual void publish_visualization_target() = 0;
};