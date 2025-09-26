#pragma once

#include <memory>

// 使用前向声明替代包含头文件
class AbstractDrone;
class ROS2Drone;
class APMROS2Drone;

class DeviceVisitor {
public:
    virtual ~DeviceVisitor() = default;
    virtual std::shared_ptr<DeviceVisitor> visit(std::shared_ptr<AbstractDrone> drone) = 0;
    virtual std::shared_ptr<DeviceVisitor> visit(std::shared_ptr<ROS2Drone> drone) = 0;
    virtual std::shared_ptr<DeviceVisitor> visit(std::shared_ptr<APMROS2Drone> drone) = 0;
    // 可以根据需要添加更多的 visit 方法，支持不同的设备类型
};
