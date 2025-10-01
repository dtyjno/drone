#pragma once

#include <Eigen/Dense>
#include <optional>
#include <string>

using namespace Eigen;

class CameraInterface {
public:
    virtual ~CameraInterface() = default;

    // 获取/设置相机与无人机的相对位置
    virtual Vector3d get_drone_to_camera() const = 0;
    virtual void set_drone_to_camera(const Vector3d& pos) = 0;

    // 获取/设置相机的旋转
    virtual Vector3d get_camera_relative_rotation() const = 0;
    virtual void set_camera_relative_rotation(const Vector3d& rot) = 0;

    // 获取/设置父节点位置与旋转
    virtual Vector3d get_parent_position() const = 0;
    virtual void set_parent_position(const Vector3d& pos) = 0;
    virtual Vector3d get_parent_rotation() const = 0;
    virtual void set_parent_rotation(const Vector3d& rot) = 0;

    // 获取相机焦距、主点、畸变系数、图像尺寸
    virtual double get_fx() const = 0;
    virtual double get_fy() const = 0;
    virtual double get_cx() const = 0;
    virtual double get_cy() const = 0;
    virtual double get_k1() const = 0;
    virtual double get_k2() const = 0;
    virtual double get_p1() const = 0;
    virtual double get_p2() const = 0;
    virtual double get_k3() const = 0;
    virtual double get_width() const = 0;
    virtual double get_height() const = 0;

    // 配置读取
    virtual void read_configs(const std::string& filename) = 0;

    // 坐标变换相关
    virtual Vector3d get_position() const = 0;
    virtual std::optional<Vector2d> worldToPixel(const Vector3d& world_point) const = 0;
    virtual std::optional<Vector3d> pixelToWorldPosition(const Vector2d& pixel_point, double object_height = 0.0) const = 0;

    // 计算像素半径和实际直径
    virtual double calculatePixelRadius(const Vector3d& world_center, double world_radius) const = 0;
    virtual double calculateRealDiameter(double pixel_diameter, double distance_to_object) const = 0;
};