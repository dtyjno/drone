#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/mount_control.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include "math.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <optional>
#include <vector>
#include <iostream>
#include <iomanip>

using namespace Eigen;

// #include <optional>
// #include <vector>
// #include <iostream>
// #include "Yolo.h"  // 为了使用YOLO::Target结构体

// ros2 topic pub /mavros/mount_control/command mavros_msgs/msg/MountControl "{mode: 2, pitch: -90.0, roll: 0.0, yaw: 0.0}"

class Camera {
public:
    Vector3d position;    // 相机位置 (tx, ty, tz)
    Vector3d rotation;    // 欧拉角 (roll, pitch, yaw)
    double fx, fy; // 相机焦距 (fx, fy)
    double cx, cy; // 相机主点 (cx, cy)
    double k1 = 0.0, k2 = 0.0, p1 = 0.0, p2 = 0.0, k3 = 0.0; // 畸变系数 (k1, k2, p1, p2, k3)
    double width, height; // 图像宽度和高度
    Camera() : position(Vector3d::Zero()), rotation(Vector3d::Zero()), fx(1.0), fy(1.0), cx(0.0), cy(0.0), width(640), height(480) {
        read_configs("camera.yaml");
    }
    Camera(const Vector3d& pos, const Vector3d& rot, double fx, double fy, double cx, double cy, double width, double height)
        : position(pos), rotation(rot), fx(fx), fy(fy), cx(cx), cy(cy), width(width), height(height) {
    }
    
    void read_configs(const std::string &filename)
	{
		YAML::Node config = Readyaml::readYAML(filename);
        fx = config["fx"].as<double>(fx);
        fy = config["fy"].as<double>(fy);
        cx = config["cx"].as<double>(cx);
        cy = config["cy"].as<double>(cy);
        k1 = config["k1"].as<double>(k1);
        k2 = config["k2"].as<double>(k2);
        p1 = config["p1"].as<double>(p1);
        p2 = config["p2"].as<double>(p2);
        k3 = config["k3"].as<double>(k3);
        width = config["width"].as<double>(width);
        height = config["height"].as<double>(height);
    }
    // 像素坐标转换为归一化坐标（带畸变校正）
    Vector2d pixelToNormalized(const Vector2d& pixel) const {
        // 更新相机矩阵和畸变系数矩阵
        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 
            fx, 0, cx, 
            0, fy, cy, 
            0, 0, 1
        );
        cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) << 
            k1, k2, p1, p2, k3);
        // 输入点准备
        cv::Point2f src_pt(pixel.x(), pixel.y());
        std::vector<cv::Point2f> src = {src_pt}, dst;
        
        // 去畸变
        cv::undistortPoints(src, dst, camera_matrix, dist_coeffs);
        
        return Vector2d(dst[0].x, dst[0].y);
    }
    // 应用相机畸变
    Vector2d applyDistortion(const Vector2d& norm_point) const {
        double x = norm_point.x();
        double y = norm_point.y();
        
        // 径向畸变
        double r2 = x*x + y*y;
        double r4 = r2*r2;
        double r6 = r4*r2;
        double radial = 1 + k1*r2 + k2*r4 + k3*r6;
        
        // 切向畸变
        double tangential_x = 2*p1*x*y + p2*(r2 + 2*x*x);
        double tangential_y = p1*(r2 + 2*y*y) + 2*p2*x*y;
        
        // 应用畸变
        double x_distorted = x * radial + tangential_x;
        double y_distorted = y * radial + tangential_y;
        
        return Vector2d(x_distorted, y_distorted);
    }
    Matrix3d eulerAnglesToRotationMatrix() const {
        double roll = rotation[0];
        double pitch = rotation[1];
        double yaw = rotation[2];
        
        // 添加调试输出
        // std::cout << "欧拉角 (弧度): roll=" << roll << ", pitch=" << pitch << ", yaw=" << yaw << std::endl;
        // std::cout << "欧拉角 (角度): roll=" << roll*180/M_PI << "°, pitch=" << pitch*180/M_PI << "°, yaw=" << yaw*180/M_PI << "°" << std::endl;
        
        Matrix3d R_x; // 绕X轴旋转(roll)
        R_x << 1, 0, 0,
            0, cos(roll), -sin(roll),
            0, sin(roll), cos(roll);
        
        Matrix3d R_y; // 绕Y轴旋转(pitch)
        R_y << cos(pitch), 0, sin(pitch),
            0, 1, 0,
            -sin(pitch), 0, cos(pitch);
        
        Matrix3d R_z; // 绕Z轴旋转(yaw)
        R_z << cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1;
        
        // 添加中间矩阵调试
        // std::cout << "R_x (roll):" << std::endl << R_x << std::endl;
        // std::cout << "R_y (pitch):" << std::endl << R_y << std::endl; 
        // std::cout << "R_z (yaw):" << std::endl << R_z << std::endl;
        
        // 使用 Z-Y-X 顺序 (yaw-pitch-roll)
        Matrix3d result = R_z * R_y * R_x;
        
        // std::cout << "最终旋转矩阵:" << std::endl << result << std::endl;
        
        return result;
    }
    
    // 世界坐标转换为像素坐标
    std::optional<Vector2d> worldToPixel(const Vector3d& world_point) const {
        // 1. 世界坐标转换为相机坐标
        Vector3d relative_pos = world_point - position;
        
        // 2. 应用相机旋转矩阵的逆变换 (世界坐标 -> 相机坐标)
        Matrix3d R = eulerAnglesToRotationMatrix();
        Vector3d cam_point = R.transpose() * relative_pos;
        
        // 3. 检查点是否在相机前方
        if (cam_point.z() <= 0) {
            // 点在相机后方，无法投影
            return std::nullopt;
        }
        
        // 4. 透视投影到归一化平面
        Vector2d norm_point(cam_point.x() / cam_point.z(), cam_point.y() / cam_point.z());
        
        // 5. 应用畸变
        Vector2d distorted_point = applyDistortion(norm_point);
        
        // 6. 转换为像素坐标
        Vector2d pixel_point;
        pixel_point.x() = fx * distorted_point.x() + cx;
        pixel_point.y() = fy * distorted_point.y() + cy;
        
        // 7. 检查是否在图像范围内
        if (pixel_point.x() >= 0 && pixel_point.x() < width &&
            pixel_point.y() >= 0 && pixel_point.y() < height) {
            return pixel_point;
        } else {
            // 投影点在图像范围外
            return std::nullopt;
        }
    }
    
    // 计算目标在图像中的像素半径
    double calculatePixelRadius(const Vector3d& world_center, double world_radius) const {
        Vector3d relative_pos = world_center - position;
        Matrix3d R = eulerAnglesToRotationMatrix();
        Vector3d cam_point = R.transpose() * relative_pos;
        
        if (cam_point.z() <= 0) {
            return 0.0; // 目标在相机后方
        }
        
        // 在相机坐标系中，目标的像素半径约等于 (world_radius / depth) * focal_length
        double depth = cam_point.z();
        double pixel_radius = (world_radius / depth) * fx; // 使用fx作为代表焦距
        
        return std::max(pixel_radius, 5.0); // 最小半径为5像素
    }
    
    // 计算物体的实际直径（基于像素直径和距离）
    double calculateRealDiameter(double pixel_diameter, double distance_to_object) const {
        // 使用透视投影公式：pixel_diameter = (real_diameter * focal_length) / distance
        // 因此：real_diameter = (pixel_diameter * distance) / focal_length
        return (pixel_diameter * distance_to_object) / fx;
    }
        
    // 计算固定高度目标的世界坐标
    std::optional<Vector3d> pixelToWorldPosition(
        const Vector2d& pixel_point,  // 像素坐标 (u, v)
        double object_height = 0.0    // 物体高度 (相对于地面)
    ) const {
        // 1. 像素坐标转换为归一化坐标（带畸变校正）
        Vector2d norm_point;
        try {
            norm_point = pixelToNormalized(pixel_point);
        } catch (const cv::Exception& e) {
            std::cerr << "Error in distortion correction: " << e.what() << std::endl;
            return std::nullopt;
        }
        
        // 添加调试输出
        // std::cout << "像素坐标: (" << pixel_point.x() << ", " << pixel_point.y() << ")" << std::endl;
        // std::cout << "归一化坐标: (" << norm_point.x() << ", " << norm_point.y() << ")" << std::endl;
        // std::cout << "相机姿态 (roll, pitch, yaw): (" << rotation[0] << ", " << rotation[1] << ", " << rotation[2] << ")" << std::endl;
        
        // 2. 处理垂直向下特殊情况(俯仰角≈-90度)
        // if (abs(camera.rotation[1] + M_PI/2) < 1e-4) {
        //     Vector3d target_relative;
        //     target_relative.x() = norm_point.x() * (camera.position.z() - ground_height + object_height);
        //     target_relative.y() = norm_point.y() * (camera.position.z() - ground_height + object_height);
        //     target_relative.z() = -(camera.position.z() - ground_height + object_height);
        //     return camera.position + target_relative;三
        // }
        
        // 3. 对于垂直向下的相机，使用简化的变换
        // 检查是否为垂直向下的相机 (pitch ≈ -90°)
        // if (abs(rotation[1] + M_PI/2) < 0.1) {
        //     std::cout << "检测到垂直向下相机，使用简化变换" << std::endl;
            
        //     // 对于垂直向下的相机，射线变换更加直接：
        //     // 1. 归一化坐标直接对应于地面上的相对位置
        //     // 2. 考虑相机的偏航角rotation[2]来正确旋转坐标
            
        //     // 计算从相机到目标的水平距离
        //     double height_diff = position.z() - object_height;
        //     std::cout << "相机高度: " << position.z() << ", 目标高度: " << object_height << ", 高度差: " << height_diff << std::endl;
            
        //     if (height_diff <= 0) {
        //         std::cerr << "错误: 相机高度(" << position.z() << ")小于等于目标高度(" << object_height << ")" << std::endl;
        //         std::cerr << "无法进行坐标转换，相机必须在目标上方" << std::endl;
        //         return std::nullopt;
        //     }
            
        //     // 在相机坐标系中，归一化坐标对应的世界位移
        //     double world_x_offset = norm_point.x() * height_diff;
        //     double world_y_offset = norm_point.y() * height_diff;
            
        //     // 应用相机偏航角旋转
        //     double cos_yaw = cos(rotation[2]);
        //     double sin_yaw = sin(rotation[2]);
            
        //     double rotated_x = cos_yaw * world_x_offset - sin_yaw * world_y_offset;
        //     double rotated_y = sin_yaw * world_x_offset + cos_yaw * world_y_offset;
            
        //     Vector3d target_world;
        //     target_world.x() = position.x() + rotated_x;
        //     target_world.y() = position.y() + rotated_y;
        //     target_world.z() = object_height;
            
        //     std::cout << "简化计算结果: (" << target_world.x() << ", " << target_world.y() << ", " << target_world.z() << ")" << std::endl;
            
        //     return target_world;
        // }
        
        // 4. 复杂相机姿态的通用处理
        Matrix3d R = eulerAnglesToRotationMatrix();
        
        // 添加旋转矩阵调试输出
        // std::cout << "旋转矩阵 R:" << std::endl;
        // std::cout << R << std::endl;
        
        // 4. 创建归一化相机坐标系中的射线方向 (Z=1)
        Vector3d ray_cam(norm_point.x(), norm_point.y(), 1.0);
        // std::cout << "ray_cam" << std::endl;
        // std::cout << ray_cam.transpose() << std::endl;  // 横向显示
        
        // 5. 转换到世界坐标系(ENU)
        // R 将相机坐标系中的方向向量转换到世界坐标系
        // 对于向下看的相机(-90度俯仰)，相机的Z轴应该指向世界的-Z方向
        Vector3d ray_world = R * ray_cam;
        // std::cout << "变换前 ray_world:" << std::endl;
        // std::cout << ray_world.transpose() << std::endl;  // 横向显示
        
        ray_world.normalize();
        // std::cout << "归一化后 ray_world:" << std::endl;
        // std::cout << ray_world.transpose() << std::endl;  // 横向显示
        
        // 验证旋转矩阵的正交性
        Matrix3d RTR = R.transpose() * R;
        // std::cout << "R^T * R (应该接近单位矩阵):" << std::endl;
        // std::cout << RTR << std::endl;

        // 6. 计算射线到目标平面的距离参数
        double target_z = object_height;  // 目标的世界Z坐标

        // 调试输出
        // std::cout << "position.z(): " << position.z() << std::endl;
        // std::cout << "target_z: " << target_z << std::endl;
        // std::cout << "ray_world.z(): " << ray_world.z() << std::endl;
        
        if (abs(ray_world.z()) < 1e-6) {
            std::cerr << "错误: 射线与目标平面平行" << std::endl;
            return std::nullopt;
        }
        
        // 首先检查射线是否能够与目标高度平面相交
        if (abs(ray_world.z()) < 1e-6) {
            std::cerr << "警告: 射线几乎水平 (ray_z ≈ 0)，无法与目标高度平面相交" << std::endl;
            return std::nullopt;
        }
        
        // 检查射线方向与目标位置的兼容性
        if (ray_world.z() > 0 && target_z < position.z()) {
            std::cerr << "错误: 射线向上 (ray_z > 0) 但目标在相机下方 (target_z < camera_z)" << std::endl;
            std::cerr << "这种情况下射线永远不会到达目标高度" << std::endl;
            
            return std::nullopt;
        }
        double s = (target_z - position.z()) / ray_world.z();
        // std::cout << "s = (" << target_z << " - " << position.z() << ") / " << ray_world.z() << " = " << s << std::endl;
                
        // 9. 检查射线参数的有效性
        if (s < 0) {
            std::cerr << "错误: 射线参数 s < 0，表示目标在射线反方向" << std::endl;
            return std::nullopt;
        }
        
        // 10. 计算最终的世界坐标
        Vector3d target_world = position + s * ray_world;
        
        // std::cout << "最终目标世界坐标: (" << target_world.x() << ", " << target_world.y() << ", " << target_world.z() << ")" << std::endl;
        
        return target_world;
    }
    
};


class CameraGimbal : public Camera  // 公有继承自 Camera 类
{
public:
  // CameraGimbal; default; // 默认构造函数
  CameraGimbal(const std::string ardupilot_namespace, rclcpp::Node *node)
  {
    ardupilot_namespace_ = ardupilot_namespace;
    pub_ = node->create_publisher<mavros_msgs::msg::MountControl>(
    ardupilot_namespace_ + "mount_control/command", 10);
    sub_ = node->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        ardupilot_namespace_ + "mount_control/status",
        10,
        [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
            RCLCPP_INFO(rclcpp::get_logger("CameraGimbal"),
                "Received MountControl command: pitch=%.2f, roll=%.2f, yaw=%.2f",
                msg->vector.x, msg->vector.y, msg->vector.z);
            gimbal_pitch = msg->vector.x;
            gimbal_roll = msg->vector.y;
            gimbal_yaw = msg->vector.z;
            if (is_gimbal_ready) {
                rotation = Vector3d(gimbal_pitch, gimbal_roll, gimbal_yaw);
            }
        }
    );
  }

  // pitch, roll, yaw 单位为角度制
  void set_gimbal(float pitch, float roll, float yaw)
  {
    mavros_msgs::msg::MountControl msg;
    msg.mode = 2; // MAV_MOUNT_MODE_MAVLINK_TARGETING
    msg.pitch = pitch;
    msg.roll = roll;
    msg.yaw = yaw;
    pub_->publish(msg);
  }
  void uset_gimbal(bool is_gimbal_ready)
  {
    this->is_gimbal_ready = is_gimbal_ready;
  }

  double get_gimbal_pitch() const { return gimbal_pitch; }
  double get_gimbal_roll() const { return gimbal_roll; }
  double get_gimbal_yaw() const { return gimbal_yaw; }

private:
  std::string ardupilot_namespace_;
  rclcpp::Publisher<mavros_msgs::msg::MountControl>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_;
  bool is_gimbal_ready = false; // 是否准备好接收命令
  double gimbal_pitch, gimbal_roll, gimbal_yaw;

};
