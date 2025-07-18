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
        
        // 使用 Z-Y-X 顺序 (yaw-pitch-roll) 来正确处理相机到世界的变换
        // 这样俯仰角-90度会正确地将相机Z轴(前方)指向世界-Z方向(下方)
        return R_z * R_y * R_x; // 注意顺序：先绕Z轴，再Y轴，最后X轴
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
        // if (abs(camera.rotation[1] + M_PI/2) < 0.1) {
        //     // std::cout << "检测到垂直向下相机，使用简化变换" << std::endl;
            
        //     // 对于垂直向下的相机，射线变换非常简单：
        //     // 相机坐标(x, y, 1) -> 世界坐标(x, y, -1)（仅翻转Z轴）
        //     Vector3d ray_world(norm_point.x(), norm_point.y(), -1.0);
        //     ray_world.normalize();
            
        //     // std::cout << "简化射线方向: (" << ray_world.x() << "," << ray_world.y() << "," << ray_world.z() << ")" << std::endl;
            
        //     // 计算交点
        //     double target_z = ground_height + object_height;
        //     if (ray_world.z() >= -0.1) {
        //         std::cerr << "错误: 射线不向下" << std::endl;
        //         return std::nullopt;
        //     }
            
        //     double s = (target_z - camera.position.z()) / ray_world.z();
        //     if (s <= 0) {
        //         std::cerr << "错误: 交点在相机后方, s = " << s << std::endl;
        //         return std::nullopt;
        //     }
            
        //     Vector3d target_relative = s * ray_world;
        //     Vector3d target_world = camera.position + target_relative;
            
        //     return target_world;
        // }
        
        // 4. 复杂相机姿态的通用处理
        Matrix3d R = eulerAnglesToRotationMatrix();
        
        // 4. 创建归一化相机坐标系中的射线方向 (Z=1)
        Vector3d ray_cam(norm_point.x(), norm_point.y(), 1.0);
        // std::cout << "ray_cam" << std::endl;
        // std::cout << ray_cam << std::endl;
        
        // 5. 转换到世界坐标系(ENU)
        // R 将相机坐标系中的方向向量转换到世界坐标系
        // 对于向下看的相机(-90度俯仰)，相机的Z轴应该指向世界的-Z方向
        Vector3d ray_world = R * ray_cam;
        ray_world.normalize();
        // std::cout << "ray_world" << std::endl;
        // std::cout << ray_world << std::endl;

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
        
        double s = (target_z - position.z()) / ray_world.z();
        // std::cout << "s = (" << target_z << " - " << position.z() << ") / " << ray_world.z() << " = " << s << std::endl;
        
        // 检查射线方向与目标位置的兼容性
        if (ray_world.z() > 0 && target_z < position.z()) {
            std::cerr << "错误: 射线向上 (ray_z > 0) 但目标在相机下方 (target_z < camera_z)" << std::endl;
            std::cerr << "这种情况下射线永远不会到达目标高度" << std::endl;
            
            return std::nullopt;
        }
        
        if (ray_world.z() < 0 && target_z > position.z()) {
            std::cerr << "错误: 射线向下 (ray_z < 0) 但目标在相机上方 (target_z > camera_z)" << std::endl;
            std::cerr << "这种情况下射线永远不会到达目标高度" << std::endl;
            return std::nullopt;
        }
        
        // 检查交点是否在射线前方
        if (s <= 0) {
            std::cerr << "警告: 计算的交点在相机后方或相机位置, s = " << s << std::endl;
            return std::nullopt;
        }
        
        // 7. 计算目标相对于相机的位置向量
        Vector3d target_relative = s * ray_world;
        // std::cout << "target_relative" << std::endl;
        // std::cout << target_relative << std::endl;
        
        // 8. 目标的世界坐标 = 相机世界坐标 + 目标相对坐标
        Vector3d target_world = position + target_relative;
        
        return target_world;
    }
    
    // 垂直位移分析结果结构体
    struct VerticalShiftResult {
        Vector2d original_pixel;      // 原始像素坐标
        Vector2d new_pixel;          // 抬升后的像素坐标
        Vector2d pixel_displacement; // 像素位移量 (dx, dy)
        Vector3d original_world;     // 原始世界坐标
        Vector3d new_world;          // 抬升后的世界坐标
        double height_change;        // 高度变化量
        bool is_valid;              // 是否计算成功
        std::string error_message;   // 错误信息
    };
    
    // 计算图像点垂直抬升后的像素位置变化
    VerticalShiftResult calculatePixelVerticalShift(
        const Vector2d& pixel_point,    // 输入的图像像素坐标
        double original_height,         // 原始高度（相对于地面）
        double height_lift             // 垂直抬升高度（米）
    ) const {
        VerticalShiftResult result;
        result.original_pixel = pixel_point;
        result.height_change = height_lift;
        result.is_valid = false;
        
        // 1. 计算原始像素点对应的世界坐标
        auto original_world_opt = pixelToWorldPosition(pixel_point, original_height);
        if (!original_world_opt.has_value()) {
            result.error_message = "无法计算原始像素点的世界坐标";
            return result;
        }
        result.original_world = original_world_opt.value();
        
        // 2. 计算抬升后的世界坐标（仅Z坐标变化）
        result.new_world = result.original_world;
        result.new_world.z() += height_lift;  // 垂直抬升
        
        // 3. 将抬升后的世界坐标转换回像素坐标
        auto new_pixel_opt = worldToPixel(result.new_world);
        if (!new_pixel_opt.has_value()) {
            result.error_message = "抬升后的点不在相机视野内";
            return result;
        }
        result.new_pixel = new_pixel_opt.value();
        
        // 4. 计算像素位移
        result.pixel_displacement = result.new_pixel - result.original_pixel;
        result.is_valid = true;
        
        return result;
    }
    
    // 批量计算多个高度抬升的像素位置变化
    std::vector<VerticalShiftResult> calculateMultipleVerticalShifts(
        const Vector2d& pixel_point,              // 输入的图像像素坐标
        double original_height,                   // 原始高度
        const std::vector<double>& height_lifts   // 多个抬升高度
    ) const {
        std::vector<VerticalShiftResult> results;
        
        for (double lift : height_lifts) {
            results.push_back(calculatePixelVerticalShift(pixel_point, original_height, lift));
        }
        
        return results;
    }
    
    // 计算垂直抬升轨迹（连续高度变化）
    std::vector<VerticalShiftResult> calculateVerticalTrajectory(
        const Vector2d& pixel_point,    // 输入的图像像素坐标
        double original_height,         // 原始高度
        double max_lift,               // 最大抬升高度
        double step_size = 0.5         // 高度步长（米）
    ) const {
        std::vector<VerticalShiftResult> trajectory;
        
        for (double lift = 0.0; lift <= max_lift; lift += step_size) {
            trajectory.push_back(calculatePixelVerticalShift(pixel_point, original_height, lift));
        }
        
        return trajectory;
    }
    
    // 示例使用函数：垂直位移分析
    void verticalShiftAnalysisExample() const {
        std::cout << "\n=== 垂直位移分析示例 ===" << std::endl;
        
        // 示例像素点（图像中心附近）
        Vector2d test_pixel(320, 240);
        double original_height = 0.0;  // 地面高度
        
        std::cout << "原始像素坐标: (" << test_pixel.x() << ", " << test_pixel.y() << ")" << std::endl;
        std::cout << "原始高度: " << original_height << "m" << std::endl;
        
        // 1. 单次抬升分析
        std::cout << "\n--- 单次抬升分析 ---" << std::endl;
        double lift_height = 5.0;  // 抬升5米
        auto result = calculatePixelVerticalShift(test_pixel, original_height, lift_height);
        
        if (result.is_valid) {
            std::cout << "抬升 " << lift_height << "m 后:" << std::endl;
            std::cout << "  新像素坐标: (" << result.new_pixel.x() << ", " << result.new_pixel.y() << ")" << std::endl;
            std::cout << "  像素位移: (" << result.pixel_displacement.x() << ", " << result.pixel_displacement.y() << ")" << std::endl;
            std::cout << "  位移幅度: " << result.pixel_displacement.norm() << " 像素" << std::endl;
        } else {
            std::cout << "计算失败: " << result.error_message << std::endl;
        }
        
        // 2. 多高度分析
        std::cout << "\n--- 多高度分析 ---" << std::endl;
        std::vector<double> test_heights = {1.0, 2.0, 3.0, 5.0, 10.0};
        auto multi_results = calculateMultipleVerticalShifts(test_pixel, original_height, test_heights);
        
        for (const auto& res : multi_results) {
            if (res.is_valid) {
                std::cout << "抬升 " << res.height_change << "m: "
                         << "像素位移(" << res.pixel_displacement.x() << ", " << res.pixel_displacement.y() << "), "
                         << "幅度 " << res.pixel_displacement.norm() << " px" << std::endl;
            } else {
                std::cout << "抬升 " << res.height_change << "m: 失败 - " << res.error_message << std::endl;
            }
        }
        
        // 3. 轨迹分析
        std::cout << "\n--- 垂直轨迹分析 ---" << std::endl;
        auto trajectory = calculateVerticalTrajectory(test_pixel, original_height, 10.0, 1.0);
        
        std::cout << "高度(m)\t像素X\t像素Y\t位移X\t位移Y\t位移幅度" << std::endl;
        for (const auto& point : trajectory) {
            if (point.is_valid) {
                std::cout << point.height_change << "\t"
                         << std::fixed << std::setprecision(1)
                         << point.new_pixel.x() << "\t"
                         << point.new_pixel.y() << "\t"
                         << point.pixel_displacement.x() << "\t"
                         << point.pixel_displacement.y() << "\t"
                         << point.pixel_displacement.norm() << std::endl;
            }
        }
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

// 工具函数：批量处理世界坐标到像素坐标的转换
struct ProjectedTarget {
    int id;
    std::string category;
    Vector2d pixel_center;
    double pixel_radius;
    Vector3d world_position;
    double world_radius;
    bool is_visible;
};

// 示例使用函数
inline void exampleUsage() {
    // 创建相机参数
    Camera camera;
    camera.position = Vector3d(0, 0, 10);  // 相机在10米高度
    camera.rotation = Vector3d(0, -M_PI/2, 0);  // 垂直向下
    camera.fx = camera.fy = 500;  // 焦距
    camera.cx = 320; camera.cy = 240;  // 图像中心
    camera.width = 640; camera.height = 480;
    
    // 示例：世界坐标点
    Vector3d world_point(5, 3, 0);  // 地面上的一个点
    
    // 转换为像素坐标
    auto pixel_result = camera.worldToPixel(world_point);
    if (pixel_result.has_value()) {
        Vector2d pixel = pixel_result.value();
        std::cout << "世界坐标 (" << world_point.transpose() << ") "
                  << "对应像素坐标 (" << pixel.transpose() << ")" << std::endl;
    } else {
        std::cout << "世界坐标点不在相机视野内" << std::endl;
    }
    
    // 计算目标的像素半径
    double world_radius = 0.5;  // 0.5米半径
    double pixel_radius = camera.calculatePixelRadius(world_point, world_radius);
    std::cout << "世界半径 " << world_radius << "m 对应像素半径 " << pixel_radius << "px" << std::endl;
    
    // 垂直位移分析示例
    camera.verticalShiftAnalysisExample();
    
    // 单点垂直位移测试
    std::cout << "\n=== 单点垂直位移测试 ===" << std::endl;
    Vector2d test_pixel(400, 300);  // 测试像素点
    double original_height = 0.0;   // 地面高度
    double lift_height = 3.0;       // 抬升3米
    
    auto shift_result = camera.calculatePixelVerticalShift(test_pixel, original_height, lift_height);
    if (shift_result.is_valid) {
        std::cout << "像素点 (" << test_pixel.x() << ", " << test_pixel.y() << ") 垂直抬升 " << lift_height << "m:" << std::endl;
        std::cout << "  原始世界坐标: (" << shift_result.original_world.transpose() << ")" << std::endl;
        std::cout << "  抬升后世界坐标: (" << shift_result.new_world.transpose() << ")" << std::endl;
        std::cout << "  新像素坐标: (" << shift_result.new_pixel.x() << ", " << shift_result.new_pixel.y() << ")" << std::endl;
        std::cout << "  像素位移: (" << shift_result.pixel_displacement.x() << ", " << shift_result.pixel_displacement.y() << ")" << std::endl;
        std::cout << "  位移距离: " << shift_result.pixel_displacement.norm() << " 像素" << std::endl;
    } else {
        std::cout << "计算失败: " << shift_result.error_message << std::endl;
    }
}
