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
    Vector3d camera_relative_position; // 相机相对于飞机的位置 (tx, ty, tz)
    Vector3d camera_relative_rotation; // 相机在世界坐标系中的旋转 (roll, pitch, yaw)
    Vector3d parent_position; // 父节点的位置 (tx, ty, tz)
    Vector3d parent_rotation; // 欧拉角 (roll, pitch, yaw)
    double fx, fy; // 相机焦距 (fx, fy)
    double cx, cy; // 相机主点 (cx, cy)
    double k1 = 0.0, k2 = 0.0, p1 = 0.0, p2 = 0.0, k3 = 0.0; // 畸变系数 (k1, k2, p1, p2, k3)
    double width, height; // 图像宽度和高度

    // ENU到NED转换矩阵
    const Matrix3d ENU2NED = (Matrix3d() << 
        0, 1, 0,
        1, 0, 0,
        0, 0, -1).finished();

    const Matrix3d NED2ENU = ENU2NED.transpose();

    const Matrix3d ESD2NED = (Matrix3d() << 
        0, -1, 0,
        1, 0, 0,
        0, 0, 1).finished();

    const Matrix3d NED2ESD = ESD2NED.transpose();

    const Matrix3d ESD2ENU = (Matrix3d() << 
        1, 0, 0,
        0, -1, 0,
        0, 0, -1).finished();

    const Matrix3d ENU2ESD = ESD2ENU.transpose();

    Vector3d get_position() const {
        return camera_relative_position + parent_position;
    }
    // Vector3d get_rotation() const {
    //     return camera_relative_rotation;
    // }

    Camera()
        : camera_relative_position(Vector3d::Zero()),
          camera_relative_rotation(Vector3d::Zero()),
          parent_position(Vector3d::Zero()),
          parent_rotation(Vector3d::Zero()),
          fx(1.0), fy(1.0), cx(0.0), cy(0.0), width(640), height(480) {
        read_configs("camera.yaml");
    }
    Camera(const Vector3d& pos, const Vector3d& rot, double fx, double fy, double cx, double cy, double width, double height)
        : camera_relative_position(Vector3d::Zero()),
          camera_relative_rotation(Vector3d::Zero()),
          parent_position(pos),
          parent_rotation(rot),
          fx(fx), fy(fy), cx(cx), cy(cy), width(width), height(height) {
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

    // 生成旋转矩阵，处理对应坐标系下旋转角的输入
    Matrix3d eulerAnglesToRotationMatrixWorldToCamera() const {
        // 正确分配欧拉角
        double roll_c  = camera_relative_rotation[0]; // X轴
        double pitch_c = camera_relative_rotation[1]; // Y轴
        double yaw_c   = camera_relative_rotation[2]; // Z轴
        
        double roll_p  = parent_rotation[0];          // X轴
        double pitch_p = parent_rotation[1];          // Y轴
        double yaw_p   = parent_rotation[2];          // Z轴

        // 相机旋转矩阵 (Z-Y-X顺序: yaw->pitch->roll)
        Matrix3d R_roll_c; // X轴
        R_roll_c << 1, 0, 0,
                    0, cos(roll_c), -sin(roll_c),
                    0, sin(roll_c), cos(roll_c);
        
        Matrix3d R_pitch_c; // Y轴
        R_pitch_c << cos(pitch_c), 0, sin(pitch_c),
                    0, 1, 0,
                    -sin(pitch_c), 0, cos(pitch_c);
        
        Matrix3d R_yaw_c; // Z轴
        R_yaw_c << cos(yaw_c), -sin(yaw_c), 0,
                sin(yaw_c), cos(yaw_c), 0,
                0, 0, 1;
        
        // 父级旋转矩阵 (同样Z-Y-X顺序)
        Matrix3d R_roll_p; // X轴
        R_roll_p << 1, 0, 0,
                    0, cos(roll_p), -sin(roll_p),
                    0, sin(roll_p), cos(roll_p);
        
        Matrix3d R_pitch_p; // Y轴
        R_pitch_p << cos(pitch_p), 0, sin(pitch_p),
                    0, 1, 0,
                    -sin(pitch_p), 0, cos(pitch_p);
        
        Matrix3d R_yaw_p; // Z轴
        R_yaw_p << cos(yaw_p), sin(yaw_p), 0,
                -sin(yaw_p), cos(yaw_p), 0,
                0, 0, 1;

        // std::cout << "相机偏航角: " << yaw_c << ", 俯仰角: " << pitch_c << ", 翻滚角: " << roll_c << std::endl;
        // std::cout << "父级偏航角: " << yaw_p << ", 俯仰角: " << pitch_p << ", 翻滚角: " << roll_p << std::endl;
        // std::cout << "R_pitch_p:\n" << R_pitch_p.transpose() << std::endl;
        // std::cout << "R_yaw_p:\n" << R_yaw_p.transpose() << std::endl;
        // std::cout << "R_roll_p:\n" << R_roll_p.transpose() << std::endl;

        // 构建旋转矩阵 (Z-Y-X顺序)
        Matrix3d R_c = R_yaw_c * R_pitch_c * R_roll_c; // 相机旋转
        Matrix3d R_p = R_yaw_p * R_pitch_p * R_roll_p; // 父级旋转

        // ENU世界到相机坐标系旋转
        Matrix3d R_total = R_p * R_c;

        // 调试输出
        // std::cout << "相机旋转矩阵:\n" << R_c << "\n";
        // std::cout << "父级旋转矩阵:\n" << R_p << "\n";
        // std::cout << "最终旋转矩阵:\n" << R_total << "\n";
        
        return R_total;
    }

    // 世界坐标转换为像素坐标
    std::optional<Vector2d> worldToPixel(const Vector3d& world_point) const {
        // 1. 世界坐标转换为相机坐标
        Vector3d relative_pos = world_point - get_position();
        // std::cout << "相机相对位置: " << relative_pos.transpose() << std::endl;
        
        // 2. 应用相机旋转矩阵的逆变换 (世界坐标 -> 相机坐标)
        Matrix3d R = eulerAnglesToRotationMatrixWorldToCamera();
        // std::cout << "相机旋转矩阵:\n" << R << "\n";
        Vector3d cam_point = ENU2ESD * R * relative_pos;

        // std::cout << "变换后的相机坐标: " << cam_point.transpose() << std::endl;

        // 3. 检查点是否在相机前方
        if (cam_point.z() <= 0) {
            // 点在相机后方，无法投影
            std::cout << "点在相机后方，无法投影" << cam_point.x() << ", " << cam_point.y() << ", " << cam_point.z() << std::endl;
            return std::nullopt;
        }
        
        // 4. 透视投影到归一化平面
        Vector2d norm_point(cam_point.x() / cam_point.z(), cam_point.y() / cam_point.z());
        
        // 5. 应用畸变
        Vector2d distorted_point = applyDistortion(norm_point);
        // std::cout << "归一化坐标: " << norm_point.transpose() << ", 畸变后: " << distorted_point.transpose() << std::endl;
        
        // 6. 转换为像素坐标
        Vector2d pixel_point;
        pixel_point.x() = fx * distorted_point.x() + cx;
        pixel_point.y() = fy * distorted_point.y() + cy;
        
        // 7. 检查是否在图像范围内
        if (pixel_point.x() >= 0 && pixel_point.x() < width &&
            pixel_point.y() >= 0 && pixel_point.y() < height) {
            // std::cout << "像素坐标: " << pixel_point.transpose() << std::endl;
            return pixel_point;
        } else {
            // 投影点在图像范围外
            return std::nullopt;
        }
    }
    
    // 计算目标在图像中的像素半径
    double calculatePixelRadius(const Vector3d& world_center, double world_radius) const {
        Vector3d relative_pos = world_center - get_position();
        Matrix3d R = eulerAnglesToRotationMatrixWorldToCamera();
        Vector3d cam_point = ENU2ESD * R * relative_pos;
        // 检查点是否在相机前方
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
        if (fx <= 0.01) { // 防止除零
            return 0.0;
        }
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
        Matrix3d R = eulerAnglesToRotationMatrixWorldToCamera();
        
        // 添加旋转矩阵调试输出
        // std::cout << "旋转矩阵 R:" << std::endl;
        // std::cout << R << std::endl;
        
        // 4. 创建归一化相机坐标系中的射线方向 (Z=1)
        Vector3d ray_cam(norm_point.x(), norm_point.y(), 1.0);
        ray_cam = ESD2ENU * ray_cam; // 转换到ENU坐标系
        // std::cout << "ray_cam ENU" << std::endl;
        // std::cout << ray_cam.transpose() << std::endl;  // 横向显示
        
        // 5. 转换到世界坐标系(ENU)
        // R 将相机坐标系中的方向向量转换到世界坐标系
        // 对于向下看的相机(-90度俯仰)，相机的Z轴应该指向世界的-Z方向

        // std::cout << "R.transpose():" << std::endl;
        // std::cout << R.transpose() << std::endl;  // 横向显示
        Vector3d ray_world = R.transpose() * ray_cam;
        // std::cout << "ray_world ENU:" << std::endl;
        // std::cout << ray_world.transpose() << std::endl;  // 横向显示
        ray_world.normalize();
        // std::cout << "归一化后 ray_world:" << std::endl;
        // std::cout << ray_world.transpose() << std::endl;  // 横向显示
        
        // 6. 计算射线到目标平面的距离参数
        double target_z = object_height;  // 目标的世界Z坐标

        // 调试输出
        // std::cout << "position.z(): " << get_position().z() << std::endl;
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
        if (ray_world.z() > 0 && target_z < get_position().z()) {
            std::cerr << "错误: 射线向上 (ray_z > 0) 但目标在相机下方 (target_z < camera_z)" << std::endl;
            std::cerr << "这种情况下射线永远不会到达目标高度" << std::endl;
            
            return std::nullopt;
        }
        double s = (target_z - get_position().z()) / ray_world.z();
        // std::cout << "s = (" << target_z << " - " << get_position().z() << ") / " << ray_world.z() << " = " << s << std::endl;
                
        // 9. 检查射线参数的有效性
        if (s < 0) {
            std::cerr << "错误: 射线参数 s < 0，表示目标在射线反方向" << std::endl;
            return std::nullopt;
        }
        
        // 10. 计算最终的世界坐标
        Vector3d target_world = get_position() + s * ray_world;
        
        // std::cout << "最终目标世界坐标: (" << target_world.x() << ", " << target_world.y() << ", " << target_world.z() << ")" << std::endl;
        
        return target_world;
    }
    
    // 垂直向下相机的简化世界坐标转像素坐标（用于高效计算）
    std::optional<Vector2d> worldToPixelVerticalDown(const Vector3d& world_point) const {
        // 检查是否为垂直向下的相机 (pitch ≈ -90°)
        // if (abs(rotation[1] + M_PI/2) > 0.1) {
        //     // 不是垂直向下相机，使用通用方法
        //     return worldToPixel(world_point);
        // }
        
        // 简化计算：垂直向下相机
        double height_diff = get_position().z() - world_point.z();
        if (height_diff <= 0) {
            return std::nullopt; // 目标在相机上方或同一水平面
        }
        
        // 计算相对位置
        double dx = world_point.x() - get_position().x();
        double dy = world_point.y() - get_position().y();
        
        // 应用偏航角旋转（逆向旋转到相机坐标系）
        double cos_yaw = cos(-(camera_relative_rotation[2] + parent_rotation[2])); // 注意负号
        double sin_yaw = sin(-(camera_relative_rotation[2] + parent_rotation[2]));

        double cam_x = cos_yaw * dx - sin_yaw * dy;
        double cam_y = sin_yaw * dx + cos_yaw * dy;
        
        // 归一化坐标
        Vector2d norm_point(cam_x / height_diff, cam_y / height_diff);
        
        // 应用畸变
        Vector2d distorted_point = applyDistortion(norm_point);
        
        // 转换为像素坐标
        Vector2d pixel_point;
        pixel_point.x() = fx * distorted_point.x() + cx;
        pixel_point.y() = fy * distorted_point.y() + cy;
        
        // 检查是否在图像范围内
        if (pixel_point.x() >= 0 && pixel_point.x() < width &&
            pixel_point.y() >= 0 && pixel_point.y() < height) {
            return pixel_point;
        } else {
            return std::nullopt;
        }
    }
    
    // 相机间坐标映射：将输入相机的像素坐标映射到垂直向下相机的像素坐标
    std::optional<Vector2d> mapPixelToVerticalDownCamera(
        const Vector2d& input_pixel,           // 输入相机的像素坐标
        double target_height = 0.0             // 目标所在的世界高度
    ) const {
        // 步骤1：将输入相机的像素坐标转换为世界坐标
        auto world_pos = pixelToWorldPosition(input_pixel, target_height);
        // std::cout << "输入像素坐标: (" << input_pixel.x() << ", " << input_pixel.y() << ")" << std::endl;
        // std::cout << "转换后的世界坐标: (" << world_pos->x() << ", " << world_pos->y() << ", " << world_pos->z() << ")" << std::endl;
        if (!world_pos.has_value()) {
            return std::nullopt; // 转换失败
        }
        
        // 步骤2：将世界坐标投影到垂直向下相机的像素坐标
        return worldToPixelVerticalDown(world_pos.value());
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
            // use_gimbal(true);
            gimbal_pitch = msg->vector.x;
            gimbal_roll = msg->vector.y;
            gimbal_yaw = msg->vector.z;
            if (is_gimbal_ready) {
                camera_relative_rotation = Vector3d(gimbal_pitch, gimbal_roll, gimbal_yaw);
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
  void use_gimbal(bool is_gimbal_ready)
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
