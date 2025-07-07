#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/mount_control.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include "math.h"
#include <opencv2/opencv.hpp>

// ros2 topic pub /mavros/mount_control/command mavros_msgs/msg/MountControl "{mode: 2, pitch: -90.0, roll: 0.0, yaw: 0.0}"

class CameraGimbal
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
        pitch_ = msg->vector.x;
        roll_ = msg->vector.y;
        yaw_ = msg->vector.z;
      });
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

  double get_pitch() const { return pitch_; }
  double get_roll() const { return roll_; }
  double get_yaw() const { return yaw_; }

private:
  std::string ardupilot_namespace_;
  rclcpp::Publisher<mavros_msgs::msg::MountControl>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_;

  double pitch_, roll_, yaw_;
};

struct CameraParams {
    Vector3d position;    // 相机位置 (tx, ty, tz)
    Vector3d rotation;    // 欧拉角 (roll, pitch, yaw) 弧度
    double fx, fy; // 相机焦距 (fx, fy)
    double cx, cy; // 相机主点 (cx, cy)
    double k1 = 0.0, k2 = 0.0, p1 = 0.0, p2 = 0.0, k3 = 0.0; // 畸变系数 (k1, k2, p1, p2, k3)
    double width, height; // 图像宽度和高度
};

// 创建旋转矩阵从欧拉角 (roll, pitch, yaw)
// 相机坐标系：X右, Y下, Z前
// 世界坐标系(ENU)：X东, Y北, Z上
inline Matrix3d eulerAnglesToRotationMatrix(const Vector3d& theta) {
    double roll = theta[0];   // 移除 +M_PI，这个偏移是不正确的
    double pitch = theta[1]; 
    double yaw = theta[2];
    
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

// 像素坐标转换为归一化坐标（带畸变校正）
inline Vector2d pixelToNormalized(const Vector2d& pixel, const CameraParams& camera) {
    // 创建OpenCV需要的相机矩阵和畸变系数
    cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 
        camera.fx, 0, camera.cx,
        0, camera.fy, camera.cy,
        0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<double>(1,5) << 
        camera.k1, camera.k2, camera.p1, camera.p2, camera.k3);
    // 输入点准备
    cv::Point2f src_pt(pixel.x(), pixel.y());
    std::vector<cv::Point2f> src = {src_pt}, dst;
    
    // 去畸变
    cv::undistortPoints(src, dst, cameraMatrix, distCoeffs);
    
    return Vector2d(dst[0].x, dst[0].y);
}

// 计算目标世界坐标
inline std::optional<Vector3d> calculateWorldPosition(
    const Vector2d& pixel_point,  // 像素坐标 (u, v)
    const CameraParams& camera,   // 相机参数
    double ground_height = 0.0,   // 地平面高度
    double object_height = 0.0    // 物体高度 (相对于地面)
) {
    // 1. 像素坐标转换为归一化坐标（带畸变校正）
    Vector2d norm_point;
    try {
        norm_point = pixelToNormalized(pixel_point, camera);
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
    if (abs(camera.rotation[1] + M_PI/2) < 0.1) {
        // std::cout << "检测到垂直向下相机，使用简化变换" << std::endl;
        
        // 对于垂直向下的相机，射线变换非常简单：
        // 相机坐标(x, y, 1) -> 世界坐标(x, y, -1)（仅翻转Z轴）
        Vector3d ray_world(norm_point.x(), norm_point.y(), -1.0);
        ray_world.normalize();
        
        // std::cout << "简化射线方向: (" << ray_world.x() << "," << ray_world.y() << "," << ray_world.z() << ")" << std::endl;
        
        // 计算交点
        double target_z = ground_height + object_height;
        if (ray_world.z() >= -0.1) {
            std::cerr << "错误: 射线不向下" << std::endl;
            return std::nullopt;
        }
        
        double s = (target_z - camera.position.z()) / ray_world.z();
        if (s <= 0) {
            std::cerr << "错误: 交点在相机后方, s = " << s << std::endl;
            return std::nullopt;
        }
        
        Vector3d target_relative = s * ray_world;
        Vector3d target_world = camera.position + target_relative;
        
        return target_world;
    }
    
    // 4. 复杂相机姿态的通用处理
    Matrix3d R = eulerAnglesToRotationMatrix(camera.rotation);
    
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
    double target_z = ground_height + object_height;  // 目标的世界Z坐标
    
    // 调试输出
    // std::cout << "camera.position.z(): " << camera.position.z() << std::endl;
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
    
    double s = (target_z - camera.position.z()) / ray_world.z();
    // std::cout << "s = (" << target_z << " - " << camera.position.z() << ") / " << ray_world.z() << " = " << s << std::endl;
    
    // 检查射线方向与目标位置的兼容性
    if (ray_world.z() > 0 && target_z < camera.position.z()) {
        std::cerr << "错误: 射线向上 (ray_z > 0) 但目标在相机下方 (target_z < camera_z)" << std::endl;
        std::cerr << "这种情况下射线永远不会到达目标高度" << std::endl;
        return std::nullopt;
    }
    
    if (ray_world.z() < 0 && target_z > camera.position.z()) {
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
    Vector3d target_world = camera.position + target_relative;
    
    return target_world;
}
