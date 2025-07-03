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
// 注意：这里的旋转矩阵是从世界坐标系到相机坐标系的变换
inline Matrix3d eulerAnglesToRotationMatrix(const Vector3d& theta) {
    Matrix3d R_x; // 绕X轴旋转
    R_x << 1, 0, 0,
           0, cos(theta[0]), -sin(theta[0]),
           0, sin(theta[0]), cos(theta[0]);
    
    Matrix3d R_y; // 绕Y轴旋转
    R_y << cos(theta[1]), 0, sin(theta[1]),
           0, 1, 0,
           -sin(theta[1]), 0, cos(theta[1]);
    
    Matrix3d R_z; // 绕Z轴旋转
    R_z << cos(theta[2]), -sin(theta[2]), 0,
           sin(theta[2]), cos(theta[2]), 0,
           0, 0, 1;
    
    return R_z * R_y * R_x; // 旋转顺序: Z-Y-X (yaw-pitch-roll)
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
    
    // // 2. 处理垂直向下特殊情况(俯仰角≈-90度)
    // if (abs(camera.rotation[1] + M_PI/2) < 1e-4) {
    //     Vector3d target_pos;
    //     target_pos.x() = camera.position.x() + norm_point.x() * (camera.position.z() - ground_height);
    //     target_pos.y() = camera.position.y() + norm_point.y() * (camera.position.z() - ground_height);
    //     target_pos.z() = ground_height - object_height;
    //     return target_pos;
    // }
    
    // 3. 创建相机外参矩阵
    Matrix3d R = eulerAnglesToRotationMatrix(camera.rotation);
    
    // 4. 创建归一化相机坐标系中的射线方向 (Z=1)
    Vector3d ray_cam(norm_point.x(), norm_point.y(), 1.0);
    std::cout << "ray_cam" << std::endl;
    std::cout << ray_cam << std::endl;
    
    // 5. 转换到世界坐标系(ENU)
    // R^T 将相机坐标系中的方向向量转换到世界坐标系
    Vector3d ray_world = R.transpose() * ray_cam;
    ray_world.normalize();
    std::cout << "ray_world" << std::endl;
    std::cout << ray_world << std::endl;

    // 6. 计算与地平面的交点 (Z = ground_height - object_height)
    if (abs(ray_world.z()) < 1e-6) {
        std::cerr << "错误: 射线与地平面平行" << std::endl;
        return std::nullopt;
    }
    
    double s = (ground_height - object_height - camera.position.z()) / ray_world.z();
    if (s <= 0) {
        std::cerr << "错误: 交点在相机后方, s = " << s << std::endl;
        return std::nullopt;
    }
    
    return camera.position + s * ray_world;
}
