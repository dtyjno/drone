#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/mount_control.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include "math.h"

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
    double focal_length;  // 焦距 (归一化坐标中可设为1)
};

// 创建旋转矩阵从欧拉角 (roll, pitch, yaw)
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
    
    return R_z * R_y * R_x;
}

// 计算目标世界坐标
inline std::optional<Vector3d> calculateWorldPosition(
    const Vector2d& image_point,  // 归一化图像坐标 (x, y)
    const CameraParams& camera,   // 相机参数
    double ground_height = 0.0,   // 地平面高度
    double object_height = 0.0    // 物体高度 (相对于地面)
) {
    // 1. 创建相机外参矩阵
    Matrix3d R = eulerAnglesToRotationMatrix(camera.rotation);
    Vector3d t = camera.position;
    
    // 2. 创建相机到世界的变换矩阵
    Matrix4d T_cw = Matrix4d::Identity();
    T_cw.block<3,3>(0,0) = R.transpose();
    T_cw.block<3,1>(0,3) = -R.transpose() * t;
    
    // 3. 创建归一化相机坐标 (Z=1)
    Vector3d p_cam(image_point.x(), image_point.y(), 1);
    
    // 4. 转换到世界坐标系的射线方向
    Vector4d p_cam_homog(p_cam.x(), p_cam.y(), p_cam.z(), 1);
    Vector4d ray_dir_homog = T_cw * p_cam_homog;
    Vector3d ray_dir = ray_dir_homog.head<3>().normalized();
    
    // 5. 计算射线与地平面 (y = ground_height - object_height) 的交点
    if (abs(ray_dir.y()) < 1e-6) {
        std::cerr << "Error: Ray is parallel to the ground plane." << std::endl;
        return std::nullopt;
    }
    
    double s = (ground_height - object_height - camera.position.y()) / ray_dir.y();
    if (s <= 0) {
        std::cerr << "Error: Intersection point is behind the camera." << std::endl;
        return std::nullopt;
    }
    
    Vector3d target_pos = camera.position + s * ray_dir;
    
    return target_pos;
}
