#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/mount_control.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

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