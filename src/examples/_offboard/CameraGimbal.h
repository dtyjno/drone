#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/mount_control.hpp>

class CameraGimbal
{
public:
  CameraGimbal(rclcpp::Node *node)
  {
    pub_ = node->create_publisher<mavros_msgs::msg::MountControl>(
      "/mavros/mount_control/command", 10);
  }

  // pitch, roll, yaw 单位为度
  void set_gimbal(float pitch, float roll, float yaw)
  {
    mavros_msgs::msg::MountControl msg;
    msg.mode = 2; // MAV_MOUNT_MODE_MAVLINK_TARGETING
    msg.pitch = pitch;
    msg.roll = roll;
    msg.yaw = yaw;
    pub_->publish(msg);
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::MountControl>::SharedPtr pub_;
};