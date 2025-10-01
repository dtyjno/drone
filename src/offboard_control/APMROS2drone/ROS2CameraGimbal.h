#pragma once

#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/mount_control.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include "../drone/CameraGimbal.h"

class ROS2CameraGimbal : public CameraGimbal  // 公有继承自 CameraGimbal 类
{
public:
  // CameraGimbal; default; // 默认构造函数
  ROS2CameraGimbal(const std::string ardupilot_namespace, rclcpp::Node::SharedPtr node)
  {
    ardupilot_namespace_ = ardupilot_namespace;
    if (!ardupilot_namespace_.empty() && ardupilot_namespace_.back() != '/')
        ardupilot_namespace_ += '/';
    pub_ = node->create_publisher<mavros_msgs::msg::MountControl>(
        ardupilot_namespace_ + "mount_control/command", 10);
    sub_ = node->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        ardupilot_namespace_ + "mount_control/status",
        10,
        [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
            RCLCPP_INFO(rclcpp::get_logger("ROS2CameraGimbal"),
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
    if (!pub_) {
        RCLCPP_ERROR(rclcpp::get_logger("ROS2CameraGimbal"), "Publisher not initialized!");
        return;
    }
    mavros_msgs::msg::MountControl msg;
    msg.mode = 2; // MAV_MOUNT_MODE_MAVLINK_TARGETING
    msg.pitch = pitch;
    msg.roll = roll;
    msg.yaw = yaw;
    RCLCPP_INFO(rclcpp::get_logger("ROS2CameraGimbal"), "Publishing to %s: pitch=%.2f roll=%.2f yaw=%.2f",
        (ardupilot_namespace_ + "mount_control/command").c_str(), pitch, roll, yaw);
    pub_->publish(msg);
  }
private:
  std::string ardupilot_namespace_;
  rclcpp::Publisher<mavros_msgs::msg::MountControl>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_;
};
