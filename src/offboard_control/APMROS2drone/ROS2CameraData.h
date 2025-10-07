#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "../drone/CameraData.h"

class ROS2CameraData : public CameraData, public rclcpp::Node {
public:
    ROS2CameraData() : Node("camera_data_node") {
        // 初始化 ROS2 节点
        RCLCPP_INFO(this->get_logger(), "ROS2 CameraData node initialized.");

        // 订阅图像话题
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_topic", 10,
            std::bind(&ROS2CameraData::image_callback, this, std::placeholders::_1)
        );
    }
private:
    // 这里可以添加 ROS2 相关的成员变量和方法
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // 处理接收到的图像消息
        RCLCPP_INFO(this->get_logger(), "Received image with height: %d, width: %d", msg->height, msg->width);
        // 将 ROS 图像消息转换为 OpenCV 格式
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        cur_camera_data.image = image;
        cur_camera_data.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        notify_image_update(cur_camera_data);

    }
    // CameraData cur_camera_data;
    // std::vector<vision_msgs::msg::BoundingBox2D> circle_raw, h_raw;
};
