#ifndef OFFBOARD_CONTROL__MODULE__ROS2YOLODETECTOR_H
#define OFFBOARD_CONTROL__MODULE__ROS2YOLODETECTOR_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "../drone/YOLODetector.h"

class ROS2YOLODetector : public YOLODetector
{
public:
    ROS2YOLODetector(rclcpp::Node::SharedPtr node)
        : node(node)
    {
        visualization_circles_publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization_targets", 10);
        // 新增Detection2DArray订阅者
        detection2d_array_sub_ = node->create_subscription<vision_msgs::msg::Detection2DArray>(
            "detection2d_array", 10,
            std::bind(&ROS2YOLODetector::detection2d_array_callback, this, std::placeholders::_1)
        );
    }

    void publish_visualization_target(void)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        for (const auto &target : targets) {
            // std::cout << "目标位置: (" << target.x << ", " << target.y << ", " << target.z << ")" << std::endl;
            visualization_msgs::msg::Marker marker;

            marker.header.frame_id = "map"; // 或者其他适当的坐标系
            marker.header.stamp = node->now();
            marker.ns = target.category; // 使用目标ID作为命名空间
            marker.id = target.id;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = target.x;
            marker.pose.position.y = target.y;
            marker.pose.position.z = target.z;
            marker.scale.x = target.caculate_pixel_radius() * 2; // 直径
            marker.scale.y = target.caculate_pixel_radius() * 2; // 直径
            marker.scale.z = 0.1; // 高度
 
            marker.color.r = target.r; // 使用目标颜色
            marker.color.g = target.g; // 使用目标颜色
            marker.color.b = target.b; // 使用目标颜色
            marker.color.a = 1.0f; // 不透明

            marker.lifetime = rclcpp::Duration(0, 0); // 永久存在

            marker_array.markers.push_back(marker);
        }
        
        visualization_circles_publisher_->publish(marker_array);
        clear_targets();
    }

protected:
    rclcpp::Node::SharedPtr node;
private:
    // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_circles_publisher_;
    // rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr circle_center_publisher_;

    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection2d_array_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    // VideoCapture cap;


    // 新增Detection2DArray回调
    void detection2d_array_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
    {
        std::vector<vision_msgs::msg::BoundingBox2D> circle_raw, h_raw;
        for (const auto &det : msg->detections) {
            // if (det.results[0].score < 0.5) {
            //     continue; // 忽略低置信度的检测
            // }
            if (det.results[0].hypothesis.class_id == "circle") {
                circle_raw.push_back(det.bbox);
            } else if (det.results[0].hypothesis.class_id == "h") {
                h_raw.push_back(det.bbox);
            } else if (det.results[0].hypothesis.class_id == "stuffed") {
                stuffed_raw.push_back(det.bbox);
            }
            // det.bbox.center.x, det.bbox.center.y, det.results 等
        }

        // 更新原始数据
        sort(circle_raw.begin(), circle_raw.end(), [this](const vision_msgs::msg::BoundingBox2D &a, const vision_msgs::msg::BoundingBox2D &b) {
            return abs(a.center.position.x - cap_frame_width / 2) + abs(a.center.position.y - cap_frame_height / 2) <
                    abs(b.center.position.x - cap_frame_width / 2) + abs(b.center.position.y - cap_frame_height / 2);
        });
        if (circle_raw.size() > 0) {
            this->circle_raw = circle_raw;
        } else {
            this->circle_raw.clear();
        }
        sort(h_raw.begin(), h_raw.end(), [this](const vision_msgs::msg::BoundingBox2D &a, const vision_msgs::msg::BoundingBox2D &b) {
            return abs(a.center.position.x - cap_frame_width / 2) + abs(a.center.position.y - cap_frame_height / 2) <
                    abs(b.center.position.x - cap_frame_width / 2) + abs(b.center.position.y - cap_frame_height / 2);
        });
        if (h_raw.size() > 0) {
            this->h_raw = h_raw;
        } else {
            this->h_raw.clear();
        }
        sort(stuffed_raw.begin(), stuffed_raw.end(), [this](const vision_msgs::msg::BoundingBox2D &a, const vision_msgs::msg::BoundingBox2D &b) {
            return abs(a.center.position.x - cap_frame_width / 2) + abs(a.center.position.y - cap_frame_height / 2) <
                    abs(b.center.position.x - cap_frame_width / 2) + abs(b.center.position.y - cap_frame_height / 2);
        });
        if (stuffed_raw.size() > 0) {
            this->stuffed_raw = stuffed_raw;
        } else {
            this->stuffed_raw.clear();
        }
        // 计算时间间隔
        auto current_time = std::chrono::steady_clock::now();
        double dt = 0.05; // 默认时间间隔
        if (last_update_time.time_since_epoch().count() > 0) {
            dt = std::chrono::duration<double>(current_time - last_update_time).count();
        }
        last_update_time = current_time;
        
        // 更新卡尔曼滤波器
        if (is_get_target(CIRCLE)) {
            circle_filter->update(get_raw_x(CIRCLE), get_raw_y(CIRCLE), dt);
        } else if (is_get_target(STUFFED)) {
            circle_filter->update(get_raw_x(STUFFED), get_raw_y(STUFFED), dt);
        }

        if (is_get_target(H)) {
            h_filter->update(get_raw_x(H), get_raw_y(H), dt);
        }
        
        // RCLCPP_INFO(this->get_logger(), "收到坐标c(%f, %f) h(%f ,%f)", get_raw_x(CIRCLE), get_raw_y(CIRCLE), get_raw_x(H), get_raw_y(H));
        // RCLCPP_INFO(this->get_logger(), "滤波后c(%f, %f) h(%f ,%f) vc(%f, %f) vh(%f, %f)",
        //     get_x(CIRCLE), get_y(CIRCLE), get_x(H), get_y(H),
        //     get_velocity_x(CIRCLE), get_velocity_y(CIRCLE),
        //     get_velocity_x(H), get_velocity_y(H));
    }

};

#endif // YOLO_H
