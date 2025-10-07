#ifndef OFFBOARD_CONTROL__MODULE__ROS2YOLODETECTOR_H
#define OFFBOARD_CONTROL__MODULE__ROS2YOLODETECTOR_H

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <vision_msgs/msg/pose2_d.hpp>


#include "../drone/YOLODetector.h"
#include "../drone/CameraInterface.h"
#include "../drone/CameraDataObserverInterface.h"

#include <onnxruntime_cxx_api.h>


class ROS2YOLODetectorImage : public YOLODetector, public CameraDataObserverInterface
{
public:
    ROS2YOLODetectorImage(rclcpp::Node::SharedPtr node, const CameraInterface &camera_interface)
        : node(node), env(ORT_LOGGING_LEVEL_WARNING, "ros2_yolo"), camera_interface_(camera_interface)
    {
        visualization_circles_publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization_targets", 10);
        // 新增Detection2DArray订阅者
        detection2d_array_sub_ = node->create_subscription<vision_msgs::msg::Detection2DArray>(
            "detection2d_array", 10,
            std::bind(&ROS2YOLODetectorImage::detection2d_array_callback, this, std::placeholders::_1)
        );

        // 加载ONNX模型
        session_options.SetIntraOpNumThreads(1);
        session = std::make_unique<Ort::Session>(env, "best_circle.onnx", session_options);
        RCLCPP_INFO(this->get_logger(), "ONNX model loaded.");

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

    void on_image_update(const cv::Mat& image) override {
        RCLCPP_INFO(this->get_logger(), "Received image with height: %d, width: %d", image.rows, image.cols);
        cur_camera_data.image = image;
        cap_frame_width = image.cols;
        cap_frame_height = image.rows;

        // 1. 图像预处理
        cv::Mat resized;
        cv::resize(image, resized, cv::Size(640, 640));
        resized.convertTo(resized, CV_32F, 1.0 / 255);

        // 2. NHWC -> NCHW
        std::vector<cv::Mat> chw(3);
        for (int i = 0; i < 3; ++i)
            chw[i] = cv::Mat(640, 640, CV_32F, resized.ptr<float>(0) + i);
        cv::split(resized, chw);
        std::vector<float> input_tensor_values;
        for (int c = 0; c < 3; ++c)
            input_tensor_values.insert(input_tensor_values.end(), (float*)chw[c].datastart, (float*)chw[c].dataend);

        // 3. 构造输入tensor
        std::array<int64_t, 4> input_shape{1, 3, 640, 640};
        Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, input_tensor_values.data(), input_tensor_values.size(), input_shape.data(), input_shape.size());

        // 4. 推理
        const char* input_names[] = {"images"};
        const char* output_names[] = {"output"};
        auto output_tensors = session->Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);

        // 5. 解析输出并分类、排序
        std::vector<vision_msgs::msg::BoundingBox2D> circle_raw, h_raw, stuffed_raw;
        for (int i = 0; i < num_boxes; ++i) {
            float conf = output[i * num_attrs + 4];
            if (conf < 0.5) continue;

            float max_prob = 0.0f;
            int class_id = -1;
            for (int j = 5; j < num_attrs; ++j) {
                float prob = output[i * num_attrs + j];
                if (prob > max_prob) {
                    max_prob = prob;
                    class_id = j - 5;
                }
            }
            if (max_prob * conf < 0.5) continue;

            // 动态修正相机内参
            float scale_x = float(cap_frame_width) / 640.0f;
            float scale_y = float(cap_frame_height) / 640.0f;
            float fx_corr = fx * scale_x;
            float fy_corr = fy * scale_y;
            float cx_corr = cx * scale_x;
            float cy_corr = cy * scale_y;

            // 框坐标
            float cx_box = output[i * num_attrs + 0] * scale_x;
            float cy_box = output[i * num_attrs + 1] * scale_y;
            float w = output[i * num_attrs + 2] * scale_x;
            float h = output[i * num_attrs + 3] * scale_y;

            vision_msgs::msg::BoundingBox2D bbox;
            vision_msgs::msg::Pose2D pose;
            pose.position.x = cx_box;
            pose.position.y = cy_box;
            pose.theta = 0.0;
            bbox.center = pose;
            bbox.size_x = w;
            bbox.size_y = h;

            // 分类
            if (class_id == 0) {
                circle_raw.push_back(bbox);
            } else if (class_id == 2) {
                h_raw.push_back(bbox);
            } else if (class_id == 1) {
                stuffed_raw.push_back(bbox);
            }
        }

        // 按距离中心点排序
        auto center_sort = [this](const vision_msgs::msg::BoundingBox2D &a, const vision_msgs::msg::BoundingBox2D &b) {
            float center_x = cap_frame_width / 2.0f;
            float center_y = cap_frame_height / 2.0f;
            float da = std::abs(a.center.position.x - center_x) + std::abs(a.center.position.y - center_y);
            float db = std::abs(b.center.position.x - center_x) + std::abs(b.center.position.y - center_y);
            return da < db;
        };
        std::sort(circle_raw.begin(), circle_raw.end(), center_sort);
        std::sort(h_raw.begin(), h_raw.end(), center_sort);
        std::sort(stuffed_raw.begin(), stuffed_raw.end(), center_sort);

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

        // 6. 发布检测结果
        // detection2d_array_pub_->publish(det_arr);
    }
protected:
    rclcpp::Node::SharedPtr node;
    CameraInterface *camera_interface_;
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


    // ONNX Runtime相关成员
    Ort::Env env;
    Ort::SessionOptions session_options;
    std::unique_ptr<Ort::Session> session;

};

#endif // YOLO_H
