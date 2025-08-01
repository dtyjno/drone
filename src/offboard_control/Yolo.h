#ifndef YOLO_H
#define YOLO_H

#include "sensor_msgs/msg/image.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "vision_msgs/msg/detection2_d_array.hpp"

#include "Readyaml.h"
#include <eigen3/Eigen/Eigen>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
//#include <opencv2/opencv.hpp>
//#include "cv_bridge/cv_bridge.h"

// using namespace cv;
// using namespace cv::dnn;

// 简单的2D卡尔曼滤波器类
class KalmanFilter2D {
public:
    KalmanFilter2D(double process_noise = 0.01, double measurement_noise = 0.1) {
        // 状态向量: [x, y, vx, vy]
        state = Eigen::Vector4d::Zero();
        
        // 状态转移矩阵 (假设恒定速度模型)
        F = Eigen::Matrix4d::Identity();
        
        // 观测矩阵 (只观测位置)
        H = Eigen::Matrix<double, 2, 4>::Zero();
        H(0, 0) = 1.0;  // 观测x
        H(1, 1) = 1.0;  // 观测y
        
        // 过程噪声协方差矩阵
        Q = Eigen::Matrix4d::Identity() * process_noise;
        
        // 测量噪声协方差矩阵
        R = Eigen::Matrix2d::Identity() * measurement_noise;
        
        // 误差协方差矩阵
        P = Eigen::Matrix4d::Identity() * 1.0;
        
        initialized = false;
    }
    
    void update(double x, double y, double dt) {
        if (!initialized) {
            // 首次初始化
            state << x, y, 0.0, 0.0;
            initialized = true;
            last_time = std::chrono::steady_clock::now();
            return;
        }
        
        // 更新状态转移矩阵的时间相关部分
        F(0, 2) = dt;  // x = x + vx * dt
        F(1, 3) = dt;  // y = y + vy * dt
        
        // 预测步骤
        state = F * state;
        P = F * P * F.transpose() + Q;
        
        // 更新步骤
        Eigen::Vector2d measurement(x, y);
        Eigen::Vector2d innovation = measurement - H * state;
        Eigen::Matrix2d S = H * P * H.transpose() + R;
        Eigen::Matrix<double, 4, 2> K = P * H.transpose() * S.inverse();
        
        state = state + K * innovation;
        P = (Eigen::Matrix4d::Identity() - K * H) * P;
    }
    
    double getX() const { return state(0); }
    double getY() const { return state(1); }
    double getVX() const { return state(2); }
    double getVY() const { return state(3); }
    
    bool isInitialized() const { return initialized; }
    
private:
    Eigen::Vector4d state;           // 状态向量 [x, y, vx, vy]
    Eigen::Matrix4d F;               // 状态转移矩阵
    Eigen::Matrix<double, 2, 4> H;   // 观测矩阵
    Eigen::Matrix4d Q;               // 过程噪声协方差
    Eigen::Matrix2d R;               // 测量噪声协方差
    Eigen::Matrix4d P;               // 误差协方差矩阵
    bool initialized;
    std::chrono::steady_clock::time_point last_time;
};

class YOLO : public rclcpp::Node
{
public:
    YOLO() : Node("image_pub")
    {
        visualization_circles_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization_targets", 10);
        // 新增Detection2DArray订阅者
        detection2d_array_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "detection2d_array", 10,
            std::bind(&YOLO::detection2d_array_callback, this, std::placeholders::_1)
        );
        read_configs("camera.yaml");
        // 初始化卡尔曼滤波器
        circle_filter = std::make_unique<KalmanFilter2D>(process_noise, measurement_noise);  // 圆形目标滤波器
        h_filter = std::make_unique<KalmanFilter2D>(process_noise, measurement_noise);       // H型目标滤波器
        last_update_time = std::chrono::steady_clock::time_point{};   // 初始化时间
    }
    enum TARGET_TYPE{
        CIRCLE, //0
        H, //+=1
        STUFFED
    };

    bool is_get_target(enum TARGET_TYPE type){
        return fabs(get_raw_x(type)) > 0.0001 && fabs(get_raw_y(type)) > 0.0001;
        return false;
    }
    
    std::vector<vision_msgs::msg::BoundingBox2D> get_raw_targets(enum TARGET_TYPE type){
        if(type == CIRCLE){
            return circle_raw;
        }
        else if(type == H){
            return h_raw;
        }
        else if(type == STUFFED){
            return stuffed_raw;
        }
        return {};
    }
    // 获取原始未滤波的坐标
    float get_raw_x(enum TARGET_TYPE type){
        if (type == CIRCLE){
            return circle_raw.size() > 0 ? circle_raw[0].center.position.x : 0.0f;
        }
        else if(type == H){
            return h_raw.size() > 0 ? h_raw[0].center.position.x : 0.0f;
        }
        else if (type == STUFFED){
            return stuffed_raw.size() > 0 ? stuffed_raw[0].center.position.x : 0.0f;
        } 
        return 0;
    }
    float get_raw_y(enum TARGET_TYPE type){
        if (type == CIRCLE){
            return circle_raw.size() > 0 ? circle_raw[0].center.position.y : 0.0f;
        }
        else if(type == H){
            return h_raw.size() > 0 ? h_raw[0].center.position.y : 0.0f;
        }
        else if (type == STUFFED){
            return stuffed_raw.size() > 0 ? stuffed_raw[0].center.position.y : 0.0f;
        } 
        return 0;
    }
    
    float get_x(enum TARGET_TYPE type){
        if(type == CIRCLE){
            return circle_filter->isInitialized() ? circle_filter->getX() : get_raw_x(CIRCLE);
        }
        else if(type == H){
            return h_filter->isInitialized() ? h_filter->getX() : get_raw_x(H);
        }
        return 0;
    }
    float get_y(enum TARGET_TYPE type){
        if(type == CIRCLE){
            return circle_filter->isInitialized() ? circle_filter->getY() : get_raw_y(CIRCLE);
        }
        else if(type == H){
            return h_filter->isInitialized() ? h_filter->getY() : get_raw_y(H);
        }
        return 0;
    }
    float get_width(enum TARGET_TYPE type){
        (void)type;
        return 0;
    }
    float get_height(enum TARGET_TYPE type){
        (void)type;
        return 0;
    }
    // 获取滤波器的速度信息
    float get_velocity_x(enum TARGET_TYPE type){
        if(type == CIRCLE && circle_filter->isInitialized()){
            return circle_filter->getVX();
        }
        else if(type == H && h_filter->isInitialized()){
            return h_filter->getVX();
        }
        return 0;
    }
    float get_velocity_y(enum TARGET_TYPE type){
        if(type == CIRCLE && circle_filter->isInitialized()){
            return circle_filter->getVY();
        }
        else if(type == H && h_filter->isInitialized()){
            return h_filter->getVY();
        }
        return 0;
    }
    void read_configs(const std::string &filename)
    {
        YAML::Node config = Readyaml::readYAML(filename);
        cap_frame_width = config["width"].as<float>();
        cap_frame_height = config["height"].as<float>();  
        process_noise = config["process_noise"].as<double>(0.01);
        measurement_noise = config["measurement_noise"].as<double>(0.5);  

    }
    
    
    int get_cap_frame_width()
    {
        return cap_frame_width;
    }
    int get_cap_frame_height()
    {
        return cap_frame_height;
    }
    class Target
    {
    public:
        Target() = default; // 默认构造函数
        Target(float x, float y, float z, float r, float g, float b, float radius, const std::string &category, int id)
            : x(x), y(y), z(z), r(r), g(g), b(b), radius(radius), category(category), id(id) {}
        float x, y, z;           // 圆心坐标
        float r = 1, g = 0, b = 0;           // 颜色
        float radius;           // 半径
        std::string category;   // 分类标签
        int id;                 // 目标ID
        float fx = 1;
        float relative_z = 1; // 相对高度
        float caculate_pixel_radius(void) const {
            if (relative_z <= 0.01f) { // 防止除零
                return 5.0f; // 返回最小半径
            }
            float pixel_radius = (radius / relative_z) * fx; // 使用fx作为代表焦距
            return std::max(pixel_radius, 5.0f); // 最小半径为5像素
        }
    };
    void append_target(const Target &target)
    {
        targets.push_back(target);
    }
    void append_targets(const std::vector<Target> &new_targets)
    {
        for (const auto &target : new_targets) {
            targets.push_back(target);
        }
    }
    void clear_targets()
    {
        targets.clear();
    }
    void publish_visualization_target(void)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        for (const auto &target : targets) {
            // std::cout << "目标位置: (" << target.x << ", " << target.y << ", " << target.z << ")" << std::endl;
            visualization_msgs::msg::Marker marker;

            marker.header.frame_id = "map"; // 或者其他适当的坐标系
            marker.header.stamp = this->now();
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
private:
    int cap_frame_width = 1920; // 待覆盖默认值 图像宽度
    int cap_frame_height = 1080; // 同上
    double process_noise = 0.01; // 过程噪声
    double measurement_noise = 0.5; // 测量噪声
    
    // 原始坐标数据
    std::vector<vision_msgs::msg::BoundingBox2D> circle_raw;
    std::vector<vision_msgs::msg::BoundingBox2D> h_raw;
    std::vector<vision_msgs::msg::BoundingBox2D> stuffed_raw;
    
    std::vector<YOLO::Target> targets; // 目标点集合
    
    // 卡尔曼滤波器
    std::unique_ptr<KalmanFilter2D> circle_filter;
    std::unique_ptr<KalmanFilter2D> h_filter;
    std::chrono::steady_clock::time_point last_update_time;
    
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
