#ifndef YOLO_H
#define YOLO_H

#include "ros2_yolo_msgs/msg/detected_box.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

#include "Readyaml.h"
#include <eigen3/Eigen/Eigen>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
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
        // publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image", 10);

        // timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(100), 
        //     std::bind(&YOLO::timer_callback, this)
        // );
        visualization_circles_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_targets", 10);

        subscriber_ = this->create_subscription<ros2_yolo_msgs::msg::DetectedBox>(
            "detected_boxes", 
            10, 
            std::bind(&YOLO::coord_callback, this, std::placeholders::_1)
        );
        read_configs("camera.yaml");
        
        // 初始化卡尔曼滤波器
        circle_filter = std::make_unique<KalmanFilter2D>(process_noise, measurement_noise);  // 圆形目标滤波器
        h_filter = std::make_unique<KalmanFilter2D>(process_noise, measurement_noise);       // H型目标滤波器
        last_update_time = std::chrono::steady_clock::time_point{};   // 初始化时间
        
        // cap.open(0, CAP_V4L2);
        // cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
        // cap.set(CAP_PROP_FRAME_WIDTH, SET_CAP_FRAME_HEIGHT);//图像的宽
        // cap.set(CAP_PROP_FRAME_HEIGHT, SET_CAP_FRAME_WIDTH);//图像的高

    }
    enum TARGET_TYPE{
        CIRCLE, //0
        H //+=1
    };

    bool is_get_target(enum TARGET_TYPE type){
        if(type == CIRCLE){
            return fabs(get_raw_x(type)) > 0.0001 && fabs(get_raw_y(type)) > 0.0001;
        }
        else if(type == H){
            return fabs(get_raw_x(type)) > 0.0001 && fabs(get_raw_y(type)) > 0.0001;
        }
        return false;
    }
	float get_x(enum TARGET_TYPE type){
        if(type == CIRCLE){
            return circle_filter->isInitialized() ? circle_filter->getX() : x_circle_raw;
        }
        else if(type == H){
            return h_filter->isInitialized() ? h_filter->getX() : x_h_raw;
        }
        return 0;
    }
    float get_y(enum TARGET_TYPE type){
        if(type == CIRCLE){
            return circle_filter->isInitialized() ? circle_filter->getY() : y_circle_raw;
        }
        else if(type == H){
            return h_filter->isInitialized() ? h_filter->getY() : y_h_raw;
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
    int get_servo_flag(){
        return flag_servo;
    }
    void read_configs(const std::string &filename)
	{
		YAML::Node config = Readyaml::readYAML(filename);
		cap_frame_width = config["cap_frame_width"].as<float>();
        cap_frame_height = config["cap_frame_height"].as<float>();  
        process_noise = config["process_noise"].as<double>(0.01);
        measurement_noise = config["measurement_noise"].as<double>(0.5);  

    }
    
    // 获取原始未滤波的坐标
    float get_raw_x(enum TARGET_TYPE type){
        if(type == CIRCLE){
            return x_circle_raw;
        }
        else if(type == H){
            return x_h_raw;
        }
        return 0;
    }
    float get_raw_y(enum TARGET_TYPE type){
        if(type == CIRCLE){
            return y_circle_raw;
        }
        else if(type == H){
            return y_h_raw;
        }
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
    
    int get_cap_frame_width()
    {
        return cap_frame_width;
    }
    int get_cap_frame_height()
    {
        return cap_frame_height;
    }
    struct Target
    {
        float x, y, z;           // 圆心坐标
        float r, g, b;           // 颜色
        float radius;           // 半径
        std::string category;   // 分类标签
        int id;                 // 目标ID
    };
    
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
            marker.scale.x = target.radius * 2; // 直径
            marker.scale.y = target.radius * 2; // 直径
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
	float x_circle_raw;
    float x_h_raw;
	float y_circle_raw;
    float y_h_raw;
    int flag_servo;

    std::vector<YOLO::Target> targets; // 目标点集合
    
    // 卡尔曼滤波器
    std::unique_ptr<KalmanFilter2D> circle_filter;
    std::unique_ptr<KalmanFilter2D> h_filter;
    std::chrono::steady_clock::time_point last_update_time;
    
    // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_circles_publisher_;
    // rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr circle_center_publisher_;

    rclcpp::Subscription<ros2_yolo_msgs::msg::DetectedBox>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    // VideoCapture cap;


    void coord_callback(const ros2_yolo_msgs::msg::DetectedBox::SharedPtr msg)
    {
        // 保存原始数据
        x_circle_raw = msg->x1;
        y_circle_raw = msg->y1;
        x_h_raw = msg->x2;
        y_h_raw = msg->y2;
        flag_servo = msg->servo;
        
        // 计算时间间隔
        auto current_time = std::chrono::steady_clock::now();
        double dt = 0.1; // 默认时间间隔
        if (last_update_time.time_since_epoch().count() > 0) {
            dt = std::chrono::duration<double>(current_time - last_update_time).count();
        }
        last_update_time = current_time;
        
        // 更新卡尔曼滤波器
        if (fabs(x_circle_raw) > 0.0001 && fabs(y_circle_raw) > 0.0001) {
            circle_filter->update(x_circle_raw, y_circle_raw, dt);
        }
        
        if (fabs(x_h_raw) > 0.0001 && fabs(y_h_raw) > 0.0001) {
            h_filter->update(x_h_raw, y_h_raw, dt);
        }
        
        // RCLCPP_INFO(this->get_logger(), "收到坐标c(%f, %f) h(%f ,%f), flag_servo = %d", x_circle_raw, y_circle_raw, x_h_raw, y_h_raw, flag_servo);
        // RCLCPP_INFO(this->get_logger(), "滤波后c(%f, %f) h(%f ,%f)", get_x(CIRCLE), get_y(CIRCLE), get_x(H), get_y(H));
    }
    
};

#endif // YOLO_H
