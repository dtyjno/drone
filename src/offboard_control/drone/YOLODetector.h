#ifndef OFFBOARD_CONTROL__MODULE__YOLODETECTOR_H
#define OFFBOARD_CONTROL__MODULE__YOLODETECTOR_H

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <chrono>
#include <memory>
#include <algorithm>

//#include <opencv2/opencv.hpp>
//#include "cv_bridge/cv_bridge.h"

// using namespace cv;
// using namespace cv::dnn;

#include "../algorithm/KalmanFilter2D.h"
#include "../utils/Readyaml.h"
#include "YOLOTargetType.h"
#include "YOLODetectorInterface.h"

class YOLODetector : public YOLODetectorInterface
{
public:
    YOLODetector()
    {
        read_configs("camera.yaml");
        // 初始化卡尔曼滤波器
        circle_filter = std::make_unique<KalmanFilter2D>(process_noise, measurement_noise);  // 圆形目标滤波器
        h_filter = std::make_unique<KalmanFilter2D>(process_noise, measurement_noise);       // H型目标滤波器
        last_update_time = std::chrono::steady_clock::time_point{};   // 初始化时间
    }

    const static std::unordered_map<YOLO_TARGET_TYPE, std::string> target_type_strings;
    static std::string enumToString(YOLO_TARGET_TYPE t);

    bool is_get_target(YOLO_TARGET_TYPE type) const override {
        return fabs(get_raw_x(type)) > 0.0001 && fabs(get_raw_y(type)) > 0.0001;
        return false;
    }

    std::vector<vision_msgs::msg::BoundingBox2D> get_raw_targets(YOLO_TARGET_TYPE type) const override {
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
    float get_raw_x(YOLO_TARGET_TYPE type) const override {
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
    float get_raw_y(YOLO_TARGET_TYPE type) const override {
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

    float get_x(YOLO_TARGET_TYPE type) const override {
        if(type == CIRCLE){
            return circle_filter->isInitialized() ? circle_filter->getX() : get_raw_x(CIRCLE);
        }
        else if(type == H){
            return h_filter->isInitialized() ? h_filter->getX() : get_raw_x(H);
        }
        return 0;
    }
    float get_y(YOLO_TARGET_TYPE type) const override {
        if(type == CIRCLE){
            return circle_filter->isInitialized() ? circle_filter->getY() : get_raw_y(CIRCLE);
        }
        else if(type == H){
            return h_filter->isInitialized() ? h_filter->getY() : get_raw_y(H);
        }
        return 0;
    }
    float get_width(YOLO_TARGET_TYPE type) const override {
        (void)type;
        return 0;
    }
    float get_height(YOLO_TARGET_TYPE type) const override {
        (void)type;
        return 0;
    }
    // 获取滤波器的速度信息
    float get_velocity_x(YOLO_TARGET_TYPE type) const override {
        if(type == CIRCLE && circle_filter->isInitialized()){
            return circle_filter->getVX();
        }
        else if(type == H && h_filter->isInitialized()){
            return h_filter->getVX();
        }
        return 0;
    }
    float get_velocity_y(YOLO_TARGET_TYPE type) const override {
        if(type == CIRCLE && circle_filter->isInitialized()){
            return circle_filter->getVY();
        }
        else if(type == H && h_filter->isInitialized()){
            return h_filter->getVY();
        }
        return 0;
    }
    void read_configs(const std::string &filename) override 
    {
        YAML::Node config = Readyaml::readYAML(filename);
        cap_frame_width = config["width"].as<float>();
        cap_frame_height = config["height"].as<float>();  
        process_noise = config["process_noise"].as<double>(0.01);
        measurement_noise = config["measurement_noise"].as<double>(0.5);  

    }


    int get_cap_frame_width() const override 
    {
        return cap_frame_width;
    }
    int get_cap_frame_height() const override 
    {
        return cap_frame_height;
    }
    void append_target(const TargetData &target) override 
    {
        targets.push_back(target);
    }
    void append_targets(const std::vector<TargetData> &new_targets) override 
    {
        for (const auto &target : new_targets) {
            targets.push_back(target);
        }
    }
    void clear_targets() override 
    {
        targets.clear();
    }
    void publish_visualization_target(void) override 
    {
        clear_targets();
    }

protected:
    int cap_frame_width = 1920; // 待覆盖默认值 图像宽度
    int cap_frame_height = 1080; // 同上
    double process_noise = 0.01; // 过程噪声
    double measurement_noise = 0.5; // 测量噪声
    
    // 原始坐标数据
    std::vector<vision_msgs::msg::BoundingBox2D> circle_raw;
    std::vector<vision_msgs::msg::BoundingBox2D> h_raw;
    std::vector<vision_msgs::msg::BoundingBox2D> stuffed_raw;

    std::vector<TargetData> targets; // 目标点集合

    // 卡尔曼滤波器
    std::unique_ptr<KalmanFilter2D> circle_filter;
    std::unique_ptr<KalmanFilter2D> h_filter;
    std::chrono::steady_clock::time_point last_update_time;
};

#endif // YOLO_H
