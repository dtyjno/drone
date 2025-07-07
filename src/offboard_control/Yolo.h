#ifndef YOLO_H
#define YOLO_H

#include "ros2_yolo_msgs/msg/detected_box.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "Readyaml.h"
//#include <opencv2/opencv.hpp>
//#include "cv_bridge/cv_bridge.h"

// using namespace cv;
// using namespace cv::dnn;
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

        subscriber_ = this->create_subscription<ros2_yolo_msgs::msg::DetectedBox>(
            "detected_boxes", 
            10, 
            std::bind(&YOLO::coord_callback, this, std::placeholders::_1)
        );
        read_configs("camera.yaml");
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
            return fabs(get_x(type)) > 0.0001 && fabs(get_y(type)) > 0.0001;
        }
        else if(type == H){
            return fabs(get_x(type)) > 0.0001 && fabs(get_y(type)) > 0.0001;
        }
        return false;
    }
	float get_x(enum TARGET_TYPE type){
        if(type == CIRCLE){
            return x_circle;
        }
        else if(type == H){
            return x_h;
        }
        return 0;
    }
    float get_y(enum TARGET_TYPE type){
        if(type == CIRCLE){
            return y_circle;
        }
        else if(type == H){
            return y_h;
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
	}
    int get_cap_frame_width()
    {
        return cap_frame_width;
    }
    int get_cap_frame_height()
    {
        return cap_frame_height;
    }
private:
    int cap_frame_width = 1920; // 待覆盖默认值 图像宽度
    int cap_frame_height = 1080; // 同上
	float x_circle;
    float x_h;
	float y_circle;
    float y_h;
    int flag_servo;
    // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Subscription<ros2_yolo_msgs::msg::DetectedBox>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    // VideoCapture cap;


    void coord_callback(const ros2_yolo_msgs::msg::DetectedBox::SharedPtr msg)
    {
        x_circle = msg->x1;
        y_circle = msg->y1;
        x_h = msg->x2;
        y_h = msg->y2;
        flag_servo = msg->servo;
        int flag = msg->servo;
		(void)flag;
        // RCLCPP_INFO(this->get_logger(), "收到坐标c(%f, %f) h(%f ,%f), flag_servo = %d", x_circle, y_circle, x_h, y_h, flag);
    }
    
};

#endif // YOLO_H
