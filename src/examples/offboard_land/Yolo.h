#ifndef YOLO_H
#define YOLO_H

#include <rclcpp/rclcpp.hpp>
#include "ros2_interfaces/msg/coord.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"


#define SET_CAP_FRAME_WIDTH 640
#define SET_CAP_FRAME_HEIGHT 480

using namespace cv;
using namespace cv::dnn;
class YOLO : public rclcpp::Node
{
public:
    YOLO() : Node("image_pub")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&YOLO::timer_callback, this)
        );

        subscriber_ = this->create_subscription<ros2_interfaces::msg::Coord>(
            "coord", 
            10, 
            std::bind(&YOLO::coord_callback, this, std::placeholders::_1)
        );
        cap.open(0, CAP_V4L2);
        cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
        cap.set(CAP_PROP_FRAME_WIDTH, SET_CAP_FRAME_HEIGHT);//图像的宽
        cap.set(CAP_PROP_FRAME_HEIGHT, SET_CAP_FRAME_WIDTH);//图像的高
    }
	float get_x(){
		return x;
	}
	float get_y(){
		return y;
	}
	float get_width(){
		return 0;
	}
	float get_height(){
		return 0;
	}
private:
	float x;
	float y;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Subscription<ros2_interfaces::msg::Coord>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    VideoCapture cap;

    void timer_callback()
    {
        Mat frame;
        cap >> frame;
        if (frame.rows > 0 && frame.cols > 0)
        {
            auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            publisher_->publish(*img_msg);
        }
        //RCLCPP_INFO(this->get_logger(), "Publishing video frame");
    }

    void coord_callback(const ros2_interfaces::msg::Coord::SharedPtr msg)
    {
         x = msg->x;
         y = msg->y;
        int flag = msg->flag_servo;
		(void)flag;
        //RCLCPP_INFO(this->get_logger(), "收到坐标(%f, %f), flag_servo = %d", x, y, flag);
    }
    
};
#endif // YOLO_H
