#ifndef YOLO_H
#define YOLO_H

#include "ros2_interfaces/msg/coord.hpp"
#include "sensor_msgs/msg/image.hpp"
//#include <opencv2/opencv.hpp>
//#include "cv_bridge/cv_bridge.h"


#define SET_CAP_FRAME_WIDTH 640
#define SET_CAP_FRAME_HEIGHT 480

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

        subscriber_ = this->create_subscription<ros2_interfaces::msg::Coord>(
            "coord", 
            10, 
            std::bind(&YOLO::coord_callback, this, std::placeholders::_1)
        );
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
            return fabs(get_x(type)) < 0.0001 && fabs(get_y(type)) < 0.0001;
        }
        else if(type == H){
            return fabs(get_x(type)) < 0.0001 && fabs(get_y(type)) < 0.0001;
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
private:
	float x_circle;
    float x_h;
	float y_circle;
    float y_h;
    // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Subscription<ros2_interfaces::msg::Coord>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    // VideoCapture cap;

    // void timer_callback()
    // {
    //     Mat frame;
    //     cap >> frame;
    //     if (frame.rows > 0 && frame.cols > 0)
    //     {
    //         auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    //         publisher_->publish(*img_msg);
    //     }
    //     //RCLCPP_INFO(this->get_logger(), "Publishing video frame");
    // }

    void coord_callback(const ros2_interfaces::msg::Coord::SharedPtr msg)
    {
        x_circle = msg->x1;
        y_circle = msg->y1;
        x_h = msg->x2;
        y_h = msg->y2;
        int flag = msg->flag_servo;
		(void)flag;
        //RCLCPP_INFO(this->get_logger(), "收到坐标(%f, %f), flag_servo = %d", x, y, flag);
    }
    
};



// #define SET_CAP_FRAME_WIDTH 640
// #define SET_CAP_FRAME_HEIGHT 480
// #include <rclcpp/rclcpp.hpp>
// #include <iostream>
// #include <vision_msgs/msg/detection2_d_array.hpp>
// // #include "sensor_msgs/msg/image.hpp"
// // #include <opencv2/opencv.hpp>
// // #include "cv_bridge/cv_bridge.h"
// // using namespace Eigen;
// // using namespace std;
// // using namespace cv;
// // using namespace cv::dnn;
// class YOLO : public rclcpp::Node{
// public:
// 	YOLO()
// 	: Node("YOLO")
// 	{
// 		// 声明回调组,实例化回调组，类型为：可重入的
// 		rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_= this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
// 		// Each of these callback groups is basically a thread
//     	// Everything assigned to one of them gets bundled into the same thread
// 		auto sub_opt = rclcpp::SubscriptionOptions();
//     	sub_opt.callback_group = callback_group_subscriber_;
// 		//打印thread_id
// 		//std::cout<<std::to_string(std::hash<std::thread::id>()(std::this_thread::get_id())).c_str()<<std::endl;
// 		yolo_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
// 		"yolo_result", 50, std::bind(&YOLO::yolo_timer_callback, this, std::placeholders::_1),sub_opt);


// 		// publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);

//         // timer_ = this->create_wall_timer(
//         //     std::chrono::milliseconds(100), 
//         //     std::bind(&YOLO::timer_callback, this)
//         // );
// 		// cap.open(0, CAP_V4L2);
//         // cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
//         // cap.set(CAP_PROP_FRAME_WIDTH, 640);//图像的宽
//         // cap.set(CAP_PROP_FRAME_HEIGHT, 480);//图像的高
// 	}
// 	int get_thread_id(){
// 		return std::hash<std::thread::id>()(std::this_thread::get_id());
// 	}
// 	float get_x(){
// 		return x;
// 	}
// 	float get_y(){
// 		return y;
// 	}
// 	float get_width(){
// 		return width;
// 	}
// 	float get_height(){
// 		return height;
// 	}
// private:
// 	// rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
// 	// rclcpp::TimerBase::SharedPtr timer_;
//     // VideoCapture cap;
// 	// void timer_callback()
//     // {
//     //     Mat frame;
//     //     cap >> frame;
//     //     if (frame.rows > 0 && frame.cols > 0)
//     //     {
//     //         auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
//     //         publisher_->publish(*img_msg);
//     //     }
//     //     RCLCPP_INFO(this->get_logger(), "Publishing video frame");
//     // }

// 	rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr yolo_sub_;
// 	/*
// 	ros2 topic pub /yolo_result vision_msgs/msg/Detection2DArray '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "string"}, detections: [{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "string"}, results: [{id: 0, score: 0.0, x: 0.0, y: 0.0, width: 0.0, height: 0.0, roi: {x_offset: 0, y_offset: 0, height: 0, width: 0}, mask_array: [0, 0, 0], mask: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: "string"}, image: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: "string"}, height: 0, width: 0, encoding: "string", is_bigendian: 0, step: 0, data: [0, 0, 0]}}]}]}'
// 	*/
// 	float x;
// 	float y;
// 	float width;
// 	float height;
// 	void yolo_timer_callback(const vision_msgs::msg::Detection2DArray msg){
// 		std::cout<<"-----------------------YOLO——START----------------------"<<std::endl;
// 		for (const auto& detection : msg.detections) {
// 			auto bbox = detection.bbox;
// 			x = bbox.center.position.x;
// 			y = bbox.center.position.y;
// 			width = bbox.size_x;
// 			height = bbox.size_y;
// 			// 这里的x和y是边界框中心的坐标，width和height是边界框的大小
// 			// 你可以在这里添加你的代码来处理这些坐标
// 			RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, width: %f, height: %f", x, y, width, height);
// 		}
// 		std::cout<<"------------------------YOLO——END-----------------------"<<std::endl;
// 	}
// };
#endif // YOLO_H
