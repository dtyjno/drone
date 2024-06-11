#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
class YOLO : public rclcpp::Node{
public:
	YOLO()
	: Node("YOLO")
	{
		// 声明回调组,实例化回调组，类型为：可重入的
		rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_= this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		// Each of these callback groups is basically a thread
    	// Everything assigned to one of them gets bundled into the same thread
		auto sub_opt = rclcpp::SubscriptionOptions();
    	sub_opt.callback_group = callback_group_subscriber_;
		std::cout<<std::to_string(std::hash<std::thread::id>()(std::this_thread::get_id())).c_str()<<std::endl;//string_thread_id
		
		yolo_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
		"yolo_result", 50, std::bind(&YOLO::yolo_timer_callback, this, std::placeholders::_1),sub_opt);
	}
private:
	rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr yolo_sub_;
	/*
	ros2 topic pub /yolo_result vision_msgs/msg/Detection2DArray '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "string"}, detections: [{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "string"}, results: [{id: 0, score: 0.0, x: 0.0, y: 0.0, width: 0.0, height: 0.0, roi: {x_offset: 0, y_offset: 0, height: 0, width: 0}, mask_array: [0, 0, 0], mask: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: "string"}, image: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: "string"}, height: 0, width: 0, encoding: "string", is_bigendian: 0, step: 0, data: [0, 0, 0]}}]}]}'
	*/
	void yolo_timer_callback(const vision_msgs::msg::Detection2DArray msg);
};