#include "yolo.hpp"




void YOLO::yolo_timer_callback(const vision_msgs::msg::Detection2DArray msg){
	std::cout<<"YOLO"<<std::endl;
	for (const auto& detection : msg.detections) {
		auto bbox = detection.bbox;
		float x = bbox.center.position.x;
		float y = bbox.center.position.y;
		float width = bbox.size_x;
		float height = bbox.size_y;
		// 这里的x和y是边界框中心的坐标，width和height是边界框的大小
		// 你可以在这里添加你的代码来处理这些坐标
		RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, width: %f, height: %f", x, y, width, height);
	}
}
