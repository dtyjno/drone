#include <rclcpp/rclcpp.hpp>
#include "Yolo.h"
#include "ServoController.h"
#include "OffboardControl.h"

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

	rclcpp::executors::MultiThreadedExecutor executor;

	auto yolo_ = std::make_shared<YOLO>();
	auto node = std::make_shared<OffboardControl>("/mavros/",yolo_);
	
	/* 运行节点，并检测退出信号*/
	executor.add_node(yolo_);
	executor.add_node(node);

	executor.spin();

	rclcpp::shutdown();
	return 0;
}
