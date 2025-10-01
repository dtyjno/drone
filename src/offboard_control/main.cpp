#include <rclcpp/rclcpp.hpp>
#include "APMROS2drone/APMROS2Drone.h"

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

	rclcpp::executors::MultiThreadedExecutor executor;

	// auto yolo_ = std::make_shared<YOLO>();
	// std::cout << "YOLO node initialized." << std::endl;
	auto offboard_control = APMROS2Drone::create("/mavros/");
	std::cout << "APMROS2Drone node initialized." << std::endl;

	/* 运行节点，并检测退出信号*/
	// executor.add_node(yolo_);
	executor.add_node(offboard_control->get_node());
	std::cout << "Nodes added to executor. Spinning..." << std::endl;
	executor.spin();

	rclcpp::shutdown();
	return 0;
}
