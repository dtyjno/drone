// * /mavros/local_position/pose [geometry_msgs/msg/PoseStamped] 1 publisher
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


class PoseSubscriber : public rclcpp::Node
{
public:
	explicit PoseSubscriber() : Node("pose_subscriber")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		
		subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose", qos,
		std::bind(&PoseSubscriber::pose_callback, this, std::placeholders::_1));


	}

private:
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
	void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const;
};

void PoseSubscriber::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
{
// RCLCPP_INFO(this->get_logger(), "Received pose: '%d'", msg->pose);
std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
std::cout << "RECEIVED SENSOR COMBINED DATA"   << std::endl;
std::cout << "============================="   << std::endl;
std::cout << "stamp:" << std::to_string(msg->header.stamp.sec) << "." << std::to_string(msg->header.stamp.nanosec) << std::endl;
std::cout << "frame_id:" << msg->header.frame_id << std::endl;
std::cout << "position x: " << msg->pose.position.x << std::endl;
std::cout << "position y: " << msg->pose.position.y  << std::endl;
std::cout << "position z: " << msg->pose.position.z  << std::endl;
std::cout << "orientation: x: " << msg->pose.orientation.x << std::endl;
std::cout << "orientation: y: " << msg->pose.orientation.y << std::endl;
std::cout << "orientation: z: " << msg->pose.orientation.z << std::endl;
std::cout << "orientation: w: " << msg->pose.orientation.w << std::endl;
}

int main(int argc, char *argv[])
{
	std::cout << "Starting sensor_combined listener node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PoseSubscriber>());

	rclcpp::shutdown();
	return 0;
}
