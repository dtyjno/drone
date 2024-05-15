// * /mavros/local_position/pose [geometry_msgs/msg/PoseStamped] 1 publisher
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/camera_image_captured.hpp>
#include <opencv2/opencv.hpp>

class CameraImageCaptured : public rclcpp::Node
{
public:
	explicit CameraImageCaptured() : Node("pose_subscriber")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		
		image_captured_subscription_ = this->create_subscription<mavros_msgs::msg::CameraImageCaptured>("/mavros/camera/image_captured", qos,
		std::bind(&CameraImageCaptured::image_captured_callback, this, std::placeholders::_1));


	}

private:
	rclcpp::Subscription<mavros_msgs::msg::CameraImageCaptured>::SharedPtr image_captured_subscription_;
	void image_captured_callback(const mavros_msgs::msg::CameraImageCaptured::SharedPtr msg) const;
};

void CameraImageCaptured::image_captured_callback(const mavros_msgs::msg::CameraImageCaptured::SharedPtr msg) const
{
	std::cout << "Image captured: " << msg->file_url << std::endl;
	cv::Mat img = cv::imread(msg->file_url);
    if (img.empty()) {
        std::cerr << "Failed to open image file: " << msg->file_url << std::endl;
        return;
    }

    cv::imshow("Image", img);
    cv::waitKey(1);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting sensor_combined listener node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CameraImageCaptured>());

	rclcpp::shutdown();
	return 0;
}
