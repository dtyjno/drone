#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/msg/command_code.hpp>
#include <rclcpp/rclcpp.hpp>

class ServoController : public rclcpp::Node {
public:
    ServoController() : Node("servo_controller") {
        client_ = this->create_client<mavros_msgs::srv::CommandLong>("mavros/cmd/command");
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
    }

    void set_servo(int servo_number, float position) {
        auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
        request->command = mavros_msgs::msg::CommandCode::DO_SET_SERVO;
        request->param1 = servo_number;
        request->param2 = position;

        auto result_future = client_->async_send_request(request);

        // Wait for the result
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Servo set successfully");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service set_servo");
        }
    }

private:
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr client_;
};
#endif  // SERVOCONTROLLER_H