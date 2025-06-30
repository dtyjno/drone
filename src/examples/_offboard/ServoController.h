#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/msg/command_code.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono> // 添加 chrono 头文件
#include "OffboardControl_Base.h"

using namespace std::chrono_literals; // 使用 chrono 字面量

// ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong "request:
//   broadcast: false
//   command: 183  # DO_SET_SERVO
//   confirmation: 0
//   param1: 1     # 舵机编号
//   param2: 1500  # 舵机位置
//   param3: 0.0
//   param4: 0.0
//   param5: 0.0
//   param6: 0.0
//   param7: 0.0"

class ServoController {
public:
    ServoController(const std::string ardupilot_namespace, OffboardControl_Base* node) 
        : node(node) 
    {
        this->ardupilot_namespace = ardupilot_namespace;
        client_ = node->create_client<mavros_msgs::srv::CommandLong>(ardupilot_namespace + "cmd/command");
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
        }
    }

    void set_servo(int servo_number, float position) {
        auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
        request->command = mavros_msgs::msg::CommandCode::DO_SET_SERVO;
        request->param1 = servo_number;
        request->param2 = position;

        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
        }
        RCLCPP_INFO(node->get_logger(), "set_servo command send");

        OffboardControl_Base* node = this->node;
        auto result_future = client_->async_send_request(request,
            [node, this](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture future) {
                try {
                    auto reply = future.get();
                    if (reply->success) {
                        RCLCPP_INFO(node->get_logger(), "set_servo: success");
                    } else {
                        RCLCPP_ERROR(node->get_logger(), "set_servo: failed");
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(node->get_logger(), "Exception while calling service: %s", e.what());
                }
            });
    }

private:
    OffboardControl_Base* node;
    std::string ardupilot_namespace;

    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr client_;
};

#endif  // SERVOCONTROLLER_H