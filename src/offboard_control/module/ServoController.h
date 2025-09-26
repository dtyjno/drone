#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/msg/command_code.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono> // 添加 chrono 头文件
#include <iomanip> 

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
    ServoController(const std::string ardupilot_namespace, rclcpp::Node::SharedPtr node) 
        : node(node) 
    {
        this->ardupilot_namespace = ardupilot_namespace;
        client_ = node->create_client<mavros_msgs::srv::CommandLong>(ardupilot_namespace + "cmd/command");
        // while (!client_->wait_for_service(std::chrono::seconds(1))) {
        //     if (!rclcpp::ok()) {
        //         RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
        //         return;
        //     }
        //     RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
        // }
        read_configs("OffboardControl.yaml");
    }

    void set_servo(int servo_number, float position) {
        auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
        RCLCPP_INFO(node->get_logger(), "Setting servo %d to position %f", servo_number, position);
        request->command = mavros_msgs::msg::CommandCode::DO_SET_SERVO;
        request->param1 = servo_number;
        request->param2 = position;

        // if (!node->debug_mode_) {
        //     // 如果不是调试模式，等待服务可用
        //     while (!client_->wait_for_service(std::chrono::seconds(1))) {
        //         if (!rclcpp::ok()) {
        //             RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
        //             return;
        //         }
        //         RCLCPP_INFO(node->get_logger(), "Set Servo Service not available, waiting again...");
        //     }
        //     RCLCPP_INFO(node->get_logger(), "set_servo command send");
        // } else {
        //     RCLCPP_INFO(node->get_logger(), "Debug mode: skipping service wait");
        // }
        
        rclcpp::Node::SharedPtr node = this->node;
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

    void read_configs(const std::string &filename) {
        YAML::Node config = Readyaml::readYAML(filename);
		try {
			servo_open_position = config["servo_open_position"].as<float>();
			servo_close_position = config["servo_close_position"].as<float>();

			RCLCPP_INFO(node->get_logger(), "读取servo_open_positiont: %f", servo_open_position);
			RCLCPP_INFO(node->get_logger(), "读取servo_close_position: %f", servo_close_position);
		} catch (const YAML::Exception &e) {
			RCLCPP_ERROR(node->get_logger(), "读取配置文件时发生错误: %s", e.what());
			return;
		}

    }
    float set_servo_open(int index = 1) {
        set_servo(index, servo_open_position);
        return servo_open_position;
    }

    float set_servo_close(int index = 1) {
        set_servo(index, servo_close_position);
        return servo_close_position;
    }
    void set_servo_open_position(float position) {
        servo_open_position = position;
    }
    void set_servo_close_position(float position) {
        servo_close_position = position;
    }
    float get_servo_open_position() const {
        return servo_open_position;
    }

    float get_servo_close_position() const {
        return servo_close_position;
    }

private:
    rclcpp::Node::SharedPtr node;
    std::string ardupilot_namespace;
    float servo_open_position;
    float servo_close_position;

    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr client_;
};

#endif  // SERVOCONTROLLER_H