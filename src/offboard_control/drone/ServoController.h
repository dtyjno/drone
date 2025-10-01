#pragma once

#include <chrono> // 添加 chrono 头文件
#include <iomanip> 

#include "ServoControllerInterface.h"

using namespace std::chrono_literals; // 使用 chrono 字面量

class ServoController : public ServoControllerInterface {
public:
    ServoController()
    {
        read_configs("OffboardControl.yaml");
    }

    void set_servo(int servo_number, float position) {
        (void) servo_number;
        (void) position;
    }

    void read_configs(const std::string &filename) {
        YAML::Node config = Readyaml::readYAML(filename);
		try {
			servo_open_position = config["servo_open_position"].as<float>();
			servo_close_position = config["servo_close_position"].as<float>();

			std::cout << "读取servo_open_position: " << servo_open_position << std::endl;
			std::cout << "读取servo_close_position: " << servo_close_position << std::endl;
		} catch (const YAML::Exception &e) {
			std::cerr << "读取配置文件时发生错误: " << e.what() << std::endl;
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
    float servo_open_position;
    float servo_close_position;
};

