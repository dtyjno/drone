#pragma once

#include <string>

class ServoControllerInterface {
public:
    virtual ~ServoControllerInterface() = default;

    // 设置舵机位置
    virtual void set_servo(int servo_number, float position) = 0;

    // 读取配置
    virtual void read_configs(const std::string &filename) = 0;

    // 设置/获取开/关位置
    virtual float set_servo_open(int index = 1) = 0;
    virtual float set_servo_close(int index = 1) = 0;
    virtual void set_servo_open_position(float position) = 0;
    virtual void set_servo_close_position(float position) = 0;
    virtual float get_servo_open_position() const = 0;
    virtual float get_servo_close_position() const = 0;
};