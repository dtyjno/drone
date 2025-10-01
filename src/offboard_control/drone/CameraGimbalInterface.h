#pragma once


class CameraGimbalInterface {
public:
    virtual ~CameraGimbalInterface() = default;

    // pitch, roll, yaw 单位为角度制
    virtual void set_gimbal(float pitch, float roll, float yaw) = 0;

    // 设置云台是否准备好
    virtual void use_gimbal(bool is_gimbal_ready) = 0;

    // 获取 pitch, roll, yaw
    virtual double get_gimbal_pitch() const = 0;
    virtual double get_gimbal_roll() const = 0;
    virtual double get_gimbal_yaw() const = 0;

    // 获取云台是否准备好
    // virtual bool isGimbalReady() const = 0;
};