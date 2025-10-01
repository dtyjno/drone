#pragma once

#include "Camera.h"
#include "CameraGimbalInterface.h"

class CameraGimbal : public Camera, public CameraGimbalInterface {
public:
    CameraGimbal() : gimbal_pitch(0), gimbal_roll(0), gimbal_yaw(0), is_gimbal_ready(false) {}

    void set_gimbal(float pitch, float roll, float yaw) override {
        gimbal_pitch = pitch;
        gimbal_roll = roll;
        gimbal_yaw = yaw;
    }
    void use_gimbal(bool is_gimbal_ready) override {
        this->is_gimbal_ready = is_gimbal_ready;
    }
    double get_gimbal_pitch() const override { return gimbal_pitch; }
    double get_gimbal_roll() const override { return gimbal_roll; }
    double get_gimbal_yaw() const override { return gimbal_yaw; }

protected:
    double gimbal_pitch, gimbal_roll, gimbal_yaw;
    bool is_gimbal_ready;
};

