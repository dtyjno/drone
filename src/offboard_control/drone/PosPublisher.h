#pragma once

#include "../utils/math.h"
// #include "../drone_interface/PosPublisherInterface.h"

class PosPublisher
{
public:
    PosPublisher() = default;
    ~PosPublisher() = default;

    virtual void publish_setpoint_raw(Vector4f p, Vector4f v) = 0;
    virtual void publish_setpoint_raw_global(double latitude, double longitude, double altitude, double yaw) = 0;
    virtual void send_local_setpoint_command(double x, double y, double z, double yaw) = 0;
    virtual bool local_setpoint_command(Vector4f now, Vector4f target, double accuracy) = 0;
    virtual void send_velocity_command(Vector4f v) = 0;
    virtual bool send_velocity_command_with_time(Vector4f v, double time) = 0;
    virtual void send_accel_command(Vector4f v) = 0;

};


