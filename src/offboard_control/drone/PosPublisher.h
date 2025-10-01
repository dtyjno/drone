#pragma once

#include "../utils/math.h"
#include "PosPublisherInterface.h"
#include "PosSubscriberInterface.h"   // <-- 添加这一行
// #include "../drone_interface/PosPublisherInterface.h"

class PosPublisher : public PosPublisherInterface
{
public:
    PosPublisher() = default;
    ~PosPublisher() = default;
    PosPublisher(std::shared_ptr<PosSubscriberInterface> pos_subscriber_, float wait_time)
    {
        this->pos_subscriber_ = pos_subscriber_;
        this->wait_time = wait_time;
    }
    std::shared_ptr<PosSubscriberInterface> pos_subscriber_;
    float wait_time;

    void publish_setpoint_raw(Vector4f p, Vector4f v) {
        pos_subscriber_->set_velocity(Vector3f{
            static_cast<float>((p.x() - pos_subscriber_->get_position().x()) > v.x() ? v.x() : (p.x() - pos_subscriber_->get_position().x())),
            static_cast<float>((p.y() - pos_subscriber_->get_position().y()) > v.y() ? v.y() : (p.y() - pos_subscriber_->get_position().y())),
            static_cast<float>((p.z() - pos_subscriber_->get_position().z()) > v.z() ? v.z() : (p.z() - pos_subscriber_->get_position().z()))
        });
        pos_subscriber_->set_velocity_yaw(static_cast<float>((p.w() - pos_subscriber_->get_yaw()) > v.w() ? v.w() : (p.w() - pos_subscriber_->get_yaw())));
        pos_subscriber_->set_position(Vector3f{static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())});
        pos_subscriber_->set_yaw(static_cast<float>(pos_subscriber_->get_velocity_yaw()));
    }
    void publish_setpoint_raw_global(double latitude, double longitude, double altitude, double yaw) {
        Vector3f gps{static_cast<float>(latitude), static_cast<float>(longitude), static_cast<float>(altitude)};
        pos_subscriber_->set_gps(gps);
        pos_subscriber_->set_yaw(static_cast<float>(yaw));
    }
    void send_local_setpoint_command(double x, double y, double z, double yaw) {
        pos_subscriber_->set_velocity(Vector3f{
            static_cast<float>((x - pos_subscriber_->get_position().x()) > 1.0f ? 1.0f : (x - pos_subscriber_->get_position().x())),
            static_cast<float>((y - pos_subscriber_->get_position().y()) > 1.0f ? 1.0f : (y - pos_subscriber_->get_position().y())),
            static_cast<float>((z - pos_subscriber_->get_position().z()) > 1.0f ? 1.0f : (z - pos_subscriber_->get_position().z()))
        });
        pos_subscriber_->set_velocity_yaw(static_cast<float>((yaw - pos_subscriber_->get_yaw()) > 1.0f ? 1.0f : (yaw - pos_subscriber_->get_yaw())));
        pos_subscriber_->set_position(Vector3f{static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
        pos_subscriber_->set_yaw(static_cast<float>(pos_subscriber_->get_velocity_yaw()));
    }
    bool local_setpoint_command(Vector4f now, Vector4f target, double accuracy) {
        send_local_setpoint_command(target.x(), target.y(), target.z(), target.w());
        (void)now; // 未使用参数
        (void)accuracy; // 未使用参数
        return true;
    }
    void send_velocity_command(Vector4f v) {
        std::cout << "send_velocity_command: v = [" << v.x() << ", " << v.y() << ", " << v.z() << ", " << v.w() << "]" << std::endl;
        pos_subscriber_->set_velocity(Vector3f{static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z())});
        Vector3f pos = pos_subscriber_->get_position();
        pos.x() += static_cast<float>(v.x() * 2);    //速度乘以 2 / wait_time
        pos.y() += static_cast<float>(v.y() * 2);
        pos.z() += static_cast<float>(v.z() * 2);
        pos_subscriber_->set_position(pos);
        pos_subscriber_->set_yaw(static_cast<float>(pos_subscriber_->get_yaw() + v.w() * wait_time));
    }
    bool send_velocity_command_with_time(Vector4f v, double time) {
        pos_subscriber_->set_velocity(Vector3f{static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z())});
        Vector3f pos = pos_subscriber_->get_position();
        pos.x() += static_cast<float>(v.x() * time);
        pos.y() += static_cast<float>(v.y() * time);
        pos.z() += static_cast<float>(v.z() * time);
        pos_subscriber_->set_position(pos);
        pos_subscriber_->set_yaw(static_cast<float>(pos_subscriber_->get_yaw() + v.w() * time));
        return true;
    }
    void send_accel_command(Vector4f v) {
        pos_subscriber_->set_linear_acceleration(Vector3f{static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z())});
        Vector3f vel = pos_subscriber_->get_velocity();
        vel.x() += static_cast<float>(v.x() * wait_time);
        vel.y() += static_cast<float>(v.y() * wait_time);
        vel.z() += static_cast<float>(v.z() * wait_time);
        pos_subscriber_->set_velocity(vel);
        Vector3f pos = pos_subscriber_->get_position();
        pos.x() += static_cast<float>(vel.x() * wait_time);
        pos.y() += static_cast<float>(vel.y() * wait_time);
        pos.z() += static_cast<float>(vel.z() * wait_time);
        pos_subscriber_->set_position(pos);
        pos_subscriber_->set_yaw(static_cast<float>(pos_subscriber_->get_yaw() + v.w() * wait_time));
    }
};


