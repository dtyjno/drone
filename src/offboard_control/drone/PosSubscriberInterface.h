#pragma once

#include "../utils/math.h"

#define DEFAULT_POS INFINITY

class PosSubscriberInterface {
public:
    virtual ~PosSubscriberInterface() = default;

    virtual Vector3f get_position() = 0;
    virtual void set_position(Vector3f pos) = 0;

	virtual Vector3f get_velocity() = 0;
	virtual void set_velocity(Vector3f vel) = 0;
	// Add method for yaw velocity
	virtual float get_velocity_yaw() = 0;
	virtual void set_velocity_yaw(float yaw_vel) = 0;
	virtual Quaternionf get_orientation() = 0;
	virtual Vector3f get_gps() = 0;
	virtual void set_gps(Vector3f g) = 0;
	virtual float get_altitude() = 0;
	virtual void set_altitude(float alt) = 0;
	virtual Vector3f get_linear_acceleration() = 0;
	virtual void set_linear_acceleration(Vector3f la) = 0;
	virtual Vector3f get_angular_velocity() = 0;
	virtual void set_angular_velocity(Vector3f av) = 0;
	virtual float get_rangefinder_height() = 0;
	virtual void set_rangefinder_height(float rf) = 0;
	virtual float get_roll() = 0;
	virtual float get_pitch() = 0;
	virtual float get_yaw() = 0;
	virtual void set_yaw(float y) = 0;
	virtual void set_rpy(float r, float p, float y) = 0;
};