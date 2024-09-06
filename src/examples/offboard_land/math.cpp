#include "math.h"
#include "Vector4.h"
#include <cmath>

bool is_zero(float _ki) {
    return _ki == 0;
}
bool is_positive(float dt) {
    return dt >= 0;
}
bool is_negative(float _error) {
    return _error <= 0;
}
bool is_equal(Vector4f a, Vector4f b, float tolerance) {
    return abs(a.x - b.x) < tolerance && abs(a.y - b.y) < tolerance && abs(a.z - b.z) < tolerance && abs(a.yaw - b.yaw) < tolerance;
}

float constrain_float(float _integrator, float _max, float _min) {
    if (_integrator > _max) {
        return _max;
    } else if (_integrator < _min) {
        return _min;
    } else {
        return _integrator;
    }
}


