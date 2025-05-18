#include "math.h"
// #include "Vector4.h"
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
    return abs(a.x() - b.x()) < tolerance && abs(a.y() - b.y()) < tolerance && abs(a.z() - b.z()) < tolerance && abs(a.w() - b.w()) < tolerance;
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


// kinematic_limit calculates the maximum acceleration or velocity in a given direction.
// based on horizontal and vertical limits.
float kinematic_limit(Vector3f direction, float max_xy, float max_z_pos, float max_z_neg)
{
    if (is_zero(direction.squaredNorm()) || is_zero(max_xy) || is_zero(max_z_pos) || is_zero(max_z_neg)) {
        return 0.0;
    }

    max_xy = fabsf(max_xy);
    max_z_pos = fabsf(max_z_pos);
    max_z_neg = fabsf(max_z_neg);

    direction.normalize();
    const float xy_length = sqrt(pow(direction.x(), 2) + pow(direction.y(), 2));

    if (is_zero(xy_length)) {
        return is_positive(direction.z()) ? max_z_pos : max_z_neg;
    }

    if (is_zero(direction.z())) {
        return max_xy;
    }

    const float slope = direction.z()/xy_length;
    if (is_positive(slope)) {
        if (fabsf(slope) < max_z_pos/max_xy) {
            return max_xy/xy_length;
        }
        return fabsf(max_z_pos/direction.z());
    }

    if (fabsf(slope) < max_z_neg/max_xy) {
        return max_xy/xy_length;
    }
    return fabsf(max_z_neg/direction.z());
}

template <typename T>
float safe_sqrt(const T v)
{
    float ret = sqrtf(static_cast<float>(v));
    if (std::isnan(ret)) {
        return 0;
    }
    return ret;
}

template float safe_sqrt<int>(const int v);
template float safe_sqrt<short>(const short v);
template float safe_sqrt<float>(const float v);
template float safe_sqrt<double>(const double v);