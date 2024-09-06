#ifndef MATH_H // 如果MATH_H没有被定义
#define MATH_H // 定义MATH_H

// #include "Eigen/Eigen"
#include <cmath>
#include "Vector4.h"
#include "Vector3.h"

// #define PI 3.1415

bool is_zero(float _ki);
bool is_positive(float dt);
bool is_negative(float _error);
template <typename T>
T min(T a, T b) {
    return a < b ? a : b;
}
template <typename T>
T max(T a, T b) {
    return a > b ? a : b;
}
// bool is_equal(float a, float b, float tolerance = 0.001);
template <typename T>
bool is_equal(T a, T b, T tolerance = 0.001) {
    return std::fabs(a - b) < tolerance;
}
bool is_equal(Vector4f a, Vector4f b, float tolerance = 0.001);
float constrain_float(float _integrator, float _max, float _min);

// rotate vector by angle in radians in xy plane leaving z untouched
template <typename T>
void rotate_xy(T &x,T &y,float rotation_rad)
{
    const T cs = cos(rotation_rad);
    const T sn = sin(rotation_rad);
    T rx = x * cs - y * sn;
    T ry = x * sn + y * cs;
    x = rx;
    y = ry;
}


#endif // MATH_H