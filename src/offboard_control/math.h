#ifndef MATH_H // 如果MATH_H没有被定义
#define MATH_H // 定义MATH_H

#include <cmath>
#include "math_utils.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

// 使用 Eigen 固定尺寸向量模板
typedef Matrix<float, 2, 1> Vector2f;  // 二维浮点向量
typedef Matrix<float, 3, 1> Vector3f;  // 三维浮点向量
typedef Matrix<float, 4, 1> Vector4f;  // 四维浮点向量
typedef Matrix<float, 4, 1> Vector4b;  // 四维布尔向量
typedef Matrix<float, 3, 1> Vector3b;  // 四维布尔向量
typedef Quaternion<float> Quaternionf;  // 四元数类型


// #include "Location.h"
// #include "vectorN.h"

// #define PI 3.1415

// bool is_zero(float _ki);
// bool is_positive(float dt);
// bool is_negative(float _error);

#undef MIN
template<typename A, typename B>
static inline auto MIN(const A &one, const B &two) -> decltype(one < two ? one : two)
{
    return one < two ? one : two;
}

#undef MAX
template<typename A, typename B>
static inline auto MAX(const A &one, const B &two) -> decltype(one > two ? one : two)
{
    return one > two ? one : two;
}

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

float kinematic_limit(Vector3f direction, float max_xy, float max_z_pos, float max_z_neg);


typedef float ftype;
template<typename T>
ftype sq(const T val)
{
    ftype v = static_cast<ftype>(val);
    return v*v;
}
static inline constexpr float sq(const float val)
{
    return val*val;
}

/*
 * A variant of sqrt() that checks the input ranges and ensures a valid value
 * as output. If a negative number is given then 0 is returned.  The reasoning
 * is that a negative number for sqrt() in our code is usually caused by small
 * numerical rounding errors, so the real input should have been zero
 */
template <typename T>
float safe_sqrt(const T v);

template <typename T>
T norm(const T& x, const T& y, const T& z) {
    return sqrt(x*x + y*y + z*z);
}
template <typename T>
T norm(const T& x, const T& y) {
    return sqrt(x*x + y*y);
}

/* Return nonzero value if X is not +-Inf or NaN.  */
# if (__GNUC_PREREQ (4,4) && !defined __SUPPORT_SNAN__) \
     || __glibc_clang_prereq (2,8)
#  define isfinite(x) __builtin_isfinite (x)
# else
#  define isfinite(x) __MATH_TG ((x), __finite, (x))
# endif
#endif // MATH_H