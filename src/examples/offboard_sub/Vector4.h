#ifndef VECTOR4_H
#define VECTOR4_H
#include <cstdint>

template <typename T>
class Vector4
{
public:
    T x,y,z,yaw;

    Vector4(){}
    Vector4(T x, T y, T z, T yaw) : x(x), y(y), z(z), yaw(yaw) {}

    // test for equality
    bool operator ==(const Vector4<T> &v) const;

    // test for inequality
    bool operator !=(const Vector4<T> &v) const;

    // subtraction
    Vector4<T> operator -(const Vector4<T> &v) const;

    // addition
    Vector4<T> operator +(const Vector4<T> &v) const;

    // negation
    Vector4<T> operator -(void) const;

    // uniform scaling
    Vector4<T> operator *(const T num) const;

    // uniform scaling
    Vector4<T> operator  /(const T num) const;

    // addition
    Vector4<T> &operator +=(const Vector4<T> &v);

    // subtraction
    Vector4<T> &operator -=(const Vector4<T> &v);

    // uniform scaling
    Vector4<T> &operator *=(const T num);

    // non-uniform scaling
    Vector4<T> &operator *=(const Vector4<T> &v);

    // uniform scaling
    Vector4<T> &operator /=(const T num);

    // rotate vector by angle in radians in xy plane leaving z untouched
    void rotate_xy(T rotation_rad);
};
typedef Vector4<int16_t>                Vector4i;
typedef Vector4<uint16_t>               Vector4ui;
typedef Vector4<int32_t>                Vector4l;
typedef Vector4<uint32_t>               Vector4ul;
typedef Vector4<float>                  Vector4f;
typedef Vector4<double>                 Vector4d;
#endif // VECTOR4_H