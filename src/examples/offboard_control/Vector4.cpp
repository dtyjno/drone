#include <cmath>
#include "math.h"
#include "Vector4.h"

template <typename T>
bool Vector4<T>::operator ==(const Vector4<T> &v) const
{
    return (is_equal(x,v.x) && is_equal(y,v.y) && is_equal(z,v.z) && is_equal(yaw,v.yaw));
}

template <typename T>
bool Vector4<T>::operator !=(const Vector4<T> &v) const
{
    return (!is_equal(x,v.x) || !is_equal(y,v.y) || !is_equal(z,v.z) != is_equal(yaw,v.yaw));
}

template <typename T>
Vector4<T> Vector4<T>::operator -(void) const
{
    return Vector4<T>(-x,-y,-z, -yaw);
}

template <typename T>
Vector4<T> Vector4<T>::operator +(const Vector4<T> &v) const
{
    return Vector4<T>(x+v.x, y+v.y, z+v.z, yaw+v.yaw);
}

template <typename T>
Vector4<T> Vector4<T>::operator -(const Vector4<T> &v) const
{
    return Vector4<T>(x-v.x, y-v.y, z-v.z, yaw-v.yaw);
}

template <typename T>
Vector4<T> Vector4<T>::operator *(const T num) const
{
    return Vector4<T>(x*num, y*num, z*num, yaw*num);
}

template <typename T>
Vector4<T> Vector4<T>::operator /(const T num) const
{
    return Vector4<T>(x/num, y/num, z/num, yaw/num);
}

template <typename T>
Vector4<T> &Vector4<T>::operator +=(const Vector4<T> &v)
{
    x+=v.x; y+=v.y; z+=v.z; yaw+=v.yaw;
    return *this;
}

template <typename T>
Vector4<T> &Vector4<T>::operator -=(const Vector4<T> &v)
{
    x -= v.x; y -= v.y; z -= v.z; yaw -= v.yaw;
    return *this;
}

template <typename T>
Vector4<T> &Vector4<T>::operator *=(const T num)
{
    x*=num; y*=num; z*=num; yaw*=num;
    return *this;
}

template <typename T>
Vector4<T> &Vector4<T>::operator *=(const Vector4<T> &v)
{
    x *= v.x; y *= v.y; z *= v.z; yaw *= v.yaw;
    return *this;
}

template <typename T>
Vector4<T> &Vector4<T>::operator /=(const T num)
{
    x /= num; y /= num; z /= num; yaw /= num;
    return *this;
}

#include <iostream>
// rotate vector by angle in radians in xy plane leaving z untouched
template <typename T>
void Vector4<T>::rotate_xy(T angle_rad)
{
    const T cs = cos(angle_rad);
    const T sn = sin(angle_rad);
    T rx = x * cs - y * sn;
    T ry = x * sn + y * cs;
    std::cout << "rotate_xy: x: " << x << " y: " << y <<
    " rx " << rx <<" ry: "<< ry <<std::endl;
    x = rx;
    y = ry;
}


// define for float and double
template class Vector4<float>;
template class Vector4<double>;

// define needed ops for Vector4l, Vector4i as needed
template Vector4<int32_t> &Vector4<int32_t>::operator +=(const Vector4<int32_t> &v);
template bool Vector4<int16_t>::operator ==(const Vector4<int16_t> &v) const;
