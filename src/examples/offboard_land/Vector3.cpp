#include <cmath>
#include "math.h"
#include "Vector3.h"

template <typename T>
Vector3<T> &Vector3<T>::operator +=(const Vector3<T> &v)
{
    x+=v.x; y+=v.y; z+=v.z;
    return *this;
}

template <typename T>
Vector3<T> &Vector3<T>::operator *=(const T num)
{
    x*=num; y*=num; z*=num;
    return *this;
}

template <typename T>
Vector3<T> &Vector3<T>::operator /=(const T num)
{
    x /= num; y /= num; z /= num;
    return *this;
}

template <typename T>
Vector3<T> &Vector3<T>::operator -=(const Vector3<T> &v)
{
    x -= v.x; y -= v.y; z -= v.z;
    return *this;
}

template <typename T>
Vector3<T> &Vector3<T>::operator *=(const Vector3<T> &v)
{
    x *= v.x; y *= v.y; z *= v.z;
    return *this;
}

template <typename T>
Vector3<T> Vector3<T>::operator /(const T num) const
{
    return Vector3<T>(x/num, y/num, z/num);
}

template <typename T>
Vector3<T> Vector3<T>::operator *(const T num) const
{
    return Vector3<T>(x*num, y*num, z*num);
}

template <typename T>
Vector3<T> Vector3<T>::operator -(const Vector3<T> &v) const
{
    return Vector3<T>(x-v.x, y-v.y, z-v.z);
}

template <typename T>
Vector3<T> Vector3<T>::operator +(const Vector3<T> &v) const
{
    return Vector3<T>(x+v.x, y+v.y, z+v.z);
}

template <typename T>
Vector3<T> Vector3<T>::operator -(void) const
{
    return Vector3<T>(-x,-y,-z);
}

template <typename T>
bool Vector3<T>::operator ==(const Vector3<T> &v) const
{
    return (is_equal(x,v.x) && is_equal(y,v.y) && is_equal(z,v.z));
}

template <typename T>
bool Vector3<T>::operator !=(const Vector3<T> &v) const
{
    return (!is_equal(x,v.x) || !is_equal(y,v.y) || !is_equal(z,v.z));
}


// rotate vector by angle in radians in xy plane leaving z untouched
template <typename T>
void Vector3<T>::rotate_xy(T angle_rad)
{
    const T cs = cos(angle_rad);
    const T sn = sin(angle_rad);
    T rx = x * cs - y * sn;
    T ry = x * sn + y * cs;
    x = rx;
    y = ry;
}


// define for float and double
template class Vector3<float>;
template class Vector3<double>;

// define needed ops for Vector3l, Vector3i as needed
template Vector3<int32_t> &Vector3<int32_t>::operator +=(const Vector3<int32_t> &v);
template bool Vector3<int16_t>::operator ==(const Vector3<int16_t> &v) const;
