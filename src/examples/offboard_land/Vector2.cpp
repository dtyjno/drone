#include <cmath>
#include "math.h"
#include "Vector2.h"

template <typename T>
Vector2<T> &Vector2<T>::operator +=(const Vector2<T> &v)
{
    x+=v.x; y+=v.y;
    return *this;
}

template <typename T>
Vector2<T> &Vector2<T>::operator *=(const T num)
{
    x*=num; y*=num; 
    return *this;
}

template <typename T>
Vector2<T> &Vector2<T>::operator /=(const T num)
{
    x /= num; y /= num; 
    return *this;
}

template <typename T>
Vector2<T> &Vector2<T>::operator -=(const Vector2<T> &v)
{
    x -= v.x; y -= v.y;
    return *this;
}

template <typename T>
Vector2<T> &Vector2<T>::operator *=(const Vector2<T> &v)
{
    x *= v.x; y *= v.y; 
    return *this;
}

template <typename T>
Vector2<T> Vector2<T>::operator /(const T num) const
{
    return Vector2<T>(x/num, y/num);
}

template <typename T>
Vector2<T> Vector2<T>::operator *(const T num) const
{
    return Vector2<T>(x*num, y*num);
}

template <typename T>
Vector2<T> Vector2<T>::operator -(const Vector2<T> &v) const
{
    return Vector2<T>(x-v.x, y-v.y);
}

template <typename T>
Vector2<T> Vector2<T>::operator +(const Vector2<T> &v) const
{
    return Vector2<T>(x+v.x, y+v.y);
}

template <typename T>
Vector2<T> Vector2<T>::operator -(void) const
{
    return Vector2<T>(-x,-y);
}

template <typename T>
bool Vector2<T>::operator ==(const Vector2<T> &v) const
{
    return (is_equal(x,v.x) && is_equal(y,v.y));
}

template <typename T>
bool Vector2<T>::operator !=(const Vector2<T> &v) const
{
    return (!is_equal(x,v.x) || !is_equal(y,v.y));
}

// cross product
template <typename T>
T Vector2<T>::operator %(const Vector2<T> &v) const
{
    return x*v.y - y*v.x;
}

// rotate vector by angle in radians in xy plane leaving z untouched
template <typename T>
void Vector2<T>::rotate_xy(T angle_rad)
{
    const T cs = cos(angle_rad);
    const T sn = sin(angle_rad);
    T rx = x * cs - y * sn;
    T ry = x * sn + y * cs;
    x = rx;
    y = ry;
}

// template <typename T>
// void Vector2<T>::zero()
// {
//     x = y = 0;
// }

template <typename T>
T Vector2<T>::length(void) const
{
    return norm(x, y);
}


// define for float and double
template class Vector2<float>;
template class Vector2<double>;

// define needed ops for Vector2l, Vector2i as needed
template Vector2<int32_t> &Vector2<int32_t>::operator +=(const Vector2<int32_t> &v);
template bool Vector2<int16_t>::operator ==(const Vector2<int16_t> &v) const;
