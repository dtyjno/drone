// #include <cmath>
#include "math.h"
// #include "Vector3.h"
#include "OffboardControl_Base.h"

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

template <typename T>
void Vector3<T>::zero()
{
    x = y = z = 0;
}

template <typename T>
T Vector3<T>::length(void) const
{
    return norm(x, y, z);
}

// return horizontal distance in meters between two locations
template <typename T>
T Vector3<T>::get_distance( Vector3<T> &loc2) const
{
    return norm(loc2.x - x, loc2.y - y);
}

template <typename T>
Vector2<T> Vector3<T>::get_distance_NE( Vector3<T> &loc2) const
{
    return {loc2.x - x,loc2.y - y};
}

template <typename T>
bool Vector3<T>::get_distance_NE( Vector2<T> &loc2) const
{
    loc2 = {loc2.x - x,loc2.y - y};
    return loc2.is_zero();
}


// return bearing in radians from location to loc2, return is 0 to 2*Pi
template <typename T>
T Vector3<T>::get_bearing( Vector3<T> &loc2) const
{
    const int32_t off_x = loc2.x - x;
    const int32_t off_y = loc2.y - y;
    ftype bearing = (M_PI*0.5) + atan2(-off_y, off_x);
    if (bearing < 0) {
        bearing += 2*M_PI;
    }
    return bearing;
}

template <typename T>
bool Vector3<T>::get_vector_xy_from_origin_NE(Vector2<T> &vec_ne) const{
    Vector2f ekf_origin = {OffboardControl_Base::start.x, OffboardControl_Base::start.y};
    // if (!AP::ahrs().get_origin(ekf_origin)) {
    //     return false;
    // }
    vec_ne.x = (x-ekf_origin.x) ;
    vec_ne.y = (y-ekf_origin.y) ;
    return true;
}

// extrapolate latitude/longitude given distances (in meters) north and east
template <typename T>
void Vector3<T>::offset(float ofs_north, float ofs_east)
{
    x+=ofs_east;
    y+=ofs_north;
}



// define for float and double
template class Vector3<float>;
template class Vector3<double>;

// define needed ops for Vector3l, Vector3i as needed
template Vector3<int32_t> &Vector3<int32_t>::operator +=(const Vector3<int32_t> &v);
template bool Vector3<int16_t>::operator ==(const Vector3<int16_t> &v) const;
