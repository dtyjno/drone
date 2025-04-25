#ifndef VECTOR3_H
#define VECTOR3_H
#include <cstdint>
#include "Vector2.h"
#include "math_utils.h"

template <typename T>
class Vector3
{
public:
    T x,y,z;

    Vector3(){}
    Vector3(T x, T y, T z) : x(x), y(y), z(z) {}

    // test for equality
    bool operator ==(const Vector3<T> &v) const;

    // test for inequality
    bool operator !=(const Vector3<T> &v) const;

    // negation
    Vector3<T> operator -(void) const;

    // addition
    Vector3<T> operator +(const Vector3<T> &v) const;

    // subtraction
    Vector3<T> operator -(const Vector3<T> &v) const;

    // uniform scaling
    Vector3<T> operator *(const T num) const;

    T operator *(const Vector3<T> &v) const {
        return x * v.x + y * v.y + z * v.z;
    }

    // uniform scaling
    Vector3<T> operator  /(const T num) const;

    // addition
    Vector3<T> &operator +=(const Vector3<T> &v);

    // subtraction
    Vector3<T> &operator -=(const Vector3<T> &v);

    // uniform scaling
    Vector3<T> &operator *=(const T num);

    // uniform scaling
    Vector3<T> &operator /=(const T num);

    // non-uniform scaling
    Vector3<T> &operator *=(const Vector3<T> &v);

    // rotate vector by angle in radians in xy plane leaving z untouched
    void rotate_xy(T rotation_rad);
    void zero();
    bool is_zero(void) const;
    // bool is_zero(void) const {
    //     return x == 0 && y == 0 && z == 0;
    // }
    T  length_squared() const
    {
        return (T)(*this * *this);
    }

    // gets the length of this vector
    T length(void) const;

    // limit xy component vector to a given length. returns true if vector was limited
    // bool limit_length_xy(T max_length);

    // normalizes this vector
    void normalize()
    {
        *this /= length();
    }
    // returns the normalized version of this vector
    Vector3<T> normalized() const
    {
        return *this/length();
    }
    T get_distance( Vector3<T> &loc2) const;
    Vector2<T> get_distance_NE( Vector3<T> &loc2) const;
    bool get_distance_NE( Vector2<T> &loc2) const;
    T get_bearing( Vector3<T> &loc2) const;
    // return bearing in centi-degrees from location to loc2, return is 0 to 35999
    int32_t get_bearing_to( Vector3<T> &loc2) const {
        // return int32_t(get_bearing(loc2) * DEGX100 + 0.5);
        return int32_t(get_bearing(loc2) * 100 + 0.5);
    }
    bool initialised() const { return (x !=0 || y != 0 || z != 0); }
    bool get_vector_xy_from_origin_NE(Vector2<T> &vec) const;
    void offset(float ofs_north, float ofs_east);

};
template<> inline bool Vector3<float>::is_zero(void) const {
    return ::is_zero(x) && ::is_zero(y) && ::is_zero(z);
}
template<> inline bool Vector3<double>::is_zero(void) const {
    return ::is_zero(x) && ::is_zero(y) && ::is_zero(z);
}

typedef Vector3<int16_t>                Vector3i;
typedef Vector3<uint16_t>               Vector3ui;
typedef Vector3<int32_t>                Vector3l;
typedef Vector3<uint32_t>               Vector3ul;
typedef Vector3<float>                  Vector3f;
typedef Vector3<double>                 Vector3d;
typedef Vector3<bool>                   Vector3b;

#endif // VECTOR3_H