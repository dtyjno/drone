#ifndef VECTOR2_H
#define VECTOR2_H
#include <cstdint>

template <typename T>
class Vector2
{
public:
    T x,y;

    Vector2(){}
    Vector2(T x, T y) : x(x), y(y) {}

    // test for equality
    bool operator ==(const Vector2<T> &v) const;

    // test for inequality
    bool operator !=(const Vector2<T> &v) const;

    // negation
    Vector2<T> operator -(void) const;

    // addition
    Vector2<T> operator +(const Vector2<T> &v) const;

    // subtraction
    Vector2<T> operator -(const Vector2<T> &v) const;

    // uniform scaling
    Vector2<T> operator *(const T num) const;

    T operator *(const Vector2<T> &v) const {
        return x * v.x + y * v.y;
    }

    // uniform scaling
    Vector2<T> operator  /(const T num) const;

    // addition
    Vector2<T> &operator +=(const Vector2<T> &v);

    // subtraction
    Vector2<T> &operator -=(const Vector2<T> &v);

    // uniform scaling
    Vector2<T> &operator *=(const T num);

    // uniform scaling
    Vector2<T> &operator /=(const T num);

    // non-uniform scaling
    Vector2<T> &operator *=(const Vector2<T> &v);

    // cross product
    T operator %(const Vector2<T> &v) const;

    // rotate vector by angle in radians in xy plane leaving z untouched
    void rotate_xy(T rotation_rad);
    void zero(){
        x = 0;
        y = 0;
    }
    bool is_zero(void) const{
        return x == 0 && y == 0;
    }
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
    Vector2<T> normalized() const
    {
        return *this/length();
    }

\
    
};
// template<> inline bool Vector2<float>::is_zero(void) const {
//     return ::is_zero(x) && ::is_zero(y) && ::is_zero(z);
// }
// template<> inline bool Vector2<double>::is_zero(void) const {
//     return ::is_zero(x) && ::is_zero(y) && ::is_zero(z);
// }

typedef Vector2<int16_t>                Vector2i;
typedef Vector2<uint16_t>               Vector2ui;
typedef Vector2<int32_t>                Vector2l;
typedef Vector2<uint32_t>               Vector2ul;
typedef Vector2<float>                  Vector2f;
typedef Vector2<double>                 Vector2d;

#endif // VECTOR2_H