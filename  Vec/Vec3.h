
#ifndef VEC3_H
#define VEC3_H


#include "Vec3.h"

class Vec3 {
public:
    Vec3();
    Vec3( float value );
    Vec3( const Vec3 &rhs );
    Vec3( float X, float Y, float Z );
    Vec3( const float* xyz );

    Vec3 & operator=( const Vec3 &rhs );
    Vec3 & operator=( const float * rhs );
    bool operator==( const Vec3 &rhs ) const;
    bool operator!=( const Vec3 &rhs) const;
    Vec3 operator+( const Vec3 &rhs ) const;
    const Vec3& operator+=( const Vec3 &rhs );
    const Vec3& operator-= ( const Vec3 &rhs );
    Vec3 operator-( const Vec3 &rhs ) const;
    Vec3 operator *( const float rhs) const;
    Vec3 operator /( const float rhs ) const;
    const Vec3& operator *=( const float rhs );
    const Vec3& operator /=( const float rhs);
    float operator[] ( const int idx ) const;
    float& operator [] ( const int idx );

    void Zero() { x = 0.0f; y = 0.0f, z = 0.0f; }
    Vec3 Cross( const Vec3& rhs ) const;
    float Dot( const Vec3& rhs ) const;

    const Vec3& Normalize();
    float GetMagnitude() const;
    bool IsValid() const;
    void GetOrtho( Vec3& u, Vec3& v ) const;

    const float * ToPtr() const { return &x; }

public:
    float x;
    float y;
    float z;
};

#endif //VEC3_H