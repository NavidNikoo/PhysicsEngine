//
// Created by Navid Nikoo on 5/7/24.
//

#ifndef VEC4_H
#define VEC4_H

class Vec4 {
public:
    Vec4();
    Vec4( const float value );
    Vec4( float X, float Y, float Z, float W);
    Vec4( const float* rhs );

    Vec4& operator=( const Vec4 &rhs );
    bool operator==( const Vec4 &rhs ) const;
    bool operator!=( const Vec4 &rhs ) const;
    Vec4 operator+( const Vec4 &rhs) const;
    const Vec4& operator+=( const Vec4 &rhs);
    const Vec4& operator-=( const Vec4 &rhs );
    const Vec4& operator*=( const Vec4 &rhs );
    const Vec4& operator/=( const Vec4 &rhs );
    const Vec4& operator-( const Vec4 &rhs ) const;
    Vec4 operator*( const float rhs ) const;
    float operator[]( const int idx ) const;
    float& operator[]( const int idx );

    float Dot( const Vec4& rhs ) const;
    const Vec4& Normalize();
    float GetMagnitude() const;
    bool IsValid() const;
    void Zero() { x = 0; y = 0; z = 0; w = 0; }

    const float* ToPtr() const { return &x; }
    float* ToPtr() { return &x; }

public:
    float x;
    float y;
    float z;
    float w;
};


#endif //VEC4_H
