#pragma once
#include <cassert>
#include <cmath>

namespace zonai
{

struct vec2
{
    float x = 0.0f;
    float y = 0.0f;

    vec2& operator+=( const vec2& rhs )
    {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    vec2& operator-=( const vec2& rhs )
    {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    vec2& operator*=( float scalar )
    {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    vec2& operator/=( float scalar )
    {
        assert( scalar != 0.0f );

        float temp = 1.0f / scalar;

        x *= temp;
        y *= temp;
        return *this;
    }
};

inline vec2 operator+( vec2 lhs, const vec2& rhs )
{
    lhs += rhs;
    return lhs;
}

inline vec2 operator-( vec2 lhs, const vec2& rhs )
{
    lhs -= rhs;
    return lhs;
}

inline vec2 operator-( const vec2& value )
{
    return { -value.x, -value.y };
}

inline vec2 operator*( vec2 value, float scalar )
{
    value *= scalar;
    return value;
}

inline vec2 operator*( float scalar, vec2 value )
{
    value *= scalar;
    return value;
}

inline vec2 operator/( vec2 value, float scalar )
{
    assert( scalar != 0.0f );

    float temp = 1.0f / scalar;
    value *= temp;
    return value;
}

// 벡터 a와 b의 내적(dot product)을 계산하여 반환합니다.
inline float Dot( const vec2& a, const vec2& b )
{
    return a.x * b.x + a.y * b.y;
}

// 벡터 a와 b의 외적(cross product)을 계산하여 반환합니다.
inline float Cross( const vec2& a, const vec2& b )
{
    return a.x * b.y - a.y * b.x;
}

// 벡터 value의 길이(length)의 제곱을 계산하여 반환합니다.
inline float LengthSquared( const vec2& value )
{
    return Dot( value, value );
}

// 벡터 value의 길이(length)를 계산하여 반환합니다.
inline float Length( const vec2& value )
{
    return std::sqrt( LengthSquared( value ) );
}

// 벡터 value를 정규화(normalize)하여 반환합니다.
inline vec2 Normalize( const vec2& value )
{
    const float length = Length( value );

    if( length == 0.0f )
    {
        return {};
    }

    return value / length;
}

}