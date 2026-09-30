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

        const float temp = 1.0f / scalar;

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

    const float temp = 1.0f / scalar;
    value *= temp;
    return value;
}

// 벡터 a와 b의 내적(dot product)을 계산해 반환함.
inline float Dot( const vec2& a, const vec2& b )
{
    return a.x * b.x + a.y * b.y;
}

// 벡터 a와 b의 외적(cross product)을 계산해 z축 scalar를 반환함.
inline float Cross( const vec2& a, const vec2& b )
{
    return a.x * b.y - a.y * b.x;
}

// z축 scalar와 XY 벡터의 외적을 계산함.
// angular velocity * 위치 벡터로 회전에 의한 선속도를 구할 때 사용함.
inline vec2 Cross( float scalar, const vec2& vector )
{
    return
    {
        -scalar * vector.y,
         scalar * vector.x
    };
}

// XY 벡터와 z축 scalar의 외적을 계산함.
inline vec2 Cross( const vec2& vector, float scalar )
{
    return
    {
         scalar * vector.y,
        -scalar * vector.x
    };
}

// 벡터 value의 길이 제곱을 반환함.
inline float LengthSquared( const vec2& value )
{
    return Dot( value, value );
}

// 벡터 value의 길이를 반환함.
inline float Length( const vec2& value )
{
    return std::sqrt( LengthSquared( value ) );
}

// 벡터 value를 정규화해 반환함.
inline vec2 Normalize( const vec2& value )
{
    const float length = Length( value );

    if( length == 0.0f )
    {
        return {};
    }

    return value / length;
}

} // namespace zonai
