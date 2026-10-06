#pragma once

#include <cassert>
#include <cmath>
#include <limits>

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

        const float inverseScalar = 1.0f / scalar;

        x *= inverseScalar;
        y *= inverseScalar;

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
    value /= scalar;

    return value;
}

inline bool IsFinite( const vec2& value )
{
    return std::isfinite( value.x ) && std::isfinite( value.y );
}

inline float Dot( const vec2& a, const vec2& b )
{
    return a.x * b.x + a.y * b.y;
}

inline float Cross( const vec2& a, const vec2& b )
{
    return a.x * b.y - a.y * b.x;
}

// z scalar x XY vector.
inline vec2 Cross( float scalar, const vec2& vector )
{
    return { -scalar * vector.y, scalar * vector.x };
}

// XY vector x z scalar.
inline vec2 Cross( const vec2& vector, float scalar )
{
    return { scalar * vector.y, -scalar * vector.x };
}

inline float LengthSquared( const vec2& value )
{
    return Dot( value, value );
}

inline float Length( const vec2& value )
{
    return std::sqrt( LengthSquared( value ) );
}

inline vec2 Normalize( const vec2& value )
{
    const float lengthSquared = LengthSquared( value );

    if( lengthSquared <= 1000.0f * std::numeric_limits<float>::min() ) return {};

    const float inverseLength = 1.0f / std::sqrt( lengthSquared );

    return value * inverseLength;
}

} // namespace zonai
