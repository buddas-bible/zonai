#pragma once
#include <cmath>

namespace zonai
{
    struct vec2
    {
        float x = 0.0f;
        float y = 0.0f;

        vec2& operator+=(const vec2& rhs)
        {
            x += rhs.x;
            y += rhs.y;
            return *this;
        }

        vec2& operator-=(const vec2& rhs)
        {
            x -= rhs.x;
            y -= rhs.y;
            return *this;
        }

        vec2& operator*=(float scalar)
        {
            x *= scalar;
            y *= scalar;
            return *this;
        }

        vec2& operator/=(float scalar)
        {
            assert(scalar != 0.0f);

			float temp = 1.0f / scalar;

            x *= temp;
            y *= temp;
            return *this;
        }
    };

    inline vec2 operator+(vec2 lhs, const vec2& rhs)
    {
        lhs += rhs;
        return lhs;
    }

    inline vec2 operator-(vec2 lhs, const vec2& rhs)
    {
        lhs -= rhs;
        return lhs;
    }

    inline vec2 operator-(const vec2& value)
    {
        return { -value.x, -value.y };
    }

    inline vec2 operator*(vec2 value, float scalar)
    {
        value *= scalar;
        return value;
    }

    inline vec2 operator*(float scalar, vec2 value)
    {
        value *= scalar;
        return value;
    }

    inline vec2 operator/(vec2 value, float scalar)
    {
        assert(scalar != 0.0f);

		float temp = 1.0f / scalar;
        value *= temp;
        return value;
    }

    inline float Dot(const vec2& a, const vec2& b)
    {
        return a.x * b.x + a.y * b.y;
    }

    inline float Cross(const vec2& a, const vec2& b)
    {
        return a.x * b.y - a.y * b.x;
    }

    inline float LengthSquared(const vec2& value)
    {
        return Dot(value, value);
    }

    inline float Length(const vec2& value)
    {
        return std::sqrt(LengthSquared(value));
    }

    inline vec2 Normalize(const vec2& value)
    {
        const float length = Length(value);

        if (length == 0.0f)
        {
            return {};
        }

        return value / length;
    }
}