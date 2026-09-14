#pragma once

namespace zonai
{
    struct Vec2
    {
        float x = 0.0f;
        float y = 0.0f;

        Vec2& operator+=(const Vec2& rhs)
        {
            x += rhs.x;
            y += rhs.y;
            return *this;
        }

        Vec2& operator-=(const Vec2& rhs)
        {
            x -= rhs.x;
            y -= rhs.y;
            return *this;
        }

        Vec2& operator*=(float scalar)
        {
            x *= scalar;
            y *= scalar;
            return *this;
        }

        Vec2& operator/=(float scalar)
        {
            x /= scalar;
            y /= scalar;
            return *this;
        }
    };

    inline Vec2 operator+(Vec2 lhs, const Vec2& rhs)
    {
        lhs += rhs;
        return lhs;
    }

    inline Vec2 operator-(Vec2 lhs, const Vec2& rhs)
    {
        lhs -= rhs;
        return lhs;
    }

    inline Vec2 operator-(const Vec2& value)
    {
        return { -value.x, -value.y };
    }

    inline Vec2 operator*(Vec2 value, float scalar)
    {
        value *= scalar;
        return value;
    }

    inline Vec2 operator*(float scalar, Vec2 value)
    {
        value *= scalar;
        return value;
    }

    inline Vec2 operator/(Vec2 value, float scalar)
    {
        value /= scalar;
        return value;
    }

    inline float Dot(const Vec2& a, const Vec2& b)
    {
        return a.x * b.x + a.y * b.y;
    }

    inline float Cross(const Vec2& a, const Vec2& b)
    {
        return a.x * b.y - a.y * b.x;
    }

    inline float LengthSquared(const Vec2& value)
    {
        return Dot(value, value);
    }

    inline float Length(const Vec2& value)
    {
        return std::sqrt(LengthSquared(value));
    }

    inline Vec2 Normalize(const Vec2& value)
    {
        const float length = Length(value);

        if (length == 0.0f)
        {
            return {};
        }

        return value / length;
    }
}