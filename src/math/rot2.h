#pragma once

#include <cmath>

#include "vec2.h"

namespace zonai
{
    struct rot2
    {
        float c = 1.0f;
        float s = 0.0f;

        static rot2 FromRadians(float radians)
        {
            return
            {
                std::cos(radians),
                std::sin(radians)
            };
        }
    };

    inline vec2 Rotate(const rot2& rotation, const vec2& vector)
    {
        return
        {
            rotation.c * vector.x - rotation.s * vector.y,
            rotation.s * vector.x + rotation.c * vector.y
        };
    }

    inline vec2 InverseRotate(const rot2& rotation, const vec2& vector)
    {
        return
        {
            rotation.c * vector.x + rotation.s * vector.y,
            -rotation.s * vector.x + rotation.c * vector.y
        };
    }

    inline rot2 operator*(const rot2& a, const rot2& b)
    {
        return
        {
            a.c * b.c - a.s * b.s,
            a.s * b.c + a.c * b.s
        };
    }
}

// x' = c * x - s * y;
// y' = s * x + c * y;