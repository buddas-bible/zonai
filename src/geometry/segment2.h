#pragma once

#include <algorithm>

#include "math/vec2.h"
#include "collision/aabb2.h"

namespace zonai
{
    struct segment2
    {
        vec2 a{};
        vec2 b{};
    };

    inline vec2 Direction(const segment2& segment)
    {
        return segment.b - segment.a;
    }

    inline float LengthSquared(const segment2& segment)
    {
        return zonai::LengthSquared(Direction(segment));
    }

    inline float Length(const segment2& segment)
    {
        return zonai::Length(Direction(segment));
    }

    inline aabb2 ComputeAABB(const segment2& segment)
    {
        return
        {
            {
                std::min(segment.a.x, segment.b.x),
                std::min(segment.a.y, segment.b.y)
            },
            {
                std::max(segment.a.x, segment.b.x),
                std::max(segment.a.y, segment.b.y)
            }
        };
    }
}