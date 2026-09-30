#pragma once

#include "math/vec2.h"
#include "collision/aabb2.h"

namespace zonai
{

struct circle2
{
    vec2 center{};
    float radius = 0.0f;
}; // sizeof: 12 bytes

inline aabb2 ComputeAABB( const circle2& circle )
{
    const vec2 radius{ circle.radius, circle.radius };

    return
    {
        circle.center - radius,
        circle.center + radius
    };
}

inline bool Contains( const circle2& circle, const vec2& point )
{
    const vec2 difference = point - circle.center;

    return LengthSquared( difference ) <=
        circle.radius * circle.radius;
}

} // namespace zonai