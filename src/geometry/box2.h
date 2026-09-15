#pragma once

#include "math/vec2.h"
#include "collision/aabb2.h"

namespace zonai
{

struct box2
{
    vec2 halfExtents{};
};

inline aabb2 ComputeAABB( const box2& box )
{
    return
    {
        -box.halfExtents,
        box.halfExtents
    };
}

inline bool Contains( const box2& box, const vec2& point )
{
    return point.x >= -box.halfExtents.x && point.x <= box.halfExtents.x &&
        point.y >= -box.halfExtents.y && point.y <= box.halfExtents.y;
}

} // namespace zonai