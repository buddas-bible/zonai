#pragma once

#include <algorithm>

#include "math/vec2.h"
#include "geometry/segment2.h"
#include "collision/aabb2.h"

namespace zonai
{

struct capsule2
{
    vec2 center1{};
    vec2 center2{};
    float radius = 0.0f;
};

inline aabb2 ComputeAABB( const capsule2& capsule )
{
    vec2 min = { std::min( capsule.center1.x, capsule.center2.x ) - capsule.radius, std::min( capsule.center1.y, capsule.center2.y ) - capsule.radius };
    vec2 max = { std::max( capsule.center1.x, capsule.center2.x ) + capsule.radius, std::max( capsule.center1.y, capsule.center2.y ) + capsule.radius };
    return { min, max };
}

inline bool Contains( const capsule2& capsule, const vec2& point )
{
    const zonai::segment2 axis{ capsule.center1, capsule.center2 };

    return zonai::DistanceSquared( axis, point ) <= capsule.radius * capsule.radius;
}

} // namespace zonai