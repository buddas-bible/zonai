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
}; // sizeof: 16 bytes

inline vec2 Direction( const segment2& segment )
{
    return segment.b - segment.a;
}

inline float LengthSquared( const segment2& segment )
{
    return zonai::LengthSquared( Direction( segment ) );
}

inline float Length( const segment2& segment )
{
    return zonai::Length( Direction( segment ) );
}

inline aabb2 ComputeAABB( const segment2& segment )
{
    return
    {
        {
            std::min( segment.a.x, segment.b.x ),
            std::min( segment.a.y, segment.b.y )
        },
        {
            std::max( segment.a.x, segment.b.x ),
            std::max( segment.a.y, segment.b.y )
        }
    };
}

inline vec2 ClosestPoint( const segment2& segment, const vec2& point )
{
    const vec2 ab = segment.b - segment.a;
    const float lengthSquared = zonai::LengthSquared( ab );

    if( lengthSquared == 0.0f )
    {
        return segment.a;
    }

    float t = zonai::Dot( point - segment.a, ab ) / lengthSquared;

    t = std::clamp( t, 0.0f, 1.0f );

    return segment.a + ab * t;
}

inline float DistanceSquared( const segment2& segment, const vec2& point )
{
    const vec2 closest = zonai::ClosestPoint( segment, point );

    return zonai::LengthSquared( point - closest );
}

} // namespace zonai