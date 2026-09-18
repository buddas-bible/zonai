#pragma once

#include <algorithm>

#include "math/vec2.h"

namespace zonai
{

struct aabb2
{
    vec2 min{};
    vec2 max{};
};

inline vec2 Center( const aabb2& box )
{
    return ( box.min + box.max ) * 0.5f;
}

inline vec2 Extents( const aabb2& box )
{
    return ( box.max - box.min ) * 0.5f;
}

inline aabb2 Union( const aabb2& a, const aabb2& b )
{
    return
    {
        {
            std::min( a.min.x, b.min.x ),
            std::min( a.min.y, b.min.y )
        },
        {
            std::max( a.max.x, b.max.x ),
            std::max( a.max.y, b.max.y )
        }
    };
}

inline float Perimeter( const aabb2& box )
{
    const float width = box.max.x - box.min.x;
    const float height = box.max.y - box.min.y;

    return 2.0f * ( width + height );
}

inline bool Contains( const aabb2& box, const vec2& point )
{
    return
        point.x >= box.min.x &&
        point.x <= box.max.x &&
        point.y >= box.min.y &&
        point.y <= box.max.y;
}

inline bool Contains( const aabb2& outer, const aabb2& inner )
{
    return
        inner.min.x >= outer.min.x &&
        inner.min.y >= outer.min.y &&
        inner.max.x <= outer.max.x &&
        inner.max.y <= outer.max.y;
}

inline bool Overlaps( const aabb2& a, const aabb2& b )
{
    if( a.max.x < b.min.x || a.min.x > b.max.x )
        return false;

    if( a.max.y < b.min.y || a.min.y > b.max.y )
        return false;

    return true;
}

} // namespace zonai
