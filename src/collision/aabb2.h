#pragma once

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

inline bool Contains( const aabb2& box, const vec2& point )
{
    return
        point.x >= box.min.x &&
        point.x <= box.max.x &&
        point.y >= box.min.y &&
        point.y <= box.max.y;
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