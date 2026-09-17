#include "geometry/polygon2.h"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace zonai
{

polygon2 MakeBox( const vec2& halfExtents )
{
    const std::array<vec2, 4> vertices =
    {
        vec2{ -halfExtents.x, -halfExtents.y },
        vec2{  halfExtents.x, -halfExtents.y },
        vec2{  halfExtents.x,  halfExtents.y },
        vec2{ -halfExtents.x,  halfExtents.y }
    };

    return MakePolygon( vertices );
}

polygon2 MakeCapsule( const vec2& center1, const vec2& center2, float radius )
{
    polygon2 capsule{};

    const vec2 direction = center2 - center1;

    if( LengthSquared( direction ) <= FLT_EPSILON )
    {
        assert( false );
        return capsule;
    }

    const vec2 axis = Normalize( direction );
    const vec2 normal{ axis.y, -axis.x };

    capsule.vertices[0] = center1;
    capsule.vertices[1] = center2;
    capsule.normals[0] = normal;
    capsule.normals[1] = -normal;
    capsule.centroid = ( center1 + center2 ) * 0.5f;
    capsule.radius = radius;
    capsule.vertexCount = 2;

    return capsule;
}

polygon2 MakePolygon( std::span<const vec2> vertices )
{
    polygon2 polygon{};

    if( vertices.size() < 3 || vertices.size() > MAX_POLYGON_VERTICES )
    {
        assert( false );
        return polygon;
    }

    polygon.vertexCount = vertices.size();

    for( std::size_t i = 0; i < polygon.vertexCount; ++i )
    {
        polygon.vertices[i] = vertices[i];
    }

    float area2 = 0.0f;

    for( std::size_t i = 0; i < polygon.vertexCount; ++i )
    {
        const std::size_t next = ( i + 1 ) % polygon.vertexCount;
        area2 += Cross( polygon.vertices[i], polygon.vertices[next] );
    }

    constexpr float epsilon = 1e-6f;

    if( std::fabs( area2 ) <= epsilon )
    {
        assert( false );
        return {};
    }

    if( area2 < 0.0f )
    {
        std::reverse(
            polygon.vertices.begin(),
            polygon.vertices.begin() + polygon.vertexCount
        );
    }

    for( std::size_t i = 0; i < polygon.vertexCount; ++i )
    {
        const std::size_t next = ( i + 1 ) % polygon.vertexCount;
        const vec2 edge = polygon.vertices[next] - polygon.vertices[i];

        if( LengthSquared( edge ) <= epsilon * epsilon )
        {
            assert( false );
            return {};
        }

        polygon.normals[i] = Normalize( { edge.y, -edge.x } );
    }

    for( std::size_t i = 0; i < polygon.vertexCount; ++i )
    {
        const std::size_t next = ( i + 1 ) % polygon.vertexCount;
        const std::size_t nextNext = ( i + 2 ) % polygon.vertexCount;

        const vec2 edge1 = polygon.vertices[next] - polygon.vertices[i];
        const vec2 edge2 = polygon.vertices[nextNext] - polygon.vertices[next];

        if( Cross( edge1, edge2 ) <= epsilon )
        {
            assert( false );
            return {};
        }
    }

    vec2 centroid{};
    float crossSum = 0.0f;

    for( std::size_t i = 0; i < polygon.vertexCount; ++i )
    {
        const std::size_t next = ( i + 1 ) % polygon.vertexCount;

        const vec2& a = polygon.vertices[i];
        const vec2& b = polygon.vertices[next];
        const float cross = Cross( a, b );

        crossSum += cross;
        centroid += ( a + b ) * cross;
    }

    if( std::fabs( crossSum ) <= epsilon )
    {
        assert( false );
        return {};
    }

    centroid /= 3.0f * crossSum;
    polygon.centroid = centroid;

    return polygon;
}

aabb2 ComputeAABB( const polygon2& polygon )
{
    if( polygon.vertexCount == 0 )
    {
        return {};
    }

    vec2 min = polygon.vertices[0];
    vec2 max = polygon.vertices[0];

    for( std::size_t i = 1; i < polygon.vertexCount; ++i )
    {
        const vec2& vertex = polygon.vertices[i];

        min.x = std::min( min.x, vertex.x );
        min.y = std::min( min.y, vertex.y );

        max.x = std::max( max.x, vertex.x );
        max.y = std::max( max.y, vertex.y );
    }

    const vec2 radius{ polygon.radius, polygon.radius };

    return
    {
        min - radius,
        max + radius
    };
}

} // namespace zonai
