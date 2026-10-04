#include "geometry/polygon2.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

namespace zonai
{

polygon2 MakeBox( const vec2& halfExtents )
{
    if( !IsFinite( halfExtents ) ||
        halfExtents.x <= 0.0f ||
        halfExtents.y <= 0.0f )
    {
        assert( false );
        return {};
    }

    polygon2 box{};
    box.vertices[0] = { -halfExtents.x, -halfExtents.y };
    box.vertices[1] = {  halfExtents.x, -halfExtents.y };
    box.vertices[2] = {  halfExtents.x,  halfExtents.y };
    box.vertices[3] = { -halfExtents.x,  halfExtents.y };
    box.normals[0] = { 0.0f, -1.0f };
    box.normals[1] = { 1.0f,  0.0f };
    box.normals[2] = { 0.0f,  1.0f };
    box.normals[3] = { -1.0f, 0.0f };
    box.vertexCount = 4;
    return box;
}

polygon2 MakeCapsule( const vec2& center1, const vec2& center2, float radius )
{
    polygon2 capsule{};

    if( !IsFinite( center1 ) ||
        !IsFinite( center2 ) ||
        !std::isfinite( radius ) ||
        radius < 0.0f )
    {
        assert( false );
        return capsule;
    }

    const vec2 direction = center2 - center1;
    constexpr float epsilon = std::numeric_limits<float>::epsilon();

    // 2-vertex core가 안정적인 normal을 가지도록 충분한 길이를 요구함.
    if( LengthSquared( direction ) <= epsilon )
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

    polygon.vertexCount = static_cast<int>( vertices.size() );

    for( int i = 0; i < polygon.vertexCount; ++i )
    {
        if( !IsFinite( vertices[i] ) )
        {
            assert( false );
            return {};
        }

        polygon.vertices[i] = vertices[i];
    }

    constexpr float epsilon = 1e-6f;

    // 첫 vertex를 기준으로 삼각분할해 큰 좌표에서도 면적 정밀도를 유지함.
    const vec2 windingOrigin = polygon.vertices[0];
    float area2 = 0.0f;

    for( int i = 1; i < polygon.vertexCount - 1; ++i )
    {
        const vec2 edge1 = polygon.vertices[i] - windingOrigin;
        const vec2 edge2 = polygon.vertices[i + 1] - windingOrigin;
        area2 += Cross( edge1, edge2 );
    }

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

    for( int i = 0; i < polygon.vertexCount; ++i )
    {
        const int next = ( i + 1 ) % polygon.vertexCount;
        const vec2 edge = polygon.vertices[next] - polygon.vertices[i];

        if( LengthSquared( edge ) <= epsilon * epsilon )
        {
            assert( false );
            return {};
        }

        polygon.normals[i] = Normalize( { edge.y, -edge.x } );
    }

    for( int i = 0; i < polygon.vertexCount; ++i )
    {
        const int next = ( i + 1 ) % polygon.vertexCount;
        const int nextNext = ( i + 2 ) % polygon.vertexCount;

        const vec2 edge1 = polygon.vertices[next] - polygon.vertices[i];
        const vec2 edge2 = polygon.vertices[nextNext] - polygon.vertices[next];

        if( Cross( edge1, edge2 ) <= epsilon )
        {
            assert( false );
            return {};
        }
    }

    // Box2D처럼 첫 vertex를 local origin으로 사용해 centroid 계산의 상쇄 오차를 줄임.
    const vec2 centroidOrigin = polygon.vertices[0];
    vec2 centroidOffset{};
    float area = 0.0f;

    for( int i = 1; i < polygon.vertexCount - 1; ++i )
    {
        const vec2 edge1 = polygon.vertices[i] - centroidOrigin;
        const vec2 edge2 = polygon.vertices[i + 1] - centroidOrigin;
        const float triangleArea = 0.5f * Cross( edge1, edge2 );

        area += triangleArea;
        centroidOffset +=
            ( edge1 + edge2 ) *
            ( triangleArea / 3.0f );
    }

    if( area <= 0.5f * epsilon )
    {
        assert( false );
        return {};
    }

    centroidOffset /= area;
    polygon.centroid = centroidOrigin + centroidOffset;

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

    for( int i = 1; i < polygon.vertexCount; ++i )
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
