#include "collision/narrowphase/collide.h"

#include <cfloat>
#include <cmath>

#include "collision/constants.h"

namespace zonai
{

localManifold2 CollideCircles( const circle2& a, const circle2& b, const transform2& transformB )
{
    localManifold2 manifold{};

    const vec2 pointA = a.center;
    const vec2 pointB = TransformPoint( transformB, b.center );

    const vec2 delta = pointB - pointA;
    const float distanceSquared = LengthSquared( delta );

    const float radiusSum = a.radius + b.radius;

    const float speculativeRadius = radiusSum + SPECULATIVE_DISTANCE;

    if( distanceSquared > speculativeRadius * speculativeRadius ) return manifold;

    const float distance = std::sqrt( distanceSquared );

    vec2 normal{};

    if( distance > 0.0f )
    {
        normal = delta / distance;
    }

    const vec2 surfaceA = pointA + normal * a.radius;
    const vec2 surfaceB = pointB - normal * b.radius;

    manifold.normal = normal;
    manifold.points[0].point = ( surfaceA + surfaceB ) * 0.5f;
    manifold.points[0].separation = distance - radiusSum;
    manifold.pointCount = 1;

    return manifold;
}

localManifold2 CollideSegmentCircle( const segment2& segment, const circle2& circle, const transform2& circleTransform )
{
    const capsule2 capsule = { segment.a, segment.b, 0.0f };

    return CollideCapsuleCircle( capsule, circle, circleTransform );
}

localManifold2 CollideCapsuleCircle( const capsule2& capsule, const circle2& circle, const transform2& circleTransform )
{
    localManifold2 manifold{};

    // 모든 계산을 capsule A의 local space에서 수행함.
    const vec2 pointB = TransformPoint( circleTransform, circle.center );

    const vec2 point1 = capsule.center1;
    const vec2 point2 = capsule.center2;
    const vec2 edge = point2 - point1;

    // circle center가 capsule axis의 어느 Voronoi 영역에 있는지 구분함.
    const float s1 = Dot( pointB - point1, edge );
    const float s2 = Dot( point2 - pointB, edge );

    vec2 pointA{};

    if( s1 < 0.0f )
    {
        pointA = point1;
    }
    else if( s2 < 0.0f )
    {
        pointA = point2;
    }
    else
    {
        const float edgeLengthSquared = Dot( edge, edge );
        const float fraction = s1 / edgeLengthSquared;

        pointA = point1 + edge * fraction;
    }

    // manifold normal은 항상 shape A에서 B를 향함.
    const vec2 delta = pointB - pointA;
    const float distanceSquared = LengthSquared( delta );

    const float radiusSum = capsule.radius + circle.radius;

    const float speculativeRadius = radiusSum + SPECULATIVE_DISTANCE;

    if( distanceSquared > speculativeRadius * speculativeRadius ) return manifold;

    const float distance = std::sqrt( distanceSquared );

    vec2 normal{};

    if( distance > 0.0f )
    {
        normal = delta / distance;
    }

    const vec2 surfaceA = pointA + normal * capsule.radius;
    const vec2 surfaceB = pointB - normal * circle.radius;

    manifold.normal = normal;
    manifold.points[0].point = ( surfaceA + surfaceB ) * 0.5f;
    manifold.points[0].separation = distance - radiusSum;
    manifold.pointCount = 1;

    return manifold;
}

localManifold2 CollidePolygonCircle( const polygon2& polygon, const circle2& circle, const transform2& circleTransform )
{
    localManifold2 manifold{};

    if( polygon.vertexCount == 0 ) return manifold;

    // 모든 계산을 polygon A의 local space에서 수행함.
    const vec2 center = TransformPoint( circleTransform, circle.center );

    const float radiusA = polygon.radius;
    const float radiusB = circle.radius;
    const float radiusSum = radiusA + radiusB;

    // 가장 큰 separation을 가진 face가 circle과 가장 가까운 separating face임.
    std::size_t normalIndex = 0;
    float separation = -FLT_MAX;

    for( std::size_t i = 0; i < polygon.vertexCount; ++i )
    {
        const float currentSeparation = Dot( polygon.normals[i], center - polygon.vertices[i] );

        if( currentSeparation > separation )
        {
            separation = currentSeparation;
            normalIndex = i;
        }
    }

    if( separation > radiusSum + SPECULATIVE_DISTANCE ) return manifold;

    const std::size_t vertexIndex1 = normalIndex;
    const std::size_t vertexIndex2 = ( vertexIndex1 + 1 ) % polygon.vertexCount;

    const vec2 vertex1 = polygon.vertices[vertexIndex1];
    const vec2 vertex2 = polygon.vertices[vertexIndex2];

    const float u1 = Dot( center - vertex1, vertex2 - vertex1 );

    const float u2 = Dot( center - vertex2, vertex1 - vertex2 );

    if( u1 < 0.0f && separation > FLT_EPSILON )
    {
        const vec2 delta = center - vertex1;
        const float distanceSquared = LengthSquared( delta );

        const float speculativeRadius = radiusSum + SPECULATIVE_DISTANCE;

        if( distanceSquared > speculativeRadius * speculativeRadius ) return manifold;

        const float distance = std::sqrt( distanceSquared );
        const vec2 normal = delta / distance;

        const vec2 surfaceA = vertex1 + normal * radiusA;
        const vec2 surfaceB = center - normal * radiusB;

        manifold.normal = normal;
        manifold.points[0].point = ( surfaceA + surfaceB ) * 0.5f;
        manifold.points[0].separation = distance - radiusSum;
        manifold.pointCount = 1;
    }
    else if( u2 < 0.0f && separation > FLT_EPSILON )
    {
        const vec2 delta = center - vertex2;
        const float distanceSquared = LengthSquared( delta );

        const float speculativeRadius = radiusSum + SPECULATIVE_DISTANCE;

        if( distanceSquared > speculativeRadius * speculativeRadius ) return manifold;

        const float distance = std::sqrt( distanceSquared );
        const vec2 normal = delta / distance;

        const vec2 surfaceA = vertex2 + normal * radiusA;
        const vec2 surfaceB = center - normal * radiusB;

        manifold.normal = normal;
        manifold.points[0].point = ( surfaceA + surfaceB ) * 0.5f;
        manifold.points[0].separation = distance - radiusSum;
        manifold.pointCount = 1;
    }
    else
    {
        // face 영역에는 circle center가 polygon 내부인 경우도 포함됨.
        const vec2 normal = polygon.normals[normalIndex];

        const float centerSeparation = Dot( center - vertex1, normal );

        const vec2 surfaceA = center + normal * ( radiusA - centerSeparation );

        const vec2 surfaceB = center - normal * radiusB;

        manifold.normal = normal;
        manifold.points[0].point = ( surfaceA + surfaceB ) * 0.5f;
        manifold.points[0].separation = separation - radiusSum;
        manifold.pointCount = 1;
    }

    return manifold;
}

} // namespace zonai
