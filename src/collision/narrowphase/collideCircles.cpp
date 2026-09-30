#include "collision/narrowphase/collide.h"

#include <cfloat>
#include <cmath>

namespace zonai
{

localManifold2 CollideCircles(
    const circle2& a,
    const circle2& b, const transform2& transformB )
{
    localManifold2 manifold = {};

    const vec2 pointA = a.center;
    const vec2 pointB = zonai::TransformPoint( transformB, b.center );

    const vec2 delta = pointB - pointA;
    const float distanceSquared = zonai::LengthSquared( delta );

    const float radiusSum = a.radius + b.radius;

    // TODO: speculative contacts
    if( distanceSquared > radiusSum * radiusSum )
    {
        return manifold;
    }

    const float distance = std::sqrt( distanceSquared );

    vec2 normal{};

    if( distance > 0.0f )
    {
        normal = delta / distance; // normalize
    }

    const vec2 surfaceA = pointA + normal * a.radius;
    const vec2 surfaceB = pointB - normal * b.radius;

    manifold.normal = normal;
    manifold.points[0].point = ( surfaceA + surfaceB ) * 0.5f;
    manifold.points[0].separation = distance - radiusSum;
    manifold.pointCount = 1;

    return manifold;
}

localManifold2 CollideSegmentCircle(
    const segment2& segment,
    const circle2& circle, const transform2& circleTransform )
{
    const capsule2 capsule =
    {
        segment.a,
        segment.b,
        0.0f
    };

    return CollideCapsuleCircle( capsule, circle, circleTransform );
}

localManifold2 CollideCapsuleCircle(
    const capsule2& capsule,
    const circle2& circle, const transform2& circleTransform )
{
    localManifold2 manifold = {};

    // circle B를 capsule A의 로컬 좌표계로 변환
    const vec2 pointB = zonai::TransformPoint( circleTransform, circle.center );

    const vec2 point1 = capsule.center1;
    const vec2 point2 = capsule.center2;

    const vec2 edge = point2 - point1;

    // circle center가 capsule axis의 어느 영역에 있는지 확인
    const float s1 = zonai::Dot( pointB - point1, edge );
    const float s2 = zonai::Dot( point2 - pointB, edge );

    vec2 pointA{};

    if( s1 < 0.0f )
    {
        // center1 영역
        pointA = point1;
    }
    else if( s2 < 0.0f )
    {
        // center2 영역
        pointA = point2;
    }
    else
    {
        // capsule axis 영역
        const float fraction = zonai::Dot( edge, edge );
        const float t = s1 / fraction;

        pointA = point1 + edge * t;
    }

    // capsule -> circle
    const vec2 delta = pointB - pointA;
    const float distanceSquared = zonai::LengthSquared( delta );

    const float radiusSum = capsule.radius + circle.radius;

    // TODO: speculative contacts
    if( distanceSquared > radiusSum * radiusSum )
    {
        return manifold;
    }

    const float distance = std::sqrt( distanceSquared );

    vec2 normal{};

    if( distance > 0.0f )
    {
        normal = delta / distance; // normalize
    }

    const vec2 surfaceA = pointA + normal * capsule.radius;
    const vec2 surfaceB = pointB - normal * circle.radius;

    manifold.normal = normal;
    manifold.points[0].point = ( surfaceA + surfaceB ) * 0.5f;
    manifold.points[0].separation = distance - radiusSum;
    manifold.pointCount = 1;

    return manifold;
}

localManifold2 CollidePolygonCircle(
    const polygon2& polygon,
    const circle2& circle, const transform2& circleTransform )
{
    localManifold2 manifold = {};

    if( polygon.vertexCount == 0 )
    {
        return manifold;
    }

    // circle B를 polygon A의 로컬 좌표계로 변환
    const vec2 center = zonai::TransformPoint( circleTransform, circle.center );

    const float radiusA = polygon.radius;
    const float radiusB = circle.radius;
    const float radiusSum = radiusA + radiusB;

    // circle center에서 가장 가까운 separating face를 찾는다.
    std::size_t normalIndex = 0;
    float separation = -FLT_MAX;

    for( std::size_t i = 0; i < polygon.vertexCount; ++i )
    {
        const float currentSeparation = zonai::Dot(
            polygon.normals[i],
            center - polygon.vertices[i]
        );

        if( currentSeparation > separation )
        {
            separation = currentSeparation;
            normalIndex = i;
        }
    }

    // TODO: speculative contacts
    if( separation > radiusSum )
    {
        return manifold;
    }

    const std::size_t vertexIndex1 = normalIndex;
    const std::size_t vertexIndex2 =
        ( vertexIndex1 + 1 ) % polygon.vertexCount;

    const vec2 vertex1 = polygon.vertices[vertexIndex1];
    const vec2 vertex2 = polygon.vertices[vertexIndex2];

    const float u1 = zonai::Dot(
        center - vertex1,
        vertex2 - vertex1
    );

    const float u2 = zonai::Dot(
        center - vertex2,
        vertex1 - vertex2
    );

    if( u1 < 0.0f && separation > FLT_EPSILON )
    {
        // vertex1 영역
        const vec2 delta = center - vertex1;
        const float distanceSquared = zonai::LengthSquared( delta );

        if( distanceSquared > radiusSum * radiusSum )
        {
            return manifold;
        }

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
        // vertex2 영역
        const vec2 delta = center - vertex2;
        const float distanceSquared = zonai::LengthSquared( delta );

        if( distanceSquared > radiusSum * radiusSum )
        {
            return manifold;
        }

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
        // face 영역. circle center가 polygon 내부에 있는 경우도 포함한다.
        const vec2 normal = polygon.normals[normalIndex];

        const float centerSeparation = zonai::Dot(
            center - vertex1,
            normal
        );

        const vec2 surfaceA =
            center + normal * ( radiusA - centerSeparation );

        const vec2 surfaceB =
            center - normal * radiusB;

        manifold.normal = normal;
        manifold.points[0].point = ( surfaceA + surfaceB ) * 0.5f;
        manifold.points[0].separation = separation - radiusSum;
        manifold.pointCount = 1;
    }

    return manifold;
}

} // namespace zonai
