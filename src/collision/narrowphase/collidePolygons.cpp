#include "collision/narrowphase/collide.h"

#include <algorithm>
#include <cfloat>
#include <cmath>

namespace zonai
{
namespace
{

struct segmentDistanceResult2
{
    vec2 point1{};
    vec2 point2{};
    float fraction1 = 0.0f;
    float fraction2 = 0.0f;
    float distanceSquared = 0.0f;
};

segmentDistanceResult2 SegmentDistance(
    const vec2& p1, const vec2& q1,
    const vec2& p2, const vec2& q2 )
{
    const vec2 d1 = q1 - p1;
    const vec2 d2 = q2 - p2;

    const float dd1 = Dot( d1, d1 );
    const float dd2 = Dot( d2, d2 );

    const vec2 offset = p1 - p2;
    const float offsetD1 = Dot( offset, d1 );
    const float offsetD2 = Dot( offset, d2 );
    const float d12 = Dot( d1, d2 );

    const float denominator = dd1 * dd2 - d12 * d12;

    float fraction1 = 0.0f;

    if( denominator != 0.0f )
    {
        fraction1 = std::clamp(
            ( d12 * offsetD2 - offsetD1 * dd2 ) /
                denominator,
            0.0f,
            1.0f
        );
    }

    float fraction2 =
        ( d12 * fraction1 + offsetD2 ) / dd2;

    if( fraction2 < 0.0f )
    {
        fraction2 = 0.0f;
        fraction1 = std::clamp(
            -offsetD1 / dd1,
            0.0f,
            1.0f
        );
    }
    else if( fraction2 > 1.0f )
    {
        fraction2 = 1.0f;
        fraction1 = std::clamp(
            ( d12 - offsetD1 ) / dd1,
            0.0f,
            1.0f
        );
    }

    const vec2 point1 = p1 + d1 * fraction1;
    const vec2 point2 = p2 + d2 * fraction2;

    return
    {
        point1,
        point2,
        fraction1,
        fraction2,
        LengthSquared( point2 - point1 )
    };
}

float FindMaxSeparation(
    std::size_t& edgeIndex,
    const polygon2& polygonA,
    const polygon2& polygonB )
{
    edgeIndex = 0;
    float maxSeparation = -FLT_MAX;

    for( std::size_t i = 0; i < polygonA.vertexCount; ++i )
    {
        const vec2 normal = polygonA.normals[i];
        const vec2 vertexA = polygonA.vertices[i];

        float separation = FLT_MAX;

        for( std::size_t j = 0; j < polygonB.vertexCount; ++j )
        {
            const float currentSeparation = Dot(
                normal,
                polygonB.vertices[j] - vertexA
            );

            separation = std::min(
                separation,
                currentSeparation
            );
        }

        if( separation > maxSeparation )
        {
            maxSeparation = separation;
            edgeIndex = i;
        }
    }

    return maxSeparation;
}

void AddContactPoint(
    localManifold2& manifold,
    const vec2& point,
    float separation )
{
    // Zonai는 아직 speculative contact를 사용하지 않는다.
    if( separation > FLT_EPSILON ||
        manifold.pointCount >= MAX_LOCAL_MANIFOLD_POINTS )
    {
        return;
    }

    manifold.points[manifold.pointCount].point = point;
    manifold.points[manifold.pointCount].separation = separation;
    ++manifold.pointCount;
}

localManifold2 ClipPolygons(
    const polygon2& polygonA,
    const polygon2& polygonB,
    std::size_t edgeA,
    std::size_t edgeB,
    bool flip )
{
    localManifold2 manifold{};

    const polygon2& reference = flip ? polygonB : polygonA;
    const polygon2& incident = flip ? polygonA : polygonB;

    const std::size_t referenceEdge = flip ? edgeB : edgeA;
    const std::size_t incidentEdge = flip ? edgeA : edgeB;

    const std::size_t referenceNext =
        ( referenceEdge + 1 ) % reference.vertexCount;
    const std::size_t incidentNext =
        ( incidentEdge + 1 ) % incident.vertexCount;

    const vec2 normal = reference.normals[referenceEdge];

    const vec2 reference1 = reference.vertices[referenceEdge];
    const vec2 reference2 = reference.vertices[referenceNext];

    const vec2 incident1 = incident.vertices[incidentEdge];
    const vec2 incident2 = incident.vertices[incidentNext];

    const vec2 tangent{ -normal.y, normal.x };

    const float lowerReference = 0.0f;
    const float upperReference = Dot(
        reference2 - reference1,
        tangent
    );

    // CCW winding 때문에 incident edge는 tangent의 반대 방향이다.
    const float upperIncident = Dot(
        incident1 - reference1,
        tangent
    );
    const float lowerIncident = Dot(
        incident2 - reference1,
        tangent
    );

    if( upperIncident < lowerReference ||
        upperReference < lowerIncident )
    {
        return manifold;
    }

    vec2 lowerPoint = incident2;

    if( lowerIncident < lowerReference &&
        upperIncident - lowerIncident > FLT_EPSILON )
    {
        const float fraction =
            ( lowerReference - lowerIncident ) /
            ( upperIncident - lowerIncident );

        lowerPoint = incident2 +
            ( incident1 - incident2 ) * fraction;
    }

    vec2 upperPoint = incident1;

    if( upperIncident > upperReference &&
        upperIncident - lowerIncident > FLT_EPSILON )
    {
        const float fraction =
            ( upperReference - lowerIncident ) /
            ( upperIncident - lowerIncident );

        upperPoint = incident2 +
            ( incident1 - incident2 ) * fraction;
    }

    const float lowerSeparation = Dot(
        lowerPoint - reference1,
        normal
    );
    const float upperSeparation = Dot(
        upperPoint - reference1,
        normal
    );

    const float radiusReference = reference.radius;
    const float radiusIncident = incident.radius;
    const float radiusSum = radiusReference + radiusIncident;

    lowerPoint += normal *
        ( 0.5f *
          ( radiusReference - radiusIncident - lowerSeparation ) );

    upperPoint += normal *
        ( 0.5f *
          ( radiusReference - radiusIncident - upperSeparation ) );

    const float lowerContactSeparation =
        lowerSeparation - radiusSum;
    const float upperContactSeparation =
        upperSeparation - radiusSum;

    manifold.normal = flip ? -normal : normal;

    if( !flip )
    {
        AddContactPoint(
            manifold,
            lowerPoint,
            lowerContactSeparation
        );
        AddContactPoint(
            manifold,
            upperPoint,
            upperContactSeparation
        );
    }
    else
    {
        AddContactPoint(
            manifold,
            upperPoint,
            upperContactSeparation
        );
        AddContactPoint(
            manifold,
            lowerPoint,
            lowerContactSeparation
        );
    }

    return manifold;
}

bool IsEndpoint( float fraction )
{
    return fraction == 0.0f || fraction == 1.0f;
}

localManifold2 MakeClosestPointManifold(
    const segmentDistanceResult2& result,
    float radiusA,
    float radiusB )
{
    localManifold2 manifold{};

    const vec2 delta = result.point2 - result.point1;
    const float distanceSquared = LengthSquared( delta );

    if( distanceSquared <= FLT_EPSILON * FLT_EPSILON )
    {
        return manifold;
    }

    const float distance = std::sqrt( distanceSquared );
    const vec2 normal = delta / distance;

    const vec2 surfaceA = result.point1 + normal * radiusA;
    const vec2 surfaceB = result.point2 - normal * radiusB;

    manifold.normal = normal;
    manifold.points[0].point =
        ( surfaceA + surfaceB ) * 0.5f;
    manifold.points[0].separation =
        distance - radiusA - radiusB;
    manifold.pointCount = 1;

    return manifold;
}

} // namespace

localManifold2 CollidePolygons(
    const polygon2& a,
    const polygon2& b, const transform2& transformB )
{
    localManifold2 manifold{};

    if( a.vertexCount == 0 || b.vertexCount == 0 )
    {
        return manifold;
    }

    // 계산 오차를 줄이기 위해 polygon A의 첫 정점을 원점으로 이동한다.
    const vec2 origin = a.vertices[0];

    polygon2 localA = a;
    polygon2 localB = b;

    for( std::size_t i = 0; i < localA.vertexCount; ++i )
    {
        localA.vertices[i] -= origin;
    }

    for( std::size_t i = 0; i < localB.vertexCount; ++i )
    {
        localB.vertices[i] =
            TransformPoint( transformB, b.vertices[i] ) - origin;
        localB.normals[i] =
            TransformVector( transformB, b.normals[i] );
    }

    std::size_t edgeA = 0;
    const float separationA =
        FindMaxSeparation( edgeA, localA, localB );

    std::size_t edgeB = 0;
    const float separationB =
        FindMaxSeparation( edgeB, localB, localA );

    const float radiusSum = localA.radius + localB.radius;

    // TODO: speculative contacts
    if( separationA > radiusSum ||
        separationB > radiusSum )
    {
        return manifold;
    }

    bool flip = false;

    if( separationA >= separationB )
    {
        const vec2 searchDirection = localA.normals[edgeA];

        edgeB = 0;
        float minDot = FLT_MAX;

        for( std::size_t i = 0; i < localB.vertexCount; ++i )
        {
            const float dot = Dot(
                searchDirection,
                localB.normals[i]
            );

            if( dot < minDot )
            {
                minDot = dot;
                edgeB = i;
            }
        }
    }
    else
    {
        flip = true;
        const vec2 searchDirection = localB.normals[edgeB];

        edgeA = 0;
        float minDot = FLT_MAX;

        for( std::size_t i = 0; i < localA.vertexCount; ++i )
        {
            const float dot = Dot(
                searchDirection,
                localA.normals[i]
            );

            if( dot < minDot )
            {
                minDot = dot;
                edgeA = i;
            }
        }
    }

    constexpr float linearSlop = 0.005f;

    // Core polygon들이 떨어져 있지만 radius 때문에 접촉하는 경우에는
    // reference/incident edge의 실제 최근접 feature를 확인한다.
    if( separationA > 0.1f * linearSlop ||
        separationB > 0.1f * linearSlop )
    {
        const std::size_t a1 = edgeA;
        const std::size_t a2 =
            ( edgeA + 1 ) % localA.vertexCount;
        const std::size_t b1 = edgeB;
        const std::size_t b2 =
            ( edgeB + 1 ) % localB.vertexCount;

        const segmentDistanceResult2 result = SegmentDistance(
            localA.vertices[a1],
            localA.vertices[a2],
            localB.vertices[b1],
            localB.vertices[b2]
        );

        const float distance = std::sqrt( result.distanceSquared );
        const float closestSeparation = distance - radiusSum;

        if( closestSeparation > 0.0f )
        {
            return manifold;
        }

        manifold = ClipPolygons(
            localA,
            localB,
            edgeA,
            edgeB,
            flip
        );

        float minSeparation = FLT_MAX;

        for( std::size_t i = 0; i < manifold.pointCount; ++i )
        {
            minSeparation = std::min(
                minSeparation,
                manifold.points[i].separation
            );
        }

        const bool vertexVertex =
            IsEndpoint( result.fraction1 ) &&
            IsEndpoint( result.fraction2 );

        if( manifold.pointCount == 0 ||
            ( vertexVertex &&
              closestSeparation + 0.1f * linearSlop < minSeparation ) )
        {
            manifold = MakeClosestPointManifold(
                result,
                localA.radius,
                localB.radius
            );
        }
    }
    else
    {
        manifold = ClipPolygons(
            localA,
            localB,
            edgeA,
            edgeB,
            flip
        );
    }

    for( std::size_t i = 0; i < manifold.pointCount; ++i )
    {
        manifold.points[i].point += origin;
    }

    return manifold;
}

} // namespace zonai
