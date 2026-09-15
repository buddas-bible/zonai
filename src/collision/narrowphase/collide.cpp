#include "collision/narrowphase/collide.h"

#include <cmath>

namespace zonai
{

bool Collide(
    const circle2& a, const transform2& transformA,
    const circle2& b, const transform2& transformB,
    manifold2& manifold )
{
    manifold.pointCount = 0;

    const vec2 centerA = zonai::TransformPoint( transformA, a.center );
    const vec2 centerB = zonai::TransformPoint( transformB, b.center );

    const vec2 delta = centerB - centerA;
    const float distanceSquared = zonai::LengthSquared( delta );

    const float radiusSum = a.radius + b.radius;

    if( distanceSquared > radiusSum * radiusSum )
    {
        return false;
    }

    const float distance = std::sqrt( distanceSquared );

    vec2 normal{ 1.f, 0.f };

    if( distance > 0.0f )
    {
        normal = delta / distance;
    }

    const vec2 pointA = centerA + normal * a.radius;
    const vec2 pointB = centerB - normal * b.radius;

    manifold.normal = normal;
    manifold.points[0].point = ( pointA + pointB ) * 0.5f;
    manifold.points[0].separation = distance - radiusSum;
    manifold.pointCount = 1;

    return true;
}

bool Collide(
    const circle2& circle, const transform2& circleTransform,
    const capsule2& capsule, const transform2& capsuleTransform,
    manifold2& manifold )
{
    manifold.pointCount = 0;

    const vec2 circleCenter = zonai::TransformPoint( circleTransform, circle.center );

    const segment2 capsuleAxis
    {
        zonai::TransformPoint( capsuleTransform, capsule.center1 ),
        zonai::TransformPoint( capsuleTransform, capsule.center2 )
    };

    const vec2 closestPoint = zonai::ClosestPoint( capsuleAxis, circleCenter );

    const vec2 delta = closestPoint - circleCenter;
    const float distanceSquared = zonai::LengthSquared( delta );

    const float radiusSum = circle.radius + capsule.radius;

    if( distanceSquared > radiusSum * radiusSum )
    {
        return false;
    }

    const float distance = std::sqrt( distanceSquared );

    vec2 normal{ 1.f, 0.f };

    if( distance > 0.f )
    {
        normal = delta / distance;
    }
    else
    {
        const vec2 axis = capsuleAxis.b - capsuleAxis.a;

        if( zonai::LengthSquared( axis ) > 0.0f )
        {
            normal = zonai::Normalize( vec2{ -axis.y, axis.x } );
        }
        else
        {
            normal = { 1.0f, 0.0f };
        }
    }

    const vec2 pointA = circleCenter + normal * circle.radius;
    const vec2 pointB = closestPoint - normal * capsule.radius;

    manifold.normal = normal;
    manifold.points[0].point = ( pointA + pointB ) * 0.5f;
    manifold.points[0].separation = distance - radiusSum;;
    manifold.pointCount = 1;

    return true;
}

} // namespace zonai