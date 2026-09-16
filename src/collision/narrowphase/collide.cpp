#include "collision/narrowphase/collide.h"

#include <cmath>

namespace zonai
{

bool CollideCircles(
    const circle2& a,
    const circle2& b, const transform2& transformB,
    manifold2& manifold )
{
    manifold = {};

    const vec2 centerA = a.center;
    const vec2 centerB = zonai::TransformPoint( transformB, b.center );

    const vec2 delta = centerB - centerA;
    const float distanceSquared = zonai::LengthSquared( delta );

    const float radiusSum = a.radius + b.radius;

    if( distanceSquared > radiusSum * radiusSum )
    {
        return false;
    }

    const float distance = std::sqrt( distanceSquared );

    vec2 normal{};

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

bool CollideCapsuleCircle(
    const capsule2& capsule,
    const circle2& circle, const transform2& circleTransform,
    manifold2& manifold )
{
    manifold = {};

    const vec2 circleCenter = zonai::TransformPoint( circleTransform, circle.center );
    const segment2 capsuleAxis = { capsule.center1, capsule.center2 };

    const vec2 closestPoint = zonai::ClosestPoint( capsuleAxis, circleCenter );

    // capsule -> circle 
    const vec2 delta = circleCenter - closestPoint;
    const float distanceSquared = zonai::LengthSquared( delta );

    const float radiusSum = capsule.radius + circle.radius;

    if( distanceSquared > radiusSum * radiusSum )
    {
        return false;
    }

    const float distance = std::sqrt( distanceSquared );

    vec2 normal{};

    if( distance > 0.f )
    {
        normal = delta / distance;
    }
    
    const vec2 pointA = closestPoint + normal * capsule.radius;
    const vec2 pointB = circleCenter - normal * circle.radius;

    manifold.normal = normal;
    manifold.points[0].point = ( pointA + pointB ) * 0.5f;
    manifold.points[0].separation = distance - radiusSum;;
    manifold.pointCount = 1;

    return true;
}

} // namespace zonai