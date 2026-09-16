#include "collision/narrowphase/collide.h"

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

    if( s1 < 0.f )
    {
        // center1 영역
		pointA = point1;
    }
	else if( s2 < 0.f )
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

    if( distance > 0.f )
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

} // namespace zonai