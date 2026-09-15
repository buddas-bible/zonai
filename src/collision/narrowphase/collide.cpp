#include "collide.h"

#include <cmath>

namespace zonai
{
    bool Collide(
        const circle2& a, const transform2& transformA,
        const circle2& b, const transform2& transformB,
        manifold2& manifold)
    {
		const vec2 centerA = zonai::TransformPoint(transformA, a.center);
		const vec2 centerB = zonai::TransformPoint(transformB, b.center);

		const vec2 delta = centerB - centerA;
		const float distanceSquared = zonai::LengthSquared(delta);

		const float radiusSum = a.radius + b.radius;
		const float radiusSumSquared = radiusSum * radiusSum;

        if (distanceSquared > radiusSumSquared)
        {
			manifold.pointCount = 0;
			return false;
        }

		const float distance = std::sqrt(distanceSquared);

        vec2 normal{ 1.f, 0.f };

		if (distance > 0.0f)
		{
			normal = delta / distance;
		}

		manifold.normal = normal;
		manifold.pointCount = 1;

		manifold.points[0].point = centerA + normal * a.radius;
		manifold.points[0].separation = distance - radiusSum;

		return true;
    }
}