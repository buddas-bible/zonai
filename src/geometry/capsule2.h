#pragma once

#include <algorithm>

#include "math/vec2.h"
#include "geometry/segment2.h"
#include "collision/aabb2.h"

namespace zonai
{
	struct capsule2
    {
		vec2 a{};
		vec2 b{};
		float radius = 0.0f;
    };

    inline aabb2 ComputeAABB(const capsule2& capsule)
    {
        vec2 min = { std::min(capsule.a.x, capsule.b.x) - capsule.radius, std::min(capsule.a.y, capsule.b.y) - capsule.radius };
        vec2 max = { std::max(capsule.a.x, capsule.b.x) + capsule.radius, std::max(capsule.a.y, capsule.b.y) + capsule.radius };
        return { min, max };
    }

    inline bool Contains(const capsule2& capsule, const vec2& point)
    {
		const zonai::segment2 axis{ capsule.a, capsule.b };

		return zonai::DistanceSquared(axis, point) <= capsule.radius * capsule.radius;
    }
}