#pragma once

#include "collision/aabb2.h"

namespace zonai
{
	struct circle2
	{
		vec2 center{};
		float radius = 0.0f;

		aabb2 ComputeAABB(const circle2& circle);
		bool Contains(const circle2& circle, const vec2& point);
	};
}