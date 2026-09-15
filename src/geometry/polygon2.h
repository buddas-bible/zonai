#pragma once

#include <array>
#include <span>

#include "math/vec2.h"
#include "collision/aabb2.h"

namespace zonai
{

constexpr std::size_t MAX_POLYGON_VERTICES = 8;

struct polygon2
{
    std::array<vec2, MAX_POLYGON_VERTICES> vertices;
    std::array<vec2, MAX_POLYGON_VERTICES> normals;
    vec2 centroid{};
	float radius = 0.0f;
    std::size_t vertexCount = 0;
};

polygon2 MakePolygon( std::span<const vec2> vertices );

aabb2 ComputeAABB( const polygon2& polygon );

} // namespace zonai