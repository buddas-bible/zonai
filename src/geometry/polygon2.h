#pragma once

#include <array>
#include <span>
#include <cstdint>
#include <cstddef>

#include "math/vec2.h"
#include "collision/aabb2.h"

namespace zonai
{

constexpr std::size_t MAX_POLYGON_VERTICES = 8;

struct polygon2
{
    std::array<vec2, MAX_POLYGON_VERTICES> vertices{};
    std::array<vec2, MAX_POLYGON_VERTICES> normals{};
    vec2 centroid{};
    float radius = 0.0f;
    std::int32_t vertexCount = 0;
}; // sizeof: 144 bytes

polygon2 MakeBox( const vec2& halfExtents );

polygon2 MakeCapsule( const vec2& center1, const vec2& center2, float radius );

polygon2 MakePolygon( std::span<const vec2> vertices );

aabb2 ComputeAABB( const polygon2& polygon );

} // namespace zonai