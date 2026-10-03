#pragma once

#include <array>
#include <cstddef>
#include <span>

#include "collision/aabb2.h"
#include "math/vec2.h"

namespace zonai
{

constexpr std::size_t MAX_POLYGON_VERTICES = 8;

struct polygon2
{
    std::array<vec2, MAX_POLYGON_VERTICES> vertices{};
    std::array<vec2, MAX_POLYGON_VERTICES> normals{};
    vec2 centroid{};
    float radius = 0.0f;
    int vertexCount = 0;
};

polygon2 MakeBox( const vec2& halfExtents );

// Capsule/segment를 polygon narrowphase에서 쓰는 2-vertex rounded core로 변환함.
polygon2 MakeCapsule( const vec2& center1, const vec2& center2, float radius );

// Convex perimeter vertex를 받아 CCW polygon으로 정규화함. Point cloud hull은 계산하지 않음.
polygon2 MakePolygon( std::span<const vec2> vertices );

aabb2 ComputeAABB( const polygon2& polygon );

} // namespace zonai
