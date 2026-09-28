#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "math/transform2.h"
#include "math/vec2.h"

namespace zonai
{

constexpr std::size_t MAX_MANIFOLD_POINTS = 2;

struct localManifoldPoint2
{
    vec2 point{};
    float separation{};
    std::uint16_t id{};
}; // sizeof: 16 bytes

struct localManifold2
{
    vec2 normal{};
    std::array<localManifoldPoint2, MAX_MANIFOLD_POINTS> points{};
    int pointCount{};
}; // sizeof: 44 bytes

struct manifoldPoint2
{
    vec2 point{};
    float separation{};
    std::uint16_t id{};
}; // sizeof: 16 bytes

struct manifold2
{
    vec2 normal{};
    std::array<manifoldPoint2, MAX_MANIFOLD_POINTS> points{};
    int pointCount{};
}; // sizeof: 44 bytes

// Shape A local-space manifold을 world-space public manifold로 변환함.
inline manifold2 ToWorldManifold(
    const localManifold2& localManifold,
    const transform2& transformA )
{
    manifold2 manifold{};
    manifold.normal =
        TransformVector( transformA, localManifold.normal );
    manifold.pointCount = localManifold.pointCount;

    for( int i = 0; i < localManifold.pointCount; ++i )
    {
        manifold.points[i].point =
            TransformPoint(
                transformA,
                localManifold.points[i].point
            );

        manifold.points[i].separation =
            localManifold.points[i].separation;

        manifold.points[i].id =
            localManifold.points[i].id;
    }

    return manifold;
}

} // namespace zonai
