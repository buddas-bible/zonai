#pragma once

#include <cstdint>
#include <cstddef>
#include <array>
#include "math/vec2.h"

namespace zonai
{

constexpr std::size_t MAX_LOCAL_MANIFOLD_POINTS = 2;

struct localManifoldPoint2
{
    vec2 point{};
    float separation{};
    std::uint16_t id{};
}; // sizeof: 16 bytes

struct localManifold2
{
    vec2 normal{};
	std::array<localManifoldPoint2, MAX_LOCAL_MANIFOLD_POINTS> points{};
    int pointCount{};
}; // sizeof: 44 bytes

} // namespace zonai