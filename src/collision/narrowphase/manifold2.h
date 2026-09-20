#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

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

} // namespace zonai
