#pragma once

#include <array>
#include <cstddef>

#include "math/vec2.h"

namespace zonai
{

constexpr std::size_t MAX_LOCAL_MANIFOLD_POINTS = 2;

struct localManifoldPoint2
{
    vec2 point{};
    float separation = 0.0f;
};

struct localManifold2
{
    vec2 normal{};
    std::array<localManifoldPoint2, MAX_LOCAL_MANIFOLD_POINTS> points{};
    std::size_t pointCount = 0;
};

} // namespace zonai