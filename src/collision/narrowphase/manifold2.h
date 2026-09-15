#pragma once

#include <array>
#include <cstddef>

#include "math/vec2.h"

namespace zonai
{

constexpr std::size_t MAX_MANIFOLD_POINTS = 2;

struct manifoldPoint2
{
    vec2 point{};
    float separation = 0.0f;
};

struct manifold2
{
    vec2 normal{};

    std::array<manifoldPoint2, MAX_MANIFOLD_POINTS> points{};
    std::size_t pointCount = 0;
};

} // namespace zonai