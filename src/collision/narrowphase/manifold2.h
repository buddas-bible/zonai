#pragma once

#include <array>
#include <cstddef>

#include "math/vec2.h"

namespace zonai
{
    constexpr std::size_t maxManifoldPoints = 2;

    struct manifoldPoint2
    {
        vec2 point{};
        float separation = 0.0f;
    };

    struct manifold2
    {
        vec2 normal{};

        std::array<manifoldPoint2, maxManifoldPoints> points{};
        std::size_t pointCount = 0;
    };
}