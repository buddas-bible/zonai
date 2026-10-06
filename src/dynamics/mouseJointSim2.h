#pragma once

#include <cstdint>
#include "math/vec2.h"

namespace zonai
{
struct mouseJointSim2
{
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;
    vec2 target{};
    vec2 localAnchorB{};
    float hertz = 5.0f;
    float dampingRatio = 0.7f;
    float maxForce = 1000.0f;
    vec2 impulse{};
    float subStepTime = 0.0f;
};
} // namespace zonai
