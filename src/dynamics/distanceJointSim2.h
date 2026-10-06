#pragma once

#include <cstdint>
#include "math/vec2.h"

namespace zonai
{
// World의 Joint cold slot과 같은 index에 저장하는 persistent solver 데이터.
struct distanceJointSim2
{
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;
    vec2 localAnchorA{};
    vec2 localAnchorB{};
    float length = 1.0f;
    float impulse = 0.0f;
    float subStepTime = 0.0f;
    bool enableSpring = false;
    float hertz = 5.0f;
    float dampingRatio = 0.7f;
};
} // namespace zonai
