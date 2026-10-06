#pragma once

#include <cstdint>
#include "math/vec2.h"

namespace zonai
{
#pragma region Simulation

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

    // 누적 임펄스. 다음 step의 warm start와 force 조회에 사용함.
    vec2 impulse{};
    float subStepTime = 0.0f;
};

#pragma endregion
} // namespace zonai
