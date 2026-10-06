#pragma once

#include <cstdint>
#include "math/vec2.h"

namespace zonai
{

// World의 조인트 슬롯에 보관하는 작용점과 이전 step의 누적 임펄스.
struct revoluteJointSim2
{
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    vec2 localAnchorA{};
    vec2 localAnchorB{};

    vec2 impulse{};
    float subStepTime = 0.0f;

    float referenceAngle = 0.0f;
    bool enableLimit = false;
    float lowerAngle = 0.0f;
    float upperAngle = 0.0f;
    float lowerImpulse = 0.0f;
    float upperImpulse = 0.0f;
};

} // namespace zonai
