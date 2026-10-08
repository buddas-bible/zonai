#pragma once

#include <cstdint>
#include "math/vec2.h"

namespace zonai
{

// World의 Weld 조인트 슬롯에 보관하는 기준 관계와 이전 substep의 누적 임펄스.
struct weldJointSim2
{
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    vec2 localAnchorA{};
    vec2 localAnchorB{};
    float referenceAngle = 0.0f;

    vec2 linearImpulse{};
    float angularImpulse = 0.0f;
    float subStepTime = 0.0f;
};

} // namespace zonai
