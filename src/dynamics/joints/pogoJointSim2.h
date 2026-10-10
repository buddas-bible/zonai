#pragma once

#include <cstdint>
#include "math/vec2.h"

namespace zonai
{

// World의 stable Joint slot에 보관되는 Pogo Joint 상태.
// 길이 오차는 B의 pogo 축으로 측정하지만 실제 힘은 contact normal 방향으로 가하는 contact/joint hybrid임.
struct pogoJointSim2
{
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    // Body origin 기준의 두 작용점과 B에 붙어 회전하는 pogo 길이 측정 축.
    vec2 localAnchorA{};
    vec2 localAnchorB{};
    vec2 localPogoAxisB{ 0.0f, 1.0f };

    // 실제 impulse를 가하는 world-space contact normal.
    vec2 normal{ 0.0f, 1.0f };

    float restLength = 0.0f;
    float hertz = 0.0f;
    float dampingRatio = 0.0f;
    float maxTensionForce = 0.0f;
    float maxCompressionForce = 0.0f;

    // 이전 solve의 누적 normal impulse와 1D spring-damper 내부 속도.
    float impulse = 0.0f;
    float velocity = 0.0f;
    float subStepTime = 0.0f;
};

} // namespace zonai
