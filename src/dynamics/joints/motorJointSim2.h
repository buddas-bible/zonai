#pragma once

#include <cstdint>
#include "math/vec2.h"

namespace zonai
{

// World의 stable Joint slot에 계속 보관되는 Motor Joint의 simulation 상태.
// 목표 상대속도와 transform spring 설정, 다음 substep warm start에 필요한 누적 impulse를 유지함.
struct motorJointSim2
{
    // 공용 Joint slot과 연결된 id, 그리고 두 Body의 simulation index.
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    // Body origin 기준의 로컬 작용점. Prepare에서 현재 COM 기준 lever arm으로 변환함.
    vec2 localAnchorA{};
    vec2 localAnchorB{};

    // B 작용점이 A 작용점에 대해 가져야 하는 목표 상대 선속도.
    vec2 linearVelocity{};
    float maxVelocityForce = 0.0f;

    // 두 anchor의 위치 오차를 줄이는 실제 spring-damper 설정.
    // 0 Hz 또는 0 max force면 선형 spring 채널은 비활성화됨.
    float linearHertz = 0.0f;
    float linearDampingRatio = 0.0f;
    float maxSpringForce = 0.0f;

    // B가 A에 대해 가져야 하는 목표 상대 각속도.
    float angularVelocity = 0.0f;
    float maxVelocityTorque = 0.0f;

    // 이전 substep에서 구한 velocity motor / transform spring 누적 impulse.
    // 두 채널은 독립적으로 cache하지만 Warm Start에서는 합쳐서 Body에 적용함.
    vec2 linearVelocityImpulse{};
    vec2 linearSpringImpulse{};
    float angularVelocityImpulse = 0.0f;
    float subStepTime = 0.0f;
};

} // namespace zonai
