#pragma once

#include <cstdint>
#include "math/vec2.h"

namespace zonai
{

// World의 stable Joint slot에 계속 보관되는 Motor Joint의 simulation 상태.
// 목표 상대속도와 힘/토크 한도, 다음 substep warm start에 필요한 누적 impulse를 유지함.
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

    // B가 A에 대해 가져야 하는 목표 상대 각속도.
    float angularVelocity = 0.0f;
    float maxVelocityTorque = 0.0f;

    // 이전 substep에서 구한 velocity motor의 누적 impulse.
    // Prepare에서 Constraint로 복사하고 Solve가 끝나면 다시 저장해 다음 warm start에 사용함.
    vec2 linearVelocityImpulse{};
    float angularVelocityImpulse = 0.0f;
    float subStepTime = 0.0f;
};

} // namespace zonai
