#pragma once

#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"
#include "dynamics/constraintSoftness2.h"
#include "dynamics/mouseJointSim2.h"

namespace zonai
{

struct mouseJointConstraint2
{
    std::int32_t jointId = -1;
    std::int32_t bodyIdB = -1;

    // Step 시작 시 질량 중심에서 작용점까지의 벡터.
    vec2 anchorB{};
    vec2 deltaCenter{};

    // 2x2 유효 질량 행렬의 두 열.
    vec2 massX{};
    vec2 massY{};
    float invMass = 0.0f;
    float invInertia = 0.0f;
    float maxImpulse = 0.0f;

    // 현재 step에서 누적한 임펄스.
    vec2 impulse{};
    constraintSoftness2 softness{};
};

// 클릭 위치의 질량 중심 기준 작용점, 2x2 유효 질량, 힘 제한을 준비함.
[[nodiscard]] mouseJointConstraint2 prepareMouseJointConstraint( const mouseJointSim2& joint, const bodySim& bodySimB, float subStepTime );

// 힘 제한 안에서 이전 step의 누적 임펄스를 먼저 적용함.
void warmStartMouseJointConstraint( const mouseJointConstraint2& constraint, bodyState& bodyStateB );

// 월드 목표점을 향한 점 스프링을 풀며, 누적 임펄스의 크기를 dt * maxForce로 제한함.
void solveMouseJointConstraint( mouseJointConstraint2& constraint, bodyState& bodyStateB );

} // namespace zonai
