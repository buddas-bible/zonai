#pragma once

#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"
#include "dynamics/constraintSoftness2.h"
#include "dynamics/joints/weldJointSim2.h"

namespace zonai
{

struct weldJointConstraint2
{
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    // Step 시작 시 질량 중심에서 각 작용점까지의 벡터.
    vec2 anchorA{};
    vec2 anchorB{};
    vec2 deltaCenter{};

    float invMassA = 0.0f;
    float invMassB = 0.0f;
    float invInertiaA = 0.0f;
    float invInertiaB = 0.0f;

    // 기준 각도를 뺀 Step 시작 시 상대 회전.
    rot2 relativeRotation{};
    float angularMass = 0.0f;

    vec2 linearImpulse{};
    float angularImpulse = 0.0f;
    constraintSoftness2 softness{};
};

// Body origin 기준 anchor를 COM 기준 lever arm으로 바꾸고 hard Weld 안정화 계수를 준비함.
[[nodiscard]] weldJointConstraint2 prepareWeldJointConstraint( const weldJointSim2& joint, const bodySim& bodySimA, const bodySim& bodySimB, float subStepTime );

// 이전 substep의 선형/각도 누적 임펄스를 두 Body에 반대로 적용함.
void warmStartWeldJointConstraint( const weldJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB );

// 상대 각도 scalar 제약을 푼 뒤 두 anchor를 일치시키는 2x2 선형 제약을 풂.
void solveWeldJointConstraint( weldJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB, bool useBias );

} // namespace zonai
