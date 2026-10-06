#pragma once

#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"
#include "dynamics/constraintSoftness2.h"
#include "dynamics/revoluteJointSim2.h"

namespace zonai
{

struct revoluteJointConstraint2
{
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    // Step 시작 시 질량 중심에서 작용점까지의 벡터.
    vec2 anchorA{};
    vec2 anchorB{};
    vec2 deltaCenter{};

    float invMassA = 0.0f;
    float invMassB = 0.0f;
    float invInertiaA = 0.0f;
    float invInertiaB = 0.0f;
    vec2 impulse{};
    constraintSoftness2 softness{};
};

// 원점 기준 작용점을 질량 중심 기준으로 바꾸고 수치 안정화 계수를 준비함.
[[nodiscard]] revoluteJointConstraint2 prepareRevoluteJointConstraint( const revoluteJointSim2& joint, const bodySim& bodySimA, const bodySim& bodySimB, float subStepTime );

// 이전 누적 임펄스를 두 작용점에 반대 방향으로 적용함.
void warmStartRevoluteJointConstraint( const revoluteJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB );

// 연결점의 상대속도를 제거함. 위치 오차는 useBias pass에서만 보정함.
void solveRevoluteJointConstraint( revoluteJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB, bool useBias );

} // namespace zonai
