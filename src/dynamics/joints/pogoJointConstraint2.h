#pragma once

#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"
#include "dynamics/joints/pogoJointSim2.h"

namespace zonai
{

// 한 substep 동안 Pogo를 풀기 위한 임시 solver 데이터.
// 길이 측정 축과 실제 impulse 방향을 분리해 계단/경사면 대응에 사용함.
struct pogoJointConstraint2
{
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    vec2 anchorA{};
    vec2 anchorB{};
    vec2 deltaCenter{};
    vec2 pogoAxisB{ 0.0f, 1.0f };
    vec2 normal{ 0.0f, 1.0f };

    float invMassA = 0.0f;
    float invMassB = 0.0f;
    float invInertiaA = 0.0f;
    float invInertiaB = 0.0f;
    float linearMass = 0.0f;

    float restLength = 0.0f;
    float hertz = 0.0f;
    float dampingRatio = 0.0f;
    float maxTensionImpulse = 0.0f;
    float maxCompressionImpulse = 0.0f;

    float impulse = 0.0f;
    float velocity = 0.0f;
    float subStepTime = 0.0f;
};

[[nodiscard]] pogoJointConstraint2 preparePogoJointConstraint( const pogoJointSim2& joint, const bodySim& bodySimA, const bodySim& bodySimB, float subStepTime );
void warmStartPogoJointConstraint( const pogoJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB );
void solvePogoJointConstraint( pogoJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB, bool useBias );

} // namespace zonai
