#pragma once

#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"
#include "dynamics/joints/moverJointSim2.h"

namespace zonai
{

// 한 substep 동안 Mover의 상대 선속도 제약을 푸는 임시 solver 데이터.
// Mover는 COM 선속도만 사용하므로 anchor와 관성 항이 없고 회전 상태를 건드리지 않음.
struct moverJointConstraint2
{
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    float invMassA = 0.0f;
    float invMassB = 0.0f;
    float linearMass = 0.0f;

    vec2 linearVelocity{};
    vec2 maxLinearImpulse{};
    vec2 linearVelocityImpulse{};
};

// 상대 선속도 effective mass와 force * h 형태의 축별 impulse 한도를 준비함.
[[nodiscard]] moverJointConstraint2 prepareMoverJointConstraint( const moverJointSim2& joint, const bodySim& bodySimA, const bodySim& bodySimB, float subStepTime );

// 이전 substep의 누적 선형 impulse를 다시 적용함. Mover는 각속도를 변경하지 않음.
void warmStartMoverJointConstraint( const moverJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB );

// 목표 상대 선속도에 필요한 누적 impulse를 x/y 각각의 actuator 한도로 제한해 적용함.
void solveMoverJointConstraint( moverJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB );

} // namespace zonai
