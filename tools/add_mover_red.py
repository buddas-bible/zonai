from pathlib import Path

Path("src/dynamics/joints/moverJointSim2.h").write_text(r'''#pragma once

#include <cstdint>
#include "math/vec2.h"

namespace zonai
{

// World의 stable Joint slot에 계속 보관되는 Mover Joint의 simulation 상태.
// 회전은 제어하지 않고 Body B가 A에 대해 가져야 하는 상대 선속도만 정의함.
struct moverJointSim2
{
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    // World 좌표계의 목표 상대 선속도와 x/y 방향별 최대 구동 힘.
    vec2 linearVelocity{};
    vec2 maxVelocityForce{};
};

} // namespace zonai
''', encoding="utf-8")

Path("src/dynamics/joints/moverJointConstraint2.h").write_text(r'''#pragma once

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

// 목표 상대 선속도에 필요한 누적 impulse를 x/y 각각의 actuator 한도로 제한해 적용함.
void solveMoverJointConstraint( moverJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB );

} // namespace zonai
''', encoding="utf-8")

Path("src/dynamics/joints/moverJointConstraint2.cpp").write_text(r'''#include "dynamics/joints/moverJointConstraint2.h"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace zonai
{

moverJointConstraint2 prepareMoverJointConstraint( const moverJointSim2& joint, const bodySim& bodySimA, const bodySim& bodySimB, float subStepTime )
{
    assert( std::isfinite( subStepTime ) && subStepTime > 0.0f );
    assert( joint.bodyIdA == bodySimA.bodyId && joint.bodyIdB == bodySimB.bodyId );

    moverJointConstraint2 constraint{};
    constraint.jointId = joint.jointId;
    constraint.bodyIdA = joint.bodyIdA;
    constraint.bodyIdB = joint.bodyIdB;
    constraint.invMassA = bodySimA.invMass;
    constraint.invMassB = bodySimB.invMass;

    const float inverseMass = constraint.invMassA + constraint.invMassB;
    constraint.linearMass = inverseMass > 0.0f ? 1.0f / inverseMass : 0.0f;
    constraint.linearVelocity = joint.linearVelocity;
    constraint.maxLinearImpulse = subStepTime * joint.maxVelocityForce;

    return constraint;
}

void solveMoverJointConstraint( moverJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB )
{
    if( constraint.linearMass == 0.0f ) return;

    const vec2 velocityError = bodyStateB.linearVelocity - bodyStateA.linearVelocity - constraint.linearVelocity;
    const vec2 deltaImpulse = -constraint.linearMass * velocityError;
    const vec2 oldImpulse = constraint.linearVelocityImpulse;

    constraint.linearVelocityImpulse += deltaImpulse;
    constraint.linearVelocityImpulse.x = std::clamp( constraint.linearVelocityImpulse.x, -constraint.maxLinearImpulse.x, constraint.maxLinearImpulse.x );
    constraint.linearVelocityImpulse.y = std::clamp( constraint.linearVelocityImpulse.y, -constraint.maxLinearImpulse.y, constraint.maxLinearImpulse.y );

    const vec2 impulse = constraint.linearVelocityImpulse - oldImpulse;
    bodyStateA.linearVelocity -= constraint.invMassA * impulse;
    bodyStateB.linearVelocity += constraint.invMassB * impulse;
}

} // namespace zonai
''', encoding="utf-8")
