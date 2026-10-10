#include "dynamics/joints/moverJointConstraint2.h"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace zonai
{

#pragma region Prepare

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

    // impulse는 force를 h만큼 적분한 값이라 같은 substep 시간에서만 이전 해를 재사용함.
    if( joint.subStepTime == subStepTime )
    {
        constraint.linearVelocityImpulse = joint.linearVelocityImpulse;
    }

    // runtime에서 힘 한도를 낮춘 경우 이전 cache도 새 축별 한도를 넘지 않게 제한함.
    constraint.linearVelocityImpulse.x = std::clamp( constraint.linearVelocityImpulse.x, -constraint.maxLinearImpulse.x, constraint.maxLinearImpulse.x );
    constraint.linearVelocityImpulse.y = std::clamp( constraint.linearVelocityImpulse.y, -constraint.maxLinearImpulse.y, constraint.maxLinearImpulse.y );

    return constraint;
}

#pragma endregion Prepare

#pragma region WarmStart

void warmStartMoverJointConstraint( const moverJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB )
{
    bodyStateA.linearVelocity -= constraint.invMassA * constraint.linearVelocityImpulse;
    bodyStateB.linearVelocity += constraint.invMassB * constraint.linearVelocityImpulse;
}

#pragma endregion WarmStart

#pragma region Solve

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

#pragma endregion Solve

} // namespace zonai
