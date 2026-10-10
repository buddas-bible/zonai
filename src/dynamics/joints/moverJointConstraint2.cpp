#include "dynamics/joints/moverJointConstraint2.h"

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
