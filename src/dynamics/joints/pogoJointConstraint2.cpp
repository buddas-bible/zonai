#include "dynamics/joints/pogoJointConstraint2.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <numbers>

namespace zonai
{

namespace
{

float springDamperVelocity( float hertz, float dampingRatio, float position, float velocity, float timeStep )
{
    const float omega = 2.0f * std::numbers::pi_v<float> * hertz;
    const float omegaH = omega * timeStep;
    return ( velocity - omega * omegaH * position ) / ( 1.0f + 2.0f * dampingRatio * omegaH + omegaH * omegaH );
}

}

#pragma region Prepare

pogoJointConstraint2 preparePogoJointConstraint( const pogoJointSim2& joint, const bodySim& bodySimA, const bodySim& bodySimB, float subStepTime )
{
    assert( std::isfinite( subStepTime ) && subStepTime > 0.0f );
    assert( joint.bodyIdA == bodySimA.bodyId && joint.bodyIdB == bodySimB.bodyId );

    pogoJointConstraint2 constraint{};
    constraint.jointId = joint.jointId;
    constraint.bodyIdA = joint.bodyIdA;
    constraint.bodyIdB = joint.bodyIdB;
    constraint.anchorA = Rotate( bodySimA.transform.rotation, joint.localAnchorA - bodySimA.localCenter );
    constraint.anchorB = Rotate( bodySimB.transform.rotation, joint.localAnchorB - bodySimB.localCenter );
    constraint.deltaCenter = bodySimB.center - bodySimA.center;
    constraint.pogoAxisB = Rotate( bodySimB.transform.rotation, joint.localPogoAxisB );
    constraint.normal = joint.normal;
    constraint.invMassA = bodySimA.invMass;
    constraint.invMassB = bodySimB.invMass;
    constraint.invInertiaA = bodySimA.invInertia;
    constraint.invInertiaB = bodySimB.invInertia;

    const float crA = Cross( constraint.anchorA, constraint.normal );
    const float crB = Cross( constraint.anchorB, constraint.normal );
    const float inverseMass = constraint.invMassA + constraint.invMassB + constraint.invInertiaA * crA * crA + constraint.invInertiaB * crB * crB;
    constraint.linearMass = inverseMass > 0.0f ? 1.0f / inverseMass : 0.0f;

    constraint.restLength = joint.restLength;
    constraint.hertz = joint.hertz;
    constraint.dampingRatio = joint.dampingRatio;
    constraint.maxTensionImpulse = subStepTime * joint.maxTensionForce;
    constraint.maxCompressionImpulse = subStepTime * joint.maxCompressionForce;
    constraint.subStepTime = subStepTime;

    if( joint.hertz > 0.0f )
    {
        // 새로 만든 Pogo(subStepTime==0)는 전달받은 상태를 seed로 쓰고, 기존 Joint는 같은 h에서만 impulse를 warm start함.
        constraint.velocity = joint.velocity;
        if( joint.subStepTime == 0.0f || joint.subStepTime == subStepTime )
        {
            constraint.impulse = std::clamp( joint.impulse, -constraint.maxTensionImpulse, constraint.maxCompressionImpulse );
        }
    }

    return constraint;
}

#pragma endregion Prepare

#pragma region WarmStart

void warmStartPogoJointConstraint( const pogoJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB )
{
    if( constraint.hertz == 0.0f || constraint.impulse == 0.0f ) return;

    const vec2 rA = Rotate( bodyStateA.deltaRotation, constraint.anchorA );
    const vec2 rB = Rotate( bodyStateB.deltaRotation, constraint.anchorB );
    const vec2 impulse = constraint.impulse * constraint.normal;

    bodyStateA.linearVelocity -= constraint.invMassA * impulse;
    bodyStateA.angularVelocity -= constraint.invInertiaA * Cross( rA, impulse );
    bodyStateB.linearVelocity += constraint.invMassB * impulse;
    bodyStateB.angularVelocity += constraint.invInertiaB * Cross( rB, impulse );
}

#pragma endregion WarmStart

#pragma region Solve

void solvePogoJointConstraint( pogoJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB, bool useBias )
{
    if( constraint.hertz == 0.0f || constraint.linearMass == 0.0f )
    {
        constraint.impulse = 0.0f;
        return;
    }

    const vec2 rA = Rotate( bodyStateA.deltaRotation, constraint.anchorA );
    const vec2 rB = Rotate( bodyStateB.deltaRotation, constraint.anchorB );

    float bias = 0.0f;
    if( useBias )
    {
        // Box2D처럼 Pogo 길이 측정축은 Prepare 시점의 방향을 substep 동안 고정함.
        // anchor lever arm은 deltaRotation을 따라가지만 측정축까지 다시 돌리면 회전이 Pogo 길이 오차를 왜곡함.
        const vec2 d = constraint.deltaCenter + bodyStateB.deltaPosition - bodyStateA.deltaPosition + rB - rA;
        const float positionError = Dot( constraint.pogoAxisB, d ) - constraint.restLength;

        // 보정 속도는 이번 위치 복원 pass에서만 사용함. relaxation pass에는 남기지 않아 mover가 계단에서 튀는 것을 줄임.
        constraint.velocity = springDamperVelocity( constraint.hertz, constraint.dampingRatio, positionError, constraint.velocity, constraint.subStepTime );
        bias = -constraint.velocity;
    }

    const vec2 velocityA = bodyStateA.linearVelocity + Cross( bodyStateA.angularVelocity, rA );
    const vec2 velocityB = bodyStateB.linearVelocity + Cross( bodyStateB.angularVelocity, rB );
    const float relativeVelocity = Dot( constraint.normal, velocityB - velocityA );

    const float oldImpulse = constraint.impulse;
    const float deltaImpulse = -constraint.linearMass * ( relativeVelocity + bias );
    constraint.impulse = std::clamp( oldImpulse + deltaImpulse, -constraint.maxTensionImpulse, constraint.maxCompressionImpulse );
    const float appliedImpulse = constraint.impulse - oldImpulse;
    const vec2 impulse = appliedImpulse * constraint.normal;

    bodyStateA.linearVelocity -= constraint.invMassA * impulse;
    bodyStateA.angularVelocity -= constraint.invInertiaA * Cross( rA, impulse );
    bodyStateB.linearVelocity += constraint.invMassB * impulse;
    bodyStateB.angularVelocity += constraint.invInertiaB * Cross( rB, impulse );
}

#pragma endregion Solve

} // namespace zonai
