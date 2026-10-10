#include "dynamics/joints/motorJointConstraint2.h"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace zonai
{

#pragma region Prepare

motorJointConstraint2 prepareMotorJointConstraint( const motorJointSim2& joint, const bodySim& bodySimA, const bodySim& bodySimB, float subStepTime )
{
    assert( std::isfinite( subStepTime ) && subStepTime > 0.0f );
    assert( joint.bodyIdA == bodySimA.bodyId && joint.bodyIdB == bodySimB.bodyId );

    motorJointConstraint2 constraint{};
    constraint.jointId = joint.jointId;
    constraint.bodyIdA = joint.bodyIdA;
    constraint.bodyIdB = joint.bodyIdB;

    // Joint는 Body origin 기준으로 저장하고 solver에서는 현재 COM 기준 lever arm을 사용함.
    constraint.anchorA = Rotate( bodySimA.transform.rotation, joint.localAnchorA - bodySimA.localCenter );
    constraint.anchorB = Rotate( bodySimB.transform.rotation, joint.localAnchorB - bodySimB.localCenter );
    constraint.invMassA = bodySimA.invMass;
    constraint.invMassB = bodySimB.invMass;
    constraint.invInertiaA = bodySimA.invInertia;
    constraint.invInertiaB = bodySimB.invInertia;

    constraint.linearVelocity = joint.linearVelocity;
    constraint.angularVelocity = joint.angularVelocity;
    constraint.maxLinearImpulse = joint.maxVelocityForce * subStepTime;
    constraint.maxAngularImpulse = joint.maxVelocityTorque * subStepTime;

    // timestep이 같을 때만 이전 누적 해를 warm start에 재사용함.
    if( joint.subStepTime == subStepTime )
    {
        constraint.linearVelocityImpulse = joint.linearVelocityImpulse;
        constraint.angularVelocityImpulse = joint.angularVelocityImpulse;
    }

    // runtime에서 힘 한도를 낮춘 경우 이전 cache도 새 한도를 넘지 않게 잘라냄.
    if( Length( constraint.linearVelocityImpulse ) > constraint.maxLinearImpulse )
    {
        constraint.linearVelocityImpulse = constraint.maxLinearImpulse * Normalize( constraint.linearVelocityImpulse );
    }
    constraint.angularVelocityImpulse = std::clamp( constraint.angularVelocityImpulse, -constraint.maxAngularImpulse, constraint.maxAngularImpulse );

    return constraint;
}

#pragma endregion Prepare

#pragma region WarmStart

void warmStartMotorJointConstraint( const motorJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB )
{
    const vec2 r_a = Rotate( bodyStateA.deltaRotation, constraint.anchorA );
    const vec2 r_b = Rotate( bodyStateB.deltaRotation, constraint.anchorB );
    const vec2 linearImpulse = constraint.linearVelocityImpulse;
    const float angularImpulse = constraint.angularVelocityImpulse;

    // dV = P / m, dW = (r x P + L) / I. A와 B에는 같은 impulse를 반대 방향으로 적용함.
    bodyStateA.linearVelocity -= constraint.invMassA * linearImpulse;
    bodyStateA.angularVelocity -= constraint.invInertiaA * ( Cross( r_a, linearImpulse ) + angularImpulse );
    bodyStateB.linearVelocity += constraint.invMassB * linearImpulse;
    bodyStateB.angularVelocity += constraint.invInertiaB * ( Cross( r_b, linearImpulse ) + angularImpulse );
}

#pragma endregion WarmStart

#pragma region Solve

void solveMotorJointConstraint( motorJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB )
{
    // 회전 motor는 Cdot = wB - wA - targetAngularVelocity를 0으로 만듦.
    const float angularK = constraint.invInertiaA + constraint.invInertiaB;
    if( constraint.maxAngularImpulse > 0.0f && angularK > 0.0f )
    {
        const float angularMass = 1.0f / angularK;
        const float velocityError = bodyStateB.angularVelocity - bodyStateA.angularVelocity - constraint.angularVelocity;
        const float deltaImpulse = -angularMass * velocityError;
        const float oldImpulse = constraint.angularVelocityImpulse;
        constraint.angularVelocityImpulse = std::clamp( oldImpulse + deltaImpulse, -constraint.maxAngularImpulse, constraint.maxAngularImpulse );
        const float impulse = constraint.angularVelocityImpulse - oldImpulse;

        bodyStateA.angularVelocity -= constraint.invInertiaA * impulse;
        bodyStateB.angularVelocity += constraint.invInertiaB * impulse;
    }

    // 현재 누적 회전을 반영한 작용점에서 상대속도를 구함.
    const vec2 r_a = Rotate( bodyStateA.deltaRotation, constraint.anchorA );
    const vec2 r_b = Rotate( bodyStateB.deltaRotation, constraint.anchorB );
    const vec2 pointVelocityA = bodyStateA.linearVelocity + Cross( bodyStateA.angularVelocity, r_a );
    const vec2 pointVelocityB = bodyStateB.linearVelocity + Cross( bodyStateB.angularVelocity, r_b );
    const vec2 velocityError = pointVelocityB - pointVelocityA - constraint.linearVelocity;

    if( constraint.maxLinearImpulse > 0.0f )
    {
        /*
        * 작용점이 COM 밖에 있으면 선형 impulse가 회전도 만들기 때문에 x/y를 2x2 block으로 함께 풂.
        * K = J M^-1 J^T, delta P = -inverse(K) * Cdot.
        * 이 제약은 목표 속도 자체가 물리 효과이므로 position bias나 relaxation용 임시 속도를 사용하지 않음.
        */
        const float k11 = constraint.invMassA + constraint.invMassB + constraint.invInertiaA * r_a.y * r_a.y + constraint.invInertiaB * r_b.y * r_b.y;
        const float k12 = -constraint.invInertiaA * r_a.x * r_a.y - constraint.invInertiaB * r_b.x * r_b.y;
        const float k22 = constraint.invMassA + constraint.invMassB + constraint.invInertiaA * r_a.x * r_a.x + constraint.invInertiaB * r_b.x * r_b.x;
        const float determinant = k11 * k22 - k12 * k12;
        const float invDet = determinant > 0.0f ? 1.0f / determinant : 0.0f;
        const vec2 correction{ invDet * ( k22 * velocityError.x - k12 * velocityError.y ), invDet * ( k11 * velocityError.y - k12 * velocityError.x ) };
        const vec2 deltaImpulse = -correction;

        const vec2 oldImpulse = constraint.linearVelocityImpulse;
        constraint.linearVelocityImpulse += deltaImpulse;
        if( Length( constraint.linearVelocityImpulse ) > constraint.maxLinearImpulse )
        {
            constraint.linearVelocityImpulse = constraint.maxLinearImpulse * Normalize( constraint.linearVelocityImpulse );
        }

        // 누적값 전체가 아니라 이번 iteration에서 증가한 양만 Body 속도에 적용함.
        const vec2 impulse = constraint.linearVelocityImpulse - oldImpulse;
        bodyStateA.linearVelocity -= constraint.invMassA * impulse;
        bodyStateA.angularVelocity -= constraint.invInertiaA * Cross( r_a, impulse );
        bodyStateB.linearVelocity += constraint.invMassB * impulse;
        bodyStateB.angularVelocity += constraint.invInertiaB * Cross( r_b, impulse );
    }
}

#pragma endregion Solve

} // namespace zonai
