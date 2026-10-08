#include "dynamics/joints/weldJointConstraint2.h"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace zonai
{

#pragma region Prepare

weldJointConstraint2 prepareWeldJointConstraint( const weldJointSim2& joint, const bodySim& bodySimA, const bodySim& bodySimB, float subStepTime )
{
    assert( std::isfinite( subStepTime ) && subStepTime > 0.0f );
    assert( joint.bodyIdA == bodySimA.bodyId && joint.bodyIdB == bodySimB.bodyId );

    weldJointConstraint2 constraint{};
    constraint.jointId = joint.jointId;
    constraint.bodyIdA = joint.bodyIdA;
    constraint.bodyIdB = joint.bodyIdB;

    constraint.anchorA = Rotate( bodySimA.transform.rotation, joint.localAnchorA - bodySimA.localCenter );
    constraint.anchorB = Rotate( bodySimB.transform.rotation, joint.localAnchorB - bodySimB.localCenter );
    constraint.deltaCenter = bodySimB.center - bodySimA.center;

    constraint.invMassA = bodySimA.invMass;
    constraint.invMassB = bodySimB.invMass;
    constraint.invInertiaA = bodySimA.invInertia;
    constraint.invInertiaB = bodySimB.invInertia;

    constraint.relativeRotation = Inverse( bodySimA.transform.rotation * rot2::FromRadians( joint.referenceAngle ) ) * bodySimB.transform.rotation;
    const float angularK = constraint.invInertiaA + constraint.invInertiaB;
    constraint.angularMass = angularK > 0.0f ? 1.0f / angularK : 0.0f;

    // 0 Hz hard constraint에 해당하는 기존 Joint 안정화 계수. Soft Weld는 다음 단계에서 별도 계수를 가짐.
    constraint.softness = makeConstraintSoftness( std::min( 60.0f, 0.25f / subStepTime ), 2.0f, subStepTime );

    const bool keepCache = joint.subStepTime == subStepTime;
    constraint.linearImpulse = keepCache ? joint.linearImpulse : vec2{};
    constraint.angularImpulse = keepCache && angularK > 0.0f ? joint.angularImpulse : 0.0f;

    return constraint;
}

#pragma endregion Prepare

#pragma region WarmStart

void warmStartWeldJointConstraint( const weldJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB )
{
    const vec2 rA = Rotate( bodyStateA.deltaRotation, constraint.anchorA );
    const vec2 rB = Rotate( bodyStateB.deltaRotation, constraint.anchorB );

    bodyStateA.linearVelocity -= constraint.invMassA * constraint.linearImpulse;
    bodyStateA.angularVelocity -= constraint.invInertiaA * ( Cross( rA, constraint.linearImpulse ) + constraint.angularImpulse );
    bodyStateB.linearVelocity += constraint.invMassB * constraint.linearImpulse;
    bodyStateB.angularVelocity += constraint.invInertiaB * ( Cross( rB, constraint.linearImpulse ) + constraint.angularImpulse );
}

#pragma endregion WarmStart

#pragma region Solve

void solveWeldJointConstraint( weldJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB, bool useBias )
{
    // Box2D의 현재 Weld처럼 각도와 선형 stiffness를 독립적으로 둘 수 있도록 3x3 block 대신 분리해서 풂.
    if( constraint.angularMass > 0.0f )
    {
        const rot2 rotation = Inverse( bodyStateA.deltaRotation ) * bodyStateB.deltaRotation * constraint.relativeRotation;
        const float angle = std::atan2( rotation.s, rotation.c );
        const float bias = useBias ? constraint.softness.biasRate * angle : 0.0f;
        const float massScale = useBias ? constraint.softness.massScale : 1.0f;
        const float impulseScale = useBias ? constraint.softness.impulseScale : 0.0f;
        const float velocity = bodyStateB.angularVelocity - bodyStateA.angularVelocity;
        const float impulse = -massScale * constraint.angularMass * ( velocity + bias ) - impulseScale * constraint.angularImpulse;

        constraint.angularImpulse += impulse;
        bodyStateA.angularVelocity -= constraint.invInertiaA * impulse;
        bodyStateB.angularVelocity += constraint.invInertiaB * impulse;
    }

    const vec2 rA = Rotate( bodyStateA.deltaRotation, constraint.anchorA );
    const vec2 rB = Rotate( bodyStateB.deltaRotation, constraint.anchorB );
    const vec2 velocityA = bodyStateA.linearVelocity + Cross( bodyStateA.angularVelocity, rA );
    const vec2 velocityB = bodyStateB.linearVelocity + Cross( bodyStateB.angularVelocity, rB );
    const vec2 relativeVelocity = velocityB - velocityA;

    const vec2 separation = constraint.deltaCenter + bodyStateB.deltaPosition - bodyStateA.deltaPosition + rB - rA;
    const vec2 bias = useBias ? constraint.softness.biasRate * separation : vec2{};
    const float massScale = useBias ? constraint.softness.massScale : 1.0f;
    const float impulseScale = useBias ? constraint.softness.impulseScale : 0.0f;

    // Point-to-point Jacobian의 2x2 effective mass. Off-center anchor의 회전 coupling을 함께 풂.
    const float k11 = constraint.invMassA + constraint.invMassB + constraint.invInertiaA * rA.y * rA.y + constraint.invInertiaB * rB.y * rB.y;
    const float k12 = -constraint.invInertiaA * rA.x * rA.y - constraint.invInertiaB * rB.x * rB.y;
    const float k22 = constraint.invMassA + constraint.invMassB + constraint.invInertiaA * rA.x * rA.x + constraint.invInertiaB * rB.x * rB.x;
    const float determinant = k11 * k22 - k12 * k12;
    const float invDet = determinant > 0.0f ? 1.0f / determinant : 0.0f;
    const vec2 rhs = relativeVelocity + bias;
    const vec2 correction{ invDet * ( k22 * rhs.x - k12 * rhs.y ), invDet * ( k11 * rhs.y - k12 * rhs.x ) };
    const vec2 impulse = -massScale * correction - impulseScale * constraint.linearImpulse;

    constraint.linearImpulse += impulse;
    bodyStateA.linearVelocity -= constraint.invMassA * impulse;
    bodyStateA.angularVelocity -= constraint.invInertiaA * Cross( rA, impulse );
    bodyStateB.linearVelocity += constraint.invMassB * impulse;
    bodyStateB.angularVelocity += constraint.invInertiaB * Cross( rB, impulse );
}

#pragma endregion Solve

} // namespace zonai
