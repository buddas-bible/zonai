#include "dynamics/joints/prismaticJointConstraint2.h"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace zonai
{

#pragma region Prepare

prismaticJointConstraint2 preparePrismaticJointConstraint( const prismaticJointSim2& joint, const bodySim& bodySimA, const bodySim& bodySimB, float subStepTime )
{
    assert( std::isfinite( subStepTime ) && subStepTime > 0.0f );
    assert( joint.bodyIdA == bodySimA.bodyId && joint.bodyIdB == bodySimB.bodyId );

    prismaticJointConstraint2 constraint{};
    constraint.jointId = joint.jointId;
    constraint.bodyIdA = joint.bodyIdA;
    constraint.bodyIdB = joint.bodyIdB;
    constraint.anchorA = Rotate( bodySimA.transform.rotation, joint.localAnchorA - bodySimA.localCenter );
    constraint.anchorB = Rotate( bodySimB.transform.rotation, joint.localAnchorB - bodySimB.localCenter );
    constraint.deltaCenter = bodySimB.center - bodySimA.center;
    constraint.axisA = Rotate( bodySimA.transform.rotation, joint.localAxisA );
    constraint.relativeRotation = Inverse( bodySimA.transform.rotation * rot2::FromRadians( joint.referenceAngle ) ) * bodySimB.transform.rotation;
    constraint.invMassA = bodySimA.invMass;
    constraint.invMassB = bodySimB.invMass;
    constraint.invInertiaA = bodySimA.invInertia;
    constraint.invInertiaB = bodySimB.invInertia;

    // Contact/기존 Joint와 같은 수치 안정화. Prismatic 자체의 물리 스프링은 아직 추가하지 않음.
    constraint.softness = makeConstraintSoftness( std::min( 60.0f, 0.25f / subStepTime ), 2.0f, subStepTime );
    const bool sameStep = joint.subStepTime == subStepTime;
    const bool hasResponse = constraint.invMassA + constraint.invMassB + constraint.invInertiaA + constraint.invInertiaB > 0.0f;
    constraint.impulse = sameStep && hasResponse ? joint.impulse : vec2{};

    constraint.enableLimit = joint.enableLimit;
    constraint.lowerTranslation = joint.lowerTranslation;
    constraint.upperTranslation = joint.upperTranslation;
    constraint.invSubStepTime = 1.0f / subStepTime;
    constraint.lowerImpulse = sameStep && constraint.enableLimit && hasResponse ? joint.lowerImpulse : 0.0f;
    constraint.upperImpulse = sameStep && constraint.enableLimit && hasResponse ? joint.upperImpulse : 0.0f;

    constraint.enableMotor = joint.enableMotor;
    constraint.motorSpeed = joint.motorSpeed;
    constraint.maxMotorImpulse = joint.maxMotorForce * subStepTime;
    constraint.motorImpulse = sameStep && constraint.enableMotor && hasResponse
        ? std::clamp( joint.motorImpulse, -constraint.maxMotorImpulse, constraint.maxMotorImpulse )
        : 0.0f;

    return constraint;
}

#pragma endregion Prepare

#pragma region WarmStart

void warmStartPrismaticJointConstraint( const prismaticJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB )
{
    const vec2 r_a = Rotate( bodyStateA.deltaRotation, constraint.anchorA );
    const vec2 r_b = Rotate( bodyStateB.deltaRotation, constraint.anchorB );
    const vec2 d = constraint.deltaCenter + bodyStateB.deltaPosition - bodyStateA.deltaPosition + r_b - r_a;
    const vec2 axis = Rotate( bodyStateA.deltaRotation, constraint.axisA );
    const vec2 perpendicular = Cross( 1.0f, axis );
    const float a1 = Cross( d + r_a, axis );
    const float a2 = Cross( r_b, axis );
    const float s1 = Cross( d + r_a, perpendicular );
    const float s2 = Cross( r_b, perpendicular );

    const float axialImpulse = constraint.motorImpulse + constraint.lowerImpulse - constraint.upperImpulse;
    const vec2 linearImpulse = axialImpulse * axis + constraint.impulse.x * perpendicular;
    const float angularImpulseA = axialImpulse * a1 + constraint.impulse.x * s1 + constraint.impulse.y;
    const float angularImpulseB = axialImpulse * a2 + constraint.impulse.x * s2 + constraint.impulse.y;

    bodyStateA.linearVelocity -= constraint.invMassA * linearImpulse;
    bodyStateA.angularVelocity -= constraint.invInertiaA * angularImpulseA;
    bodyStateB.linearVelocity += constraint.invMassB * linearImpulse;
    bodyStateB.angularVelocity += constraint.invInertiaB * angularImpulseB;
}

#pragma endregion WarmStart

#pragma region Solve

void solvePrismaticJointConstraint( prismaticJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB, bool useBias )
{
    const vec2 r_a = Rotate( bodyStateA.deltaRotation, constraint.anchorA );
    const vec2 r_b = Rotate( bodyStateB.deltaRotation, constraint.anchorB );
    const vec2 d = constraint.deltaCenter + bodyStateB.deltaPosition - bodyStateA.deltaPosition + r_b - r_a;
    const vec2 axis = Rotate( bodyStateA.deltaRotation, constraint.axisA );

    if( constraint.enableMotor )
    {
        const float a1 = Cross( d + r_a, axis );
        const float a2 = Cross( r_b, axis );
        const float k = constraint.invMassA + constraint.invMassB + constraint.invInertiaA * a1 * a1 + constraint.invInertiaB * a2 * a2;
        const float axialMass = k > 0.0f ? 1.0f / k : 0.0f;
        const float velocity = Dot( axis, bodyStateB.linearVelocity - bodyStateA.linearVelocity ) + a2 * bodyStateB.angularVelocity - a1 * bodyStateA.angularVelocity;
        const float deltaImpulse = axialMass * ( constraint.motorSpeed - velocity );
        const float oldImpulse = constraint.motorImpulse;
        constraint.motorImpulse = std::clamp( oldImpulse + deltaImpulse, -constraint.maxMotorImpulse, constraint.maxMotorImpulse );
        const float impulse = constraint.motorImpulse - oldImpulse;
        const vec2 linearImpulse = impulse * axis;

        bodyStateA.linearVelocity -= constraint.invMassA * linearImpulse;
        bodyStateA.angularVelocity -= constraint.invInertiaA * impulse * a1;
        bodyStateB.linearVelocity += constraint.invMassB * linearImpulse;
        bodyStateB.angularVelocity += constraint.invInertiaB * impulse * a2;
    }

    if( constraint.enableLimit )
    {
        const float a1 = Cross( d + r_a, axis );
        const float a2 = Cross( r_b, axis );
        const float k = constraint.invMassA + constraint.invMassB + constraint.invInertiaA * a1 * a1 + constraint.invInertiaB * a2 * a2;
        const float axialMass = k > 0.0f ? 1.0f / k : 0.0f;
        const float translation = Dot( axis, d );

        auto solveLimit = [&]( float separation, float direction, float& accumulatedImpulse )
        {
            float bias = 0.0f;
            float massScale = 1.0f;
            float impulseScale = 0.0f;

            // 경계 안에서는 C / h로 다음 substep의 경계 통과를 미리 막고, 위반한 오차는 bias pass에서만 줄임.
            if( separation > 0.0f )
            {
                bias = separation * constraint.invSubStepTime;
            }
            else if( useBias )
            {
                bias = constraint.softness.biasRate * separation;
                massScale = constraint.softness.massScale;
                impulseScale = constraint.softness.impulseScale;
            }

            const float velocity = direction * ( Dot( axis, bodyStateB.linearVelocity - bodyStateA.linearVelocity ) + a2 * bodyStateB.angularVelocity - a1 * bodyStateA.angularVelocity );
            const float deltaImpulse = -massScale * axialMass * ( velocity + bias ) - impulseScale * accumulatedImpulse;
            const float oldImpulse = accumulatedImpulse;
            accumulatedImpulse = std::max( 0.0f, oldImpulse + deltaImpulse );
            const float impulse = direction * ( accumulatedImpulse - oldImpulse );
            const vec2 linearImpulse = impulse * axis;

            bodyStateA.linearVelocity -= constraint.invMassA * linearImpulse;
            bodyStateA.angularVelocity -= constraint.invInertiaA * impulse * a1;
            bodyStateB.linearVelocity += constraint.invMassB * linearImpulse;
            bodyStateB.angularVelocity += constraint.invInertiaB * impulse * a2;
        };

        // 모터가 만든 축속도도 경계를 통과하지 못하도록 그 다음에 limit을 적용함.
        solveLimit( translation - constraint.lowerTranslation, 1.0f, constraint.lowerImpulse );
        solveLimit( constraint.upperTranslation - translation, -1.0f, constraint.upperImpulse );
    }

    const vec2 perpendicular = Cross( 1.0f, axis );
    const float s1 = Cross( d + r_a, perpendicular );
    const float s2 = Cross( r_b, perpendicular );

    const vec2 v_a = bodyStateA.linearVelocity;
    const float w_a = bodyStateA.angularVelocity;
    const vec2 v_b = bodyStateB.linearVelocity;
    const float w_b = bodyStateB.angularVelocity;
    vec2 velocity{};
    velocity.x = Dot( perpendicular, v_b - v_a ) + s2 * w_b - s1 * w_a;
    velocity.y = w_b - w_a;

    vec2 bias{};
    float massScale = 1.0f;
    float impulseScale = 0.0f;
    if( useBias )
    {
        const rot2 rotation = Inverse( bodyStateA.deltaRotation ) * bodyStateB.deltaRotation * constraint.relativeRotation;
        const vec2 separation{ Dot( perpendicular, d ), std::atan2( rotation.s, rotation.c ) };
        bias = constraint.softness.biasRate * separation;
        massScale = constraint.softness.massScale;
        impulseScale = constraint.softness.impulseScale;
    }

    /*
    * Prismatic은 축 수직 이동과 상대 회전을 함께 잠금.
    * J = [-p, -s1, p, s2; 0, -1, 0, 1], K = J * invM * J^T.
    * 중심 밖 작용점에서는 두 행이 결합되므로 같은 2x2 block에서 풀어야 레일이 단단함.
    */
    const float k11 = constraint.invMassA + constraint.invMassB + constraint.invInertiaA * s1 * s1 + constraint.invInertiaB * s2 * s2;
    const float k12 = constraint.invInertiaA * s1 + constraint.invInertiaB * s2;
    const float k22 = constraint.invInertiaA + constraint.invInertiaB;
    const float determinant = k11 * k22 - k12 * k12;
    const vec2 rhs = velocity + bias;

    vec2 correction{};
    constexpr float determinantEpsilon = 1.0e-12f;
    if( determinant > determinantEpsilon )
    {
        const float invDet = 1.0f / determinant;
        correction.x = invDet * ( k22 * rhs.x - k12 * rhs.y );
        correction.y = invDet * ( k11 * rhs.y - k12 * rhs.x );
    }
    else
    {
        // 고정 회전처럼 한 행만 반응할 수 있는 경우 지원되는 행만 scalar로 풀어 NaN을 만들지 않음.
        correction.x = k11 > 0.0f ? rhs.x / k11 : 0.0f;
        correction.y = k22 > 0.0f ? rhs.y / k22 : 0.0f;
    }

    const vec2 deltaImpulse = -massScale * correction - impulseScale * constraint.impulse;
    constraint.impulse += deltaImpulse;

    const vec2 linearImpulse = deltaImpulse.x * perpendicular;
    const float angularImpulseA = deltaImpulse.x * s1 + deltaImpulse.y;
    const float angularImpulseB = deltaImpulse.x * s2 + deltaImpulse.y;
    bodyStateA.linearVelocity -= constraint.invMassA * linearImpulse;
    bodyStateA.angularVelocity -= constraint.invInertiaA * angularImpulseA;
    bodyStateB.linearVelocity += constraint.invMassB * linearImpulse;
    bodyStateB.angularVelocity += constraint.invInertiaB * angularImpulseB;
}

#pragma endregion Solve

} // namespace zonai
