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
    assert( std::isfinite( joint.linearHertz ) && joint.linearHertz >= 0.0f );
    assert( std::isfinite( joint.linearDampingRatio ) && joint.linearDampingRatio >= 0.0f );
    assert( std::isfinite( joint.maxSpringForce ) && joint.maxSpringForce >= 0.0f );
    assert( std::isfinite( joint.referenceAngle ) );
    assert( std::isfinite( joint.angularHertz ) && joint.angularHertz >= 0.0f );
    assert( std::isfinite( joint.angularDampingRatio ) && joint.angularDampingRatio >= 0.0f );
    assert( std::isfinite( joint.maxSpringTorque ) && joint.maxSpringTorque >= 0.0f );

    motorJointConstraint2 constraint{};
    constraint.jointId = joint.jointId;
    constraint.bodyIdA = joint.bodyIdA;
    constraint.bodyIdB = joint.bodyIdB;

    // Joint는 Body origin 기준으로 저장하고 solver에서는 현재 COM 기준 lever arm을 사용함.
    constraint.anchorA = Rotate( bodySimA.transform.rotation, joint.localAnchorA - bodySimA.localCenter );
    constraint.anchorB = Rotate( bodySimB.transform.rotation, joint.localAnchorB - bodySimB.localCenter );
    constraint.deltaCenter = bodySimB.center - bodySimA.center;
    constraint.relativeRotation = Inverse( bodySimA.transform.rotation * rot2::FromRadians( joint.referenceAngle ) ) * bodySimB.transform.rotation;
    constraint.invMassA = bodySimA.invMass;
    constraint.invMassB = bodySimB.invMass;
    constraint.invInertiaA = bodySimA.invInertia;
    constraint.invInertiaB = bodySimB.invInertia;

    const float angularK = constraint.invInertiaA + constraint.invInertiaB;
    constraint.angularMass = angularK > 0.0f ? 1.0f / angularK : 0.0f;

    constraint.linearVelocity = joint.linearVelocity;
    constraint.angularVelocity = joint.angularVelocity;
    constraint.linearHertz = joint.linearHertz;
    constraint.angularHertz = joint.angularHertz;
    constraint.linearSpring = joint.linearHertz > 0.0f ? makeConstraintSoftness( joint.linearHertz, joint.linearDampingRatio, subStepTime ) : constraintSoftness2{};
    constraint.angularSpring = joint.angularHertz > 0.0f ? makeConstraintSoftness( joint.angularHertz, joint.angularDampingRatio, subStepTime ) : constraintSoftness2{};
    constraint.maxLinearImpulse = joint.maxVelocityForce * subStepTime;
    constraint.maxLinearSpringImpulse = joint.maxSpringForce * subStepTime;
    constraint.maxAngularImpulse = joint.maxVelocityTorque * subStepTime;
    constraint.maxAngularSpringImpulse = joint.maxSpringTorque * subStepTime;

    // timestep이 같을 때만 이전 누적 해를 warm start에 재사용함.
    if( joint.subStepTime == subStepTime )
    {
        constraint.linearVelocityImpulse = joint.linearVelocityImpulse;
        constraint.linearSpringImpulse = joint.linearSpringImpulse;
        constraint.angularVelocityImpulse = joint.angularVelocityImpulse;
        constraint.angularSpringImpulse = joint.angularSpringImpulse;
    }

    // runtime에서 힘 한도를 낮춘 경우 이전 cache도 새 한도를 넘지 않게 잘라냄.
    if( Length( constraint.linearVelocityImpulse ) > constraint.maxLinearImpulse )
    {
        constraint.linearVelocityImpulse = constraint.maxLinearImpulse * Normalize( constraint.linearVelocityImpulse );
    }
    if( Length( constraint.linearSpringImpulse ) > constraint.maxLinearSpringImpulse )
    {
        constraint.linearSpringImpulse = constraint.maxLinearSpringImpulse * Normalize( constraint.linearSpringImpulse );
    }
    constraint.angularVelocityImpulse = std::clamp( constraint.angularVelocityImpulse, -constraint.maxAngularImpulse, constraint.maxAngularImpulse );
    constraint.angularSpringImpulse = std::clamp( constraint.angularSpringImpulse, -constraint.maxAngularSpringImpulse, constraint.maxAngularSpringImpulse );

    return constraint;
}

#pragma endregion Prepare

#pragma region WarmStart

void warmStartMotorJointConstraint( const motorJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB )
{
    const vec2 r_a = Rotate( bodyStateA.deltaRotation, constraint.anchorA );
    const vec2 r_b = Rotate( bodyStateB.deltaRotation, constraint.anchorB );
    const vec2 linearImpulse = constraint.linearVelocityImpulse + constraint.linearSpringImpulse;
    const float angularImpulse = constraint.angularVelocityImpulse + constraint.angularSpringImpulse;

    // Velocity Motor와 transform spring은 cache를 따로 보관하지만 Body에는 두 물리 impulse의 합을 적용함.
    bodyStateA.linearVelocity -= constraint.invMassA * linearImpulse;
    bodyStateA.angularVelocity -= constraint.invInertiaA * ( Cross( r_a, linearImpulse ) + angularImpulse );
    bodyStateB.linearVelocity += constraint.invMassB * linearImpulse;
    bodyStateB.angularVelocity += constraint.invInertiaB * ( Cross( r_b, linearImpulse ) + angularImpulse );
}

#pragma endregion WarmStart

#pragma region Solve

void solveMotorJointConstraint( motorJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB )
{
    const vec2 r_a = Rotate( bodyStateA.deltaRotation, constraint.anchorA );
    const vec2 r_b = Rotate( bodyStateB.deltaRotation, constraint.anchorB );
    const float angularK = constraint.invInertiaA + constraint.invInertiaB;

    // Angular spring은 referenceAngle에 대한 상대각도 오차를 실제 spring-damper 토크로 줄임.
    if( constraint.angularHertz > 0.0f && constraint.maxAngularSpringImpulse > 0.0f && constraint.angularMass > 0.0f )
    {
        const rot2 rotation = Inverse( bodyStateA.deltaRotation ) * bodyStateB.deltaRotation * constraint.relativeRotation;
        const float angle = std::atan2( rotation.s, rotation.c );
        const float relativeVelocity = bodyStateB.angularVelocity - bodyStateA.angularVelocity;

        const float oldImpulse = constraint.angularSpringImpulse;
        const float deltaImpulse = -constraint.angularSpring.massScale * constraint.angularMass * ( relativeVelocity + constraint.angularSpring.biasRate * angle ) - constraint.angularSpring.impulseScale * oldImpulse;
        constraint.angularSpringImpulse = std::clamp( oldImpulse + deltaImpulse, -constraint.maxAngularSpringImpulse, constraint.maxAngularSpringImpulse );
        const float impulse = constraint.angularSpringImpulse - oldImpulse;

        bodyStateA.angularVelocity -= constraint.invInertiaA * impulse;
        bodyStateB.angularVelocity += constraint.invInertiaB * impulse;
    }

    // Linear spring은 anchor 위치 오차 C를 spring-damper bias로 바꿔 실제 복원 impulse를 만듦.
    // Contact의 penetration bias와 달리 spring impulse는 relax pass에서도 제거하지 않고 물리 응답으로 유지함.
    if( constraint.linearHertz > 0.0f && constraint.maxLinearSpringImpulse > 0.0f )
    {
        const vec2 pointVelocityA = bodyStateA.linearVelocity + Cross( bodyStateA.angularVelocity, r_a );
        const vec2 pointVelocityB = bodyStateB.linearVelocity + Cross( bodyStateB.angularVelocity, r_b );
        const vec2 relativeVelocity = pointVelocityB - pointVelocityA;
        const vec2 separation = constraint.deltaCenter + bodyStateB.deltaPosition - bodyStateA.deltaPosition + r_b - r_a;
        const vec2 rhs = relativeVelocity + constraint.linearSpring.biasRate * separation;

        const float k11 = constraint.invMassA + constraint.invMassB + constraint.invInertiaA * r_a.y * r_a.y + constraint.invInertiaB * r_b.y * r_b.y;
        const float k12 = -constraint.invInertiaA * r_a.x * r_a.y - constraint.invInertiaB * r_b.x * r_b.y;
        const float k22 = constraint.invMassA + constraint.invMassB + constraint.invInertiaA * r_a.x * r_a.x + constraint.invInertiaB * r_b.x * r_b.x;
        const float determinant = k11 * k22 - k12 * k12;
        const float invDet = determinant > 0.0f ? 1.0f / determinant : 0.0f;
        const vec2 correction{ invDet * ( k22 * rhs.x - k12 * rhs.y ), invDet * ( k11 * rhs.y - k12 * rhs.x ) };

        const vec2 oldImpulse = constraint.linearSpringImpulse;
        const vec2 deltaImpulse = -constraint.linearSpring.massScale * correction - constraint.linearSpring.impulseScale * oldImpulse;
        constraint.linearSpringImpulse += deltaImpulse;
        if( Length( constraint.linearSpringImpulse ) > constraint.maxLinearSpringImpulse )
        {
            constraint.linearSpringImpulse = constraint.maxLinearSpringImpulse * Normalize( constraint.linearSpringImpulse );
        }

        const vec2 impulse = constraint.linearSpringImpulse - oldImpulse;
        bodyStateA.linearVelocity -= constraint.invMassA * impulse;
        bodyStateA.angularVelocity -= constraint.invInertiaA * Cross( r_a, impulse );
        bodyStateB.linearVelocity += constraint.invMassB * impulse;
        bodyStateB.angularVelocity += constraint.invInertiaB * Cross( r_b, impulse );
    }

    const bool solveLinear = constraint.maxLinearImpulse > 0.0f;
    const bool solveAngular = constraint.maxAngularImpulse > 0.0f && angularK > 0.0f;

    if( solveLinear && solveAngular )
    {
        const vec2 pointVelocityA = bodyStateA.linearVelocity + Cross( bodyStateA.angularVelocity, r_a );
        const vec2 pointVelocityB = bodyStateB.linearVelocity + Cross( bodyStateB.angularVelocity, r_b );
        const vec2 linearError = pointVelocityB - pointVelocityA - constraint.linearVelocity;
        const float angularError = bodyStateB.angularVelocity - bodyStateA.angularVelocity - constraint.angularVelocity;

        /*
        * COM 밖의 anchor에서는 linear impulse가 r x P만큼 각속도도 바꾸므로 linear / angular Motor는 독립 제약이 아님.
        * 두 채널을 순서대로 풀면 뒤에서 적용한 linear impulse가 앞서 맞춘 angular target을 다시 깨뜨릴 수 있음.
        * 그래서 [Px, Py, L]을 하나의 3x3 block으로 풀어 같은 순간의 세 velocity error를 함께 제거함.
        */
        const float k11 = constraint.invMassA + constraint.invMassB + constraint.invInertiaA * r_a.y * r_a.y + constraint.invInertiaB * r_b.y * r_b.y;
        const float k12 = -constraint.invInertiaA * r_a.x * r_a.y - constraint.invInertiaB * r_b.x * r_b.y;
        const float k22 = constraint.invMassA + constraint.invMassB + constraint.invInertiaA * r_a.x * r_a.x + constraint.invInertiaB * r_b.x * r_b.x;
        const float k13 = -constraint.invInertiaA * r_a.y - constraint.invInertiaB * r_b.y;
        const float k23 = constraint.invInertiaA * r_a.x + constraint.invInertiaB * r_b.x;
        const float k33 = angularK;

        const float cofactor11 = k22 * k33 - k23 * k23;
        const float cofactor12 = k13 * k23 - k12 * k33;
        const float cofactor13 = k12 * k23 - k13 * k22;
        const float cofactor22 = k11 * k33 - k13 * k13;
        const float cofactor23 = k12 * k13 - k11 * k23;
        const float cofactor33 = k11 * k22 - k12 * k12;
        const float determinant = k11 * cofactor11 + k12 * cofactor12 + k13 * cofactor13;

        if( determinant > 0.0f )
        {
            const float invDet = 1.0f / determinant;
            const vec2 deltaLinearImpulse{
                -invDet * ( cofactor11 * linearError.x + cofactor12 * linearError.y + cofactor13 * angularError ),
                -invDet * ( cofactor12 * linearError.x + cofactor22 * linearError.y + cofactor23 * angularError )
            };
            const float deltaAngularImpulse = -invDet * ( cofactor13 * linearError.x + cofactor23 * linearError.y + cofactor33 * angularError );

            const vec2 oldLinearImpulse = constraint.linearVelocityImpulse;
            const float oldAngularImpulse = constraint.angularVelocityImpulse;
            constraint.linearVelocityImpulse += deltaLinearImpulse;
            constraint.angularVelocityImpulse += deltaAngularImpulse;

            // 힘과 토크 한도는 서로 다른 actuator 예산이므로 누적 impulse도 각각의 한도로 제한함.
            if( Length( constraint.linearVelocityImpulse ) > constraint.maxLinearImpulse )
            {
                constraint.linearVelocityImpulse = constraint.maxLinearImpulse * Normalize( constraint.linearVelocityImpulse );
            }
            constraint.angularVelocityImpulse = std::clamp( constraint.angularVelocityImpulse, -constraint.maxAngularImpulse, constraint.maxAngularImpulse );

            const vec2 linearImpulse = constraint.linearVelocityImpulse - oldLinearImpulse;
            const float angularImpulse = constraint.angularVelocityImpulse - oldAngularImpulse;
            bodyStateA.linearVelocity -= constraint.invMassA * linearImpulse;
            bodyStateA.angularVelocity -= constraint.invInertiaA * ( Cross( r_a, linearImpulse ) + angularImpulse );
            bodyStateB.linearVelocity += constraint.invMassB * linearImpulse;
            bodyStateB.angularVelocity += constraint.invInertiaB * ( Cross( r_b, linearImpulse ) + angularImpulse );
            return;
        }
    }

    // 한 채널만 활성화됐거나 3x3 block이 singular하면 풀 수 있는 제약을 독립 경로로 처리함.
    if( solveAngular )
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

    if( solveLinear )
    {
        const vec2 pointVelocityA = bodyStateA.linearVelocity + Cross( bodyStateA.angularVelocity, r_a );
        const vec2 pointVelocityB = bodyStateB.linearVelocity + Cross( bodyStateB.angularVelocity, r_b );
        const vec2 velocityError = pointVelocityB - pointVelocityA - constraint.linearVelocity;

        /*
        * linear Motor만 풀 때도 작용점이 COM 밖에 있으면 x/y impulse가 회전과 결합되므로 2x2 block으로 함께 풂.
        * K = J M^-1 J^T, delta P = -inverse(K) * Cdot.
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
