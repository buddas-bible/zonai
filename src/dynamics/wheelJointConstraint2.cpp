#include "dynamics/wheelJointConstraint2.h"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace zonai
{

#pragma region Prepare

wheelJointConstraint2 prepareWheelJointConstraint( const wheelJointSim2& joint, const bodySim& bodySimA, const bodySim& bodySimB, float subStepTime )
{
    assert( std::isfinite( subStepTime ) && subStepTime > 0.0f );
    assert( joint.bodyIdA == bodySimA.bodyId && joint.bodyIdB == bodySimB.bodyId );

    wheelJointConstraint2 constraint{};
    constraint.jointId = joint.jointId;
    constraint.bodyIdA = joint.bodyIdA;
    constraint.bodyIdB = joint.bodyIdB;
    constraint.anchorA = Rotate( bodySimA.transform.rotation, joint.localAnchorA - bodySimA.localCenter );
    constraint.anchorB = Rotate( bodySimB.transform.rotation, joint.localAnchorB - bodySimB.localCenter );
    constraint.deltaCenter = bodySimB.center - bodySimA.center;
    constraint.axisA = Rotate( bodySimA.transform.rotation, joint.localAxisA );
    constraint.invMassA = bodySimA.invMass;
    constraint.invMassB = bodySimB.invMass;
    constraint.invInertiaA = bodySimA.invInertia;
    constraint.invInertiaB = bodySimB.invInertia;

    /*
    * Box2D의 point-to-line 제약. A가 회전하면 이동 축도 회전하므로 A의 팔 길이는 d+r_a임.
    * J = [-axis, -(d+r_a) x axis, axis, r_b x axis], effectiveMass = 1 / (J * invMass * J^T).
    * 유효 질량은 전체 Step 시작에 한 번 준비하고, solve/warm start의 작용점·축·팔 길이는 현재 회전을 반영함.
    */
    const vec2 d = constraint.deltaCenter + constraint.anchorB - constraint.anchorA;
    const vec2 perpendicular = Cross( 1.0f, constraint.axisA );
    const float s1 = Cross( d + constraint.anchorA, perpendicular );
    const float s2 = Cross( constraint.anchorB, perpendicular );
    const float kp = constraint.invMassA + constraint.invMassB + constraint.invInertiaA * s1 * s1 + constraint.invInertiaB * s2 * s2;
    constraint.perpendicularMass = kp > 0.0f ? 1.0f / kp : 0.0f;
    const float a1 = Cross( d + constraint.anchorA, constraint.axisA );
    const float a2 = Cross( constraint.anchorB, constraint.axisA );
    const float ka = constraint.invMassA + constraint.invMassB + constraint.invInertiaA * a1 * a1 + constraint.invInertiaB * a2 * a2;
    constraint.axialMass = ka > 0.0f ? 1.0f / ka : 0.0f;

    // 수직 제약의 수치 안정화와 사용자가 지정한 물리 서스펜션을 구분함.
    constraint.softness = makeConstraintSoftness( std::min( 60.0f, 0.25f / subStepTime ), 2.0f, subStepTime );
    constraint.enableSpring = joint.enableSpring && joint.hertz > 0.0f;
    constraint.springSoftness = makeConstraintSoftness( joint.hertz, joint.dampingRatio, subStepTime );
    const bool sameStep = joint.subStepTime == subStepTime;
    constraint.impulse = sameStep && kp > 0.0f ? joint.impulse : 0.0f;
    constraint.springImpulse = sameStep && constraint.enableSpring && ka > 0.0f ? joint.springImpulse : 0.0f;

    constraint.enableLimit = joint.enableLimit;
    constraint.lowerTranslation = joint.lowerTranslation;
    constraint.upperTranslation = joint.upperTranslation;
    constraint.invSubStepTime = 1.0f / subStepTime;
    constraint.lowerImpulse = sameStep && constraint.enableLimit && ka > 0.0f ? joint.lowerImpulse : 0.0f;
    constraint.upperImpulse = sameStep && constraint.enableLimit && ka > 0.0f ? joint.upperImpulse : 0.0f;

    return constraint;
}

#pragma endregion Prepare

#pragma region WarmStart

void warmStartWheelJointConstraint( const wheelJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB )
{
    const vec2 r_a = Rotate( bodyStateA.deltaRotation, constraint.anchorA );
    const vec2 r_b = Rotate( bodyStateB.deltaRotation, constraint.anchorB );
    const vec2 d = constraint.deltaCenter + bodyStateB.deltaPosition - bodyStateA.deltaPosition + r_b - r_a;
    const vec2 axis = Rotate( bodyStateA.deltaRotation, constraint.axisA );
    const vec2 perpendicular = Cross( 1.0f, axis );

    // 하한은 양의 축 방향, 상한은 음의 축 방향임. A에는 회전하는 축의 반작용까지 포함함.
    const float axialImpulse = constraint.springImpulse + constraint.lowerImpulse - constraint.upperImpulse;
    const vec2 impulse = axialImpulse * axis + constraint.impulse * perpendicular;
    const float angularImpulseA = axialImpulse * Cross( d + r_a, axis ) + constraint.impulse * Cross( d + r_a, perpendicular );
    const float angularImpulseB = axialImpulse * Cross( r_b, axis ) + constraint.impulse * Cross( r_b, perpendicular );
    bodyStateA.linearVelocity -= constraint.invMassA * impulse;
    bodyStateA.angularVelocity -= constraint.invInertiaA * angularImpulseA;
    bodyStateB.linearVelocity += constraint.invMassB * impulse;
    bodyStateB.angularVelocity += constraint.invInertiaB * angularImpulseB;
}

#pragma endregion WarmStart

#pragma region Solve

void solveWheelJointConstraint( wheelJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB, bool useBias )
{
    const vec2 r_a = Rotate( bodyStateA.deltaRotation, constraint.anchorA );
    const vec2 r_b = Rotate( bodyStateB.deltaRotation, constraint.anchorB );
    const vec2 d = constraint.deltaCenter + bodyStateB.deltaPosition - bodyStateA.deltaPosition + r_b - r_a;
    const vec2 axis = Rotate( bodyStateA.deltaRotation, constraint.axisA );
    const float a1 = Cross( d + r_a, axis );
    const float a2 = Cross( r_b, axis );
    const float translation = Dot( axis, d );

    if( constraint.enableSpring )
    {
        const vec2 v_a = bodyStateA.linearVelocity;
        const float w_a = bodyStateA.angularVelocity;
        const vec2 v_b = bodyStateB.linearVelocity;
        const float w_b = bodyStateB.angularVelocity;
        const float velocity = Dot( axis, v_b - v_a ) + a2 * w_b - a1 * w_a;

        // Box2D처럼 물리 스프링은 relaxation에서도 복원 bias를 유지함. 0 Hz/off는 축을 잠그지 않음.
        const float bias = constraint.springSoftness.biasRate * translation;
        const float impulse = -constraint.springSoftness.massScale * constraint.axialMass * ( velocity + bias ) - constraint.springSoftness.impulseScale * constraint.springImpulse;
        constraint.springImpulse += impulse;
        const vec2 linearImpulse = impulse * axis;
        bodyStateA.linearVelocity -= constraint.invMassA * linearImpulse;
        bodyStateA.angularVelocity -= constraint.invInertiaA * impulse * a1;
        bodyStateB.linearVelocity += constraint.invMassB * linearImpulse;
        bodyStateB.angularVelocity += constraint.invInertiaB * impulse * a2;
    }

    if( constraint.enableLimit && constraint.axialMass > 0.0f )
    {
        auto solveLimit = [&]( float separation, float direction, float& accumulatedImpulse )
        {
            float bias = 0.0f;
            float massScale = 1.0f;
            float impulseScale = 0.0f;

            // 범위 안에서는 남은 거리 / h로 경계 통과를 예측함. 위반한 위치는 보정 pass에서만 줄임.
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

            // 하한/상한의 C와 Cdot 부호를 맞춰 누적 임펄스를 0 이상으로 제한함. 안쪽 복귀는 허용함.
            const float velocity = direction * ( Dot( axis, bodyStateB.linearVelocity - bodyStateA.linearVelocity ) + a2 * bodyStateB.angularVelocity - a1 * bodyStateA.angularVelocity );
            const float deltaImpulse = -massScale * constraint.axialMass * ( velocity + bias ) - impulseScale * accumulatedImpulse;
            const float oldImpulse = accumulatedImpulse;
            accumulatedImpulse = std::max( 0.0f, oldImpulse + deltaImpulse );
            const float impulse = direction * ( accumulatedImpulse - oldImpulse );
            const vec2 linearImpulse = impulse * axis;
            bodyStateA.linearVelocity -= constraint.invMassA * linearImpulse;
            bodyStateA.angularVelocity -= constraint.invInertiaA * impulse * a1;
            bodyStateB.linearVelocity += constraint.invMassB * linearImpulse;
            bodyStateB.angularVelocity += constraint.invInertiaB * impulse * a2;
        };

        // Box2D처럼 스프링 → 하한 → 상한 → 수직 제약 순서. 제한은 스프링 off/0 Hz에서도 작동함.
        solveLimit( translation - constraint.lowerTranslation, 1.0f, constraint.lowerImpulse );
        solveLimit( constraint.upperTranslation - translation, -1.0f, constraint.upperImpulse );
    }

    // 스프링과 제한이 갱신한 최신 속도로 수직 제약을 풂. 중심 밖 연결점도 각도를 고정하지는 않음.
    const vec2 perpendicular = Cross( 1.0f, axis );
    const float s1 = Cross( d + r_a, perpendicular );
    const float s2 = Cross( r_b, perpendicular );
    const vec2 v_a = bodyStateA.linearVelocity;
    const float w_a = bodyStateA.angularVelocity;
    const vec2 v_b = bodyStateB.linearVelocity;
    const float w_b = bodyStateB.angularVelocity;
    const float velocity = Dot( perpendicular, v_b - v_a ) + s2 * w_b - s1 * w_a;
    const float bias = useBias ? constraint.softness.biasRate * Dot( perpendicular, d ) : 0.0f;
    const float massScale = useBias ? constraint.softness.massScale : 1.0f;
    const float impulseScale = useBias ? constraint.softness.impulseScale : 0.0f;
    const float impulse = -massScale * constraint.perpendicularMass * ( velocity + bias ) - impulseScale * constraint.impulse;

    constraint.impulse += impulse;
    const vec2 linearImpulse = impulse * perpendicular;
    bodyStateA.linearVelocity -= constraint.invMassA * linearImpulse;
    bodyStateA.angularVelocity -= constraint.invInertiaA * impulse * s1;
    bodyStateB.linearVelocity += constraint.invMassB * linearImpulse;
    bodyStateB.angularVelocity += constraint.invInertiaB * impulse * s2;
}

#pragma endregion Solve

} // namespace zonai
