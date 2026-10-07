#include "dynamics/joints/revoluteJointConstraint2.h"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace zonai
{

#pragma region Prepare

revoluteJointConstraint2 prepareRevoluteJointConstraint( const revoluteJointSim2& joint, const bodySim& bodySimA, const bodySim& bodySimB, float subStepTime )
{
    assert( std::isfinite( subStepTime ) && subStepTime > 0.0f );
    assert( joint.bodyIdA == bodySimA.bodyId && joint.bodyIdB == bodySimB.bodyId );

    revoluteJointConstraint2 constraint{};
    constraint.jointId = joint.jointId;
    constraint.bodyIdA = joint.bodyIdA;
    constraint.bodyIdB = joint.bodyIdB;

    // Box2D처럼 원점 기준으로 저장하고, solver에서는 질량 중심 기준의 회전 효과를 계산함.
    constraint.anchorA = Rotate( bodySimA.transform.rotation, joint.localAnchorA - bodySimA.localCenter );
    constraint.anchorB = Rotate( bodySimB.transform.rotation, joint.localAnchorB - bodySimB.localCenter );

    constraint.deltaCenter = bodySimB.center - bodySimA.center;
    constraint.invMassA = bodySimA.invMass;
    constraint.invMassB = bodySimB.invMass;
    constraint.invInertiaA = bodySimA.invInertia;
    constraint.invInertiaB = bodySimB.invInertia;

    // 기존 고정 거리 제약과 같은 수치 안정화. 물리 회전 스프링을 추가하는 설정은 아님.
    constraint.softness = makeConstraintSoftness( std::min( 60.0f, 0.25f / subStepTime ), 2.0f, subStepTime );
    constraint.impulse = joint.subStepTime == subStepTime ? joint.impulse : vec2{};

    constraint.relativeRotation = Inverse( bodySimA.transform.rotation * rot2::FromRadians( joint.referenceAngle ) ) * bodySimB.transform.rotation;
    constraint.enableLimit = joint.enableLimit;
    constraint.lowerAngle = joint.lowerAngle;
    constraint.upperAngle = joint.upperAngle;
    const float k = constraint.invInertiaA + constraint.invInertiaB;
    constraint.angularMass = k > 0.0f ? 1.0f / k : 0.0f;
    constraint.invSubStepTime = 1.0f / subStepTime;

    // 회전할 수 없거나 제한을 끈 상태에서는 과거 제한 임펄스를 적용하지 않음.
    const bool keepAngularImpulse = joint.subStepTime == subStepTime && joint.enableLimit && k > 0.0f;
    constraint.lowerImpulse = keepAngularImpulse ? joint.lowerImpulse : 0.0f;
    constraint.upperImpulse = keepAngularImpulse ? joint.upperImpulse : 0.0f;

    constraint.enableMotor = joint.enableMotor;
    constraint.motorSpeed = joint.motorSpeed;
    constraint.maxMotorImpulse = joint.maxMotorTorque * subStepTime;
    // Box2D처럼 토크 * h로 누적 임펄스를 제한함. 이전 값도 현재 한도를 넘지 않게 준비함.
    const bool keepMotorImpulse = joint.subStepTime == subStepTime && joint.enableMotor && k > 0.0f;
    constraint.motorImpulse = keepMotorImpulse ? std::clamp( joint.motorImpulse, -constraint.maxMotorImpulse, constraint.maxMotorImpulse ) : 0.0f;

    return constraint;
}

#pragma endregion Prepare

#pragma region WarmStart

void warmStartRevoluteJointConstraint( const revoluteJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB )
{
    const vec2 r_a = Rotate( bodyStateA.deltaRotation, constraint.anchorA );
    const vec2 r_b = Rotate( bodyStateB.deltaRotation, constraint.anchorB );
    const vec2 impulse = constraint.impulse;
    const float angularImpulse = constraint.motorImpulse + constraint.lowerImpulse - constraint.upperImpulse;

    // dV = P / m, dW = (r x P + L) / I. 연결점과 각도 임펄스의 반작용을 A에 적용함.
    bodyStateA.linearVelocity -= constraint.invMassA * impulse;
    bodyStateA.angularVelocity -= constraint.invInertiaA * ( Cross( r_a, impulse ) + angularImpulse );
    bodyStateB.linearVelocity += constraint.invMassB * impulse;
    bodyStateB.angularVelocity += constraint.invInertiaB * ( Cross( r_b, impulse ) + angularImpulse );
}

#pragma endregion WarmStart

#pragma region Solve

void solveRevoluteJointConstraint( revoluteJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB, bool useBias )
{
    if( constraint.enableMotor && constraint.angularMass > 0.0f )
    {
        // Cdot = wB - wA - motorSpeed. 속도 제약이므로 위치 bias 없이 제동·역회전도 같은 식으로 풂.
        const float velocity = bodyStateB.angularVelocity - bodyStateA.angularVelocity;
        const float deltaImpulse = constraint.angularMass * ( constraint.motorSpeed - velocity );
        const float oldImpulse = constraint.motorImpulse;
        constraint.motorImpulse = std::clamp( oldImpulse + deltaImpulse, -constraint.maxMotorImpulse, constraint.maxMotorImpulse );
        const float impulse = constraint.motorImpulse - oldImpulse;
        bodyStateA.angularVelocity -= constraint.invInertiaA * impulse;
        bodyStateB.angularVelocity += constraint.invInertiaB * impulse;
    }

    if( constraint.enableLimit && constraint.angularMass > 0.0f )
    {
        const rot2 rotation = Inverse( bodyStateA.deltaRotation ) * bodyStateB.deltaRotation * constraint.relativeRotation;
        const float angle = std::atan2( rotation.s, rotation.c );

        auto solveLimit = [&]( float separation, float direction, float& accumulatedImpulse )
        {
            float bias = 0.0f;
            float massScale = 1.0f;
            float impulseScale = 0.0f;

            // 범위 안에서는 남은 각도 / h로 경계 통과를 예측함. 위반한 위치는 보정 pass에서만 줄임.
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

            // Cdot = direction * (wB - wA), dP = -angularMass * (Cdot + bias).
            // 각 경계는 바깥 회전만 막으므로 누적 임펄스를 0 이상으로 제한함. 안쪽 복귀는 허용함.
            const float velocity = direction * ( bodyStateB.angularVelocity - bodyStateA.angularVelocity );
            const float deltaImpulse = -massScale * constraint.angularMass * ( velocity + bias ) - impulseScale * accumulatedImpulse;
            const float oldImpulse = accumulatedImpulse;
            accumulatedImpulse = std::max( 0.0f, oldImpulse + deltaImpulse );
            const float impulse = direction * ( accumulatedImpulse - oldImpulse );
            bodyStateA.angularVelocity -= constraint.invInertiaA * impulse;
            bodyStateB.angularVelocity += constraint.invInertiaB * impulse;
        };

        // Box2D처럼 모터 → 하한 → 상한 → 연결점 순서. 제한이 모터의 바깥 회전도 막음.
        solveLimit( angle - constraint.lowerAngle, 1.0f, constraint.lowerImpulse );
        solveLimit( constraint.upperAngle - angle, -1.0f, constraint.upperImpulse );
    }

    // Box2D처럼 현재 누적 회전을 반영한 작용점으로 매 반복의 K를 다시 구함.
    const vec2 r_a = Rotate( bodyStateA.deltaRotation, constraint.anchorA );
    const vec2 r_b = Rotate( bodyStateB.deltaRotation, constraint.anchorB );

    // 작용점 속도 v + w x r의 차이. 연결점 제약은 각도 자체를 고정하지 않음.
    const vec2 v_a = bodyStateA.linearVelocity;
    const float w_a = bodyStateA.angularVelocity;
    const vec2 v_b = bodyStateB.linearVelocity;
    const float w_b = bodyStateB.angularVelocity;
    const vec2 v_p2 = v_b + Cross( w_b, r_b );
    const vec2 v_r = v_p2 - v_a - Cross( w_a, r_a );

    const vec2 separation = constraint.deltaCenter + bodyStateB.deltaPosition - bodyStateA.deltaPosition + r_b - r_a;
    const vec2 bias = useBias ? constraint.softness.biasRate * separation : vec2{};
    const float massScale = useBias ? constraint.softness.massScale : 1.0f;
    const float impulseScale = useBias ? constraint.softness.impulseScale : 0.0f;

    /*
    * 작용점이 질량 중심 밖에 있으면 x/y 임펄스 모두 회전에 영향을 주므로 두 축을 함께 풂.
    * K = [ k11 k12; k12 k22 ], delta P = -massScale * inverse(K) * (v_r + bias) - impulseScale * P
    * 완화 pass는 위치 bias와 softness를 끄고 현재 작용점의 상대속도만 제거함.
    */
    const float k11 = constraint.invMassA + constraint.invMassB + constraint.invInertiaA * r_a.y * r_a.y + constraint.invInertiaB * r_b.y * r_b.y;
    const float k12 = -constraint.invInertiaA * r_a.x * r_a.y - constraint.invInertiaB * r_b.x * r_b.y;
    const float k22 = constraint.invMassA + constraint.invMassB + constraint.invInertiaA * r_a.x * r_a.x + constraint.invInertiaB * r_b.x * r_b.x;
    const float determinant = k11 * k22 - k12 * k12;
    const float invDet = determinant > 0.0f ? 1.0f / determinant : 0.0f;
    const vec2 rhs = v_r + bias;
    const vec2 correction{ invDet * ( k22 * rhs.x - k12 * rhs.y ), invDet * ( k11 * rhs.y - k12 * rhs.x ) };
    const vec2 impulse = -massScale * correction - impulseScale * constraint.impulse;

    constraint.impulse += impulse;

    // 이번 반복에서 증가한 임펄스만 적용함. 두 물체의 총 선형 운동량은 유지됨.
    bodyStateA.linearVelocity -= constraint.invMassA * impulse;
    bodyStateA.angularVelocity -= constraint.invInertiaA * Cross( r_a, impulse );
    bodyStateB.linearVelocity += constraint.invMassB * impulse;
    bodyStateB.angularVelocity += constraint.invInertiaB * Cross( r_b, impulse );
}

#pragma endregion Solve

} // namespace zonai
