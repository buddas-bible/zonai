#include "dynamics/revoluteJointConstraint2.h"

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

    return constraint;
}

#pragma endregion Prepare

#pragma region WarmStart

void warmStartRevoluteJointConstraint( const revoluteJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB )
{
    const vec2 r_a = Rotate( bodyStateA.deltaRotation, constraint.anchorA );
    const vec2 r_b = Rotate( bodyStateB.deltaRotation, constraint.anchorB );
    const vec2 impulse = constraint.impulse;

    // dV = P / m, dW = (r x P) / I. 같은 작용점 임펄스의 반작용을 A에 적용함.
    bodyStateA.linearVelocity -= constraint.invMassA * impulse;
    bodyStateA.angularVelocity -= constraint.invInertiaA * Cross( r_a, impulse );
    bodyStateB.linearVelocity += constraint.invMassB * impulse;
    bodyStateB.angularVelocity += constraint.invInertiaB * Cross( r_b, impulse );
}

#pragma endregion WarmStart

#pragma region Solve

void solveRevoluteJointConstraint( revoluteJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB, bool useBias )
{
    // Box2D처럼 현재 누적 회전을 반영한 작용점으로 매 반복의 K를 다시 구함.
    const vec2 r_a = Rotate( bodyStateA.deltaRotation, constraint.anchorA );
    const vec2 r_b = Rotate( bodyStateB.deltaRotation, constraint.anchorB );

    // 작용점 속도 v + w x r의 차이. 각도 자체를 고정하는 항은 없음.
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
