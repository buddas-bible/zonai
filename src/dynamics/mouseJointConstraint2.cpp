#include "dynamics/mouseJointConstraint2.h"

#include <cassert>

namespace zonai
{
#pragma region Prepare
mouseJointConstraint2 prepareMouseJointConstraint( const mouseJointSim2& joint, const bodySim& bodySimB, float subStepTime )
{
    assert( subStepTime > 0.0f && joint.bodyIdB == bodySimB.bodyId );

    mouseJointConstraint2 constraint{};
    constraint.jointId = joint.jointId;
    constraint.bodyIdB = joint.bodyIdB;

    // 클릭한 위치를 질량 중심 기준의 작용점으로 바꿈.
    constraint.anchorB = Rotate( bodySimB.transform.rotation, joint.localAnchorB - bodySimB.localCenter );
    constraint.deltaCenter = bodySimB.center - joint.target;
    constraint.invMass = bodySimB.invMass;
    constraint.invInertia = bodySimB.invInertia;

    /*
    * 작용점의 x/y 속도는 회전으로 결합되므로 2x2 행렬 K를 구하고 역행렬을 사용함.
    * K = [ k11 k12; k12 k22 ], M = inverse(K)
    * massX와 massY는 유효 질량 행렬 M의 열이며, M * rhs로 임펄스를 구함.
    */
    const vec2 r_b = constraint.anchorB;
    const float k11 = bodySimB.invMass + bodySimB.invInertia * r_b.y * r_b.y;
    const float k12 = -bodySimB.invInertia * r_b.x * r_b.y;
    const float k22 = bodySimB.invMass + bodySimB.invInertia * r_b.x * r_b.x;
    const float determinant = k11 * k22 - k12 * k12;
    // 행렬식이 0이면 역행렬이 없으므로 임펄스가 0이 되도록 둠.
    const float invDet = determinant > 0.0f ? 1.0f / determinant : 0.0f;
    constraint.massX = { invDet * k22, -invDet * k12 };
    constraint.massY = { -invDet * k12, invDet * k11 };

    constraint.softness = makeConstraintSoftness( joint.hertz, joint.dampingRatio, subStepTime );
    // P = F * dt. 이번 step에서 허용할 누적 임펄스의 크기.
    constraint.maxImpulse = subStepTime * joint.maxForce;
    constraint.impulse = joint.subStepTime == subStepTime ? joint.impulse : vec2{};

    // 최대 힘을 낮춘 경우에도 warm start가 새 임펄스 상한을 넘지 않도록 제한함.
    if( Length( constraint.impulse ) > constraint.maxImpulse )
    {
        constraint.impulse = constraint.maxImpulse * Normalize( constraint.impulse );
    }

    return constraint;
}
#pragma endregion Prepare

#pragma region WarmStart
void warmStartMouseJointConstraint( const mouseJointConstraint2& constraint, bodyState& bodyStateB )
{
    const vec2 r_b = Rotate( bodyStateB.deltaRotation, constraint.anchorB );
    // 선형 속도와 각속도에 임펄스를 적용함.
    bodyStateB.linearVelocity += constraint.invMass * constraint.impulse;
    bodyStateB.angularVelocity += constraint.invInertia * Cross( r_b, constraint.impulse );
}
#pragma endregion WarmStart

#pragma region Solve
void solveMouseJointConstraint( mouseJointConstraint2& constraint, bodyState& bodyStateB )
{
    const vec2 r_b = Rotate( bodyStateB.deltaRotation, constraint.anchorB );

    // 목표 위치에서 현재 작용점까지의 오차.
    const vec2 positionError = constraint.deltaCenter + bodyStateB.deltaPosition + r_b;

    // 작용점의 속도. 목표점은 별도의 물체 속도를 사용하지 않음.
    const vec2 v_p2 = bodyStateB.linearVelocity + Cross( bodyStateB.angularVelocity, r_b );

    // 위치 오차를 속도 bias로 환산하고, 유효 질량을 곱해 임펄스를 구함.
    const vec2 rhs = v_p2 + constraint.softness.biasRate * positionError;
    const vec2 impulseCorrection = constraint.massX * rhs.x + constraint.massY * rhs.y;

    const vec2 oldImpulse = constraint.impulse;
    constraint.impulse += -constraint.softness.massScale * impulseCorrection - constraint.softness.impulseScale * oldImpulse;

    // Box2D의 점 스프링처럼 누적 임펄스 벡터의 크기를 |P| <= dt * maxForce로 제한함.
    // 물리 스프링은 완화 pass에서도 위치 bias를 유지함.
    if( Length( constraint.impulse ) > constraint.maxImpulse )
    {
        constraint.impulse = constraint.maxImpulse * Normalize( constraint.impulse );
    }

    // 이번 반복에서 증가한 임펄스만 속도에 반영함.
    const vec2 impulse = constraint.impulse - oldImpulse;

    // dV = P / m, dW = (r x P) / I
    bodyStateB.linearVelocity += constraint.invMass * impulse;
    bodyStateB.angularVelocity += constraint.invInertia * Cross( r_b, impulse );
}
#pragma endregion Solve
} // namespace zonai
