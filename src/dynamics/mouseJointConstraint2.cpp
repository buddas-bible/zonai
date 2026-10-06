#include "dynamics/mouseJointConstraint2.h"

#include <cassert>

namespace zonai
{
#pragma region Prepare
mouseJointConstraint2 prepareMouseJointConstraint( const mouseJointSim2& joint, const bodySim& body, float subStepTime )
{
    assert( subStepTime > 0.0f && joint.bodyIdB == body.bodyId );
    mouseJointConstraint2 constraint{};
    constraint.jointId = joint.jointId;
    constraint.bodyIdB = joint.bodyIdB;
    constraint.anchorB = Rotate( body.transform.rotation, joint.localAnchorB - body.localCenter );
    constraint.deltaCenter = body.center - joint.target;
    constraint.invMass = body.invMass;
    constraint.invInertia = body.invInertia;

    /*
    * anchor의 x/y 속도는 회전으로 결합되므로 2x2 유효 질량 행렬을 사용함.
    * K = [ k11 k12; k12 k22 ], M = inverse(K)
    * massX와 massY는 M의 열이며, M * rhs로 임펄스를 구함.
    */
    const vec2 r_b = constraint.anchorB;
    const float k11 = body.invMass + body.invInertia * r_b.y * r_b.y;
    const float k12 = -body.invInertia * r_b.x * r_b.y;
    const float k22 = body.invMass + body.invInertia * r_b.x * r_b.x;
    const float determinant = k11 * k22 - k12 * k12;
    const float inverse = determinant > 0.0f ? 1.0f / determinant : 0.0f;
    constraint.massX = { inverse * k22, -inverse * k12 };
    constraint.massY = { -inverse * k12, inverse * k11 };

    constraint.softness = makeConstraintSoftness( joint.hertz, joint.dampingRatio, subStepTime );
    constraint.maxImpulse = subStepTime * joint.maxForce;
    constraint.impulse = joint.subStepTime == subStepTime ? joint.impulse : vec2{};

    // Force 제한이 바뀌어도 warm start가 새 budget을 초과하지 않음.
    if( Length( constraint.impulse ) > constraint.maxImpulse )
    {
        constraint.impulse = constraint.maxImpulse * Normalize( constraint.impulse );
    }

    return constraint;
}
#pragma endregion

#pragma region WarmStart
void warmStartMouseJointConstraint( const mouseJointConstraint2& constraint, bodyState& state )
{
    const vec2 r_b = Rotate( state.deltaRotation, constraint.anchorB );
    state.linearVelocity += constraint.invMass * constraint.impulse;
    state.angularVelocity += constraint.invInertia * Cross( r_b, constraint.impulse );
}
#pragma endregion

#pragma region Solve
void solveMouseJointConstraint( mouseJointConstraint2& constraint, bodyState& state )
{
    const vec2 r_b = Rotate( state.deltaRotation, constraint.anchorB );
    const vec2 error = constraint.deltaCenter + state.deltaPosition + r_b;
    const vec2 v_p2 = state.linearVelocity + Cross( state.angularVelocity, r_b );
    const vec2 rhs = v_p2 + constraint.softness.biasRate * error;
    const vec2 correction = constraint.massX * rhs.x + constraint.massY * rhs.y;

    const vec2 oldImpulse = constraint.impulse;
    constraint.impulse += -constraint.softness.massScale * correction - constraint.softness.impulseScale * oldImpulse;

    // Box2D point spring처럼 누적 vector 전체를 |P| <= h*maxForce로 제한함.
    // Spring은 relax pass에서도 bias를 유지함. Rigid Distance의 bias/relax와 다름.
    if( Length( constraint.impulse ) > constraint.maxImpulse )
    {
        constraint.impulse = constraint.maxImpulse * Normalize( constraint.impulse );
    }

    // 이번 반복에서 증가한 임펄스만 속도에 반영함.
    const vec2 impulse = constraint.impulse - oldImpulse;
    state.linearVelocity += constraint.invMass * impulse;
    state.angularVelocity += constraint.invInertia * Cross( r_b, impulse );
}
#pragma endregion
} // namespace zonai
