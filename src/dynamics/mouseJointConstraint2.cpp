#include "dynamics/mouseJointConstraint2.h"

#include <cassert>

namespace zonai
{
#pragma region Prepare
mouseJointConstraint2 prepareMouseJointConstraint( const mouseJointSim2& joint, const bodySim& body, float subStepTime )
{
    assert( subStepTime > 0.0f && joint.bodyIdB == body.bodyId );
    mouseJointConstraint2 constraint{};
    constraint.jointId = joint.jointId; constraint.bodyIdB = joint.bodyIdB;
    constraint.anchorB = Rotate( body.transform.rotation, joint.localAnchorB - body.localCenter );
    constraint.deltaCenter = body.center - joint.target;
    constraint.invMass = body.invMass; constraint.invInertia = body.invInertia;
    // 점의 x/y 속도는 off-center torque로 결합됨. Scalar 두 개가 아닌 2x2 K를 뒤집음.
    const vec2 r = constraint.anchorB;
    const float a = body.invMass + body.invInertia * r.y * r.y;
    const float b = -body.invInertia * r.x * r.y;
    const float d = body.invMass + body.invInertia * r.x * r.x;
    const float determinant = a * d - b * b;
    const float inverse = determinant > 0.0f ? 1.0f / determinant : 0.0f;
    constraint.massX = { inverse * d, -inverse * b }; constraint.massY = { -inverse * b, inverse * a };
    constraint.softness = makeConstraintSoftness( joint.hertz, joint.dampingRatio, subStepTime );
    constraint.maxImpulse = subStepTime * joint.maxForce;
    constraint.impulse = joint.subStepTime == subStepTime ? joint.impulse : vec2{};
    // Force 제한이 바뀌어도 warm start가 새 budget을 초과하지 않음.
    if( Length( constraint.impulse ) > constraint.maxImpulse ) { constraint.impulse = constraint.maxImpulse * Normalize( constraint.impulse ); }
    return constraint;
}
#pragma endregion

#pragma region WarmStart
void warmStartMouseJointConstraint( const mouseJointConstraint2& constraint, bodyState& state )
{
    const vec2 r = Rotate( state.deltaRotation, constraint.anchorB );
    state.linearVelocity += constraint.invMass * constraint.impulse;
    state.angularVelocity += constraint.invInertia * Cross( r, constraint.impulse );
}
#pragma endregion

#pragma region Solve
void solveMouseJointConstraint( mouseJointConstraint2& constraint, bodyState& state )
{
    const vec2 r = Rotate( state.deltaRotation, constraint.anchorB );
    const vec2 error = constraint.deltaCenter + state.deltaPosition + r;
    const vec2 velocity = state.linearVelocity + Cross( state.angularVelocity, r );
    const vec2 rhs = velocity + constraint.softness.biasRate * error;
    const vec2 correction = constraint.massX * rhs.x + constraint.massY * rhs.y;
    const vec2 oldImpulse = constraint.impulse;
    constraint.impulse += -constraint.softness.massScale * correction - constraint.softness.impulseScale * oldImpulse;
    // Box2D point spring처럼 누적 vector 전체를 |P| <= h*maxForce로 제한함.
    // Spring은 relax pass에서도 bias를 유지함. Rigid Distance의 bias/relax와 다름.
    if( Length( constraint.impulse ) > constraint.maxImpulse ) { constraint.impulse = constraint.maxImpulse * Normalize( constraint.impulse ); }
    const vec2 impulse = constraint.impulse - oldImpulse;
    state.linearVelocity += constraint.invMass * impulse;
    state.angularVelocity += constraint.invInertia * Cross( r, impulse );
}
#pragma endregion
} // namespace zonai
