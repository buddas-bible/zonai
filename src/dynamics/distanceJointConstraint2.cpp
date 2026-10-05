#include "dynamics/distanceJointConstraint2.h"

#include <algorithm>
#include <cassert>

namespace zonai
{
#pragma region Prepare
distanceJointConstraint2 prepareDistanceJointConstraint( const distanceJointSim2& joint, const bodySim& bodyA, const bodySim& bodyB, float subStepTime )
{
    assert( subStepTime > 0.0f );
    assert( joint.bodyIdA == bodyA.bodyId && joint.bodyIdB == bodyB.bodyId );
    distanceJointConstraint2 constraint{};
    constraint.jointId = joint.jointId;
    constraint.bodyIdA = joint.bodyIdA;
    constraint.bodyIdB = joint.bodyIdB;
    // Body origin 기준 anchor를 COM 기준 lever arm으로 바꿔야 회전 Jacobian이 맞음.
    constraint.anchorA = Rotate( bodyA.transform.rotation, joint.localAnchorA - bodyA.localCenter );
    constraint.anchorB = Rotate( bodyB.transform.rotation, joint.localAnchorB - bodyB.localCenter );
    constraint.deltaCenter = bodyB.center - bodyA.center;
    constraint.invMassA = bodyA.invMass; constraint.invMassB = bodyB.invMass;
    constraint.invInertiaA = bodyA.invInertia; constraint.invInertiaB = bodyB.invInertia;
    constraint.length = joint.length;
    const vec2 axis = Normalize( constraint.deltaCenter + constraint.anchorB - constraint.anchorA );
    const float crossA = Cross( constraint.anchorA, axis );
    const float crossB = Cross( constraint.anchorB, axis );
    const float k = bodyA.invMass + bodyB.invMass + bodyA.invInertia * crossA * crossA + bodyB.invInertia * crossB * crossB;
    constraint.axialMass = k > 0.0f ? 1.0f / k : 0.0f;
    // 이전 h에서 만든 impulse를 새 h에 그대로 적용하지 않음.
    constraint.impulse = joint.subStepTime == subStepTime ? joint.impulse : 0.0f;
    // Box2D rigid distance의 수치 안정화. Spring 기능을 의미하는 softness가 아님.
    constraint.softness = makeConstraintSoftness( std::min( 60.0f, 0.25f / subStepTime ), 2.0f, subStepTime );
    return constraint;
}
#pragma endregion

#pragma region WarmStart
void warmStartDistanceJointConstraint( const distanceJointConstraint2& constraint, bodyState& stateA, bodyState& stateB )
{
    const vec2 rA = Rotate( stateA.deltaRotation, constraint.anchorA );
    const vec2 rB = Rotate( stateB.deltaRotation, constraint.anchorB );
    const vec2 axis = Normalize( constraint.deltaCenter + stateB.deltaPosition - stateA.deltaPosition + rB - rA );
    const vec2 impulse = constraint.impulse * axis;
    stateA.linearVelocity -= constraint.invMassA * impulse;
    stateA.angularVelocity -= constraint.invInertiaA * Cross( rA, impulse );
    stateB.linearVelocity += constraint.invMassB * impulse;
    stateB.angularVelocity += constraint.invInertiaB * Cross( rB, impulse );
}
#pragma endregion

#pragma region Solve
void solveDistanceJointConstraint( distanceJointConstraint2& constraint, bodyState& stateA, bodyState& stateB, bool useBias )
{
    const vec2 rA = Rotate( stateA.deltaRotation, constraint.anchorA );
    const vec2 rB = Rotate( stateB.deltaRotation, constraint.anchorB );
    const vec2 delta = constraint.deltaCenter + stateB.deltaPosition - stateA.deltaPosition + rB - rA;
    // 완전히 겹친 anchor는 방향을 정할 수 없어 correction이 0임. NaN을 만들지 않음.
    const vec2 axis = Normalize( delta );
    const vec2 relativeVelocity = stateB.linearVelocity + Cross( stateB.angularVelocity, rB ) - stateA.linearVelocity - Cross( stateA.angularVelocity, rA );
    const float bias = useBias ? constraint.softness.biasRate * ( Length( delta ) - constraint.length ) : 0.0f;
    const float massScale = useBias ? constraint.softness.massScale : 1.0f;
    const float impulseScale = useBias ? constraint.softness.impulseScale : 0.0f;
    const float deltaImpulse = -massScale * constraint.axialMass * ( Dot( axis, relativeVelocity ) + bias ) - impulseScale * constraint.impulse;
    // Contact와 달리 양방향 제약이므로 인장(음수) / 압축(양수) impulse를 모두 허용함.
    constraint.impulse += deltaImpulse;
    const vec2 impulse = deltaImpulse * axis;
    stateA.linearVelocity -= constraint.invMassA * impulse;
    stateA.angularVelocity -= constraint.invInertiaA * Cross( rA, impulse );
    stateB.linearVelocity += constraint.invMassB * impulse;
    stateB.angularVelocity += constraint.invInertiaB * Cross( rB, impulse );
}
#pragma endregion
} // namespace zonai
