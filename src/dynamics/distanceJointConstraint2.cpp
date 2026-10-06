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
    constraint.invMassA = bodyA.invMass;
    constraint.invMassB = bodyB.invMass;
    constraint.invInertiaA = bodyA.invInertia;
    constraint.invInertiaB = bodyB.invInertia;
    constraint.length = joint.length;
    // Box2D의 mode 우선순위: spring off 또는 같은 limit은 length를 고정하는 rigid임.
    constraint.enableSpring = joint.enableSpring && ( !joint.enableLimit || joint.minLength < joint.maxLength );
    constraint.hertz = joint.hertz;
    constraint.enableLimit = joint.enableLimit && constraint.enableSpring;
    constraint.minLength = joint.minLength;
    constraint.maxLength = joint.maxLength;
    constraint.invSubStepTime = 1.0f / subStepTime;

    /*
    * 축 방향의 유효 질량은 anchor의 회전 효과까지 포함함.
    * K = invMassA + invMassB + invInertiaA * (r_a x axis)^2 + invInertiaB * (r_b x axis)^2
    * axialMass = 1 / K. 움직일 수 없는 조합이면 0으로 둠.
    */
    const vec2 axis = Normalize( constraint.deltaCenter + constraint.anchorB - constraint.anchorA );
    const float crossA = Cross( constraint.anchorA, axis );
    const float crossB = Cross( constraint.anchorB, axis );
    const float k = bodyA.invMass + bodyB.invMass + bodyA.invInertia * crossA * crossA + bodyB.invInertia * crossB * crossB;
    constraint.axialMass = k > 0.0f ? 1.0f / k : 0.0f;

    // 이전 h에서 만든 impulse를 새 h에 그대로 적용하지 않음.
    const bool sameStep = joint.subStepTime == subStepTime;
    constraint.impulse = sameStep && ( !constraint.enableSpring || joint.hertz > 0.0f ) ? joint.impulse : 0.0f;
    constraint.lowerImpulse = sameStep && constraint.enableLimit ? joint.lowerImpulse : 0.0f;
    constraint.upperImpulse = sameStep && constraint.enableLimit ? joint.upperImpulse : 0.0f;

    // Rigid의 수치 안정화와 사용자가 지정하는 물리 spring 주파수를 구분함.
    constraint.limitSoftness = makeConstraintSoftness( std::min( 60.0f, 0.25f / subStepTime ), 2.0f, subStepTime );
    constraint.softness = constraint.enableSpring ? makeConstraintSoftness( joint.hertz, joint.dampingRatio, subStepTime ) : constraint.limitSoftness;

    return constraint;
}
#pragma endregion

#pragma region WarmStart
void warmStartDistanceJointConstraint( const distanceJointConstraint2& constraint, bodyState& stateA, bodyState& stateB )
{
    const vec2 r_a = Rotate( stateA.deltaRotation, constraint.anchorA );
    const vec2 r_b = Rotate( stateB.deltaRotation, constraint.anchorB );
    const vec2 axis = Normalize( constraint.deltaCenter + stateB.deltaPosition - stateA.deltaPosition + r_b - r_a );

    // Lower는 압축, upper는 인장 방향이므로 누적 임펄스의 부호가 반대임.
    const vec2 impulse = ( constraint.impulse + constraint.lowerImpulse - constraint.upperImpulse ) * axis;

    stateA.linearVelocity -= constraint.invMassA * impulse;
    stateA.angularVelocity -= constraint.invInertiaA * Cross( r_a, impulse );

    stateB.linearVelocity += constraint.invMassB * impulse;
    stateB.angularVelocity += constraint.invInertiaB * Cross( r_b, impulse );
}
#pragma endregion

#pragma region Solve
void solveDistanceJointConstraint( distanceJointConstraint2& constraint, bodyState& stateA, bodyState& stateB, bool useBias )
{
    const vec2 r_a = Rotate( stateA.deltaRotation, constraint.anchorA );
    const vec2 r_b = Rotate( stateB.deltaRotation, constraint.anchorB );
    const vec2 delta = constraint.deltaCenter + stateB.deltaPosition - stateA.deltaPosition + r_b - r_a;
    // 완전히 겹친 anchor는 방향을 정할 수 없어 correction이 0임. NaN을 만들지 않음.
    const vec2 axis = Normalize( delta );
    const float currentLength = Length( delta );

    // 각 제약을 푼 뒤 갱신된 속도로 다음 제약의 상대속도를 다시 계산함.
    auto axisVelocity = [&]()
    {
        return Dot( axis, stateB.linearVelocity + Cross( stateB.angularVelocity, r_b ) - stateA.linearVelocity - Cross( stateA.angularVelocity, r_a ) );
    };

    auto applyImpulse = [&]( float axialImpulse )
    {
        const vec2 impulse = axialImpulse * axis;
        stateA.linearVelocity -= constraint.invMassA * impulse;
        stateA.angularVelocity -= constraint.invInertiaA * Cross( r_a, impulse );
        stateB.linearVelocity += constraint.invMassB * impulse;
        stateB.angularVelocity += constraint.invInertiaB * Cross( r_b, impulse );
    };

    // Spring Hertz 0은 spring 힘만 끔. Limit은 독립적으로 계속 풀어야 함.
    if( !constraint.enableSpring || constraint.hertz > 0.0f )
    {
        // 물리 spring은 두 pass 모두 같은 softness/bias를 사용함. Rigid만 relax에서 bias를 제거함.
        const bool softPass = constraint.enableSpring || useBias;
        const float bias = softPass ? constraint.softness.biasRate * ( currentLength - constraint.length ) : 0.0f;
        const float massScale = softPass ? constraint.softness.massScale : 1.0f;
        const float impulseScale = softPass ? constraint.softness.impulseScale : 0.0f;
        const float deltaImpulse = -massScale * constraint.axialMass * ( axisVelocity() + bias ) - impulseScale * constraint.impulse;
        // Spring/rigid는 양방향 제약이므로 인장(음수) / 압축(양수)을 모두 허용함.
        constraint.impulse += deltaImpulse;
        applyImpulse( deltaImpulse );
    }

    if( constraint.enableLimit )
    {
        auto solveLimit = [&]( float separation, float direction, float& accumulatedImpulse )
        {
            float bias = 0.0f;
            float massScale = 1.0f;
            float impulseScale = 0.0f;

            // 범위 안에서는 남은 간격/h로 다음 적분의 boundary crossing을 미리 막음.
            // 이미 범위를 벗어났으면 bias pass만 오차를 보정하고 relax는 속도만 제한함.
            if( separation > 0.0f )
            {
                bias = separation * constraint.invSubStepTime;
            }
            else if( useBias )
            {
                bias = constraint.limitSoftness.biasRate * separation;
                massScale = constraint.limitSoftness.massScale;
                impulseScale = constraint.limitSoftness.impulseScale;
            }

            const float deltaImpulse = -massScale * constraint.axialMass * ( direction * axisVelocity() + bias ) - impulseScale * accumulatedImpulse;
            const float oldImpulse = accumulatedImpulse;
            accumulatedImpulse = std::max( 0.0f, oldImpulse + deltaImpulse );
            applyImpulse( direction * ( accumulatedImpulse - oldImpulse ) );
        };

        // Lower는 축 방향으로 밀고 upper는 반대 방향으로 당김. 각 cache는 비음수임.
        solveLimit( currentLength - constraint.minLength, 1.0f, constraint.lowerImpulse );
        solveLimit( constraint.maxLength - currentLength, -1.0f, constraint.upperImpulse );
    }
}
#pragma endregion
} // namespace zonai
