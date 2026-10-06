#include "dynamics/distanceJointConstraint2.h"

#include <algorithm>
#include <cassert>

namespace zonai
{
#pragma region Prepare
distanceJointConstraint2 prepareDistanceJointConstraint( const distanceJointSim2& joint, const bodySim& bodySimA, const bodySim& bodySimB, float subStepTime )
{
    assert( subStepTime > 0.0f );
    assert( joint.bodyIdA == bodySimA.bodyId && joint.bodyIdB == bodySimB.bodyId );

    distanceJointConstraint2 constraint{};
    constraint.jointId = joint.jointId;
    constraint.bodyIdA = joint.bodyIdA;
    constraint.bodyIdB = joint.bodyIdB;

    // 물체 원점 기준의 작용점을 질량 중심 기준으로 바꿈. 회전 효과는 이 거리로 계산함.
    constraint.anchorA = Rotate( bodySimA.transform.rotation, joint.localAnchorA - bodySimA.localCenter );
    constraint.anchorB = Rotate( bodySimB.transform.rotation, joint.localAnchorB - bodySimB.localCenter );
    constraint.deltaCenter = bodySimB.center - bodySimA.center;

    // 역질량과 역관성. 임펄스를 선형 속도와 각속도로 환산할 때 사용함.
    constraint.invMassA = bodySimA.invMass;
    constraint.invMassB = bodySimB.invMass;
    constraint.invInertiaA = bodySimA.invInertia;
    constraint.invInertiaB = bodySimB.invInertia;
    constraint.length = joint.length;

    // Box2D처럼 스프링을 끄거나 거리의 하한과 상한이 같으면 length를 고정함.
    constraint.enableSpring = joint.enableSpring && ( !joint.enableLimit || joint.minLength < joint.maxLength );
    constraint.hertz = joint.hertz;
    constraint.enableLimit = joint.enableLimit && constraint.enableSpring;
    constraint.minLength = joint.minLength;
    constraint.maxLength = joint.maxLength;
    constraint.invSubStepTime = 1.0f / subStepTime;

    // Box2D처럼 고정 거리 모드에서는 모터를 무시함. 0 Hz는 스프링 힘만 끄므로 모터는 유지함.
    constraint.enableMotor = joint.enableMotor && constraint.enableSpring;
    constraint.motorSpeed = joint.motorSpeed;
    constraint.maxMotorImpulse = subStepTime * joint.maxMotorForce;

    /*
    * 축 방향의 임펄스 공식에서 분모 K를 구함. 두 작용점의 회전 효과까지 포함함.
    * K = invMassA + invMassB + invInertiaA * (r_a x axis)^2 + invInertiaB * (r_b x axis)^2
    * axialMass = 1 / K. 두 물체가 움직일 수 없는 조합이면 0으로 둠.
    */
    const vec2 axis = Normalize( constraint.deltaCenter + constraint.anchorB - constraint.anchorA );
    const float crossA = Cross( constraint.anchorA, axis );
    const float crossB = Cross( constraint.anchorB, axis );
    const float k = bodySimA.invMass + bodySimB.invMass + bodySimA.invInertia * crossA * crossA + bodySimB.invInertia * crossB * crossB;
    constraint.axialMass = k > 0.0f ? 1.0f / k : 0.0f;

    // Warm start는 이전 누적 임펄스를 초기값으로 사용함. 시간 간격이 바뀌면 이전 값을 비움.
    const bool sameStep = joint.subStepTime == subStepTime;
    constraint.impulse = sameStep && ( !constraint.enableSpring || joint.hertz > 0.0f ) ? joint.impulse : 0.0f;
    constraint.lowerImpulse = sameStep && constraint.enableLimit ? joint.lowerImpulse : 0.0f;
    constraint.upperImpulse = sameStep && constraint.enableLimit ? joint.upperImpulse : 0.0f;
    constraint.motorImpulse = sameStep && constraint.enableMotor ? std::clamp( joint.motorImpulse, -constraint.maxMotorImpulse, constraint.maxMotorImpulse ) : 0.0f;

    // 고정 거리와 거리 제한은 수치 안정화용 계수를 사용함. 물리 스프링은 사용자가 지정한 주파수를 사용함.
    constraint.limitSoftness = makeConstraintSoftness( std::min( 60.0f, 0.25f / subStepTime ), 2.0f, subStepTime );
    constraint.softness = constraint.enableSpring ? makeConstraintSoftness( joint.hertz, joint.dampingRatio, subStepTime ) : constraint.limitSoftness;

    return constraint;
}
#pragma endregion Prepare

#pragma region WarmStart
void warmStartDistanceJointConstraint( const distanceJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB )
{
    // 질량 중심에서 현재 작용점까지의 벡터. Step 중 누적한 회전을 반영함.
    const vec2 r_a = Rotate( bodyStateA.deltaRotation, constraint.anchorA );
    const vec2 r_b = Rotate( bodyStateB.deltaRotation, constraint.anchorB );
    const vec2 axis = Normalize( constraint.deltaCenter + bodyStateB.deltaPosition - bodyStateA.deltaPosition + r_b - r_a );

    // 이전 step의 누적 임펄스. 하한은 축 방향으로 밀고 상한은 반대로 당기므로 부호가 반대임.
    const vec2 impulse = ( constraint.impulse + constraint.lowerImpulse - constraint.upperImpulse + constraint.motorImpulse ) * axis;

    bodyStateA.linearVelocity -= constraint.invMassA * impulse;
    bodyStateA.angularVelocity -= constraint.invInertiaA * Cross( r_a, impulse );

    bodyStateB.linearVelocity += constraint.invMassB * impulse;
    bodyStateB.angularVelocity += constraint.invInertiaB * Cross( r_b, impulse );
}
#pragma endregion WarmStart

#pragma region Solve
void solveDistanceJointConstraint( distanceJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB, bool useBias )
{
    // 질량 중심에서 현재 작용점까지의 벡터. Step 중 누적한 회전을 반영함.
    const vec2 r_a = Rotate( bodyStateA.deltaRotation, constraint.anchorA );
    const vec2 r_b = Rotate( bodyStateB.deltaRotation, constraint.anchorB );
    const vec2 delta = constraint.deltaCenter + bodyStateB.deltaPosition - bodyStateA.deltaPosition + r_b - r_a;
    // 작용점이 같으면 방향을 정할 수 없으므로 Normalize가 영벡터를 반환함.
    const vec2 axis = Normalize( delta );
    const float currentLength = Length( delta );

    // 각 제약을 푼 뒤 갱신된 속도로 다음 제약의 상대속도를 다시 계산함.
    auto computeAxialVelocity = [&]()
    {
        const vec2 v_a = bodyStateA.linearVelocity;
        const float w_a = bodyStateA.angularVelocity;
        const vec2 v_b = bodyStateB.linearVelocity;
        const float w_b = bodyStateB.angularVelocity;

        // 작용점의 속도 v + w x r에서 축 방향 상대속도를 구함.
        const vec2 v_p2 = v_b + Cross( w_b, r_b );
        const vec2 v_r = v_p2 - v_a - Cross( w_a, r_a );
        return Dot( axis, v_r );
    };

    auto applyAxialImpulse = [&]( float axialImpulse )
    {
        // 임펄스 벡터. dV = P / m, dW = (r x P) / I
        const vec2 impulse = axialImpulse * axis;
        bodyStateA.linearVelocity -= constraint.invMassA * impulse;
        bodyStateA.angularVelocity -= constraint.invInertiaA * Cross( r_a, impulse );
        bodyStateB.linearVelocity += constraint.invMassB * impulse;
        bodyStateB.angularVelocity += constraint.invInertiaB * Cross( r_b, impulse );
    };

    // 스프링 주파수가 0이면 스프링 힘만 끔. 거리 제한은 별도로 계산함.
    if( !constraint.enableSpring || constraint.hertz > 0.0f )
    {
        // Box2D처럼 물리 스프링은 보정과 완화 pass 모두 bias를 유지함. 고정 거리는 완화 pass에서 위치 보정을 끔.
        const bool softPass = constraint.enableSpring || useBias;
        const float bias = softPass ? constraint.softness.biasRate * ( currentLength - constraint.length ) : 0.0f;
        const float massScale = softPass ? constraint.softness.massScale : 1.0f;
        const float impulseScale = softPass ? constraint.softness.impulseScale : 0.0f;

        // 축 방향 속도와 위치 오차를 줄이는 임펄스 크기.
        const float deltaImpulse = -massScale * constraint.axialMass * ( computeAxialVelocity() + bias ) - impulseScale * constraint.impulse;
        // 스프링과 고정 거리 제약은 양방향으로 작용하므로 인장(음수)과 압축(양수)을 모두 허용함.
        constraint.impulse += deltaImpulse;
        applyAxialImpulse( deltaImpulse );
    }

    if( constraint.enableMotor )
    {
        /*
        * 현재 축속도를 목표 속도로 바꾸는 임펄스. 위치 bias 없이 두 pass에서 같은 식을 사용함.
        * dP = axialMass * (motorSpeed - v_r). 최대 힘 F는 시간 간격 h의 임펄스 F * h로 바꿈.
        * 반복 계산마다 더하는 값이 아니라 누적값을 제한해야 전체 모터 힘이 F를 넘지 않음.
        */
        const float deltaImpulse = constraint.axialMass * ( constraint.motorSpeed - computeAxialVelocity() );
        const float oldImpulse = constraint.motorImpulse;
        constraint.motorImpulse = std::clamp( oldImpulse + deltaImpulse, -constraint.maxMotorImpulse, constraint.maxMotorImpulse );
        applyAxialImpulse( constraint.motorImpulse - oldImpulse );
    }

    // Box2D처럼 모터 다음에 limit을 풀어 모터가 만든 속도도 거리 경계에서 제한함.
    if( constraint.enableLimit )
    {
        auto solveLimit = [&]( float separation, float direction, float& accumulatedImpulse )
        {
            float bias = 0.0f;
            float massScale = 1.0f;
            float impulseScale = 0.0f;

            // 범위 안에서는 남은 간격 / 시간 간격으로 다음 적분에서 경계를 넘지 않도록 제한함.
            // 이미 범위를 벗어났으면 보정 pass에서 위치 오차를 줄이고, 완화 pass에서는 속도만 제한함.
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

            // 축 방향 속도와 위치 오차를 줄이는 임펄스 크기.
            const float deltaImpulse = -massScale * constraint.axialMass * ( direction * computeAxialVelocity() + bias ) - impulseScale * accumulatedImpulse;
            // 거리 제한은 한 방향으로만 작용함. 누적 임펄스를 0 이상으로 제한함.
            const float oldImpulse = accumulatedImpulse;
            accumulatedImpulse = std::max( 0.0f, oldImpulse + deltaImpulse );
            applyAxialImpulse( direction * ( accumulatedImpulse - oldImpulse ) );
        };

        // 하한은 축 방향으로 밀고 상한은 반대 방향으로 당김. 하한을 적용한 속도로 상한을 다시 계산함.
        solveLimit( currentLength - constraint.minLength, 1.0f, constraint.lowerImpulse );
        solveLimit( constraint.maxLength - currentLength, -1.0f, constraint.upperImpulse );
    }
}
#pragma endregion Solve
} // namespace zonai
