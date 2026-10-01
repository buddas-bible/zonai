#include "dynamics/contactConstraint2.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <numbers>

namespace zonai
{

contactSoftness2 MakeContactSoftness( float hertz, float dampingRatio, float timeStep )
{
    assert( std::isfinite( hertz ) );
    assert( std::isfinite( dampingRatio ) );
    assert( std::isfinite( timeStep ) );
    assert( hertz >= 0.0f );
    assert( dampingRatio >= 0.0f );
    assert( timeStep >= 0.0f );

    // Hertz가 0이면 penetration bias만 끄고
    // 기존 rigid normal solve 동작은 그대로 유지함.
    if( hertz == 0.0f || timeStep == 0.0f )
    {
        return {};
    }

    const float omega = 2.0f * std::numbers::pi_v<float> * hertz;

    const float a1 = 2.0f * dampingRatio + timeStep * omega;

    const float a2 = timeStep * omega * a1;

    const float a3 = 1.0f / ( 1.0f + a2 );

    contactSoftness2 softness{};
    softness.biasRate = omega / a1;
    softness.massScale = a2 * a3;
    softness.impulseScale = a3;

    return softness;
}

contactConstraint2 PrepareContactConstraint(
    const contactSim2& contactSim,
    const bodySim& bodySimA, const bodyState& bodyStateA,
    const bodySim& bodySimB, const bodyState& bodyStateB )
{
    assert( contactSim.contactId != contactSim2::NULL_INDEX );
    assert( contactSim.bodyIdA == bodySimA.bodyId );
    assert( contactSim.bodyIdB == bodySimB.bodyId );
    assert( contactSim.manifold.pointCount > 0 );
    assert( contactSim.manifold.pointCount <= static_cast<int>( MAX_MANIFOLD_POINTS ) );

    contactConstraint2 constraint{};

    constraint.contactId = contactSim.contactId;
    constraint.bodyIdA = contactSim.bodyIdA;
    constraint.bodyIdB = contactSim.bodyIdB;

    constraint.invMassA = contactSim.invMassA;
    constraint.invInertiaA = contactSim.invInertiaA;

    constraint.invMassB = contactSim.invMassB;
    constraint.invInertiaB = contactSim.invInertiaB;

    /*
    * local manifold은 shape A local space 기준이므로
    * solver가 사용할 normal과 contact point를 world space로 변환함.
    */
    constraint.normal = TransformVector( bodySimA.transform, contactSim.manifold.normal );

    constraint.pointCount = contactSim.manifold.pointCount;

    for( int i = 0; i < constraint.pointCount; ++i )
    {
        const localManifoldPoint2& manifoldPoint = contactSim.manifold.points[i];

        contactConstraintPoint2& point = constraint.points[i];

        const vec2 worldPoint = TransformPoint( bodySimA.transform, manifoldPoint.point );

        // center of mass에서 contact point까지의 lever arm.
        //
        //     rA = P - cA
        //     rB = P - cB
        point.anchorA = worldPoint - bodySimA.center;
        point.anchorB = worldPoint - bodySimB.center;

        point.baseSeparation =
            manifoldPoint.separation - Dot( point.anchorB - point.anchorA, constraint.normal );

        // 같은 contact point가 이전 step에도 존재했다면
        // 이전 normal / tangent 누적 impulse를 초기 추정값으로 가져옴.
        point.normalImpulse = contactSim.impulses[i].normalImpulse;

        point.tangentImpulse = contactSim.impulses[i].tangentImpulse;

        /*
        * Contact point의 실제 선속도
        *
        * body의 COM 선속도 v에 회전에 의한 접선속도 w x r을 더함.
        *
        *     vPointA = vA + wA x rA
        *     vPointB = vB + wB x rB
        *
        * normal 방향 상대속도:
        *
        *     vn = dot(
        *         vPointB - vPointA,
        *         normal
        *     )
        *
        * vn < 0 : 서로 접근 중
        * vn > 0 : 서로 멀어지는 중
        */
		const vec2& v_a = bodyStateA.linearVelocity;
		const float& w_a = bodyStateA.angularVelocity;

		const vec2& v_b = bodyStateB.linearVelocity;
		const float& w_b = bodyStateB.angularVelocity;

        const vec2& r_a = point.anchorA;
        const vec2& r_b = point.anchorB;

        const vec2 v_p1 = v_a + Cross( w_a, r_a );
        const vec2 v_p2 = v_b + Cross( w_b, r_b );
		const vec2 v_r = v_p2 - v_p1;

        point.relativeNormalVelocity = Dot( v_r, constraint.normal );

        /*
        * normal impulse Jn을 contact point에 적용하면
        * 선운동과 회전운동이 모두 normal 방향 상대속도에 영향을 줌.
        *
        *     rnA = rA x n
        *     rnB = rB x n
        *
        * normal 방향 inverse effective mass:
        *
        *     K =
        *         invMassA +
        *         invMassB +
        *         invInertiaA * rnA^2 +
        *         invInertiaB * rnB^2
        *
        * Solver는
        *
        *     DeltaVn = K * Jn
        *
        * 관계를 뒤집어 impulse를 구하므로:
        *
        *     normalMass = 1 / K
        */
        const float rnA = Cross( r_a, constraint.normal );
        const float rnB = Cross( r_b, constraint.normal );

		// 임펄스 공식의 분모에 해당하는 effective mass의 역수.
        const float inverseEffectiveMass =
            constraint.invMassA +
            constraint.invMassB +
            constraint.invInertiaA * rnA * rnA +
            constraint.invInertiaB * rnB * rnB;

        point.normalMass = inverseEffectiveMass > 0.0f ?
            1.0f / inverseEffectiveMass : 0.0f;

        /*
        * tangent 방향도 같은 방식으로 effective mass를 계산함.
        *
        * normal n=(nx, ny)에 대한 right perpendicular:
        *
        *     tangent = (ny, -nx)
        *
        * friction impulse는 이 축의 상대속도를 0에 가깝게 만들되
        * Coulomb 한계 안에서만 누적됨.
        */
        const vec2 tangent
        {
            constraint.normal.y,
            -constraint.normal.x
        };

        const float rtA = Cross( r_a, tangent );
        const float rtB = Cross( r_b, tangent );

        const float tangentInverseEffectiveMass =
            constraint.invMassA +
            constraint.invMassB +
            constraint.invInertiaA * rtA * rtA +
            constraint.invInertiaB * rtB * rtB;

        point.tangentMass = tangentInverseEffectiveMass > 0.0f ? 
            1.0f / tangentInverseEffectiveMass : 0.0f;
    }

    return constraint;
}


void WarmStartContactConstraint( const contactConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB )
{
    vec2 linearVelocityA = bodyStateA.linearVelocity;
    float angularVelocityA = bodyStateA.angularVelocity;

    vec2 linearVelocityB = bodyStateB.linearVelocity;
    float angularVelocityB = bodyStateB.angularVelocity;

    const vec2 tangent
    {
        constraint.normal.y,
        -constraint.normal.x
    };

    /*
    * 이전 step에서 수렴한 normal / tangent 누적 impulse를
    * 초기값으로 먼저 적용함.
    *
    * 이후 iterative solve는 0부터 다시 계산하지 않고
    * 이 impulse에서 필요한 DeltaLambda만 추가 / 제거함.
    */
    for( int i = 0; i < constraint.pointCount; ++i )
    {
        const contactConstraintPoint2& point = constraint.points[i];

        const vec2 impulseVector =
            constraint.normal * point.normalImpulse +
            tangent * point.tangentImpulse;

        linearVelocityA -= impulseVector * constraint.invMassA;

        angularVelocityA -=
            constraint.invInertiaA *
            Cross( point.anchorA, impulseVector );

        linearVelocityB += impulseVector * constraint.invMassB;

        angularVelocityB +=
            constraint.invInertiaB *
            Cross( point.anchorB, impulseVector );
    }

    bodyStateA.linearVelocity = linearVelocityA;
    bodyStateA.angularVelocity = angularVelocityA;

    bodyStateB.linearVelocity = linearVelocityB;
    bodyStateB.angularVelocity = angularVelocityB;
}

void SolveContactConstraint( contactConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB, bool useBias )
{
    vec2 v_a = bodyStateA.linearVelocity;
    float w_a = bodyStateA.angularVelocity;

    vec2 v_b = bodyStateB.linearVelocity;
    float w_b = bodyStateB.angularVelocity;

    const vec2 dp = bodyStateB.deltaPosition - bodyStateA.deltaPosition;
    const rot2& dqA = bodyStateA.deltaRotation;
    const rot2& dqB = bodyStateB.deltaRotation;

    for( int i = 0; i < constraint.pointCount; ++i )
    {
        contactConstraintPoint2& point = constraint.points[i];

        /*
        * 현재 contact point의 normal 상대속도를 매 iteration마다 다시 계산함.
        *
        * 앞의 Contact가 velocity를 바꾸면 뒤 Contact가 보는 상대속도도 바뀌므로
        * Prepare 단계에서 계산한 값만 계속 사용하면 sequential impulse가 되지 않음.
        */
        // 질량 중심에서 충돌 지점까지의 벡터
        const vec2 v_p1 = v_a + Cross( w_a, point.anchorA );
        const vec2 v_p2 = v_b + Cross( w_b, point.anchorB );
		const vec2 v_r = v_p2 - v_p1;

        // 충돌 지점에서 노말 방향으로의 상대 속도
        const float normalVelocity = Dot( v_r, constraint.normal );

        // Position integration에서 누적된 delta transform으로 현재 separation을 다시 계산함.
        const vec2 ds =
            dp + Rotate( dqB, point.anchorB ) - Rotate( dqA, point.anchorA );

        const float separation = point.baseSeparation + Dot( ds, constraint.normal );

        /*
        * penetration correction은 실제 위치를 바로 순간이동시키지 않고
        * 이번 position integration에서 사용할 분리 속도를 만들어냄.
        *
        * separation < 0이면:
        *
        *     velocityBias =
        *         massScale * biasRate * separation
        *
        * separation이 음수이므로 velocityBias도 음수이고,
        * solver는 이를 상쇄하기 위해 +normal impulse를 생성함.
        *
        * soft constraint:
        *
        *     DeltaLambda =
        *         -normalMass *
        *          ( massScale * vn + velocityBias )
        *         -impulseScale * lambdaOld
        *
        * useBias=false인 relax 단계에서는
        * massScale=1, impulseScale=0, velocityBias=0으로 돌아가
        * 기존 rigid velocity constraint와 같은 식이 됨.
        */
        float massScale = 1.0f;
        float impulseScale = 0.0f;
        float velocityBias = 0.0f;

        if( separation > 0.0f )
        {
            /*
            * speculative contact
            *
            * 아직 떨어져 있는 거리 s를 이번 step dt보다 빠르게 닫지 못하게 함.
            *
            *     vn + s / dt >= 0
            */
            velocityBias = separation * constraint.invTimeStep;
        }
        else if( useBias )
        {
            massScale = constraint.softness.massScale;
            impulseScale = constraint.softness.impulseScale;
            velocityBias = massScale * constraint.softness.biasRate * separation;

            // 깊은 관통에서도 지나치게 큰 보정 속도를 만들지 않음.
            velocityBias = std::max( velocityBias, -constraint.maxPushSpeed );
        }

        const float oldImpulse = point.normalImpulse;

        // useBias == false라면
        // 
		// DeltaLambda = -normalMass * vn
        const float incrementalImpulse =
            -point.normalMass * ( massScale * normalVelocity + velocityBias ) - impulseScale * oldImpulse;

        const float candidateImpulse = oldImpulse + incrementalImpulse;

        point.normalImpulse = std::max( candidateImpulse, 0.0f );

        const float impulse = point.normalImpulse - oldImpulse;

        const vec2 impulseVector = constraint.normal * impulse;

        /*
        * A에는 -P, B에는 +P를 적용함.
        *
        * 선속도:
        *
        *     v_A -= invMassA * P
        *     v_B += invMassB * P
        *
        * 각속도:
        *
        *     w_A -= invIA * ( rA x P )
        *     w_B += invIB * ( rB x P )
        */
        v_a -= impulseVector * constraint.invMassA;
        w_a -= constraint.invInertiaA * Cross( point.anchorA, impulseVector );

        v_b += impulseVector * constraint.invMassB;
        w_b += constraint.invInertiaB * Cross( point.anchorB, impulseVector );
    }

    /*
    * 마찰 Friction
    *
    * penetration push 단계에서는 normal correction만 만들고,
    * position 적분 뒤의 relax 단계에서 실제 tangential velocity를 줄임.
    *
    *     DeltaLambdaT = -tangentMass * vt
    *
    * Coulomb friction은 normal impulse가 만들어낼 수 있는 범위 안에서만
    * tangent impulse를 허용함.
    *
    *     |lambdaT| <= mu * lambdaN
    */
    if( !useBias && constraint.friction > 0.0f )
    {
        const vec2 tangent
        {
            constraint.normal.y,
            -constraint.normal.x
        };

        for( int i = 0; i < constraint.pointCount; ++i )
        {
            contactConstraintPoint2& point = constraint.points[i];

            const vec2 v_p1 = v_a + Cross( w_a, point.anchorA );
            const vec2 v_p2 = v_b + Cross( w_b, point.anchorB );
			const vec2 v_r = v_p2 - v_p1;

            const float tangentVelocity = Dot( v_r, tangent );

            const float oldImpulse = point.tangentImpulse;

            const float incrementalImpulse = -point.tangentMass * tangentVelocity;

            const float maxFrictionImpulse = constraint.friction * point.normalImpulse;

            const float candidateImpulse = oldImpulse + incrementalImpulse;

            point.tangentImpulse = std::clamp( candidateImpulse, -maxFrictionImpulse, maxFrictionImpulse );

            const float impulse = point.tangentImpulse - oldImpulse;

            const vec2 impulseVector = tangent * impulse;

            v_a -= impulseVector * constraint.invMassA;
            w_a -= constraint.invInertiaA * Cross( point.anchorA, impulseVector );

            v_b += impulseVector * constraint.invMassB;
            w_b += constraint.invInertiaB * Cross( point.anchorB, impulseVector );
        }
    }

    bodyStateA.linearVelocity = v_a;
    bodyStateA.angularVelocity = w_a;

    bodyStateB.linearVelocity = v_b;
    bodyStateB.angularVelocity = w_b;
}


void ApplyRestitutionContactConstraint(
    contactConstraint2& constraint,
    bodyState& bodyStateA,
    bodyState& bodyStateB,
    float threshold )
{
    assert( std::isfinite( threshold ) );
    assert( threshold >= 0.0f );

    if( constraint.restitution <= 0.0f )
    {
        return;
    }

    vec2 linearVelocityA = bodyStateA.linearVelocity;
    float angularVelocityA = bodyStateA.angularVelocity;

    vec2 linearVelocityB = bodyStateB.linearVelocity;
    float angularVelocityB = bodyStateB.angularVelocity;

    for( int i = 0; i < constraint.pointCount; ++i )
    {
        contactConstraintPoint2& point = constraint.points[i];

        /*
        * restitution은 현재 vn이 아니라 Prepare 때 저장한
        * 충돌 전 relativeNormalVelocity를 기준으로 결정함.
        *
        * 예를 들어:
        *
        *     relativeNormalVelocity = -4
        *     restitution = 0.5
        *
        * 이면 normal solve가 접근 속도를 0까지 막은 뒤
        * restitution은 최종 목표를 +2로 만듦.
        *
        * 낮은 속도 접촉까지 튀기면 resting contact가 떨릴 수 있으므로
        * threshold보다 빠른 충돌에만 적용함.
        */
        if( point.relativeNormalVelocity >= -threshold ||
            point.normalImpulse <= 0.0f )
        {
            continue;
        }

        const vec2 velocityA = linearVelocityA + Cross( angularVelocityA, point.anchorA );
        const vec2 velocityB = linearVelocityB + Cross( angularVelocityB, point.anchorB );

        const float normalVelocity = Dot( velocityB - velocityA, constraint.normal );

        // targetVn = -restitution * relativeNormalVelocity
        //
        // 따라서:
        //
        //     DeltaLambda
        //         = normalMass * ( targetVn - vn )
        //         = -normalMass *
        //           ( vn + restitution * relativeNormalVelocity )
        const float oldImpulse = point.normalImpulse;

        const float incrementalImpulse =
            -point.normalMass * ( normalVelocity + constraint.restitution * point.relativeNormalVelocity );

        point.normalImpulse = std::max( oldImpulse + incrementalImpulse, 0.0f );

        const float impulse = point.normalImpulse - oldImpulse;

        const vec2 impulseVector = constraint.normal * impulse;

        linearVelocityA -= impulseVector * constraint.invMassA;
        angularVelocityA -= constraint.invInertiaA * Cross( point.anchorA, impulseVector );

        linearVelocityB += impulseVector * constraint.invMassB;
        angularVelocityB += constraint.invInertiaB * Cross( point.anchorB, impulseVector );
    }

    bodyStateA.linearVelocity = linearVelocityA;
    bodyStateA.angularVelocity = angularVelocityA;

    bodyStateB.linearVelocity = linearVelocityB;
    bodyStateB.angularVelocity = angularVelocityB;
}


void StoreContactImpulses( const contactConstraint2& constraint, contactSim2& contactSim )
{
    assert( constraint.contactId != contactSim2::NULL_INDEX );
    assert( constraint.contactId == contactSim.contactId );
    assert( constraint.pointCount == contactSim.manifold.pointCount );

    for( int i = 0; i < constraint.pointCount; ++i )
    {
        contactSim.impulses[i].normalImpulse = constraint.points[i].normalImpulse;
        contactSim.impulses[i].tangentImpulse = constraint.points[i].tangentImpulse;
    }
}


} // namespace zonai
