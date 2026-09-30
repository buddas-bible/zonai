#include "dynamics/contactConstraint2.h"

#include <algorithm>
#include <cassert>

namespace zonai
{

contactConstraint2 PrepareContactConstraint(
    const contactSim2& contactSim,
    const BodySim& bodySimA,
    const BodyState& bodyStateA,
    const BodySim& bodySimB,
    const BodyState& bodyStateB )
{
    assert( contactSim.contactId != contactSim2::NULL_INDEX );
    assert( contactSim.bodyIdA == bodySimA.bodyId );
    assert( contactSim.bodyIdB == bodySimB.bodyId );
    assert( contactSim.manifold.pointCount > 0 );
    assert(
        contactSim.manifold.pointCount <=
        static_cast<int>( MAX_MANIFOLD_POINTS )
    );

    contactConstraint2 constraint{};

    constraint.bodyIdA = contactSim.bodyIdA;
    constraint.bodyIdB = contactSim.bodyIdB;

    constraint.invMassA = contactSim.invMassA;
    constraint.invInertiaA = contactSim.invInertiaA;

    constraint.invMassB = contactSim.invMassB;
    constraint.invInertiaB = contactSim.invInertiaB;

    /*
    * local manifold은 Shape A local space 기준이므로
    * solver가 사용할 normal과 contact point를 world space로 변환함.
    */
    constraint.normal =
        TransformVector(
            bodySimA.transform,
            contactSim.manifold.normal
        );

    constraint.pointCount =
        contactSim.manifold.pointCount;

    for( int i = 0; i < constraint.pointCount; ++i )
    {
        const localManifoldPoint2& manifoldPoint =
            contactSim.manifold.points[i];

        contactConstraintPoint2& point =
            constraint.points[i];

        const vec2 worldPoint =
            TransformPoint(
                bodySimA.transform,
                manifoldPoint.point
            );

        // center of mass에서 contact point까지의 lever arm.
        //
        //     rA = P - cA
        //     rB = P - cB
        point.anchorA =
            worldPoint - bodySimA.center;

        point.anchorB =
            worldPoint - bodySimB.center;

        point.separation =
            manifoldPoint.separation;

        /*
        * Contact point의 실제 선속도
        *
        * Body의 COM 선속도 v에 회전에 의한 접선속도 w x r을 더함.
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
        const vec2 velocityA =
            bodyStateA.linearVelocity +
            Cross(
                bodyStateA.angularVelocity,
                point.anchorA
            );

        const vec2 velocityB =
            bodyStateB.linearVelocity +
            Cross(
                bodyStateB.angularVelocity,
                point.anchorB
            );

        point.relativeNormalVelocity =
            Dot(
                velocityB - velocityA,
                constraint.normal
            );

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
        const float rnA =
            Cross(
                point.anchorA,
                constraint.normal
            );

        const float rnB =
            Cross(
                point.anchorB,
                constraint.normal
            );

        const float inverseEffectiveMass =
            constraint.invMassA +
            constraint.invMassB +
            constraint.invInertiaA * rnA * rnA +
            constraint.invInertiaB * rnB * rnB;

        point.normalMass =
            inverseEffectiveMass > 0.0f
                ? 1.0f / inverseEffectiveMass
                : 0.0f;
    }

    return constraint;
}

void SolveContactConstraint(
    contactConstraint2& constraint,
    BodyState& bodyStateA,
    BodyState& bodyStateB )
{
    vec2 linearVelocityA =
        bodyStateA.linearVelocity;

    float angularVelocityA =
        bodyStateA.angularVelocity;

    vec2 linearVelocityB =
        bodyStateB.linearVelocity;

    float angularVelocityB =
        bodyStateB.angularVelocity;

    for( int i = 0; i < constraint.pointCount; ++i )
    {
        contactConstraintPoint2& point =
            constraint.points[i];

        /*
        * 현재 contact point의 normal 상대속도를 매 iteration마다 다시 계산함.
        *
        * 앞의 Contact가 velocity를 바꾸면 뒤 Contact가 보는 상대속도도 바뀌므로
        * Prepare 단계에서 계산한 값만 계속 사용하면 sequential impulse가 되지 않음.
        */
        const vec2 velocityA =
            linearVelocityA +
            Cross(
                angularVelocityA,
                point.anchorA
            );

        const vec2 velocityB =
            linearVelocityB +
            Cross(
                angularVelocityB,
                point.anchorB
            );

        const float normalVelocity =
            Dot(
                velocityB - velocityA,
                constraint.normal
            );

        /*
        * normal 방향 상대속도를 0으로 만들기 위한 incremental impulse:
        *
        *     DeltaLambda = -normalMass * vn
        *
        * 하지만 Contact는 두 Body를 서로 당길 수 없음.
        * 따라서 누적 impulse lambda는 항상 0 이상이어야 함.
        *
        *     lambdaNew = max( lambdaOld + DeltaLambda, 0 )
        *     DeltaLambda = lambdaNew - lambdaOld
        *
        * 이미 서로 멀어지고 있으면 vn > 0이므로
        * 음수 impulse가 clamp되어 아무 힘도 가하지 않음.
        */
        const float oldImpulse =
            point.normalImpulse;

        const float candidateImpulse =
            oldImpulse -
            point.normalMass * normalVelocity;

        point.normalImpulse =
            std::max(
                candidateImpulse,
                0.0f
            );

        const float impulse =
            point.normalImpulse -
            oldImpulse;

        const vec2 impulseVector =
            constraint.normal * impulse;

        /*
        * A에는 -P, B에는 +P를 적용함.
        *
        * 선속도:
        *
        *     vA -= invMassA * P
        *     vB += invMassB * P
        *
        * 각속도:
        *
        *     wA -= invIA * ( rA x P )
        *     wB += invIB * ( rB x P )
        */
        linearVelocityA -=
            impulseVector *
            constraint.invMassA;

        angularVelocityA -=
            constraint.invInertiaA *
            Cross(
                point.anchorA,
                impulseVector
            );

        linearVelocityB +=
            impulseVector *
            constraint.invMassB;

        angularVelocityB +=
            constraint.invInertiaB *
            Cross(
                point.anchorB,
                impulseVector
            );
    }

    bodyStateA.linearVelocity =
        linearVelocityA;

    bodyStateA.angularVelocity =
        angularVelocityA;

    bodyStateB.linearVelocity =
        linearVelocityB;

    bodyStateB.angularVelocity =
        angularVelocityB;
}

} // namespace zonai
