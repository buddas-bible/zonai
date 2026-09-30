#include "dynamics/contactConstraint2.h"

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

} // namespace zonai
