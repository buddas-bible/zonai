#include <cassert>
#include <cmath>

#include "dynamics/contactConstraint2.h"

namespace
{

bool NearlyEqual(
    float a,
    float b,
    float epsilon = 1e-5f )
{
    return std::fabs( a - b ) <= epsilon;
}

} // namespace

int main()
{
    using namespace zonai;

    // 회전이 있는 두 Body의 contact point 상대속도와 normal effective mass를 검증함.
    {
        contactSim2 contactSim{};
        contactSim.contactId = 0;
        contactSim.bodyIdA = 0;
        contactSim.bodyIdB = 1;

        contactSim.invMassA = 1.0f;
        contactSim.invInertiaA = 2.0f;

        contactSim.invMassB = 0.5f;
        contactSim.invInertiaB = 0.25f;

        contactSim.manifold.normal = { 1.0f, 0.0f };
        contactSim.manifold.pointCount = 1;
        contactSim.manifold.points[0].point = { 1.0f, 1.0f };
        contactSim.manifold.points[0].separation = -0.1f;

        BodySim bodySimA{};
        bodySimA.bodyId = 0;
        bodySimA.transform = {};
        bodySimA.center = { 0.0f, 0.0f };

        BodySim bodySimB{};
        bodySimB.bodyId = 1;
        bodySimB.transform = {};
        bodySimB.center = { 2.0f, 0.0f };

        BodyState bodyStateA{};
        bodyStateA.linearVelocity = { 1.0f, 0.0f };
        bodyStateA.angularVelocity = 2.0f;

        BodyState bodyStateB{};
        bodyStateB.linearVelocity = { -1.0f, 0.0f };
        bodyStateB.angularVelocity = -1.0f;

        const contactConstraint2 constraint =
            PrepareContactConstraint(
                contactSim,
                bodySimA,
                bodyStateA,
                bodySimB,
                bodyStateB
            );

        assert( constraint.pointCount == 1 );
        assert( NearlyEqual( constraint.normal.x, 1.0f ) );
        assert( NearlyEqual( constraint.normal.y, 0.0f ) );

        const contactConstraintPoint2& point =
            constraint.points[0];

        assert( NearlyEqual( point.anchorA.x, 1.0f ) );
        assert( NearlyEqual( point.anchorA.y, 1.0f ) );
        assert( NearlyEqual( point.anchorB.x, -1.0f ) );
        assert( NearlyEqual( point.anchorB.y, 1.0f ) );
        assert( NearlyEqual( point.separation, -0.1f ) );

        /*
        * A contact point velocity:
        *
        *     vA + wA x rA
        *   = (1,0) + 2 x (1,1)
        *   = (1,0) + (-2,2)
        *   = (-1,2)
        *
        * B:
        *
        *     vB + wB x rB
        *   = (-1,0) + (-1) x (-1,1)
        *   = (-1,0) + (1,1)
        *   = (0,1)
        *
        * normal=(1,0)이므로:
        *
        *     vn = dot( (0,1)-(-1,2), (1,0) )
        *        = 1
        */
        assert(
            NearlyEqual(
                point.relativeNormalVelocity,
                1.0f
            )
        );

        /*
        * rnA = rA x n = -1
        * rnB = rB x n = -1
        *
        * K = 1 + 0.5 + 2*1 + 0.25*1
        *   = 3.75
        *
        * normalMass = 1 / K
        */
        assert(
            NearlyEqual(
                point.normalMass,
                1.0f / 3.75f
            )
        );
    }

    // Body A가 Static처럼 inverse mass / inertia가 0이면
    // Dynamic Body B의 질량만 effective mass에 기여함.
    {
        contactSim2 contactSim{};
        contactSim.contactId = 0;
        contactSim.bodyIdA = 0;
        contactSim.bodyIdB = 1;

        contactSim.invMassA = 0.0f;
        contactSim.invInertiaA = 0.0f;

        contactSim.invMassB = 0.5f;
        contactSim.invInertiaB = 0.0f;

        contactSim.manifold.normal = { 0.0f, 1.0f };
        contactSim.manifold.pointCount = 1;
        contactSim.manifold.points[0].point = {};

        BodySim bodySimA{};
        bodySimA.bodyId = 0;

        BodySim bodySimB{};
        bodySimB.bodyId = 1;

        BodyState bodyStateA{};
        BodyState bodyStateB{};
        bodyStateB.linearVelocity = { 0.0f, -3.0f };

        const contactConstraint2 constraint =
            PrepareContactConstraint(
                contactSim,
                bodySimA,
                bodyStateA,
                bodySimB,
                bodyStateB
            );

        const contactConstraintPoint2& point =
            constraint.points[0];

        // B가 normal 반대 방향으로 움직이므로 서로 접근 중.
        assert(
            NearlyEqual(
                point.relativeNormalVelocity,
                -3.0f
            )
        );

        // K = invMassB = 0.5 -> normalMass = 2.
        assert(
            NearlyEqual(
                point.normalMass,
                2.0f
            )
        );
    }

    return 0;
}
