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

    // Static A와 Dynamic B가 서로 접근할 때 normal impulse가 B의 접근 속도를 제거함.
    {
        contactConstraint2 constraint{};
        constraint.bodyIdA = 0;
        constraint.bodyIdB = 1;
        constraint.normal = { 1.0f, 0.0f };
        constraint.invMassA = 0.0f;
        constraint.invInertiaA = 0.0f;
        constraint.invMassB = 0.5f;
        constraint.invInertiaB = 0.0f;
        constraint.pointCount = 1;

        contactConstraintPoint2& point =
            constraint.points[0];

        point.normalMass = 2.0f;

        BodyState bodyStateA{};

        BodyState bodyStateB{};
        bodyStateB.linearVelocity = { -3.0f, 0.0f };

        SolveContactConstraint(
            constraint,
            bodyStateA,
            bodyStateB,
            false
        );

        // DeltaLambda = -normalMass * vn
        //             = -2 * (-3) = 6
        //
        // DeltaV_B = invMassB * 6 = 3
        //
        // 따라서 -3 + 3 = 0.
        assert(
            NearlyEqual(
                bodyStateB.linearVelocity.x,
                0.0f
            )
        );
        assert(
            NearlyEqual(
                point.normalImpulse,
                6.0f
            )
        );
    }

    // 이미 분리 중이면 Contact가 음수 impulse로 서로 끌어당기면 안 됨.
    {
        contactConstraint2 constraint{};
        constraint.bodyIdA = 0;
        constraint.bodyIdB = 1;
        constraint.normal = { 1.0f, 0.0f };
        constraint.invMassA = 0.0f;
        constraint.invMassB = 0.5f;
        constraint.pointCount = 1;
        constraint.points[0].normalMass = 2.0f;

        BodyState bodyStateA{};

        BodyState bodyStateB{};
        bodyStateB.linearVelocity = { 3.0f, 0.0f };

        SolveContactConstraint(
            constraint,
            bodyStateA,
            bodyStateB,
            false
        );

        assert(
            NearlyEqual(
                bodyStateB.linearVelocity.x,
                3.0f
            )
        );
        assert(
            constraint.points[0].normalImpulse == 0.0f
        );
    }


    // Prepare 단계에서 ContactSim에 캐싱된 normal impulse를 constraint 초기값으로 가져옴.
    {
        contactSim2 contactSim{};
        contactSim.contactId = 7;
        contactSim.bodyIdA = 0;
        contactSim.bodyIdB = 1;
        contactSim.invMassA = 0.0f;
        contactSim.invMassB = 0.5f;
        contactSim.manifold.normal = { 1.0f, 0.0f };
        contactSim.manifold.pointCount = 1;
        contactSim.manifold.points[0].point = {};
        contactSim.impulses[0].normalImpulse = 6.0f;
        contactSim.impulses[0].tangentImpulse = -1.5f;

        BodySim bodySimA{};
        bodySimA.bodyId = 0;

        BodySim bodySimB{};
        bodySimB.bodyId = 1;

        BodyState bodyStateA{};
        BodyState bodyStateB{};

        const contactConstraint2 constraint =
            PrepareContactConstraint(
                contactSim,
                bodySimA,
                bodyStateA,
                bodySimB,
                bodyStateB
            );

        assert( constraint.contactId == 7 );
        assert(
            NearlyEqual(
                constraint.points[0].normalImpulse,
                6.0f
            )
        );
        assert(
            NearlyEqual(
                constraint.points[0].tangentImpulse,
                -1.5f
            )
        );
    }

    // Warm start는 이전 step의 누적 impulse 전체를 solver 시작 전에 한 번 적용함.
    {
        contactConstraint2 constraint{};
        constraint.contactId = 0;
        constraint.bodyIdA = 0;
        constraint.bodyIdB = 1;
        constraint.normal = { 1.0f, 0.0f };
        constraint.invMassA = 0.0f;
        constraint.invInertiaA = 0.0f;
        constraint.invMassB = 0.5f;
        constraint.invInertiaB = 0.0f;
        constraint.pointCount = 1;

        contactConstraintPoint2& point =
            constraint.points[0];

        point.normalMass = 2.0f;
        point.normalImpulse = 6.0f;

        BodyState bodyStateA{};

        BodyState bodyStateB{};
        bodyStateB.linearVelocity = { -3.0f, 0.0f };

        WarmStartContactConstraint(
            constraint,
            bodyStateA,
            bodyStateB
        );

        // 이전 impulse 6을 먼저 적용하면:
        //
        // DeltaV_B = invMassB * 6
        //          = 0.5 * 6
        //          = 3
        //
        // 따라서 -3 + 3 = 0.
        assert(
            NearlyEqual(
                bodyStateB.linearVelocity.x,
                0.0f
            )
        );

        // 이미 warm start 결과로 vn == 0이므로
        // 다음 solve에서는 추가 impulse가 없어야 함.
        SolveContactConstraint(
            constraint,
            bodyStateA,
            bodyStateB,
            false
        );

        assert(
            NearlyEqual(
                point.normalImpulse,
                6.0f
            )
        );
    }

    // Hertz / damping ratio로 만든 softness는 massScale + impulseScale = 1을 유지함.
    {
        const contactSoftness2 softness =
            MakeContactSoftness(
                7.5f,
                10.0f,
                1.0f / 60.0f
            );

        assert( softness.biasRate > 0.0f );
        assert( softness.massScale > 0.0f );
        assert( softness.massScale < 1.0f );
        assert( softness.impulseScale > 0.0f );
        assert( softness.impulseScale < 1.0f );
        assert(
            NearlyEqual(
                softness.massScale + softness.impulseScale,
                1.0f
            )
        );
    }

    // Push는 penetration을 줄일 분리 속도를 만들고,
    // position 적분 뒤의 Relax는 그 보정 속도를 다시 제거할 수 있음.
    {
        contactConstraint2 constraint{};
        constraint.bodyIdA = 0;
        constraint.bodyIdB = 1;
        constraint.normal = { 1.0f, 0.0f };
        constraint.invMassA = 0.0f;
        constraint.invMassB = 1.0f;
        constraint.softness.biasRate = 2.0f;
        constraint.softness.massScale = 0.5f;
        constraint.softness.impulseScale = 0.5f;
        constraint.maxPushSpeed = 3.0f;
        constraint.pointCount = 1;

        contactConstraintPoint2& point =
            constraint.points[0];

        point.separation = -0.1f;
        point.normalMass = 1.0f;

        BodyState bodyStateA{};
        BodyState bodyStateB{};

        SolveContactConstraint(
            constraint,
            bodyStateA,
            bodyStateB,
            true
        );

        // velocityBias = 0.5 * 2 * -0.1 = -0.1
        // DeltaLambda = -(0 + -0.1) = 0.1
        assert(
            NearlyEqual(
                bodyStateB.linearVelocity.x,
                0.1f
            )
        );
        assert(
            NearlyEqual(
                point.normalImpulse,
                0.1f
            )
        );

        // 실제 World::Step에서는 여기 사이에 position integration이 일어남.
        // Relax는 correction velocity만 제거하고 이미 이동한 position은 보존함.
        SolveContactConstraint(
            constraint,
            bodyStateA,
            bodyStateB,
            false
        );

        assert(
            NearlyEqual(
                bodyStateB.linearVelocity.x,
                0.0f
            )
        );
        assert(
            NearlyEqual(
                point.normalImpulse,
                0.0f
            )
        );
    }

    // tangent effective mass도 lever arm과 inverse mass / inertia를 포함해 계산함.
    {
        contactSim2 contactSim{};
        contactSim.contactId = 11;
        contactSim.bodyIdA = 0;
        contactSim.bodyIdB = 1;
        contactSim.invMassA = 0.0f;
        contactSim.invMassB = 0.5f;
        contactSim.manifold.normal = { 0.0f, 1.0f };
        contactSim.manifold.pointCount = 1;
        contactSim.manifold.points[0].point = {};

        BodySim bodySimA{};
        bodySimA.bodyId = 0;

        BodySim bodySimB{};
        bodySimB.bodyId = 1;

        BodyState bodyStateA{};
        BodyState bodyStateB{};

        const contactConstraint2 constraint =
            PrepareContactConstraint(
                contactSim,
                bodySimA,
                bodyStateA,
                bodySimB,
                bodyStateB
            );

        // tangent=(1,0), lever arm=0이므로
        // Kt=invMassB=0.5 -> tangentMass=2.
        assert(
            NearlyEqual(
                constraint.points[0].tangentMass,
                2.0f
            )
        );
    }

    // Warm start는 normal impulse뿐 아니라 이전 tangent impulse도 함께 적용함.
    {
        contactConstraint2 constraint{};
        constraint.bodyIdA = 0;
        constraint.bodyIdB = 1;
        constraint.normal = { 0.0f, 1.0f };
        constraint.invMassA = 0.0f;
        constraint.invMassB = 1.0f;
        constraint.pointCount = 1;
        constraint.points[0].normalImpulse = 2.0f;
        constraint.points[0].tangentImpulse = -0.5f;

        BodyState bodyStateA{};
        BodyState bodyStateB{};

        WarmStartContactConstraint(
            constraint,
            bodyStateA,
            bodyStateB
        );

        // normal=(0,1), tangent=(1,0)
        assert( NearlyEqual( bodyStateB.linearVelocity.x, -0.5f ) );
        assert( NearlyEqual( bodyStateB.linearVelocity.y, 2.0f ) );
    }

    // Coulomb friction은 tangent 속도를 줄이되 mu * normalImpulse를 넘지 않음.
    {
        contactConstraint2 constraint{};
        constraint.bodyIdA = 0;
        constraint.bodyIdB = 1;
        constraint.normal = { 0.0f, 1.0f };
        constraint.invMassA = 0.0f;
        constraint.invMassB = 1.0f;
        constraint.friction = 0.5f;
        constraint.pointCount = 1;

        contactConstraintPoint2& point =
            constraint.points[0];

        point.normalMass = 1.0f;
        point.tangentMass = 1.0f;
        point.normalImpulse = 2.0f;

        BodyState bodyStateA{};
        BodyState bodyStateB{};
        bodyStateB.linearVelocity = { 4.0f, 0.0f };

        SolveContactConstraint(
            constraint,
            bodyStateA,
            bodyStateB,
            false
        );

        // 필요한 friction impulse는 -4지만
        // max = mu * normalImpulse = 0.5 * 2 = 1.
        assert( NearlyEqual( point.tangentImpulse, -1.0f ) );
        assert( NearlyEqual( bodyStateB.linearVelocity.x, 3.0f ) );
    }

    // normal impulse가 없으면 Coulomb friction도 물체를 임의로 멈출 수 없음.
    {
        contactConstraint2 constraint{};
        constraint.bodyIdA = 0;
        constraint.bodyIdB = 1;
        constraint.normal = { 0.0f, 1.0f };
        constraint.invMassA = 0.0f;
        constraint.invMassB = 1.0f;
        constraint.friction = 0.6f;
        constraint.pointCount = 1;
        constraint.points[0].normalMass = 1.0f;
        constraint.points[0].tangentMass = 1.0f;

        BodyState bodyStateA{};
        BodyState bodyStateB{};
        bodyStateB.linearVelocity = { 4.0f, 0.0f };

        SolveContactConstraint(
            constraint,
            bodyStateA,
            bodyStateB,
            false
        );

        assert( NearlyEqual( constraint.points[0].tangentImpulse, 0.0f ) );
        assert( NearlyEqual( bodyStateB.linearVelocity.x, 4.0f ) );
    }

    // Solve가 끝난 누적 impulse를 persistent ContactSim에 다시 저장함.
    {
        contactConstraint2 constraint{};
        constraint.contactId = 3;
        constraint.pointCount = 1;
        constraint.points[0].normalImpulse = 4.25f;
        constraint.points[0].tangentImpulse = -0.75f;

        contactSim2 contactSim{};
        contactSim.contactId = 3;
        contactSim.manifold.pointCount = 1;

        StoreContactImpulses(
            constraint,
            contactSim
        );

        assert(
            NearlyEqual(
                contactSim.impulses[0].normalImpulse,
                4.25f
            )
        );
        assert(
            NearlyEqual(
                contactSim.impulses[0].tangentImpulse,
                -0.75f
            )
        );
    }

    return 0;
}
