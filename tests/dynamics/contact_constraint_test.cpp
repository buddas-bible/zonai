#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <source_location>
#include <cmath>

#include "dynamics/contactConstraint2.h"

namespace
{

// Release에서도 solver 검사를 실행하고 실패한 위치를 출력함.
void check( bool condition, const std::source_location& location = std::source_location::current() )
{
    if( !condition )
    {
        std::fprintf( stderr, "%s:%u: solver check failed\n", location.file_name(), location.line() );
        std::exit( EXIT_FAILURE );
    }
}

bool NearlyEqual( float a, float b, float epsilon = 1e-5f )
{
    return std::fabs( a - b ) <= epsilon;
}

} // namespace

int main()
{
    using namespace zonai;

    // 회전이 있는 두 body의 contact point 상대속도와 normal effective mass를 검증함.
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

        bodySim bodySimA{};
        bodySimA.bodyId = 0;
        bodySimA.transform = {};
        bodySimA.center = { 0.0f, 0.0f };

        bodySim bodySimB{};
        bodySimB.bodyId = 1;
        bodySimB.transform = {};
        bodySimB.center = { 2.0f, 0.0f };

        bodyState bodyStateA{};
        bodyStateA.linearVelocity = { 1.0f, 0.0f };
        bodyStateA.angularVelocity = 2.0f;

        bodyState bodyStateB{};
        bodyStateB.linearVelocity = { -1.0f, 0.0f };
        bodyStateB.angularVelocity = -1.0f;

        const contactConstraint2 constraint =
            PrepareContactConstraint(
                contactSim,
                bodySimA, bodyStateA,
                bodySimB, bodyStateB
            );

        check( constraint.pointCount == 1 );
        check( NearlyEqual( constraint.normal.x, 1.0f ) );
        check( NearlyEqual( constraint.normal.y, 0.0f ) );

        const contactConstraintPoint2& point = constraint.points[0];

        check( NearlyEqual( point.anchorA.x, 1.0f ) );
        check( NearlyEqual( point.anchorA.y, 1.0f ) );
        check( NearlyEqual( point.anchorB.x, -1.0f ) );
        check( NearlyEqual( point.anchorB.y, 1.0f ) );
        // baseSeparation은 anchor 차이를 제외한 기준값이고,
        // identity delta transform을 다시 더하면 원래 separation -0.1이 복원됨.
        check( NearlyEqual( point.baseSeparation, 1.9f ) );

        const vec2 ds = point.anchorB - point.anchorA;
        check( NearlyEqual( point.baseSeparation + Dot( ds, constraint.normal ), -0.1f ) );

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
        check( NearlyEqual( point.relativeNormalVelocity, 1.0f ) );

        /*
        * rnA = rA x n = -1
        * rnB = rB x n = -1
        *
        * K = 1 + 0.5 + 2*1 + 0.25*1
        *   = 3.75
        *
        * normalMass = 1 / K
        */
        check( NearlyEqual( point.normalMass, 1.0f / 3.75f ) );
    }

    // body A가 Static처럼 inverse mass / inertia가 0이면
    // Dynamic body B의 질량만 effective mass에 기여함.
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

        bodySim bodySimA{};
        bodySimA.bodyId = 0;

        bodySim bodySimB{};
        bodySimB.bodyId = 1;

        bodyState bodyStateA{};
        bodyState bodyStateB{};
        bodyStateB.linearVelocity = { 0.0f, -3.0f };

        const contactConstraint2 constraint =
            PrepareContactConstraint(
                contactSim,
                bodySimA, bodyStateA,
                bodySimB, bodyStateB
            );

        const contactConstraintPoint2& point = constraint.points[0];

        // B가 normal 반대 방향으로 움직이므로 서로 접근 중.
        check( NearlyEqual( point.relativeNormalVelocity, -3.0f ) );

        // K = invMassB = 0.5 -> normalMass = 2.
        check( NearlyEqual( point.normalMass, 2.0f ) );
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

        contactConstraintPoint2& point = constraint.points[0];

        point.normalMass = 2.0f;

        bodyState bodyStateA{};

        bodyState bodyStateB{};
        bodyStateB.linearVelocity = { -3.0f, 0.0f };

        SolveContactConstraint( constraint, bodyStateA, bodyStateB, false );

        // DeltaLambda = -normalMass * vn
        //             = -2 * (-3) = 6
        //
        // DeltaV_B = invMassB * 6 = 3
        //
        // 따라서 -3 + 3 = 0.
        check( NearlyEqual( bodyStateB.linearVelocity.x, 0.0f ) );
        check( NearlyEqual( point.normalImpulse, 6.0f ) );
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

        bodyState bodyStateA{};

        bodyState bodyStateB{};
        bodyStateB.linearVelocity = { 3.0f, 0.0f };

        SolveContactConstraint( constraint, bodyStateA, bodyStateB, false );

        check( NearlyEqual( bodyStateB.linearVelocity.x, 3.0f ) );
        check( constraint.points[0].normalImpulse == 0.0f );
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

        bodySim bodySimA{};
        bodySimA.bodyId = 0;

        bodySim bodySimB{};
        bodySimB.bodyId = 1;

        bodyState bodyStateA{};
        bodyState bodyStateB{};

        const contactConstraint2 constraint =
            PrepareContactConstraint(
                contactSim,
                bodySimA, bodyStateA,
                bodySimB, bodyStateB
            );

        check( constraint.contactId == 7 );
        check( NearlyEqual( constraint.points[0].normalImpulse, 6.0f ) );
        check( NearlyEqual( constraint.points[0].tangentImpulse, -1.5f ) );
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

        contactConstraintPoint2& point = constraint.points[0];

        point.normalMass = 2.0f;
        point.normalImpulse = 6.0f;

        bodyState bodyStateA{};

        bodyState bodyStateB{};
        bodyStateB.linearVelocity = { -3.0f, 0.0f };

        WarmStartContactConstraint( constraint, bodyStateA, bodyStateB );

        // 이전 impulse 6을 먼저 적용하면:
        //
        // DeltaV_B = invMassB * 6
        //          = 0.5 * 6
        //          = 3
        //
        // 따라서 -3 + 3 = 0.
        check( NearlyEqual( bodyStateB.linearVelocity.x, 0.0f ) );

        // 이미 warm start 결과로 vn == 0이므로
        // 다음 solve에서는 추가 impulse가 없어야 함.
        SolveContactConstraint( constraint, bodyStateA, bodyStateB, false );

        check( NearlyEqual( point.normalImpulse, 6.0f ) );

        // warm start로 실제 적용한 normal impulse도 이번 step의 compression 양으로 추적함.
        check( NearlyEqual( point.totalNormalImpulse, 6.0f ) );
    }

    // Hertz / damping ratio로 만든 softness는 massScale + impulseScale = 1을 유지함.
    {
        const contactSoftness2 softness = MakeContactSoftness( 7.5f, 10.0f, 1.0f / 60.0f );

        check( softness.biasRate > 0.0f );
        check( softness.massScale > 0.0f );
        check( softness.massScale < 1.0f );
        check( softness.impulseScale > 0.0f );
        check( softness.impulseScale < 1.0f );
        check( NearlyEqual( softness.massScale + softness.impulseScale, 1.0f ) );
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

        contactConstraintPoint2& point = constraint.points[0];

        point.baseSeparation = -0.1f;
        point.normalMass = 1.0f;

        bodyState bodyStateA{};
        bodyState bodyStateB{};

        SolveContactConstraint( constraint, bodyStateA, bodyStateB, true );

        // velocityBias = 0.5 * 2 * -0.1 = -0.1
        // DeltaLambda = -(0 + -0.1) = 0.1
        check( NearlyEqual( bodyStateB.linearVelocity.x, 0.1f ) );
        check( NearlyEqual( point.normalImpulse, 0.1f ) );

        // 실제 world::Step에서는 여기 사이에 position integration이 일어남.
        // Relax는 correction velocity만 제거하고 이미 이동한 position은 보존함.
        SolveContactConstraint( constraint, bodyStateA, bodyStateB, false );

        check( NearlyEqual( bodyStateB.linearVelocity.x, 0.0f ) );
        check( NearlyEqual( point.normalImpulse, 0.0f ) );
    }

    // Position integration에서 생긴 deltaPosition을 반영해 현재 separation을 다시 계산함.
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
        constraint.invTimeStep = 60.0f;
        constraint.pointCount = 1;

        contactConstraintPoint2& point = constraint.points[0];
        point.baseSeparation = -0.1f;
        point.normalMass = 1.0f;

        bodyState bodyStateA{};
        bodyState bodyStateB{};

        // Prepare 때는 0.1m 관통했지만 position integration에서 B가 +0.2m 이동해
        // 현재 separation은 +0.1m가 되었으므로 penetration push가 더 생기면 안 됨.
        bodyStateB.deltaPosition = { 0.2f, 0.0f };

        SolveContactConstraint( constraint, bodyStateA, bodyStateB, true );

        check( NearlyEqual( bodyStateB.linearVelocity.x, 0.0f ) );
        check( NearlyEqual( point.normalImpulse, 0.0f ) );
    }

    // deltaRotation도 contact anchor를 회전시켜 현재 separation에 반영됨.
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

        contactConstraintPoint2& point = constraint.points[0];
        point.anchorB = { 0.0f, 1.0f };
        point.baseSeparation = 0.0f;
        point.normalMass = 1.0f;

        bodyState bodyStateA{};
        bodyState bodyStateB{};
        bodyStateB.deltaRotation = rot2::FromRadians( 1.57079632679f );

        SolveContactConstraint( constraint, bodyStateA, bodyStateB, true );

        // anchorB=(0,1)이 90도 회전하면 (-1,0)이 되어 separation=-1.
        // velocityBias=0.5*2*-1=-1이므로 +normal impulse 1이 만들어짐.
        check( NearlyEqual( bodyStateB.linearVelocity.x, 1.0f ) );
        check( NearlyEqual( point.normalImpulse, 1.0f ) );
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

        bodySim bodySimA{};
        bodySimA.bodyId = 0;

        bodySim bodySimB{};
        bodySimB.bodyId = 1;

        bodyState bodyStateA{};
        bodyState bodyStateB{};

        const contactConstraint2 constraint =
            PrepareContactConstraint(
                contactSim,
                bodySimA, bodyStateA,
                bodySimB, bodyStateB
            );

        // tangent=(1,0), lever arm=0이므로
        // Kt=invMassB=0.5 -> tangentMass=2.
        check( NearlyEqual( constraint.points[0].tangentMass, 2.0f ) );
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

        bodyState bodyStateA{};
        bodyState bodyStateB{};

        WarmStartContactConstraint( constraint, bodyStateA, bodyStateB );

        // normal=(0,1), tangent=(1,0)
        check( NearlyEqual( bodyStateB.linearVelocity.x, -0.5f ) );
        check( NearlyEqual( bodyStateB.linearVelocity.y, 2.0f ) );
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

        contactConstraintPoint2& point = constraint.points[0];

        point.normalMass = 1.0f;
        point.tangentMass = 1.0f;
        point.normalImpulse = 2.0f;

        bodyState bodyStateA{};
        bodyState bodyStateB{};
        bodyStateB.linearVelocity = { 4.0f, 0.0f };

        SolveContactConstraint( constraint, bodyStateA, bodyStateB, false );

        // 필요한 friction impulse는 -4지만
        // max = mu * normalImpulse = 0.5 * 2 = 1.
        check( NearlyEqual( point.tangentImpulse, -1.0f ) );
        check( NearlyEqual( bodyStateB.linearVelocity.x, 3.0f ) );
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

        bodyState bodyStateA{};
        bodyState bodyStateB{};
        bodyStateB.linearVelocity = { 4.0f, 0.0f };

        SolveContactConstraint( constraint, bodyStateA, bodyStateB, false );

        check( NearlyEqual( constraint.points[0].tangentImpulse, 0.0f ) );
        check( NearlyEqual( bodyStateB.linearVelocity.x, 4.0f ) );
    }

    // speculative contact는 separation / dt보다 빠르게 gap을 닫지 못하게 함.
    {
        contactConstraint2 constraint{};
        constraint.bodyIdA = 0;
        constraint.bodyIdB = 1;
        constraint.normal = { 1.0f, 0.0f };
        constraint.invMassA = 0.0f;
        constraint.invMassB = 1.0f;
        constraint.invTimeStep = 60.0f;
        constraint.pointCount = 1;

        contactConstraintPoint2& point = constraint.points[0];

        point.baseSeparation = 0.01f;
        point.normalMass = 1.0f;

        bodyState bodyStateA{};
        bodyState bodyStateB{};
        bodyStateB.linearVelocity = { -2.0f, 0.0f };

        SolveContactConstraint( constraint, bodyStateA, bodyStateB, true );

        check( NearlyEqual( bodyStateB.linearVelocity.x, -0.6f ) );
        check( NearlyEqual( point.normalImpulse, 1.4f ) );

        SolveContactConstraint( constraint, bodyStateA, bodyStateB, false );

        check( NearlyEqual( bodyStateB.linearVelocity.x, -0.6f ) );
    }

    // restitution은 solver 전 접근 속도를 기준으로 목표 반발 속도를 만듦.
    {
        contactConstraint2 constraint{};
        constraint.bodyIdA = 0;
        constraint.bodyIdB = 1;
        constraint.normal = { 1.0f, 0.0f };
        constraint.invMassA = 0.0f;
        constraint.invMassB = 1.0f;
        constraint.restitution = 0.5f;
        constraint.pointCount = 1;

        contactConstraintPoint2& point = constraint.points[0];

        point.normalMass = 1.0f;
        point.normalImpulse = 4.0f;
        point.totalNormalImpulse = 4.0f;
        point.relativeNormalVelocity = -4.0f;

        bodyState bodyStateA{};
        bodyState bodyStateB{};

        ApplyRestitutionContactConstraint( constraint, bodyStateA, bodyStateB, 1.0f );

        // normal solve가 vn을 0까지 막았다고 가정하면
        // e=0.5, 충돌 전 vn=-4이므로 최종 목표 vn은 +2임.
        check( NearlyEqual( bodyStateB.linearVelocity.x, 2.0f ) );
        check( NearlyEqual( point.normalImpulse, 6.0f ) );
        check( NearlyEqual( point.totalNormalImpulse, 6.0f ) );
        check( NearlyEqual( point.restitutionImpulse, 2.0f ) );
    }

    // 이전 normal impulse가 남아 있어도 이번 step에 실제 compression이 없으면 bounce를 만들지 않음.
    {
        contactConstraint2 constraint{};
        constraint.bodyIdA = 0;
        constraint.bodyIdB = 1;
        constraint.normal = { 1.0f, 0.0f };
        constraint.invMassA = 0.0f;
        constraint.invMassB = 1.0f;
        constraint.restitution = 1.0f;
        constraint.invTimeStep = 10.0f;
        constraint.pointCount = 1;

        contactConstraintPoint2& point = constraint.points[0];

        point.baseSeparation = 0.1f;
        point.normalMass = 1.0f;
        point.normalImpulse = 4.0f;
        point.relativeNormalVelocity = -4.0f;

        bodyState bodyStateA{};

        bodyState bodyStateB{};
        bodyStateB.linearVelocity = { -1.0f, 0.0f };

        ApplyRestitutionContactConstraint( constraint, bodyStateA, bodyStateB, 1.0f );

        // compressionImpulse=0이므로 restitution은 armed되지 않음.
        // speculative bias s/dt=1이 현재 vn=-1을 그대로 허용함.
        check( NearlyEqual( bodyStateB.linearVelocity.x, -1.0f ) );
        check( NearlyEqual( point.normalImpulse, 4.0f ) );
        check( NearlyEqual( point.restitutionImpulse, 0.0f ) );
    }

    // threshold보다 느린 접촉은 restitution을 적용하지 않아 resting contact가 튀지 않음.
    {
        contactConstraint2 constraint{};
        constraint.bodyIdA = 0;
        constraint.bodyIdB = 1;
        constraint.normal = { 1.0f, 0.0f };
        constraint.invMassA = 0.0f;
        constraint.invMassB = 1.0f;
        constraint.restitution = 1.0f;
        constraint.pointCount = 1;

        contactConstraintPoint2& point = constraint.points[0];

        point.normalMass = 1.0f;
        point.normalImpulse = 0.5f;
        point.relativeNormalVelocity = -0.5f;

        bodyState bodyStateA{};
        bodyState bodyStateB{};

        ApplyRestitutionContactConstraint( constraint, bodyStateA, bodyStateB, 1.0f );

        check( NearlyEqual( bodyStateB.linearVelocity.x, 0.0f ) );
        check( NearlyEqual( point.normalImpulse, 0.5f ) );
    }

#pragma region RestitutionConservation

    // 두 Dynamic body의 대칭 충돌에서 momentum과 Poisson 반발 예산을 검사함.
    // 반복 restitution이 이미 사용한 압축량을 다시 반발 에너지로 만들면 안 됨.
    for( const float restitution : { 0.0f, 0.5f, 1.0f } )
    {
        contactConstraint2 constraint{};
        constraint.normal = { 1.0f, 0.0f };
        constraint.invMassA = 1.0f;
        constraint.invMassB = 1.0f;
        constraint.restitution = restitution;
        constraint.pointCount = 1;
        constraint.points[0].normalMass = 0.5f;
        constraint.points[0].relativeNormalVelocity = -2.0f;
        bodyState stateA{}, stateB{};
        stateA.linearVelocity = { 1.0f, 0.0f };
        stateB.linearVelocity = { -1.0f, 0.0f };
        SolveContactConstraint( constraint, stateA, stateB, false );
        for( int iteration = 0; iteration < 8; ++iteration )
        {
            ApplyRestitutionContactConstraint( constraint, stateA, stateB, 1.0f );
            check( NearlyEqual( stateA.linearVelocity.x + stateB.linearVelocity.x, 0.0f ) );
            check( NearlyEqual( stateA.linearVelocity.x, -restitution ) );
            check( NearlyEqual( stateB.linearVelocity.x, restitution ) );
            const auto& point = constraint.points[0];
            check( NearlyEqual( point.totalNormalImpulse - point.restitutionImpulse, 1.0f ) );
            check( NearlyEqual( point.restitutionImpulse, restitution ) );
            check( LengthSquared( stateA.linearVelocity ) + LengthSquared( stateB.linearVelocity ) <= 2.0f + 1e-5f );
        }
    }

#pragma endregion RestitutionConservation

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

        StoreContactImpulses( constraint, contactSim );

        check( NearlyEqual( contactSim.impulses[0].normalImpulse, 4.25f ) );
        check( NearlyEqual( contactSim.impulses[0].tangentImpulse, -0.75f ) );
    }

    return 0;
}
