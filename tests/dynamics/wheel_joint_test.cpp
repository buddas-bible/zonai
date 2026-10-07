#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <numbers>
#include "dynamics/joints/wheelJointConstraint2.h"

using namespace zonai;

namespace
{

void check( bool ok, const char* message )
{
    if( !ok )
    {
        std::fprintf( stderr, "%s\n", message );
        std::exit( EXIT_FAILURE );
    }
}

bool near( float a, float b )
{
    return std::abs( a - b ) < 0.00001f;
}

}

int main()
{
    bodySim bodySimA{};
    bodySimA.bodyId = 0;
    bodySimA.invMass = 1.0f;
    bodySimA.invInertia = 1.0f;
    bodySim bodySimB{};
    bodySimB.bodyId = 1;
    bodySimB.invMass = 2.0f;
    bodySimB.invInertia = 2.0f;
    wheelJointSim2 joint{};
    joint.bodyIdA = 0;
    joint.bodyIdB = 1;
    joint.enableSpring = false;
    const float h = 1.0f / 60.0f;
    auto constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    bodyState stateA{};
    bodyState stateB{};
    stateB.linearVelocity = { 3.0f, 4.0f };
    stateB.angularVelocity = 5.0f;
    solveWheelJointConstraint( constraint, stateA, stateB, false );
    check( near( stateA.linearVelocity.x, 1.0f ) && near( stateB.linearVelocity.x, 1.0f ), "wheel removes lateral relative velocity with opposite impulses" );
    check( stateB.linearVelocity.y == 4.0f && stateB.angularVelocity == 5.0f, "spring off leaves axial translation and wheel rotation free" );
    check( near( stateA.linearVelocity.x + stateB.linearVelocity.x / 2.0f, 1.5f ), "wheel preserves total linear momentum" );

    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateA = {};
    stateB = {};
    stateA.deltaRotation = rot2::FromRadians( std::numbers::pi_v<float> / 2.0f );
    stateB.linearVelocity = { 3.0f, 4.0f };
    solveWheelJointConstraint( constraint, stateA, stateB, false );
    check( near( stateA.linearVelocity.y, 4.0f / 3.0f ) && near( stateB.linearVelocity.y, 4.0f / 3.0f ) && near( stateB.linearVelocity.x, 3.0f ), "translation axis follows latest A rotation" );

    // A가 회전하면 A의 축도 회전함. d+r_a의 팔 길이를 빼면 축에서 멀리 있는 B를 놓침.
    bodySimB.center = { 0.0f, 2.0f };
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateA = {};
    stateB = {};
    stateA.angularVelocity = 1.0f;
    solveWheelJointConstraint( constraint, stateA, stateB, false );
    check( near( constraint.impulse, 2.0f / 7.0f ) && near( stateA.angularVelocity, 3.0f / 7.0f ) && near( stateB.linearVelocity.x, -4.0f / 7.0f ), "rotating A line includes separation in angular Jacobian and effective mass" );

    bodySimA.invMass = 0.0f;
    bodySimA.invInertia = 0.0f;
    bodySimB.invMass = 1.0f;
    bodySimB.invInertia = 1.0f;
    bodySimB.center = {};
    joint.localAnchorB = { 0.0f, 1.0f };
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateA = {};
    stateB = {};
    stateB.angularVelocity = 2.0f;
    solveWheelJointConstraint( constraint, stateA, stateB, false );
    check( near( stateB.linearVelocity.x, 1.0f ) && near( stateB.angularVelocity, 1.0f ), "off-center wheel couples point velocity and rotation" );

    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateB = {};
    stateB.deltaRotation = rot2::FromRadians( std::numbers::pi_v<float> / 2.0f );
    stateB.angularVelocity = 2.0f;
    solveWheelJointConstraint( constraint, stateA, stateB, false );
    check( LengthSquared( stateB.linearVelocity ) < 0.00000001f && near( stateB.angularVelocity, 2.0f ), "off-center wheel uses latest B rotation for point velocity" );

    bodySimB.localCenter = { 1.0f, 0.0f };
    bodySimB.transform.rotation = rot2::FromRadians( std::numbers::pi_v<float> / 2.0f );
    joint.localAnchorB = { 2.0f, 0.0f };
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    check( near( constraint.anchorB.x, 0.0f ) && near( constraint.anchorB.y, 1.0f ), "wheel origin anchor becomes rotated COM lever arm" );
    bodySimB.localCenter = {};
    bodySimB.transform.rotation = {};
    joint.localAnchorB = {};
    bodySimB.center = { 1.0f, 0.0f };
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateB = {};
    solveWheelJointConstraint( constraint, stateA, stateB, false );
    check( LengthSquared( stateB.linearVelocity ) == 0.0f, "lateral relaxation adds no position bias" );
    solveWheelJointConstraint( constraint, stateA, stateB, true );
    check( stateB.linearVelocity.x < 0.0f, "biased pass restores wheel to translation line" );

    joint.enableSpring = true;
    joint.hertz = 3.0f;
    joint.dampingRatio = 0.7f;
    bodySimB.center = { 0.0f, 1.0f };
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateB = {};
    solveWheelJointConstraint( constraint, stateA, stateB, false );
    check( stateB.linearVelocity.y < 0.0f && constraint.springImpulse < 0.0f, "real suspension spring retains restoring bias during relaxation" );

    bodySimB.center = {};
    joint.localAnchorB = { 1.0f, 1.0f };
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateB = {};
    solveWheelJointConstraint( constraint, stateA, stateB, false );
    const vec2 pointVelocity = stateB.linearVelocity + Cross( stateB.angularVelocity, constraint.anchorB );
    check( constraint.springImpulse < 0.0f && near( pointVelocity.x, 0.0f ), "lateral constraint uses velocity updated by off-center spring" );
    bodySimB.center = { 0.0f, 1.0f };
    joint.localAnchorB = {};

    joint.subStepTime = h;
    joint.impulse = 2.0f;
    joint.springImpulse = 3.0f;
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateA = {};
    stateB = {};
    stateA.deltaRotation = rot2::FromRadians( std::numbers::pi_v<float> / 2.0f );
    warmStartWheelJointConstraint( constraint, stateA, stateB );
    check( near( stateB.linearVelocity.x, -3.0f ) && near( stateB.linearVelocity.y, -2.0f ), "warm start combines spring and lateral impulses along latest axis" );
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h / 2.0f );
    check( constraint.impulse == 0.0f && constraint.springImpulse == 0.0f, "timestep change clears wheel cache" );
    joint.hertz = 0.0f;
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateA = {};
    stateB = {};
    stateB.linearVelocity.y = 4.0f;
    warmStartWheelJointConstraint( constraint, stateA, stateB );
    solveWheelJointConstraint( constraint, stateA, stateB, false );
    check( constraint.springImpulse == 0.0f && stateB.linearVelocity.y == 4.0f, "zero Hertz removes spring cache and force without locking translation" );
    joint.enableSpring = false;
    joint.hertz = 3.0f;
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    check( constraint.springImpulse == 0.0f, "disabled spring ignores old axial impulse" );

    bodySimA.invMass = 1.0f;
    bodySimA.invInertia = 1.0f;
    bodySimB.invMass = 2.0f;
    bodySimB.invInertia = 2.0f;
    bodySimB.center = { 0.0f, 2.0f };
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateA = {};
    stateB = {};
    stateB.deltaPosition = { 0.0f, 1.0f };
    warmStartWheelJointConstraint( constraint, stateA, stateB );
    check( near( stateA.angularVelocity, -6.0f ) && near( stateB.linearVelocity.x, -4.0f ), "warm start includes current separation in A angular reaction" );
    bodySimA.invMass = 0.0f;
    bodySimA.invInertia = 0.0f;
    bodySimB.invMass = 0.0f;
    bodySimB.invInertia = 0.0f;
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateA = {};
    stateB = {};
    warmStartWheelJointConstraint( constraint, stateA, stateB );
    solveWheelJointConstraint( constraint, stateA, stateB, true );
    check( IsFinite( stateB.linearVelocity ) && constraint.impulse == 0.0f && constraint.springImpulse == 0.0f, "zero effective mass produces neither impulse nor NaN" );

    // 범위 안에서는 남은 거리 / h만큼 접근할 수 있지만 경계를 지나갈 속도는 제거함.
    bodySimA = {};
    bodySimA.bodyId = 0;
    bodySimB = {};
    bodySimB.bodyId = 1;
    bodySimB.invMass = 1.0f;
    bodySimB.invInertia = 1.0f;
    joint = {};
    joint.bodyIdA = 0;
    joint.bodyIdB = 1;
    joint.enableSpring = false;
    joint.enableLimit = true;
    joint.lowerTranslation = -0.5f;
    joint.upperTranslation = 0.5f;
    bodySimB.center = { 0.0f, 0.4f };
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateA = {};
    stateB = {};
    stateB.linearVelocity.y = 12.0f;
    stateB.angularVelocity = 3.0f;
    solveWheelJointConstraint( constraint, stateA, stateB, false );
    check( near( stateB.linearVelocity.y, 6.0f ) && near( constraint.upperImpulse, 6.0f ) && constraint.lowerImpulse == 0.0f, "upper speculation permits remaining distance but prevents crossing" );
    check( stateB.angularVelocity == 3.0f, "translation limit leaves centered wheel rotation free" );
    solveWheelJointConstraint( constraint, stateA, stateB, false );
    check( near( stateB.linearVelocity.y, 6.0f ) && near( constraint.upperImpulse, 6.0f ), "limit applies only increment of accumulated impulse" );

    bodySimB.center.y = -0.4f;
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateB = {};
    stateB.linearVelocity.y = -12.0f;
    solveWheelJointConstraint( constraint, stateA, stateB, false );
    check( near( stateB.linearVelocity.y, -6.0f ) && near( constraint.lowerImpulse, 6.0f ) && constraint.upperImpulse == 0.0f, "lower speculation has opposite reaction sign" );
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateB = {};
    stateB.linearVelocity.y = 2.0f;
    solveWheelJointConstraint( constraint, stateA, stateB, false );
    check( constraint.lowerImpulse == 0.0f && stateB.linearVelocity.y == 2.0f, "unilateral lower impulse permits return into range" );

    bodySimB.center.y = 0.7f;
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateB = {};
    solveWheelJointConstraint( constraint, stateA, stateB, false );
    check( stateB.linearVelocity.y == 0.0f, "violated translation adds no position bias during relaxation" );
    solveWheelJointConstraint( constraint, stateA, stateB, true );
    check( stateB.linearVelocity.y < 0.0f && constraint.upperImpulse > 0.0f, "biased pass restores violated upper boundary" );
    bodySimB.center.y = -0.7f;
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateB = {};
    solveWheelJointConstraint( constraint, stateA, stateB, true );
    check( stateB.linearVelocity.y > 0.0f && constraint.lowerImpulse > 0.0f, "biased pass restores violated lower boundary" );

    bodySimB.center = {};
    joint.lowerTranslation = 0.0f;
    joint.upperTranslation = 0.0f;
    for( const float speed : { -2.0f, 2.0f } )
    {
        constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
        stateB = {};
        stateB.linearVelocity.y = speed;
        solveWheelJointConstraint( constraint, stateA, stateB, false );
        check( stateB.linearVelocity.y == 0.0f && constraint.lowerImpulse >= 0.0f && constraint.upperImpulse >= 0.0f, "equal translations hold axial velocity in both directions" );
    }

    // 중심 밖 상한 임펄스가 회전을 바꾸면, 뒤의 수직 제약은 갱신된 점 속도를 읽어야 함.
    joint.localAnchorB = { 1.0f, 1.0f };
    joint.lowerTranslation = -2.0f;
    joint.upperTranslation = 1.0f;
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateB = {};
    stateB.linearVelocity.y = 2.0f;
    solveWheelJointConstraint( constraint, stateA, stateB, false );
    check( near( constraint.upperImpulse, 1.0f ) && near( stateB.linearVelocity.x, -0.5f ) && near( stateB.angularVelocity, -0.5f ), "limit then lateral constraint uses latest off-center point velocity" );

    joint.localAnchorB = {};
    joint.lowerTranslation = -0.5f;
    joint.upperTranslation = 0.5f;
    joint.enableSpring = true;
    bodySimB.center.y = 0.5f;
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateB = {};
    solveWheelJointConstraint( constraint, stateA, stateB, false );
    check( constraint.springImpulse < 0.0f && constraint.upperImpulse == 0.0f && stateB.linearVelocity.y < 0.0f, "spring can return inward from upper boundary without limit pulling outward" );
    bodySimB.center.y = 0.2f;
    joint.upperTranslation = 0.2f;
    joint.lowerTranslation = 0.1f;
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateB = {};
    stateB.deltaPosition.y = -0.1f;
    solveWheelJointConstraint( constraint, stateA, stateB, false );
    check( constraint.springImpulse < 0.0f && constraint.lowerImpulse > 0.0f && near( stateB.linearVelocity.y, 0.0f ), "lower limit uses current translation and spring updated velocity" );

    joint.enableSpring = false;
    joint.lowerTranslation = -0.5f;
    joint.upperTranslation = 0.5f;
    bodySimB.center = {};
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateA = {};
    stateB = {};
    stateA.deltaRotation = rot2::FromRadians( std::numbers::pi_v<float> / 2.0f );
    stateB.deltaPosition = { -0.5f, 0.0f };
    stateB.linearVelocity.x = -2.0f;
    solveWheelJointConstraint( constraint, stateA, stateB, false );
    check( std::abs( stateB.linearVelocity.x ) < 0.00001f && constraint.upperImpulse > 1.9f, "translation limit follows current A axis and accumulated position" );

    bodySimA.invMass = 1.0f;
    bodySimB.invMass = 2.0f;
    joint.subStepTime = h;
    joint.lowerImpulse = 2.0f;
    joint.upperImpulse = 3.0f;
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateA = {};
    stateB = {};
    warmStartWheelJointConstraint( constraint, stateA, stateB );
    check( near( stateA.linearVelocity.y, 1.0f ) && near( stateB.linearVelocity.y, -2.0f ), "warm start combines signed lower minus upper impulses with both reactions" );
    check( near( stateA.linearVelocity.y + stateB.linearVelocity.y / 2.0f, 0.0f ), "limit warm start preserves linear momentum" );
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h / 2.0f );
    check( constraint.lowerImpulse == 0.0f && constraint.upperImpulse == 0.0f, "substep time change discards both limit caches" );
    joint.enableLimit = false;
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    check( constraint.lowerImpulse == 0.0f && constraint.upperImpulse == 0.0f, "disabled limits ignore old impulses" );
    joint.enableLimit = true;
    bodySimA.invMass = 0.0f;
    bodySimB.invMass = 0.0f;
    bodySimB.invInertia = 0.0f;
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateA = {};
    stateB = {};
    warmStartWheelJointConstraint( constraint, stateA, stateB );
    solveWheelJointConstraint( constraint, stateA, stateB, true );
    check( constraint.lowerImpulse == 0.0f && constraint.upperImpulse == 0.0f && IsFinite( stateB.linearVelocity ), "zero axial mass discards limit caches and stays finite" );

    // 중심 연결의 I_A=1, I_B=0.5: 목표 상대속도 3에는 각임펄스 1이 필요함.
    bodySimA = {};
    bodySimA.bodyId = 0;
    bodySimA.invMass = 1.0f;
    bodySimA.invInertia = 1.0f;
    bodySimB = {};
    bodySimB.bodyId = 1;
    bodySimB.invMass = 1.0f;
    bodySimB.invInertia = 2.0f;
    joint = {};
    joint.bodyIdA = 0;
    joint.bodyIdB = 1;
    joint.enableSpring = false;
    joint.enableMotor = true;
    joint.motorSpeed = 3.0f;
    joint.maxMotorTorque = 120.0f;
    for( const bool useBias : { false, true } )
    {
        constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
        stateA = {};
        stateB = {};
        solveWheelJointConstraint( constraint, stateA, stateB, useBias );
        check( near( stateA.angularVelocity, -1.0f ) && near( stateB.angularVelocity, 2.0f ), "wheel motor reaches relative speed with opposite angular reactions" );
        check( near( stateA.angularVelocity + stateB.angularVelocity / 2.0f, 0.0f ), "centered motor preserves angular momentum" );
        solveWheelJointConstraint( constraint, stateA, stateB, useBias );
        check( near( constraint.motorImpulse, 1.0f ), "motor reapplies only accumulated impulse increment" );
    }
    joint.maxMotorTorque = 30.0f;
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateA = {};
    stateB = {};
    for( int i = 0; i < 8; ++i ) solveWheelJointConstraint( constraint, stateA, stateB, false );
    check( near( constraint.motorImpulse, 0.5f ) && near( stateB.angularVelocity - stateA.angularVelocity, 1.5f ), "motor torque times substep time bounds total impulse across iterations" );
    joint.motorSpeed = 0.0f;
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateA = {};
    stateB = {};
    stateB.angularVelocity = 3.0f;
    solveWheelJointConstraint( constraint, stateA, stateB, false );
    check( near( constraint.motorImpulse, -0.5f ) && near( stateB.angularVelocity - stateA.angularVelocity, 1.5f ), "zero speed brakes within negative torque limit" );
    joint.maxMotorTorque = 0.0f;
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateB = {};
    stateB.angularVelocity = 3.0f;
    stateA = {};
    solveWheelJointConstraint( constraint, stateA, stateB, false );
    check( stateB.angularVelocity == 3.0f && constraint.motorImpulse == 0.0f, "zero torque turns off motor force" );

    bodySimA.invMass = 0.0f;
    bodySimA.invInertia = 0.0f;
    bodySimB.invInertia = 1.0f;
    joint.motorSpeed = 3.0f;
    joint.maxMotorTorque = 30.0f;
    joint.localAnchorB = { 1.0f, 1.0f };
    joint.enableLimit = true;
    joint.lowerTranslation = -2.0f;
    joint.upperTranslation = 1.0f;
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateA = {};
    stateB = {};
    solveWheelJointConstraint( constraint, stateA, stateB, false );
    check( near( constraint.motorImpulse, 0.5f ) && near( constraint.upperImpulse, 0.25f ), "off-center upper limit reads motor updated angular velocity" );
    check( near( stateB.linearVelocity.x, 0.125f ) && near( stateB.linearVelocity.y, -0.25f ) && near( stateB.angularVelocity, 0.125f ), "lateral constraint reads latest motor and limit point velocity" );

    joint.localAnchorB = {};
    joint.enableLimit = false;
    bodySimA.invInertia = 1.0f;
    bodySimB.invInertia = 2.0f;
    joint.subStepTime = h;
    joint.motorImpulse = 2.0f;
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateA = {};
    stateB = {};
    warmStartWheelJointConstraint( constraint, stateA, stateB );
    check( near( constraint.motorImpulse, 0.5f ) && near( stateA.angularVelocity, -0.5f ) && near( stateB.angularVelocity, 1.0f ), "warm motor cache is clamped and applies both angular reactions" );
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h / 2.0f );
    check( constraint.motorImpulse == 0.0f, "changed substep time discards motor cache" );
    joint.enableMotor = false;
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    check( constraint.motorImpulse == 0.0f, "disabled motor ignores stale angular cache" );
    joint.enableMotor = true;
    bodySimA.invInertia = 0.0f;
    bodySimB.invInertia = 0.0f;
    constraint = prepareWheelJointConstraint( joint, bodySimA, bodySimB, h );
    stateA = {};
    stateB = {};
    solveWheelJointConstraint( constraint, stateA, stateB, false );
    check( constraint.motorImpulse == 0.0f && std::isfinite( stateB.angularVelocity ), "fixed rotation discards motor cache without NaN" );

    return EXIT_SUCCESS;
}
