#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numbers>
#include "dynamics/wheelJointConstraint2.h"

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

    return EXIT_SUCCESS;
}
