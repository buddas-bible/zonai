#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "dynamics/joints/weldJointConstraint2.h"

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
    const float h = 1.0f / 60.0f;

    bodySim bodySimA{};
    bodySimA.bodyId = 0;
    bodySimA.invMass = 1.0f;
    bodySimA.invInertia = 1.0f;
    bodySim bodySimB{};
    bodySimB.bodyId = 1;
    bodySimB.invMass = 1.0f;
    bodySimB.invInertia = 1.0f;

    weldJointSim2 joint{};
    joint.bodyIdA = 0;
    joint.bodyIdB = 1;

    auto constraint = prepareWeldJointConstraint( joint, bodySimA, bodySimB, h );
    bodyState stateA{};
    bodyState stateB{};
    stateB.linearVelocity = { 2.0f, -3.0f };
    stateB.angularVelocity = 4.0f;
    solveWeldJointConstraint( constraint, stateA, stateB, false );
    check( near( stateA.linearVelocity.x, stateB.linearVelocity.x ) && near( stateA.linearVelocity.y, stateB.linearVelocity.y ), "Weld removes relative anchor velocity" );
    check( near( stateA.angularVelocity, stateB.angularVelocity ), "Weld removes relative angular velocity" );

    bodySimA.invMass = 0.0f;
    bodySimA.invInertia = 0.0f;
    bodySimB.invMass = 1.0f;
    bodySimB.invInertia = 1.0f;
    bodySimB.center = { 1.0f, 2.0f };
    bodySimB.transform.position = bodySimB.center;
    constraint = prepareWeldJointConstraint( joint, bodySimA, bodySimB, h );
    stateA = {};
    stateB = {};
    solveWeldJointConstraint( constraint, stateA, stateB, false );
    check( LengthSquared( stateB.linearVelocity ) == 0.0f, "Weld relaxation adds no linear position bias" );
    solveWeldJointConstraint( constraint, stateA, stateB, true );
    check( stateB.linearVelocity.x < 0.0f && stateB.linearVelocity.y < 0.0f, "Weld bias restores anchor position error" );

    bodySimB.center = {};
    bodySimB.transform = { {}, rot2::FromRadians( 0.25f ) };
    constraint = prepareWeldJointConstraint( joint, bodySimA, bodySimB, h );
    stateB = {};
    solveWeldJointConstraint( constraint, stateA, stateB, false );
    check( stateB.angularVelocity == 0.0f, "Weld relaxation adds no angular position bias" );
    solveWeldJointConstraint( constraint, stateA, stateB, true );
    check( stateB.angularVelocity < 0.0f, "Weld bias restores relative angle error" );

    bodySimB.transform = {};
    bodySimB.localCenter = { 1.0f, 0.0f };
    joint.localAnchorB = { 2.0f, 1.0f };
    constraint = prepareWeldJointConstraint( joint, bodySimA, bodySimB, h );
    check( near( constraint.anchorB.x, 1.0f ) && near( constraint.anchorB.y, 1.0f ), "Weld origin anchor becomes COM lever arm" );
    stateB = {};
    stateB.linearVelocity = { 1.0f, 0.5f };
    stateB.angularVelocity = 2.0f;
    solveWeldJointConstraint( constraint, stateA, stateB, false );
    check( IsFinite( stateB.linearVelocity ) && std::isfinite( stateB.angularVelocity ), "off-center Weld coupling stays finite" );
    check( std::abs( stateB.linearVelocity.x ) < 1.0f || std::abs( stateB.linearVelocity.y ) < 0.5f || std::abs( stateB.angularVelocity ) < 2.0f, "off-center Weld changes coupled motion" );

    bodySimA.invMass = 0.0f;
    bodySimA.invInertia = 0.0f;
    bodySimB.invMass = 0.0f;
    bodySimB.invInertia = 0.0f;
    bodySimB.localCenter = {};
    joint.localAnchorB = {};
    constraint = prepareWeldJointConstraint( joint, bodySimA, bodySimB, h );
    stateA = {};
    stateB = {};
    warmStartWeldJointConstraint( constraint, stateA, stateB );
    solveWeldJointConstraint( constraint, stateA, stateB, true );
    check( IsFinite( stateA.linearVelocity ) && IsFinite( stateB.linearVelocity ) && std::isfinite( stateA.angularVelocity ) && std::isfinite( stateB.angularVelocity ), "degenerate Weld effective mass stays finite" );

    bodySimA.invMass = 1.0f;
    bodySimA.invInertia = 1.0f;
    bodySimB.invMass = 1.0f;
    bodySimB.invInertia = 1.0f;
    joint.subStepTime = h;
    joint.impulse = { 2.0f, -3.0f };
    joint.angularImpulse = 4.0f;
    constraint = prepareWeldJointConstraint( joint, bodySimA, bodySimB, h );
    stateA = {};
    stateB = {};
    warmStartWeldJointConstraint( constraint, stateA, stateB );
    check( near( stateA.linearVelocity.x, -2.0f ) && near( stateA.linearVelocity.y, 3.0f ), "Weld warm start applies opposite linear impulse to A" );
    check( near( stateB.linearVelocity.x, 2.0f ) && near( stateB.linearVelocity.y, -3.0f ), "Weld warm start applies opposite linear impulse to B" );
    check( near( stateA.angularVelocity, -4.0f ) && near( stateB.angularVelocity, 4.0f ), "Weld warm start applies cached angular impulse" );

    constraint = prepareWeldJointConstraint( joint, bodySimA, bodySimB, h / 2.0f );
    check( LengthSquared( constraint.impulse ) == 0.0f && constraint.angularImpulse == 0.0f, "timestep change clears Weld cache" );
    return EXIT_SUCCESS;
}
