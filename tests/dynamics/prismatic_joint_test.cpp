#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "dynamics/joints/prismaticJointConstraint2.h"

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

    prismaticJointSim2 joint{};
    joint.bodyIdA = 0;
    joint.bodyIdB = 1;

    auto constraint = preparePrismaticJointConstraint( joint, bodySimA, bodySimB, h );
    bodyState stateA{};
    bodyState stateB{};
    stateB.linearVelocity = { 3.0f, 2.0f };
    stateB.angularVelocity = 4.0f;
    solvePrismaticJointConstraint( constraint, stateA, stateB, false );
    check( stateB.linearVelocity.x == 3.0f && stateA.linearVelocity.x == 0.0f, "basic prismatic leaves axial velocity free" );
    check( near( stateA.linearVelocity.y, stateB.linearVelocity.y ) && near( stateA.angularVelocity, stateB.angularVelocity ), "basic prismatic removes lateral and relative angular velocity" );

    bodySimA.invMass = 0.0f;
    bodySimA.invInertia = 0.0f;
    bodySimB.invMass = 1.0f;
    bodySimB.invInertia = 1.0f;
    bodySimB.center = { 0.0f, 1.0f };
    constraint = preparePrismaticJointConstraint( joint, bodySimA, bodySimB, h );
    stateA = {};
    stateB = {};
    solvePrismaticJointConstraint( constraint, stateA, stateB, false );
    check( LengthSquared( stateB.linearVelocity ) == 0.0f && stateB.angularVelocity == 0.0f, "prismatic relaxation adds no position bias" );
    solvePrismaticJointConstraint( constraint, stateA, stateB, true );
    check( stateB.linearVelocity.y < 0.0f, "biased pass restores lateral rail error" );

    bodySimB.center = {};
    bodySimB.transform.rotation = rot2::FromRadians( 0.25f );
    constraint = preparePrismaticJointConstraint( joint, bodySimA, bodySimB, h );
    stateB = {};
    solvePrismaticJointConstraint( constraint, stateA, stateB, false );
    check( stateB.angularVelocity == 0.0f, "angular relaxation adds no position bias" );
    solvePrismaticJointConstraint( constraint, stateA, stateB, true );
    check( stateB.angularVelocity < 0.0f, "biased pass restores relative angle" );

    bodySimB.transform.rotation = {};
    bodySimB.localCenter = { 1.0f, 0.0f };
    joint.localAnchorB = { 2.0f, 1.0f };
    constraint = preparePrismaticJointConstraint( joint, bodySimA, bodySimB, h );
    check( near( constraint.anchorB.x, 1.0f ) && near( constraint.anchorB.y, 1.0f ), "prismatic origin anchor becomes COM lever arm" );
    stateB = {};
    stateB.angularVelocity = 2.0f;
    solvePrismaticJointConstraint( constraint, stateA, stateB, false );
    check( IsFinite( stateB.linearVelocity ) && std::isfinite( stateB.angularVelocity ) && std::abs( stateB.linearVelocity.y ) > 0.0f && std::abs( stateB.angularVelocity ) < 2.0f, "off-center prismatic couples lateral and angular motion" );

    bodySimA.invMass = 0.0f;
    bodySimA.invInertia = 0.0f;
    bodySimB.invMass = 0.0f;
    bodySimB.invInertia = 0.0f;
    joint.localAnchorB = {};
    bodySimB.localCenter = {};
    constraint = preparePrismaticJointConstraint( joint, bodySimA, bodySimB, h );
    stateA = {};
    stateB = {};
    warmStartPrismaticJointConstraint( constraint, stateA, stateB );
    solvePrismaticJointConstraint( constraint, stateA, stateB, true );
    check( IsFinite( stateA.linearVelocity ) && IsFinite( stateB.linearVelocity ) && std::isfinite( stateA.angularVelocity ) && std::isfinite( stateB.angularVelocity ) && LengthSquared( constraint.impulse ) == 0.0f, "degenerate prismatic effective mass stays finite" );

    bodySimA.invMass = 1.0f;
    bodySimA.invInertia = 1.0f;
    bodySimB.invMass = 1.0f;
    bodySimB.invInertia = 1.0f;
    joint.subStepTime = h;
    joint.impulse = { 2.0f, 3.0f };
    constraint = preparePrismaticJointConstraint( joint, bodySimA, bodySimB, h );
    stateA = {};
    stateB = {};
    warmStartPrismaticJointConstraint( constraint, stateA, stateB );
    check( near( stateA.linearVelocity.y, -2.0f ) && near( stateB.linearVelocity.y, 2.0f ), "prismatic warm start applies opposite lateral impulse" );
    check( near( stateA.angularVelocity, -3.0f ) && near( stateB.angularVelocity, 3.0f ), "prismatic warm start applies opposite angular impulse" );

    constraint = preparePrismaticJointConstraint( joint, bodySimA, bodySimB, h / 2.0f );
    check( LengthSquared( constraint.impulse ) == 0.0f, "timestep change clears prismatic cache" );

    return EXIT_SUCCESS;
}
