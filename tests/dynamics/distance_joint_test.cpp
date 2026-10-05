#include <cmath>
#include <cstdlib>
#include <iostream>

#include "dynamics/distanceJointConstraint2.h"

using namespace zonai;

namespace
{
void check( bool condition, const char* message )
{
    if( !condition ) { std::cerr << message << '\n'; std::exit( EXIT_FAILURE ); }
}

bool near( float a, float b ) { return std::abs( a - b ) < 0.0001f; }
}

int main()
{
    bodySim a{}, b{};
    a.bodyId = 0; b.bodyId = 1;
    a.invMass = 1.0f; b.invMass = 0.5f;
    b.center = { 2.0f, 0.0f };
    distanceJointSim2 joint{};
    joint.jointId = 0; joint.bodyIdA = 0; joint.bodyIdB = 1; joint.length = 2.0f;
    auto constraint = prepareDistanceJointConstraint( joint, a, b, 1.0f / 60.0f );
    check( near( constraint.axialMass, 2.0f / 3.0f ), "unequal mass effective mass" );
    bodyState stateA{}, stateB{};
    stateB.linearVelocity = { 3.0f, 0.0f };
    solveDistanceJointConstraint( constraint, stateA, stateB, false );
    check( near( stateA.linearVelocity.x, 2.0f ) && near( stateB.linearVelocity.x, 2.0f ), "tension removes relative velocity" );
    check( near( stateA.linearVelocity.x + 2.0f * stateB.linearVelocity.x, 6.0f ), "linear momentum conserved" );
    check( constraint.impulse < 0.0f, "bilateral tension impulse stays negative" );

    constraint = prepareDistanceJointConstraint( joint, a, b, 1.0f / 60.0f );
    stateA = {}; stateB = {}; stateB.linearVelocity = { -3.0f, 0.0f };
    solveDistanceJointConstraint( constraint, stateA, stateB, false );
    check( constraint.impulse > 0.0f && near( stateA.linearVelocity.x, stateB.linearVelocity.x ), "compression also constrained" );

    a.localCenter = { 0.0f, 1.0f }; a.invInertia = 2.0f;
    joint.localAnchorA = { 0.0f, 2.0f }; joint.localAnchorB = { 0.0f, 1.0f };
    constraint = prepareDistanceJointConstraint( joint, a, b, 1.0f / 60.0f );
    check( near( constraint.anchorA.y, 1.0f ) && near( constraint.axialMass, 1.0f / 3.5f ), "anchors measured from COM" );
    stateA = {}; stateB = {}; stateB.linearVelocity = { 3.0f, 0.0f };
    solveDistanceJointConstraint( constraint, stateA, stateB, false );
    check( stateA.angularVelocity < 0.0f, "off center tension torque sign" );
    const float relative = stateB.linearVelocity.x - stateA.linearVelocity.x + stateA.angularVelocity;
    check( near( relative, 0.0f ), "anchor angular velocity included" );

    a = {}; a.bodyId = 0; b.invMass = 1.0f; joint.localAnchorA = {}; joint.localAnchorB = {};
    constraint = prepareDistanceJointConstraint( joint, a, b, 1.0f / 60.0f );
    stateA = {}; stateB = {}; stateA.linearVelocity = { 2.0f, 0.0f };
    solveDistanceJointConstraint( constraint, stateA, stateB, false );
    check( near( stateA.linearVelocity.x, 2.0f ) && near( stateB.linearVelocity.x, 2.0f ), "kinematic velocity included without changing it" );

    joint.impulse = -4.0f; joint.subStepTime = 1.0f / 60.0f;
    constraint = prepareDistanceJointConstraint( joint, a, b, joint.subStepTime );
    check( near( constraint.impulse, -4.0f ), "same step keeps warm start" );
    stateA = {}; stateB = {};
    warmStartDistanceJointConstraint( constraint, stateA, stateB );
    check( near( stateB.linearVelocity.x, -4.0f ), "warm start applies axial impulse" );
    constraint = prepareDistanceJointConstraint( joint, a, b, 1.0f / 120.0f );
    check( constraint.impulse == 0.0f, "changed step invalidates cached impulse" );

    b.center = {};
    constraint = prepareDistanceJointConstraint( joint, a, b, 1.0f / 120.0f );
    stateA = {}; stateB = {};
    solveDistanceJointConstraint( constraint, stateA, stateB, true );
    check( IsFinite( stateA.linearVelocity ) && IsFinite( stateB.linearVelocity ) && std::isfinite( constraint.impulse ), "coincident anchors finite" );
    check( LengthSquared( stateB.linearVelocity ) == 0.0f, "zero axis cannot choose correction direction" );
    return EXIT_SUCCESS;
}
