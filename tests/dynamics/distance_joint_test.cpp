#include <cmath>
#include <cstdlib>
#include <iostream>
#include <numbers>

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
    a = {}; b = {}; a.bodyId = 0; b.bodyId = 1; a.invMass = 1.0f; b.invMass = 0.5f; b.center = { 2.0f, 0.0f };
    joint = {}; joint.bodyIdA = 0; joint.bodyIdB = 1; joint.length = 1.0f;
    joint.enableSpring = true; joint.hertz = 2.0f; joint.dampingRatio = 0.7f;
    const float h = 1.0f / 60.0f, omega = 2.0f * std::numbers::pi_v<float> * joint.hertz;
    const float expected = -( 2.0f / 3.0f ) * h * omega * omega / ( 1.0f + 2.0f * joint.dampingRatio * h * omega + h * h * omega * omega );
    constraint = prepareDistanceJointConstraint( joint, a, b, h ); stateA = {}; stateB = {};
    solveDistanceJointConstraint( constraint, stateA, stateB, false );
    check( near( constraint.impulse, expected ), "spring relax uses physical Hertz/damping bias" );
    check( near( stateA.linearVelocity.x + 2.0f * stateB.linearVelocity.x, 0.0f ), "spring conserves two-body linear momentum" );
    auto biasConstraint = prepareDistanceJointConstraint( joint, a, b, h ); bodyState biasA{}, biasB{};
    solveDistanceJointConstraint( biasConstraint, biasA, biasB, true );
    check( near( biasConstraint.impulse, constraint.impulse ), "spring equation identical in both passes" );
    joint.dampingRatio = 2.0f;
    biasConstraint = prepareDistanceJointConstraint( joint, a, b, h ); biasA = {}; biasB = {};
    solveDistanceJointConstraint( biasConstraint, biasA, biasB, false );
    check( std::abs( biasConstraint.impulse ) < std::abs( expected ), "damping changes spring response" );
    joint.hertz = 0.0f; joint.impulse = -3.0f; joint.subStepTime = h;
    constraint = prepareDistanceJointConstraint( joint, a, b, h ); stateA = {}; stateB = {}; stateB.linearVelocity = { 2.0f, 0.0f };
    warmStartDistanceJointConstraint( constraint, stateA, stateB ); solveDistanceJointConstraint( constraint, stateA, stateB, true );
    check( constraint.impulse == 0.0f && near( stateB.linearVelocity.x, 2.0f ), "zero Hertz frees axis and discards warm impulse" );
    joint.enableSpring = false;
    constraint = prepareDistanceJointConstraint( joint, a, b, h ); stateA = {}; stateB = {}; stateB.linearVelocity = { 2.0f, 0.0f };
    solveDistanceJointConstraint( constraint, stateA, stateB, false );
    check( near( stateA.linearVelocity.x, stateB.linearVelocity.x ), "spring off restores rigid velocity constraint" );
    // 0 Hz로 spring 힘을 끄면 limit의 unilateral/speculative 응답을 따로 확인할 수 있음.
    joint = {}; joint.bodyIdA = 0; joint.bodyIdB = 1; joint.length = 2.0f;
    joint.enableSpring = true; joint.hertz = 0.0f; joint.enableLimit = true; joint.minLength = 1.0f; joint.maxLength = 3.0f;
    for( const float direction : { -1.0f, 1.0f } )
    {
        b.center = { 2.0f, 0.0f };
        constraint = prepareDistanceJointConstraint( joint, a, b, h ); stateA = {}; stateB = {}; stateB.linearVelocity.x = direction * 120.0f;
        solveDistanceJointConstraint( constraint, stateA, stateB, false );
        check( near( stateB.linearVelocity.x - stateA.linearVelocity.x, direction * 60.0f ), "speculative limit permits travel only to boundary" );
        check( near( stateA.linearVelocity.x + 2.0f * stateB.linearVelocity.x, direction * 240.0f ), "limit conserves linear momentum" );
        check( near( direction < 0.0f ? constraint.lowerImpulse : constraint.upperImpulse, 40.0f ), "limit signed direction and unilateral impulse" );
        constraint = prepareDistanceJointConstraint( joint, a, b, h ); stateA = {}; stateB = {}; stateB.linearVelocity.x = direction * 3.0f;
        solveDistanceJointConstraint( constraint, stateA, stateB, true );
        check( constraint.lowerImpulse == 0.0f && constraint.upperImpulse == 0.0f && near( stateB.linearVelocity.x, direction * 3.0f ), "interior motion not locked" );
        b.center.x = direction < 0.0f ? 1.0f : 3.0f;
        constraint = prepareDistanceJointConstraint( joint, a, b, h ); stateA = {}; stateB = {}; stateB.linearVelocity.x = direction * 3.0f;
        solveDistanceJointConstraint( constraint, stateA, stateB, false );
        check( near( stateA.linearVelocity.x, stateB.linearVelocity.x ), "boundary blocks outward axis velocity" );
        constraint = prepareDistanceJointConstraint( joint, a, b, h ); stateA = {}; stateB = {}; stateB.linearVelocity.x = -direction * 3.0f;
        solveDistanceJointConstraint( constraint, stateA, stateB, true );
        check( constraint.lowerImpulse == 0.0f && constraint.upperImpulse == 0.0f, "boundary allows motion back into interval" );
        b.center.x = direction < 0.0f ? 0.5f : 3.5f;
        constraint = prepareDistanceJointConstraint( joint, a, b, h ); stateA = {}; stateB = {};
        solveDistanceJointConstraint( constraint, stateA, stateB, true );
        check( direction * ( stateB.linearVelocity.x - stateA.linearVelocity.x ) < 0.0f, "bias repairs violated limit" );
        constraint = prepareDistanceJointConstraint( joint, a, b, h ); stateA = {}; stateB = {};
        solveDistanceJointConstraint( constraint, stateA, stateB, false );
        check( constraint.lowerImpulse == 0.0f && constraint.upperImpulse == 0.0f, "relax does not inject violation bias" );
    }
    b.center = { 2.0f, 0.0f }; joint.subStepTime = h; joint.lowerImpulse = 3.0f; joint.upperImpulse = 1.0f; joint.impulse = -4.0f;
    constraint = prepareDistanceJointConstraint( joint, a, b, h ); stateA = {}; stateB = {};
    warmStartDistanceJointConstraint( constraint, stateA, stateB );
    check( near( stateA.linearVelocity.x, -2.0f ) && near( stateB.linearVelocity.x, 1.0f ), "warm start sums lower minus upper without inactive spring cache" );
    constraint = prepareDistanceJointConstraint( joint, a, b, h * 0.5f );
    check( constraint.impulse == 0.0f && constraint.lowerImpulse == 0.0f && constraint.upperImpulse == 0.0f, "changed h resets all three caches" );
    joint.enableSpring = false;
    constraint = prepareDistanceJointConstraint( joint, a, b, h );
    check( constraint.lowerImpulse == 0.0f && constraint.upperImpulse == 0.0f, "rigid mode discards limit caches" );
    stateA = {}; stateB = {}; stateB.linearVelocity.x = 3.0f; solveDistanceJointConstraint( constraint, stateA, stateB, false );
    check( near( stateA.linearVelocity.x, stateB.linearVelocity.x ), "rigid mode overrides range" );
    joint.enableSpring = true; joint.minLength = joint.maxLength = 1.5f; joint.impulse = 0.0f; b.center.x = 1.5f;
    constraint = prepareDistanceJointConstraint( joint, a, b, h ); stateA = {}; stateB = {};
    solveDistanceJointConstraint( constraint, stateA, stateB, true );
    check( stateB.linearVelocity.x > stateA.linearVelocity.x && constraint.lowerImpulse == 0.0f && constraint.upperImpulse == 0.0f, "equal limits use rigid rest length as Box2D does" );
    return EXIT_SUCCESS;
}
