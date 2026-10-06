#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numbers>
#include "dynamics/revoluteJointConstraint2.h"

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
    bodySimB.invInertia = 1.0f;
    revoluteJointSim2 joint{};
    joint.jointId = 0;
    joint.bodyIdA = 0;
    joint.bodyIdB = 1;
    const float h = 1.0f / 60.0f;
    auto constraint = prepareRevoluteJointConstraint( joint, bodySimA, bodySimB, h );
    bodyState stateA{};
    bodyState stateB{};
    stateA.angularVelocity = 10.0f;
    stateB.angularVelocity = -20.0f;
    stateB.linearVelocity = { 3.0f, -6.0f };
    solveRevoluteJointConstraint( constraint, stateA, stateB, false );
    check( near( stateA.linearVelocity.x, 1.0f ) && near( stateA.linearVelocity.y, -2.0f ), "A receives opposite anchor impulse" );
    check( near( stateB.linearVelocity.x, 1.0f ) && near( stateB.linearVelocity.y, -2.0f ), "relative anchor velocity is removed in both axes" );
    check( stateA.angularVelocity == 10.0f && stateB.angularVelocity == -20.0f, "centered hinge allows free relative rotation" );

    // 두 Dynamic 물체가 모두 중심 밖에서 연결될 때 A의 회전 속도와 반작용도 계산해야 함.
    revoluteJointSim2 pairJoint = joint;
    pairJoint.localAnchorA = { 1.0f, 0.0f };
    pairJoint.localAnchorB = { 1.0f, 1.0f };
    auto pairConstraint = prepareRevoluteJointConstraint( pairJoint, bodySimA, bodySimB, h );
    bodyState pairStateA{};
    bodyState pairStateB{};
    pairStateA.angularVelocity = 1.0f;
    pairStateB.linearVelocity = { 1.0f, 1.0f };
    solveRevoluteJointConstraint( pairConstraint, pairStateA, pairStateB, false );
    check( near( pairStateA.angularVelocity, 20.0f / 19.0f ) && near( pairStateB.angularVelocity, 4.0f / 19.0f ), "off-center dynamic pair receives opposite angular responses" );
    const vec2 pointVelocityA = pairStateA.linearVelocity + Cross( pairStateA.angularVelocity, pairConstraint.anchorA );
    const vec2 pointVelocityB = pairStateB.linearVelocity + Cross( pairStateB.angularVelocity, pairConstraint.anchorB );
    check( LengthSquared( pointVelocityB - pointVelocityA ) < 0.00000001f, "dynamic A angular point velocity participates in coupled solve" );
    check( near( pairStateA.linearVelocity.x + pairStateB.linearVelocity.x / 2.0f, 0.5f ) && near( pairStateA.linearVelocity.y + pairStateB.linearVelocity.y / 2.0f, 0.5f ), "dynamic pair preserves total linear momentum" );

    bodySimA.invMass = 0.0f;
    bodySimA.invInertia = 0.0f;
    bodySimB.invMass = 1.0f;
    joint.localAnchorA = { 1.0f, 1.0f };
    joint.localAnchorB = { 1.0f, 1.0f };
    constraint = prepareRevoluteJointConstraint( joint, bodySimA, bodySimB, h );
    stateA = {};
    stateB = {};
    stateB.linearVelocity = { 1.0f, 0.0f };
    solveRevoluteJointConstraint( constraint, stateA, stateB, false );
    check( near( stateB.linearVelocity.x, 1.0f / 3.0f ) && near( stateB.linearVelocity.y, -1.0f / 3.0f ) && near( stateB.angularVelocity, 1.0f / 3.0f ), "off-center 2x2 mass couples translation and rotation" );
    check( LengthSquared( stateA.linearVelocity ) == 0.0f && stateA.angularVelocity == 0.0f, "static A remains still" );

    constraint = prepareRevoluteJointConstraint( joint, bodySimA, bodySimB, h );
    stateB = {};
    stateB.linearVelocity = { 1.0f, -1.0f };
    stateB.angularVelocity = 1.0f;
    solveRevoluteJointConstraint( constraint, stateA, stateB, false );
    check( LengthSquared( constraint.impulse ) < 0.00000001f && stateB.angularVelocity == 1.0f, "rotation about stationary anchor needs no correction" );

    constraint = prepareRevoluteJointConstraint( joint, bodySimA, bodySimB, h );
    stateB = {};
    stateB.deltaRotation = rot2::FromRadians( std::numbers::pi_v<float> / 2.0f );
    stateB.linearVelocity = { 1.0f, 0.0f };
    solveRevoluteJointConstraint( constraint, stateA, stateB, false );
    check( near( stateB.linearVelocity.x, 1.0f / 3.0f ) && near( stateB.linearVelocity.y, 1.0f / 3.0f ) && near( stateB.angularVelocity, 1.0f / 3.0f ), "effective mass follows latest anchor rotation" );

    bodySimB.localCenter = { 1.0f, 0.0f };
    bodySimB.transform.rotation = rot2::FromRadians( std::numbers::pi_v<float> / 2.0f );
    joint.localAnchorB = { 2.0f, 0.0f };
    joint.impulse = { 2.0f, 3.0f };
    joint.subStepTime = h;
    constraint = prepareRevoluteJointConstraint( joint, bodySimA, bodySimB, h );
    check( near( constraint.anchorB.x, 0.0f ) && near( constraint.anchorB.y, 1.0f ), "local origin anchor becomes rotated COM lever arm" );
    stateB = {};
    stateB.deltaRotation = rot2::FromRadians( std::numbers::pi_v<float> / 2.0f );
    warmStartRevoluteJointConstraint( constraint, stateA, stateB );
    check( near( stateB.linearVelocity.x, 2.0f ) && near( stateB.linearVelocity.y, 3.0f ) && near( stateB.angularVelocity, -3.0f ), "warm start uses latest rotated anchor" );
    constraint = prepareRevoluteJointConstraint( joint, bodySimA, bodySimB, h / 2.0f );
    check( LengthSquared( constraint.impulse ) == 0.0f, "changed substep time discards cached impulse" );

    joint.localAnchorA = {};
    joint.localAnchorB = {};
    joint.impulse = {};
    bodySimB.localCenter = {};
    bodySimB.transform.rotation = {};
    bodySimB.center = { 1.0f, 2.0f };
    constraint = prepareRevoluteJointConstraint( joint, bodySimA, bodySimB, h );
    stateB = {};
    solveRevoluteJointConstraint( constraint, stateA, stateB, false );
    check( LengthSquared( stateB.linearVelocity ) == 0.0f, "relaxation pass adds no positional bias" );
    solveRevoluteJointConstraint( constraint, stateA, stateB, true );
    check( stateB.linearVelocity.x < 0.0f && stateB.linearVelocity.y < 0.0f, "biased pass closes anchor separation" );

    bodySimB.invMass = 0.0f;
    bodySimB.invInertia = 0.0f;
    constraint = prepareRevoluteJointConstraint( joint, bodySimA, bodySimB, h );
    stateB = {};
    solveRevoluteJointConstraint( constraint, stateA, stateB, true );
    check( IsFinite( stateB.linearVelocity ) && LengthSquared( constraint.impulse ) == 0.0f, "singular effective mass does not create NaN" );

    bodySim limitA{};
    limitA.bodyId = 0;
    limitA.invMass = 1.0f;
    limitA.invInertia = 1.0f;
    bodySim limitB{};
    limitB.bodyId = 1;
    limitB.invMass = 1.0f;
    limitB.invInertia = 2.0f;
    revoluteJointSim2 limited{};
    limited.bodyIdA = 0;
    limited.bodyIdB = 1;
    limited.enableLimit = true;
    limited.lowerAngle = -0.5f;
    limited.upperAngle = 0.5f;
    limitB.transform.rotation = rot2::FromRadians( -0.5f );
    constraint = prepareRevoluteJointConstraint( limited, limitA, limitB, h );
    stateA = {};
    stateB = {};
    stateB.angularVelocity = -3.0f;
    solveRevoluteJointConstraint( constraint, stateA, stateB, false );
    check( near( constraint.lowerImpulse, 1.0f ) && constraint.upperImpulse == 0.0f && near( stateA.angularVelocity, -1.0f ) && near( stateB.angularVelocity, -1.0f ), "lower limit applies opposite angular impulses and preserves angular momentum" );

    limitB.transform.rotation = rot2::FromRadians( 0.5f );
    constraint = prepareRevoluteJointConstraint( limited, limitA, limitB, h );
    stateA = {};
    stateB = {};
    stateB.angularVelocity = 3.0f;
    solveRevoluteJointConstraint( constraint, stateA, stateB, false );
    check( near( constraint.upperImpulse, 1.0f ) && constraint.lowerImpulse == 0.0f && near( stateA.angularVelocity, 1.0f ) && near( stateB.angularVelocity, 1.0f ), "upper limit opposes outward rotation" );
    constraint = prepareRevoluteJointConstraint( limited, limitA, limitB, h );
    stateA = {};
    stateB = {};
    stateB.angularVelocity = -1.0f;
    solveRevoluteJointConstraint( constraint, stateA, stateB, false );
    check( stateB.angularVelocity == -1.0f && constraint.upperImpulse == 0.0f, "upper boundary permits inward rotation" );

    limitB.transform.rotation = {};
    constraint = prepareRevoluteJointConstraint( limited, limitA, limitB, h );
    stateA = {};
    stateB = {};
    stateB.angularVelocity = 0.2f;
    solveRevoluteJointConstraint( constraint, stateA, stateB, false );
    check( stateB.angularVelocity == 0.2f && constraint.lowerImpulse == 0.0f && constraint.upperImpulse == 0.0f, "interior rotation remains free" );
    limitB.transform.rotation = rot2::FromRadians( 0.49f );
    constraint = prepareRevoluteJointConstraint( limited, limitA, limitB, h );
    stateA = {};
    stateB = {};
    stateB.angularVelocity = 6.0f;
    solveRevoluteJointConstraint( constraint, stateA, stateB, false );
    check( near( stateB.angularVelocity - stateA.angularVelocity, 0.6f ), "speculation limits rotation to remaining angle over timestep" );

    limitB.transform.rotation = rot2::FromRadians( 0.7f );
    constraint = prepareRevoluteJointConstraint( limited, limitA, limitB, h );
    stateA = {};
    stateB = {};
    solveRevoluteJointConstraint( constraint, stateA, stateB, false );
    check( stateA.angularVelocity == 0.0f && stateB.angularVelocity == 0.0f, "angular relaxation adds no violation bias" );
    solveRevoluteJointConstraint( constraint, stateA, stateB, true );
    check( stateB.angularVelocity < stateA.angularVelocity && constraint.upperImpulse > 0.0f, "biased pass returns angle toward upper boundary" );

    limited.referenceAngle = 3.0f;
    limited.lowerAngle = -0.2f;
    limited.upperAngle = 0.2f;
    limitB.transform.rotation = rot2::FromRadians( -3.08f );
    constraint = prepareRevoluteJointConstraint( limited, limitA, limitB, h );
    stateA = {};
    stateB = {};
    stateB.deltaRotation = rot2::FromRadians( 0.1f );
    stateB.angularVelocity = 1.0f;
    solveRevoluteJointConstraint( constraint, stateA, stateB, true );
    check( constraint.upperImpulse > 0.0f && constraint.lowerImpulse == 0.0f, "reference angle wraps across pi and uses latest delta rotation" );

    constraint = prepareRevoluteJointConstraint( limited, limitA, limitB, h );
    stateA = {};
    stateB = {};
    stateA.deltaRotation = rot2::FromRadians( 0.3f );
    stateB.angularVelocity = 0.2f;
    solveRevoluteJointConstraint( constraint, stateA, stateB, true );
    check( constraint.upperImpulse == 0.0f && constraint.lowerImpulse == 0.0f && stateB.angularVelocity == 0.2f, "relative angle includes inverse latest rotation of A" );

    limited.subStepTime = h;
    limited.lowerImpulse = 2.0f;
    limited.upperImpulse = 0.5f;
    constraint = prepareRevoluteJointConstraint( limited, limitA, limitB, h );
    stateA = {};
    stateB = {};
    warmStartRevoluteJointConstraint( constraint, stateA, stateB );
    check( near( stateA.angularVelocity, -1.5f ) && near( stateB.angularVelocity, 3.0f ), "warm start uses signed lower minus upper impulse" );
    constraint = prepareRevoluteJointConstraint( limited, limitA, limitB, h / 2.0f );
    check( constraint.lowerImpulse == 0.0f && constraint.upperImpulse == 0.0f, "timestep change invalidates angular cache" );
    limited.enableLimit = false;
    constraint = prepareRevoluteJointConstraint( limited, limitA, limitB, h );
    stateA = {};
    stateB = {};
    warmStartRevoluteJointConstraint( constraint, stateA, stateB );
    check( stateA.angularVelocity == 0.0f && stateB.angularVelocity == 0.0f, "disabled limits ignore stale angular cache" );
    limited.enableLimit = true;
    limitA.invInertia = 0.0f;
    limitB.invInertia = 0.0f;
    constraint = prepareRevoluteJointConstraint( limited, limitA, limitB, h );
    stateA = {};
    stateB = {};
    warmStartRevoluteJointConstraint( constraint, stateA, stateB );
    solveRevoluteJointConstraint( constraint, stateA, stateB, true );
    check( constraint.lowerImpulse == 0.0f && constraint.upperImpulse == 0.0f && std::isfinite( stateB.angularVelocity ), "zero rotational inverse mass disables angular correction" );

    return EXIT_SUCCESS;
}
