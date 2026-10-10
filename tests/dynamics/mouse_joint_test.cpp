#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "dynamics/joints/motorJointConstraint2.h"
#include "dynamics/joints/mouseJointConstraint2.h"

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
    return std::abs( a - b ) < 0.0001f;
}

}

int main()
{
    bodySim body{};
    body.bodyId = 1;
    body.invMass = 0.5f;
    body.invInertia = 1.0f;
    mouseJointSim2 joint{};
    joint.jointId = 0;
    joint.bodyIdB = 1;
    joint.maxForce = 10.0f;
    joint.target = { 10.0f, 10.0f };
    auto constraint = prepareMouseJointConstraint( joint, body, 1.0f / 60.0f );
    bodyState state{};
    solveMouseJointConstraint( constraint, state );
    check( state.linearVelocity.x > 0.0f && near( state.linearVelocity.x, state.linearVelocity.y ), "target attracts in both axes" );
    check( near( Length( constraint.impulse ), joint.maxForce / 60.0f ), "accumulated vector impulse limited by h times force" );
    solveMouseJointConstraint( constraint, state );
    check( Length( constraint.impulse ) <= joint.maxForce / 60.0f + 0.000001f, "second pass shares force budget" );
    joint.maxForce = 0.0f;
    joint.impulse = { 2.0f, 3.0f };
    joint.subStepTime = 1.0f / 60.0f;
    constraint = prepareMouseJointConstraint( joint, body, joint.subStepTime );
    state = {};
    warmStartMouseJointConstraint( constraint, state );
    solveMouseJointConstraint( constraint, state );
    check( LengthSquared( state.linearVelocity ) == 0.0f && LengthSquared( constraint.impulse ) == 0.0f, "zero force also clamps warm start" );
    joint.maxForce = 1000.0f;
    joint.impulse = {};
    joint.localAnchorB = { 0.0f, 2.0f };
    body.localCenter = { 0.0f, 1.0f };
    body.center = { 0.0f, 1.0f };
    joint.target = { 2.0f, 2.0f };
    constraint = prepareMouseJointConstraint( joint, body, joint.subStepTime );
    state = {};
    check( near( constraint.anchorB.y, 1.0f ), "origin anchor converted to COM lever arm" );
    solveMouseJointConstraint( constraint, state );
    check( state.angularVelocity < 0.0f, "off-center pull rotates clockwise" );
    joint.impulse = { 0.02f, -0.01f };
    constraint = prepareMouseJointConstraint( joint, body, joint.subStepTime );
    state = {};
    warmStartMouseJointConstraint( constraint, state );
    check( near( state.linearVelocity.x, 0.01f ) && near( state.angularVelocity, -0.02f ), "warm start includes lever arm" );
    constraint = prepareMouseJointConstraint( joint, body, 1.0f / 120.0f );
    check( LengthSquared( constraint.impulse ) == 0.0f, "changed h discards cache" );
    joint.impulse = {};
    joint.localAnchorB = { 1.0f, 1.0f };
    joint.target = { 1.0f, 1.0f };
    body.localCenter = {};
    body.center = {};
    body.invMass = 1.0f;
    body.invInertia = 1.0f;
    constraint = prepareMouseJointConstraint( joint, body, 1.0f / 60.0f );
    check( near( constraint.massX.x, 2.0f / 3.0f ) && near( constraint.massX.y, 1.0f / 3.0f ), "off-diagonal inverse effective mass" );
    state = {};
    state.linearVelocity = { 1.0f, 0.0f };
    solveMouseJointConstraint( constraint, state );
    check( state.linearVelocity.y < 0.0f && state.angularVelocity > 0.0f, "two-axis velocity coupling" );
    joint.target = { 4.0f, 1.0f };
    joint.hertz = 0.0f;
    constraint = prepareMouseJointConstraint( joint, body, 1.0f / 60.0f );
    state = {};
    solveMouseJointConstraint( constraint, state );
    check( LengthSquared( state.linearVelocity ) == 0.0f, "zero Hertz removes positional spring" );
    joint.hertz = 5.0f;
    body.invMass = 0.0f;
    body.invInertia = 0.0f;
    state = {};
    constraint = prepareMouseJointConstraint( joint, body, 1.0f / 120.0f );
    solveMouseJointConstraint( constraint, state );
    check( IsFinite( state.linearVelocity ) && state.angularVelocity == 0.0f, "singular mass remains finite" );

    // Motor Joint Stage 1 solver는 위치 오차를 없애는 대신 두 Body의 상대 속도를 목표값으로 맞춤.
    bodySim motorBodyA{};
    motorBodyA.bodyId = 0;
    motorBodyA.invMass = 1.0f;
    motorBodyA.invInertia = 1.0f;
    bodySim motorBodyB{};
    motorBodyB.bodyId = 1;
    motorBodyB.invMass = 1.0f;
    motorBodyB.invInertia = 1.0f;
    motorJointSim2 motorJoint{};
    motorJoint.jointId = 0;
    motorJoint.bodyIdA = 0;
    motorJoint.bodyIdB = 1;
    motorJoint.linearVelocity = { 2.0f, -1.0f };
    motorJoint.maxVelocityForce = 1000.0f;
    motorJoint.angularVelocity = 3.0f;
    motorJoint.maxVelocityTorque = 1000.0f;
    const float motorH = 1.0f / 60.0f;
    auto motorConstraint = prepareMotorJointConstraint( motorJoint, motorBodyA, motorBodyB, motorH );
    bodyState motorStateA{};
    bodyState motorStateB{};
    solveMotorJointConstraint( motorConstraint, motorStateA, motorStateB );
    check( near( motorStateB.linearVelocity.x - motorStateA.linearVelocity.x, 2.0f ) && near( motorStateB.linearVelocity.y - motorStateA.linearVelocity.y, -1.0f ), "motor reaches desired relative linear velocity" );
    check( near( motorStateB.angularVelocity - motorStateA.angularVelocity, 3.0f ), "motor reaches desired relative angular velocity" );

    motorJoint.linearVelocity = { 10.0f, 0.0f };
    motorJoint.maxVelocityForce = 6.0f;
    motorJoint.angularVelocity = 10.0f;
    motorJoint.maxVelocityTorque = 12.0f;
    motorConstraint = prepareMotorJointConstraint( motorJoint, motorBodyA, motorBodyB, motorH );
    motorStateA = {};
    motorStateB = {};
    solveMotorJointConstraint( motorConstraint, motorStateA, motorStateB );
    check( near( Length( motorConstraint.linearVelocityImpulse ), motorJoint.maxVelocityForce * motorH ), "linear motor impulse is limited by max force times h" );
    check( near( std::abs( motorConstraint.angularVelocityImpulse ), motorJoint.maxVelocityTorque * motorH ), "angular motor impulse is limited by max torque times h" );

    return EXIT_SUCCESS;
}
