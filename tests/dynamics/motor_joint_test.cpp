#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "dynamics/joints/motorJointConstraint2.h"

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
    bodySim bodyA{};
    bodyA.bodyId = 0;
    bodyA.invMass = 1.0f;
    bodyA.invInertia = 1.0f;

    bodySim bodyB{};
    bodyB.bodyId = 1;
    bodyB.invMass = 1.0f;
    bodyB.invInertia = 1.0f;

    motorJointSim2 joint{};
    joint.jointId = 0;
    joint.bodyIdA = 0;
    joint.bodyIdB = 1;
    joint.linearVelocity = { 2.0f, -1.0f };
    joint.maxVelocityForce = 1000.0f;
    joint.angularVelocity = 3.0f;
    joint.maxVelocityTorque = 1000.0f;

    const float h = 1.0f / 60.0f;
    auto constraint = prepareMotorJointConstraint( joint, bodyA, bodyB, h );
    bodyState stateA{};
    bodyState stateB{};

    solveMotorJointConstraint( constraint, stateA, stateB );
    check( near( stateB.linearVelocity.x - stateA.linearVelocity.x, 2.0f ) && near( stateB.linearVelocity.y - stateA.linearVelocity.y, -1.0f ), "motor reaches desired relative linear velocity" );
    check( near( stateB.angularVelocity - stateA.angularVelocity, 3.0f ), "motor reaches desired relative angular velocity" );

    joint.linearVelocity = { 10.0f, 0.0f };
    joint.maxVelocityForce = 6.0f;
    joint.angularVelocity = 10.0f;
    joint.maxVelocityTorque = 12.0f;
    constraint = prepareMotorJointConstraint( joint, bodyA, bodyB, h );
    stateA = {};
    stateB = {};
    solveMotorJointConstraint( constraint, stateA, stateB );
    check( near( Length( constraint.linearVelocityImpulse ), joint.maxVelocityForce * h ), "linear motor impulse is limited by max force times h" );
    check( near( std::abs( constraint.angularVelocityImpulse ), joint.maxVelocityTorque * h ), "angular motor impulse is limited by max torque times h" );

    return EXIT_SUCCESS;
}
