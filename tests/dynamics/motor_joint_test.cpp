#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "dynamics/joints/motorJointConstraint2.h"
#include "dynamics/world.h"

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

    // World가 Motor를 공용 Joint graph / solver 경로에 넣고 query까지 되돌려주는지 검증함.
    world simulation;
    simulation.SetGravity( {} );
    const auto ground = simulation.CreateBody();
    const auto driven = simulation.CreateBody( bodyType::Dynamic );
    ( void )simulation.CreateShape( driven, circle2{ {}, 0.5f } );

    motorJointDef definition{};
    definition.bodyA = ground;
    definition.bodyB = driven;
    definition.linearVelocity = { 1.5f, -0.5f };
    definition.maxVelocityForce = 1000.0f;
    definition.angularVelocity = 2.0f;
    definition.maxVelocityTorque = 1000.0f;
    const auto motorId = simulation.createMotorJoint( definition );
    const auto data = simulation.getMotorJointData( motorId );
    check( near( data.linearVelocity.x, definition.linearVelocity.x ) && near( data.angularVelocity, definition.angularVelocity ), "Motor create/query preserves target velocities" );

    simulation.Step( 1.0f / 60.0f, 1 );
    check( near( simulation.GetBodyLinearVelocity( driven ).x, definition.linearVelocity.x ) && near( simulation.GetBodyLinearVelocity( driven ).y, definition.linearVelocity.y ), "World Motor reaches target linear velocity" );
    check( near( simulation.GetBodyAngularVelocity( driven ), definition.angularVelocity ), "World Motor reaches target angular velocity" );

    return EXIT_SUCCESS;
}
