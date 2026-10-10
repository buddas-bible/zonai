#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "dynamics/joints/moverJointConstraint2.h"

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

    bodySim bodyB{};
    bodyB.bodyId = 1;
    bodyB.invMass = 1.0f;
    bodyB.invInertia = 1.0f;

    moverJointSim2 joint{};
    joint.jointId = 0;
    joint.bodyIdA = 0;
    joint.bodyIdB = 1;
    joint.linearVelocity = { 2.0f, -1.0f };
    joint.maxVelocityForce = { 1000.0f, 1000.0f };

    const float h = 1.0f / 60.0f;
    auto constraint = prepareMoverJointConstraint( joint, bodyA, bodyB, h );
    bodyState stateA{};
    bodyState stateB{};

    solveMoverJointConstraint( constraint, stateA, stateB );

    check( near( stateB.linearVelocity.x - stateA.linearVelocity.x, 2.0f ) && near( stateB.linearVelocity.y - stateA.linearVelocity.y, -1.0f ), "Mover reaches desired relative linear velocity" );
    check( near( stateA.angularVelocity, 0.0f ) && near( stateB.angularVelocity, 0.0f ), "Mover does not affect rotation" );

    return EXIT_SUCCESS;
}
