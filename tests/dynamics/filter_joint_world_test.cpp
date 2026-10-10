#include <cmath>
#include <cstdio>
#include <cstdlib>

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

bool near( float a, float b, float epsilon = 0.0001f )
{
    return std::abs( a - b ) < epsilon;
}

}

int main()
{
    constexpr float h = 1.0f / 60.0f;

    world simulation;
    simulation.SetGravity( {} );

    const bodyId bodyA = simulation.CreateBody( bodyType::Dynamic );
    const bodyId bodyB = simulation.CreateBody( bodyType::Dynamic );
    ( void )simulation.CreateShape( bodyA, circle2{ {}, 0.5f } );
    ( void )simulation.CreateShape( bodyB, circle2{ {}, 0.5f } );

    simulation.Step( h, 1 );
    check( simulation.GetContactCount() == 1, "overlapping bodies initially create a Contact" );

    filterJointDef definition{};
    definition.bodyA = bodyA;
    definition.bodyB = bodyB;

    const jointId filter = simulation.createFilterJoint( definition );
    check( simulation.GetContactCount() == 0, "creating Filter Joint immediately removes existing contacts" );
    check( simulation.getJointCount() == 1, "Filter Joint uses the common joint lifecycle" );

    const filterJointData data = simulation.getFilterJointData( filter );
    check( !data.collideConnected, "Filter Joint disables connected-body collision by default" );

    simulation.SetBodyLinearVelocity( bodyA, { -1.0f, 0.0f } );
    simulation.SetBodyLinearVelocity( bodyB, { 2.0f, 0.0f } );
    simulation.Step( h, 1 );
    check( simulation.GetContactCount() == 0, "Filter Joint keeps future contacts suppressed" );
    check( near( simulation.GetBodyLinearVelocity( bodyA ).x, -1.0f ) && near( simulation.GetBodyLinearVelocity( bodyB ).x, 2.0f ), "Filter Joint does not solve a velocity constraint" );

    simulation.SetBodyTransform( bodyA, {} );
    simulation.SetBodyTransform( bodyB, {} );
    simulation.SetBodyLinearVelocity( bodyA, {} );
    simulation.SetBodyLinearVelocity( bodyB, {} );
    simulation.destroyJoint( filter );
    simulation.Step( h, 1 );
    check( simulation.GetContactCount() == 1, "destroying Filter Joint restores collision candidates" );

    return EXIT_SUCCESS;
}
