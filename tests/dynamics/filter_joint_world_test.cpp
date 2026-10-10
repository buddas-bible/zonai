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

    // 기본 Filter는 기존 Contact를 즉시 제거하고 이후 후보도 막지만 solver impulse는 만들지 않음.
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
    check( simulation.getJointCount() == 1 && simulation.GetBody( bodyA ).jointCount == 1 && simulation.GetBody( bodyB ).jointCount == 1, "Filter Joint uses the common joint graph" );

    const filterJointData data = simulation.getFilterJointData( filter );
    check( data.bodyA == bodyA && data.bodyB == bodyB && !data.collideConnected, "Filter query preserves endpoints and disables connected collision by default" );

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
    check( !simulation.IsValid( filter ) && simulation.getJointCount() == 0, "destroy invalidates Filter handle" );
    simulation.Step( h, 1 );
    check( simulation.GetContactCount() == 1, "destroying Filter Joint restores collision candidates" );

    // 여러 non-collide Joint가 같은 Body pair를 막는 경우 마지막 하나가 사라질 때까지 collision을 복구하면 안 됨.
    const jointId filterA = simulation.createFilterJoint( definition );
    const jointId filterB = simulation.createFilterJoint( definition );
    check( simulation.GetContactCount() == 0 && simulation.getJointCount() == 2, "multiple Filter Joints may block the same body pair" );
    simulation.destroyJoint( filterA );
    simulation.Step( h, 1 );
    check( simulation.GetContactCount() == 0 && simulation.IsValid( filterB ), "remaining Filter Joint keeps collision suppressed after one destroy" );

    simulation.destroyJoint( filterB );
    simulation.Step( h, 1 );
    check( simulation.GetContactCount() == 1, "collision returns only after the last blocking Filter is destroyed" );

    // collideConnected=true는 collision을 건드리지 않고 Joint graph / sleep 연결만 제공함.
    definition.collideConnected = true;
    const jointId collidingFilter = simulation.createFilterJoint( definition );
    simulation.Step( h, 1 );
    check( simulation.GetContactCount() == 1 && simulation.getFilterJointData( collidingFilter ).collideConnected, "collideConnected Filter preserves contacts" );

    simulation.SetBodyAwake( bodyA, false );
    check( !simulation.IsBodyAwake( bodyA ) && !simulation.IsBodyAwake( bodyB ), "Filter Joint couples connected dynamic bodies for sleeping" );
    simulation.SetBodyAwake( bodyB, true );
    check( simulation.IsBodyAwake( bodyA ) && simulation.IsBodyAwake( bodyB ), "Filter Joint propagates wake through the joint graph" );

    world foreign;
    check( !foreign.IsValid( collidingFilter ), "foreign World rejects Filter handle" );

    // Slot 재사용은 generation을 갱신하고 body 파괴는 연결 Filter를 함께 정리함.
    simulation.destroyJoint( collidingFilter );
    const jointId reused = simulation.createFilterJoint( definition );
    check( simulation.IsValid( reused ) && !simulation.IsValid( collidingFilter ), "Filter slot reuse preserves generation validation" );
    simulation.DestroyBody( bodyB );
    check( !simulation.IsValid( reused ) && simulation.GetBody( bodyA ).jointCount == 0 && simulation.getJointCount() == 0, "body destruction removes connected Filter Joint" );

    return EXIT_SUCCESS;
}
