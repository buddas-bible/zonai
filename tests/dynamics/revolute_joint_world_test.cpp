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

}

int main()
{
    world simulation;
    simulation.SetGravity( {} );
    const auto ground = simulation.CreateBody();
    const auto body = simulation.CreateBody( bodyType::Dynamic, { { 1.0f, 0.0f }, {} } );
    const auto shape = simulation.CreateShape( body, circle2{ {}, 0.3f } );
    revoluteJointDef definition{};
    definition.bodyA = ground;
    definition.bodyB = body;
    definition.localAnchorB = { -1.0f, 0.0f };
    const auto joint = simulation.createRevoluteJoint( definition );
    simulation.ApplyAngularImpulse( body, 0.1f );
    for( int i = 0; i < 180; ++i )
    {
        simulation.Step( 1.0f / 60.0f, 4 );
        const auto data = simulation.getRevoluteJointData( joint );
        check( Length( data.anchorB - data.anchorA ) < 0.015f, "off-center anchor stays pinned while body rotates" );
        check( IsFinite( data.force ), "reaction force remains finite" );
    }
    check( std::abs( simulation.GetBodyTransform( body ).rotation.s ) > 0.01f, "hinge permits actual body rotation" );
    check( simulation.getJointCount() == 1 && simulation.GetBody( body ).jointCount == 1, "hinge shares joint graph" );
    check( LengthSquared( simulation.getRevoluteJointData( joint ).force ) > 0.0f, "impulse cache is stored for force query" );
    simulation.SetShapeDensity( shape, 2.0f );
    check( LengthSquared( simulation.getRevoluteJointData( joint ).force ) == 0.0f, "mass change clears hinge cache" );
    const auto anchorBefore = simulation.getRevoluteJointData( joint ).anchorB;
    ( void )simulation.CreateShape( body, circle2{ { 1.0f, 0.0f }, 0.2f } );
    check( Length( simulation.getRevoluteJointData( joint ).anchorB - anchorBefore ) < 0.00001f, "COM shift preserves origin-based anchor" );
    check( LengthSquared( simulation.getRevoluteJointData( joint ).force ) == 0.0f, "COM shift clears hinge cache" );
    simulation.Step( 1.0f / 60.0f, 4 );
    check( Length( simulation.getRevoluteJointData( joint ).anchorB - simulation.getRevoluteJointData( joint ).anchorA ) < 0.015f, "shifted COM still uses correct lever arm" );
    simulation.SetBodyTransform( body, { { 1.0f, 0.0f }, {} } );
    check( LengthSquared( simulation.getRevoluteJointData( joint ).force ) == 0.0f, "pose change clears hinge cache" );
    simulation.destroyJoint( joint );
    check( !simulation.IsValid( joint ) && simulation.getJointCount() == 0, "destroy invalidates hinge handle" );
    const auto reused = simulation.createRevoluteJoint( definition );
    check( simulation.IsValid( reused ) && !simulation.IsValid( joint ), "slot reuse preserves generation validation" );
    simulation.DestroyBody( body );
    check( !simulation.IsValid( reused ) && simulation.GetBody( ground ).jointCount == 0, "body destruction removes connected hinge" );

    world pair;
    pair.SetGravity( {} );
    const auto a = pair.CreateBody( bodyType::Dynamic );
    const auto b = pair.CreateBody( bodyType::Dynamic );
    ( void )pair.CreateShape( a, circle2{ {}, 0.5f } );
    ( void )pair.CreateShape( b, circle2{ {}, 0.5f } );
    definition = {};
    definition.bodyA = a;
    definition.bodyB = b;
    const auto pairJoint = pair.createRevoluteJoint( definition );
    pair.Step( 1.0f / 60.0f, 4 );
    check( pair.GetContactCount() == 0, "connected collision suppression uses shared joint path" );
    pair.SetBodyAwake( a, false );
    check( !pair.IsBodyAwake( b ), "connected dynamic bodies sleep together" );
    pair.ApplyAngularImpulse( b, 0.1f );
    check( pair.IsBodyAwake( a ) && pair.IsBodyAwake( b ), "hinge propagates wake" );
    pair.Step( 1.0f / 60.0f, 4 );
    check( Length( pair.getRevoluteJointData( pairJoint ).anchorB - pair.getRevoluteJointData( pairJoint ).anchorA ) < 0.001f, "dynamic pair maintains shared center" );
    pair.destroyJoint( pairJoint );
    pair.Step( 1.0f / 60.0f, 4 );
    check( pair.GetContactCount() == 1, "destroying hinge restores candidate contacts" );
    definition.collideConnected = true;
    const auto colliding = pair.createRevoluteJoint( definition );
    pair.Step( 1.0f / 60.0f, 4 );
    check( pair.GetContactCount() == 1 && pair.getRevoluteJointData( colliding ).collideConnected, "explicit connected collisions remain enabled" );

    world foreign;
    check( !foreign.IsValid( colliding ), "foreign World rejects hinge handle" );

    world moving;
    moving.SetGravity( {} );
    bodyDef drive{};
    drive.type = bodyType::Kinematic;
    drive.linearVelocity = { 0.2f, 0.1f };
    drive.angularVelocity = 0.3f;
    const auto driver = moving.CreateBody( drive );
    const auto follower = moving.CreateBody( bodyType::Dynamic, { { 0.0f, -1.0f }, {} } );
    ( void )moving.CreateShape( follower, circle2{ {}, 0.3f } );
    definition = {};
    definition.bodyA = driver;
    definition.bodyB = follower;
    definition.localAnchorB = { 0.0f, 1.0f };
    const auto movingJoint = moving.createRevoluteJoint( definition );
    for( int i = 0; i < 120; ++i )
    {
        moving.Step( 1.0f / 60.0f, 4 );
    }
    const auto movingData = moving.getRevoluteJointData( movingJoint );
    check( Length( movingData.anchorB - movingData.anchorA ) < 0.02f, "dynamic follower tracks moving kinematic anchor" );
    check( moving.GetBodyLinearVelocity( driver ).x == drive.linearVelocity.x && moving.GetBodyAngularVelocity( driver ) == drive.angularVelocity, "hinge does not change prescribed kinematic motion" );

    return EXIT_SUCCESS;
}
