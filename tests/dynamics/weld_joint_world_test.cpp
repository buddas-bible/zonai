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

    weldJointDef definition{};
    definition.bodyA = ground;
    definition.bodyB = body;
    definition.localAnchorB = { -1.0f, 0.0f };
    const auto joint = simulation.createWeldJoint( definition );

    simulation.ApplyLinearImpulseToCenter( body, { 1.0f, 2.0f } );
    simulation.ApplyAngularImpulse( body, 0.5f );
    for( int i = 0; i < 180; ++i )
    {
        simulation.Step( 1.0f / 60.0f, 4 );
        const auto data = simulation.getWeldJointData( joint );
        check( Length( data.anchorB - data.anchorA ) < 0.015f, "Weld keeps off-center anchors together" );
        check( std::abs( data.currentAngle ) < 0.015f, "Weld keeps relative angle fixed" );
        check( IsFinite( data.force ) && std::isfinite( data.torque ), "Weld reaction remains finite" );
    }
    check( simulation.getJointCount() == 1 && simulation.GetBody( body ).jointCount == 1, "Weld shares common joint graph" );

    const auto settled = simulation.getWeldJointData( joint );
    check( LengthSquared( settled.force ) >= 0.0f && std::isfinite( settled.torque ), "Weld stores reaction query state" );
    simulation.SetShapeDensity( shape, 2.0f );
    auto data = simulation.getWeldJointData( joint );
    check( LengthSquared( data.force ) == 0.0f && data.torque == 0.0f, "mass change clears Weld cache" );

    const auto anchorBefore = data.anchorB;
    ( void )simulation.CreateShape( body, circle2{ { 1.0f, 0.0f }, 0.2f } );
    data = simulation.getWeldJointData( joint );
    check( Length( data.anchorB - anchorBefore ) < 0.00001f, "COM shift preserves origin-based Weld anchor" );
    check( LengthSquared( data.force ) == 0.0f && data.torque == 0.0f, "COM shift clears Weld cache" );

    simulation.SetBodyTransform( body, { { 1.0f, 0.0f }, {} } );
    data = simulation.getWeldJointData( joint );
    check( LengthSquared( data.force ) == 0.0f && data.torque == 0.0f, "pose change clears Weld cache" );

    simulation.destroyJoint( joint );
    check( !simulation.IsValid( joint ) && simulation.getJointCount() == 0, "destroy invalidates Weld handle" );
    const auto reused = simulation.createWeldJoint( definition );
    check( simulation.IsValid( reused ) && !simulation.IsValid( joint ), "Weld slot reuse preserves generation validation" );
    simulation.DestroyBody( body );
    check( !simulation.IsValid( reused ) && simulation.GetBody( ground ).jointCount == 0, "body destruction removes connected Weld" );

    world pair;
    pair.SetGravity( {} );
    const auto a = pair.CreateBody( bodyType::Dynamic );
    const auto b = pair.CreateBody( bodyType::Dynamic );
    ( void )pair.CreateShape( a, circle2{ {}, 0.5f } );
    ( void )pair.CreateShape( b, circle2{ {}, 0.5f } );
    definition = {};
    definition.bodyA = a;
    definition.bodyB = b;
    const auto pairJoint = pair.createWeldJoint( definition );
    pair.Step( 1.0f / 60.0f, 4 );
    check( pair.GetContactCount() == 0, "Weld connected collision suppression uses shared joint path" );
    pair.SetBodyAwake( a, false );
    check( !pair.IsBodyAwake( b ), "Weld connected dynamic bodies sleep together" );
    pair.ApplyAngularImpulse( b, 0.1f );
    check( pair.IsBodyAwake( a ) && pair.IsBodyAwake( b ), "Weld propagates wake" );
    for( int i = 0; i < 60; ++i ) pair.Step( 1.0f / 60.0f, 4 );
    data = pair.getWeldJointData( pairJoint );
    check( Length( data.anchorB - data.anchorA ) < 0.001f && std::abs( data.currentAngle ) < 0.001f, "dynamic Weld pair maintains relative transform" );
    pair.destroyJoint( pairJoint );
    pair.Step( 1.0f / 60.0f, 4 );
    check( pair.GetContactCount() == 1, "destroying Weld restores candidate contacts" );

    definition.collideConnected = true;
    const auto colliding = pair.createWeldJoint( definition );
    pair.Step( 1.0f / 60.0f, 4 );
    check( pair.GetContactCount() == 1 && pair.getWeldJointData( colliding ).collideConnected, "explicit Weld connected collisions remain enabled" );

    world foreign;
    check( !foreign.IsValid( colliding ), "foreign World rejects Weld handle" );

    world reference;
    reference.SetGravity( {} );
    const auto fixed = reference.CreateBody( bodyType::Static, { {}, rot2::FromRadians( 0.7f ) } );
    const auto welded = reference.CreateBody( bodyType::Dynamic, { {}, rot2::FromRadians( 1.0f ) } );
    ( void )reference.CreateShape( welded, circle2{ {}, 0.3f } );
    definition = {};
    definition.bodyA = fixed;
    definition.bodyB = welded;
    definition.referenceAngle = 0.3f;
    const auto referenceJoint = reference.createWeldJoint( definition );
    check( std::abs( reference.getWeldJointData( referenceJoint ).currentAngle ) < 0.00001f, "Weld query angle is relative to explicit reference" );
    reference.ApplyAngularImpulse( welded, reference.GetBodyRotationalInertia( welded ) );
    for( int i = 0; i < 120; ++i ) reference.Step( 1.0f / 60.0f, 4 );
    check( std::abs( reference.getWeldJointData( referenceJoint ).currentAngle ) < 0.01f, "Weld restores explicit reference angle" );

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
    const auto movingJoint = moving.createWeldJoint( definition );
    for( int i = 0; i < 120; ++i ) moving.Step( 1.0f / 60.0f, 4 );
    const auto movingData = moving.getWeldJointData( movingJoint );
    check( Length( movingData.anchorB - movingData.anchorA ) < 0.02f && std::abs( movingData.currentAngle ) < 0.02f, "dynamic follower tracks moving kinematic Weld frame" );
    check( moving.GetBodyLinearVelocity( driver ).x == drive.linearVelocity.x && moving.GetBodyAngularVelocity( driver ) == drive.angularVelocity, "Weld does not change prescribed kinematic motion" );

    return EXIT_SUCCESS;
}
