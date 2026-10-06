#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numbers>
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

    for( const int subSteps : { 1, 4 } )
    {
        world limits;
        limits.SetGravity( {} );
        const auto fixed = limits.CreateBody( bodyType::Static, { {}, rot2::FromRadians( 0.7f ) } );
        const auto rotating = limits.CreateBody( bodyType::Dynamic, { {}, rot2::FromRadians( 1.0f ) } );
        const auto rotatingShape = limits.CreateShape( rotating, circle2{ {}, 0.3f } );
        definition = {};
        definition.bodyA = fixed;
        definition.bodyB = rotating;
        definition.referenceAngle = 0.3f;
        definition.enableLimit = true;
        definition.lowerAngle = -0.4f;
        definition.upperAngle = 0.4f;
        const auto limitedJoint = limits.createRevoluteJoint( definition );
        check( std::abs( limits.getRevoluteJointData( limitedJoint ).currentAngle ) < 0.00001f, "query angle is relative to explicit reference" );
        const float appliedTorque = limits.GetBodyRotationalInertia( rotating ) * 2.0f;
        for( int i = 0; i < 240; ++i )
        {
            limits.ApplyTorque( rotating, appliedTorque );
            limits.Step( 1.0f / 60.0f, subSteps );
        }
        const auto upper = limits.getRevoluteJointData( limitedJoint );
        check( upper.currentAngle >= 0.39f && upper.currentAngle < 0.41f, "upper limit holds under sustained torque for one and four substeps" );
        check( std::abs( upper.torque + appliedTorque ) < appliedTorque * 0.02f, "stored angular impulse reports opposing reaction torque" );
        limits.ApplyAngularImpulse( rotating, -limits.GetBodyRotationalInertia( rotating ) );
        for( int i = 0; i < 12; ++i ) limits.Step( 1.0f / 60.0f, subSteps );
        check( limits.getRevoluteJointData( limitedJoint ).currentAngle < upper.currentAngle - 0.1f, "upper boundary permits inward return in World" );
        for( int i = 0; i < 240; ++i )
        {
            limits.ApplyTorque( rotating, -appliedTorque );
            limits.Step( 1.0f / 60.0f, subSteps );
        }
        const auto lower = limits.getRevoluteJointData( limitedJoint );
        check( lower.currentAngle <= -0.39f && lower.currentAngle > -0.41f && lower.torque > 0.0f, "lower limit holds and reports positive restoring torque" );

        limits.SetShapeDensity( rotatingShape, 2.0f );
        check( limits.getRevoluteJointData( limitedJoint ).torque == 0.0f, "mass change clears angular cache" );
        limits.ApplyTorque( rotating, -appliedTorque );
        limits.Step( 1.0f / 60.0f, subSteps );
        limits.SetBodyTransform( rotating, { {}, rot2::FromRadians( 1.0f ) } );
        check( limits.getRevoluteJointData( limitedJoint ).torque == 0.0f, "pose change clears angular cache" );
        limits.SetBodyAwake( rotating, false );
        limits.setRevoluteJointLimit( limitedJoint, true, -0.4f, 0.4f );
        check( !limits.IsBodyAwake( rotating ), "unchanged limit setter preserves sleep" );
        const auto isolated = limits.CreateBody( bodyType::Dynamic );
        ( void )limits.CreateShape( isolated, circle2{ {}, 0.2f } );
        limits.SetBodyAwake( isolated, false );
        limits.setRevoluteJointLimit( limitedJoint, true, 0.2f, 0.2f );
        check( limits.IsBodyAwake( rotating ) && !limits.IsBodyAwake( isolated ), "changed range wakes connected body without crossing static boundary" );
        for( int i = 0; i < 120; ++i ) limits.Step( 1.0f / 60.0f, subSteps );
        check( std::abs( limits.getRevoluteJointData( limitedJoint ).currentAngle - 0.2f ) < 0.005f, "equal limits lock the requested angle" );
        limits.setRevoluteJointLimit( limitedJoint, false, -0.4f, 0.4f );
        check( limits.getRevoluteJointData( limitedJoint ).torque == 0.0f, "disabling limit clears stored torque" );
        limits.ApplyAngularImpulse( rotating, limits.GetBodyRotationalInertia( rotating ) );
        for( int i = 0; i < 60; ++i ) limits.Step( 1.0f / 60.0f, subSteps );
        check( limits.getRevoluteJointData( limitedJoint ).currentAngle > 0.8f, "disabled limit restores free rotation" );
        limits.setRevoluteJointLimit( limitedJoint, true, -10.0f, 10.0f );
        const auto clamped = limits.getRevoluteJointData( limitedJoint );
        check( clamped.lowerAngle == -0.99f * std::numbers::pi_v<float> && clamped.upperAngle == 0.99f * std::numbers::pi_v<float>, "limit range stays away from wrapped angle branch cut" );
    }

    return EXIT_SUCCESS;
}
