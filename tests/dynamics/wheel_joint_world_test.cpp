#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
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
    for( const int subSteps : { 1, 4 } )
    {
        world simulation;
        const auto base = simulation.CreateBody();
        const auto wheel = simulation.CreateBody( bodyType::Dynamic, { { 0.0f, -0.3f }, {} } );
        const auto shape = simulation.CreateShape( wheel, circle2{ {}, 0.3f } );
        wheelJointDef definition{};
        definition.bodyA = base;
        definition.bodyB = wheel;
        const auto joint = simulation.createWheelJoint( definition );
        auto data = simulation.getWheelJointData( joint );
        check( data.enableSpring && data.hertz == 3.0f && data.dampingRatio == 0.7f && std::abs( data.currentTranslation + 0.3f ) < 0.00001f, "wheel defaults and signed origin translation" );
        simulation.ApplyAngularImpulse( wheel, simulation.GetBodyRotationalInertia( wheel ) * 2.0f );
        for( int i = 0; i < 180; ++i )
        {
            simulation.Step( 1.0f / 60.0f, subSteps );
            data = simulation.getWheelJointData( joint );
            check( std::abs( data.lateralError ) < 0.002f && IsFinite( data.force ), "suspension stays on line with finite reaction" );
        }
        check( data.currentTranslation < -0.01f && data.currentTranslation > -0.06f, "suspension settles with gravity deflection for one and four substeps" );
        const float weight = -simulation.GetGravity().y * simulation.GetBodyMass( wheel );
        check( std::abs( data.springForce - weight ) < weight * 0.02f && std::abs( data.force.y - weight ) < weight * 0.02f, "stored spring impulse balances gravity and reports force" );
        check( std::abs( simulation.GetBodyAngularVelocity( wheel ) - 2.0f ) < 0.00001f, "centered wheel remains free to rotate under suspension" );
        simulation.SetShapeDensity( shape, 2.0f );
        check( LengthSquared( simulation.getWheelJointData( joint ).force ) == 0.0f, "mass change clears both wheel caches" );
        const auto anchorBefore = simulation.getWheelJointData( joint ).anchorB;
        ( void )simulation.CreateShape( wheel, circle2{ { 0.4f, 0.0f }, 0.1f } );
        check( Length( simulation.getWheelJointData( joint ).anchorB - anchorBefore ) < 0.00001f, "COM shift preserves original wheel anchor" );
        simulation.Step( 1.0f / 60.0f, subSteps );
        simulation.SetBodyTransform( wheel, { { 0.0f, -0.2f }, {} } );
        check( LengthSquared( simulation.getWheelJointData( joint ).force ) == 0.0f, "pose change clears wheel cache" );
        simulation.SetBodyAwake( wheel, false );
        simulation.setWheelJointSpring( joint, true, 3.0f, 0.7f );
        check( !simulation.IsBodyAwake( wheel ), "identical spring settings preserve sleep" );
        const auto isolated = simulation.CreateBody( bodyType::Dynamic, { { 5.0f, 5.0f }, {} } );
        ( void )simulation.CreateShape( isolated, circle2{ {}, 0.1f } );
        simulation.SetBodyAwake( isolated, false );
        simulation.setWheelJointSpring( joint, true, 0.0f, 0.7f );
        check( simulation.IsBodyAwake( wheel ) && !simulation.IsBodyAwake( isolated ) && LengthSquared( simulation.getWheelJointData( joint ).force ) == 0.0f, "spring change clears coupled caches and wakes only connected component" );
        simulation.SetGravity( {} );
        simulation.SetBodyAngularVelocity( wheel, 0.0f );
        simulation.SetBodyLinearVelocity( wheel, { 0.0f, 1.0f } );
        for( int i = 0; i < 10; ++i ) simulation.Step( 1.0f / 60.0f, subSteps );
        check( simulation.GetContactCount() == 0, "free translation fixture has no external contact" );
        check( std::abs( simulation.GetBodyLinearVelocity( wheel ).y - 1.0f ) < 0.00001f && simulation.getWheelJointData( joint ).springForce == 0.0f, "zero Hertz restores free axial motion in World" );
        simulation.setWheelJointSpring( joint, false, 3.0f, 0.7f );
        simulation.Step( 1.0f / 60.0f, subSteps );
        check( std::abs( simulation.GetBodyLinearVelocity( wheel ).y - 1.0f ) < 0.00001f, "spring off preserves free axial motion" );
        simulation.destroyJoint( joint );
        const auto reused = simulation.createWheelJoint( definition );
        check( !simulation.IsValid( joint ) && simulation.IsValid( reused ) && LengthSquared( simulation.getWheelJointData( reused ).force ) == 0.0f, "wheel slot reuse resets cache and generation" );
        simulation.DestroyBody( wheel );
        check( !simulation.IsValid( reused ) && simulation.GetBody( base ).jointCount == 0, "body destruction removes wheel from shared joint graph" );
    }

    world pair;
    pair.SetGravity( {} );
    const auto a = pair.CreateBody( bodyType::Dynamic );
    const auto b = pair.CreateBody( bodyType::Dynamic );
    ( void )pair.CreateShape( a, circle2{ {}, 0.5f } );
    ( void )pair.CreateShape( b, circle2{ {}, 0.5f } );
    wheelJointDef definition{};
    definition.bodyA = a;
    definition.bodyB = b;
    const auto pairJoint = pair.createWheelJoint( definition );
    pair.Step( 1.0f / 60.0f, 4 );
    check( pair.GetContactCount() == 0, "connected wheel collision uses common exclusion" );
    pair.SetBodyAwake( a, false );
    check( !pair.IsBodyAwake( b ), "wheel pair shares sleep" );
    pair.setWheelJointSpring( pairJoint, true, 5.0f, 0.7f );
    check( pair.IsBodyAwake( a ) && pair.IsBodyAwake( b ), "spring setter wakes both connected dynamic bodies" );
    world foreign;
    check( !foreign.IsValid( pairJoint ), "foreign World rejects wheel ID" );
    pair.destroyJoint( pairJoint );
    pair.Step( 1.0f / 60.0f, 4 );
    check( pair.GetContactCount() == 1, "wheel deletion restores connected contacts" );

    world moving;
    moving.SetGravity( {} );
    bodyDef drive{};
    drive.type = bodyType::Kinematic;
    drive.linearVelocity = { 0.2f, 0.1f };
    drive.angularVelocity = 0.3f;
    const auto driver = moving.CreateBody( drive );
    const auto follower = moving.CreateBody( bodyType::Dynamic, { { 0.0f, 2.0f }, {} } );
    ( void )moving.CreateShape( follower, circle2{ {}, 0.3f } );
    definition = {};
    definition.bodyA = driver;
    definition.bodyB = follower;
    definition.enableSpring = false;
    const auto movingJoint = moving.createWheelJoint( definition );
    for( int i = 0; i < 120; ++i ) moving.Step( 1.0f / 60.0f, 4 );
    const auto data = moving.getWheelJointData( movingJoint );
    check( std::abs( data.lateralError ) < 0.02f && std::abs( data.axis.x ) > 0.5f, "wheel follows moving and rotating kinematic line" );
    check( moving.GetBodyLinearVelocity( driver ).x == drive.linearVelocity.x && moving.GetBodyAngularVelocity( driver ) == drive.angularVelocity, "wheel leaves prescribed kinematic motion unchanged" );

    return EXIT_SUCCESS;
}
