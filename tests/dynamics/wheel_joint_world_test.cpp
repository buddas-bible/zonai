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
    pair.SetBodyAwake( a, false );
    pair.setWheelJointLimit( pairJoint, true, -0.5f, 0.5f );
    check( pair.IsBodyAwake( a ) && pair.IsBodyAwake( b ), "limit setter wakes both connected dynamic bodies" );
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
    definition.enableLimit = true;
    definition.lowerTranslation = 1.5f;
    definition.upperTranslation = 2.5f;
    const auto movingJoint = moving.createWheelJoint( definition );
    for( int i = 0; i < 120; ++i ) moving.Step( 1.0f / 60.0f, 4 );
    const auto data = moving.getWheelJointData( movingJoint );
    check( std::abs( data.lateralError ) < 0.02f && std::abs( data.axis.x ) > 0.5f, "wheel follows moving and rotating kinematic line" );
    check( data.currentTranslation >= 1.48f && data.currentTranslation <= 2.52f, "translation range follows moving kinematic frame" );
    check( moving.GetBodyLinearVelocity( driver ).x == drive.linearVelocity.x && moving.GetBodyAngularVelocity( driver ) == drive.angularVelocity, "wheel leaves prescribed kinematic motion unchanged" );

    for( const int subSteps : { 1, 4 } )
    {
        world limited;
        const auto anchor = limited.CreateBody();
        const auto wheel = limited.CreateBody( bodyType::Dynamic );
        const auto shape = limited.CreateShape( wheel, circle2{ {}, 0.3f } );
        definition = {};
        definition.bodyA = anchor;
        definition.bodyB = wheel;
        definition.enableSpring = false;
        definition.enableLimit = true;
        definition.lowerTranslation = -0.5f;
        definition.upperTranslation = 0.5f;
        const auto joint = limited.createWheelJoint( definition );
        limited.SetBodyAngularVelocity( wheel, 2.0f );
        for( int i = 0; i < 240; ++i )
        {
            limited.Step( 1.0f / 60.0f, subSteps );
            const auto current = limited.getWheelJointData( joint );
            check( current.currentTranslation >= -0.515f && current.currentTranslation <= 0.515f && IsFinite( current.force ), "spring-off limit keeps gravity motion in range for one and four substeps" );
        }
        auto current = limited.getWheelJointData( joint );
        const float weight = -limited.GetGravity().y * limited.GetBodyMass( wheel );
        check( std::abs( current.limitForce - weight ) < weight * 0.03f && std::abs( current.force.y - current.limitForce ) < 0.00001f && current.springForce == 0.0f, "stored limit reaction balances gravity and contributes to total force" );
        check( limited.GetBodyAngularVelocity( wheel ) == 2.0f, "linear limit preserves free wheel rotation" );
        const float forceBefore = current.limitForce;
        limited.SetBodyAwake( wheel, false );
        limited.setWheelJointLimit( joint, true, -0.5f, 0.5f );
        check( !limited.IsBodyAwake( wheel ) && limited.getWheelJointData( joint ).limitForce == forceBefore, "identical limits preserve sleep and cached reaction" );
        const auto isolated = limited.CreateBody( bodyType::Dynamic, { { 5.0f, 5.0f }, {} } );
        ( void )limited.CreateShape( isolated, circle2{ {}, 0.1f } );
        limited.SetBodyAwake( isolated, false );
        limited.setWheelJointLimit( joint, false, -0.5f, 0.5f );
        check( limited.IsBodyAwake( wheel ) && !limited.IsBodyAwake( isolated ) && LengthSquared( limited.getWheelJointData( joint ).force ) == 0.0f, "limit change clears coupled caches and wakes only connected component" );
        for( int i = 0; i < 30; ++i ) limited.Step( 1.0f / 60.0f, subSteps );
        check( limited.getWheelJointData( joint ).currentTranslation < -1.0f && limited.getWheelJointData( joint ).limitForce == 0.0f, "disabling limit restores free gravity motion" );

        limited.SetGravity( {} );
        limited.SetBodyTransform( wheel, {} );
        limited.SetBodyLinearVelocity( wheel, { 0.0f, 10.0f } );
        limited.setWheelJointSpring( joint, true, 0.0f, 0.7f );
        limited.setWheelJointLimit( joint, true, -0.3f, 0.3f );
        for( int i = 0; i < 60; ++i ) limited.Step( 1.0f / 60.0f, subSteps );
        current = limited.getWheelJointData( joint );
        check( current.currentTranslation > 0.28f && current.currentTranslation < 0.315f && current.springForce == 0.0f, "zero Hertz limit stops positive axis impulse at upper boundary" );
        limited.SetBodyLinearVelocity( wheel, { 0.0f, -2.0f } );
        limited.Step( 1.0f / 60.0f, subSteps );
        check( limited.GetBodyLinearVelocity( wheel ).y < -1.9f, "upper boundary permits inward return" );
        limited.setWheelJointLimit( joint, true, 0.1f, 0.1f );
        for( int i = 0; i < 120; ++i ) limited.Step( 1.0f / 60.0f, subSteps );
        check( std::abs( limited.getWheelJointData( joint ).currentTranslation - 0.1f ) < 0.002f, "equal limit translations maintain chosen offset" );

        limited.SetBodyTransform( wheel, { { 0.0f, 0.2f }, {} } );
        limited.SetBodyLinearVelocity( wheel, {} );
        limited.setWheelJointLimit( joint, true, 0.1f, 0.4f );
        limited.setWheelJointSpring( joint, true, 3.0f, 0.7f );
        for( int i = 0; i < 240; ++i ) limited.Step( 1.0f / 60.0f, subSteps );
        current = limited.getWheelJointData( joint );
        check( current.currentTranslation >= 0.08f && current.currentTranslation <= 0.11f && current.springForce < 0.0f && current.limitForce > 0.0f, "lower limit resists spring pulling toward excluded neutral position" );
        check( std::abs( current.springForce + current.limitForce ) < std::abs( current.springForce ) * 0.03f, "separate spring and limit forces reveal opposing equilibrium reactions" );
        limited.setWheelJointSpring( joint, false, 3.0f, 0.7f );
        check( LengthSquared( limited.getWheelJointData( joint ).force ) == 0.0f, "spring change clears old limit reaction as well" );
        limited.SetGravity( { 0.0f, -10.0f } );
        for( int i = 0; i < 120; ++i ) limited.Step( 1.0f / 60.0f, subSteps );
        check( limited.getWheelJointData( joint ).limitForce > 0.0f, "limit cache is repopulated by gravity after settings change" );
        limited.SetShapeDensity( shape, 2.0f );
        check( limited.getWheelJointData( joint ).limitForce == 0.0f, "mass change clears limit caches" );
        limited.Step( 1.0f / 60.0f, subSteps );
        limited.SetBodyTransform( wheel, {} );
        check( limited.getWheelJointData( joint ).limitForce == 0.0f, "pose change clears limit caches" );
        limited.destroyJoint( joint );
        const auto reused = limited.createWheelJoint( definition );
        check( !limited.IsValid( joint ) && limited.getWheelJointData( reused ).limitForce == 0.0f, "reused Wheel slot discards old limit cache and handle" );
    }

    for( const int subSteps : { 1, 4 } )
    {
        world powered;
        const auto anchor = powered.CreateBody();
        const auto wheel = powered.CreateBody( bodyType::Dynamic, { { 0.0f, -0.3f }, {} } );
        const auto shape = powered.CreateShape( wheel, circle2{ {}, 0.3f } );
        definition = {};
        definition.bodyA = anchor;
        definition.bodyB = wheel;
        definition.enableLimit = true;
        definition.lowerTranslation = -0.5f;
        definition.upperTranslation = 0.5f;
        definition.enableMotor = true;
        definition.motorSpeed = 3.0f;
        definition.maxMotorTorque = 0.005f;
        const auto joint = powered.createWheelJoint( definition );
        powered.Step( 1.0f / 60.0f, subSteps );
        const float expectedSpeed = 0.005f / ( 60.0f * powered.GetBodyRotationalInertia( wheel ) );
        check( std::abs( powered.GetBodyAngularVelocity( wheel ) - expectedSpeed ) < expectedSpeed * 0.01f, "motor angular acceleration uses substep impulse budget for one and four substeps" );
        for( int i = 0; i < 180; ++i )
        {
            powered.Step( 1.0f / 60.0f, subSteps );
            const auto current = powered.getWheelJointData( joint );
            check( std::abs( current.motorTorque ) <= 0.005001f && std::abs( current.lateralError ) < 0.015f && current.currentTranslation >= -0.515f && current.currentTranslation <= 0.515f, "motor respects torque budget while suspension and limits retain wheel position" );
        }
        const float previousTorque = powered.getWheelJointData( joint ).motorTorque;
        powered.SetBodyAwake( wheel, false );
        powered.setWheelJointMotor( joint, true, 3.0f, 0.005f );
        check( !powered.IsBodyAwake( wheel ) && previousTorque > 0.004f && powered.getWheelJointData( joint ).motorTorque == previousTorque, "identical motor settings preserve sleep and torque cache" );
        const auto isolated = powered.CreateBody( bodyType::Dynamic, { { 5.0f, 5.0f }, {} } );
        ( void )powered.CreateShape( isolated, circle2{ {}, 0.1f } );
        powered.SetBodyAwake( isolated, false );
        powered.setWheelJointMotor( joint, true, 3.0f, 0.5f );
        check( powered.IsBodyAwake( wheel ) && !powered.IsBodyAwake( isolated ) && powered.getWheelJointData( joint ).motorTorque == 0.0f && LengthSquared( powered.getWheelJointData( joint ).force ) == 0.0f, "motor change clears coupled caches and wakes only connected component" );
        for( int i = 0; i < 60; ++i ) powered.Step( 1.0f / 60.0f, subSteps );
        check( std::abs( powered.GetBodyAngularVelocity( wheel ) - 3.0f ) < 0.001f, "centered wheel motor reaches target under suspension gravity load" );
        powered.setWheelJointMotor( joint, true, -3.0f, 0.5f );
        for( int i = 0; i < 60; ++i ) powered.Step( 1.0f / 60.0f, subSteps );
        check( std::abs( powered.GetBodyAngularVelocity( wheel ) + 3.0f ) < 0.001f, "wheel motor reverses rotation" );
        powered.setWheelJointMotor( joint, true, 0.0f, 0.5f );
        for( int i = 0; i < 60; ++i ) powered.Step( 1.0f / 60.0f, subSteps );
        check( std::abs( powered.GetBodyAngularVelocity( wheel ) ) < 0.001f, "zero target brakes wheel without fixing its angle" );
        powered.setWheelJointMotor( joint, true, 3.0f, 0.005f );
        powered.Step( 1.0f / 60.0f, subSteps );
        check( powered.getWheelJointData( joint ).motorTorque > 0.004f, "motor query reports stored angular impulse divided by h" );
        powered.setWheelJointLimit( joint, true, -0.4f, 0.4f );
        check( powered.getWheelJointData( joint ).motorTorque == 0.0f, "limit setting change clears coupled motor cache" );
        powered.Step( 1.0f / 60.0f, subSteps );
        powered.setWheelJointSpring( joint, true, 4.0f, 0.7f );
        check( powered.getWheelJointData( joint ).motorTorque == 0.0f, "spring setting change clears coupled motor cache" );
        powered.Step( 1.0f / 60.0f, subSteps );
        powered.SetShapeDensity( shape, 2.0f );
        check( powered.getWheelJointData( joint ).motorTorque == 0.0f, "inertia change clears motor cache" );
        powered.Step( 1.0f / 60.0f, subSteps );
        powered.SetBodyTransform( wheel, {} );
        check( powered.getWheelJointData( joint ).motorTorque == 0.0f, "pose change clears motor cache" );
        powered.setWheelJointMotor( joint, false, 3.0f, 0.5f );
        powered.SetBodyAngularVelocity( wheel, 2.0f );
        powered.Step( 1.0f / 60.0f, subSteps );
        check( powered.GetBodyAngularVelocity( wheel ) == 2.0f && powered.getWheelJointData( joint ).motorTorque == 0.0f, "motor off restores free rotation" );
        powered.destroyJoint( joint );
        const auto reused = powered.createWheelJoint( definition );
        check( !powered.IsValid( joint ) && powered.getWheelJointData( reused ).motorTorque == 0.0f, "reused slot discards motor cache and old handle" );
    }

    moving.setWheelJointMotor( movingJoint, true, 2.0f, 1.0f );
    for( int i = 0; i < 120; ++i ) moving.Step( 1.0f / 60.0f, 4 );
    check( moving.GetBodyAngularVelocity( driver ) == drive.angularVelocity && std::abs( moving.GetBodyAngularVelocity( follower ) - drive.angularVelocity - 2.0f ) < 0.001f, "motor targets relative speed while preserving prescribed kinematic rotation" );

    return EXIT_SUCCESS;
}
