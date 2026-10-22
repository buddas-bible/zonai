#include "rigidBodyDemo.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <string_view>

using namespace zonai;
using namespace zonai::sandbox;

namespace
{
void check( bool ok, const char* message )
{
    if( ok ) return;
    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}
}

void checkDistanceMotorForceDemo()
{
    for( int substeps : { 1, 4, 8 } )
    {
        demoSession playback{ createRigidBodyDemo };
        playback.selectDemo( demoKind::distanceMotorForceComparison );
        auto& scene = static_cast<rigidBodyDemo&>( playback.getDemo() );
        auto& physics = scene.getWorld();
        check( std::string_view{ getDemoEntry( demoKind::distanceMotorForceComparison ).category } == "조인트", "motor comparison retains the existing joint category" );
        check( scene.getShapes().size() == 3 && physics.GetBodyCount() == 4 && physics.getJointCount() == 3 && LengthSquared( physics.GetGravity() ) == 0.0f, "motor force comparison creates three equal resting masses and distance motors without gravity" );
        const auto visuals = std::vector<visualShape>{ scene.getShapes().begin(), scene.getShapes().end() };
        const auto joints = std::vector<jointId>{ scene.getMotorComparisonJoints().begin(), scene.getMotorComparisonJoints().end() };
        const auto advance = [&]( int frames )
        {
            for( int frame = 0; frame < frames; ++frame )
            {
                scene.step( 1.0f / 60.0f, substeps );
                for( const auto id : joints )
                {
                    const auto joint = physics.getDistanceJointData( id );
                    check( std::abs( joint.motorForce ) <= joint.maxMotorForce + 1.0e-5f, "distance motors never exceed their edited force limits" );
                }
            }
        };
        const auto speed = [&]( std::size_t i ) { return -physics.GetBodyLinearVelocity( visuals[i].bodyHandle ).y; };
        for( std::size_t i = 0; i < 3; ++i )
        {
            const auto joint = physics.getDistanceJointData( joints[i] );
            check( joint.enableSpring && joint.hertz == 0.0f && joint.enableMotor && !joint.enableLimit && joint.currentLength == 1.0f && physics.GetBodyMass( visuals[i].bodyHandle ) == physics.GetBodyMass( visuals[0].bodyHandle ) && !physics.IsBodySleepEnabled( visuals[i].bodyHandle ) && physics.GetBodyLinearDamping( visuals[i].bodyHandle ) == 0.0f, "comparison isolates motors with equal masses no spring force or body damping and no distance limits" );
        }
        advance( 15 );
        check( speed( 0 ) == 0.0f && speed( 1 ) > 0.08f && speed( 1 ) < 0.12f && std::abs( speed( 2 ) - 1.0f ) < 1.0e-4f, "zero force stays still while weak force accelerates and sufficient force tracks the target" );
        advance( 45 );
        check( speed( 0 ) == 0.0f && speed( 1 ) > 0.35f && speed( 1 ) < 0.45f && std::abs( speed( 2 ) - 1.0f ) < 1.0e-4f, "weak motor keeps accelerating below the common target while sufficient motor holds it" );
        physics.setDistanceJointMotor( joints[0], true, 1.0f, 2.0f );
        physics.setDistanceJointMotor( joints[2], true, 0.5f, 2.0f );
        physics.SetShapeDensity( visuals[1].shapeHandle, 2.0f );
        for( int repeat = 0; repeat < 2; ++repeat )
        {
            scene.resetCollisionTrial();
            check( physics.getDistanceJointData( joints[0] ).maxMotorForce == 2.0f && physics.getDistanceJointData( joints[2] ).motorSpeed == 0.5f && physics.GetShapeDensity( visuals[1].shapeHandle ) == 2.0f, "motor reset preserves edited force target speed and body density" );
            for( std::size_t i = 0; i < 3; ++i )
            {
                const auto body = visuals[i].bodyHandle;
                check( physics.IsValid( joints[i] ) && scene.getMotorComparisonJoints()[i] == joints[i] && physics.IsValid( visuals[i].shapeHandle ) && physics.GetBodyTransform( body ).position.x == -3.0f + 3.0f * static_cast<float>( i ) && physics.GetBodyTransform( body ).position.y == 0.5f && LengthSquared( physics.GetBodyLinearVelocity( body ) ) == 0.0f && physics.GetBodyAngularVelocity( body ) == 0.0f, "motor reset restores original positions and rest without replacing joint or shape handles" );
            }
            advance( 30 );
            check( std::abs( speed( 0 ) - 1.0f ) < 1.0e-4f && speed( 1 ) > 0.08f && speed( 1 ) < 0.12f && std::abs( speed( 2 ) - 0.5f ) < 1.0e-4f, "force and target edits change tracking while denser weak-motor mass accelerates more slowly" );
        }
        physics.setDistanceJointMotor( joints[2], true, -0.5f, 2.0f );
        scene.resetCollisionTrial();
        advance( 15 );
        check( std::abs( speed( 2 ) + 0.5f ) < 1.0e-4f, "negative motor target contracts the distance instead of extending it" );
        physics.setDistanceJointMotor( joints[0], true, 1.0f, 0.0f );
        physics.SetBodyLinearDamping( visuals[0].bodyHandle, 0.2f );
        physics.ApplyForceToCenter( visuals[0].bodyHandle, { 100, 0 } );
        demoInput drag{};
        drag.mousePosition = physics.GetBodyTransform( visuals[0].bodyHandle ).position;
        drag.mousePressed = drag.mouseHeld = drag.right = true;
        scene.handleInput( drag );
        check( physics.IsValid( scene.getMouseJoint() ), "motor comparison keeps existing mouse dragging" );
        playback.setPlaying( true );
        scene.resetCollisionTrial();
        check( playback.isPlaying() && !physics.IsValid( scene.getMouseJoint() ) && physics.GetBodyLinearDamping( visuals[0].bodyHandle ) == 0.2f, "motor reset preserves playing state and body edits while cancelling drag" );
        playback.setPlaying( false );
        scene.resetCollisionTrial();
        advance( 30 );
        check( !playback.isPlaying() && physics.GetBodyTransform( visuals[0].bodyHandle ).position.x == -3.0f && physics.GetBodyTransform( visuals[0].bodyHandle ).position.y == 0.5f && LengthSquared( physics.GetBodyLinearVelocity( visuals[0].bodyHandle ) ) == 0.0f, "zero motor force stays still after reset clears held and accumulated forces and retains pause" );
        playback.reset();
        auto& defaults = static_cast<rigidBodyDemo&>( playback.getDemo() );
        check( !playback.isPlaying() && !defaults.getWorld().IsValid( joints[0] ) && defaults.getWorld().getDistanceJointData( defaults.getMotorComparisonJoints()[0] ).maxMotorForce == 0.0f && defaults.getWorld().getDistanceJointData( defaults.getMotorComparisonJoints()[2] ).motorSpeed == 1.0f, "full scene reset restores default motors with fresh handles and pauses" );
    }
}
