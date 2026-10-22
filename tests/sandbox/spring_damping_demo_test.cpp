#include "rigidBodyDemo.h"

#include <algorithm>
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

void checkSpringDampingDemo()
{
    for( int substeps : { 1, 4, 8 } )
    {
        demoSession playback{ createRigidBodyDemo };
        playback.selectDemo( demoKind::springDampingComparison );
        auto& scene = static_cast<rigidBodyDemo&>( playback.getDemo() );
        auto& physics = scene.getWorld();
        check( std::string_view{ getDemoEntry( demoKind::springDampingComparison ).category } == "조인트", "spring comparison retains the existing joint category" );
        check( scene.getShapes().size() == 3 && physics.GetBodyCount() == 4 && physics.getJointCount() == 3 && LengthSquared( physics.GetGravity() ) == 0.0f, "spring comparison creates three dynamic masses and distance springs without gravity" );
        const auto visuals = std::vector<visualShape>{ scene.getShapes().begin(), scene.getShapes().end() };
        const auto joints = std::vector<jointId>{ scene.getSpringComparisonJoints().begin(), scene.getSpringComparisonJoints().end() };
        const auto advance = [&]( int frames ) { for( int frame = 0; frame < frames; ++frame ) scene.step( 1.0f / 60.0f, substeps ); };
        for( std::size_t i = 0; i < 3; ++i )
        {
            const auto joint = physics.getDistanceJointData( joints[i] );
            check( joint.enableSpring && !joint.enableLimit && !joint.enableMotor && joint.length == 1.0f && joint.currentLength == 1.5f && physics.GetBodyMass( visuals[i].bodyHandle ) == physics.GetBodyMass( visuals[0].bodyHandle ) && LengthSquared( physics.GetBodyLinearVelocity( visuals[i].bodyHandle ) ) == 0.0f && !physics.IsBodySleepEnabled( visuals[i].bodyHandle ), "comparison isolates damping with equal resting stretched masses and active springs" );
        }
        std::array<float, 3> minimum{ 1.5f, 1.5f, 1.5f };
        std::array<float, 3> latePeak{};
        for( int frame = 0; frame < 120; ++frame )
        {
            advance( 1 );
            for( std::size_t i = 0; i < 3; ++i )
            {
                const float length = physics.getDistanceJointData( joints[i] ).currentLength;
                minimum[i] = std::min( minimum[i], length );
                if( frame >= 60 ) latePeak[i] = std::max( latePeak[i], std::abs( length - 1.0f ) );
            }
        }
        check( minimum[0] < 0.9f && minimum[2] > 0.99f, "low damping oscillates across the target while overdamping approaches without overshoot" );
        check( latePeak[0] > 0.01f && latePeak[0] > 5.0f * latePeak[1] && std::abs( physics.getDistanceJointData( joints[1] ).currentLength - 1.0f ) < 0.01f && std::abs( physics.getDistanceJointData( joints[2] ).currentLength - 1.0f ) < 0.02f, "damping reduces residual oscillation over the interval while damped masses approach the target" );
        physics.setDistanceJointSpring( joints[0], true, 2.0f, 2.0f );
        physics.setDistanceJointSpring( joints[1], true, 1.0f, 0.7f );
        physics.SetShapeDensity( visuals[1].shapeHandle, 2.0f );
        for( int repeat = 0; repeat < 2; ++repeat )
        {
            scene.resetCollisionTrial();
            check( physics.getDistanceJointData( joints[0] ).dampingRatio == 2.0f && physics.getDistanceJointData( joints[1] ).hertz == 1.0f && physics.GetShapeDensity( visuals[1].shapeHandle ) == 2.0f, "spring reset preserves independently edited damping frequency and density" );
            for( std::size_t i = 0; i < 3; ++i )
            {
                const auto body = visuals[i].bodyHandle;
                check( physics.IsValid( joints[i] ) && scene.getSpringComparisonJoints()[i] == joints[i] && physics.IsValid( visuals[i].shapeHandle ) && physics.GetBodyTransform( body ).position.x == -3.0f + 3.0f * static_cast<float>( i ) && physics.GetBodyTransform( body ).position.y == 0.0f && LengthSquared( physics.GetBodyLinearVelocity( body ) ) == 0.0f && physics.GetBodyAngularVelocity( body ) == 0.0f, "spring reset restores initial stretched positions and rest without replacing handles" );
            }
            advance( 15 );
            const auto a = physics.getDistanceJointData( joints[0] );
            const auto b = physics.getDistanceJointData( joints[1] );
            const auto c = physics.getDistanceJointData( joints[2] );
            check( a.currentLength > 1.0f && std::abs( a.currentLength - c.currentLength ) < 1.0e-4f, "editing damping reproduces the matching overdamped trajectory after every reset" );
            check( b.currentLength > 1.1f, "lowering frequency slows the initial spring return" );
        }
        physics.setDistanceJointSpring( joints[0], true, 0.0f, 2.0f );
        physics.SetBodyLinearDamping( visuals[0].bodyHandle, 0.2f );
        physics.ApplyForceToCenter( visuals[0].bodyHandle, { 100, 0 } );
        demoInput drag{};
        drag.mousePosition = physics.GetBodyTransform( visuals[0].bodyHandle ).position;
        drag.mousePressed = drag.mouseHeld = drag.right = true;
        scene.handleInput( drag );
        check( physics.IsValid( scene.getMouseJoint() ), "spring comparison retains the existing mouse drag interaction" );
        playback.setPlaying( true );
        scene.resetCollisionTrial();
        check( playback.isPlaying() && !physics.IsValid( scene.getMouseJoint() ) && physics.GetBodyLinearDamping( visuals[0].bodyHandle ) == 0.2f, "spring reset preserves playing state and body edits while cancelling drag" );
        playback.setPlaying( false );
        scene.resetCollisionTrial();
        advance( 30 );
        check( !playback.isPlaying() && physics.GetBodyTransform( visuals[0].bodyHandle ).position.x == -3.0f && physics.GetBodyTransform( visuals[0].bodyHandle ).position.y == 0.0f && LengthSquared( physics.GetBodyLinearVelocity( visuals[0].bodyHandle ) ) == 0.0f, "zero frequency removes spring force and reset clears held and accumulated forces while retaining pause" );
        playback.reset();
        auto& defaults = static_cast<rigidBodyDemo&>( playback.getDemo() );
        check( !playback.isPlaying() && !defaults.getWorld().IsValid( joints[0] ) && defaults.getWorld().getDistanceJointData( defaults.getSpringComparisonJoints()[0] ).hertz == 2.0f && defaults.getWorld().getDistanceJointData( defaults.getSpringComparisonJoints()[0] ).dampingRatio == 0.0f, "full scene reset restores default springs with fresh handles and pauses" );
    }
}
