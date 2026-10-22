#include "rigidBodyDemo.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
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

void checkDistanceLimitMotorDemo()
{
    for( int substeps : { 1, 4, 8 } )
    {
        demoSession playback{ createRigidBodyDemo };
        playback.selectDemo( demoKind::distanceLimitMotorComparison );
        auto& scene = static_cast<rigidBodyDemo&>( playback.getDemo() );
        auto& physics = scene.getWorld();
        check( scene.getShapes().size() == 2 && physics.GetBodyCount() == 3 && physics.getJointCount() == 2 && LengthSquared( physics.GetGravity() ) == 0.0f, "limit motor comparison creates two equal resting masses without gravity" );
        const auto a = scene.getShapes()[0], b = scene.getShapes()[1];
        const std::array joints{ scene.getLimitComparisonJoints()[0], scene.getLimitComparisonJoints()[1] };
        const auto advance = [&]( int frames ) { for( int frame = 0; frame < frames; ++frame ) scene.step( 1.0f / 60.0f, substeps ); };
        const auto data = [&]( int i ) { return physics.getDistanceJointData( joints[i] ); };
        const auto speed = [&]( int i ) { return -physics.GetBodyLinearVelocity( scene.getShapes()[i].bodyHandle ).y; };
        for( int i = 0; i < 2; ++i )
        {
            const auto joint = data( i );
            check( joint.currentLength == 1.0f && joint.enableSpring && joint.hertz == 0.0f && joint.enableMotor && joint.motorSpeed == 1.0f && joint.maxMotorForce == 2.0f && joint.minLength == 0.75f && joint.maxLength == 1.5f && physics.GetBodyMass( a.bodyHandle ) == physics.GetBodyMass( b.bodyHandle ), "comparison isolates equal motors with no spring force and common distance range" );
        }
        advance( 60 );
        check( data( 0 ).currentLength > 1.95f && speed( 0 ) > 0.99f && std::abs( data( 1 ).currentLength - 1.5f ) < 0.005f && std::abs( speed( 1 ) ) < 0.001f, "unlimited motor extends while enabled upper limit stops the same motor" );
        for( const auto id : joints ) physics.setDistanceJointMotor( id, true, -1.0f, 2.0f );
        advance( 6 );
        check( data( 1 ).currentLength < 1.45f && speed( 1 ) < -0.99f, "reversing target releases the upper limit immediately" );
        advance( 54 );
        check( data( 0 ).currentLength > 1.0f && data( 0 ).currentLength < 1.15f && speed( 0 ) < -0.99f && std::abs( data( 1 ).currentLength - 0.75f ) < 0.005f && std::abs( speed( 1 ) ) < 0.001f, "enabled lower limit stops contraction while unlimited motor continues" );
        physics.setDistanceJointMotor( joints[1], true, 1.0f, 2.0f );
        advance( 6 );
        check( data( 1 ).currentLength > 0.8f && speed( 1 ) > 0.99f, "positive target releases the lower limit" );
        physics.setDistanceJointLimit( joints[0], true, 0.5f, 1.25f );
        physics.setDistanceJointLimit( joints[1], false, 0.75f, 1.75f );
        for( const auto id : joints ) physics.setDistanceJointMotor( id, true, 0.5f, 2.0f );
        physics.SetShapeDensity( b.shapeHandle, 2.0f );
        for( int repeat = 0; repeat < 2; ++repeat )
        {
            playback.setPlaying( repeat == 0 );
            scene.resetCollisionTrial();
            check( playback.isPlaying() == ( repeat == 0 ) && data( 0 ).enableLimit && !data( 1 ).enableLimit && data( 0 ).minLength == 0.5f && data( 0 ).maxLength == 1.25f && data( 1 ).maxLength == 1.75f && data( 1 ).motorSpeed == 0.5f && physics.GetShapeDensity( b.shapeHandle ) == 2.0f, "trial reset preserves edited limits speed density and playback state" );
            for( int i = 0; i < 2; ++i )
            {
                const auto visual = scene.getShapes()[i];
                check( scene.getLimitComparisonJoints()[i] == joints[i] && physics.IsValid( visual.shapeHandle ) && physics.GetBodyTransform( visual.bodyHandle ).position.x == -2.0f + 4.0f * static_cast<float>( i ) && physics.GetBodyTransform( visual.bodyHandle ).position.y == 0.5f && LengthSquared( physics.GetBodyLinearVelocity( visual.bodyHandle ) ) == 0.0f && physics.GetBodyAngularVelocity( visual.bodyHandle ) == 0.0f, "trial reset preserves handles and restores original poses and rest" );
            }
            advance( 60 );
            check( std::abs( data( 0 ).currentLength - 1.25f ) < 0.005f && std::abs( speed( 0 ) ) < 0.001f && data( 1 ).currentLength > 1.45f && std::abs( speed( 1 ) - 0.5f ) < 0.001f, "edited enable bounds and speed change repeated trial results" );
        }
        physics.setDistanceJointLimit( joints[1], true, 0.9f, 1.75f );
        physics.setDistanceJointMotor( joints[1], true, -0.5f, 2.0f );
        scene.resetCollisionTrial();
        advance( 60 );
        check( std::abs( data( 1 ).currentLength - 0.9f ) < 0.005f && std::abs( speed( 1 ) ) < 0.001f, "edited lower bound stops the edited negative target" );
        playback.reset();
        auto& defaults = static_cast<rigidBodyDemo&>( playback.getDemo() );
        const auto restored = defaults.getWorld().getDistanceJointData( defaults.getLimitComparisonJoints()[1] );
        check( !playback.isPlaying() && !defaults.getWorld().IsValid( joints[0] ) && restored.enableLimit && restored.minLength == 0.75f && restored.maxLength == 1.5f && restored.motorSpeed == 1.0f, "full reset restores default limits and motors with fresh handles" );
        check( std::string_view{ getDemoEntry( demoKind::distanceLimitMotorComparison ).category } == "조인트", "limit comparison retains joint category" );
    }
}
