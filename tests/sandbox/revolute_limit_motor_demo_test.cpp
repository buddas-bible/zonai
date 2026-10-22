#include "rigidBodyDemo.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numbers>
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

void checkRevoluteLimitMotorDemo()
{
    constexpr float boundary = 0.25f * std::numbers::pi_v<float>;
    for( int substeps : { 1, 4, 8 } )
    {
        demoSession playback{ createRigidBodyDemo };
        playback.selectDemo( demoKind::revoluteLimitMotorComparison );
        auto& scene = static_cast<rigidBodyDemo&>( playback.getDemo() );
        auto& physics = scene.getWorld();
        check( scene.getShapes().size() == 2 && physics.GetBodyCount() == 3 && physics.getJointCount() == 2 && LengthSquared( physics.GetGravity() ) == 0.0f, "angle limit comparison creates two resting center hinged rods without gravity" );
        const std::array visuals{ scene.getShapes()[0], scene.getShapes()[1] };
        const std::array joints{ scene.getAngleLimitComparisonJoints()[0], scene.getAngleLimitComparisonJoints()[1] };
        const auto data = [&]( int i ) { return physics.getRevoluteJointData( joints[i] ); };
        const auto speed = [&]( int i ) { return physics.GetBodyAngularVelocity( visuals[i].bodyHandle ); };
        const auto advance = [&]( int frames )
        {
            for( int frame = 0; frame < frames; ++frame )
            {
                scene.step( 1.0f / 60.0f, substeps );
                for( int i = 0; i < 2; ++i )
                {
                    const auto joint = data( i );
                    check( std::abs( joint.motorTorque ) <= joint.maxMotorTorque + 1.0e-5f && LengthSquared( joint.anchorB - joint.anchorA ) < 1.0e-8f && LengthSquared( physics.GetBodyLinearVelocity( visuals[i].bodyHandle ) ) < 1.0e-8f, "limit comparison retains torque cap and fixed center hinges" );
                }
            }
        };
        for( int i = 0; i < 2; ++i )
        {
            const auto joint = data( i );
            check( joint.currentAngle == 0.0f && joint.referenceAngle == 0.0f && joint.enableMotor && joint.motorSpeed == 2.0f && joint.maxMotorTorque == 0.2f && joint.lowerAngle == -boundary && joint.upperAngle == boundary && physics.GetBodyRotationalInertia( visuals[i].bodyHandle ) == physics.GetBodyRotationalInertia( visuals[0].bodyHandle ) && !physics.IsBodySleepEnabled( visuals[i].bodyHandle ) && physics.GetBodyAngularDamping( visuals[i].bodyHandle ) == 0.0f, "comparison isolates equal motors and inertia with common angle range without damping or sleep" );
        }
        advance( 60 );
        check( data( 0 ).currentAngle > 1.8f && speed( 0 ) > 1.99f && std::abs( data( 1 ).currentAngle - boundary ) < 0.02f && std::abs( speed( 1 ) ) < 0.001f, "unlimited motor rotates while enabled upper angle limit stops the same motor" );
        for( const auto id : joints ) physics.setRevoluteJointMotor( id, true, -2.0f, 0.2f );
        advance( 15 );
        check( data( 1 ).currentAngle < 0.5f && speed( 1 ) < -1.99f, "reversing target releases the upper angle limit" );
        advance( 45 );
        check( data( 0 ).currentAngle > 0.4f && data( 0 ).currentAngle < 0.8f && speed( 0 ) < -1.99f && std::abs( data( 1 ).currentAngle + boundary ) < 0.02f && std::abs( speed( 1 ) ) < 0.001f, "enabled lower angle limit stops negative rotation while unlimited motor keeps rotating" );
        physics.setRevoluteJointMotor( joints[1], true, 2.0f, 0.2f );
        advance( 15 );
        check( data( 1 ).currentAngle > -0.5f && speed( 1 ) > 1.99f, "positive target releases the lower angle limit" );
        physics.setRevoluteJointLimit( joints[0], true, -0.5f, 0.5f );
        physics.setRevoluteJointLimit( joints[1], false, -0.75f, 1.0f );
        for( const auto id : joints ) physics.setRevoluteJointMotor( id, true, 1.0f, 0.2f );
        physics.SetShapeDensity( visuals[1].shapeHandle, 2.0f );
        for( int repeat = 0; repeat < 2; ++repeat )
        {
            playback.setPlaying( repeat == 0 );
            scene.resetCollisionTrial();
            check( playback.isPlaying() == ( repeat == 0 ) && data( 0 ).enableLimit && !data( 1 ).enableLimit && data( 0 ).lowerAngle == -0.5f && data( 0 ).upperAngle == 0.5f && data( 1 ).upperAngle == 1.0f && data( 1 ).motorSpeed == 1.0f && physics.GetShapeDensity( visuals[1].shapeHandle ) == 2.0f, "trial reset preserves edited limits speed density and playback state" );
            for( int i = 0; i < 2; ++i )
            {
                const auto pose = physics.GetBodyTransform( visuals[i].bodyHandle );
                check( scene.getAngleLimitComparisonJoints()[i] == joints[i] && physics.IsValid( visuals[i].shapeHandle ) && pose.position.x == -2.0f + 4.0f * static_cast<float>( i ) && pose.position.y == 0.0f && pose.rotation.s == 0.0f && speed( i ) == 0.0f, "trial reset retains handles and restores original poses and rest" );
            }
            advance( 60 );
            check( std::abs( data( 0 ).currentAngle - 0.5f ) < 0.02f && std::abs( speed( 0 ) ) < 0.001f && data( 1 ).currentAngle > 0.85f && std::abs( speed( 1 ) - 1.0f ) < 0.001f, "edited enable bounds and speed change repeated trial results" );
        }
        physics.setRevoluteJointLimit( joints[1], true, -0.2f, 1.0f );
        physics.setRevoluteJointMotor( joints[1], true, -1.0f, 0.2f );
        scene.resetCollisionTrial();
        advance( 60 );
        check( std::abs( data( 1 ).currentAngle + 0.2f ) < 0.02f && std::abs( speed( 1 ) ) < 0.001f, "edited lower angle stops the edited negative target" );
        playback.reset();
        auto& defaults = static_cast<rigidBodyDemo&>( playback.getDemo() );
        const auto restored = defaults.getWorld().getRevoluteJointData( defaults.getAngleLimitComparisonJoints()[1] );
        check( !playback.isPlaying() && !defaults.getWorld().IsValid( joints[0] ) && restored.enableLimit && restored.lowerAngle == -boundary && restored.upperAngle == boundary && restored.motorSpeed == 2.0f, "full reset restores default angle limits and motors with fresh handles" );
        check( std::string_view{ getDemoEntry( demoKind::revoluteLimitMotorComparison ).category } == "조인트", "angle limit comparison retains joint category" );
    }
}
