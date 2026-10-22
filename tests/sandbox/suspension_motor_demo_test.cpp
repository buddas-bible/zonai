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

void checkSuspensionMotorDemo()
{
    for( int substeps : { 1, 4, 8 } )
    {
        demoSession playback{ createRigidBodyDemo };
        playback.selectDemo( demoKind::suspensionMotorComparison );
        auto& scene = static_cast<rigidBodyDemo&>( playback.getDemo() );
        auto& physics = scene.getWorld();
        check( scene.getShapes().size() == 2 && physics.GetBodyCount() == 3 && physics.getJointCount() == 2 && physics.GetGravity().x == 0.0f && physics.GetGravity().y == -9.8f, "suspension motor comparison creates two resting wheels with gravity and wheel joints" );
        const std::array visuals{ scene.getShapes()[0], scene.getShapes()[1] };
        const std::array joints{ scene.getSuspensionMotorJoints()[0], scene.getSuspensionMotorJoints()[1] };
        const auto data = [&]( int i ) { return physics.getWheelJointData( joints[i] ); };
        const auto speed = [&]( int i ) { return physics.GetBodyAngularVelocity( visuals[i].bodyHandle ); };
        const auto advance = [&]( int frames )
        {
            for( int frame = 0; frame < frames; ++frame )
            {
                scene.step( 1.0f / 60.0f, substeps );
                check( std::abs( data( 0 ).currentTranslation - data( 1 ).currentTranslation ) < 1.0e-5f, "rotation motors leave identical spring sag trajectories unchanged" );
                for( int i = 0; i < 2; ++i )
                {
                    const auto joint = data( i );
                    check( std::abs( joint.lateralError ) < 1.0e-5f && physics.GetBodyTransform( visuals[i].bodyHandle ).position.x == -2.0f + 4.0f * static_cast<float>( i ) && std::abs( joint.motorTorque ) <= joint.maxMotorTorque + 1.0e-6f, "suspension motors retain travel axis and torque caps" );
                }
            }
        };
        for( int i = 0; i < 2; ++i )
        {
            const auto joint = data( i );
            check( joint.enableSpring && joint.hertz == 1.0f && joint.dampingRatio == 0.7f && !joint.enableLimit && joint.motorSpeed == 4.0f && joint.maxMotorTorque == 0.05f && joint.currentTranslation == 0.0f && speed( i ) == 0.0f && physics.GetBodyMass( visuals[i].bodyHandle ) == physics.GetBodyMass( visuals[0].bodyHandle ) && physics.GetBodyLinearDamping( visuals[i].bodyHandle ) == 0.0f && physics.GetBodyAngularDamping( visuals[i].bodyHandle ) == 0.0f && !physics.IsBodySleepEnabled( visuals[i].bodyHandle ), "comparison isolates motor activation with equal resting wheels and springs" );
        }
        advance( 30 );
        check( speed( 0 ) == 0.0f && std::abs( speed( 1 ) - 4.0f ) < 1.0e-4f, "disabled motor stays still while enabled motor reaches target angular speed" );
        advance( 150 );
        check( std::abs( data( 0 ).currentTranslation + 0.248f ) < 0.015f && std::abs( physics.GetBodyLinearVelocity( visuals[0].bodyHandle ).y ) < 0.005f, "both suspensions settle to the same gravitational deflection despite different rotation" );
        physics.setWheelJointMotor( joints[1], true, -4.0f, 0.05f );
        advance( 60 );
        check( std::abs( speed( 1 ) + 4.0f ) < 1.0e-4f, "negative target reverses a rotating suspension wheel" );
        physics.setWheelJointMotor( joints[1], false, -4.0f, 0.05f );
        advance( 15 );
        check( std::abs( speed( 1 ) + 4.0f ) < 1.0e-4f && data( 1 ).motorTorque == 0.0f, "disabling motor preserves free rotation without damping" );
        physics.setWheelJointMotor( joints[1], true, 0.0f, 0.05f );
        advance( 30 );
        check( std::abs( speed( 1 ) ) < 1.0e-4f, "enabled zero target brakes wheel rotation without changing suspension sag" );
        physics.setWheelJointMotor( joints[0], true, 2.0f, 0.05f );
        physics.setWheelJointMotor( joints[1], false, 1.0f, 0.025f );
        physics.SetShapeDensity( visuals[1].shapeHandle, 2.0f );
        for( int repeat = 0; repeat < 2; ++repeat )
        {
            playback.setPlaying( repeat == 0 );
            scene.resetCollisionTrial();
            check( playback.isPlaying() == ( repeat == 0 ) && data( 0 ).enableMotor && !data( 1 ).enableMotor && data( 0 ).motorSpeed == 2.0f && data( 1 ).motorSpeed == 1.0f && data( 1 ).maxMotorTorque == 0.025f && physics.GetShapeDensity( visuals[1].shapeHandle ) == 2.0f, "trial reset preserves edited motor activation speed torque density and playback state" );
            for( int i = 0; i < 2; ++i )
            {
                const auto pose = physics.GetBodyTransform( visuals[i].bodyHandle );
                check( scene.getSuspensionMotorJoints()[i] == joints[i] && physics.IsValid( visuals[i].shapeHandle ) && pose.position.x == -2.0f + 4.0f * static_cast<float>( i ) && pose.position.y == 1.0f && pose.rotation.s == 0.0f && LengthSquared( physics.GetBodyLinearVelocity( visuals[i].bodyHandle ) ) == 0.0f && speed( i ) == 0.0f, "trial reset preserves handles and restores initial poses and rest" );
            }
            advance( 30 );
            check( std::abs( speed( 0 ) - 2.0f ) < 1.0e-4f && speed( 1 ) == 0.0f, "edited activation and speed change repeated rotation while sag remains equal" );
        }
        physics.setWheelJointMotor( joints[0], true, 0.0f, 0.0f );
        advance( 30 );
        check( std::abs( speed( 0 ) - 2.0f ) < 1.0e-4f && data( 0 ).motorTorque == 0.0f, "zero torque cannot brake an already rotating suspension wheel" );
        physics.setWheelJointMotor( joints[0], true, 4.0f, 0.001f );
        scene.resetCollisionTrial();
        advance( 15 );
        check( speed( 0 ) > 0.09f && speed( 0 ) < 0.11f, "edited weak torque slows acceleration without changing spring motion" );
        physics.ApplyTorque( visuals[1].bodyHandle, 100.0f );
        demoInput drag{};
        drag.mousePosition = physics.GetBodyTransform( visuals[1].bodyHandle ).position;
        drag.mousePressed = drag.mouseHeld = true;
        scene.handleInput( drag );
        check( physics.IsValid( scene.getMouseJoint() ), "suspension motor comparison retains mouse dragging" );
        scene.resetCollisionTrial();
        check( !physics.IsValid( scene.getMouseJoint() ), "trial reset cancels active dragging" );
        advance( 30 );
        check( speed( 1 ) == 0.0f, "trial reset clears accumulated torque on a disabled motor wheel" );
        playback.reset();
        auto& defaults = static_cast<rigidBodyDemo&>( playback.getDemo() );
        const auto restored = defaults.getWorld().getWheelJointData( defaults.getSuspensionMotorJoints()[1] );
        check( !playback.isPlaying() && !defaults.getWorld().IsValid( joints[0] ) && restored.enableMotor && restored.motorSpeed == 4.0f && restored.maxMotorTorque == 0.05f && !defaults.getWorld().getWheelJointData( defaults.getSuspensionMotorJoints()[0] ).enableMotor, "full reset restores default motors with fresh handles and pauses" );
        check( std::string_view{ getDemoEntry( demoKind::suspensionMotorComparison ).category } == "조인트", "suspension motor comparison retains joint category" );
    }
}
