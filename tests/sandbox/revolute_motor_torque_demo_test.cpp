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

void checkRevoluteMotorTorqueDemo()
{
    for( int substeps : { 1, 4, 8 } )
    {
        demoSession playback{ createRigidBodyDemo };
        playback.selectDemo( demoKind::revoluteMotorTorqueComparison );
        auto& scene = static_cast<rigidBodyDemo&>( playback.getDemo() );
        auto& physics = scene.getWorld();
        check( scene.getShapes().size() == 3 && physics.GetBodyCount() == 4 && physics.getJointCount() == 3 && LengthSquared( physics.GetGravity() ) == 0.0f, "torque comparison creates three resting rods with fixed center hinges without gravity" );
        const std::array visuals{ scene.getShapes()[0], scene.getShapes()[1], scene.getShapes()[2] };
        const std::array joints{ scene.getTorqueComparisonJoints()[0], scene.getTorqueComparisonJoints()[1], scene.getTorqueComparisonJoints()[2] };
        const auto data = [&]( int i ) { return physics.getRevoluteJointData( joints[i] ); };
        const auto speed = [&]( int i ) { return physics.GetBodyAngularVelocity( visuals[i].bodyHandle ); };
        const auto advance = [&]( int frames )
        {
            for( int frame = 0; frame < frames; ++frame )
            {
                scene.step( 1.0f / 60.0f, substeps );
                for( int i = 0; i < 3; ++i )
                {
                    const auto joint = data( i );
                    check( std::abs( joint.motorTorque ) <= joint.maxMotorTorque + 1.0e-5f, "actual motor torque respects the edited torque limit" );
                    check( LengthSquared( joint.anchorB - joint.anchorA ) < 1.0e-8f && LengthSquared( physics.GetBodyLinearVelocity( visuals[i].bodyHandle ) ) < 1.0e-8f, "center hinges retain fixed positions without linear motion" );
                }
            }
        };
        const float limits[] = { 0.0f, 0.01f, 0.2f };
        for( int i = 0; i < 3; ++i )
        {
            const auto joint = data( i );
            check( joint.maxMotorTorque == limits[i] && joint.enableMotor && !joint.enableLimit && joint.motorSpeed == 2.0f && joint.referenceAngle == 0.0f && physics.GetBodyMass( visuals[i].bodyHandle ) == physics.GetBodyMass( visuals[0].bodyHandle ) && physics.GetBodyRotationalInertia( visuals[i].bodyHandle ) == physics.GetBodyRotationalInertia( visuals[0].bodyHandle ) && !physics.IsBodySleepEnabled( visuals[i].bodyHandle ) && physics.GetBodyAngularDamping( visuals[i].bodyHandle ) == 0.0f, "comparison isolates equal inertia motors without limits damping or automatic sleep" );
        }
        advance( 15 );
        check( speed( 0 ) == 0.0f && speed( 1 ) > 0.13f && speed( 1 ) < 0.16f && std::abs( speed( 2 ) - 2.0f ) < 1.0e-4f, "zero torque stays still weak torque accelerates and sufficient torque tracks the target" );
        advance( 45 );
        check( speed( 0 ) == 0.0f && speed( 1 ) > 0.55f && speed( 1 ) < 0.60f && std::abs( speed( 2 ) - 2.0f ) < 1.0e-4f, "weak motor continues accelerating below target after one second" );
        physics.setRevoluteJointMotor( joints[2], true, -2.0f, 0.2f );
        advance( 60 );
        check( std::abs( speed( 2 ) + 2.0f ) < 1.0e-4f, "negative target reverses an already rotating rod" );
        physics.setRevoluteJointMotor( joints[2], true, 0.0f, 0.2f );
        physics.SetBodyAngularVelocity( visuals[0].bodyHandle, 2.0f );
        physics.setRevoluteJointMotor( joints[0], true, 0.0f, 0.0f );
        advance( 30 );
        check( std::abs( speed( 2 ) ) < 1.0e-4f && speed( 0 ) == 2.0f, "zero target brakes with torque while zero torque preserves free rotation" );
        physics.setRevoluteJointMotor( joints[0], true, 2.0f, 0.2f );
        physics.setRevoluteJointMotor( joints[2], true, 1.0f, 0.2f );
        physics.SetShapeDensity( visuals[1].shapeHandle, 2.0f );
        for( int repeat = 0; repeat < 2; ++repeat )
        {
            playback.setPlaying( repeat == 0 );
            scene.resetCollisionTrial();
            check( playback.isPlaying() == ( repeat == 0 ) && data( 0 ).maxMotorTorque == 0.2f && data( 2 ).motorSpeed == 1.0f && physics.GetShapeDensity( visuals[1].shapeHandle ) == 2.0f, "trial reset preserves edited torque speed density and playback state" );
            for( int i = 0; i < 3; ++i )
            {
                const auto pose = physics.GetBodyTransform( visuals[i].bodyHandle );
                check( scene.getTorqueComparisonJoints()[i] == joints[i] && physics.IsValid( visuals[i].shapeHandle ) && pose.position.x == -2.0f + 2.0f * static_cast<float>( i ) && pose.position.y == 0.0f && pose.rotation.s == 0.0f && speed( i ) == 0.0f, "trial reset retains handles and restores original rod poses and rest" );
            }
            advance( 30 );
            check( std::abs( speed( 0 ) - 2.0f ) < 1.0e-4f && speed( 1 ) > 0.13f && speed( 1 ) < 0.16f && std::abs( speed( 2 ) - 1.0f ) < 1.0e-4f, "edited torque speed and inertia change repeated acceleration and tracking" );
        }
        physics.setRevoluteJointMotor( joints[0], true, 0.0f, 0.0f );
        physics.SetBodyAngularDamping( visuals[0].bodyHandle, 0.2f );
        physics.ApplyTorque( visuals[0].bodyHandle, 100.0f );
        demoInput drag{};
        drag.mousePosition = physics.GetBodyTransform( visuals[0].bodyHandle ).position;
        drag.mousePressed = drag.mouseHeld = drag.right = true;
        scene.handleInput( drag );
        check( physics.IsValid( scene.getMouseJoint() ), "torque comparison retains existing mouse dragging" );
        scene.resetCollisionTrial();
        check( !physics.IsValid( scene.getMouseJoint() ) && physics.GetBodyAngularDamping( visuals[0].bodyHandle ) == 0.2f, "trial reset cancels drag and preserves edited body damping" );
        advance( 30 );
        check( speed( 0 ) == 0.0f && physics.GetBodyTransform( visuals[0].bodyHandle ).rotation.s == 0.0f, "trial reset clears accumulated torque and preserves rest without motor braking" );
        playback.reset();
        auto& defaults = static_cast<rigidBodyDemo&>( playback.getDemo() );
        check( !playback.isPlaying() && !defaults.getWorld().IsValid( joints[0] ) && defaults.getWorld().getRevoluteJointData( defaults.getTorqueComparisonJoints()[0] ).maxMotorTorque == 0.0f && defaults.getWorld().getRevoluteJointData( defaults.getTorqueComparisonJoints()[2] ).motorSpeed == 2.0f, "full reset restores default motors with fresh handles and pauses" );
        check( std::string_view{ getDemoEntry( demoKind::revoluteMotorTorqueComparison ).category } == "조인트", "torque comparison retains the joint category" );
    }
}
