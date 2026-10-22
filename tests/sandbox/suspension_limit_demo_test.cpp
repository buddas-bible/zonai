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

void checkSuspensionLimitDemo()
{
    for( int substeps : { 1, 4, 8 } )
    {
        demoSession playback{ createRigidBodyDemo };
        playback.selectDemo( demoKind::suspensionLimitComparison );
        auto& scene = static_cast<rigidBodyDemo&>( playback.getDemo() );
        auto& physics = scene.getWorld();
        check( scene.getShapes().size() == 2 && physics.GetBodyCount() == 3 && physics.getJointCount() == 2 && physics.GetGravity().x == 0.0f && physics.GetGravity().y == -9.8f, "suspension limit comparison creates two equal wheels with gravity and wheel joints" );
        const std::array visuals{ scene.getShapes()[0], scene.getShapes()[1] };
        const std::array joints{ scene.getSuspensionLimitJoints()[0], scene.getSuspensionLimitJoints()[1] };
        const auto data = [&]( int i ) { return physics.getWheelJointData( joints[i] ); };
        const auto advance = [&]( int frames )
        {
            for( int frame = 0; frame < frames; ++frame )
            {
                scene.step( 1.0f / 60.0f, substeps );
                for( int i = 0; i < 2; ++i )
                {
                    const auto joint = data( i );
                    check( std::abs( joint.lateralError ) < 1.0e-5f && physics.GetBodyTransform( visuals[i].bodyHandle ).position.x == -2.0f + 4.0f * static_cast<float>( i ) && std::abs( physics.GetBodyAngularVelocity( visuals[i].bodyHandle ) - 2.0f ) < 1.0e-5f, "suspension limits preserve vertical travel axis and free wheel rotation" );
                    check( !joint.enableLimit || ( joint.currentTranslation >= joint.lowerTranslation - 0.015f && joint.currentTranslation <= joint.upperTranslation + 0.015f ), "enabled suspension limits keep travel within edited bounds" );
                }
            }
        };
        for( int i = 0; i < 2; ++i )
        {
            const auto joint = data( i );
            check( joint.enableSpring && joint.hertz == 0.5f && joint.dampingRatio == 0.7f && !joint.enableMotor && joint.lowerTranslation == -0.4f && joint.upperTranslation == 0.4f && joint.currentTranslation == 0.0f && joint.axis.x == 0.0f && joint.axis.y == 1.0f && physics.GetBodyMass( visuals[i].bodyHandle ) == physics.GetBodyMass( visuals[0].bodyHandle ) && physics.GetBodyLinearDamping( visuals[i].bodyHandle ) == 0.0f && physics.GetBodyAngularDamping( visuals[i].bodyHandle ) == 0.0f && !physics.IsBodySleepEnabled( visuals[i].bodyHandle ), "comparison isolates travel limits with equal springs masses and initial free spin" );
        }
        advance( 240 );
        check( data( 0 ).currentTranslation < -0.95f && std::abs( data( 1 ).currentTranslation + 0.4f ) < 0.015f && std::abs( physics.GetBodyLinearVelocity( visuals[1].bodyHandle ).y ) < 0.001f && data( 1 ).limitForce > 0.0f, "enabled lower limit stops sag while unrestricted suspension settles farther down" );
        check( std::abs( data( 1 ).springForce + data( 1 ).limitForce - 9.8f * physics.GetBodyMass( visuals[1].bodyHandle ) ) < 0.02f, "spring and lower limit reactions together support body weight" );
        physics.SetGravity( { 0.0f, 9.8f } );
        advance( 240 );
        check( data( 0 ).currentTranslation > 0.95f && std::abs( data( 1 ).currentTranslation - 0.4f ) < 0.015f && std::abs( physics.GetBodyLinearVelocity( visuals[1].bodyHandle ).y ) < 0.001f && data( 1 ).limitForce < 0.0f, "reversed gravity releases lower limit and stops travel at the upper limit" );
        check( std::abs( data( 1 ).springForce + data( 1 ).limitForce + 9.8f * physics.GetBodyMass( visuals[1].bodyHandle ) ) < 0.02f, "spring and upper limit reactions oppose reversed gravity" );
        physics.setWheelJointLimit( joints[0], true, -0.2f, 0.2f );
        physics.setWheelJointLimit( joints[1], false, -0.6f, 0.6f );
        physics.SetShapeDensity( visuals[1].shapeHandle, 2.0f );
        for( int repeat = 0; repeat < 2; ++repeat )
        {
            playback.setPlaying( repeat == 0 );
            scene.resetCollisionTrial();
            check( playback.isPlaying() == ( repeat == 0 ) && data( 0 ).enableLimit && !data( 1 ).enableLimit && data( 0 ).lowerTranslation == -0.2f && data( 0 ).upperTranslation == 0.2f && data( 1 ).lowerTranslation == -0.6f && data( 1 ).upperTranslation == 0.6f && physics.GetShapeDensity( visuals[1].shapeHandle ) == 2.0f && physics.GetGravity().y == 9.8f, "trial reset preserves limit edits density gravity and playback state" );
            for( int i = 0; i < 2; ++i )
            {
                const auto pose = physics.GetBodyTransform( visuals[i].bodyHandle );
                check( scene.getSuspensionLimitJoints()[i] == joints[i] && physics.IsValid( visuals[i].shapeHandle ) && pose.position.x == -2.0f + 4.0f * static_cast<float>( i ) && pose.position.y == 1.0f && pose.rotation.s == 0.0f && LengthSquared( physics.GetBodyLinearVelocity( visuals[i].bodyHandle ) ) == 0.0f && physics.GetBodyAngularVelocity( visuals[i].bodyHandle ) == 2.0f, "trial reset preserves handles and restores initial poses linear rest and free spin" );
            }
            advance( 240 );
            check( std::abs( data( 0 ).currentTranslation - 0.2f ) < 0.015f && data( 1 ).currentTranslation > 0.95f, "edited enable and upper bound change repeated suspension travel" );
        }
        physics.SetGravity( { 0.0f, -9.8f } );
        physics.setWheelJointLimit( joints[1], true, -0.6f, 0.6f );
        scene.resetCollisionTrial();
        advance( 240 );
        check( std::abs( data( 0 ).currentTranslation + 0.2f ) < 0.015f && std::abs( data( 1 ).currentTranslation + 0.6f ) < 0.015f, "edited lower bounds change repeated sag limits" );
        physics.ApplyTorque( visuals[0].bodyHandle, 100.0f );
        demoInput drag{};
        drag.mousePosition = physics.GetBodyTransform( visuals[0].bodyHandle ).position;
        drag.mousePressed = drag.mouseHeld = drag.right = true;
        scene.handleInput( drag );
        check( physics.IsValid( scene.getMouseJoint() ), "suspension limit comparison retains mouse dragging" );
        scene.resetCollisionTrial();
        check( !physics.IsValid( scene.getMouseJoint() ), "trial reset cancels active dragging" );
        advance( 30 );
        playback.reset();
        auto& defaults = static_cast<rigidBodyDemo&>( playback.getDemo() );
        const auto restored = defaults.getWorld().getWheelJointData( defaults.getSuspensionLimitJoints()[1] );
        check( !playback.isPlaying() && !defaults.getWorld().IsValid( joints[0] ) && restored.enableLimit && restored.lowerTranslation == -0.4f && restored.upperTranslation == 0.4f && defaults.getWorld().GetGravity().y == -9.8f, "full reset restores default limits and gravity with fresh handles and pauses" );
        check( std::string_view{ getDemoEntry( demoKind::suspensionLimitComparison ).category } == "조인트", "suspension limit comparison retains joint category" );
    }
}
