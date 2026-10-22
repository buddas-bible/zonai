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

void checkSuspensionLoadDemo()
{
    for( int substeps : { 1, 4, 8 } )
    {
        demoSession playback{ createRigidBodyDemo };
        playback.selectDemo( demoKind::suspensionLoadComparison );
        auto& scene = static_cast<rigidBodyDemo&>( playback.getDemo() );
        auto& physics = scene.getWorld();
        check( scene.getShapes().size() == 3 && physics.GetBodyCount() == 4 && physics.getJointCount() == 3 && physics.GetGravity().x == 0.0f && physics.GetGravity().y == -9.8f, "suspension comparison creates three equal wheels with gravity and wheel joints" );
        const std::array visuals{ scene.getShapes()[0], scene.getShapes()[1], scene.getShapes()[2] };
        const std::array joints{ scene.getSuspensionComparisonJoints()[0], scene.getSuspensionComparisonJoints()[1], scene.getSuspensionComparisonJoints()[2] };
        const auto data = [&]( int i ) { return physics.getWheelJointData( joints[i] ); };
        const auto advance = [&]( int frames )
        {
            for( int frame = 0; frame < frames; ++frame )
            {
                scene.step( 1.0f / 60.0f, substeps );
                for( int i = 0; i < 3; ++i )
                {
                    check( std::abs( data( i ).lateralError ) < 1.0e-5f && physics.GetBodyTransform( visuals[i].bodyHandle ).position.x == -3.0f + 3.0f * static_cast<float>( i ) && std::abs( physics.GetBodyAngularVelocity( visuals[i].bodyHandle ) - 2.0f ) < 1.0e-5f, "suspension preserves vertical travel axis and free wheel rotation" );
                }
            }
        };
        const float frequency[] = { 0.5f, 1.0f, 2.0f };
        for( int i = 0; i < 3; ++i )
        {
            const auto joint = data( i );
            check( joint.enableSpring && joint.dampingRatio == 0.7f && !joint.enableMotor && !joint.enableLimit && joint.currentTranslation == 0.0f && joint.axis.x == 0.0f && joint.axis.y == 1.0f && physics.GetBodyMass( visuals[i].bodyHandle ) == physics.GetBodyMass( visuals[0].bodyHandle ) && physics.GetBodyLinearDamping( visuals[i].bodyHandle ) == 0.0f && physics.GetBodyAngularDamping( visuals[i].bodyHandle ) == 0.0f && !physics.IsBodySleepEnabled( visuals[i].bodyHandle ), "comparison isolates frequency with equal masses damping ratio and no motor limits or body damping" );
        }
        advance( 30 );
        const std::array early{ data( 0 ).currentTranslation, data( 1 ).currentTranslation, data( 2 ).currentTranslation };
        check( early[0] < early[1] - 0.15f && early[1] < early[2] - 0.1f, "lower frequency sags farther under the same gravitational load" );
        advance( 450 );
        check( std::abs( early[0] - data( 0 ).currentTranslation ) > 0.2f && std::abs( early[2] - data( 2 ).currentTranslation ) < 0.01f, "higher frequency settles sooner while the soft suspension still approaches equilibrium" );
        for( int i = 0; i < 3; ++i )
        {
            const float omega = 2.0f * std::numbers::pi_v<float> * frequency[i];
            check( data( i ).hertz == frequency[i] && std::abs( data( i ).currentTranslation + 9.8f / ( omega * omega ) ) < 0.015f && std::abs( physics.GetBodyLinearVelocity( visuals[i].bodyHandle ).y ) < 0.005f && std::abs( data( i ).springForce - 9.8f * physics.GetBodyMass( visuals[i].bodyHandle ) ) < 0.01f, "settled suspension deflection and spring reaction balance body weight" );
        }
        physics.setWheelJointSpring( joints[0], true, 2.0f, 0.7f );
        physics.setWheelJointSpring( joints[1], true, 1.0f, 2.0f );
        physics.SetShapeDensity( visuals[1].shapeHandle, 2.0f );
        for( int repeat = 0; repeat < 2; ++repeat )
        {
            playback.setPlaying( repeat == 0 );
            scene.resetCollisionTrial();
            check( playback.isPlaying() == ( repeat == 0 ) && data( 0 ).hertz == 2.0f && data( 1 ).dampingRatio == 2.0f && physics.GetShapeDensity( visuals[1].shapeHandle ) == 2.0f, "suspension reset preserves edited frequency damping density and playback state" );
            for( int i = 0; i < 3; ++i )
            {
                const auto pose = physics.GetBodyTransform( visuals[i].bodyHandle );
                check( scene.getSuspensionComparisonJoints()[i] == joints[i] && physics.IsValid( visuals[i].shapeHandle ) && pose.position.x == -3.0f + 3.0f * static_cast<float>( i ) && pose.position.y == 1.0f && pose.rotation.s == 0.0f && LengthSquared( physics.GetBodyLinearVelocity( visuals[i].bodyHandle ) ) == 0.0f && physics.GetBodyAngularVelocity( visuals[i].bodyHandle ) == 2.0f, "suspension reset preserves handles and restores initial poses linear rest and free spin" );
            }
            advance( 30 );
            check( std::abs( data( 0 ).currentTranslation - data( 2 ).currentTranslation ) < 1.0e-5f && data( 1 ).currentTranslation > early[1] + 0.04f, "frequency and damping edits change repeated suspension motion" );
        }
        advance( 450 );
        check( std::abs( data( 1 ).currentTranslation + 9.8f / ( 4.0f * std::numbers::pi_v<float> * std::numbers::pi_v<float> ) ) < 0.015f && std::abs( data( 1 ).springForce - 9.8f * physics.GetBodyMass( visuals[1].bodyHandle ) ) < 0.01f, "edited density keeps frequency based deflection while increasing the supporting force" );
        physics.setWheelJointSpring( joints[0], true, 0.0f, 0.7f );
        physics.ApplyTorque( visuals[0].bodyHandle, 100.0f );
        demoInput drag{};
        drag.mousePosition = physics.GetBodyTransform( visuals[0].bodyHandle ).position;
        drag.mousePressed = drag.mouseHeld = drag.right = true;
        scene.handleInput( drag );
        check( physics.IsValid( scene.getMouseJoint() ), "suspension comparison retains mouse dragging" );
        scene.resetCollisionTrial();
        check( !physics.IsValid( scene.getMouseJoint() ), "suspension reset cancels active dragging" );
        advance( 30 );
        check( data( 0 ).currentTranslation < -1.1f && data( 0 ).springForce == 0.0f, "zero frequency removes support under gravity while reset clears accumulated torque and retains free rotation" );
        playback.reset();
        auto& defaults = static_cast<rigidBodyDemo&>( playback.getDemo() );
        const auto restored = defaults.getWorld().getWheelJointData( defaults.getSuspensionComparisonJoints()[0] );
        check( !playback.isPlaying() && !defaults.getWorld().IsValid( joints[0] ) && restored.hertz == 0.5f && restored.dampingRatio == 0.7f, "full reset restores default suspension with fresh handles and pauses" );
        check( std::string_view{ getDemoEntry( demoKind::suspensionLoadComparison ).category } == "조인트", "suspension comparison retains joint category" );
    }
}
