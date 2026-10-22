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

void checkWeldElasticityDemo()
{
    for( int substeps : { 1, 4, 8 } )
    {
        demoSession playback{ createRigidBodyDemo };
        playback.selectDemo( demoKind::weldElasticityComparison );
        auto& scene = static_cast<rigidBodyDemo&>( playback.getDemo() );
        auto& physics = scene.getWorld();
        check( scene.getShapes().size() == 3 && physics.GetBodyCount() == 4 && physics.getJointCount() == 3 && physics.GetGravity().x == 0.0f && physics.GetGravity().y == -9.8f, "weld comparison creates three equal rods under gravity" );
        const std::array visuals{ scene.getShapes()[0], scene.getShapes()[1], scene.getShapes()[2] };
        const std::array joints{ scene.getWeldComparisonJoints()[0], scene.getWeldComparisonJoints()[1], scene.getWeldComparisonJoints()[2] };
        const auto data = [&]( int i ) { return physics.getWeldJointData( joints[i] ); };
        const auto error = [&]( int i ) { const auto joint = data( i ); return Length( joint.anchorB - joint.anchorA ); };
        const auto advance = [&]( int frames ) { for( int frame = 0; frame < frames; ++frame ) scene.step( 1.0f / 60.0f, substeps ); };
        for( int i = 0; i < 3; ++i )
        {
            const auto joint = data( i );
            check( joint.referenceAngle == 0.0f && joint.currentAngle == 0.0f && error( i ) == 0.0f && joint.linearDampingRatio == 0.7f && joint.angularDampingRatio == 0.7f && physics.GetBodyMass( visuals[i].bodyHandle ) == physics.GetBodyMass( visuals[0].bodyHandle ) && physics.GetBodyLinearDamping( visuals[i].bodyHandle ) == 0.0f && physics.GetBodyAngularDamping( visuals[i].bodyHandle ) == 0.0f && !physics.IsBodySleepEnabled( visuals[i].bodyHandle ), "weld comparison isolates compliance with equal resting rods reference angle and damping ratios" );
        }
        advance( 360 );
        check( error( 0 ) < 0.01f && std::abs( data( 0 ).currentAngle ) < 0.01f && std::abs( physics.GetBodyTransform( visuals[0].bodyHandle ).position.y - 1.0f ) < 0.01f, "zero hertz weld keeps both attachment and orientation rigid under gravity" );
        check( error( 1 ) > 0.1f && std::abs( data( 1 ).currentAngle ) < 0.01f && data( 1 ).anchorB.y < data( 1 ).anchorA.y - 0.1f, "linear compliance sags while keeping rod orientation rigid" );
        check( error( 2 ) < 0.015f && data( 2 ).currentAngle < -0.1f, "angular compliance bends downward while keeping attachment rigid" );
        check( data( 0 ).linearHertz == 0.0f && data( 0 ).angularHertz == 0.0f && data( 1 ).linearHertz == 1.0f && data( 1 ).angularHertz == 0.0f && data( 2 ).linearHertz == 0.0f && data( 2 ).angularHertz == 1.0f, "default weld channels match hard linear only and angular only comparison" );
        physics.setWeldJointLinearTuning( joints[0], 1.0f, 0.7f );
        physics.setWeldJointAngularTuning( joints[2], 0.0f, 0.7f );
        physics.SetShapeDensity( visuals[1].shapeHandle, 2.0f );
        for( int repeat = 0; repeat < 2; ++repeat )
        {
            playback.setPlaying( repeat == 0 );
            scene.resetCollisionTrial();
            check( playback.isPlaying() == ( repeat == 0 ) && data( 0 ).linearHertz == 1.0f && data( 2 ).angularHertz == 0.0f && physics.GetShapeDensity( visuals[1].shapeHandle ) == 2.0f, "weld reset preserves independently edited channels density and playback state" );
            for( int i = 0; i < 3; ++i )
            {
                const auto pose = physics.GetBodyTransform( visuals[i].bodyHandle );
                check( scene.getWeldComparisonJoints()[i] == joints[i] && physics.IsValid( visuals[i].shapeHandle ) && pose.position.x == -3.0f + 3.0f * static_cast<float>( i ) && pose.position.y == 1.0f && pose.rotation.s == 0.0f && LengthSquared( physics.GetBodyLinearVelocity( visuals[i].bodyHandle ) ) == 0.0f && physics.GetBodyAngularVelocity( visuals[i].bodyHandle ) == 0.0f, "weld reset preserves handles and restores initial poses and rest" );
            }
            advance( 60 );
            check( error( 0 ) > 0.1f && std::abs( error( 0 ) - error( 1 ) ) < 1.0e-4f && std::abs( data( 0 ).currentAngle ) < 0.01f && error( 2 ) < 0.01f && std::abs( data( 2 ).currentAngle ) < 0.01f, "linear frequency edit reproduces matching sag while zero angular frequency restores rigid orientation" );
        }
        physics.setWeldJointLinearTuning( joints[1], 0.0f, 0.7f );
        scene.resetCollisionTrial();
        advance( 120 );
        check( error( 1 ) < 0.01f && std::abs( data( 1 ).currentAngle ) < 0.01f, "zero linear frequency restores rigid attachment instead of removing support" );
        demoInput drag{};
        drag.mousePosition = physics.GetBodyTransform( visuals[0].bodyHandle ).position;
        drag.mousePressed = drag.mouseHeld = true;
        scene.handleInput( drag );
        check( physics.IsValid( scene.getMouseJoint() ), "weld comparison retains mouse dragging" );
        scene.resetCollisionTrial();
        check( !physics.IsValid( scene.getMouseJoint() ), "weld reset cancels active dragging" );
        playback.reset();
        auto& defaults = static_cast<rigidBodyDemo&>( playback.getDemo() );
        const auto restored = defaults.getWorld().getWeldJointData( defaults.getWeldComparisonJoints()[2] );
        check( !playback.isPlaying() && !defaults.getWorld().IsValid( joints[0] ) && restored.linearHertz == 0.0f && restored.angularHertz == 1.0f && defaults.getWorld().getWeldJointData( defaults.getWeldComparisonJoints()[0] ).linearHertz == 0.0f, "full reset restores default weld channels with fresh handles and pauses" );
        check( std::string_view{ getDemoEntry( demoKind::weldElasticityComparison ).category } == "조인트", "weld comparison retains joint category" );
    }
}
