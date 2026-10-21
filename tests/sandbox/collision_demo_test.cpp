#include "rigidBodyDemo.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string_view>
#include <vector>

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

bool same( vec2 a, vec2 b )
{
    return a.x == b.x && a.y == b.y;
}

}

void checkCcdDemo()
{
    for( int substeps : { 1, 4, 8 } )
    {
        demoSession playback{ createRigidBodyDemo };
        playback.selectDemo( demoKind::ccdComparison );
        auto& scene = static_cast<rigidBodyDemo&>( playback.getDemo() );
        auto& simulation = scene.getWorld();
        playback.stepOnce( substeps );
        const auto projectile = scene.getImpulseBody();
        check( simulation.IsValid( projectile ) && simulation.HadBodyTimeOfImpact( projectile ) && simulation.GetBodyTransform( projectile ).position.x < -0.1f, "CCD enabled trial stops the fast projectile in front of the thin wall after one fixed frame" );
        check( scene.getShapes().size() == 2 && simulation.GetBodyCount() == 2 && std::string_view{ getDemoEntry( scene.getKind() ).category } == "강체", "CCD trial uses the existing category with one editable wall and projectile" );
        check( !simulation.IsBodyBullet( projectile ), "static wall CCD works without enabling Bullet" );
        const auto wall = scene.getShapes()[0];
        const auto probe = scene.getShapes()[1];
        simulation.SetContinuousEnabled( false );
        scene.resetCollisionTrial();
        check( !simulation.IsContinuousEnabled() && same( simulation.GetBodyLinearVelocity( projectile ), { 100, 0 } ) && same( simulation.GetBodyTransform( projectile ).position, { -0.8f, 0 } ), "CCD-off reset preserves the setting and restores the same launch conditions" );
        playback.stepOnce( substeps );
        check( simulation.GetBodyTransform( projectile ).position.x > 0.15f && !simulation.HadBodyTimeOfImpact( projectile ), "CCD disabled trial tunnels through the same thin wall in one fixed frame" );
        simulation.SetShapeFriction( wall.shapeHandle, 0.4f );
        simulation.SetShapeRestitution( probe.shapeHandle, 0.6f );
        simulation.SetShapeDensity( probe.shapeHandle, 2.0f );
        simulation.SetBodyLinearDamping( projectile, 0.1f );
        simulation.SetBodyGravityScale( projectile, 0.3f );
        simulation.SetBodySafetyFactor( projectile, 0.2f );
        simulation.SetBodyBullet( projectile, true );
        simulation.SetGravity( { 0, -1 } );
        const float mass = simulation.GetBodyMass( projectile );
        demoInput drag{};
        drag.mousePosition = simulation.GetBodyTransform( projectile ).position;
        drag.mousePressed = drag.mouseHeld = drag.left = true;
        scene.handleInput( drag );
        check( simulation.IsValid( scene.getMouseJoint() ), "CCD projectile retains existing drag interaction" );
        scene.resetCollisionTrial();
        check( !simulation.IsValid( scene.getMouseJoint() ) && simulation.IsValid( probe.bodyHandle ) && simulation.IsValid( wall.shapeHandle ) && simulation.IsBodyAwake( projectile ), "CCD trial reset cancels input and retains handles while waking the projectile" );
        check( !simulation.IsContinuousEnabled() && simulation.GetShapeFriction( wall.shapeHandle ) == 0.4f && simulation.GetShapeRestitution( probe.shapeHandle ) == 0.6f && simulation.GetShapeDensity( probe.shapeHandle ) == 2.0f && simulation.GetBodyMass( projectile ) == mass && simulation.GetBodyLinearDamping( projectile ) == 0.1f && simulation.GetBodyGravityScale( projectile ) == 0.3f && simulation.GetBodySafetyFactor( projectile ) == 0.2f && simulation.IsBodyBullet( projectile ) && same( simulation.GetGravity(), { 0, -1 } ), "CCD trial reset preserves inspector material body world and CCD edits" );
        playback.stepOnce( substeps );
        check( simulation.GetBodyTransform( projectile ).position.x > 0.15f && !simulation.HadBodyTimeOfImpact( projectile ), "Bullet cannot override disabled world CCD during repeated trials" );
        simulation.SetContinuousEnabled( true );
        scene.resetCollisionTrial();
        playback.stepOnce( substeps );
        check( simulation.HadBodyTimeOfImpact( projectile ) && simulation.GetBodyTransform( projectile ).position.x < -0.1f, "edited repeated trial still catches the wall when CCD is reenabled" );
        check( !playback.isPlaying(), "CCD single-step comparisons remain paused" );
        playback.setPlaying( true );
        scene.resetCollisionTrial();
        check( playback.isPlaying(), "CCD trial reset retains play state" );
        playback.reset();
        auto& defaults = static_cast<rigidBodyDemo&>( playback.getDemo() );
        check( !playback.isPlaying() && defaults.getWorld().IsContinuousEnabled() && !defaults.getWorld().IsValid( projectile ), "full CCD scene reset restores default settings with fresh handles" );
    }
}

void checkFilterComparisonDemo()
{
    for( int substeps : { 1, 4, 8 } )
    {
        demoSession playback{ createRigidBodyDemo };
        playback.selectDemo( demoKind::collisionFilterComparison );
        auto& scene = static_cast<rigidBodyDemo&>( playback.getDemo() );
        auto& simulation = scene.getWorld();
        check( scene.getShapes().size() == 4 && simulation.GetBodyCount() == 4 && same( simulation.GetGravity(), {} ), "filter comparison creates two editable head-on pairs without gravity" );
        const auto visuals = std::vector<visualShape>{ scene.getShapes().begin(), scene.getShapes().end() };
        const auto upper = visuals[0].bodyHandle;
        const auto lower = visuals[2].bodyHandle;
        for( int frame = 0; frame < 90; ++frame )
            playback.stepOnce( substeps );
        check( simulation.GetBodyTransform( upper ).position.x < 0 && simulation.GetBodyLinearVelocity( upper ).x < 0, "allowed upper pair collides and rebounds" );
        check( simulation.GetBodyTransform( lower ).position.x > 0 && simulation.GetBodyLinearVelocity( lower ).x > 0, "masked lower pair passes through without collision" );
        auto filter = simulation.GetShapeFilter( visuals[2].shapeHandle );
        filter.maskBits |= 2;
        simulation.SetShapeFilter( visuals[2].shapeHandle, filter );
        scene.resetCollisionTrial();
        for( std::size_t i = 0; i < visuals.size(); ++i )
        {
            const auto body = visuals[i].bodyHandle;
            check( scene.getShapes()[i].shapeHandle == visuals[i].shapeHandle && same( simulation.GetBodyTransform( body ).position, { i % 2 == 0 ? -2.0f : 2.0f, i < 2 ? 1.5f : -1.5f } ) && same( simulation.GetBodyLinearVelocity( body ), { i % 2 == 0 ? 2.0f : -2.0f, 0 } ), "filter reset retains handles and restores identical starting conditions" );
        }
        check( simulation.GetShapeFilter( visuals[2].shapeHandle ).maskBits == filter.maskBits, "filter reset preserves the edited lower mask" );
        for( int frame = 0; frame < 90; ++frame )
            playback.stepOnce( substeps );
        check( simulation.GetBodyLinearVelocity( upper ).x < 0 && simulation.GetBodyLinearVelocity( lower ).x < 0, "allowing the lower mask makes both pairs collide on the repeated trial" );
        filter = simulation.GetShapeFilter( visuals[0].shapeHandle );
        filter.maskBits &= ~std::uint64_t{ 2 };
        simulation.SetShapeFilter( visuals[0].shapeHandle, filter );
        filter = simulation.GetShapeFilter( visuals[3].shapeHandle );
        filter.maskBits &= ~std::uint64_t{ 1 };
        simulation.SetShapeFilter( visuals[3].shapeHandle, filter );
        scene.resetCollisionTrial();
        for( int frame = 0; frame < 90; ++frame )
            playback.stepOnce( substeps );
        check( simulation.GetBodyTransform( upper ).position.x > 0 && simulation.GetBodyTransform( lower ).position.x > 0 && simulation.GetBodyLinearVelocity( upper ).x > 0 && simulation.GetBodyLinearVelocity( lower ).x > 0, "excluding the counterpart in either side mask makes the pair pass through" );
        for( const auto& visual : visuals )
        {
            filter = simulation.GetShapeFilter( visual.shapeHandle );
            filter.maskBits = ~std::uint64_t{ 0 };
            filter.groupIndex = 7;
            simulation.SetShapeFilter( visual.shapeHandle, filter );
        }
        collisionMatrix matrix;
        matrix.setPair( 0, 1, false );
        playback.setCollisionMatrix( matrix );
        scene.resetCollisionTrial();
        check( playback.getCollisionMatrix() == matrix && simulation.GetShapeFilter( visuals[0].shapeHandle ).groupIndex == 7, "filter reset preserves common collision settings and edited groups" );
        for( int frame = 0; frame < 90; ++frame )
            playback.stepOnce( substeps );
        check( simulation.GetBodyTransform( upper ).position.x > 0 && simulation.GetBodyTransform( lower ).position.x > 0, "common collision exclusion still blocks pairs with permissive masks and positive groups" );
        playback.reset();
        auto& defaults = static_cast<rigidBodyDemo&>( playback.getDemo() );
        check( !playback.isPlaying() && playback.getCollisionMatrix() == matrix && !defaults.getWorld().IsValid( upper ), "full filter reset retains common settings and replaces handles while pausing" );
        check( defaults.getWorld().GetShapeFilter( defaults.getShapes()[0].shapeHandle ).maskBits == ~std::uint64_t{ 0 } && defaults.getWorld().GetShapeFilter( defaults.getShapes()[2].shapeHandle ).maskBits == ~std::uint64_t{ 2 } && defaults.getWorld().GetShapeFilter( defaults.getShapes()[0].shapeHandle ).groupIndex == 0, "full filter reset restores initial masks and groups" );
    }
}

void checkSensorWallDemo()
{
    for( int substeps : { 1, 4, 8 } )
    {
        demoSession playback{ createRigidBodyDemo };
        playback.selectDemo( demoKind::sensorWallComparison );
        auto& scene = static_cast<rigidBodyDemo&>( playback.getDemo() );
        auto& simulation = scene.getWorld();
        check( scene.getShapes().size() == 4 && simulation.GetBodyCount() == 4 && same( simulation.GetGravity(), {} ), "sensor comparison creates identical wall and sensor lanes without gravity" );
        const auto wall = scene.getShapes()[0];
        const auto stopped = scene.getShapes()[1];
        const auto sensor = scene.getShapes()[2];
        const auto visitor = scene.getShapes()[3];
        int begins = 0, ends = 0;
        for( int frame = 0; frame < 180; ++frame )
        {
            playback.stepOnce( substeps );
            for( const auto& event : simulation.GetSensorBeginEvents() )
            {
                check( event.sensorShapeId == sensor.shapeHandle && event.visitorShapeId == visitor.shapeHandle, "sensor entry identifies the lower sensor and visitor" );
                ++begins;
            }
            for( const auto& event : simulation.GetSensorEndEvents() )
            {
                check( event.sensorShapeId == sensor.shapeHandle && event.visitorShapeId == visitor.shapeHandle, "sensor exit identifies the lower sensor and visitor" );
                ++ends;
            }
            if( frame == 89 )
            {
                check( simulation.GetBodyTransform( stopped.bodyHandle ).position.x < -1.0f && std::abs( simulation.GetBodyLinearVelocity( stopped.bodyHandle ).x ) < 0.01f, "solid wall stops the upper circle" );
                check( std::abs( simulation.GetBodyTransform( visitor.bodyHandle ).position.x ) < 0.01f && same( simulation.GetBodyLinearVelocity( visitor.bodyHandle ), { 2, 0 } ), "sensor lets the lower circle pass without changing its velocity" );
                check( simulation.GetShapeSensorCapacity( sensor.shapeHandle ) == 1 && begins == 1 && ends == 0 && simulation.GetShapeContactCapacity( sensor.shapeHandle ) == 0, "sensor entry tracks one overlap without creating a solid contact" );
            }
        }
        check( simulation.GetShapeSensorCapacity( sensor.shapeHandle ) == 0 && begins == 1 && ends == 1 && simulation.GetBodyTransform( visitor.bodyHandle ).position.x > 2.5f, "sensor exit clears the overlap after one entry and exit" );
        for( const auto eventsShape : { sensor.shapeHandle, visitor.shapeHandle } )
        {
            scene.resetCollisionTrial();
            for( int frame = 0; frame < 90; ++frame )
                playback.stepOnce( substeps );
            check( simulation.GetShapeSensorCapacity( sensor.shapeHandle ) == 1, "repeated sensor trial detects entry again" );
            simulation.SetShapeSensorEventsEnabled( eventsShape, false );
            playback.stepOnce( substeps );
            check( simulation.GetShapeSensorCapacity( sensor.shapeHandle ) == 0 && simulation.GetSensorBeginEvents().empty() && simulation.GetSensorEndEvents().size() == 1 && same( simulation.GetBodyLinearVelocity( visitor.bodyHandle ), { 2, 0 } ), "disabling either sensor or visitor events stops detection on the next step without blocking motion" );
            scene.resetCollisionTrial();
            check( !simulation.AreShapeSensorEventsEnabled( eventsShape ), "sensor trial reset preserves disabled event settings" );
            for( int frame = 0; frame < 90; ++frame )
            {
                playback.stepOnce( substeps );
                check( simulation.GetShapeSensorCapacity( sensor.shapeHandle ) == 0 && simulation.GetSensorBeginEvents().empty(), "disabled sensor trial never records entry" );
            }
            simulation.SetShapeSensorEventsEnabled( eventsShape, true );
            playback.stepOnce( substeps );
            check( simulation.GetShapeSensorCapacity( sensor.shapeHandle ) == 1 && simulation.GetSensorBeginEvents().size() == 1, "reenabling events detects the visitor already inside" );
            scene.resetCollisionTrial();
            check( simulation.GetShapeSensorCapacity( sensor.shapeHandle ) == 0 && simulation.GetSensorBeginEvents().empty() && same( simulation.GetBodyTransform( visitor.bodyHandle ).position, { -3, -1.5f } ) && same( simulation.GetBodyLinearVelocity( visitor.bodyHandle ), { 2, 0 } ), "sensor trial reset clears the previous overlap without advancing launch conditions" );
        }
        auto filter = simulation.GetShapeFilter( visitor.shapeHandle );
        filter.maskBits = 0;
        simulation.SetShapeFilter( visitor.shapeHandle, filter );
        scene.resetCollisionTrial();
        for( int frame = 0; frame < 90; ++frame )
            playback.stepOnce( substeps );
        check( simulation.GetShapeSensorCapacity( sensor.shapeHandle ) == 0 && simulation.GetShapeFilter( visitor.shapeHandle ).maskBits == 0 && same( simulation.GetBodyLinearVelocity( visitor.bodyHandle ), { 2, 0 } ), "sensor trial preserves an edited mask and excludes detection without blocking motion" );
        collisionMatrix matrix;
        matrix.setPair( 0, 0, false );
        playback.setCollisionMatrix( matrix );
        scene.resetCollisionTrial();
        check( playback.getCollisionMatrix() == matrix && simulation.IsValid( sensor.shapeHandle ) && simulation.IsValid( wall.shapeHandle ), "sensor trial reset retains common settings and existing sensor and wall handles" );
        playback.reset();
        auto& defaults = static_cast<rigidBodyDemo&>( playback.getDemo() );
        check( !playback.isPlaying() && playback.getCollisionMatrix() == matrix && !defaults.getWorld().IsValid( visitor.bodyHandle ) && defaults.getWorld().AreShapeSensorEventsEnabled( defaults.getShapes()[2].shapeHandle ) && defaults.getWorld().AreShapeSensorEventsEnabled( defaults.getShapes()[3].shapeHandle ) && defaults.getWorld().GetShapeFilter( defaults.getShapes()[3].shapeHandle ).maskBits == ~std::uint64_t{ 0 }, "full sensor reset restores default event and mask settings while preserving common settings" );
    }
}

void checkCollisionDemos()
{
    checkSensorWallDemo();
    checkFilterComparisonDemo();
    checkCcdDemo();
    rigidBodyDemo scene{ demoKind::collisionComparison };
    auto& simulation = scene.getWorld();
    check( scene.getShapes().size() == 6 && simulation.GetBodyCount() == 6, "collision comparison creates three independently editable dynamic pairs" );
    const auto visuals = std::vector<visualShape>{ scene.getShapes().begin(), scene.getShapes().end() };
    const auto first = visuals[0];
    const auto initial = simulation.GetBodyTransform( first.bodyHandle );
    check( same( simulation.GetGravity(), {} ), "head-on collision comparison starts without gravity" );
    const vec2 velocity = simulation.GetBodyLinearVelocity( first.bodyHandle );
    check( velocity.x > 0 && simulation.GetBodyLinearVelocity( visuals[1].bodyHandle ).x < 0, "collision pair starts with opposing velocities" );
    for( int i = 0; i < 90; ++i )
        scene.step( 1.0f / 60.0f, 4 );
    check( simulation.GetBodyLinearVelocity( first.bodyHandle ).x < 0, "head-on pair collides and rebounds" );
    simulation.SetShapeDensity( first.shapeHandle, 3.0f );
    simulation.SetShapeFriction( first.shapeHandle, 0.7f );
    simulation.SetShapeRestitution( first.shapeHandle, 0.8f );
    simulation.SetBodyLinearDamping( first.bodyHandle, 0.2f );
    simulation.SetBodyAngularDamping( first.bodyHandle, 0.3f );
    simulation.SetBodyGravityScale( first.bodyHandle, 0.4f );
    simulation.SetBodyBullet( first.bodyHandle, true );
    const float mass = simulation.GetBodyMass( first.bodyHandle );
    const vec2 gravity{ 1, -2 };
    simulation.SetGravity( gravity );
    simulation.ApplyForceToCenter( first.bodyHandle, { 300, 400 } );
    simulation.SetBodyAngularVelocity( first.bodyHandle, 5.0f );
    demoInput input{};
    input.mousePosition = simulation.GetBodyTransform( first.bodyHandle ).position;
    input.mousePressed = input.mouseHeld = input.left = true;
    scene.handleInput( input );
    check( simulation.IsValid( scene.getMouseJoint() ), "collision body supports existing mouse inspector interaction" );
    scene.resetCollisionTrial();
    check( !simulation.IsValid( scene.getMouseJoint() ), "trial reset cancels dragging and held inputs" );
    check( same( simulation.GetBodyTransform( first.bodyHandle ).position, initial.position ) && same( simulation.GetBodyLinearVelocity( first.bodyHandle ), velocity ) && simulation.GetBodyAngularVelocity( first.bodyHandle ) == 0.0f, "trial reset restores initial pose and launch velocity" );
    check( simulation.GetShapeDensity( first.shapeHandle ) == 3.0f && simulation.GetBodyMass( first.bodyHandle ) == mass && simulation.GetShapeFriction( first.shapeHandle ) == 0.7f && simulation.GetShapeRestitution( first.shapeHandle ) == 0.8f, "trial reset preserves inspector material density and mass" );
    check( same( simulation.GetGravity(), gravity ) && simulation.GetBodyLinearDamping( first.bodyHandle ) == 0.2f && simulation.GetBodyAngularDamping( first.bodyHandle ) == 0.3f && simulation.GetBodyGravityScale( first.bodyHandle ) == 0.4f && simulation.IsBodyBullet( first.bodyHandle ), "trial reset preserves edited world and body settings" );
    for( std::size_t i = 0; i < visuals.size(); ++i )
        check( scene.getShapes()[i].bodyHandle == visuals[i].bodyHandle && scene.getShapes()[i].shapeHandle == visuals[i].shapeHandle && simulation.IsValid( visuals[i].shapeHandle ), "trial reset preserves inspector body and shape handles" );
    check( scene.getContacts().empty(), "trial reset removes contacts from the previous collision" );
    simulation.SetGravity( {} );
    simulation.SetBodyLinearDamping( first.bodyHandle, 0.0f );
    scene.step( 1.0f / 60.0f, 4 );
    check( same( simulation.GetBodyLinearVelocity( first.bodyHandle ), velocity ), "trial reset clears accumulated forces and held force before the next step" );
    rigidBodyDemo friction{ demoKind::frictionComparison };
    check( friction.getShapes().size() == 6, "friction comparison creates three editable ramps and boxes" );
    for( int i = 0; i < 90; ++i )
        friction.step( 1.0f / 60.0f, 4 );
    const auto frictionX = [&]( std::size_t i )
    {
        return friction.getWorld().GetBodyTransform( friction.getShapes()[i].bodyHandle ).position.x;
    };
    check( frictionX( 1 ) > frictionX( 3 ) + 0.2f && frictionX( 3 ) > frictionX( 5 ) + 0.1f, "otherwise identical ramps visibly separate low medium and high friction motion" );
    friction.getWorld().SetShapeFriction( friction.getShapes()[1].shapeHandle, 1.0f );
    friction.resetCollisionTrial();
    const float editedStart = frictionX( 1 );
    for( int i = 0; i < 90; ++i )
        friction.step( 1.0f / 60.0f, 4 );
    check( std::abs( frictionX( 1 ) - editedStart ) < 0.15f, "edited friction is retained and changes the repeated trial" );

    rigidBodyDemo restitution{ demoKind::restitutionComparison };
    check( restitution.getShapes().size() == 4, "restitution comparison creates one editable floor and three circles" );
    float peaks[3]{};
    for( int i = 0; i < 180; ++i )
    {
        restitution.step( 1.0f / 60.0f, 4 );
        if( i < 50 ) continue;
        for( std::size_t j = 0; j < 3; ++j )
            peaks[j] = std::max( peaks[j], restitution.getWorld().GetBodyTransform( restitution.getShapes()[j + 1].bodyHandle ).position.y );
    }
    check( peaks[0] + 0.2f < peaks[1] && peaks[1] + 0.5f < peaks[2], "equal-height drops visibly separate restitution rebound heights" );
    restitution.getWorld().SetShapeRestitution( restitution.getShapes()[3].shapeHandle, 0.0f );
    restitution.resetCollisionTrial();
    float editedPeak = 0.0f;
    for( int i = 0; i < 180; ++i )
    {
        restitution.step( 1.0f / 60.0f, 4 );
        if( i >= 50 ) editedPeak = std::max( editedPeak, restitution.getWorld().GetBodyTransform( restitution.getShapes()[3].bodyHandle ).position.y );
    }
    check( editedPeak + 0.5f < peaks[2], "edited restitution is retained and changes the repeated trial" );

    rigidBodyDemo stack{ demoKind::boxStack };
    check( stack.getShapes().size() == 9, "stack trial creates an editable floor and eight boxes" );
    for( int i = 0; i < 360; ++i )
        stack.step( 1.0f / 60.0f, 4 );
    for( std::size_t i = 1; i < stack.getShapes().size(); ++i )
    {
        const auto pose = stack.getWorld().GetBodyTransform( stack.getShapes()[i].bodyHandle );
        check( IsFinite( pose.position ) && std::abs( pose.position.x ) < 0.3f && std::abs( pose.position.y - ( static_cast<float>( i ) - 0.5f ) ) < 0.3f, "stack remains upright and supported after settling" );
    }
    stack.resetCollisionTrial();
    rigidBodyDemo freshStack{ demoKind::boxStack };
    for( int i = 0; i < 30; ++i )
    {
        stack.step( 1.0f / 60.0f, 4 );
        freshStack.step( 1.0f / 60.0f, 4 );
    }
    for( std::size_t i = 1; i < stack.getShapes().size(); ++i )
        check( Length( stack.getWorld().GetBodyTransform( stack.getShapes()[i].bodyHandle ).position - freshStack.getWorld().GetBodyTransform( freshStack.getShapes()[i].bodyHandle ).position ) < 1.0e-4f, "reset stack starts without previous contact or force history affecting the repeated trial" );

    for( demoKind kind : { demoKind::collisionComparison, demoKind::frictionComparison, demoKind::restitutionComparison, demoKind::boxStack, demoKind::ccdComparison, demoKind::collisionFilterComparison, demoKind::sensorWallComparison } )
    {
        check( std::string_view{ getDemoEntry( kind ).category } == "강체", "trial demos retain the existing rigid body category" );
        demoSession playback{ createRigidBodyDemo };
        playback.selectDemo( kind );
        auto& trial = static_cast<rigidBodyDemo&>( playback.getDemo() );
        auto& physics = trial.getWorld();
        const auto shape = trial.getShapes().back();
        const auto pose = physics.GetBodyTransform( shape.bodyHandle );
        const auto launch = physics.GetBodyLinearVelocity( shape.bodyHandle );
        physics.SetShapeFriction( shape.shapeHandle, 0.6f );
        physics.SetShapeRestitution( shape.shapeHandle, 0.4f );
        physics.SetShapeDensity( shape.shapeHandle, 2.5f );
        physics.SetBodySleepEnabled( shape.bodyHandle, false );
        physics.SetBodySleepThreshold( shape.bodyHandle, 0.2f );
        const auto mass = physics.GetBodyMass( shape.bodyHandle );
        auto filter = physics.GetShapeFilter( shape.shapeHandle );
        filter.categoryBits = 3;
        filter.maskBits = 5;
        physics.SetShapeFilter( shape.shapeHandle, filter );
        physics.SetBodyTransform( shape.bodyHandle, { { 30, 40 }, rot2::FromRadians( 1.0f ) } );
        physics.SetBodyLinearVelocity( shape.bodyHandle, { 5, 6 } );
        physics.ApplyForceToCenter( shape.bodyHandle, { 100, 200 } );
        trial.resetCollisionTrial();
        check( physics.IsValid( shape.bodyHandle ) && physics.IsValid( shape.shapeHandle ) && same( physics.GetBodyTransform( shape.bodyHandle ).position, pose.position ) && physics.GetBodyTransform( shape.bodyHandle ).rotation.c == pose.rotation.c && physics.GetBodyTransform( shape.bodyHandle ).rotation.s == pose.rotation.s && same( physics.GetBodyLinearVelocity( shape.bodyHandle ), launch ), "all trial resets preserve handles and restore initial pose and velocity" );
        check( physics.GetShapeFriction( shape.shapeHandle ) == 0.6f && physics.GetShapeRestitution( shape.shapeHandle ) == 0.4f && physics.GetShapeDensity( shape.shapeHandle ) == 2.5f && physics.GetBodyMass( shape.bodyHandle ) == mass && physics.GetShapeFilter( shape.shapeHandle ).categoryBits == 3 && physics.GetShapeFilter( shape.shapeHandle ).maskBits == 5 && !physics.IsBodySleepEnabled( shape.bodyHandle ) && physics.GetBodySleepThreshold( shape.bodyHandle ) == 0.2f && physics.IsBodyAwake( shape.bodyHandle ), "all trial resets preserve edited materials filters mass and sleep settings while waking bodies" );
        check( !playback.isPlaying(), "trial reset retains pause state" );
        playback.setPlaying( true );
        trial.resetCollisionTrial();
        check( playback.isPlaying(), "trial reset retains playing state" );
        playback.reset();
        check( !playback.isPlaying() && !static_cast<rigidBodyDemo&>( playback.getDemo() ).getWorld().IsValid( shape.bodyHandle ), "full reset still replaces the scene and pauses" );
    }
}
