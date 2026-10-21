#include "rigidBodyDemo.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>
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

void checkSleepWakeDemo()
{
    for( int substeps : { 1, 4, 8 } )
    {
        demoSession playback{ createRigidBodyDemo };
        playback.selectDemo( demoKind::sleepWakeComparison );
        auto& scene = static_cast<rigidBodyDemo&>( playback.getDemo() );
        auto& simulation = scene.getWorld();
        check( scene.getShapes().size() == 3 && simulation.GetBodyCount() == 3, "sleep comparison creates one floor and two editable boxes" );
        const auto left = scene.getShapes()[1];
        const auto right = scene.getShapes()[2];
        const float threshold = simulation.GetBodySleepThreshold( left.bodyHandle );
        for( int frame = 0; frame < 240; ++frame )
            playback.stepOnce( substeps );
        check( !simulation.IsBodyAwake( left.bodyHandle ) && simulation.IsBodyAwake( right.bodyHandle ), "only the sleep-enabled left box sleeps after the identical drops settle" );
        check( std::abs( simulation.GetBodyTransform( left.bodyHandle ).position.y - 0.5f ) < 0.1f && std::abs( simulation.GetBodyTransform( right.bodyHandle ).position.y - 0.5f ) < 0.1f && std::abs( simulation.GetBodyTransform( left.bodyHandle ).position.x + 2.0f ) < 0.1f && std::abs( simulation.GetBodyTransform( right.bodyHandle ).position.x - 2.0f ) < 0.1f, "both sleep trials remain supported at rest on the same floor" );
        simulation.SetBodySleepEnabled( left.bodyHandle, false );
        simulation.SetBodySleepEnabled( right.bodyHandle, true );
        scene.resetCollisionTrial();
        check( !simulation.IsBodySleepEnabled( left.bodyHandle ) && simulation.IsBodySleepEnabled( right.bodyHandle ) && simulation.IsBodyAwake( left.bodyHandle ) && simulation.IsBodyAwake( right.bodyHandle ), "sleep reset wakes both boxes and retains swapped sleep settings" );
        for( int frame = 0; frame < 240; ++frame )
            playback.stepOnce( substeps );
        check( simulation.IsBodyAwake( left.bodyHandle ) && !simulation.IsBodyAwake( right.bodyHandle ), "swapping sleep permission swaps which box sleeps in the repeated trial" );
        demoInput drag{};
        drag.mousePosition = simulation.GetBodyTransform( right.bodyHandle ).position;
        drag.mousePressed = drag.mouseHeld = true;
        scene.handleInput( drag );
        check( simulation.IsValid( scene.getMouseJoint() ) && simulation.IsBodyAwake( right.bodyHandle ), "dragging the sleeping box creates the existing mouse joint and wakes it" );
        drag.mousePressed = false;
        drag.mousePosition.y += 0.75f;
        scene.handleInput( drag );
        for( int frame = 0; frame < 30; ++frame )
            playback.stepOnce( substeps );
        check( simulation.IsBodyAwake( right.bodyHandle ) && simulation.GetBodyTransform( right.bodyHandle ).position.y > 0.6f, "held drag moves the awake box away from its resting pose" );
        scene.handleInput( {} );
        check( !simulation.IsValid( scene.getMouseJoint() ), "releasing the sleep demo drag removes the temporary joint" );
        for( int frame = 0; frame < 240; ++frame )
            playback.stepOnce( substeps );
        check( !simulation.IsBodyAwake( right.bodyHandle ), "released sleep-enabled box returns to sleep after settling again" );
        drag.mousePressed = drag.mouseHeld = true;
        drag.mousePosition = simulation.GetBodyTransform( right.bodyHandle ).position;
        scene.handleInput( drag );
        simulation.SetBodySleepThreshold( left.bodyHandle, 0.12f );
        simulation.SetBodySleepThreshold( right.bodyHandle, 0.08f );
        collisionMatrix matrix;
        matrix.setPair( 2, 3, false );
        playback.setCollisionMatrix( matrix );
        playback.setPlaying( true );
        scene.resetCollisionTrial();
        check( playback.isPlaying() && playback.getCollisionMatrix() == matrix && !simulation.IsValid( scene.getMouseJoint() ) && simulation.IsValid( left.shapeHandle ) && simulation.IsValid( right.shapeHandle ), "sleep reset cancels dragging and preserves handles common settings and play state" );
        check( simulation.IsBodyAwake( left.bodyHandle ) && simulation.IsBodyAwake( right.bodyHandle ) && same( simulation.GetBodyTransform( left.bodyHandle ).position, { -2, 1.5f } ) && same( simulation.GetBodyTransform( right.bodyHandle ).position, { 2, 1.5f } ) && same( simulation.GetBodyLinearVelocity( left.bodyHandle ), {} ) && same( simulation.GetBodyLinearVelocity( right.bodyHandle ), {} ) && simulation.GetBodySleepThreshold( left.bodyHandle ) == 0.12f && simulation.GetBodySleepThreshold( right.bodyHandle ) == 0.08f, "sleep reset restores initial awake poses and velocities while preserving edited thresholds" );
        playback.setPlaying( false );
        simulation.SetSleepingEnabled( false );
        scene.resetCollisionTrial();
        for( int frame = 0; frame < 240; ++frame )
            playback.stepOnce( substeps );
        check( !simulation.IsSleepingEnabled() && simulation.IsBodyAwake( left.bodyHandle ) && simulation.IsBodyAwake( right.bodyHandle ), "trial reset retains disabled world sleep so neither box sleeps" );
        playback.reset();
        auto& defaults = static_cast<rigidBodyDemo&>( playback.getDemo() );
        const auto defaultLeft = defaults.getShapes()[1].bodyHandle;
        const auto defaultRight = defaults.getShapes()[2].bodyHandle;
        check( !playback.isPlaying() && playback.getCollisionMatrix() == matrix && !defaults.getWorld().IsValid( left.bodyHandle ) && defaults.getWorld().IsSleepingEnabled() && defaults.getWorld().IsBodySleepEnabled( defaultLeft ) && !defaults.getWorld().IsBodySleepEnabled( defaultRight ) && defaults.getWorld().GetBodySleepThreshold( defaultLeft ) == threshold, "full sleep reset restores scene defaults with fresh handles while retaining common settings" );
    }
}

void checkDampingDemo()
{
    for( int substeps : { 1, 4, 8 } )
    {
        demoSession playback{ createRigidBodyDemo };
        playback.selectDemo( demoKind::dampingComparison );
        auto& scene = static_cast<rigidBodyDemo&>( playback.getDemo() );
        auto& simulation = scene.getWorld();
        check( scene.getShapes().size() == 3 && simulation.GetBodyCount() == 3 && same( simulation.GetGravity(), {} ), "damping comparison creates three editable boxes without gravity" );
        const auto visuals = std::vector<visualShape>{ scene.getShapes().begin(), scene.getShapes().end() };
        for( int frame = 0; frame < 60; ++frame )
            playback.stepOnce( substeps );
        float speeds[3]{}, spins[3]{};
        for( std::size_t i = 0; i < 3; ++i )
        {
            speeds[i] = Length( simulation.GetBodyLinearVelocity( visuals[i].bodyHandle ) );
            spins[i] = simulation.GetBodyAngularVelocity( visuals[i].bodyHandle );
        }
        check( speeds[0] > speeds[1] + 0.4f && speeds[1] > speeds[2] + 0.4f && spins[0] > spins[1] + 0.4f && spins[1] > spins[2] + 0.4f, "increasing damping visibly reduces both linear and angular speed" );
        check( speeds[0] == 2.0f && spins[0] == 2.0f && simulation.GetContactCount() == 0, "zero damping preserves both launch speeds without contacts" );
        const auto middle = visuals[1];
        simulation.SetBodyLinearDamping( middle.bodyHandle, 2.0f );
        scene.resetCollisionTrial();
        for( int frame = 0; frame < 60; ++frame )
            playback.stepOnce( substeps );
        check( Length( simulation.GetBodyLinearVelocity( middle.bodyHandle ) ) < speeds[1] - 0.4f && std::abs( simulation.GetBodyAngularVelocity( middle.bodyHandle ) - spins[1] ) < 1.0e-6f, "editing only linear damping changes translation while preserving angular speed" );
        simulation.SetBodyLinearDamping( middle.bodyHandle, 0.5f );
        simulation.SetBodyAngularDamping( middle.bodyHandle, 2.0f );
        scene.resetCollisionTrial();
        for( int frame = 0; frame < 60; ++frame )
            playback.stepOnce( substeps );
        check( std::abs( Length( simulation.GetBodyLinearVelocity( middle.bodyHandle ) ) - speeds[1] ) < 1.0e-6f && simulation.GetBodyAngularVelocity( middle.bodyHandle ) < spins[1] - 0.4f, "editing only angular damping changes rotation while preserving linear speed" );
        for( int frame = 0; frame < 180; ++frame )
            playback.stepOnce( substeps );
        for( const auto& visual : visuals )
            check( !simulation.IsBodySleepEnabled( visual.bodyHandle ) && simulation.IsBodyAwake( visual.bodyHandle ) && Length( simulation.GetBodyLinearVelocity( visual.bodyHandle ) ) > 0.0f && simulation.GetBodyAngularVelocity( visual.bodyHandle ) > 0.0f, "automatic sleep does not replace the damping comparison with a stopped body" );
        demoInput drag{};
        drag.mousePosition = simulation.GetBodyTransform( middle.bodyHandle ).position;
        drag.mousePressed = drag.mouseHeld = true;
        scene.handleInput( drag );
        check( simulation.IsValid( scene.getMouseJoint() ), "damping boxes retain existing drag interaction" );
        simulation.SetBodySleepEnabled( middle.bodyHandle, true );
        simulation.SetGravity( { 1, -1 } );
        collisionMatrix matrix;
        matrix.setPair( 2, 3, false );
        playback.setCollisionMatrix( matrix );
        playback.setPlaying( true );
        scene.resetCollisionTrial();
        check( playback.isPlaying() && playback.getCollisionMatrix() == matrix && !simulation.IsValid( scene.getMouseJoint() ) && same( simulation.GetGravity(), { 1, -1 } ) && simulation.GetBodyLinearDamping( middle.bodyHandle ) == 0.5f && simulation.GetBodyAngularDamping( middle.bodyHandle ) == 2.0f && simulation.IsBodySleepEnabled( middle.bodyHandle ), "damping reset cancels drag while retaining edited coefficients sleep gravity common settings and play state" );
        for( std::size_t i = 0; i < visuals.size(); ++i )
        {
            const auto body = visuals[i].bodyHandle;
            const auto pose = simulation.GetBodyTransform( body );
            check( simulation.IsValid( visuals[i].shapeHandle ) && same( pose.position, { -3.0f + 3.0f * static_cast<float>( i ), 0 } ) && pose.rotation.c == 1.0f && pose.rotation.s == 0.0f && same( simulation.GetBodyLinearVelocity( body ), { 0, 2 } ) && simulation.GetBodyAngularVelocity( body ) == 2.0f, "damping reset retains handles and restores initial transforms and launch velocities" );
        }
        playback.reset();
        auto& defaults = static_cast<rigidBodyDemo&>( playback.getDemo() );
        const auto defaultMiddle = defaults.getShapes()[1].bodyHandle;
        check( !playback.isPlaying() && playback.getCollisionMatrix() == matrix && !defaults.getWorld().IsValid( middle.bodyHandle ) && same( defaults.getWorld().GetGravity(), {} ) && !defaults.getWorld().IsBodySleepEnabled( defaultMiddle ) && defaults.getWorld().GetBodyLinearDamping( defaultMiddle ) == 0.5f && defaults.getWorld().GetBodyAngularDamping( defaultMiddle ) == 0.5f, "full damping reset restores default coefficients gravity and sleep settings while retaining common settings" );
    }
}

void checkGravityScaleDemo()
{
    for( int substeps : { 1, 4, 8 } )
    {
        demoSession playback{ createRigidBodyDemo };
        playback.selectDemo( demoKind::gravityScaleComparison );
        auto& scene = static_cast<rigidBodyDemo&>( playback.getDemo() );
        auto& simulation = scene.getWorld();
        check( scene.getShapes().size() == 3 && simulation.GetBodyCount() == 3, "gravity comparison creates three editable boxes without a floor" );
        const auto visuals = std::vector<visualShape>{ scene.getShapes().begin(), scene.getShapes().end() };
        for( int frame = 0; frame < 60; ++frame )
            playback.stepOnce( substeps );
        const auto first = visuals[0].bodyHandle;
        const auto middle = visuals[1].bodyHandle;
        const auto last = visuals[2].bodyHandle;
        const float middleY = simulation.GetBodyTransform( middle ).position.y;
        const float lastY = simulation.GetBodyTransform( last ).position.y;
        check( same( simulation.GetBodyTransform( first ).position, { -3, 3 } ) && same( simulation.GetBodyLinearVelocity( first ), {} ) && middleY < 3.0f && lastY < middleY && std::abs( ( 3.0f - middleY ) - 0.5f * ( 3.0f - lastY ) ) < 2.0e-4f, "zero gravity scale stays still while half scale falls half as far as full scale" );
        check( std::abs( simulation.GetBodyLinearVelocity( middle ).y + 5.0f ) < 2.0e-4f && std::abs( simulation.GetBodyLinearVelocity( last ).y + 10.0f ) < 2.0e-4f && simulation.GetContactCount() == 0, "gravity scales independently control falling velocity without contacts" );
        for( const auto& visual : visuals )
            check( !simulation.IsBodySleepEnabled( visual.bodyHandle ) && simulation.IsBodyAwake( visual.bodyHandle ) && simulation.GetBodyAngularVelocity( visual.bodyHandle ) == 0.0f, "sleep and rotation do not interfere with the gravity comparison" );
        simulation.SetBodyGravityScale( middle, -1.0f );
        scene.resetCollisionTrial();
        for( int frame = 0; frame < 60; ++frame )
            playback.stepOnce( substeps );
        check( simulation.GetBodyTransform( middle ).position.y > 3.0f && std::abs( simulation.GetBodyLinearVelocity( middle ).y - 10.0f ) < 2.0e-4f && simulation.GetBodyTransform( last ).position.y < 3.0f, "edited negative gravity scale moves opposite to the positive-scale box" );
        simulation.SetGravity( { 0, 4 } );
        scene.resetCollisionTrial();
        for( int frame = 0; frame < 60; ++frame )
            playback.stepOnce( substeps );
        check( same( simulation.GetGravity(), { 0, 4 } ) && simulation.GetBodyTransform( middle ).position.y < 3.0f && simulation.GetBodyTransform( last ).position.y > 3.0f && std::abs( simulation.GetBodyLinearVelocity( middle ).y + 4.0f ) < 2.0e-4f && std::abs( simulation.GetBodyLinearVelocity( last ).y - 4.0f ) < 2.0e-4f && simulation.GetBodyTransform( first ).position.y == 3.0f, "edited world gravity is retained by reset and reverses both nonzero scales" );
        simulation.SetGravity( {} );
        scene.resetCollisionTrial();
        for( int frame = 0; frame < 60; ++frame )
            playback.stepOnce( substeps );
        for( const auto& visual : visuals )
            check( simulation.GetBodyTransform( visual.bodyHandle ).position.y == 3.0f && same( simulation.GetBodyLinearVelocity( visual.bodyHandle ), {} ), "zero world gravity keeps every scale at rest after reset" );
        demoInput drag{};
        drag.mousePosition = simulation.GetBodyTransform( middle ).position;
        drag.mousePressed = drag.mouseHeld = drag.left = true;
        scene.handleInput( drag );
        check( simulation.IsValid( scene.getMouseJoint() ), "gravity comparison retains existing drag interaction" );
        simulation.ApplyForceToCenter( first, { 300, 0 } );
        collisionMatrix matrix;
        matrix.setPair( 2, 3, false );
        playback.setCollisionMatrix( matrix );
        playback.setPlaying( true );
        scene.resetCollisionTrial();
        check( playback.isPlaying() && playback.getCollisionMatrix() == matrix && !simulation.IsValid( scene.getMouseJoint() ) && same( simulation.GetGravity(), {} ) && simulation.GetBodyGravityScale( middle ) == -1.0f, "gravity reset cancels drag and preserves edited scale world gravity common settings and play state" );
        for( std::size_t i = 0; i < visuals.size(); ++i )
        {
            const auto body = visuals[i].bodyHandle;
            const auto pose = simulation.GetBodyTransform( body );
            check( simulation.IsValid( visuals[i].shapeHandle ) && same( pose.position, { -3.0f + 3.0f * static_cast<float>( i ), 3 } ) && pose.rotation.c == 1.0f && pose.rotation.s == 0.0f && same( simulation.GetBodyLinearVelocity( body ), {} ) && simulation.GetBodyAngularVelocity( body ) == 0.0f, "gravity reset retains handles and restores initial transforms and zero velocities" );
        }
        playback.setPlaying( false );
        playback.stepOnce( substeps );
        check( same( simulation.GetBodyLinearVelocity( first ), {} ), "gravity reset clears pending force and held input before the next step" );
        playback.reset();
        auto& defaults = static_cast<rigidBodyDemo&>( playback.getDemo() );
        const auto defaultMiddle = defaults.getShapes()[1].bodyHandle;
        check( !playback.isPlaying() && playback.getCollisionMatrix() == matrix && !defaults.getWorld().IsValid( middle ) && same( defaults.getWorld().GetGravity(), { 0, -10 } ) && !defaults.getWorld().IsBodySleepEnabled( defaultMiddle ) && defaults.getWorld().GetBodyGravityScale( defaultMiddle ) == 0.5f, "full gravity reset restores default world gravity scale and sleep settings while retaining common settings" );
    }
}

void checkMassImpulseDemo()
{
    for( int substeps : { 1, 4, 8 } )
    {
        demoSession playback{ createRigidBodyDemo };
        playback.selectDemo( demoKind::massImpulseComparison );
        auto& scene = static_cast<rigidBodyDemo&>( playback.getDemo() );
        auto& simulation = scene.getWorld();
        check( scene.getShapes().size() == 3 && simulation.GetBodyCount() == 3 && same( simulation.GetGravity(), {} ), "mass comparison creates three editable boxes without gravity" );
        const auto visuals = std::vector<visualShape>{ scene.getShapes().begin(), scene.getShapes().end() };
        const float masses[] = { 1.0f, 2.0f, 4.0f };
        const float speeds[] = { 2.0f, 1.0f, 0.5f };
        scene.applyMassTrialImpulse();
        for( std::size_t i = 0; i < visuals.size(); ++i )
        {
            const auto body = visuals[i].bodyHandle;
            check( simulation.GetBodyMass( body ) == masses[i] && same( simulation.GetBodyLinearVelocity( body ), { 0, speeds[i] } ), "equal center impulse immediately produces inverse-mass velocity changes" );
            check( same( simulation.GetBodyTransform( body ).position, { -3.0f + 3.0f * static_cast<float>( i ), 0 } ) && simulation.GetBodyAngularVelocity( body ) == 0.0f && !simulation.IsBodySleepEnabled( body ), "paused impulse does not move rotate or enable sleep for the comparison boxes" );
        }
        check( !playback.isPlaying(), "applying a mass trial impulse preserves pause state" );
        scene.applyMassTrialImpulse();
        for( std::size_t i = 0; i < visuals.size(); ++i )
            check( same( simulation.GetBodyLinearVelocity( visuals[i].bodyHandle ), { 0, 2.0f * speeds[i] } ), "repeated equal impulses accumulate without resetting velocity" );
        const auto middle = visuals[1];
        simulation.SetShapeDensity( middle.shapeHandle, 4.0f );
        scene.resetCollisionTrial();
        scene.applyMassTrialImpulse();
        check( simulation.GetShapeDensity( middle.shapeHandle ) == 4.0f && simulation.GetBodyMass( middle.bodyHandle ) == 4.0f && same( simulation.GetBodyLinearVelocity( middle.bodyHandle ), { 0, 0.5f } ), "edited density survives reset and the next impulse uses the current mass" );
        simulation.SetBodyAngularVelocity( middle.bodyHandle, 1.0f );
        scene.applyMassTrialImpulse();
        check( same( simulation.GetBodyLinearVelocity( middle.bodyHandle ), { 0, 1 } ) && simulation.GetBodyAngularVelocity( middle.bodyHandle ) == 1.0f, "center impulse accumulates translation without changing an edited spin" );
        for( int frame = 0; frame < 60; ++frame )
            playback.stepOnce( substeps );
        check( std::abs( simulation.GetBodyTransform( visuals[0].bodyHandle ).position.y - 4.0f ) < 2.0e-4f && std::abs( simulation.GetBodyTransform( middle.bodyHandle ).position.y - 1.0f ) < 2.0e-4f && std::abs( simulation.GetBodyTransform( visuals[2].bodyHandle ).position.y - 1.0f ) < 2.0e-4f && simulation.GetContactCount() == 0, "subsequent steps move the impulse trial without gravity damping or contacts" );
        demoInput drag{};
        drag.mousePosition = simulation.GetBodyTransform( middle.bodyHandle ).position;
        drag.mousePressed = drag.mouseHeld = true;
        scene.handleInput( drag );
        check( simulation.IsValid( scene.getMouseJoint() ), "mass comparison retains existing drag interaction" );
        simulation.SetGravity( { 0, -1 } );
        simulation.SetBodyLinearDamping( middle.bodyHandle, 0.2f );
        simulation.SetBodyAngularDamping( middle.bodyHandle, 0.3f );
        simulation.SetBodySleepEnabled( middle.bodyHandle, true );
        collisionMatrix matrix;
        matrix.setPair( 2, 3, false );
        playback.setCollisionMatrix( matrix );
        playback.setPlaying( true );
        scene.resetCollisionTrial();
        check( playback.isPlaying() && playback.getCollisionMatrix() == matrix && !simulation.IsValid( scene.getMouseJoint() ) && same( simulation.GetGravity(), { 0, -1 } ) && simulation.GetBodyLinearDamping( middle.bodyHandle ) == 0.2f && simulation.GetBodyAngularDamping( middle.bodyHandle ) == 0.3f && simulation.IsBodySleepEnabled( middle.bodyHandle ) && simulation.GetBodyMass( middle.bodyHandle ) == 4.0f, "mass reset cancels drag while preserving edited density damping sleep gravity common settings and play state" );
        for( std::size_t i = 0; i < visuals.size(); ++i )
        {
            const auto body = visuals[i].bodyHandle;
            const auto pose = simulation.GetBodyTransform( body );
            check( simulation.IsValid( visuals[i].shapeHandle ) && same( pose.position, { -3.0f + 3.0f * static_cast<float>( i ), 0 } ) && pose.rotation.c == 1.0f && pose.rotation.s == 0.0f && same( simulation.GetBodyLinearVelocity( body ), {} ) && simulation.GetBodyAngularVelocity( body ) == 0.0f, "mass reset retains handles and restores initial transforms and zero velocities without relaunching" );
        }
        scene.applyMassTrialImpulse();
        check( same( simulation.GetBodyLinearVelocity( middle.bodyHandle ), { 0, 0.5f } ) && simulation.GetBodyTransform( middle.bodyHandle ).position.y == 0.0f, "impulse remains instantaneous after world and body settings are edited" );
        playback.reset();
        auto& defaults = static_cast<rigidBodyDemo&>( playback.getDemo() );
        const auto defaultMiddle = defaults.getShapes()[1].bodyHandle;
        check( !playback.isPlaying() && playback.getCollisionMatrix() == matrix && !defaults.getWorld().IsValid( middle.bodyHandle ) && same( defaults.getWorld().GetGravity(), {} ) && !defaults.getWorld().IsBodySleepEnabled( defaultMiddle ) && defaults.getWorld().GetBodyMass( defaultMiddle ) == 2.0f && same( defaults.getWorld().GetBodyLinearVelocity( defaultMiddle ), {} ), "full mass reset restores default density and resting state while retaining common settings" );
    }
    rigidBodyDemo other{ demoKind::collisionComparison };
    const auto body = other.getShapes()[0].bodyHandle;
    const auto velocity = other.getWorld().GetBodyLinearVelocity( body );
    other.applyMassTrialImpulse();
    check( same( other.getWorld().GetBodyLinearVelocity( body ), velocity ), "mass trial action does not alter other scenes" );
}

void checkImpulsePointDemo()
{
    for( int substeps : { 1, 4, 8 } )
    {
        demoSession playback{ createRigidBodyDemo };
        playback.selectDemo( demoKind::impulsePointComparison );
        auto& scene = static_cast<rigidBodyDemo&>( playback.getDemo() );
        auto& simulation = scene.getWorld();
        check( scene.getShapes().size() == 3 && simulation.GetBodyCount() == 3 && same( simulation.GetGravity(), {} ), "point comparison creates three identical editable boxes without gravity" );
        const auto visuals = std::vector<visualShape>{ scene.getShapes().begin(), scene.getShapes().end() };
        const float spins[] = { 0.0f, -6.0f, 6.0f };
        const vec2 points[] = { { -3, 0 }, { -0.5f, 0 }, { 3.5f, 0 } };
        scene.applyPointTrialImpulse();
        for( std::size_t i = 0; i < visuals.size(); ++i )
        {
            const auto body = visuals[i].bodyHandle;
            check( same( simulation.GetBodyLinearVelocity( body ), { 0, 2 } ) && std::abs( simulation.GetBodyAngularVelocity( body ) - spins[i] ) < 1.0e-5f, "equal impulse gives equal translation with zero center spin and opposite side spins" );
            const auto point = scene.getImpulseTrialPoint( i );
            check( point && same( *point, points[i] ) && simulation.GetBodyTransform( body ).position.y == 0.0f && !simulation.IsBodySleepEnabled( body ), "point query exposes the initial center and local side offsets without advancing motion" );
        }
        check( !playback.isPlaying() && !scene.getImpulseTrialPoint( 3 ), "point action preserves pause state and invalid point queries fail" );
        scene.applyPointTrialImpulse();
        for( std::size_t i = 0; i < visuals.size(); ++i )
            check( same( simulation.GetBodyLinearVelocity( visuals[i].bodyHandle ), { 0, 4 } ) && std::abs( simulation.GetBodyAngularVelocity( visuals[i].bodyHandle ) - 2.0f * spins[i] ) < 1.0e-5f, "repeated point impulses accumulate both linear and angular changes" );
        for( int frame = 0; frame < 60; ++frame )
            playback.stepOnce( substeps );
        for( std::size_t i = 0; i < visuals.size(); ++i )
            check( std::abs( simulation.GetBodyTransform( visuals[i].bodyHandle ).position.y - 4.0f ) < 2.0e-4f && std::abs( simulation.GetBodyAngularVelocity( visuals[i].bodyHandle ) - 2.0f * spins[i] ) < 1.0e-5f, "point trial steps preserve undamped motion without contacts" );
        scene.resetCollisionTrial();
        const auto left = visuals[1];
        const auto right = visuals[2];
        simulation.SetBodyTransform( left.bodyHandle, { { 1, 2 }, rot2::FromRadians( 0.5f * std::numbers::pi_v<float> ) } );
        simulation.SetBodyTransform( right.bodyHandle, { { 4, 2 }, rot2::FromRadians( std::numbers::pi_v<float> ) } );
        const auto leftPoint = scene.getImpulseTrialPoint( 1 );
        const auto rightPoint = scene.getImpulseTrialPoint( 2 );
        check( leftPoint && rightPoint && Length( *leftPoint - vec2{ 1, 1.5f } ) < 1.0e-6f && Length( *rightPoint - vec2{ 3.5f, 2 } ) < 1.0e-6f, "point query follows edited body translation and rotation" );
        scene.applyPointTrialImpulse();
        check( same( simulation.GetBodyLinearVelocity( left.bodyHandle ), { 0, 2 } ) && same( simulation.GetBodyLinearVelocity( right.bodyHandle ), { 0, 2 } ) && std::abs( simulation.GetBodyAngularVelocity( left.bodyHandle ) ) < 1.0e-5f && std::abs( simulation.GetBodyAngularVelocity( right.bodyHandle ) + 6.0f ) < 1.0e-5f, "rotated local points change torque while the impulse remains world-up" );
        simulation.SetShapeDensity( right.shapeHandle, 2.0f );
        scene.resetCollisionTrial();
        scene.applyPointTrialImpulse();
        check( simulation.GetBodyMass( right.bodyHandle ) == 2.0f && same( simulation.GetBodyLinearVelocity( right.bodyHandle ), { 0, 1 } ) && std::abs( simulation.GetBodyAngularVelocity( right.bodyHandle ) - 3.0f ) < 1.0e-5f, "point impulse uses the edited mass and rotational inertia after reset" );
        demoInput drag{};
        drag.mousePosition = simulation.GetBodyTransform( left.bodyHandle ).position;
        drag.mousePressed = drag.mouseHeld = true;
        scene.handleInput( drag );
        check( simulation.IsValid( scene.getMouseJoint() ), "point comparison retains existing drag interaction" );
        simulation.SetGravity( { 0, -1 } );
        simulation.SetBodyLinearDamping( right.bodyHandle, 0.2f );
        simulation.SetBodyAngularDamping( right.bodyHandle, 0.3f );
        simulation.SetBodySleepEnabled( right.bodyHandle, true );
        collisionMatrix matrix;
        matrix.setPair( 2, 3, false );
        playback.setCollisionMatrix( matrix );
        playback.setPlaying( true );
        scene.resetCollisionTrial();
        check( playback.isPlaying() && playback.getCollisionMatrix() == matrix && !simulation.IsValid( scene.getMouseJoint() ) && same( simulation.GetGravity(), { 0, -1 } ) && simulation.GetBodyLinearDamping( right.bodyHandle ) == 0.2f && simulation.GetBodyAngularDamping( right.bodyHandle ) == 0.3f && simulation.IsBodySleepEnabled( right.bodyHandle ) && simulation.GetBodyMass( right.bodyHandle ) == 2.0f, "point reset cancels drag and preserves density body world common settings and play state" );
        for( std::size_t i = 0; i < visuals.size(); ++i )
        {
            const auto body = visuals[i].bodyHandle;
            const auto pose = simulation.GetBodyTransform( body );
            const auto point = scene.getImpulseTrialPoint( i );
            check( simulation.IsValid( visuals[i].shapeHandle ) && same( pose.position, { -3.0f + 3.0f * static_cast<float>( i ), 0 } ) && pose.rotation.c == 1.0f && pose.rotation.s == 0.0f && same( simulation.GetBodyLinearVelocity( body ), {} ) && simulation.GetBodyAngularVelocity( body ) == 0.0f && point && same( *point, points[i] ), "point reset restores initial transforms zero velocities and point locations without relaunching" );
        }
        playback.reset();
        auto& defaults = static_cast<rigidBodyDemo&>( playback.getDemo() );
        const auto defaultRight = defaults.getShapes()[2].bodyHandle;
        check( !playback.isPlaying() && playback.getCollisionMatrix() == matrix && !defaults.getWorld().IsValid( right.bodyHandle ) && same( defaults.getWorld().GetGravity(), {} ) && !defaults.getWorld().IsBodySleepEnabled( defaultRight ) && defaults.getWorld().GetBodyMass( defaultRight ) == 1.0f && same( defaults.getWorld().GetBodyLinearVelocity( defaultRight ), {} ), "full point reset restores resting scene defaults while retaining common settings" );
    }
    rigidBodyDemo other{ demoKind::collisionComparison };
    const auto body = other.getShapes()[0].bodyHandle;
    const auto velocity = other.getWorld().GetBodyLinearVelocity( body );
    other.applyPointTrialImpulse();
    check( !other.getImpulseTrialPoint( 0 ) && same( other.getWorld().GetBodyLinearVelocity( body ), velocity ), "point trial queries and actions do not alter other scenes" );
}

void checkRotationalInertiaDemo()
{
    for( int substeps : { 1, 4, 8 } )
    {
        demoSession playback{ createRigidBodyDemo };
        playback.selectDemo( demoKind::rotationalInertiaComparison );
        auto& scene = static_cast<rigidBodyDemo&>( playback.getDemo() );
        auto& simulation = scene.getWorld();
        check( scene.getShapes().size() == 2 && simulation.GetBodyCount() == 2 && same( simulation.GetGravity(), {} ), "inertia comparison creates two bodies without gravity" );
        const auto square = scene.getShapes()[0];
        const auto bar = scene.getShapes()[1];
        const float inertias[] = { 1.0f / 6.0f, 17.0f / 48.0f };
        const float spins[] = { 6.0f, 48.0f / 17.0f };
        for( std::size_t i = 0; i < 2; ++i )
        {
            const auto body = scene.getShapes()[i].bodyHandle;
            check( simulation.GetBodyMass( body ) == 1.0f && std::abs( simulation.GetBodyRotationalInertia( body ) - inertias[i] ) < 1.0e-6f && !simulation.IsBodySleepEnabled( body ), "equal mass shapes have different analytic inertias and no automatic sleep" );
        }
        scene.applyInertiaTrialImpulse();
        for( std::size_t i = 0; i < 2; ++i )
        {
            const auto body = scene.getShapes()[i].bodyHandle;
            check( std::abs( simulation.GetBodyAngularVelocity( body ) - spins[i] ) < 1.0e-5f && same( simulation.GetBodyLinearVelocity( body ), {} ), "equal angular impulse changes spin according to inertia without translation" );
            check( same( simulation.GetBodyTransform( body ).position, { i == 0 ? -2.5f : 2.5f, 0 } ) && simulation.GetBodyTransform( body ).rotation.s == 0.0f, "paused angular impulse leaves poses unchanged" );
        }
        check( !playback.isPlaying(), "inertia action preserves pause state" );
        scene.applyInertiaTrialImpulse();
        for( int frame = 0; frame < 60; ++frame )
            playback.stepOnce( substeps );
        for( std::size_t i = 0; i < 2; ++i )
        {
            const auto body = scene.getShapes()[i].bodyHandle;
            check( std::abs( simulation.GetBodyAngularVelocity( body ) - 2.0f * spins[i] ) < 1.0e-5f && same( simulation.GetBodyTransform( body ).position, { i == 0 ? -2.5f : 2.5f, 0 } ) && std::abs( simulation.GetBodyTransform( body ).rotation.s ) > 0.1f, "repeated angular impulses accumulate and rotate bodies in place without damping" );
        }
        simulation.SetBodyLinearVelocity( bar.bodyHandle, { 2, -3 } );
        scene.applyInertiaTrialImpulse();
        check( same( simulation.GetBodyLinearVelocity( bar.bodyHandle ), { 2, -3 } ), "angular impulse preserves an edited nonzero linear velocity" );
        simulation.SetShapeDensity( bar.shapeHandle, 2.0f );
        scene.resetCollisionTrial();
        check( simulation.GetBodyMass( bar.bodyHandle ) == 2.0f && std::abs( simulation.GetBodyRotationalInertia( bar.bodyHandle ) - 17.0f / 24.0f ) < 1.0e-6f && simulation.GetBodyAngularVelocity( bar.bodyHandle ) == 0.0f && same( simulation.GetBodyLinearVelocity( bar.bodyHandle ), {} ) && simulation.GetBodyTransform( bar.bodyHandle ).rotation.s == 0.0f, "trial reset restores pose and rest while preserving edited density and handles" );
        scene.applyInertiaTrialImpulse();
        check( std::abs( simulation.GetBodyAngularVelocity( square.bodyHandle ) - 6.0f ) < 1.0e-5f && std::abs( simulation.GetBodyAngularVelocity( bar.bodyHandle ) - 24.0f / 17.0f ) < 1.0e-5f, "angular impulse uses current inertia after density editing and reset" );
        playback.reset();
        auto& defaults = static_cast<rigidBodyDemo&>( playback.getDemo() );
        const auto body = defaults.getShapes()[1].bodyHandle;
        check( !defaults.getWorld().IsValid( bar.bodyHandle ) && defaults.getWorld().GetBodyMass( body ) == 1.0f && defaults.getWorld().GetBodyAngularVelocity( body ) == 0.0f && !playback.isPlaying(), "full inertia reset restores default density and resting state" );
    }
    rigidBodyDemo other{ demoKind::collisionComparison };
    const auto body = other.getShapes()[0].bodyHandle;
    other.applyInertiaTrialImpulse();
    check( other.getWorld().GetBodyAngularVelocity( body ) == 0.0f, "inertia action does not alter other scenes" );
}

void checkForceImpulseDemo()
{
    for( int substeps : { 1, 4, 8 } )
    {
        demoSession playback{ createRigidBodyDemo };
        playback.selectDemo( demoKind::forceImpulseComparison );
        auto& scene = static_cast<rigidBodyDemo&>( playback.getDemo() );
        auto& physics = scene.getWorld();
        check( scene.getShapes().size() == 2 && physics.GetBodyCount() == 2 && same( physics.GetGravity(), {} ), "force trial creates two identical resting boxes without gravity" );
        const auto force = scene.getShapes()[0];
        const auto impulse = scene.getShapes()[1];
        for( const auto& visual : scene.getShapes() )
            check( physics.GetBodyMass( visual.bodyHandle ) == 1.0f && !physics.IsBodySleepEnabled( visual.bodyHandle ) && same( physics.GetBodyLinearVelocity( visual.bodyHandle ), {} ), "force comparison starts with equal mass and sleep disabled" );
        playback.stepOnce( substeps );
        check( scene.getForceTrialTime() == 0.0 && same( physics.GetBodyLinearVelocity( force.bodyHandle ), {} ), "force trial waits for explicit start" );
        scene.startForceImpulseTrial();
        check( !playback.isPlaying() && scene.getForceTrialTime() == 0.0 && same( physics.GetBodyLinearVelocity( force.bodyHandle ), {} ) && same( physics.GetBodyLinearVelocity( impulse.bodyHandle ), { 0, 2 } ) && physics.GetBodyTransform( impulse.bodyHandle ).position.y == 0.0f, "start applies only the instantaneous impulse without advancing time or poses" );
        playback.advance( 0.5f, substeps );
        scene.step( 0.0f, substeps );
        check( scene.getForceTrialTime() == 0.0 && same( physics.GetBodyLinearVelocity( force.bodyHandle ), {} ), "zero step does not consume force duration" );
        for( int frame = 0; frame < 30; ++frame )
            playback.stepOnce( substeps );
        check( std::abs( scene.getForceTrialTime() - 0.5 ) < 1.0e-6 && std::abs( physics.GetBodyLinearVelocity( force.bodyHandle ).y - 1.0f ) < 3.0e-5f && physics.GetBodyLinearVelocity( impulse.bodyHandle ).y == 2.0f && physics.GetBodyTransform( force.bodyHandle ).position.y < physics.GetBodyTransform( impulse.bodyHandle ).position.y, "halfway force accelerates gradually while impulse already has full speed and travels farther" );
        for( int frame = 0; frame < 30; ++frame )
            playback.stepOnce( substeps );
        check( std::abs( physics.GetBodyLinearVelocity( force.bodyHandle ).y - 2.0f ) < 3.0e-5f && physics.GetBodyLinearVelocity( impulse.bodyHandle ).y == 2.0f, "one second of equal total impulse produces matching speeds" );
        for( int frame = 0; frame < 30; ++frame )
            playback.stepOnce( substeps );
        check( std::abs( scene.getForceTrialTime() - 1.5 ) < 1.0e-6 && std::abs( physics.GetBodyLinearVelocity( force.bodyHandle ).y - 2.0f ) < 3.0e-5f && physics.GetBodyAngularVelocity( force.bodyHandle ) == 0.0f && physics.GetBodyAngularVelocity( impulse.bodyHandle ) == 0.0f, "force stops after one second while elapsed time continues without torque" );
        scene.startForceImpulseTrial();
        scene.step( 0.75f, substeps );
        scene.step( 0.5f, substeps );
        check( std::abs( physics.GetBodyLinearVelocity( force.bodyHandle ).y - 2.0f ) < 3.0e-5f && std::abs( scene.getForceTrialTime() - 1.25 ) < 1.0e-6, "step crossing one second applies only remaining force duration" );
        physics.SetShapeDensity( force.shapeHandle, 2.0f );
        physics.SetShapeDensity( impulse.shapeHandle, 2.0f );
        scene.startForceImpulseTrial();
        check( physics.GetBodyMass( force.bodyHandle ) == 2.0f && physics.GetBodyLinearVelocity( impulse.bodyHandle ).y == 1.0f && same( physics.GetBodyLinearVelocity( force.bodyHandle ), {} ) && scene.getForceTrialTime() == 0.0, "restarting preserves edited density and restores initial conditions before applying impulse" );
        for( int frame = 0; frame < 60; ++frame )
            playback.stepOnce( substeps );
        check( std::abs( physics.GetBodyLinearVelocity( force.bodyHandle ).y - 1.0f ) < 3.0e-5f && physics.GetBodyLinearVelocity( impulse.bodyHandle ).y == 1.0f, "density edit changes force and impulse response equally" );
        scene.startForceImpulseTrial();
        scene.step( 0.25f, substeps );
        scene.resetCollisionTrial();
        scene.step( 0.25f, substeps );
        check( physics.GetShapeDensity( force.shapeHandle ) == 2.0f && physics.GetShapeDensity( impulse.shapeHandle ) == 2.0f && scene.getForceTrialTime() == 0.0 && same( physics.GetBodyLinearVelocity( force.bodyHandle ), {} ) && same( physics.GetBodyLinearVelocity( impulse.bodyHandle ), {} ), "trial reset preserves edits and cancels active force without relaunching" );
        playback.reset();
        auto& defaults = static_cast<rigidBodyDemo&>( playback.getDemo() );
        check( defaults.getForceTrialTime() == 0.0 && defaults.getWorld().GetBodyMass( defaults.getShapes()[0].bodyHandle ) == 1.0f && !playback.isPlaying(), "full force reset restores default density and idle state" );
    }
    rigidBodyDemo other{ demoKind::collisionComparison };
    const auto body = other.getShapes()[0].bodyHandle;
    const auto velocity = other.getWorld().GetBodyLinearVelocity( body );
    other.startForceImpulseTrial();
    check( same( other.getWorld().GetBodyLinearVelocity( body ), velocity ), "force trial start does not alter other scenes" );
}

void checkTorqueImpulseDemo()
{
    for( int substeps : { 1, 4, 8 } )
    {
        demoSession playback{ createRigidBodyDemo };
        playback.selectDemo( demoKind::torqueImpulseComparison );
        auto& scene = static_cast<rigidBodyDemo&>( playback.getDemo() );
        auto& physics = scene.getWorld();
        check( scene.getShapes().size() == 2 && physics.GetBodyCount() == 2 && same( physics.GetGravity(), {} ), "torque trial creates two identical resting boxes without gravity" );
        const auto torque = scene.getShapes()[0];
        const auto impulse = scene.getShapes()[1];
        for( const auto& visual : scene.getShapes() )
            check( physics.GetBodyMass( visual.bodyHandle ) == 1.0f && std::abs( physics.GetBodyRotationalInertia( visual.bodyHandle ) - 1.0f / 6.0f ) < 1.0e-6f && !physics.IsBodySleepEnabled( visual.bodyHandle ), "torque comparison starts with equal analytic inertia and sleep disabled" );
        playback.stepOnce( substeps );
        check( scene.getTorqueTrialTime() == 0.0 && physics.GetBodyAngularVelocity( torque.bodyHandle ) == 0.0f, "torque trial waits for explicit start" );
        scene.startTorqueImpulseTrial();
        check( !playback.isPlaying() && scene.getTorqueTrialTime() == 0.0 && physics.GetBodyAngularVelocity( torque.bodyHandle ) == 0.0f && std::abs( physics.GetBodyAngularVelocity( impulse.bodyHandle ) - 6.0f ) < 1.0e-5f && physics.GetBodyTransform( impulse.bodyHandle ).rotation.s == 0.0f, "start applies one angular impulse without advancing time or rotation" );
        playback.advance( 0.5f, substeps );
        scene.step( 0.0f, substeps );
        check( scene.getTorqueTrialTime() == 0.0 && physics.GetBodyAngularVelocity( torque.bodyHandle ) == 0.0f, "paused and zero steps do not consume torque duration" );
        for( int frame = 0; frame < 30; ++frame )
            playback.stepOnce( substeps );
        check( std::abs( scene.getTorqueTrialTime() - 0.5 ) < 1.0e-6 && std::abs( physics.GetBodyAngularVelocity( torque.bodyHandle ) - 3.0f ) < 1.0e-4f && std::abs( physics.GetBodyAngularVelocity( impulse.bodyHandle ) - 6.0f ) < 1.0e-5f, "halfway torque accelerates gradually while angular impulse already has full spin" );
        for( int frame = 0; frame < 30; ++frame )
            playback.stepOnce( substeps );
        check( std::abs( physics.GetBodyAngularVelocity( torque.bodyHandle ) - 6.0f ) < 1.0e-4f && std::abs( physics.GetBodyAngularVelocity( impulse.bodyHandle ) - 6.0f ) < 1.0e-5f, "one second of torque matches angular impulse spin" );
        for( int frame = 0; frame < 30; ++frame )
            playback.stepOnce( substeps );
        check( std::abs( scene.getTorqueTrialTime() - 1.5 ) < 1.0e-6 && std::abs( physics.GetBodyAngularVelocity( torque.bodyHandle ) - 6.0f ) < 1.0e-4f, "torque stops after one second while elapsed time continues" );
        for( std::size_t i = 0; i < 2; ++i )
        {
            const auto body = scene.getShapes()[i].bodyHandle;
            check( same( physics.GetBodyLinearVelocity( body ), {} ) && same( physics.GetBodyTransform( body ).position, { i == 0 ? -2.5f : 2.5f, 0 } ) && std::abs( physics.GetBodyTransform( body ).rotation.s ) > 0.1f, "torque and angular impulse rotate in place without changing linear motion" );
        }
        scene.startTorqueImpulseTrial();
        for( int frame = 0; frame < 9; ++frame )
            scene.step( 0.1f, substeps );
        scene.step( 0.12f, substeps );
        check( std::abs( physics.GetBodyAngularVelocity( torque.bodyHandle ) - 6.0f ) < 1.0e-4f && std::abs( scene.getTorqueTrialTime() - 1.02 ) < 1.0e-6, "crossing one second delivers only the remaining torque duration" );
        physics.SetShapeDensity( torque.shapeHandle, 2.0f );
        physics.SetShapeDensity( impulse.shapeHandle, 2.0f );
        scene.startTorqueImpulseTrial();
        check( physics.GetBodyMass( torque.bodyHandle ) == 2.0f && std::abs( physics.GetBodyAngularVelocity( impulse.bodyHandle ) - 3.0f ) < 1.0e-5f && physics.GetBodyAngularVelocity( torque.bodyHandle ) == 0.0f && scene.getTorqueTrialTime() == 0.0, "restart preserves edited density and resets before applying one current-inertia impulse" );
        for( int frame = 0; frame < 60; ++frame )
            playback.stepOnce( substeps );
        check( std::abs( physics.GetBodyAngularVelocity( torque.bodyHandle ) - 3.0f ) < 1.0e-4f && std::abs( physics.GetBodyAngularVelocity( impulse.bodyHandle ) - 3.0f ) < 1.0e-5f, "density editing changes torque and angular impulse response equally" );
        scene.startTorqueImpulseTrial();
        physics.SetBodyLinearVelocity( torque.bodyHandle, { 1, 2 } );
        physics.SetBodyLinearVelocity( impulse.bodyHandle, { 1, 2 } );
        scene.step( 0.25f, substeps );
        check( same( physics.GetBodyLinearVelocity( torque.bodyHandle ), { 1, 2 } ) && same( physics.GetBodyLinearVelocity( impulse.bodyHandle ), { 1, 2 } ), "torque preserves nonzero edited linear velocities" );
        scene.resetCollisionTrial();
        scene.step( 0.25f, substeps );
        check( physics.GetShapeDensity( torque.shapeHandle ) == 2.0f && physics.GetShapeDensity( impulse.shapeHandle ) == 2.0f && scene.getTorqueTrialTime() == 0.0 && physics.GetBodyAngularVelocity( torque.bodyHandle ) == 0.0f && physics.GetBodyAngularVelocity( impulse.bodyHandle ) == 0.0f && same( physics.GetBodyLinearVelocity( torque.bodyHandle ), {} ), "reset preserves density and cancels active torque while restoring rest" );
        demoInput drag{};
        drag.mousePosition = physics.GetBodyTransform( torque.bodyHandle ).position;
        drag.mousePressed = drag.mouseHeld = true;
        scene.handleInput( drag );
        check( physics.IsValid( scene.getMouseJoint() ), "torque comparison retains existing drag interaction" );
        physics.SetBodyLinearDamping( torque.bodyHandle, 0.2f );
        physics.SetBodyAngularDamping( torque.bodyHandle, 0.3f );
        physics.SetGravity( { 0, -1 } );
        collisionMatrix matrix;
        matrix.setPair( 2, 3, false );
        playback.setCollisionMatrix( matrix );
        playback.setPlaying( true );
        scene.startTorqueImpulseTrial();
        check( !physics.IsValid( scene.getMouseJoint() ) && playback.isPlaying() && playback.getCollisionMatrix() == matrix && same( physics.GetGravity(), { 0, -1 } ) && physics.GetBodyLinearDamping( torque.bodyHandle ) == 0.2f && physics.GetBodyAngularDamping( torque.bodyHandle ) == 0.3f && physics.GetShapeDensity( torque.shapeHandle ) == 2.0f, "torque restart cancels drag while preserving edited damping gravity density common settings and play state" );
        playback.reset();
        auto& defaults = static_cast<rigidBodyDemo&>( playback.getDemo() );
        check( defaults.getTorqueTrialTime() == 0.0 && defaults.getWorld().GetBodyMass( defaults.getShapes()[0].bodyHandle ) == 1.0f && !playback.isPlaying(), "full torque reset restores defaults and idle state" );
    }
    rigidBodyDemo other{ demoKind::collisionComparison };
    const auto body = other.getShapes()[0].bodyHandle;
    const auto velocity = other.getWorld().GetBodyLinearVelocity( body );
    other.startTorqueImpulseTrial();
    check( same( other.getWorld().GetBodyLinearVelocity( body ), velocity ) && other.getWorld().GetBodyAngularVelocity( body ) == 0.0f, "torque trial start does not alter other scenes" );
}

void checkDynamicKinematicDemo()
{
    for( int substeps : { 1, 4, 8 } )
    {
        demoSession playback{ createRigidBodyDemo };
        playback.selectDemo( demoKind::dynamicKinematicComparison );
        auto& scene = static_cast<rigidBodyDemo&>( playback.getDemo() );
        auto& physics = scene.getWorld();
        check( scene.getShapes().size() == 4 && physics.GetBodyCount() == 4 && same( physics.GetGravity(), {} ), "body type comparison creates two independent collision pairs without gravity" );
        const auto visuals = std::vector<visualShape>{ scene.getShapes().begin(), scene.getShapes().end() };
        for( std::size_t i = 0; i < visuals.size(); ++i )
        {
            const auto body = visuals[i].bodyHandle;
            check( same( physics.GetBodyTransform( body ).position, { i % 2 == 0 ? -2.0f : 0.0f, i < 2 ? 1.5f : -1.5f } ) && same( physics.GetBodyLinearVelocity( body ), { i % 2 == 0 ? 2.0f : 0.0f, 0 } ) && physics.GetShapeRestitution( visuals[i].shapeHandle ) == 0.0f && physics.GetShapeFriction( visuals[i].shapeHandle ) == 0.0f, "pairs start with matching geometry material and prescribed mover speed" );
        }
        for( int frame = 0; frame < 60; ++frame )
            playback.stepOnce( substeps );
        const float dynamicSpeed = physics.GetBodyLinearVelocity( visuals[0].bodyHandle ).x;
        check( std::abs( dynamicSpeed - 1.0f ) < 0.1f && std::abs( physics.GetBodyLinearVelocity( visuals[1].bodyHandle ).x - 1.0f ) < 0.1f, "equal mass dynamic collision slows the mover and transfers momentum to its target" );
        check( physics.GetBodyLinearVelocity( visuals[2].bodyHandle ).x == 2.0f && std::abs( physics.GetBodyLinearVelocity( visuals[3].bodyHandle ).x - 2.0f ) < 0.1f && physics.GetBodyTransform( visuals[3].bodyHandle ).position.x > physics.GetBodyTransform( visuals[1].bodyHandle ).position.x + 0.2f, "kinematic mover retains prescribed speed while pushing the target farther" );
        check( physics.GetBody( visuals[0].bodyHandle ).type == bodyType::Dynamic && physics.GetBody( visuals[2].bodyHandle ).type == bodyType::Kinematic && physics.GetBodyMass( visuals[2].bodyHandle ) == 0.0f, "only the lower mover is kinematic with zero solver mass" );
        for( const auto& visual : visuals )
            if( physics.GetBody( visual.bodyHandle ).type == bodyType::Dynamic ) check( !physics.IsBodySleepEnabled( visual.bodyHandle ) && physics.GetBodyMass( visual.bodyHandle ) == 1.0f, "dynamic bodies start with equal mass and automatic sleep disabled" );
        physics.SetShapeDensity( visuals[0].shapeHandle, 3.0f );
        physics.SetShapeDensity( visuals[2].shapeHandle, 3.0f );
        physics.SetShapeDensity( visuals[3].shapeHandle, 3.0f );
        for( int repeat = 0; repeat < 2; ++repeat )
        {
            scene.resetCollisionTrial();
            check( physics.GetBodyMass( visuals[0].bodyHandle ) == 3.0f && physics.GetBodyMass( visuals[2].bodyHandle ) == 0.0f && physics.GetBodyMass( visuals[3].bodyHandle ) == 3.0f, "reset preserves density edits while kinematic solver mass stays zero" );
            for( std::size_t i = 0; i < visuals.size(); ++i )
            {
                const auto body = visuals[i].bodyHandle;
                check( physics.IsValid( visuals[i].shapeHandle ) && same( physics.GetBodyTransform( body ).position, { i % 2 == 0 ? -2.0f : 0.0f, i < 2 ? 1.5f : -1.5f } ) && same( physics.GetBodyLinearVelocity( body ), { i % 2 == 0 ? 2.0f : 0.0f, 0 } ), "body type trial reset restores identical poses velocities and handles" );
            }
            for( int frame = 0; frame < 60; ++frame )
                playback.stepOnce( substeps );
            check( std::abs( physics.GetBodyLinearVelocity( visuals[0].bodyHandle ).x - 1.5f ) < 0.1f && std::abs( physics.GetBodyLinearVelocity( visuals[1].bodyHandle ).x - 1.5f ) < 0.1f && physics.GetBodyLinearVelocity( visuals[0].bodyHandle ).x > dynamicSpeed + 0.3f, "increased dynamic mover density changes the repeated collision momentum split" );
            check( physics.GetBodyLinearVelocity( visuals[2].bodyHandle ).x == 2.0f && std::abs( physics.GetBodyLinearVelocity( visuals[3].bodyHandle ).x - 2.0f ) < 0.1f, "edited kinematic shape density and heavier target do not slow the prescribed mover" );
        }
        physics.SetShapeRestitution( visuals[1].shapeHandle, 1.0f );
        scene.resetCollisionTrial();
        check( physics.GetShapeRestitution( visuals[1].shapeHandle ) == 1.0f, "body type trial preserves edited restitution" );
        for( int frame = 0; frame < 60; ++frame )
            playback.stepOnce( substeps );
        check( physics.GetBodyLinearVelocity( visuals[1].bodyHandle ).x > 2.5f && physics.GetBodyLinearVelocity( visuals[0].bodyHandle ).x < 1.2f, "restitution edit changes dynamic collision response on the repeated trial" );
        playback.reset();
        auto& defaults = static_cast<rigidBodyDemo&>( playback.getDemo() );
        check( !defaults.getWorld().IsValid( visuals[0].bodyHandle ) && defaults.getWorld().GetBodyMass( defaults.getShapes()[0].bodyHandle ) == 1.0f && defaults.getWorld().GetShapeRestitution( defaults.getShapes()[1].shapeHandle ) == 0.0f && defaults.getWorld().GetBody( defaults.getShapes()[2].bodyHandle ).type == bodyType::Kinematic && !playback.isPlaying(), "full body type reset restores default materials density and scene types" );
    }
}

void checkCollisionDemos()
{
    checkDynamicKinematicDemo();
    checkTorqueImpulseDemo();
    checkForceImpulseDemo();
    checkRotationalInertiaDemo();
    checkImpulsePointDemo();
    checkMassImpulseDemo();
    checkGravityScaleDemo();
    checkDampingDemo();
    checkSleepWakeDemo();
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

    for( demoKind kind : { demoKind::collisionComparison, demoKind::frictionComparison, demoKind::restitutionComparison, demoKind::boxStack, demoKind::ccdComparison, demoKind::collisionFilterComparison, demoKind::sensorWallComparison, demoKind::sleepWakeComparison, demoKind::dampingComparison, demoKind::gravityScaleComparison, demoKind::massImpulseComparison, demoKind::impulsePointComparison, demoKind::rotationalInertiaComparison, demoKind::forceImpulseComparison, demoKind::torqueImpulseComparison, demoKind::dynamicKinematicComparison } )
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
