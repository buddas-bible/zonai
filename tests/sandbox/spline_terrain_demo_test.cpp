#include "rigidBodyDemo.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <string_view>
#include <variant>
#include <limits>
#include "geometry/catmullRomSpline2.h"

using namespace zonai;
using namespace zonai::sandbox;

namespace
{

void check( bool ok, const char* message )
{
    if( !ok )
    {
        std::fprintf( stderr, "%s\n", message );
        std::exit( EXIT_FAILURE );
    }
}

bool near( vec2 a, vec2 b )
{
    return LengthSquared( a - b ) < 1.0e-8f;
}

bool hasPoint( const rigidBodyDemo& scene, vec2 point )
{
    for( std::size_t i = 0; i < scene.getTerrainPointCount(); ++i )
        if( near( scene.getTerrainPoint( i ), point ) ) return true;
    return false;
}

} // namespace

int main()
{
    const auto entries = getDemoEntries();
    const auto entry = std::find_if( entries.begin(), entries.end(),
        []( const demoEntry& candidate )
        {
            return std::string_view{ candidate.name } == "스플라인 지형";
        } );

    check( entry != entries.end(), "spline terrain demo is registered" );
    check( std::string_view{ entry->category } == "지형", "spline terrain belongs to the terrain category" );

    rigidBodyDemo terrain{ entry->kind };
    world& physics = terrain.getWorld();
    const std::vector<visualShape> shapes{ terrain.getShapes().begin(), terrain.getShapes().end() };

    check( physics.GetChainCount() == 1 && physics.GetBodyCount() == 4, "spline terrain uses one static chain and three dynamic probes" );

    const chainId parent = physics.GetShapeParentChain( shapes[0].shapeHandle );
    check( physics.IsValid( parent ), "sampled terrain is a real parent chain" );

    const std::array<vec2, 9> controlPoints{
        vec2{ 8.0f, -2.0f }, vec2{ 6.0f, -1.2f }, vec2{ 4.0f, -1.0f },
        vec2{ 2.0f, 0.0f }, vec2{ 0.0f, 0.8f }, vec2{ -2.0f, 0.1f },
        vec2{ -4.0f, -0.8f }, vec2{ -6.0f, -0.4f }, vec2{ -8.0f, -1.2f }
    };

    const auto expected = SampleAdaptive( { controlPoints }, 0.01f );
    check( expected.size() >= 2 && terrain.getTerrainPointCount() == expected.size(), "terrain uses the adaptive one-centimeter samples" );
    const std::size_t edgeCount = expected.size() - 1;
    check( shapes.size() == edgeCount + 3, "adaptive terrain retains its three probes" );
    for( std::size_t i = 0; i < expected.size(); ++i )
        check( near( terrain.getTerrainPoint( i ), expected[i] ), "collision chain uses adaptive sample positions" );

    for( std::size_t i = 0; i < controlPoints.size(); ++i )
        check( hasPoint( terrain, controlPoints[i] ), "sampled chain passes through every spline control point" );

    check( LengthSquared( terrain.getTerrainPoint( 1 ) - vec2{ 7.5f, -1.8f } ) > 1.0e-6f, "terrain follows the cubic curve rather than only splitting straight edges" );

    for( std::size_t i = 0; i < edgeCount; ++i )
    {
        const shape& child = physics.GetShape( shapes[i].shapeHandle );
        const chainSegment2& edge = std::get<chainSegment2>( child.geometry );
        const chainId childParent = physics.GetShapeParentChain( shapes[i].shapeHandle );

        check( childParent.index1 == parent.index1 && childParent.generation == parent.generation, "all sampled edges share the existing chain owner" );
        check( near( edge.segment.a, terrain.getTerrainPoint( i ) ) && near( edge.segment.b, terrain.getTerrainPoint( i + 1 ) ), "chain collision geometry uses the sampled points" );
        check( LengthSquared( edge.segment.b - edge.segment.a ) > LINEAR_SLOP * LINEAR_SLOP, "every sampled edge satisfies the chain minimum length" );
        check( Cross( Normalize( edge.segment.b - edge.segment.a ), 1.0f ).y > 0.0f, "right-to-left sample order keeps terrain front faces upward" );

        const vec2 previous = i == 0 ? edge.segment.a + ( edge.segment.a - edge.segment.b ) : terrain.getTerrainPoint( i - 1 );
        const vec2 next = i + 1 == edgeCount ? edge.segment.b + ( edge.segment.b - edge.segment.a ) : terrain.getTerrainPoint( i + 2 );

        check( near( edge.ghost1, previous ) && near( edge.ghost2, next ), "ghost points extend endpoint edges and connect adjacent samples" );
        check( child.friction == 0.8f, "sampled terrain preserves the existing terrain material" );
    }

    demoInput input{};
    input.mousePosition = controlPoints[1];
    input.mousePressed = true;
    input.mouseHeld = true;
    terrain.handleInput( input );
    check( terrain.getTerrainPointDragIndex().has_value() && *terrain.getTerrainPointDragIndex() == 1, "spline press selects the control point rather than the sample index" );
    check( !physics.IsValid( terrain.getMouseJoint() ), "control point drag takes priority over mouse joints" );
    input.mousePosition.y += 1.0f;
    input.mousePressed = false;
    terrain.handleInput( input );
    check( hasPoint( terrain, { 6.0f, -0.2f } ), "control drag resamples chain geometry while paused" );
    check( terrain.getSplineControlPoints().size() == 9 && near( terrain.getSplineControlPoints()[1], { 6.0f, -0.2f } ), "control drag commits the stored control point" );
    check( terrain.getTerrainPointCount() != expected.size(), "control drag changes adaptive topology" );
    const std::size_t editedCount = terrain.getTerrainPointCount() - 1;
    for( std::size_t i = 0; i < 3; ++i )
        check( terrain.getShapes()[editedCount + i].shapeHandle == shapes[edgeCount + i].shapeHandle, "topology edit preserves probe identities" );
    std::vector<vec2> validSamples( terrain.getTerrainPointCount() );
    for( std::size_t i = 0; i < validSamples.size(); ++i ) validSamples[i] = terrain.getTerrainPoint( i );
    input.mousePosition = controlPoints[0];
    terrain.handleInput( input );
    input.mousePosition = controlPoints[0] + vec2{ 0.0f, -0.001f };
    terrain.handleInput( input );
    for( std::size_t i = 0; i < validSamples.size(); ++i )
        check( near( terrain.getTerrainPoint( i ), validSamples[i] ), "invalid control drag leaves the entire sampled chain unchanged" );
    check( near( terrain.getSplineControlPoints()[1], { 6.0f, -0.2f } ), "invalid drag keeps the last valid control point" );
    for( std::size_t i = 0; i < std::min( edgeCount, editedCount ); ++i ) check( physics.IsValid( shapes[i].shapeHandle ), "control drag preserves common child handles" );
    terrain.cancelInput();
    check( !terrain.getTerrainPointDragIndex().has_value(), "cancel clears control drag selection" );

    for( const vec2 sample : validSamples )
    {
        float nearest = std::numeric_limits<float>::infinity();
        for( const vec2 control : terrain.getSplineControlPoints() ) nearest = std::min( nearest, LengthSquared( sample - control ) );
        if( nearest > 0.0625f ) { input.mousePosition = sample; break; }
    }
    input.mousePressed = true;
    terrain.handleInput( input );
    check( !terrain.getTerrainPointDragIndex().has_value(), "intermediate samples are not selectable control points" );
    terrain.cancelInput();

    input.mousePosition = controlPoints[0];
    input.mousePressed = true;
    terrain.handleInput( input );
    input.mousePosition = { 8.5f, -1.5f };
    input.mousePressed = false;
    terrain.handleInput( input );
    const auto& updatedFirst = std::get<chainSegment2>( physics.GetShape( shapes[0].shapeHandle ).geometry );
    check( near( updatedFirst.segment.a, { 8.5f, -1.5f } ), "endpoint control drag updates the first sample" );
    check( near( updatedFirst.ghost1, updatedFirst.segment.a + ( updatedFirst.segment.a - updatedFirst.segment.b ) ), "endpoint control drag recomputes the external ghost" );
    terrain.cancelInput();

    {
        rigidBodyDemo transformed{ entry->kind };
        auto& simulation = transformed.getWorld();
        const transform2 pose{ { 1.5f, 0.75f }, rot2::FromRadians( 0.25f ) };
        simulation.SetBodyTransform( transformed.getShapes()[0].bodyHandle, pose );
        transformed.setSplineTrialFraction( 0.0f );
        transformed.launchTerrainProbe( 0, terrainTrial::front );
        const vec2 a = TransformPoint( pose, transformed.getTerrainPoint( 0 ) );
        const vec2 b = TransformPoint( pose, transformed.getTerrainPoint( 1 ) );
        const vec2 normal = Cross( Normalize( b - a ), 1.0f );

        check( near( simulation.GetBodyTransform( transformed.getImpulseBody() ).position, a + 1.5f * normal ), "spline terrain launch converts sampled local points to world space" );
        check( near( simulation.GetBodyLinearVelocity( transformed.getImpulseBody() ), -8.0f * normal ), "spline terrain launch rotates its front-face approach velocity" );
        check( hasPoint( transformed, controlPoints[1] ), "moving the terrain body preserves sampled local geometry" );
        demoInput press{};
        press.mousePosition = TransformPoint( pose, controlPoints[4] );
        press.mousePressed = press.mouseHeld = true;
        transformed.handleInput( press );
        check( transformed.getTerrainPointDragIndex().has_value() && *transformed.getTerrainPointDragIndex() == 4, "transformed spline control is selected in local coordinates" );
        press.mousePosition = TransformPoint( pose, { 0.5f, 1.6f } );
        press.mousePressed = false;
        transformed.handleInput( press );
        check( hasPoint( transformed, { 0.5f, 1.6f } ), "transformed control drag stores local sampled geometry" );
        press.mouseHeld = false;
        transformed.handleInput( press );
        check( !transformed.getTerrainPointDragIndex().has_value(), "release clears spline control drag" );
    }

    {
        rigidBodyDemo touching{ entry->kind };
        touching.getWorld().SetBodyTransform( touching.getImpulseBody(), { { 0.0f, 1.2f }, {} } );
        touching.step( 0.001f, 4 );
        check( !touching.getContacts().empty(), "paused contact refresh fixture" );
        demoInput drag{};
        drag.mousePosition = controlPoints[4];
        drag.mousePressed = drag.mouseHeld = true;
        touching.handleInput( drag );
        drag.mousePosition = { 0.0f, -0.2f };
        drag.mousePressed = false;
        touching.handleInput( drag );
        check( touching.getContacts().empty(), "paused control edit immediately removes obsolete contact display" );
    }

    for( const int substeps : { 1, 4, 8 } )
        for( std::size_t probe = 0; probe < 3; ++probe )
            for( const auto trial : { terrainTrial::seams, terrainTrial::front, terrainTrial::back } )
            {
                rigidBodyDemo scene{ entry->kind };
                auto& simulation = scene.getWorld();
                const auto selected = scene.getShapes()[edgeCount + probe];
                scene.launchTerrainProbe( probe, trial );
                const auto initialObservation = scene.getTerrainTrialObservation();
                check( initialObservation.active && initialObservation.probeIndex == probe && initialObservation.trial == trial && initialObservation.elapsed == 0.0f && !initialObservation.firstContactTime, "launch starts a new observation for its selected probe and trial" );
                check( initialObservation.launchSpeed == ( trial == terrainTrial::seams ? 6.0f : 8.0f ) && ( trial == terrainTrial::seams ? !initialObservation.launchFraction : initialObservation.launchFraction == 0.5f ), "observation records applied speed and front/back position instead of live settings" );
                const vec2 velocity = simulation.GetBodyLinearVelocity( selected.bodyHandle );
                const vec2 normal = trial == terrainTrial::seams ? Cross( Normalize( velocity ), 1.0f ) : ( trial == terrainTrial::back ? 1.0f : -1.0f ) * Normalize( velocity );
                const vec2 target = simulation.GetBodyTransform( selected.bodyHandle ).position - ( trial == terrainTrial::back ? -1.0f : 1.0f ) * 1.5f * normal;
                bool contacted = false, crossed = false;
                int contactFrames = 0;
                std::optional<float> firstContact{};

                for( int frame = 0; frame < 120; ++frame )
                {
                    scene.step( 1.0f / 60.0f, substeps );
                    const vec2 position = simulation.GetBodyTransform( selected.bodyHandle ).position;
                    check( IsFinite( position ), "spline terrain trials remain finite" );
                    const float side = Dot( position - target, normal );
                    bool frameContact = false;

                    for( const auto& contact : scene.getContacts() )
                    {
                        if( contact.manifold.pointCount == 0 ) continue;
                        if( contact.shapeA != selected.shapeHandle && contact.shapeB != selected.shapeHandle ) continue;
                        const auto other = contact.shapeA == selected.shapeHandle ? contact.shapeB : contact.shapeA;
                        if( !simulation.IsValid( simulation.GetShapeParentChain( other ) ) ) continue;
                        contacted = true;
                        frameContact = true;
                        if( trial == terrainTrial::back && !crossed && side < 0.0f ) check( false, "sampled terrain must not block a probe approaching its back face" );
                    }

                    if( frameContact ) ++contactFrames;
                    if( frameContact && !firstContact ) firstContact = static_cast<float>( frame + 1 ) / 60.0f;
                    const auto observation = scene.getTerrainTrialObservation();
                    check( observation.active && observation.touching == frameContact && std::abs( observation.elapsed - static_cast<float>( frame + 1 ) / 60.0f ) < 1.0e-4f, "observation follows completed physics steps and selected terrain contacts" );
                    check( observation.firstContactTime.has_value() == firstContact.has_value(), "first contact is reported only after an observed terrain contact" );
                    if( firstContact ) check( std::abs( *observation.firstContactTime - *firstContact ) < 1.0e-4f, "first contact observation remains the original step-end time" );
                    if( trial == terrainTrial::seams && contacted )
                        for( std::size_t i = 0; i < edgeCount; ++i )
                        {
                            const vec2 edgeA = scene.getTerrainPoint( i ), edgeB = scene.getTerrainPoint( i + 1 );
                            if( position.x > edgeA.x || position.x < edgeB.x ) continue;
                            const vec2 front = Cross( Normalize( edgeB - edgeA ), 1.0f );

                            check( Dot( position - edgeA, front ) >= -0.05f, "seam traversal stays on the terrain front side after contact" );
                        }

                    if( side > 0.8f ) crossed = true;
                }

                if( trial == terrainTrial::back ) check( crossed, "back trial crosses sampled terrain to the front side" );
                else check( contacted, "front and seam trials contact sampled chain geometry" );
                if( trial == terrainTrial::seams )
                {
                    check( simulation.GetBodyTransform( selected.bodyHandle ).position.x < 7.0f, "seam trial moves across multiple sampled edges" );
                    check( contactFrames >= 5, "seam traversal retains support across multiple frames" );
                }

                scene.resetTerrainProbes();
                check( !scene.getTerrainTrialObservation().active, "probe reset clears trial observations" );
                for( std::size_t i = edgeCount; i < edgeCount + 3; ++i )
                    check( LengthSquared( simulation.GetBodyLinearVelocity( scene.getShapes()[i].bodyHandle ) ) == 0.0f, "spline terrain probe reset stops every probe" );
                check( simulation.GetChainCount() == 1 && simulation.GetShapeCount() == edgeCount + 3, "trials and reset preserve the sampled terrain chain" );
            }

    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        check( scene.getSplineSamplingError() == 0.01f, "spline sampling defaults to one centimeter" );
        const auto probe = scene.getShapes()[scene.getTerrainPointCount() - 1];
        const transform2 pose = scene.getWorld().GetBodyTransform( probe.bodyHandle );
        scene.getWorld().SetShapeFriction( probe.shapeHandle, 0.37f );
        const std::size_t initialCount = scene.getTerrainPointCount();
        scene.setSplineSamplingError( 0.05f );
        check( scene.getSplineSamplingError() == 0.05f && scene.getTerrainPointCount() < initialCount, "larger error produces fewer terrain edges" );
        scene.setSplineSamplingError( 0.005f );
        check( scene.getSplineSamplingError() == 0.005f && scene.getTerrainPointCount() > initialCount, "smaller error produces more terrain edges" );
        check( scene.getShapes()[scene.getTerrainPointCount() - 1].shapeHandle == probe.shapeHandle, "resampling preserves probe handles" );
        check( near( scene.getWorld().GetBodyTransform( probe.bodyHandle ).position, pose.position ) && scene.getWorld().GetShape( probe.shapeHandle ).friction == 0.37f, "resampling preserves probe pose and material" );
        const std::size_t count = scene.getTerrainPointCount();
        const auto first = scene.getShapes()[0].shapeHandle;
        for( const float invalid : { 0.0f, -1.0f, 0.004f, 0.051f, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity() } )
            scene.setSplineSamplingError( invalid );
        check( scene.getSplineSamplingError() == 0.005f && scene.getTerrainPointCount() == count && scene.getShapes()[0].shapeHandle == first, "invalid error changes preserve the setting and terrain" );
        demoInput drag{};
        drag.mousePosition = scene.getSplineControlPoints()[1];
        drag.mousePressed = drag.mouseHeld = true;
        scene.handleInput( drag );
        drag.mousePressed = false;
        drag.mousePosition.y += 1.0f;
        scene.handleInput( drag );
        check( near( scene.getSplineControlPoints()[1], drag.mousePosition ), "drag fixture must apply" );
        const auto samples = SampleAdaptive( { scene.getSplineControlPoints() }, 0.005f );
        check( scene.getTerrainPointCount() == samples.size(), "dragging uses the selected sampling error" );
        for( std::size_t i = 0; i < samples.size(); ++i ) check( near( scene.getTerrainPoint( i ), samples[i] ), "dragging retains selected-error sample positions" );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        scene.resetTerrainProbes();
        check( scene.getSplineSamplingError() == 0.005f && scene.getTerrainPointCount() == samples.size(), "probe launch and reset preserve the error setting" );
        rigidBodyDemo reset{ demoKind::splineTerrain };
        check( reset.getSplineSamplingError() == 0.01f, "fresh scene resets sampling to one centimeter" );
        rigidBodyDemo polygon{ demoKind::terrainChain };
        const std::size_t polygonCount = polygon.getTerrainPointCount();
        polygon.setSplineSamplingError( 0.05f );
        check( polygon.getTerrainPointCount() == polygonCount && polygon.getSplineSamplingError() == 0.01f, "sampling control does not alter polygon terrain" );
    }

    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        scene.setSplineSamplingError( 0.005f );
        demoInput drag{};
        drag.mousePosition = scene.getSplineControlPoints()[1];
        drag.mousePressed = drag.mouseHeld = true;
        scene.handleInput( drag );
        drag.mousePressed = false;
        drag.mousePosition = scene.getSplineControlPoints()[0];
        const vec2 control = scene.getSplineControlPoints()[1];
        std::vector<vec2> before;
        for( std::size_t i = 0; i < scene.getTerrainPointCount(); ++i ) before.push_back( scene.getTerrainPoint( i ) );
        const auto first = scene.getShapes()[0].shapeHandle;
        scene.handleInput( drag );
        check( near( scene.getSplineControlPoints()[1], control ), "collapsed controls must reject the drag" );
        check( scene.getSplineSamplingError() == 0.005f && scene.getTerrainPointCount() == before.size() && scene.getShapes()[0].shapeHandle == first, "failed drag preserves the selected error and handles" );
        for( std::size_t i = 0; i < before.size(); ++i ) check( near( scene.getTerrainPoint( i ), before[i] ), "failed resampling preserves every terrain point" );
    }

    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        scene.setSplineSamplingError( 0.005f );
        const auto controls = scene.getSplineControlPoints();
        const vec2 midpoint = Evaluate( { controls }, 1, 0.5f );
        const vec2 next = controls[2];
        const auto first = scene.getShapes()[0].shapeHandle;
        scene.getWorld().SetShapeFriction( first, 0.42f );
        const std::size_t oldEdges = scene.getTerrainPointCount() - 1;
        const std::array<visualShape, 3> probes{ scene.getShapes()[oldEdges], scene.getShapes()[oldEdges + 1], scene.getShapes()[oldEdges + 2] };
        scene.getWorld().SetBodyLinearVelocity( probes[0].bodyHandle, { 2.0f, 3.0f } );
        demoInput drag{};
        drag.mousePosition = controls[1];
        drag.mousePressed = drag.mouseHeld = true;
        scene.handleInput( drag );
        check( scene.getTerrainPointDragIndex() == 1, "insert fixture starts with an active drag" );
        check( scene.insertSplineControlPoint( 1 ), "insert a control at the chosen segment midpoint" );
        check( scene.getSplineControlPoints().size() == 10 && near( scene.getSplineControlPoints()[2], midpoint ) && near( scene.getSplineControlPoints()[3], next ), "insertion preserves order and uses the cubic midpoint" );
        check( !scene.getTerrainPointDragIndex(), "insertion ends the old index-based drag" );
        check( scene.getSplineSamplingError() == 0.005f && scene.getShapes()[0].shapeHandle == first && scene.getWorld().GetShape( first ).friction == 0.42f, "insertion retains sampling error and surviving edge properties" );
        const auto samples = SampleAdaptive( { scene.getSplineControlPoints() }, 0.005f );
        check( scene.getTerrainPointCount() == samples.size(), "insertion resamples at the selected error" );
        for( std::size_t i = 0; i < samples.size(); ++i ) check( near( scene.getTerrainPoint( i ), samples[i] ), "inserted geometry uses the selected-error samples" );
        for( std::size_t i = 0; i < probes.size(); ++i ) check( scene.getShapes()[samples.size() - 1 + i].shapeHandle == probes[i].shapeHandle, "insertion retains every probe" );
        check( near( scene.getWorld().GetBodyLinearVelocity( probes[0].bodyHandle ), { 2.0f, 3.0f } ), "insertion preserves probe motion" );
        drag.mousePosition = midpoint;
        scene.handleInput( drag );
        check( scene.getTerrainPointDragIndex() == 2, "inserted controls remain draggable" );
        check( scene.removeSplineControlPoint( 2 ) && scene.getSplineControlPoints().size() == 9, "remove the inserted control" );
        check( !scene.getTerrainPointDragIndex(), "deletion ends the old drag" );
        const std::size_t count = scene.getTerrainPointCount();
        check( !scene.insertSplineControlPoint( 8 ) && !scene.insertSplineControlPoint( std::numeric_limits<std::size_t>::max() ) && !scene.removeSplineControlPoint( 9 ), "out-of-range topology edits are rejected" );
        check( scene.getSplineControlPoints().size() == 9 && scene.getTerrainPointCount() == count, "invalid topology edits leave geometry intact" );
        const vec2 second = scene.getSplineControlPoints()[1];
        check( scene.removeSplineControlPoint( 0 ) && near( scene.getSplineControlPoints()[0], second ), "deleting the first endpoint preserves the remaining order" );
        while( scene.getSplineControlPoints().size() > 2 ) check( scene.removeSplineControlPoint( scene.getSplineControlPoints().size() - 1 ), "endpoint deletion supports an open two-point spline" );
        check( !scene.removeSplineControlPoint( 0 ) && !scene.removeSplineControlPoint( 1 ), "deletion preserves the two-point minimum" );
        check( scene.insertSplineControlPoint( 0 ), "two-point splines still support insertion" );
        scene.resetTerrainProbes();
        check( scene.getSplineControlPoints().size() == 3 && scene.getSplineSamplingError() == 0.005f, "probe reset preserves edited controls and error" );
        for( std::size_t i = 0; i < probes.size(); ++i ) check( scene.getShapes()[scene.getTerrainPointCount() - 1 + i].shapeHandle == probes[i].shapeHandle, "repeated endpoint edits preserve every probe" );
        rigidBodyDemo polygon{ demoKind::terrainChain };
        check( !polygon.insertSplineControlPoint( 0 ) && !polygon.removeSplineControlPoint( 0 ), "spline topology actions do not alter polygon terrain" );
    }

    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        demoInput drag{};
        drag.mousePosition = scene.getSplineControlPoints()[0];
        drag.mousePressed = drag.mouseHeld = true;
        scene.handleInput( drag );
        drag.mousePressed = false;
        drag.mousePosition = scene.getSplineControlPoints()[1] + vec2{ 0.006f, 0.0f };
        scene.handleInput( drag );
        check( near( scene.getSplineControlPoints()[0], drag.mousePosition ), "short-segment insertion fixture must apply" );
        std::vector<vec2> before;
        for( std::size_t i = 0; i < scene.getTerrainPointCount(); ++i ) before.push_back( scene.getTerrainPoint( i ) );
        check( !scene.insertSplineControlPoint( 0 ), "insertion that creates too-short edges is rejected" );
        check( scene.getSplineControlPoints().size() == 9 && scene.getTerrainPointCount() == before.size() && scene.getTerrainPointDragIndex() == 0, "failed insertion preserves controls, geometry and drag selection" );
        for( std::size_t i = 0; i < before.size(); ++i ) check( near( scene.getTerrainPoint( i ), before[i] ), "failed insertion preserves every sampled point" );
    }

    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        demoInput drag{};
        drag.mousePosition = scene.getSplineControlPoints()[2];
        drag.mousePressed = drag.mouseHeld = true;
        scene.handleInput( drag );
        drag.mousePressed = false;
        drag.mousePosition = scene.getSplineControlPoints()[0];
        scene.handleInput( drag );
        check( near( scene.getSplineControlPoints()[2], scene.getSplineControlPoints()[0] ), "deletion rejection fixture must have repeated nonadjacent controls" );
        std::vector<vec2> before;
        for( std::size_t i = 0; i < scene.getTerrainPointCount(); ++i ) before.push_back( scene.getTerrainPoint( i ) );
        check( !scene.removeSplineControlPoint( 1 ), "deletion that collapses neighboring controls is rejected" );
        check( scene.getSplineControlPoints().size() == 9 && scene.getTerrainPointCount() == before.size() && scene.getTerrainPointDragIndex() == 2, "failed deletion preserves controls, geometry and drag selection" );
        for( std::size_t i = 0; i < before.size(); ++i ) check( near( scene.getTerrainPoint( i ), before[i] ), "failed deletion preserves every sampled point" );
    }

    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        const std::vector<vec2> original{ scene.getSplineControlPoints().begin(), scene.getSplineControlPoints().end() };
        check( !scene.canUndoSplineEdit() && !scene.canRedoSplineEdit() && !scene.undoSplineEdit() && !scene.redoSplineEdit(), "fresh spline scenes have no edit history" );
        const auto first = scene.getShapes()[0].shapeHandle;
        const auto probe = scene.getShapes()[scene.getTerrainPointCount() - 1];
        check( scene.insertSplineControlPoint( 1 ) && scene.canUndoSplineEdit(), "insertion records one undo state" );
        scene.getWorld().SetShapeFriction( first, 0.63f );
        scene.getWorld().SetShapeFriction( probe.shapeHandle, 0.27f );
        scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, { 3.0f, 4.0f } );
        const transform2 pose = scene.getWorld().GetBodyTransform( probe.bodyHandle );
        check( scene.undoSplineEdit() && !scene.canUndoSplineEdit() && scene.canRedoSplineEdit(), "undo transfers the single state to redo" );
        check( scene.getSplineControlPoints().size() == original.size(), "undo restores control count" );
        for( std::size_t i = 0; i < original.size(); ++i ) check( near( scene.getSplineControlPoints()[i], original[i] ), "undo restores each control" );
        check( scene.getShapes()[0].shapeHandle == first && scene.getWorld().GetShape( first ).friction == 0.63f, "undo preserves the surviving edge's current material" );
        check( scene.getShapes()[scene.getTerrainPointCount() - 1].shapeHandle == probe.shapeHandle && scene.getWorld().GetShape( probe.shapeHandle ).friction == 0.27f, "undo preserves the probe handle and current material" );
        check( near( scene.getWorld().GetBodyTransform( probe.bodyHandle ).position, pose.position ) && near( scene.getWorld().GetBodyLinearVelocity( probe.bodyHandle ), { 3.0f, 4.0f } ), "undo preserves current probe pose and velocity" );
        check( scene.redoSplineEdit() && scene.getSplineControlPoints().size() == 10 && !scene.canRedoSplineEdit(), "redo restores insertion" );
        check( scene.undoSplineEdit(), "redo itself remains undoable" );
        check( !scene.insertSplineControlPoint( 99 ) && !scene.removeSplineControlPoint( 99 ), "invalid edits fail with a redo state" );
        scene.setSplineSamplingError( 0.01f );
        scene.setSplineSamplingError( -1.0f );
        demoInput drag{};
        drag.mousePosition = scene.getSplineControlPoints()[1];
        drag.mousePressed = drag.mouseHeld = true;
        scene.handleInput( drag );
        drag.mousePressed = false;
        drag.mousePosition = scene.getSplineControlPoints()[0];
        scene.handleInput( drag );
        scene.cancelInput();
        check( scene.canRedoSplineEdit() && !scene.canUndoSplineEdit(), "failed and unchanged inputs do not replace history" );
        scene.setSplineSamplingError( 0.005f );
        check( scene.canUndoSplineEdit() && !scene.canRedoSplineEdit(), "successful new edits invalidate redo" );
        check( scene.undoSplineEdit() && scene.getSplineSamplingError() == 0.01f, "undo restores sampling error" );
        check( scene.redoSplineEdit() && scene.getSplineSamplingError() == 0.005f, "redo restores sampling error" );
        scene.beginSplineSamplingEdit();
        scene.setSplineSamplingError( 0.02f );
        scene.setSplineSamplingError( 0.04f );
        scene.endSplineSamplingEdit();
        check( scene.undoSplineEdit() && scene.getSplineSamplingError() == 0.005f, "a sampling gesture undoes all intermediate changes" );
        check( scene.redoSplineEdit() && scene.getSplineSamplingError() == 0.04f, "a sampling gesture redoes its final error" );
        drag.mousePosition = scene.getSplineControlPoints()[1];
        const vec2 beforeDrag = drag.mousePosition;
        drag.mousePressed = true;
        scene.handleInput( drag );
        drag.mousePressed = false;
        drag.mousePosition.y += 0.3f;
        scene.handleInput( drag );
        drag.mousePosition.y += 0.5f;
        scene.handleInput( drag );
        drag.mouseHeld = false;
        scene.handleInput( drag );
        check( near( scene.getSplineControlPoints()[1], drag.mousePosition ), "history drag fixture applies both updates" );
        check( scene.undoSplineEdit() && near( scene.getSplineControlPoints()[1], beforeDrag ), "one undo restores the entire drag" );
        check( scene.redoSplineEdit() && near( scene.getSplineControlPoints()[1], drag.mousePosition ), "redo restores the final drag position" );
        drag.mousePressed = drag.mouseHeld = true;
        scene.handleInput( drag );
        drag.mousePressed = false;
        drag.mousePosition.y += 0.2f;
        scene.handleInput( drag );
        scene.cancelInput();
        check( scene.undoSplineEdit() && near( scene.getSplineControlPoints()[1], beforeDrag + vec2{ 0.0f, 0.8f } ), "input cancellation closes the drag gesture without discarding its undo" );
        check( scene.removeSplineControlPoint( 2 ) && scene.undoSplineEdit() && scene.getSplineControlPoints().size() == 9, "deletion can be undone" );
        scene.resetTerrainProbes();
        check( scene.canRedoSplineEdit(), "probe reset preserves edit history" );
        check( scene.redoSplineEdit() && scene.getSplineControlPoints().size() == 8, "deletion can be redone after probe reset" );
        rigidBodyDemo reset{ demoKind::splineTerrain };
        check( !reset.canUndoSplineEdit() && !reset.canRedoSplineEdit(), "fresh scene reset clears history" );
    }

    for( const bool redo : { false, true } )
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        check( scene.insertSplineControlPoint( 1 ), "restore failure fixture inserts a control" );
        if( redo ) check( scene.undoSplineEdit(), "restore failure fixture prepares redo" );
        const std::size_t count = scene.getSplineControlPoints().size();
        scene.getWorld().DestroyChain( scene.getWorld().GetShapeParentChain( scene.getShapes()[0].shapeHandle ) );
        check( redo ? !scene.redoSplineEdit() : !scene.undoSplineEdit(), "restoring against a retired chain fails" );
        check( scene.getSplineControlPoints().size() == count && ( redo ? scene.canRedoSplineEdit() : scene.canUndoSplineEdit() ), "failed restoration preserves controls and history" );
    }

    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        const std::vector<vec2> original{ scene.getSplineControlPoints().begin(), scene.getSplineControlPoints().end() };
        check( !scene.resetSplineTerrain() && !scene.canUndoSplineEdit(), "resetting an original terrain is unchanged and records no edit" );
        check( scene.insertSplineControlPoint( 1 ), "terrain reset fixture inserts a point" );
        scene.setSplineSamplingError( 0.005f );
        const std::vector<vec2> edited{ scene.getSplineControlPoints().begin(), scene.getSplineControlPoints().end() };
        const auto first = scene.getShapes()[0].shapeHandle;
        const bodyId ground = scene.getShapes()[0].bodyHandle;
        const auto probe = scene.getShapes()[scene.getTerrainPointCount() - 1];
        scene.getWorld().SetBodyTransform( ground, { { 1.5f, 0.75f }, rot2::FromRadians( 0.25f ) } );
        scene.getWorld().SetShapeFriction( first, 0.63f );
        scene.getWorld().SetShapeFriction( probe.shapeHandle, 0.27f );
        scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, { 3.0f, 4.0f } );
        const transform2 pose = scene.getWorld().GetBodyTransform( probe.bodyHandle );
        demoInput drag{};
        drag.mousePosition = TransformPoint( scene.getWorld().GetBodyTransform( ground ), scene.getSplineControlPoints()[1] );
        drag.mousePressed = drag.mouseHeld = true;
        scene.handleInput( drag );
        check( scene.resetSplineTerrain(), "edited terrain resets to its original shape" );
        check( scene.getSplineControlPoints().size() == original.size() && scene.getSplineSamplingError() == 0.01f && !scene.getTerrainPointDragIndex(), "reset restores nine controls and one centimeter and ends dragging" );
        for( std::size_t i = 0; i < original.size(); ++i ) check( near( scene.getSplineControlPoints()[i], original[i] ), "reset restores each original control" );
        const auto samples = SampleAdaptive( { original }, 0.01f );
        check( scene.getTerrainPointCount() == samples.size(), "reset rebuilds original adaptive samples" );
        for( std::size_t i = 0; i < samples.size(); ++i ) check( near( scene.getTerrainPoint( i ), samples[i] ), "reset restores every local collision point" );
        check( scene.getShapes()[0].bodyHandle == ground && near( scene.getWorld().GetBodyTransform( ground ).position, { 1.5f, 0.75f } ), "terrain reset preserves the existing ground body transform" );
        check( scene.getShapes()[0].shapeHandle == first && scene.getWorld().GetShape( first ).friction == 0.63f, "terrain reset preserves surviving edge material" );
        check( scene.getShapes()[samples.size() - 1].shapeHandle == probe.shapeHandle && scene.getWorld().GetShape( probe.shapeHandle ).friction == 0.27f, "terrain reset preserves probe identity and material" );
        check( near( scene.getWorld().GetBodyTransform( probe.bodyHandle ).position, pose.position ) && near( scene.getWorld().GetBodyLinearVelocity( probe.bodyHandle ), { 3.0f, 4.0f } ), "terrain reset preserves current probe pose and motion" );
        check( !scene.resetSplineTerrain() && scene.canUndoSplineEdit(), "unchanged reset retains the previous undo" );
        check( scene.undoSplineEdit() && scene.getSplineSamplingError() == 0.005f && scene.getSplineControlPoints().size() == edited.size(), "undo restores the pre-reset controls and error" );
        for( std::size_t i = 0; i < edited.size(); ++i ) check( near( scene.getSplineControlPoints()[i], edited[i] ), "undo restores every edited control" );
        check( scene.redoSplineEdit() && scene.getSplineSamplingError() == 0.01f, "redo reapplies terrain reset" );
        check( !scene.resetSplineTerrain() && scene.undoSplineEdit(), "unchanged reset after redo does not replace history" );
        check( scene.resetSplineTerrain() && !scene.canRedoSplineEdit(), "new reset clears an existing redo" );
        check( scene.undoSplineEdit(), "failure fixture restores edited geometry" );
        scene.getWorld().DestroyChain( scene.getWorld().GetShapeParentChain( scene.getShapes()[0].shapeHandle ) );
        check( !scene.resetSplineTerrain() && scene.getSplineControlPoints().size() == edited.size() && scene.getSplineSamplingError() == 0.005f && scene.canRedoSplineEdit(), "failed reset preserves controls, error and redo" );
        rigidBodyDemo polygon{ demoKind::terrainChain };
        check( !polygon.resetSplineTerrain() && !polygon.canUndoSplineEdit(), "spline reset does not affect polygon terrain" );
    }

    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        while( scene.getSplineControlPoints().size() > 2 ) check( scene.removeSplineControlPoint( scene.getSplineControlPoints().size() - 1 ), "short terrain trial fixture removes endpoints" );
        check( scene.getTerrainPointCount() == 2, "short terrain trial fixture has one edge" );
        const vec2 a = scene.getTerrainPoint( 0 ), b = scene.getTerrainPoint( 1 );
        const vec2 tangent = Normalize( b - a ), normal = Cross( tangent, 1.0f );
        for( const float fraction : { 0.0f, 0.25f, 0.5f, 1.0f } )
        {
            scene.setSplineTrialFraction( fraction );
            for( std::size_t probe = 0; probe < 3; ++probe )
                for( const terrainTrial trial : { terrainTrial::front, terrainTrial::back, terrainTrial::seams } )
                {
                    scene.launchTerrainProbe( probe, trial );
                    const bodyId body = scene.getShapes()[1 + probe].bodyHandle;
                    const float side = trial == terrainTrial::back ? -1.0f : 1.0f;
                    const vec2 target = trial == terrainTrial::seams ? 0.5f * ( a + b ) : a + fraction * ( b - a );
                    check( near( scene.getWorld().GetBodyTransform( body ).position, target + side * 1.5f * normal ), "one-edge terrain trials launch from its valid edge" );
                    check( near( scene.getWorld().GetBodyLinearVelocity( body ), trial == terrainTrial::seams ? 6.0f * tangent : -side * 8.0f * normal ), "one-edge terrain trials retain front, back and seam directions" );
                }
        }
    }

    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        check( scene.getSplineTrialFraction() == 0.5f, "trial position defaults to half the sampled length" );
        scene.setSplineTrialFraction( 0.75f );
        check( scene.getSplineTrialFraction() == 0.75f && !scene.canUndoSplineEdit(), "trial position settings do not create geometry history" );
        while( scene.getSplineControlPoints().size() > 2 ) check( scene.removeSplineControlPoint( scene.getSplineControlPoints().size() - 1 ), "trial position fixture reduces to two controls" );
        check( scene.insertSplineControlPoint( 0 ) && scene.getTerrainPointCount() == 3, "trial position fixture has two child edges" );
        auto& simulation = scene.getWorld();
        const chainId parent = simulation.GetShapeParentChain( scene.getShapes()[0].shapeHandle );
        const std::array<vec2, 3> points{ vec2{ 0, 0 }, vec2{ 3, 0 }, vec2{ 3, 4 } };
        check( simulation.SetChainPoints( parent, points, { -3, 0 }, { 3, 8 } ), "trial fixture has unequal three- and four-meter edges" );
        const transform2 pose{ { 1.5f, 0.75f }, rot2::FromRadians( 0.25f ) };
        simulation.SetBodyTransform( scene.getShapes()[0].bodyHandle, pose );
        const std::array<float, 4> fractions{ 0.0f, 0.25f, 0.75f, 1.0f };
        const std::array<vec2, 4> targets{ vec2{ 0, 0 }, vec2{ 1.75f, 0 }, vec2{ 3, 2.25f }, vec2{ 3, 4 } };
        for( std::size_t i = 0; i < fractions.size(); ++i )
            for( std::size_t probe = 0; probe < 3; ++probe )
                for( const terrainTrial trial : { terrainTrial::front, terrainTrial::back } )
                {
                    scene.setSplineTrialFraction( fractions[i] );
                    const bodyId body = scene.getShapes()[2 + probe].bodyHandle;
                    const vec2 tangent = TransformVector( pose, i < 2 ? vec2{ 1, 0 } : vec2{ 0, 1 } );
                    const transform2 beforePose = simulation.GetBodyTransform( body );
                    const vec2 beforeVelocity = simulation.GetBodyLinearVelocity( body );
                    const int beforeContacts = simulation.GetContactCount();
                    const rigidBodyDemo& readonly = scene;
                    const auto target = readonly.getTerrainTrialTarget( trial );
                    check( target && near( target->point, TransformPoint( pose, targets[i] ) ) && near( target->tangent, tangent ), "read-only target query returns the selected world point and tangent" );
                    check( near( simulation.GetBodyTransform( body ).position, beforePose.position ) && near( simulation.GetBodyLinearVelocity( body ), beforeVelocity ) && simulation.GetContactCount() == beforeContacts, "target query preserves probe pose, motion and contacts" );
                    scene.launchTerrainProbe( probe, trial );
                    const vec2 normal = Cross( tangent, 1.0f );
                    const float side = trial == terrainTrial::back ? -1.0f : 1.0f;
                    check( near( simulation.GetBodyTransform( body ).position, TransformPoint( pose, targets[i] ) + side * 1.5f * normal ), "launch uses sampled length fraction and the selected edge in world space" );
                    check( near( simulation.GetBodyLinearVelocity( body ), -side * 8.0f * normal ), "selected trial position preserves front/back launch speed and direction" );
                }
        const std::array<vec2, 3> equalEdges{ vec2{ 0, 0 }, vec2{ 3, 0 }, vec2{ 3, 3 } };
        check( simulation.SetChainPoints( parent, equalEdges, { -3, 0 }, { 3, 6 } ), "join fixture has equal edge lengths" );
        scene.setSplineTrialFraction( 0.5f );
        const auto join = scene.getTerrainTrialTarget( terrainTrial::front );
        check( join && near( join->point, TransformPoint( pose, { 3, 0 } ) ) && near( join->tangent, TransformVector( pose, { 1, 0 } ) ), "join target query uses the ending edge" );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        const vec2 firstNormal = Cross( TransformVector( pose, { 1, 0 } ), 1.0f );
        check( near( simulation.GetBodyTransform( scene.getImpulseBody() ).position, TransformPoint( pose, { 3, 0 } ) + 1.5f * firstNormal ), "an exact join uses the edge ending at that join" );
        scene.setSplineTrialFraction( 1.0f );
        scene.launchTerrainProbe( 0, terrainTrial::seams );
        check( near( simulation.GetBodyTransform( scene.getImpulseBody() ).position, TransformPoint( pose, { 1.5f, 0 } ) + 1.5f * firstNormal ), "seam trial keeps its first-edge midpoint regardless of fraction" );
        for( const float invalid : { -0.1f, 1.1f, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity() } ) scene.setSplineTrialFraction( invalid );
        check( scene.getSplineTrialFraction() == 1.0f, "invalid trial fractions leave the setting unchanged" );
        scene.resetTerrainProbes();
        check( scene.resetSplineTerrain() && scene.undoSplineEdit() && scene.redoSplineEdit(), "position persistence fixture resets and restores geometry" );
        check( scene.getSplineTrialFraction() == 1.0f, "probe and geometry reset and undo/redo preserve the trial fraction" );
        const auto restored = scene.getTerrainTrialTarget( terrainTrial::front );
        check( restored && near( restored->point, TransformPoint( pose, { -8.0f, -1.2f } ) ), "target query follows geometry reset and undo/redo" );
        rigidBodyDemo polygon{ demoKind::terrainChain };
        polygon.setSplineTrialFraction( 0.75f );
        check( polygon.getSplineTrialFraction() == 0.5f && !polygon.canUndoSplineEdit(), "trial fraction editing is spline-only and does not record geometry history" );
        const auto polygonTarget = polygon.getTerrainTrialTarget( terrainTrial::front );
        check( polygonTarget && near( polygonTarget->point, 0.5f * ( polygon.getTerrainPoint( 3 ) + polygon.getTerrainPoint( 4 ) ) ), "polygon target query preserves the existing launch location" );
        simulation.DestroyChain( simulation.GetShapeParentChain( scene.getShapes()[0].shapeHandle ) );
        check( !scene.getTerrainTrialTarget( terrainTrial::front ), "retired terrain has no preview target" );
        rigidBodyDemo playground{ demoKind::playground };
        check( !playground.getTerrainTrialTarget( terrainTrial::front ), "non-terrain scenes have no preview target" );
    }

    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        while( scene.getSplineControlPoints().size() > 2 ) check( scene.removeSplineControlPoint( scene.getSplineControlPoints().size() - 1 ), "trial picking fixture reduces control count" );
        check( scene.insertSplineControlPoint( 0 ), "trial picking fixture creates two edges" );
        scene.setSplineSamplingError( 0.02f );
        check( scene.undoSplineEdit() && scene.canRedoSplineEdit() && !scene.canUndoSplineEdit(), "trial picking fixture retains redo" );
        auto& simulation = scene.getWorld();
        const chainId parent = simulation.GetShapeParentChain( scene.getShapes()[0].shapeHandle );
        const std::array<vec2, 3> points{ vec2{ 0, 0 }, vec2{ 3, 0 }, vec2{ 3, 4 } };
        check( simulation.SetChainPoints( parent, points, { -3, 0 }, { 3, 8 } ), "trial picking uses unequal edge lengths" );
        const transform2 pose{ { 1.5f, 0.75f }, rot2::FromRadians( 0.25f ) };
        simulation.SetBodyTransform( scene.getShapes()[0].bodyHandle, pose );
        const auto probe = scene.getShapes()[2];
        simulation.SetBodyLinearVelocity( probe.bodyHandle, { 2, 3 } );
        const transform2 beforePose = simulation.GetBodyTransform( probe.bodyHandle );
        const std::vector<vec2> controls{ scene.getSplineControlPoints().begin(), scene.getSplineControlPoints().end() };
        demoInput pick{};
        pick.mousePressed = pick.mouseHeld = pick.selectTrialPosition = true;
        pick.mousePosition = TransformPoint( pose, { 1.75f, -2 } );
        scene.handleInput( pick );
        check( std::abs( scene.getSplineTrialFraction() - 0.25f ) < 1.0e-6f, "Shift press selects the closest point by cumulative length" );
        pick.mousePressed = pick.selectTrialPosition = false;
        pick.mousePosition = TransformPoint( pose, { 5, 2.25f } );
        scene.handleInput( pick );
        check( std::abs( scene.getSplineTrialFraction() - 0.75f ) < 1.0e-6f, "trial drag remains active after releasing Shift" );
        pick.mousePosition = TransformPoint( pose, { -10, 0 } );
        scene.handleInput( pick );
        check( scene.getSplineTrialFraction() == 0.0f, "trial projection clamps before the path start" );
        pick.mousePosition = TransformPoint( pose, { 3, 10 } );
        scene.handleInput( pick );
        check( scene.getSplineTrialFraction() == 1.0f, "trial projection clamps after the path end" );
        scene.cancelInput();
        pick.mousePosition = TransformPoint( pose, { 1.75f, -2 } );
        scene.handleInput( pick );
        check( scene.getSplineTrialFraction() == 1.0f, "cancelled trial selection needs a fresh press" );
        pick.mousePressed = pick.selectTrialPosition = true;
        scene.handleInput( pick );
        check( std::abs( scene.getSplineTrialFraction() - 0.25f ) < 1.0e-6f, "a fresh Shift press restarts trial selection" );
        pick.mousePressed = pick.mouseHeld = false;
        scene.handleInput( pick );
        pick.mouseHeld = true;
        pick.mousePosition = TransformPoint( pose, { 5, 2.25f } );
        scene.handleInput( pick );
        check( std::abs( scene.getSplineTrialFraction() - 0.25f ) < 1.0e-6f, "mouse release closes trial selection" );
        simulation.SetBodyTransform( scene.getShapes()[0].bodyHandle, {} );
        pick.mousePressed = true;
        pick.mousePosition = { 2, 1 };
        scene.handleInput( pick );
        check( std::abs( scene.getSplineTrialFraction() - 2.0f / 7.0f ) < 1.0e-6f, "equal-distance projections prefer the earlier edge" );
        pick.mousePosition = simulation.GetBodyTransform( probe.bodyHandle ).position;
        scene.handleInput( pick );
        check( !simulation.IsValid( scene.getMouseJoint() ) && !scene.getTerrainPointDragIndex(), "trial picking does not grab probes or controls" );
        check( near( simulation.GetBodyTransform( probe.bodyHandle ).position, beforePose.position ) && near( simulation.GetBodyLinearVelocity( probe.bodyHandle ), { 2, 3 } ), "trial picking preserves probe pose and velocity" );
        check( !scene.canUndoSplineEdit() && scene.canRedoSplineEdit() && scene.getSplineSamplingError() == 0.01f, "trial picking preserves geometry history and error" );
        for( std::size_t i = 0; i < controls.size(); ++i ) check( near( scene.getSplineControlPoints()[i], controls[i] ), "trial picking does not move controls" );
        for( std::size_t i = 0; i < points.size(); ++i ) check( near( scene.getTerrainPoint( i ), points[i] ), "trial picking does not change collision geometry" );
        simulation.DestroyChain( parent );
        scene.handleInput( pick );
        scene.cancelInput();
    }

    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        demoInput drag{};
        drag.mousePosition = scene.getSplineControlPoints()[1];
        drag.mousePressed = drag.mouseHeld = true;
        scene.handleInput( drag );
        drag.mousePressed = false;
        drag.selectTrialPosition = true;
        drag.mousePosition.y += 0.5f;
        scene.handleInput( drag );
        check( near( scene.getSplineControlPoints()[1], drag.mousePosition ) && scene.getSplineTrialFraction() == 0.5f, "pressing Shift mid-control-drag does not switch modes" );
        scene.cancelInput();
        drag.mousePosition = scene.getWorld().GetBodyTransform( scene.getImpulseBody() ).position;
        drag.mousePressed = true;
        drag.selectTrialPosition = false;
        scene.handleInput( drag );
        check( scene.getWorld().IsValid( scene.getMouseJoint() ), "ordinary left drag still grabs the probe" );
        drag.mousePressed = false;
        drag.selectTrialPosition = true;
        scene.handleInput( drag );
        check( scene.getWorld().IsValid( scene.getMouseJoint() ) && scene.getSplineTrialFraction() == 0.5f, "pressing Shift mid-probe-drag does not switch modes" );
        scene.cancelInput();
        rigidBodyDemo polygon{ demoKind::terrainChain };
        drag.mousePosition = polygon.getTerrainPoint( 1 );
        drag.mousePressed = true;
        polygon.handleInput( drag );
        check( polygon.getTerrainPointDragIndex() == 1 && polygon.getSplineTrialFraction() == 0.5f, "Shift trial selection is spline-only" );
    }

    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        auto& simulation = scene.getWorld();
        check( scene.getSplineTrialSpeed() == 8.0f, "trial speed defaults to eight meters per second" );
        const transform2 pose{ { 1.5f, 0.75f }, rot2::FromRadians( 0.25f ) };
        simulation.SetBodyTransform( scene.getShapes()[0].bodyHandle, pose );
        scene.setSplineTrialFraction( 0.25f );
        const auto target = scene.getTerrainTrialTarget( terrainTrial::front );
        check( target.has_value(), "speed fixture has a trial target" );
        const vec2 normal = Cross( target->tangent, 1.0f );
        const bodyId firstProbe = scene.getImpulseBody();
        const transform2 beforePose = simulation.GetBodyTransform( firstProbe );
        simulation.SetBodyLinearVelocity( firstProbe, { 2, 3 } );
        scene.setSplineTrialSpeed( 12.5f );
        check( scene.getSplineTrialSpeed() == 12.5f && near( simulation.GetBodyLinearVelocity( firstProbe ), { 2, 3 } ) && near( simulation.GetBodyTransform( firstProbe ).position, beforePose.position ), "speed settings apply only to the next launch" );
        check( !scene.canUndoSplineEdit() && !scene.canRedoSplineEdit() && scene.getSplineTrialFraction() == 0.25f, "speed settings preserve trial position and geometry history" );
        for( const float speed : { 1.0f, 8.0f, 12.5f, 20.0f } )
            for( std::size_t probe = 0; probe < 3; ++probe )
                for( const terrainTrial trial : { terrainTrial::front, terrainTrial::back } )
                {
                    scene.setSplineTrialSpeed( speed );
                    scene.launchTerrainProbe( probe, trial );
                    const bodyId body = scene.getShapes()[scene.getTerrainPointCount() - 1 + probe].bodyHandle;
                    const float side = trial == terrainTrial::back ? -1.0f : 1.0f;
                    check( near( simulation.GetBodyLinearVelocity( body ), -side * speed * normal ), "every probe uses selected front/back speed and direction" );
                    check( near( simulation.GetBodyTransform( body ).position, target->point + side * 1.5f * normal ), "speed does not change launch position or distance" );
                }
        for( const float invalid : { 0.0f, 0.99f, 20.01f, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity() } ) scene.setSplineTrialSpeed( invalid );
        check( scene.getSplineTrialSpeed() == 20.0f, "invalid speeds preserve the last valid setting" );
        const auto seam = scene.getTerrainTrialTarget( terrainTrial::seams );
        check( seam.has_value(), "speed fixture has a seam target" );
        scene.launchTerrainProbe( 0, terrainTrial::seams );
        check( near( simulation.GetBodyLinearVelocity( firstProbe ), 6.0f * seam->tangent ), "seam trial retains its six-meter-per-second speed" );
        scene.resetTerrainProbes();
        check( scene.insertSplineControlPoint( 1 ) && scene.resetSplineTerrain() && scene.undoSplineEdit() && scene.redoSplineEdit(), "speed persistence fixture edits and restores terrain" );
        check( scene.getSplineTrialSpeed() == 20.0f && scene.getSplineTrialFraction() == 0.25f, "probe reset and terrain editing/history preserve speed and position settings" );
        rigidBodyDemo reset{ demoKind::splineTerrain };
        check( reset.getSplineTrialSpeed() == 8.0f, "fresh scenes restore default trial speed" );
        rigidBodyDemo polygon{ demoKind::terrainChain };
        polygon.setSplineTrialSpeed( 20.0f );
        polygon.launchTerrainProbe( 0, terrainTrial::front );
        check( polygon.getSplineTrialSpeed() == 8.0f && std::abs( Length( polygon.getWorld().GetBodyLinearVelocity( polygon.getImpulseBody() ) ) - 8.0f ) < 1.0e-5f, "polygon terrain keeps its existing trial speed" );
    }

    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        check( !scene.getTerrainTrialObservation().active, "fresh scenes have no trial observation" );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        scene.step( 0.0f, 4 );
        check( scene.getTerrainTrialObservation().elapsed == 0.0f && !scene.getTerrainTrialObservation().firstContactTime, "zero-time steps do not advance trial observations" );
        scene.step( 1.0f / 60.0f, 4 );
        const auto before = scene.getTerrainTrialObservation();
        scene.setSplineTrialFraction( 0.75f );
        scene.setSplineTrialSpeed( 12.5f );
        check( !scene.insertSplineControlPoint( 99 ), "invalid edit observation fixture is rejected" );
        scene.setSplineSamplingError( scene.getSplineSamplingError() );
        check( scene.getTerrainTrialObservation().active && scene.getTerrainTrialObservation().elapsed == before.elapsed, "next-launch settings and failed/unchanged edits retain observation" );
        check( scene.insertSplineControlPoint( 1 ) && !scene.getTerrainTrialObservation().active, "successful geometry edits clear observation" );
        scene.launchTerrainProbe( 2, terrainTrial::back );
        check( scene.getTerrainTrialObservation().active && scene.getTerrainTrialObservation().probeIndex == 2 && scene.getTerrainTrialObservation().trial == terrainTrial::back && scene.getTerrainTrialObservation().elapsed == 0.0f, "a new launch replaces the old observation" );
        check( scene.undoSplineEdit() && !scene.getTerrainTrialObservation().active, "geometry undo clears observation" );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        check( scene.redoSplineEdit() && !scene.getTerrainTrialObservation().active, "geometry redo clears observation" );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        check( scene.resetSplineTerrain() && !scene.getTerrainTrialObservation().active, "terrain restoration clears observation" );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        check( !scene.resetSplineTerrain() && scene.getTerrainTrialObservation().active, "unchanged terrain restoration preserves observation" );
        scene.getWorld().DestroyChain( scene.getWorld().GetShapeParentChain( scene.getShapes()[0].shapeHandle ) );
        scene.step( 1.0f / 60.0f, 4 );
        check( !scene.getTerrainTrialObservation().active, "retired terrain invalidates observation on the next step" );
    }

    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        auto& simulation = scene.getWorld();
        simulation.SetGravity( {} );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        const auto circle = scene.getShapes()[scene.getTerrainPointCount() - 1];
        const auto capsule = scene.getShapes()[scene.getTerrainPointCount()];
        simulation.SetBodyTransform( circle.bodyHandle, { { 0, 10 }, {} } );
        simulation.SetBodyTransform( capsule.bodyHandle, { { 0.1f, 10 }, {} } );
        simulation.SetBodyLinearVelocity( circle.bodyHandle, {} );
        simulation.SetBodyLinearVelocity( capsule.bodyHandle, {} );
        scene.step( 1.0f / 60.0f, 4 );
        bool probeContact = false;
        for( const auto& contact : scene.getContacts() )
            probeContact |= contact.manifold.pointCount > 0 && ( ( contact.shapeA == circle.shapeHandle && contact.shapeB == capsule.shapeHandle ) || ( contact.shapeA == capsule.shapeHandle && contact.shapeB == circle.shapeHandle ) );
        check( probeContact, "probe-only contact fixture must touch" );
        check( scene.getTerrainTrialObservation().active && !scene.getTerrainTrialObservation().touching && !scene.getTerrainTrialObservation().firstContactTime, "probe-probe contacts do not count as terrain observations" );
        rigidBodyDemo polygon{ demoKind::terrainChain };
        polygon.launchTerrainProbe( 0, terrainTrial::front );
        check( !polygon.getTerrainTrialObservation().active, "new observation UI is spline-only" );
    }

    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        scene.setSplineTrialFraction( 0.25f );
        scene.setSplineTrialSpeed( 12.5f );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        scene.setSplineTrialFraction( 0.75f );
        scene.setSplineTrialSpeed( 20.0f );
        scene.step( 1.0f / 60.0f, 4 );
        auto observation = scene.getTerrainTrialObservation();
        check( observation.launchFraction == 0.25f && observation.launchSpeed == 12.5f, "next-launch settings and stepping do not overwrite recorded launch conditions" );
        scene.launchTerrainProbe( 99, terrainTrial::back );
        observation = scene.getTerrainTrialObservation();
        check( observation.launchFraction == 0.25f && observation.launchSpeed == 12.5f && observation.trial == terrainTrial::front, "failed launches preserve previous conditions" );
        scene.launchTerrainProbe( 2, terrainTrial::back );
        observation = scene.getTerrainTrialObservation();
        check( observation.launchFraction == 0.75f && observation.launchSpeed == 20.0f && observation.probeIndex == 2, "successful new launches replace conditions with their applied settings" );
        scene.launchTerrainProbe( 1, terrainTrial::seams );
        observation = scene.getTerrainTrialObservation();
        check( !observation.launchFraction && observation.launchSpeed == 6.0f, "seam conditions record the fixed starting edge and actual six-meter-per-second speed" );
        scene.resetTerrainProbes();
        observation = scene.getTerrainTrialObservation();
        check( !observation.active && !observation.launchFraction && observation.launchSpeed == 0.0f, "clearing observation also clears recorded conditions" );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        check( scene.insertSplineControlPoint( 1 ), "conditions clearing fixture edits terrain" );
        observation = scene.getTerrainTrialObservation();
        check( !observation.active && !observation.launchFraction && observation.launchSpeed == 0.0f, "successful geometry edits clear old launch conditions" );
    }

    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        check( !scene.getTerrainTrialReference() && !scene.pinTerrainTrialReference(), "fresh scene has no reference and cannot pin an absent trial" );
        scene.launchTerrainProbe( 1, terrainTrial::front );
        check( !scene.pinTerrainTrialReference(), "reference needs at least one completed positive-duration step" );
        for( int i = 0; i < 120; ++i ) scene.step( 1.0f / 60.0f, 4 );
        const auto baseline = scene.getTerrainTrialObservation();
        check( baseline.firstContactTime.has_value() && scene.pinTerrainTrialReference(), "a progressed trial can be pinned with its contact result" );
        scene.step( 1.0f / 60.0f, 4 );
        scene.setSplineTrialFraction( 0.75f );
        scene.setSplineTrialSpeed( 12.5f );
        scene.resetTerrainProbes();
        auto reference = scene.getTerrainTrialReference();
        check( reference && reference->elapsed == baseline.elapsed && reference->firstContactTime == baseline.firstContactTime && reference->touching == baseline.touching && reference->launchSpeed == 8.0f && reference->launchFraction == 0.5f && reference->probeIndex == 1 && reference->trial == terrainTrial::front, "reference retains the full copied result across progress, settings and probe reset" );
        check( !scene.pinTerrainTrialReference() && scene.getTerrainTrialReference()->elapsed == baseline.elapsed, "failed pin preserves the old reference" );
        reference->elapsed = 99.0f;
        check( scene.getTerrainTrialReference()->elapsed == baseline.elapsed, "reference queries return independent value snapshots" );
        scene.launchTerrainProbe( 2, terrainTrial::back );
        check( !scene.pinTerrainTrialReference(), "an unprogressed new launch cannot replace the reference" );
        scene.step( 1.0f / 60.0f, 4 );
        check( scene.getTerrainTrialReference()->probeIndex == 1, "new launches retain the pinned baseline until explicitly replaced" );
        check( scene.pinTerrainTrialReference(), "pinning again replaces the reference" );
        reference = scene.getTerrainTrialReference();
        check( reference && reference->probeIndex == 2 && reference->trial == terrainTrial::back && reference->launchSpeed == 12.5f && reference->launchFraction == 0.75f && reference->elapsed == 1.0f / 60.0f, "replacement reference uses the new trial's copied conditions and timing" );
        const auto current = scene.getTerrainTrialObservation();
        scene.clearTerrainTrialReference();
        check( !scene.getTerrainTrialReference() && scene.getTerrainTrialObservation().elapsed == current.elapsed && scene.getTerrainTrialObservation().active, "clearing the reference preserves current observation" );
        check( scene.pinTerrainTrialReference(), "cleared reference can be pinned again" );
        check( !scene.insertSplineControlPoint( 99 ), "invalid edit reference fixture is rejected" );
        scene.setSplineSamplingError( scene.getSplineSamplingError() );
        check( scene.getTerrainTrialReference().has_value(), "failed and unchanged edits retain reference" );
        check( scene.insertSplineControlPoint( 1 ) && !scene.getTerrainTrialReference(), "successful geometry edits clear reference" );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        scene.step( 1.0f / 60.0f, 4 );
        check( scene.pinTerrainTrialReference() && scene.undoSplineEdit() && !scene.getTerrainTrialReference(), "geometry undo clears reference" );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        scene.step( 1.0f / 60.0f, 4 );
        check( scene.pinTerrainTrialReference() && scene.redoSplineEdit() && !scene.getTerrainTrialReference(), "geometry redo clears reference" );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        scene.step( 1.0f / 60.0f, 4 );
        check( scene.pinTerrainTrialReference() && scene.resetSplineTerrain() && !scene.getTerrainTrialReference(), "terrain restoration clears reference" );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        scene.step( 1.0f / 60.0f, 4 );
        check( scene.pinTerrainTrialReference() && !scene.resetSplineTerrain() && scene.getTerrainTrialReference(), "unchanged restoration retains reference" );
        scene.resetTerrainProbes();
        scene.getWorld().DestroyChain( scene.getWorld().GetShapeParentChain( scene.getShapes()[0].shapeHandle ) );
        scene.step( 1.0f / 60.0f, 4 );
        check( !scene.getTerrainTrialReference(), "retired terrain clears a reference even without an active observation" );
        rigidBodyDemo polygon{ demoKind::terrainChain };
        polygon.launchTerrainProbe( 0, terrainTrial::front );
        polygon.step( 1.0f / 60.0f, 4 );
        check( !polygon.pinTerrainTrialReference() && !polygon.getTerrainTrialReference(), "trial references are spline-only" );
    }

    for( terrainTrial trial : { terrainTrial::front, terrainTrial::back, terrainTrial::seams } )
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        check( !scene.relaunchTerrainTrialReference() && !scene.getTerrainTrialObservation().active, "relaunch without reference does not launch" );
        check( scene.insertSplineControlPoint( 1 ) && scene.undoSplineEdit(), "relaunch fixture retains redo history" );
        scene.setSplineTrialFraction( 0.25f );
        scene.setSplineTrialSpeed( 12.5f );
        scene.launchTerrainProbe( 1, trial );
        const auto body = scene.getShapes()[scene.getTerrainPointCount()].bodyHandle;
        const transform2 launchPose = scene.getWorld().GetBodyTransform( body );
        const vec2 launchVelocity = scene.getWorld().GetBodyLinearVelocity( body );
        scene.step( 1.0f / 60.0f, 4 );
        check( scene.pinTerrainTrialReference(), "relaunch fixture pins progressed trial" );
        const auto reference = *scene.getTerrainTrialReference();
        scene.setSplineTrialFraction( 0.75f );
        scene.setSplineTrialSpeed( 3.0f );
        scene.launchTerrainProbe( 2, terrainTrial::back );
        scene.getWorld().SetShapeFriction( scene.getShapes()[0].shapeHandle, 0.37f );
        scene.getWorld().SetGravity( { 1.0f, -3.0f } );
        check( scene.relaunchTerrainTrialReference(), "reference conditions can relaunch through the existing launch path" );
        const auto current = scene.getTerrainTrialObservation();
        check( current.active && current.probeIndex == 1 && current.trial == trial && current.launchSpeed == reference.launchSpeed && current.launchFraction == reference.launchFraction, "relaunch restores recorded probe and launch conditions" );
        check( current.elapsed == 0.0f && !current.touching && !current.firstContactTime, "relaunch starts a fresh observation" );
        check( near( scene.getWorld().GetBodyTransform( body ).position, launchPose.position ) && near( scene.getWorld().GetBodyLinearVelocity( body ), launchVelocity ), "relaunch restores the initial pose and velocity rather than the progressed state" );
        check( scene.getSplineTrialFraction() == ( trial == terrainTrial::seams ? 0.75f : 0.25f ) && scene.getSplineTrialSpeed() == ( trial == terrainTrial::seams ? 3.0f : 12.5f ), "relaunch synchronizes front/back settings and retains unused seam settings" );
        check( scene.getTerrainTrialReference()->elapsed == reference.elapsed && scene.getTerrainTrialReference()->launchFraction == reference.launchFraction && scene.canRedoSplineEdit() && !scene.canUndoSplineEdit(), "relaunch preserves pinned result and edit history" );
        check( scene.getWorld().GetShapeFriction( scene.getShapes()[0].shapeHandle ) == 0.37f, "relaunch uses current terrain material" );
        check( near( scene.getWorld().GetGravity(), { 1.0f, -3.0f } ), "relaunch preserves current gravity" );
        scene.getWorld().DestroyBody( body );
        const auto before = scene.getTerrainTrialObservation();
        scene.setSplineTrialFraction( 0.6f );
        scene.setSplineTrialSpeed( 4.0f );
        check( !scene.relaunchTerrainTrialReference() && scene.getSplineTrialFraction() == 0.6f && scene.getSplineTrialSpeed() == 4.0f && scene.getTerrainTrialObservation().probeIndex == before.probeIndex, "retired probe rejects relaunch before changing settings or observation" );
    }

    for( bool retireChain : { false, true } )
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        scene.setSplineTrialFraction( 0.25f );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        scene.step( 1.0f / 60.0f, 4 );
        check( scene.pinTerrainTrialReference(), "invalid relaunch fixture pins a trial" );
        scene.setSplineTrialFraction( 0.75f );
        scene.setSplineTrialSpeed( 4.0f );
        if( retireChain ) scene.getWorld().DestroyChain( scene.getWorld().GetShapeParentChain( scene.getShapes()[0].shapeHandle ) );
        else scene.getWorld().DestroyShape( scene.getShapes()[scene.getTerrainPointCount() - 1].shapeHandle );
        check( !scene.relaunchTerrainTrialReference() && scene.getSplineTrialFraction() == 0.75f && scene.getSplineTrialSpeed() == 4.0f && scene.getTerrainTrialObservation().elapsed > 0.0f && scene.getTerrainTrialReference(), "invalid terrain or probe shape rejects relaunch without changing state" );
    }

    for( bool slower : { false, true } )
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::noReference, "comparison requires a pinned reference" );
        scene.getWorld().SetGravity( {} );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        const auto probe = scene.getShapes()[scene.getTerrainPointCount() - 1];
        const auto target = *scene.getTerrainTrialTarget( terrainTrial::front );
        scene.getWorld().SetBodyTransform( probe.bodyHandle, { target.point + 0.4f * Cross( target.tangent, 1.0f ), {} } );
        scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
        scene.step( slower ? 0.125f : 0.25f, 4 );
        check( scene.getTerrainTrialObservation().firstContactTime && scene.pinTerrainTrialReference(), "comparison fixture pins a first contact observation" );
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::ready && scene.getTerrainTrialComparison().firstContactTimeDifference == 0.0f, "pinned current observation compares with zero time difference" );
        scene.setSplineTrialFraction( 0.75f );
        scene.setSplineTrialSpeed( 12.5f );
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::ready, "comparison uses recorded conditions rather than next-launch settings" );
        scene.resetTerrainProbes();
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::noObservation, "comparison distinguishes missing current observation" );
        check( scene.relaunchTerrainTrialReference(), "comparison fixture relaunches recorded conditions" );
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::waitingForContact, "matching conditions without both contact times do not produce a difference" );
        scene.getWorld().SetBodyTransform( probe.bodyHandle, { target.point + 0.4f * Cross( target.tangent, 1.0f ), {} } );
        scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
        scene.step( slower ? 0.25f : 0.125f, 4 );
        const auto before = scene.getTerrainTrialObservation();
        const auto pose = scene.getWorld().GetBodyTransform( probe.bodyHandle );
        const vec2 velocity = scene.getWorld().GetBodyLinearVelocity( probe.bodyHandle );
        const int contacts = scene.getWorld().GetContactCount();
        const auto comparison = scene.getTerrainTrialComparison();
        check( comparison.status == terrainTrialComparisonStatus::ready && comparison.firstContactTimeDifference == ( slower ? 0.125f : -0.125f ), "comparison subtracts reference from current first-contact observation time" );
        check( scene.getTerrainTrialObservation().elapsed == before.elapsed && scene.getTerrainTrialObservation().firstContactTime == before.firstContactTime && near( scene.getWorld().GetBodyTransform( probe.bodyHandle ).position, pose.position ) && near( scene.getWorld().GetBodyLinearVelocity( probe.bodyHandle ), velocity ) && scene.getWorld().GetContactCount() == contacts, "comparison query does not change observation or physics" );
        for( int mismatch = 0; mismatch < 4; ++mismatch )
        {
            scene.setSplineTrialFraction( mismatch == 2 ? 0.75f : 0.5f );
            scene.setSplineTrialSpeed( mismatch == 3 ? 12.5f : 8.0f );
            scene.launchTerrainProbe( mismatch == 0 ? 1 : 0, mismatch == 1 ? terrainTrial::back : terrainTrial::front );
            check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::differentConditions, "each recorded launch condition participates in comparison" );
        }
        check( scene.insertSplineControlPoint( 1 ) && scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::noReference, "geometry edits clear comparison availability" );
    }
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        scene.launchTerrainProbe( 0, terrainTrial::seams );
        scene.step( 1.0f / 60.0f, 4 );
        check( scene.pinTerrainTrialReference(), "seam comparison fixture pins a trial" );
        scene.setSplineTrialFraction( 0.75f );
        scene.setSplineTrialSpeed( 12.5f );
        scene.launchTerrainProbe( 0, terrainTrial::seams );
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::waitingForContact, "seam comparison ignores unused slider settings" );
        scene.clearTerrainTrialReference();
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::noReference, "clearing the reference clears comparison availability" );
    }

    for( int subStepCount : { 1, 4, 8 } )
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        scene.getWorld().SetGravity( {} );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        scene.step( 0.0625f, 2 );
        check( !scene.getTerrainTrialObservation().firstContactTime && scene.getTerrainTrialObservation().firstContactTimeStep == 0.0f && scene.getTerrainTrialObservation().firstContactSubStepCount == 0, "pre-contact observation has no contact step settings" );
        check( scene.pinTerrainTrialReference(), "contact settings fixture pins pre-contact observation" );
        const auto probe = scene.getShapes()[scene.getTerrainPointCount() - 1];
        const auto target = *scene.getTerrainTrialTarget( terrainTrial::front );
        scene.getWorld().SetBodyTransform( probe.bodyHandle, { target.point + 0.4f * Cross( target.tangent, 1.0f ), {} } );
        scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
        scene.step( 0.0f, 16 );
        check( !scene.getTerrainTrialObservation().firstContactTime && scene.getTerrainTrialObservation().firstContactSubStepCount == 0, "zero-duration step does not record contact settings" );
        scene.step( 0.125f, subStepCount );
        const auto contact = scene.getTerrainTrialObservation();
        check( contact.firstContactTime == 0.1875f && contact.firstContactTimeStep == 0.125f && contact.firstContactSubStepCount == subStepCount, "first contact records that step's duration and subdivision count" );
        check( !scene.getTerrainTrialReference()->firstContactTime && scene.getTerrainTrialReference()->firstContactTimeStep == 0.0f && scene.getTerrainTrialReference()->firstContactSubStepCount == 0, "pre-contact reference does not acquire later contact settings" );
        check( scene.pinTerrainTrialReference(), "contact settings can be pinned with the observation" );
        scene.step( 0.25f, 2 );
        check( scene.getTerrainTrialObservation().firstContactTimeStep == 0.125f && scene.getTerrainTrialObservation().firstContactSubStepCount == subStepCount && scene.getTerrainTrialReference()->firstContactTimeStep == 0.125f && scene.getTerrainTrialReference()->firstContactSubStepCount == subStepCount, "later steps preserve first contact and pinned settings" );
        check( scene.relaunchTerrainTrialReference() && !scene.getTerrainTrialObservation().firstContactTime && scene.getTerrainTrialObservation().firstContactTimeStep == 0.0f && scene.getTerrainTrialObservation().firstContactSubStepCount == 0 && scene.getTerrainTrialReference()->firstContactSubStepCount == subStepCount, "relaunch clears current contact settings and retains the reference" );
        scene.getWorld().SetBodyTransform( probe.bodyHandle, { target.point + 0.4f * Cross( target.tangent, 1.0f ), {} } );
        scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
        scene.step( 0.0625f, 2 );
        check( scene.getTerrainTrialObservation().firstContactTimeStep == 0.0625f && scene.getTerrainTrialObservation().firstContactSubStepCount == 2 && scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::ready, "new trial records new contact settings without changing comparison policy" );
        scene.resetTerrainProbes();
        check( scene.getTerrainTrialObservation().firstContactTimeStep == 0.0f && scene.getTerrainTrialObservation().firstContactSubStepCount == 0 && scene.getTerrainTrialReference()->firstContactSubStepCount == subStepCount, "probe reset clears current contact settings and retains pinned settings" );
        check( scene.insertSplineControlPoint( 1 ) && !scene.getTerrainTrialReference(), "terrain edits clear the pinned contact settings with its observation" );
    }

    for( int difference = 0; difference < 4; ++difference )
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        scene.getWorld().SetGravity( {} );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        const auto probe = scene.getShapes()[scene.getTerrainPointCount() - 1];
        const auto target = *scene.getTerrainTrialTarget( terrainTrial::front );
        scene.getWorld().SetBodyTransform( probe.bodyHandle, { target.point + 0.4f * Cross( target.tangent, 1.0f ), {} } );
        scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
        scene.step( 0.125f, 4 );
        check( scene.pinTerrainTrialReference(), "step comparison fixture pins first contact" );
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::ready && scene.getTerrainTrialComparison().contactStepSettingsMatch, "identical recorded contact step settings match" );
        check( scene.relaunchTerrainTrialReference() && scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::waitingForContact, "step comparison waits for both contact observations" );
        scene.getWorld().SetBodyTransform( probe.bodyHandle, { target.point + 0.4f * Cross( target.tangent, 1.0f ), {} } );
        scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
        scene.step( difference & 1 ? 0.25f : 0.125f, difference & 2 ? 8 : 4 );
        const auto comparison = scene.getTerrainTrialComparison();
        check( comparison.status == terrainTrialComparisonStatus::ready && comparison.contactStepSettingsMatch == ( difference == 0 ), "duration and subdivision count both participate in contact step comparison" );
        check( comparison.firstContactTimeDifference == ( difference & 1 ? 0.125f : 0.0f ), "different step settings retain the observed time difference" );
        scene.step( 0.0625f, 2 );
        check( scene.getTerrainTrialComparison().contactStepSettingsMatch == comparison.contactStepSettingsMatch, "step comparison reads recorded settings rather than later steps" );
        scene.launchTerrainProbe( 1, terrainTrial::front );
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::differentConditions, "different launch conditions do not produce a contact step match result" );
    }

    for( int difference = 0; difference < 4; ++difference )
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        scene.getWorld().SetGravity( {} );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        const auto probe = scene.getShapes()[scene.getTerrainPointCount() - 1];
        const auto target = *scene.getTerrainTrialTarget( terrainTrial::front );
        scene.getWorld().SetBodyTransform( probe.bodyHandle, { { 0.0f, 10.0f }, {} } );
        scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
        scene.step( 0.0f, 32 );
        scene.step( 0.125f, 4 );
        check( !scene.getTerrainTrialObservation().firstContactTime && !scene.getTerrainTrialObservation().preContactSettingsChanged, "first positive step initializes settings without counting a preceding zero step" );
        check( scene.pinTerrainTrialReference(), "step history fixture pins pre-contact observation" );
        scene.step( difference & 1 ? 0.0625f : 0.125f, difference & 2 ? 8 : 4 );
        scene.step( 0.125f, 4 );
        check( scene.getTerrainTrialObservation().preContactSettingsChanged == ( difference != 0 ), "pre-contact changes remain recorded after restoring initial settings" );
        check( !scene.getTerrainTrialReference()->preContactSettingsChanged, "pinned step history does not acquire later changes" );
        scene.step( 0.0f, 32 );
        scene.getWorld().SetBodyTransform( probe.bodyHandle, { target.point + 0.4f * Cross( target.tangent, 1.0f ), {} } );
        scene.step( 0.125f, 4 );
        check( scene.getTerrainTrialObservation().firstContactTime && scene.getTerrainTrialObservation().preContactSettingsChanged == ( difference != 0 ), "contact freezes the recorded pre-contact change flag" );
        check( scene.pinTerrainTrialReference(), "completed step history can be pinned" );
        scene.step( 0.0625f, 2 );
        check( scene.getTerrainTrialObservation().preContactSettingsChanged == ( difference != 0 ) && scene.getTerrainTrialReference()->preContactSettingsChanged == ( difference != 0 ), "post-contact setting changes do not modify current or pinned history" );
        check( scene.relaunchTerrainTrialReference() && !scene.getTerrainTrialObservation().preContactSettingsChanged && scene.getTerrainTrialObservation().initialSubStepCount == 0, "relaunch starts new step history while retaining reference" );
        scene.resetTerrainProbes();
        check( !scene.getTerrainTrialObservation().preContactSettingsChanged && scene.getTerrainTrialObservation().initialSubStepCount == 0 && scene.getTerrainTrialReference()->preContactSettingsChanged == ( difference != 0 ), "probe reset clears current history and retains reference" );
        check( scene.insertSplineControlPoint( 1 ) && !scene.getTerrainTrialReference(), "terrain edits invalidate pinned step history" );
    }

    const struct
    {
        bool referenceChanged;
        bool currentChanged;
        float timeStep;
        terrainTrialStepComparison expected;
        float difference;
    } stepSummaries[] = {
        { false, false, 0.125f, terrainTrialStepComparison::constantMatch, 0.0f },
        { false, false, 0.25f, terrainTrialStepComparison::constantDifferent, 0.125f },
        { false, true, 0.125f, terrainTrialStepComparison::changed, 0.0625f },
        { false, true, 0.25f, terrainTrialStepComparison::changed, 0.1875f },
        { true, false, 0.125f, terrainTrialStepComparison::changed, -0.0625f },
        { true, false, 0.25f, terrainTrialStepComparison::changed, 0.0625f },
        { true, true, 0.125f, terrainTrialStepComparison::changed, 0.0f },
        { true, true, 0.25f, terrainTrialStepComparison::changed, 0.125f }
    };
    for( const auto& summary : stepSummaries )
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        scene.getWorld().SetGravity( {} );
        check( scene.getTerrainTrialComparison().stepSettings == terrainTrialStepComparison::unavailable, "step summary is unavailable without a reference" );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        const auto probe = scene.getShapes()[scene.getTerrainPointCount() - 1];
        const auto target = *scene.getTerrainTrialTarget( terrainTrial::front );
        const auto observeContact = [&]( bool changed, float timeStep )
        {
            if( changed )
            {
                scene.getWorld().SetBodyTransform( probe.bodyHandle, { { 0.0f, 10.0f }, {} } );
                scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
                scene.step( 0.0625f, 2 );
            }
            scene.getWorld().SetBodyTransform( probe.bodyHandle, { target.point + 0.4f * Cross( target.tangent, 1.0f ), {} } );
            scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
            scene.step( timeStep, 4 );
        };
        observeContact( summary.referenceChanged, 0.125f );
        check( scene.pinTerrainTrialReference() && scene.relaunchTerrainTrialReference(), "step summary fixture pins and relaunches" );
        check( scene.getTerrainTrialComparison().stepSettings == terrainTrialStepComparison::unavailable, "step summary waits for both contact observations" );
        observeContact( summary.currentChanged, summary.timeStep );
        const auto comparison = scene.getTerrainTrialComparison();
        check( comparison.status == terrainTrialComparisonStatus::ready && comparison.stepSettings == summary.expected, "step summary distinguishes constant settings and prioritizes either changed history" );
        check( comparison.firstContactTimeDifference == summary.difference, "step summary retains the signed observed time difference" );
        scene.launchTerrainProbe( 1, terrainTrial::front );
        check( scene.getTerrainTrialComparison().stepSettings == terrainTrialStepComparison::unavailable, "different launch conditions do not produce a step summary" );
    }

    for( vec2 gravity : { vec2{ 1.0f, -3.0f }, vec2{ 0.0f, -4.0f } } )
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        scene.getWorld().SetGravity( { 0.0f, -3.0f } );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        check( near( scene.getTerrainTrialObservation().launchGravity, { 0.0f, -3.0f } ), "launch records the applied world gravity" );
        scene.step( 0.0625f, 4 );
        check( scene.pinTerrainTrialReference(), "gravity fixture pins a progressed launch" );
        scene.getWorld().SetGravity( gravity );
        scene.step( 0.0625f, 4 );
        check( near( scene.getTerrainTrialObservation().launchGravity, { 0.0f, -3.0f } ) && near( scene.getTerrainTrialReference()->launchGravity, { 0.0f, -3.0f } ), "later gravity changes do not overwrite current or pinned launch conditions" );
        check( scene.getTerrainTrialComparison().status != terrainTrialComparisonStatus::differentConditions, "comparison uses recorded gravity instead of current world settings" );
        scene.launchTerrainProbe( 99, terrainTrial::front );
        check( near( scene.getTerrainTrialObservation().launchGravity, { 0.0f, -3.0f } ), "failed launch retains recorded gravity" );
        check( scene.relaunchTerrainTrialReference() && near( scene.getWorld().GetGravity(), gravity ) && near( scene.getTerrainTrialObservation().launchGravity, gravity ), "reference relaunch keeps current gravity and records it for the new trial" );
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::differentConditions, "each gravity component participates in recorded launch condition comparison" );
        scene.getWorld().SetGravity( { 0.0f, -3.0f } );
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::differentConditions, "restoring world gravity does not retroactively change launch conditions" );
        check( scene.relaunchTerrainTrialReference() && scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::waitingForContact, "matching recorded gravity allows comparison to wait for contact" );
        scene.resetTerrainProbes();
        check( !scene.getTerrainTrialObservation().active && near( scene.getTerrainTrialObservation().launchGravity, {} ) && near( scene.getTerrainTrialReference()->launchGravity, { 0.0f, -3.0f } ), "probe reset clears current gravity metadata and retains the reference" );
        scene.launchTerrainProbe( 1, terrainTrial::seams );
        check( near( scene.getTerrainTrialObservation().launchGravity, { 0.0f, -3.0f } ), "seam launches record gravity too" );
        check( scene.insertSplineControlPoint( 1 ) && !scene.getTerrainTrialObservation().active && !scene.getTerrainTrialReference(), "terrain edits clear current and pinned gravity conditions" );
    }

    for( vec2 gravity : { vec2{}, vec2{ 1.0f, 0.0f }, vec2{ 0.0f, -3.0f } } )
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        scene.getWorld().SetGravity( {} );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        const auto probe = scene.getShapes()[scene.getTerrainPointCount() - 1];
        const auto target = *scene.getTerrainTrialTarget( terrainTrial::front );
        scene.getWorld().SetBodyTransform( probe.bodyHandle, { { 0.0f, 10.0f }, {} } );
        scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
        scene.getWorld().SetGravity( { 2.0f, -4.0f } );
        scene.step( 0.0f, 4 );
        scene.getWorld().SetGravity( {} );
        scene.step( 0.0625f, 4 );
        check( !scene.getTerrainTrialObservation().preContactGravityChanged && scene.pinTerrainTrialReference(), "zero-duration gravity changes do not count" );
        scene.getWorld().SetGravity( gravity );
        scene.step( 0.0625f, 4 );
        scene.getWorld().SetGravity( {} );
        scene.step( 0.0625f, 4 );
        const bool changed = LengthSquared( gravity ) > 0.0f;
        check( scene.getTerrainTrialObservation().preContactGravityChanged == changed && !scene.getTerrainTrialReference()->preContactGravityChanged, "observed gravity changes stay latched after restoration without changing reference" );
        scene.getWorld().SetBodyTransform( probe.bodyHandle, { target.point + 0.4f * Cross( target.tangent, 1.0f ), {} } );
        scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
        scene.step( 0.125f, 4 );
        check( scene.getTerrainTrialObservation().firstContactTime && scene.pinTerrainTrialReference(), "gravity history fixture pins first contact" );
        scene.getWorld().SetGravity( { 2.0f, -4.0f } );
        scene.step( 0.0625f, 4 );
        check( scene.getTerrainTrialObservation().preContactGravityChanged == changed && scene.getTerrainTrialReference()->preContactGravityChanged == changed && scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::ready, "post-contact gravity changes preserve history and existing comparison policy" );
        check( scene.relaunchTerrainTrialReference() && !scene.getTerrainTrialObservation().preContactGravityChanged, "relaunch clears current gravity history" );
        scene.resetTerrainProbes();
        check( !scene.getTerrainTrialObservation().preContactGravityChanged && scene.getTerrainTrialReference()->preContactGravityChanged == changed, "probe reset retains pinned gravity history" );
        check( scene.insertSplineControlPoint( 1 ) && !scene.getTerrainTrialReference(), "terrain edits invalidate pinned gravity history" );
    }

    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        scene.getWorld().SetGravity( {} );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        const auto target = *scene.getTerrainTrialTarget( terrainTrial::front );
        scene.getWorld().SetBodyTransform( scene.getImpulseBody(), { target.point + 0.4f * Cross( target.tangent, 1.0f ), {} } );
        scene.getWorld().SetBodyLinearVelocity( scene.getImpulseBody(), {} );
        scene.getWorld().SetGravity( { 1.0f, 0.0f } );
        scene.step( 0.125f, 4 );
        check( scene.getTerrainTrialObservation().firstContactTime && scene.getTerrainTrialObservation().preContactGravityChanged, "gravity change on the first contact step is included" );
    }

    for( int changes = 0; changes < 4; ++changes )
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        scene.getWorld().SetGravity( {} );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        const auto target = *scene.getTerrainTrialTarget( terrainTrial::front );
        const auto observeContact = [&]( bool changed )
        {
            if( changed )
            {
                scene.getWorld().SetBodyTransform( scene.getImpulseBody(), { { 0.0f, 10.0f }, {} } );
                scene.getWorld().SetBodyLinearVelocity( scene.getImpulseBody(), {} );
                scene.getWorld().SetGravity( { 1.0f, 0.0f } );
                scene.step( 0.125f, 4 );
                scene.getWorld().SetGravity( {} );
            }
            scene.getWorld().SetBodyTransform( scene.getImpulseBody(), { target.point + 0.4f * Cross( target.tangent, 1.0f ), {} } );
            scene.getWorld().SetBodyLinearVelocity( scene.getImpulseBody(), {} );
            scene.step( 0.125f, 4 );
        };
        observeContact( changes & 1 );
        check( scene.pinTerrainTrialReference() && scene.relaunchTerrainTrialReference(), "gravity summary fixture pins and relaunches" );
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::waitingForContact && !scene.getTerrainTrialComparison().gravityChangesObserved, "gravity summary waits for comparable contact observations" );
        observeContact( changes & 2 );
        const auto comparison = scene.getTerrainTrialComparison();
        check( comparison.status == terrainTrialComparisonStatus::ready && comparison.gravityChangesObserved == ( changes != 0 ), "gravity summary includes current, reference or both observed changes" );
        const float differences[] = { 0.0f, -0.125f, 0.125f, 0.0f };
        check( comparison.firstContactTimeDifference == differences[changes] && comparison.stepSettings == terrainTrialStepComparison::constantMatch, "gravity summary preserves time difference and independent step summary" );
        scene.launchTerrainProbe( 1, terrainTrial::front );
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::differentConditions && !scene.getTerrainTrialComparison().gravityChangesObserved, "mismatched launches do not produce a gravity summary" );
        scene.resetTerrainProbes();
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::noObservation && !scene.getTerrainTrialComparison().gravityChangesObserved, "missing current observation does not produce a gravity summary" );
        scene.clearTerrainTrialReference();
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::noReference && !scene.getTerrainTrialComparison().gravityChangesObserved, "missing reference does not produce a gravity summary" );
    }

    for( bool frictionChanged : { false, true } )
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        const auto probe = scene.getShapes()[scene.getTerrainPointCount() - 1];
        scene.getWorld().SetShapeFriction( probe.shapeHandle, 0.25f );
        scene.getWorld().SetShapeRestitution( probe.shapeHandle, 0.5f );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        check( scene.getTerrainTrialObservation().launchFriction == 0.25f && scene.getTerrainTrialObservation().launchRestitution == 0.5f, "launch records the selected probe friction and restitution" );
        scene.step( 0.0625f, 4 );
        check( scene.pinTerrainTrialReference(), "material fixture pins a progressed launch" );
        const float friction = frictionChanged ? 0.75f : 0.25f;
        const float restitution = frictionChanged ? 0.5f : 0.8f;
        scene.getWorld().SetShapeFriction( probe.shapeHandle, friction );
        scene.getWorld().SetShapeRestitution( probe.shapeHandle, restitution );
        check( scene.getTerrainTrialObservation().launchFriction == 0.25f && scene.getTerrainTrialObservation().launchRestitution == 0.5f && scene.getTerrainTrialReference()->launchFriction == 0.25f && scene.getTerrainTrialReference()->launchRestitution == 0.5f && scene.getTerrainTrialComparison().status != terrainTrialComparisonStatus::differentConditions, "live material edits do not rewrite recorded current or reference conditions" );
        scene.launchTerrainProbe( 99, terrainTrial::front );
        check( scene.getTerrainTrialObservation().launchFriction == 0.25f && scene.getTerrainTrialObservation().launchRestitution == 0.5f, "failed launch preserves recorded material" );
        check( scene.relaunchTerrainTrialReference() && scene.getTerrainTrialObservation().launchFriction == friction && scene.getTerrainTrialObservation().launchRestitution == restitution && scene.getWorld().GetShapeFriction( probe.shapeHandle ) == friction && scene.getWorld().GetShapeRestitution( probe.shapeHandle ) == restitution, "reference relaunch uses current material and records it" );
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::differentConditions, "each recorded material property participates in condition comparison" );
        scene.getWorld().SetShapeFriction( probe.shapeHandle, 0.25f );
        scene.getWorld().SetShapeRestitution( probe.shapeHandle, 0.5f );
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::differentConditions, "restoring live material does not retroactively change launch records" );
        check( scene.relaunchTerrainTrialReference() && scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::waitingForContact, "equal recorded material allows contact comparison" );
        scene.resetTerrainProbes();
        check( scene.getTerrainTrialObservation().launchFriction == 0.0f && scene.getTerrainTrialObservation().launchRestitution == 0.0f && scene.getTerrainTrialReference()->launchFriction == 0.25f, "probe reset clears current material metadata and keeps reference" );
        scene.launchTerrainProbe( 0, terrainTrial::seams );
        check( scene.getTerrainTrialObservation().launchFriction == 0.25f && scene.getTerrainTrialObservation().launchRestitution == 0.5f, "seam launch records probe material" );
        scene.getWorld().DestroyShape( probe.shapeHandle );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        check( scene.getTerrainTrialObservation().trial == terrainTrial::seams && scene.getTerrainTrialObservation().launchSpeed == 6.0f, "retired probe shape rejects launch before reading material or changing state" );
    }

    for( int change = 0; change < 3; ++change )
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        scene.getWorld().SetGravity( {} );
        const auto probe = scene.getShapes()[scene.getTerrainPointCount() - 1];
        scene.getWorld().SetShapeFriction( probe.shapeHandle, 0.25f );
        scene.getWorld().SetShapeRestitution( probe.shapeHandle, 0.5f );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        const auto target = *scene.getTerrainTrialTarget( terrainTrial::front );
        scene.getWorld().SetBodyTransform( probe.bodyHandle, { { 0.0f, 10.0f }, {} } );
        scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
        scene.getWorld().SetShapeFriction( probe.shapeHandle, 0.75f );
        scene.step( 0.0f, 4 );
        scene.getWorld().SetShapeFriction( probe.shapeHandle, 0.25f );
        scene.getWorld().SetShapeFriction( scene.getShapes()[0].shapeHandle, 0.9f );
        scene.getWorld().SetShapeRestitution( scene.getShapes()[scene.getTerrainPointCount()].shapeHandle, 0.9f );
        scene.step( 0.0625f, 4 );
        check( !scene.getTerrainTrialObservation().preContactMaterialChanged && scene.pinTerrainTrialReference(), "zero-duration and other-shape material changes do not count" );
        scene.getWorld().SetShapeFriction( probe.shapeHandle, change == 1 ? 0.75f : 0.25f );
        scene.getWorld().SetShapeRestitution( probe.shapeHandle, change == 2 ? 0.8f : 0.5f );
        scene.step( 0.0625f, 4 );
        scene.getWorld().SetShapeFriction( probe.shapeHandle, 0.25f );
        scene.getWorld().SetShapeRestitution( probe.shapeHandle, 0.5f );
        scene.step( 0.0625f, 4 );
        check( scene.getTerrainTrialObservation().preContactMaterialChanged == ( change != 0 ) && !scene.getTerrainTrialReference()->preContactMaterialChanged, "observed material changes stay latched after restoration without modifying reference" );
        scene.getWorld().SetBodyTransform( probe.bodyHandle, { target.point + 0.4f * Cross( target.tangent, 1.0f ), {} } );
        scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
        scene.step( 0.125f, 4 );
        check( scene.getTerrainTrialObservation().firstContactTime && scene.pinTerrainTrialReference(), "material history fixture pins first contact" );
        scene.getWorld().SetShapeFriction( probe.shapeHandle, 0.75f );
        scene.getWorld().SetShapeRestitution( probe.shapeHandle, 0.8f );
        scene.step( 0.0625f, 4 );
        check( scene.getTerrainTrialObservation().preContactMaterialChanged == ( change != 0 ) && scene.getTerrainTrialReference()->preContactMaterialChanged == ( change != 0 ) && scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::ready, "post-contact material changes preserve history and comparison policy" );
        check( scene.relaunchTerrainTrialReference() && !scene.getTerrainTrialObservation().preContactMaterialChanged, "relaunch starts fresh material history" );
        scene.resetTerrainProbes();
        check( !scene.getTerrainTrialObservation().preContactMaterialChanged && scene.getTerrainTrialReference()->preContactMaterialChanged == ( change != 0 ), "probe reset retains pinned material history" );
        check( scene.insertSplineControlPoint( 1 ) && !scene.getTerrainTrialReference(), "terrain edits invalidate pinned material history" );
    }

    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        scene.getWorld().SetGravity( {} );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        const auto probe = scene.getShapes()[scene.getTerrainPointCount() - 1];
        const auto target = *scene.getTerrainTrialTarget( terrainTrial::front );
        scene.getWorld().SetBodyTransform( probe.bodyHandle, { target.point + 0.4f * Cross( target.tangent, 1.0f ), {} } );
        scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
        scene.getWorld().SetShapeFriction( probe.shapeHandle, scene.getTerrainTrialObservation().launchFriction + 0.5f );
        scene.step( 0.125f, 4 );
        check( scene.getTerrainTrialObservation().firstContactTime && scene.getTerrainTrialObservation().preContactMaterialChanged, "material change on the first contact step is included" );
    }

    for( int changes = 0; changes < 4; ++changes )
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        scene.getWorld().SetGravity( {} );
        const auto probe = scene.getShapes()[scene.getTerrainPointCount() - 1];
        scene.getWorld().SetShapeFriction( probe.shapeHandle, 0.25f );
        scene.getWorld().SetShapeRestitution( probe.shapeHandle, 0.5f );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        const auto target = *scene.getTerrainTrialTarget( terrainTrial::front );
        const auto observeContact = [&]( bool changed )
        {
            if( changed )
            {
                scene.getWorld().SetBodyTransform( probe.bodyHandle, { { 0.0f, 10.0f }, {} } );
                scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
                scene.getWorld().SetShapeFriction( probe.shapeHandle, 0.75f );
                scene.step( 0.125f, 4 );
                scene.getWorld().SetShapeFriction( probe.shapeHandle, 0.25f );
            }
            scene.getWorld().SetBodyTransform( probe.bodyHandle, { target.point + 0.4f * Cross( target.tangent, 1.0f ), {} } );
            scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
            scene.step( 0.125f, 4 );
        };
        observeContact( changes & 1 );
        check( scene.pinTerrainTrialReference() && scene.relaunchTerrainTrialReference(), "material summary fixture pins and relaunches" );
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::waitingForContact && !scene.getTerrainTrialComparison().materialChangesObserved, "material summary waits for comparable contact observations" );
        observeContact( changes & 2 );
        const auto comparison = scene.getTerrainTrialComparison();
        check( comparison.status == terrainTrialComparisonStatus::ready && comparison.materialChangesObserved == ( changes != 0 ), "material summary includes current, reference or both observed changes" );
        const float differences[] = { 0.0f, -0.125f, 0.125f, 0.0f };
        check( comparison.firstContactTimeDifference == differences[changes] && comparison.stepSettings == terrainTrialStepComparison::constantMatch && !comparison.gravityChangesObserved, "material summary preserves time difference and independent step/gravity summaries" );
        scene.launchTerrainProbe( 1, terrainTrial::front );
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::differentConditions && !scene.getTerrainTrialComparison().materialChangesObserved, "mismatched launches do not produce a material summary" );
        scene.resetTerrainProbes();
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::noObservation && !scene.getTerrainTrialComparison().materialChangesObserved, "missing current observation does not produce a material summary" );
        scene.clearTerrainTrialReference();
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::noReference && !scene.getTerrainTrialComparison().materialChangesObserved, "missing reference does not produce a material summary" );
    }

    for( std::size_t probeIndex = 0; probeIndex < 3; ++probeIndex )
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        const auto probe = scene.getShapes()[scene.getTerrainPointCount() - 1 + probeIndex];
        scene.getWorld().SetShapeDensity( probe.shapeHandle, 1.0f );
        const float mass = scene.getWorld().GetBodyMass( probe.bodyHandle );
        scene.launchTerrainProbe( probeIndex, terrainTrial::front );
        check( mass > 0.0f && scene.getTerrainTrialObservation().launchMass == mass, "launch records the selected probe body mass" );
        scene.step( 0.0625f, 4 );
        check( scene.pinTerrainTrialReference(), "mass fixture pins a progressed trial" );
        scene.getWorld().SetShapeDensity( probe.shapeHandle, 2.0f );
        const float changedMass = scene.getWorld().GetBodyMass( probe.bodyHandle );
        check( changedMass > mass, "density fixture must change actual body mass" );
        check( scene.getTerrainTrialObservation().launchMass == mass && scene.getTerrainTrialReference()->launchMass == mass && scene.getTerrainTrialComparison().status != terrainTrialComparisonStatus::differentConditions, "live density changes do not rewrite recorded current or reference mass" );
        scene.launchTerrainProbe( 99, terrainTrial::front );
        check( scene.getTerrainTrialObservation().launchMass == mass, "failed launch preserves recorded mass" );
        check( scene.relaunchTerrainTrialReference() && scene.getTerrainTrialObservation().launchMass == changedMass && scene.getWorld().GetBodyMass( probe.bodyHandle ) == changedMass && scene.getWorld().GetShapeDensity( probe.shapeHandle ) == 2.0f, "reference relaunch keeps current mass and density and records new mass" );
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::differentConditions, "recorded mass participates in launch condition comparison" );
        scene.getWorld().SetShapeDensity( probe.shapeHandle, 1.0f );
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::differentConditions, "restoring live density does not retroactively change recorded mass" );
        check( scene.relaunchTerrainTrialReference() && scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::waitingForContact, "equal recorded mass allows contact comparison" );
        scene.resetTerrainProbes();
        check( scene.getTerrainTrialObservation().launchMass == 0.0f && scene.getTerrainTrialReference()->launchMass == mass, "probe reset clears current mass and keeps reference" );
        scene.launchTerrainProbe( probeIndex, terrainTrial::seams );
        check( scene.getTerrainTrialObservation().launchMass == mass, "seam trial records selected probe mass" );
        check( scene.insertSplineControlPoint( 1 ) && !scene.getTerrainTrialObservation().active && !scene.getTerrainTrialReference(), "terrain edits clear current and pinned mass records" );
    }

    for( bool changed : { false, true } )
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        scene.getWorld().SetGravity( {} );
        const auto probe = scene.getShapes()[scene.getTerrainPointCount() - 1];
        scene.getWorld().SetShapeDensity( probe.shapeHandle, 1.0f );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        const auto target = *scene.getTerrainTrialTarget( terrainTrial::front );
        scene.getWorld().SetBodyTransform( probe.bodyHandle, { { 0.0f, 10.0f }, {} } );
        scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
        scene.getWorld().SetShapeDensity( probe.shapeHandle, 2.0f );
        scene.step( 0.0f, 4 );
        scene.getWorld().SetShapeDensity( probe.shapeHandle, 1.0f );
        scene.getWorld().SetShapeDensity( scene.getShapes()[scene.getTerrainPointCount()].shapeHandle, 3.0f );
        scene.step( 0.0625f, 4 );
        check( !scene.getTerrainTrialObservation().preContactMassChanged && scene.pinTerrainTrialReference(), "zero-duration and other-probe mass changes do not count" );
        scene.getWorld().SetShapeDensity( probe.shapeHandle, changed ? 2.0f : 1.0f );
        scene.step( 0.0625f, 4 );
        scene.getWorld().SetShapeDensity( probe.shapeHandle, 1.0f );
        scene.step( 0.0625f, 4 );
        check( scene.getTerrainTrialObservation().preContactMassChanged == changed && !scene.getTerrainTrialReference()->preContactMassChanged, "observed mass changes stay latched after restoration without modifying reference" );
        scene.getWorld().SetBodyTransform( probe.bodyHandle, { target.point + 0.4f * Cross( target.tangent, 1.0f ), {} } );
        scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
        scene.step( 0.125f, 4 );
        check( scene.getTerrainTrialObservation().firstContactTime && scene.pinTerrainTrialReference(), "mass history fixture pins first contact" );
        scene.getWorld().SetShapeDensity( probe.shapeHandle, 2.0f );
        scene.step( 0.0625f, 4 );
        check( scene.getTerrainTrialObservation().preContactMassChanged == changed && scene.getTerrainTrialReference()->preContactMassChanged == changed && scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::ready, "post-contact mass changes preserve history and comparison policy" );
        check( scene.relaunchTerrainTrialReference() && !scene.getTerrainTrialObservation().preContactMassChanged, "relaunch starts fresh mass history" );
        scene.resetTerrainProbes();
        check( !scene.getTerrainTrialObservation().preContactMassChanged && scene.getTerrainTrialReference()->preContactMassChanged == changed, "probe reset retains pinned mass history" );
        check( scene.insertSplineControlPoint( 1 ) && !scene.getTerrainTrialReference(), "terrain edits invalidate pinned mass history" );
    }

    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        scene.getWorld().SetGravity( {} );
        const auto probe = scene.getShapes()[scene.getTerrainPointCount() - 1];
        scene.getWorld().SetShapeDensity( probe.shapeHandle, 1.0f );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        const auto target = *scene.getTerrainTrialTarget( terrainTrial::front );
        scene.getWorld().SetBodyTransform( probe.bodyHandle, { target.point + 0.4f * Cross( target.tangent, 1.0f ), {} } );
        scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
        scene.getWorld().SetShapeDensity( probe.shapeHandle, 2.0f );
        scene.step( 0.125f, 4 );
        check( scene.getTerrainTrialObservation().firstContactTime && scene.getTerrainTrialObservation().preContactMassChanged, "mass change on the first contact step is included" );
    }

    for( int changes = 0; changes < 4; ++changes )
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        scene.getWorld().SetGravity( {} );
        const auto probe = scene.getShapes()[scene.getTerrainPointCount() - 1];
        scene.getWorld().SetShapeDensity( probe.shapeHandle, 1.0f );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        const auto target = *scene.getTerrainTrialTarget( terrainTrial::front );
        const auto observeContact = [&]( bool changed )
        {
            if( changed )
            {
                scene.getWorld().SetBodyTransform( probe.bodyHandle, { { 0.0f, 10.0f }, {} } );
                scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
                scene.getWorld().SetShapeDensity( probe.shapeHandle, 2.0f );
                scene.step( 0.125f, 4 );
                scene.getWorld().SetShapeDensity( probe.shapeHandle, 1.0f );
            }
            scene.getWorld().SetBodyTransform( probe.bodyHandle, { target.point + 0.4f * Cross( target.tangent, 1.0f ), {} } );
            scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
            scene.step( 0.125f, 4 );
        };
        observeContact( changes & 1 );
        check( scene.pinTerrainTrialReference() && scene.relaunchTerrainTrialReference(), "mass summary fixture pins and relaunches" );
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::waitingForContact && !scene.getTerrainTrialComparison().massChangesObserved, "mass summary waits for comparable contact observations" );
        observeContact( changes & 2 );
        const auto comparison = scene.getTerrainTrialComparison();
        check( comparison.status == terrainTrialComparisonStatus::ready && comparison.massChangesObserved == ( changes != 0 ), "mass summary includes current, reference or both observed changes" );
        const float differences[] = { 0.0f, -0.125f, 0.125f, 0.0f };
        check( comparison.firstContactTimeDifference == differences[changes] && comparison.stepSettings == terrainTrialStepComparison::constantMatch && !comparison.gravityChangesObserved && !comparison.materialChangesObserved, "mass summary preserves time difference and independent step/gravity/material summaries" );
        scene.launchTerrainProbe( 1, terrainTrial::front );
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::differentConditions && !scene.getTerrainTrialComparison().massChangesObserved, "mismatched launches do not produce a mass summary" );
        scene.resetTerrainProbes();
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::noObservation && !scene.getTerrainTrialComparison().massChangesObserved, "missing current observation does not produce a mass summary" );
        scene.clearTerrainTrialReference();
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::noReference && !scene.getTerrainTrialComparison().massChangesObserved, "missing reference does not produce a mass summary" );
    }

    for( std::size_t probeIndex = 0; probeIndex < 3; ++probeIndex )
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        const auto probe = scene.getShapes()[scene.getTerrainPointCount() - 1 + probeIndex];
        scene.getWorld().SetShapeDensity( probe.shapeHandle, 1.0f );
        const float inertia = scene.getWorld().GetBodyRotationalInertia( probe.bodyHandle );
        scene.launchTerrainProbe( probeIndex, terrainTrial::front );
        check( inertia > 0.0f && scene.getTerrainTrialObservation().launchInertia == inertia, "launch records selected probe body rotational inertia" );
        scene.step( 0.0625f, 4 );
        check( scene.pinTerrainTrialReference(), "inertia fixture pins a progressed trial" );
        scene.getWorld().SetShapeDensity( probe.shapeHandle, 2.0f );
        const float changedInertia = scene.getWorld().GetBodyRotationalInertia( probe.bodyHandle );
        check( changedInertia > inertia && scene.getTerrainTrialObservation().launchInertia == inertia && scene.getTerrainTrialReference()->launchInertia == inertia, "live inertia changes do not rewrite current or reference records" );
        scene.launchTerrainProbe( 99, terrainTrial::front );
        check( scene.getTerrainTrialObservation().launchInertia == inertia, "failed launch retains recorded inertia" );
        check( scene.relaunchTerrainTrialReference() && scene.getTerrainTrialObservation().launchInertia == changedInertia && scene.getWorld().GetBodyRotationalInertia( probe.bodyHandle ) == changedInertia && scene.getWorld().GetShapeDensity( probe.shapeHandle ) == 2.0f, "reference relaunch keeps current physical settings and records new inertia" );
        scene.resetTerrainProbes();
        check( scene.getTerrainTrialObservation().launchInertia == 0.0f && scene.getTerrainTrialReference()->launchInertia == inertia, "probe reset clears current inertia and retains reference" );
        scene.launchTerrainProbe( probeIndex, terrainTrial::seams );
        check( scene.getTerrainTrialObservation().launchInertia == changedInertia, "seam launch records selected probe inertia" );
        check( scene.insertSplineControlPoint( 1 ) && !scene.getTerrainTrialObservation().active && !scene.getTerrainTrialReference(), "terrain edits clear current and pinned inertia records" );
    }
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        const auto probe = scene.getShapes()[scene.getTerrainPointCount() - 1];
        scene.getWorld().SetShapeDensity( probe.shapeHandle, 1.0f );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        scene.step( 0.0625f, 4 );
        check( scene.pinTerrainTrialReference(), "inertia-only fixture pins a trial" );
        const float mass = scene.getTerrainTrialReference()->launchMass;
        const float inertia = scene.getTerrainTrialReference()->launchInertia;
        scene.getWorld().SetShapeDensity( probe.shapeHandle, 0.5f );
        const auto extra = scene.getWorld().CreateShape( probe.bodyHandle, circle2{ { 2.0f, 0.0f }, 0.45f }, {}, 0.5f );
        check( scene.getWorld().GetBodyMass( probe.bodyHandle ) == mass && scene.getWorld().GetBodyRotationalInertia( probe.bodyHandle ) != inertia, "redistributed mass changes inertia without changing total mass" );
        check( scene.relaunchTerrainTrialReference() && scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::differentConditions, "recorded inertia mismatch is independent of recorded mass" );
        scene.getWorld().DestroyShape( extra );
        scene.getWorld().SetShapeDensity( probe.shapeHandle, 1.0f );
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::differentConditions, "restoring live inertia cannot rewrite recorded conditions" );
        check( scene.relaunchTerrainTrialReference() && scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::waitingForContact, "equal recorded inertia allows contact comparison" );
    }

    for( bool changed : { false, true } )
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        scene.getWorld().SetGravity( {} );
        const auto probe = scene.getShapes()[scene.getTerrainPointCount() - 1];
        scene.getWorld().SetShapeDensity( probe.shapeHandle, 1.0f );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        const auto target = *scene.getTerrainTrialTarget( terrainTrial::front );
        scene.getWorld().SetBodyTransform( probe.bodyHandle, { { 0.0f, 10.0f }, {} } );
        scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
        scene.getWorld().SetShapeDensity( probe.shapeHandle, 2.0f );
        scene.step( 0.0f, 4 );
        scene.getWorld().SetShapeDensity( probe.shapeHandle, 1.0f );
        scene.getWorld().SetShapeDensity( scene.getShapes()[scene.getTerrainPointCount()].shapeHandle, 3.0f );
        scene.step( 0.0625f, 4 );
        check( !scene.getTerrainTrialObservation().preContactInertiaChanged && scene.pinTerrainTrialReference(), "zero-duration and other-probe inertia changes do not count" );
        scene.getWorld().SetShapeDensity( probe.shapeHandle, changed ? 2.0f : 1.0f );
        scene.step( 0.0625f, 4 );
        scene.getWorld().SetShapeDensity( probe.shapeHandle, 1.0f );
        scene.step( 0.0625f, 4 );
        check( scene.getTerrainTrialObservation().preContactInertiaChanged == changed && !scene.getTerrainTrialReference()->preContactInertiaChanged, "observed inertia changes stay latched after restoration without modifying reference" );
        scene.getWorld().SetBodyTransform( probe.bodyHandle, { target.point + 0.4f * Cross( target.tangent, 1.0f ), {} } );
        scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
        scene.step( 0.125f, 4 );
        check( scene.getTerrainTrialObservation().firstContactTime && scene.pinTerrainTrialReference(), "inertia history fixture pins first contact" );
        scene.getWorld().SetShapeDensity( probe.shapeHandle, 2.0f );
        scene.step( 0.0625f, 4 );
        check( scene.getTerrainTrialObservation().preContactInertiaChanged == changed && scene.getTerrainTrialReference()->preContactInertiaChanged == changed && scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::ready, "post-contact inertia changes preserve history and comparison policy" );
        check( scene.relaunchTerrainTrialReference() && !scene.getTerrainTrialObservation().preContactInertiaChanged, "relaunch starts fresh inertia history" );
        scene.resetTerrainProbes();
        check( !scene.getTerrainTrialObservation().preContactInertiaChanged && scene.getTerrainTrialReference()->preContactInertiaChanged == changed, "probe reset retains pinned inertia history" );
        check( scene.insertSplineControlPoint( 1 ) && !scene.getTerrainTrialReference(), "terrain edits invalidate pinned inertia history" );
    }
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        scene.getWorld().SetGravity( {} );
        const auto probe = scene.getShapes()[scene.getTerrainPointCount() - 1];
        scene.getWorld().SetShapeDensity( probe.shapeHandle, 1.0f );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        const auto target = *scene.getTerrainTrialTarget( terrainTrial::front );
        scene.getWorld().SetBodyTransform( probe.bodyHandle, { target.point + 0.4f * Cross( target.tangent, 1.0f ), {} } );
        scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
        scene.getWorld().SetShapeDensity( probe.shapeHandle, 0.5f );
        const auto extra = scene.getWorld().CreateShape( probe.bodyHandle, circle2{ { 2.0f, 0.0f }, 0.45f }, {}, 0.5f );
        check( scene.getWorld().GetBodyMass( probe.bodyHandle ) == scene.getTerrainTrialObservation().launchMass && scene.getWorld().GetBodyRotationalInertia( probe.bodyHandle ) != scene.getTerrainTrialObservation().launchInertia, "inertia history fixture changes inertia without changing mass" );
        scene.step( 0.125f, 4 );
        check( scene.getTerrainTrialObservation().firstContactTime && scene.getTerrainTrialObservation().preContactInertiaChanged && !scene.getTerrainTrialObservation().preContactMassChanged, "first-contact-step inertia change is recorded independently of mass" );
        scene.getWorld().DestroyShape( extra );
    }

    for( int changes = 0; changes < 4; ++changes )
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        scene.getWorld().SetGravity( {} );
        const auto probe = scene.getShapes()[scene.getTerrainPointCount() - 1];
        scene.getWorld().SetShapeDensity( probe.shapeHandle, 1.0f );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        const auto target = *scene.getTerrainTrialTarget( terrainTrial::front );
        const auto observeContact = [&]( bool changed )
        {
            if( changed )
            {
                scene.getWorld().SetBodyTransform( probe.bodyHandle, { { 0.0f, 10.0f }, {} } );
                scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
                scene.getWorld().SetShapeDensity( probe.shapeHandle, 0.5f );
                const auto extra = scene.getWorld().CreateShape( probe.bodyHandle, circle2{ { 2.0f, 0.0f }, 0.45f }, {}, 0.5f );
                check( scene.getWorld().GetBodyMass( probe.bodyHandle ) == scene.getTerrainTrialObservation().launchMass && scene.getWorld().GetBodyRotationalInertia( probe.bodyHandle ) != scene.getTerrainTrialObservation().launchInertia, "inertia summary fixture changes inertia without changing mass" );
                scene.step( 0.125f, 4 );
                scene.getWorld().DestroyShape( extra );
                scene.getWorld().SetShapeDensity( probe.shapeHandle, 1.0f );
            }
            scene.getWorld().SetBodyTransform( probe.bodyHandle, { target.point + 0.4f * Cross( target.tangent, 1.0f ), {} } );
            scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
            scene.step( 0.125f, 4 );
        };
        observeContact( changes & 1 );
        check( scene.pinTerrainTrialReference() && scene.relaunchTerrainTrialReference(), "inertia summary fixture pins and relaunches" );
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::waitingForContact && !scene.getTerrainTrialComparison().inertiaChangesObserved, "inertia summary waits for comparable contact observations" );
        observeContact( changes & 2 );
        const auto comparison = scene.getTerrainTrialComparison();
        check( comparison.status == terrainTrialComparisonStatus::ready && comparison.inertiaChangesObserved == ( changes != 0 ), "inertia summary includes current, reference or both observed changes" );
        const float differences[] = { 0.0f, -0.125f, 0.125f, 0.0f };
        check( comparison.firstContactTimeDifference == differences[changes] && comparison.stepSettings == terrainTrialStepComparison::constantMatch && !comparison.gravityChangesObserved && !comparison.materialChangesObserved && !comparison.massChangesObserved, "inertia summary preserves time difference and independent step/gravity/material/mass summaries" );
        scene.launchTerrainProbe( 1, terrainTrial::front );
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::differentConditions && !scene.getTerrainTrialComparison().inertiaChangesObserved, "mismatched launches do not produce an inertia summary" );
        scene.resetTerrainProbes();
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::noObservation && !scene.getTerrainTrialComparison().inertiaChangesObserved, "missing current observation does not produce an inertia summary" );
        scene.clearTerrainTrialReference();
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::noReference && !scene.getTerrainTrialComparison().inertiaChangesObserved, "missing reference does not produce an inertia summary" );
    }

    const float gravityScales[] = { 0.0f, 2.0f, -1.0f };
    for( std::size_t probeIndex = 0; probeIndex < 3; ++probeIndex )
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        const auto probe = scene.getShapes()[scene.getTerrainPointCount() - 1 + probeIndex];
        scene.getWorld().SetBodyGravityScale( probe.bodyHandle, 1.0f );
        scene.launchTerrainProbe( probeIndex, terrainTrial::front );
        check( scene.getTerrainTrialObservation().launchGravityScale == 1.0f, "launch records selected probe gravity scale" );
        scene.step( 0.0625f, 4 );
        check( scene.pinTerrainTrialReference(), "gravity scale fixture pins a progressed trial" );
        const float scale = gravityScales[probeIndex];
        scene.getWorld().SetBodyGravityScale( probe.bodyHandle, scale );
        scene.step( 0.0625f, 4 );
        check( scene.getTerrainTrialObservation().launchGravityScale == 1.0f && scene.getTerrainTrialReference()->launchGravityScale == 1.0f && scene.getTerrainTrialComparison().status != terrainTrialComparisonStatus::differentConditions, "live gravity scale changes do not rewrite recorded current or reference conditions" );
        scene.launchTerrainProbe( 99, terrainTrial::front );
        check( scene.getTerrainTrialObservation().launchGravityScale == 1.0f, "failed launch retains recorded gravity scale" );
        check( scene.relaunchTerrainTrialReference() && scene.getTerrainTrialObservation().launchGravityScale == scale && scene.getWorld().GetBodyGravityScale( probe.bodyHandle ) == scale, "reference relaunch keeps current gravity scale and records it anew" );
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::differentConditions, "recorded gravity scale participates in condition comparison" );
        scene.getWorld().SetBodyGravityScale( probe.bodyHandle, 1.0f );
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::differentConditions, "restoring live gravity scale does not rewrite launch records" );
        check( scene.relaunchTerrainTrialReference() && scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::waitingForContact, "equal recorded gravity scale allows contact comparison" );
        scene.resetTerrainProbes();
        check( scene.getTerrainTrialObservation().launchGravityScale == 0.0f && scene.getTerrainTrialReference()->launchGravityScale == 1.0f, "probe reset clears current gravity scale and retains reference" );
        scene.launchTerrainProbe( probeIndex, terrainTrial::seams );
        check( scene.getTerrainTrialObservation().launchGravityScale == 1.0f, "seam launch records selected probe gravity scale" );
        check( scene.insertSplineControlPoint( 1 ) && !scene.getTerrainTrialObservation().active && !scene.getTerrainTrialReference(), "terrain edits clear current and pinned gravity scale records" );
    }

    for( float scale : { 1.0f, 0.0f, 2.0f, -1.0f } )
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        scene.getWorld().SetGravity( {} );
        const auto probe = scene.getShapes()[scene.getTerrainPointCount() - 1];
        scene.getWorld().SetBodyGravityScale( probe.bodyHandle, 1.0f );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        const auto target = *scene.getTerrainTrialTarget( terrainTrial::front );
        scene.getWorld().SetBodyTransform( probe.bodyHandle, { { 0.0f, 10.0f }, {} } );
        scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
        scene.getWorld().SetBodyGravityScale( probe.bodyHandle, 2.0f );
        scene.step( 0.0f, 4 );
        scene.getWorld().SetBodyGravityScale( probe.bodyHandle, 1.0f );
        scene.getWorld().SetBodyGravityScale( scene.getShapes()[scene.getTerrainPointCount()].bodyHandle, 3.0f );
        scene.getWorld().SetGravity( { 1.0f, 0.0f } );
        scene.step( 0.0625f, 4 );
        check( !scene.getTerrainTrialObservation().preContactGravityScaleChanged && scene.getTerrainTrialObservation().preContactGravityChanged && scene.pinTerrainTrialReference(), "zero-duration, global gravity and other-probe changes do not count as selected scale changes" );
        scene.getWorld().SetGravity( {} );
        scene.getWorld().SetBodyGravityScale( probe.bodyHandle, scale );
        scene.step( 0.0625f, 4 );
        scene.getWorld().SetBodyGravityScale( probe.bodyHandle, 1.0f );
        scene.step( 0.0625f, 4 );
        const bool changed = scale != 1.0f;
        check( scene.getTerrainTrialObservation().preContactGravityScaleChanged == changed && !scene.getTerrainTrialReference()->preContactGravityScaleChanged, "observed scale changes stay latched after restoration without modifying reference" );
        scene.getWorld().SetBodyTransform( probe.bodyHandle, { target.point + 0.4f * Cross( target.tangent, 1.0f ), {} } );
        scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
        scene.step( 0.125f, 4 );
        check( scene.getTerrainTrialObservation().firstContactTime && scene.pinTerrainTrialReference(), "gravity scale history fixture pins first contact" );
        scene.getWorld().SetBodyGravityScale( probe.bodyHandle, 2.0f );
        scene.step( 0.0625f, 4 );
        check( scene.getTerrainTrialObservation().preContactGravityScaleChanged == changed && scene.getTerrainTrialReference()->preContactGravityScaleChanged == changed && scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::ready, "post-contact scale changes preserve history and comparison policy" );
        check( scene.relaunchTerrainTrialReference() && !scene.getTerrainTrialObservation().preContactGravityScaleChanged, "relaunch starts fresh gravity scale history" );
        scene.resetTerrainProbes();
        check( !scene.getTerrainTrialObservation().preContactGravityScaleChanged && scene.getTerrainTrialReference()->preContactGravityScaleChanged == changed, "probe reset retains pinned gravity scale history" );
        check( scene.insertSplineControlPoint( 1 ) && !scene.getTerrainTrialReference(), "terrain edits invalidate pinned gravity scale history" );
    }

    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        scene.getWorld().SetGravity( {} );
        scene.getWorld().SetBodyGravityScale( scene.getImpulseBody(), 1.0f );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        const auto target = *scene.getTerrainTrialTarget( terrainTrial::front );
        scene.getWorld().SetBodyTransform( scene.getImpulseBody(), { target.point + 0.4f * Cross( target.tangent, 1.0f ), {} } );
        scene.getWorld().SetBodyLinearVelocity( scene.getImpulseBody(), {} );
        scene.getWorld().SetBodyGravityScale( scene.getImpulseBody(), 2.0f );
        scene.step( 0.125f, 4 );
        check( scene.getTerrainTrialObservation().firstContactTime && scene.getTerrainTrialObservation().preContactGravityScaleChanged && !scene.getTerrainTrialObservation().preContactGravityChanged, "first-contact-step gravity scale change is recorded independently of global gravity" );
    }

    for( int changes = 0; changes < 4; ++changes )
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        scene.getWorld().SetGravity( {} );
        const auto probe = scene.getShapes()[scene.getTerrainPointCount() - 1];
        scene.getWorld().SetBodyGravityScale( probe.bodyHandle, 1.0f );
        scene.launchTerrainProbe( 0, terrainTrial::front );
        const auto target = *scene.getTerrainTrialTarget( terrainTrial::front );
        const auto observeContact = [&]( bool changed )
        {
            if( changed )
            {
                scene.getWorld().SetBodyTransform( probe.bodyHandle, { { 0.0f, 10.0f }, {} } );
                scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
                scene.getWorld().SetBodyGravityScale( probe.bodyHandle, 2.0f );
                scene.step( 0.125f, 4 );
                scene.getWorld().SetBodyGravityScale( probe.bodyHandle, 1.0f );
            }
            scene.getWorld().SetBodyTransform( probe.bodyHandle, { target.point + 0.4f * Cross( target.tangent, 1.0f ), {} } );
            scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
            scene.step( 0.125f, 4 );
        };
        observeContact( changes & 1 );
        check( scene.pinTerrainTrialReference() && scene.relaunchTerrainTrialReference(), "gravity scale summary fixture pins and relaunches" );
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::waitingForContact && !scene.getTerrainTrialComparison().gravityScaleChangesObserved, "gravity scale summary waits for comparable contact observations" );
        observeContact( changes & 2 );
        const auto comparison = scene.getTerrainTrialComparison();
        check( comparison.status == terrainTrialComparisonStatus::ready && comparison.gravityScaleChangesObserved == ( changes != 0 ), "gravity scale summary includes current, reference or both observed changes" );
        const float differences[] = { 0.0f, -0.125f, 0.125f, 0.0f };
        check( comparison.firstContactTimeDifference == differences[changes] && comparison.stepSettings == terrainTrialStepComparison::constantMatch && !comparison.gravityChangesObserved && !comparison.materialChangesObserved && !comparison.massChangesObserved && !comparison.inertiaChangesObserved, "gravity scale summary preserves time difference and independent step/gravity/material/mass/inertia summaries" );
        scene.launchTerrainProbe( 1, terrainTrial::front );
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::differentConditions && !scene.getTerrainTrialComparison().gravityScaleChangesObserved, "mismatched launches do not produce an gravity scale summary" );
        scene.resetTerrainProbes();
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::noObservation && !scene.getTerrainTrialComparison().gravityScaleChangesObserved, "missing current observation does not produce an gravity scale summary" );
        scene.clearTerrainTrialReference();
        check( scene.getTerrainTrialComparison().status == terrainTrialComparisonStatus::noReference && !scene.getTerrainTrialComparison().gravityScaleChangesObserved, "missing reference does not produce an gravity scale summary" );
    }

    for( const std::size_t segment : { 0u, 3u, 7u } )
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        const std::vector<vec2> original{ scene.getSplineControlPoints().begin(), scene.getSplineControlPoints().end() };
        const vec2 clicked = 0.5f * ( original[segment] + original[segment + 1] ) + vec2{ 0.0f, 0.15f };
        const auto edge = scene.getShapes()[0];
        const auto probe = scene.getShapes()[scene.getTerrainPointCount() - 1];
        const transform2 ground{ { 2.0f, 3.0f }, rot2::FromRadians( 0.3f ) };
        scene.getWorld().SetBodyTransform( edge.bodyHandle, ground );
        scene.getWorld().SetShapeFriction( edge.shapeHandle, 0.42f );
        scene.getWorld().SetBodyLinearVelocity( probe.bodyHandle, { 2.0f, 3.0f } );
        demoInput click{};
        click.mousePosition = TransformPoint( ground, clicked );
        click.mousePressed = click.mouseHeld = click.insertControlPoint = click.selectTrialPosition = true;
        scene.handleInput( click );
        check( scene.getSplineControlPoints().size() == original.size() + 1, "Ctrl click inserts one spline control" );
        check( near( scene.getSplineControlPoints()[segment + 1], clicked ), "inserted control uses the clicked local position instead of a curve midpoint" );
        for( std::size_t i = 0; i < original.size(); ++i )
            check( near( scene.getSplineControlPoints()[i <= segment ? i : i + 1], original[i] ), "nearest adjacent pair preserves A C B order and both endpoints" );
        check( scene.getTerrainPointDragIndex() == segment + 1 && scene.getSplineTrialFraction() == 0.5f && !scene.getWorld().IsValid( scene.getMouseJoint() ), "Ctrl insertion selects C and takes precedence over Shift and probe dragging" );
        check( scene.getShapes()[0].shapeHandle == edge.shapeHandle && scene.getWorld().GetShapeFriction( edge.shapeHandle ) == 0.42f && scene.getShapes()[scene.getTerrainPointCount() - 1].shapeHandle == probe.shapeHandle && near( scene.getWorld().GetBodyLinearVelocity( probe.bodyHandle ), { 2.0f, 3.0f } ), "click insertion preserves surviving edge properties and probe state" );
        const auto samples = SampleAdaptive( { scene.getSplineControlPoints() }, scene.getSplineSamplingError() );
        check( scene.getTerrainPointCount() == samples.size(), "click insertion updates adaptive chain geometry" );
        for( std::size_t i = 0; i < samples.size(); ++i ) check( near( scene.getTerrainPoint( i ), samples[i] ), "click insertion retains the existing sampling path" );
        click.mousePressed = click.insertControlPoint = click.selectTrialPosition = false;
        click.mousePosition = scene.getWorld().GetBodyTransform( probe.bodyHandle ).position;
        scene.handleInput( click );
        check( scene.getSplineControlPoints().size() == original.size() + 1 && near( scene.getSplineControlPoints()[segment + 1], clicked ) && !scene.getWorld().IsValid( scene.getMouseJoint() ), "holding the click after releasing modifiers neither inserts again nor drags" );
        click.mouseHeld = false;
        scene.handleInput( click );
        check( !scene.getTerrainPointDragIndex() && scene.undoSplineEdit(), "one Ctrl click creates one undoable edit" );
        for( std::size_t i = 0; i < original.size(); ++i ) check( near( scene.getSplineControlPoints()[i], original[i] ), "undo restores original control order" );
        check( !scene.canUndoSplineEdit() && scene.redoSplineEdit() && near( scene.getSplineControlPoints()[segment + 1], clicked ), "redo restores exactly the clicked control" );
        click.mousePressed = click.mouseHeld = true;
        click.mousePosition = TransformPoint( ground, clicked );
        scene.handleInput( click );
        click.mousePressed = false;
        click.insertControlPoint = true;
        click.mousePosition = TransformPoint( ground, clicked + vec2{ 0.0f, 0.1f } );
        scene.handleInput( click );
        check( scene.getSplineControlPoints().size() == original.size() + 1 && near( scene.getSplineControlPoints()[segment + 1], clicked + vec2{ 0.0f, 0.1f } ), "Ctrl pressed during an existing control drag does not insert" );
        scene.cancelInput();
        check( !scene.getTerrainPointDragIndex(), "cancel clears click editing input" );
    }
    {
        rigidBodyDemo scene{ demoKind::splineTerrain };
        demoInput click{};
        click.mousePosition = scene.getSplineControlPoints()[3];
        click.mousePressed = click.mouseHeld = click.insertControlPoint = true;
        scene.handleInput( click );
        check( scene.getSplineControlPoints().size() == 9 && !scene.canUndoSplineEdit() && !scene.getTerrainPointDragIndex() && !scene.getWorld().IsValid( scene.getMouseJoint() ), "duplicate-point insertion rejects the edit without history or fallback dragging" );
        click.mousePressed = click.insertControlPoint = false;
        click.mousePosition = scene.getWorld().GetBodyTransform( scene.getShapes()[scene.getTerrainPointCount() - 1].bodyHandle ).position;
        scene.handleInput( click );
        check( scene.getSplineControlPoints().size() == 9 && !scene.getWorld().IsValid( scene.getMouseJoint() ), "rejected insertion consumes the held gesture" );
        scene.cancelInput();
        click.mousePosition = 0.5f * ( scene.getSplineControlPoints()[1] + scene.getSplineControlPoints()[2] );
        click.mousePressed = click.insertControlPoint = true;
        scene.handleInput( click );
        check( scene.getSplineControlPoints().size() == 10 && scene.undoSplineEdit(), "cancel allows a fresh insertion" );
        click.mousePosition = scene.getSplineControlPoints()[3];
        scene.handleInput( click );
        check( scene.getSplineControlPoints().size() == 9 && scene.canRedoSplineEdit(), "rejected insertion preserves redo history" );
        rigidBodyDemo polygon{ demoKind::terrainChain };
        click.mousePosition = polygon.getTerrainPoint( 1 );
        polygon.handleInput( click );
        check( polygon.getTerrainPointCount() == 9 && polygon.getTerrainPointDragIndex() == 1, "Ctrl insertion leaves polygon terrain input unchanged" );
    }

    return 0;
}
