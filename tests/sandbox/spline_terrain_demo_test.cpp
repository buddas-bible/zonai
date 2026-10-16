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
        transformed.launchTerrainProbe( 0, terrainTrial::front );
        const vec2 a = TransformPoint( pose, transformed.getTerrainPoint( 3 ) );
        const vec2 b = TransformPoint( pose, transformed.getTerrainPoint( 4 ) );
        const vec2 normal = Cross( Normalize( b - a ), 1.0f );

        check( near( simulation.GetBodyTransform( transformed.getImpulseBody() ).position, 0.5f * ( a + b ) + 1.5f * normal ), "spline terrain launch converts sampled local points to world space" );
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
                const vec2 a = scene.getTerrainPoint( 3 ), b = scene.getTerrainPoint( 4 );
                const vec2 normal = Cross( Normalize( b - a ), 1.0f );
                bool contacted = false, crossed = false;
                int contactFrames = 0;

                for( int frame = 0; frame < 120; ++frame )
                {
                    scene.step( 1.0f / 60.0f, substeps );
                    const vec2 position = simulation.GetBodyTransform( selected.bodyHandle ).position;
                    check( IsFinite( position ), "spline terrain trials remain finite" );
                    const float side = Dot( position - 0.5f * ( a + b ), normal );
                    bool frameContact = false;

                    for( const auto& contact : scene.getContacts() )
                    {
                        if( contact.shapeA != selected.shapeHandle && contact.shapeB != selected.shapeHandle ) continue;
                        const auto other = contact.shapeA == selected.shapeHandle ? contact.shapeB : contact.shapeA;
                        if( !simulation.IsValid( simulation.GetShapeParentChain( other ) ) ) continue;
                        contacted = true;
                        frameContact = true;
                        if( trial == terrainTrial::back && !crossed && side < 0.0f ) check( false, "sampled terrain must not block a probe approaching its back face" );
                    }

                    if( frameContact ) ++contactFrames;
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

    return 0;
}
