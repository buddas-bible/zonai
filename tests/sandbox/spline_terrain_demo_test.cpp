#include "rigidBodyDemo.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <string_view>
#include <variant>

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
    const auto shapes = terrain.getShapes();

    check( physics.GetChainCount() == 1 && physics.GetBodyCount() == 4, "spline terrain uses one static chain and three dynamic probes" );
    check( terrain.getTerrainPointCount() == 33 && shapes.size() == 35, "eight spline segments at four subdivisions create thirty-two chain children" );

    const chainId parent = physics.GetShapeParentChain( shapes[0].shapeHandle );
    check( physics.IsValid( parent ), "sampled terrain is a real parent chain" );

    const std::array<vec2, 9> controlPoints{
        vec2{ 8.0f, -2.0f }, vec2{ 6.0f, -1.2f }, vec2{ 4.0f, -1.0f },
        vec2{ 2.0f, 0.0f }, vec2{ 0.0f, 0.8f }, vec2{ -2.0f, 0.1f },
        vec2{ -4.0f, -0.8f }, vec2{ -6.0f, -0.4f }, vec2{ -8.0f, -1.2f }
    };

    for( std::size_t i = 0; i < controlPoints.size(); ++i )
        check( near( terrain.getTerrainPoint( 4 * i ), controlPoints[i] ), "sampled chain passes through every spline control point" );

    check( LengthSquared( terrain.getTerrainPoint( 1 ) - vec2{ 7.5f, -1.8f } ) > 1.0e-6f, "terrain follows the cubic curve rather than only splitting straight edges" );

    for( std::size_t i = 0; i < 32; ++i )
    {
        const shape& child = physics.GetShape( shapes[i].shapeHandle );
        const chainSegment2& edge = std::get<chainSegment2>( child.geometry );
        const chainId childParent = physics.GetShapeParentChain( shapes[i].shapeHandle );

        check( childParent.index1 == parent.index1 && childParent.generation == parent.generation, "all sampled edges share the existing chain owner" );
        check( near( edge.segment.a, terrain.getTerrainPoint( i ) ) && near( edge.segment.b, terrain.getTerrainPoint( i + 1 ) ), "chain collision geometry uses the sampled points" );
        check( LengthSquared( edge.segment.b - edge.segment.a ) > LINEAR_SLOP * LINEAR_SLOP, "every sampled edge satisfies the chain minimum length" );
        check( Cross( Normalize( edge.segment.b - edge.segment.a ), 1.0f ).y > 0.0f, "right-to-left sample order keeps terrain front faces upward" );

        const vec2 previous = i == 0 ? edge.segment.a + ( edge.segment.a - edge.segment.b ) : terrain.getTerrainPoint( i - 1 );
        const vec2 next = i == 31 ? edge.segment.b + ( edge.segment.b - edge.segment.a ) : terrain.getTerrainPoint( i + 2 );

        check( near( edge.ghost1, previous ) && near( edge.ghost2, next ), "ghost points extend endpoint edges and connect adjacent samples" );
        check( child.friction == 0.8f, "sampled terrain preserves the existing terrain material" );
    }

    demoInput input{};
    input.mousePosition = terrain.getTerrainPoint( 4 );
    input.mousePressed = true;
    input.mouseHeld = true;
    terrain.handleInput( input );
    check( terrain.getTerrainPointDragIndex().has_value() && *terrain.getTerrainPointDragIndex() == 1, "spline press selects the control point rather than the sample index" );
    check( !physics.IsValid( terrain.getMouseJoint() ), "control point drag takes priority over mouse joints" );
    input.mousePosition.y += 1.0f;
    input.mousePressed = false;
    terrain.handleInput( input );
    check( near( terrain.getTerrainPoint( 4 ), { 6.0f, -0.2f } ), "control drag resamples chain geometry while paused" );
    check( terrain.getSplineControlPoints().size() == 9 && near( terrain.getSplineControlPoints()[1], { 6.0f, -0.2f } ), "control drag commits the stored control point" );
    std::array<vec2, 33> validSamples{};
    for( std::size_t i = 0; i < validSamples.size(); ++i ) validSamples[i] = terrain.getTerrainPoint( i );
    input.mousePosition = controlPoints[0];
    terrain.handleInput( input );
    input.mousePosition = controlPoints[0] + vec2{ 0.0f, -0.001f };
    terrain.handleInput( input );
    for( std::size_t i = 0; i < validSamples.size(); ++i )
        check( near( terrain.getTerrainPoint( i ), validSamples[i] ), "invalid control drag leaves the entire sampled chain unchanged" );
    check( near( terrain.getSplineControlPoints()[1], { 6.0f, -0.2f } ), "invalid drag keeps the last valid control point" );
    for( std::size_t i = 0; i < 32; ++i ) check( physics.IsValid( shapes[i].shapeHandle ), "control drag preserves every chain child handle" );
    terrain.cancelInput();
    check( !terrain.getTerrainPointDragIndex().has_value(), "cancel clears control drag selection" );

    input.mousePosition = validSamples[1];
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
        check( near( transformed.getTerrainPoint( 4 ), controlPoints[1] ), "moving the terrain body preserves sampled local geometry" );
        demoInput press{};
        press.mousePosition = TransformPoint( pose, controlPoints[4] );
        press.mousePressed = press.mouseHeld = true;
        transformed.handleInput( press );
        check( transformed.getTerrainPointDragIndex().has_value() && *transformed.getTerrainPointDragIndex() == 4, "transformed spline control is selected in local coordinates" );
        press.mousePosition = TransformPoint( pose, { 0.5f, 1.6f } );
        press.mousePressed = false;
        transformed.handleInput( press );
        check( near( transformed.getTerrainPoint( 16 ), { 0.5f, 1.6f } ), "transformed control drag stores local sampled geometry" );
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
                const auto selected = scene.getShapes()[32 + probe];
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
                        for( std::size_t i = 0; i < 32; ++i )
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
                for( std::size_t i = 32; i < 35; ++i )
                    check( LengthSquared( simulation.GetBodyLinearVelocity( scene.getShapes()[i].bodyHandle ) ) == 0.0f, "spline terrain probe reset stops every probe" );
                check( simulation.GetChainCount() == 1 && simulation.GetShapeCount() == 35, "trials and reset preserve the sampled terrain chain" );
            }

    return 0;
}
