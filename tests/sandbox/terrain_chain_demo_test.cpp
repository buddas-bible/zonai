#include "rigidBodyDemo.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string_view>
#include <variant>

using namespace zonai;
using namespace zonai::sandbox;

namespace
{

bool samePoint( vec2 a, vec2 b )
{
    return a.x == b.x && a.y == b.y;
}

} // namespace

int main()
{
    int failures = 0;
    const auto check = [&]( bool condition, const char* message )
    {
        if( condition ) return;

        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    };

    const auto entries = getDemoEntries();
    const auto terrainEntry = std::find_if( entries.begin(), entries.end(),
        []( const demoEntry& entry )
        {
            return std::string_view{ entry.name } == "체인 지형";
        } );

    check( terrainEntry != entries.end(), "terrain chain demo is registered" );
    if( terrainEntry == entries.end() ) return EXIT_FAILURE;

    check( std::string_view{ terrainEntry->category } == "지형", "terrain chain demo uses the terrain category" );

    rigidBodyDemo terrain{ terrainEntry->kind };
    world& simulation = terrain.getWorld();

    check( simulation.GetChainCount() == 1, "terrain scene owns one parent chain" );

    chainId parentChain{};
    std::size_t chainChildCount = 0;

    for( const visualShape& visual : terrain.getShapes() )
    {
        if( !simulation.IsValid( visual.shapeHandle ) ) continue;

        const chainId parent = simulation.GetShapeParentChain( visual.shapeHandle );
        if( !simulation.IsValid( parent ) ) continue;

        if( !simulation.IsValid( parentChain ) )
        {
            parentChain = parent;
        }
        else
        {
            check( parent.index1 == parentChain.index1 && parent.generation == parentChain.generation,
                "all terrain child shapes belong to one parent chain" );
        }

        ++chainChildCount;
    }

    check( simulation.IsValid( parentChain ), "terrain child shapes expose their parent chain" );
    if( simulation.IsValid( parentChain ) )
    {
        check( simulation.GetChainSegmentCount( parentChain ) == 8, "terrain parent chain owns eight segments" );
    }
    check( chainChildCount == 8, "terrain demo exposes all eight chain segments for debug drawing" );

    bool sawTerrainContact = false;
    for( int i = 0; i < 240 && !sawTerrainContact; ++i )
    {
        terrain.step( 1.0f / 60.0f, 4 );

        for( const contactData& contact : terrain.getContacts() )
        {
            const chainId parentA = simulation.GetShapeParentChain( contact.shapeA );
            const chainId parentB = simulation.GetShapeParentChain( contact.shapeB );
            const bool shapeAIsTerrain = simulation.IsValid( parentA ) && parentA.index1 == parentChain.index1 && parentA.generation == parentChain.generation;
            const bool shapeBIsTerrain = simulation.IsValid( parentB ) && parentB.index1 == parentChain.index1 && parentB.generation == parentChain.generation;
            if( shapeAIsTerrain || shapeBIsTerrain )
            {
                sawTerrainContact = true;
                break;
            }
        }
    }

    check( sawTerrainContact, "dynamic probes make real contact with the chain terrain" );

    for( const int substeps : { 1, 4, 8 } )
        for( std::size_t probe = 0; probe < 3; ++probe )
            for( const auto trial : { terrainTrial::seams, terrainTrial::front, terrainTrial::back } )
            {
                rigidBodyDemo scene{ demoKind::terrainChain };
                auto& physics = scene.getWorld();
                const auto body = scene.getShapes()[8 + probe].bodyHandle;
                scene.launchTerrainProbe( probe, trial );
                const vec2 a = scene.getTerrainPoint( 3 ), b = scene.getTerrainPoint( 4 );
                const vec2 normal = Cross( Normalize( b - a ), 1.0f );
                bool contacted = false, crossed = false;
                for( int frame = 0; frame < 120; ++frame )
                {
                    scene.step( 1.0f / 60.0f, substeps );
                    const auto position = physics.GetBodyTransform( body ).position;
                    check( IsFinite( position ), "terrain trials must remain finite" );
                    const float side = Dot( position - 0.5f * ( a + b ), normal );
                    for( const auto& contact : scene.getContacts() )
                    {
                        const auto selected = scene.getShapes()[8 + probe].shapeHandle;
                        if( contact.shapeA != selected && contact.shapeB != selected ) continue;
                        const auto other = contact.shapeA == selected ? contact.shapeB : contact.shapeA;
                        if( !physics.IsValid( physics.GetShapeParentChain( other ) ) ) continue;
                        contacted = true;
                        if( trial == terrainTrial::back && !crossed && side < 0.0f ) check( false, "back face must not block the approaching probe" );
                    }
                    if( side > 0.8f ) crossed = true;
                }
                if( trial == terrainTrial::back ) check( crossed, "back trial must pass through to the front side" );
                else check( contacted, "front and seam trials must exercise terrain contact" );
                if( trial == terrainTrial::seams ) check( physics.GetBodyTransform( body ).position.x < 6.0f, "seam trial must pass the first internal vertex" );
                physics.SetShapeFriction( scene.getShapes()[8 + probe].shapeHandle, 0.37f );
                physics.SetGravity( { 0.0f, -3.0f } );
                const auto initialPoint = scene.getTerrainPoint( 4 );
                demoInput edit{}; edit.mouseHeld = edit.mousePressed = true; edit.mousePosition = initialPoint;
                scene.handleInput( edit );
                edit.mousePressed = false; edit.mousePosition.y += 0.2f; scene.handleInput( edit );
                scene.resetTerrainProbes();
                check( samePoint( scene.getTerrainPoint( 4 ), edit.mousePosition ), "probe reset must preserve edited terrain" );
                check( physics.GetShape( scene.getShapes()[8 + probe].shapeHandle ).friction == 0.37f && physics.GetGravity().y == -3.0f, "probe reset must preserve material and gravity" );
                check( !scene.getTerrainPointDragIndex() && LengthSquared( physics.GetBodyLinearVelocity( body ) ) == 0.0f, "probe reset must cancel editing and stop motion" );
                check( physics.GetBodyCount() == 4 && physics.GetShapeCount() == 11 && physics.GetChainCount() == 1 && physics.getJointCount() == 0, "repeated terrain trials must reuse existing objects" );
            }

    for( const auto transform : { transform2{ { 20.0f, -3.0f }, {} }, transform2{ {}, rot2::FromRadians( 0.6f ) }, transform2{ { 20.0f, -3.0f }, rot2::FromRadians( 0.6f ) } } )
    {
        rigidBodyDemo scene{ demoKind::terrainChain };
        auto& physics = scene.getWorld();
        physics.SetBodyTransform( scene.getShapes()[0].bodyHandle, transform );
        demoInput click{};
        click.mousePressed = click.mouseHeld = true;
        click.mousePosition = TransformPoint( transform, scene.getTerrainPoint( 4 ) );
        scene.handleInput( click );
        check( scene.getTerrainPointDragIndex() == 4, "transformed terrain must select the visible world-space point" );
        check( !physics.IsValid( scene.getMouseJoint() ), "terrain point selection must not create a mouse joint" );
        const vec2 destination{ 0.3f, 1.1f };
        click.mousePressed = false;
        click.mousePosition = TransformPoint( transform, destination );
        scene.handleInput( click );
        check( Length( scene.getTerrainPoint( 4 ) - destination ) < 0.0001f, "world-space drag must update the corresponding local chain point" );
        const auto validPoint = scene.getTerrainPoint( 4 );
        click.mousePosition = TransformPoint( transform, scene.getTerrainPoint( 3 ) );
        scene.handleInput( click );
        check( samePoint( scene.getTerrainPoint( 4 ), validPoint ), "transformed drag must still reject collapsed edges" );
        scene.cancelInput();
        check( !scene.getTerrainPointDragIndex(), "transformed terrain drag must cancel cleanly" );
    }

    // 완만한 동일 경사의 이음새를 사용해 실제 지형 굴곡·마찰과 불필요한 걸림을 구분함.
    for( const int substeps : { 1, 4, 8 } )
        for( std::size_t probe = 0; probe < 3; ++probe )
        {
            rigidBodyDemo source{ demoKind::terrainChain };
            world simulation{};
            const auto ground = simulation.CreateBody();
            std::array<vec2, 9> points{};
            for( std::size_t i = 0; i < points.size(); ++i ) points[i] = { 8.0f - 2.0f * static_cast<float>( i ), 0.03f * static_cast<float>( i ) };
            chainDef chain{};
            chain.points = points;
            chain.ghost1 = { 10.0f, -0.03f };
            chain.ghost2 = { -10.0f, 0.27f };
            chain.friction = 0.0f;
            const auto terrain = simulation.CreateChain( ground, chain );
            const vec2 tangent = Normalize( points[1] - points[0] );
            const vec2 normal = Cross( tangent, 1.0f );
            const float supportHeight = probe == 1 ? 0.3f : 0.45f;
            const vec2 start = 0.5f * ( points[0] + points[1] );
            const auto body = simulation.CreateBody( bodyType::Dynamic, { start + ( supportHeight + 0.02f ) * normal, rot2::FromRadians( std::atan2( tangent.y, tangent.x ) ) } );
            const auto shape = simulation.CreateShape( body, source.getWorld().GetShape( source.getShapes()[8 + probe].shapeHandle ).geometry );
            simulation.SetShapeFriction( shape, 0.0f );
            simulation.SetBodyLinearVelocity( body, 4.0f * tangent );
            std::array<bool, 8> visited{};
            std::array<shapeId, 8> segments{};
            check( simulation.GetChainSegments( terrain, segments ) == segments.size(), "multi-seam fixture must expose all eight segments" );
            for( int frame = 0; frame < 180; ++frame )
            {
                simulation.Step( 1.0f / 60.0f, substeps );
                const auto position = simulation.GetBodyTransform( body ).position;
                const auto velocity = simulation.GetBodyLinearVelocity( body );
                check( IsFinite( position ) && IsFinite( velocity ), "multi-seam traversal must remain finite" );
                check( Dot( velocity, tangent ) > 3.4f, "smooth seams must not stop forward motion" );
                if( frame > 30 ) check( std::abs( Dot( position - start, normal ) - supportHeight ) < 0.08f, "smooth seams must not launch or drop the probe" );
                std::array<contactData, 8> contacts{};
                const auto count = simulation.GetBodyContactData( body, contacts );
                for( std::size_t i = 0; i < count; ++i )
                    for( std::size_t edge = 0; edge < segments.size(); ++edge )
                        if( contacts[i].shapeA == segments[edge] || contacts[i].shapeB == segments[edge] ) visited[edge] = true;
            }
            check( Dot( simulation.GetBodyTransform( body ).position - start, tangent ) > 10.0f, "probe must pass multiple successive internal vertices" );
            check( std::count( visited.begin(), visited.end(), true ) >= 6, "multi-seam test must exercise at least six chain segments" );
        }

    // Terrain demo는 parent chain point를 직접 선택해서 SetChainPoint 경로로 실시간 편집함.
    rigidBodyDemo editableTerrain{ terrainEntry->kind };
    check( editableTerrain.getTerrainPointCount() == 9, "terrain editor exposes nine editable points" );

    const vec2 editStart = editableTerrain.getTerrainPoint( 4 );
    check( samePoint( editStart, { 0.0f, 0.8f } ), "terrain editor point query" );

    demoInput press{};
    press.mousePosition = editStart;
    press.mousePressed = true;
    press.mouseHeld = true;
    editableTerrain.handleInput( press );

    const auto selectedPoint = editableTerrain.getTerrainPointDragIndex();
    check( selectedPoint.has_value() && *selectedPoint == 4, "terrain point press did not start point drag" );
    check( !editableTerrain.getWorld().IsValid( editableTerrain.getMouseJoint() ), "terrain point drag created a mouse joint" );

    const vec2 draggedPoint{ 0.5f, 1.6f };
    demoInput drag{};
    drag.mousePosition = draggedPoint;
    drag.mouseHeld = true;
    editableTerrain.handleInput( drag );
    check( samePoint( editableTerrain.getTerrainPoint( 4 ), draggedPoint ), "terrain point drag did not update chain geometry" );

    // Editor 입력은 debug assert를 일으킬 수 있는 퇴화 edge를 World에 넘기기 전에 거부함.
    const vec2 validPoint = editableTerrain.getTerrainPoint( 4 );
    drag.mousePosition = editableTerrain.getTerrainPoint( 3 );
    editableTerrain.handleInput( drag );
    check( samePoint( editableTerrain.getTerrainPoint( 4 ), validPoint ), "terrain point drag collapsed an adjacent edge" );

    demoInput release{};
    release.mousePosition = validPoint;
    editableTerrain.handleInput( release );
    check( !editableTerrain.getTerrainPointDragIndex().has_value(), "terrain point release kept drag state" );

    press.mousePosition = editableTerrain.getTerrainPoint( 2 );
    editableTerrain.handleInput( press );
    check( editableTerrain.getTerrainPointDragIndex().has_value(), "terrain point second press fixture" );
    editableTerrain.cancelInput();
    check( !editableTerrain.getTerrainPointDragIndex().has_value(), "cancel input kept terrain point drag state" );

    // Open chain의 내부 point는 두 실제 edge와 양쪽 ghost adjacency까지 최대 네 child에 나타남.
    world openWorld{};
    const bodyId openBody = openWorld.CreateBody();
    const std::array<vec2, 5> openPoints{
        vec2{ 4.0f, 0.0f },
        vec2{ 2.0f, 0.0f },
        vec2{ 0.0f, 0.0f },
        vec2{ -2.0f, 0.0f },
        vec2{ -4.0f, 0.0f }
    };
    chainDef openDefinition{};
    openDefinition.points = openPoints;
    openDefinition.ghost1 = { 6.0f, 0.0f };
    openDefinition.ghost2 = { -6.0f, 0.0f };

    const chainId openChain = openWorld.CreateChain( openBody, openDefinition );
    std::array<shapeId, 4> openSegments{};
    check( openWorld.GetChainSegments( openChain, openSegments ) == openSegments.size(), "open edit segment fixture" );

    const vec2 raisedPoint{ 0.0f, 1.0f };
    openWorld.SetChainPoint( openChain, 2, raisedPoint );

    std::array<shapeId, 4> stableOpenSegments{};
    check( openWorld.GetChainSegments( openChain, stableOpenSegments ) == stableOpenSegments.size(), "open edited segment query" );
    for( std::size_t i = 0; i < openSegments.size(); ++i )
    {
        check( stableOpenSegments[i] == openSegments[i], "open chain child handle changed" );
    }

    const chainSegment2& open0 = std::get<chainSegment2>( openWorld.GetShape( openSegments[0] ).geometry );
    const chainSegment2& open1 = std::get<chainSegment2>( openWorld.GetShape( openSegments[1] ).geometry );
    const chainSegment2& open2 = std::get<chainSegment2>( openWorld.GetShape( openSegments[2] ).geometry );
    const chainSegment2& open3 = std::get<chainSegment2>( openWorld.GetShape( openSegments[3] ).geometry );
    check( samePoint( open0.ghost2, raisedPoint ), "open point did not update left ghost2" );
    check( samePoint( open1.segment.b, raisedPoint ), "open point did not update previous edge endpoint" );
    check( samePoint( open2.segment.a, raisedPoint ), "open point did not update next edge endpoint" );
    check( samePoint( open3.ghost1, raisedPoint ), "open point did not update right ghost1" );

    // Endpoint를 움직여도 chain 바깥의 ghost point는 그대로 두고 내부 adjacency만 이어야 함.
    const vec2 raisedEndpoint{ 4.0f, 1.0f };
    openWorld.SetChainPoint( openChain, 0, raisedEndpoint );
    const chainSegment2& endpoint0 = std::get<chainSegment2>( openWorld.GetShape( openSegments[0] ).geometry );
    const chainSegment2& endpoint1 = std::get<chainSegment2>( openWorld.GetShape( openSegments[1] ).geometry );
    check( samePoint( endpoint0.ghost1, openDefinition.ghost1 ), "open endpoint changed external ghost1" );
    check( samePoint( endpoint0.segment.a, raisedEndpoint ), "open endpoint did not update first edge" );
    check( samePoint( endpoint1.ghost1, raisedEndpoint ), "open endpoint did not update adjacent ghost1" );
    check( samePoint( endpoint0.ghost2, raisedPoint ), "open endpoint disturbed unrelated internal ghost" );

    // Loop는 양 끝이 없으므로 같은 point의 adjacency가 배열 경계를 넘어 wrap-around 함.
    world loopWorld{};
    const bodyId loopBody = loopWorld.CreateBody();
    const std::array<vec2, 4> loopPoints{
        vec2{ -2.0f, -1.0f },
        vec2{ -2.0f, 1.0f },
        vec2{ 2.0f, 1.0f },
        vec2{ 2.0f, -1.0f }
    };
    chainDef loopDefinition{};
    loopDefinition.points = loopPoints;
    loopDefinition.isLoop = true;
    const chainId loopChain = loopWorld.CreateChain( loopBody, loopDefinition );
    std::array<shapeId, 4> loopSegments{};
    check( loopWorld.GetChainSegments( loopChain, loopSegments ) == loopSegments.size(), "loop edit segment fixture" );

    const vec2 movedLoopPoint{ -3.0f, -2.0f };
    loopWorld.SetChainPoint( loopChain, 0, movedLoopPoint );
    const chainSegment2& loop0 = std::get<chainSegment2>( loopWorld.GetShape( loopSegments[0] ).geometry );
    const chainSegment2& loop1 = std::get<chainSegment2>( loopWorld.GetShape( loopSegments[1] ).geometry );
    const chainSegment2& loop2 = std::get<chainSegment2>( loopWorld.GetShape( loopSegments[2] ).geometry );
    const chainSegment2& loop3 = std::get<chainSegment2>( loopWorld.GetShape( loopSegments[3] ).geometry );
    check( samePoint( loop0.segment.a, movedLoopPoint ), "loop point did not update own edge" );
    check( samePoint( loop1.ghost1, movedLoopPoint ), "loop point did not wrap into ghost1" );
    check( samePoint( loop2.ghost2, movedLoopPoint ), "loop point did not wrap into ghost2" );
    check( samePoint( loop3.segment.b, movedLoopPoint ), "loop point did not wrap into previous edge" );

    // Geometry replacement는 stable child handle을 유지하면서 stale contact를 버리고 새 proxy 위치에서 pair를 다시 찾아야 함.
    world contactWorld{};
    const bodyId contactTerrainBody = contactWorld.CreateBody();
    const std::array<vec2, 3> contactPoints{ vec2{ 2.0f, -2.0f }, vec2{ 0.0f, -2.0f }, vec2{ -2.0f, -2.0f } };
    chainDef contactDefinition{};
    contactDefinition.points = contactPoints;
    contactDefinition.ghost1 = { 4.0f, -2.0f };
    contactDefinition.ghost2 = { -4.0f, -2.0f };
    const chainId contactChain = contactWorld.CreateChain( contactTerrainBody, contactDefinition );
    std::array<shapeId, 2> contactSegments{};
    check( contactWorld.GetChainSegments( contactChain, contactSegments ) == contactSegments.size(), "contact edit segment fixture" );

    const bodyId probeBody = contactWorld.CreateBody( bodyType::Dynamic, { { 0.0f, 0.45f }, {} } );
    const shapeId probeShape = contactWorld.CreateShape( probeBody, circle2{ {}, 0.5f } );
    contactWorld.UpdateCollisions( []( const contactData& ) {} );
    std::array<contactData, 2> contacts{};
    check( contactWorld.GetShapeContactData( probeShape, contacts ) == 0, "contact edit fixture starts overlapping" );

    contactWorld.SetChainPoint( contactChain, 1, { 0.0f, 0.0f } );
    std::array<shapeId, 2> movedContactSegments{};
    check( contactWorld.GetChainSegments( contactChain, movedContactSegments ) == movedContactSegments.size(), "contact edited segment query" );
    check( movedContactSegments == contactSegments, "contact edit changed child handles" );

    contactWorld.UpdateCollisions( []( const contactData& ) {} );
    check( contactWorld.GetShapeContactData( probeShape, contacts ) > 0, "moved chain proxy did not create contact" );
    const contactId oldContact = contacts[0].id;

    contactWorld.SetBodyAwake( probeBody, false );
    check( !contactWorld.IsBodyAwake( probeBody ), "contact edit sleep fixture" );

    contactWorld.SetChainPoint( contactChain, 1, { 0.0f, -2.0f } );
    check( contactWorld.IsBodyAwake( probeBody ), "chain point edit did not wake body from destroyed contact" );
    check( !contactWorld.IsValid( oldContact ), "chain point edit kept stale contact" );
    contactWorld.UpdateCollisions( []( const contactData& ) {} );
    check( contactWorld.GetShapeContactData( probeShape, contacts ) == 0, "chain point edit recreated stale contact" );

    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
