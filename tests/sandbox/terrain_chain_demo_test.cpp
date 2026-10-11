#include "rigidBodyDemo.h"

#include <algorithm>
#include <array>
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

    contactWorld.SetChainPoint( contactChain, 1, { 0.0f, -2.0f } );
    check( !contactWorld.IsValid( oldContact ), "chain point edit kept stale contact" );
    contactWorld.UpdateCollisions( []( const contactData& ) {} );
    check( contactWorld.GetShapeContactData( probeShape, contacts ) == 0, "chain point edit recreated stale contact" );

    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
