#include <array>
#include <cstdio>
#include <cstdlib>

#include "dynamics/world.h"
#include "geometry/circle2.h"

using namespace zonai;

namespace
{

void check( bool condition, const char* message )
{
    if( !condition )
    {
        std::fprintf( stderr, "%s\n", message );
        std::exit( EXIT_FAILURE );
    }
}

void update( world& simulation )
{
    simulation.UpdateCollisions( []( const contactData& ) {} );
}

#pragma region ContactQueries

void checkContactQueries()
{
    world simulation{};
    const bodyId owner = simulation.CreateBody( bodyType::Dynamic );
    const shapeId first = simulation.CreateShape( owner, circle2{ {}, 1.0f } );
    const shapeId second = simulation.CreateShape( owner, circle2{ { 10.0f, 0.0f }, 1.0f } );
    bodyId emptyTarget{};
    for( const float x : { 1.5f, 11.5f, -2.1f } )
    {
        const bodyId target = simulation.CreateBody( bodyType::Static, { { x, 0.0f }, {} } );
        (void)simulation.CreateShape( target, circle2{ {}, 1.0f } );
        emptyTarget = target;
    }
    update( simulation );
    check( simulation.GetBodyContactCapacity( owner ) == 3, "conservative contact capacity fixture" );
    check( simulation.GetShapeContactCapacity( second ) == 3, "shape capacity excludes sibling candidates" );
    std::array<contactData, 3> contacts{};
    check( simulation.GetBodyContactData( owner, {} ) == 0 && simulation.GetShapeContactData( first, {} ) == 0, "empty contact output" );
    check( simulation.GetBodyContactData( owner, std::span{ contacts }.first( 1 ) ) == 1 && contacts[1].id == contactId{}, "short contact output" );
    check( simulation.GetBodyContactData( owner, contacts ) == 2 && contacts[2].id == contactId{}, "zero-point candidate consumes output" );
    check( simulation.GetShapeContactData( second, contacts ) == 1, "shape query includes sibling contact" );
    check( contacts[0].shapeA == second || contacts[0].shapeB == second, "shape query returned wrong pair" );
    const contactData snapshot = contacts[0];
    const contactData direct = simulation.GetContactData( snapshot.id );
    check( direct.shapeA == snapshot.shapeA && direct.shapeB == snapshot.shapeB && direct.manifold.pointCount == 1, "direct contact snapshot" );
    simulation.SetBodyTransform( emptyTarget, { { -2.005f, 0.0f }, {} } );
    update( simulation );
    check( simulation.GetBodyContactData( owner, contacts ) == 3, "speculative contact query omitted" );
    bool hasSpeculative = false;
    for( const contactData& data : contacts ) { hasSpeculative |= data.manifold.points[0].separation > 0.0f; }
    check( hasSpeculative, "speculative contact fixture" );
    int callbacks = 0;
    simulation.UpdateCollisions( [&]( const contactData& data )
    {
        check( simulation.IsValid( data.id ) && data.manifold.pointCount == 1, "collision callback snapshot" );
        ++callbacks;
    } );
    check( callbacks == 2, "collision callback includes speculative-only pair" );
    simulation.SetBodyTransform( owner, { { 100.0f, 0.0f }, {} } );
    update( simulation );
    check( !simulation.IsValid( snapshot.id ) && snapshot.manifold.pointCount == 1, "copied contact snapshot changed after removal" );
}

#pragma endregion
#pragma region SensorFiltersAndLimits

void checkSensorQueries()
{
    world simulation{};
    const bodyId owner = simulation.CreateBody();
    const shapeId sensor = simulation.CreateSensorShape( owner, circle2{ {}, 2.0f }, { 1, 2, 0 } );
    const shapeId self = simulation.CreateShape( owner, circle2{ {}, 0.25f }, { 2, 1, 0 } );
    simulation.SetShapeSensorEventsEnabled( sensor, true );
    simulation.SetShapeSensorEventsEnabled( self, true );
    std::array<shapeId, 3> visitors{};
    int index = 0;
    for( const bodyType type : { bodyType::Static, bodyType::Kinematic, bodyType::Dynamic } )
    {
        const bodyId visitorBody = simulation.CreateBody( type );
        visitors[index] = simulation.CreateShape( visitorBody, circle2{ {}, 0.25f }, { 2, 1, 0 } );
        simulation.SetShapeSensorEventsEnabled( visitors[index++], true );
    }
    const bodyId outsideBody = simulation.CreateBody( bodyType::Static, { { 2.3f, 0.0f }, {} } );
    const shapeId outside = simulation.CreateShape( outsideBody, circle2{ {}, 0.25f }, { 2, 1, 0 } );
    simulation.SetShapeSensorEventsEnabled( outside, true );
    simulation.Step( 0.0f );
    check( simulation.GetShapeSensorCapacity( sensor ) == 3 && simulation.GetSensorBeginEvents().size() == 3, "sensor all-tree query or same-body exclusion" );
    check( simulation.GetContactCount() == 0, "sensor query created solid contact" );
    std::array<shapeId, 4> output{};
    check( simulation.GetShapeSensorData( sensor, {} ) == 0, "empty sensor output" );
    check( simulation.GetShapeSensorData( sensor, std::span{ output }.first( 1 ) ) == 1 && output[1] == shapeId{}, "short sensor output" );
    check( simulation.GetShapeSensorData( sensor, output ) == 3 && output[3] == shapeId{}, "sensor output overrun" );
    check( output[0] == visitors[0] && output[1] == visitors[1] && output[2] == visitors[2], "sensor overlap order or duplicates" );
    check( simulation.GetShapeSensorCapacity( self ) == 0 && simulation.GetShapeSensorData( self, output ) == 0, "solid shape sensor query" );
    std::array<contactData, 1> contacts{};
    check( simulation.GetShapeContactCapacity( sensor ) == 0 && simulation.GetShapeContactData( sensor, contacts ) == 0, "sensor contact query" );
    simulation.Step( 0.0f );
    check( simulation.GetSensorBeginEvents().empty() && simulation.GetSensorEndEvents().empty(), "persistent sensor overlap repeated event" );
    simulation.SetShapeSensorEventsEnabled( sensor, false );
    check( simulation.GetShapeSensorCapacity( sensor ) == 3, "sensor opt-out changed snapshot immediately" );
    simulation.Step( 0.0f );
    check( simulation.GetShapeSensorCapacity( sensor ) == 0 && simulation.GetSensorEndEvents().size() == 3, "sensor opt-out missed end events" );
    simulation.Step( 0.0f );
    check( simulation.GetSensorEndEvents().empty(), "sensor end events survived next step" );
}

void checkSensorFilters()
{
    // Removing either mask check or group precedence must change these independently specified results.
    struct filterCase { collisionFilter sensor; collisionFilter visitor; bool overlaps; };
    constexpr std::uint64_t highBit = 1ull << 63;
    const std::array cases
    {
        filterCase{ { highBit, 1, 0 }, { 1, highBit, 0 }, true },
        filterCase{ { highBit, 0, 0 }, { 1, highBit, 0 }, false },
        filterCase{ { highBit, 1, 0 }, { 1, 0, 0 }, false },
        filterCase{ { highBit, 0, 7 }, { 1, 0, 7 }, true },
        filterCase{ { highBit, 1, -7 }, { 1, highBit, -7 }, false }
    };
    for( const bool visitorIsSensor : { false, true } )
    {
        for( const filterCase& entry : cases )
        {
            world simulation{};
            const bodyId owner = simulation.CreateBody();
            const shapeId sensor = simulation.CreateSensorShape( owner, circle2{ {}, 2.0f }, entry.sensor );
            const bodyId target = simulation.CreateBody();
            const shapeId visitor = visitorIsSensor ? simulation.CreateSensorShape( target, circle2{ {}, 0.25f }, entry.visitor ) : simulation.CreateShape( target, circle2{ {}, 0.25f }, entry.visitor );
            simulation.SetShapeSensorEventsEnabled( sensor, true );
            simulation.Step( 0.0f );
            check( simulation.GetShapeSensorCapacity( sensor ) == 0, "visitor opt-in ignored" );
            simulation.SetShapeSensorEventsEnabled( visitor, true );
            simulation.Step( 0.0f );
            check( simulation.GetShapeSensorCapacity( sensor ) == ( entry.overlaps ? 1u : 0u ), "sensor mask/group filter" );
            simulation.SetShapeFilter( visitor, { 1, 0, 0 } );
            simulation.Step( 0.0f );
            check( simulation.GetShapeSensorCapacity( sensor ) == 0 && simulation.GetSensorEndEvents().size() == ( entry.overlaps ? ( visitorIsSensor ? 2u : 1u ) : 0u ), "stationary sensor refilter" );
        }
    }
}

#pragma endregion
#pragma region SensorLifetime

void checkVisitorReuse()
{
    world simulation{};
    const bodyId owner = simulation.CreateBody();
    const shapeId sensor = simulation.CreateSensorShape( owner, circle2{ {}, 2.0f } );
    const bodyId visitorBody = simulation.CreateBody();
    const shapeId oldVisitor = simulation.CreateShape( visitorBody, circle2{ {}, 0.25f } );
    simulation.SetShapeSensorEventsEnabled( sensor, true );
    simulation.SetShapeSensorEventsEnabled( oldVisitor, true );
    simulation.Step( 0.0f );
    check( simulation.GetSensorBeginEvents().size() == 1, "visitor lifetime fixture" );
    const sensorBeginEvent2 copiedBegin = simulation.GetSensorBeginEvents()[0];
    simulation.DestroyShape( oldVisitor );
    const shapeId newVisitor = simulation.CreateShape( visitorBody, circle2{ {}, 0.25f } );
    simulation.SetShapeSensorEventsEnabled( newVisitor, true );
    check( oldVisitor.index1 == newVisitor.index1 && oldVisitor.generation != newVisitor.generation, "visitor reuse fixture" );
    std::array<shapeId, 1> output{};
    check( simulation.GetShapeSensorData( sensor, output ) == 1 && output[0] == oldVisitor && !simulation.IsValid( output[0] ), "sensor snapshot resolved reused slot to new visitor" );
    simulation.Step( 0.0f );
    check( simulation.GetSensorEndEvents().size() == 1 && simulation.GetSensorEndEvents()[0].visitorShapeId == oldVisitor, "old generation end lost" );
    check( simulation.GetSensorBeginEvents().size() == 1 && simulation.GetSensorBeginEvents()[0].visitorShapeId == newVisitor, "new generation begin lost" );
    check( copiedBegin.visitorShapeId == oldVisitor && simulation.GetShapeSensorData( sensor, output ) == 1 && output[0] == newVisitor, "sensor copied event or refreshed query" );
    simulation.DestroyBody( visitorBody );
    simulation.DestroyBody( owner );
    simulation.Step( 0.0f );
    check( simulation.GetSensorEndEvents().size() == 1 && simulation.GetSensorEndEvents()[0].sensorShapeId == sensor && simulation.GetSensorEndEvents()[0].visitorShapeId == newVisitor, "pending end without live sensors" );
    simulation.Step( 0.0f );
    check( simulation.GetSensorBeginEvents().empty() && simulation.GetSensorEndEvents().empty(), "pending end published twice" );
}

void checkSensorSwap()
{
    world simulation{};
    const bodyId owner = simulation.CreateBody();
    const shapeId first = simulation.CreateSensorShape( owner, circle2{ {}, 2.0f } );
    const shapeId second = simulation.CreateSensorShape( owner, circle2{ { 10.0f, 0.0f }, 2.0f } );
    const bodyId visitorBody = simulation.CreateBody();
    const shapeId visitor = simulation.CreateShape( visitorBody, circle2{ { 10.0f, 0.0f }, 0.25f } );
    const shapeId firstVisitor = simulation.CreateShape( visitorBody, circle2{ {}, 0.25f } );
    simulation.SetShapeSensorEventsEnabled( first, true );
    simulation.SetShapeSensorEventsEnabled( second, true );
    simulation.SetShapeSensorEventsEnabled( visitor, true );
    simulation.SetShapeSensorEventsEnabled( firstVisitor, true );
    simulation.Step( 0.0f );
    check( simulation.GetSensorBeginEvents().size() == 2, "sensor swap fixture" );
    simulation.DestroyShape( first );
    check( simulation.GetSensorEndEvents().empty(), "sensor removal changed published events immediately" );
    simulation.Step( 0.0f );
    check( simulation.GetSensorEndEvents().size() == 1 && simulation.GetSensorEndEvents()[0].sensorShapeId == first && simulation.GetSensorEndEvents()[0].visitorShapeId == firstVisitor, "sensor removal lost historical end" );
    std::array<shapeId, 1> output{};
    check( simulation.GetShapeSensorData( second, output ) == 1 && output[0] == visitor, "sensor dense swap broke query" );
    simulation.SetShapeSensorEventsEnabled( visitor, false );
    simulation.Step( 0.0f );
    check( simulation.GetSensorEndEvents().size() == 1 && simulation.GetSensorEndEvents()[0].sensorShapeId == second, "visitor opt-out after sensor swap" );
}

#pragma endregion
#pragma region TreeQueryCallback

void checkTreeCallback()
{
    dynamicTree tree{};
    const aabb2 bounds{ { -1.0f, -1.0f }, { 1.0f, 1.0f } };
    int calls = 0;
    const auto stop = [&]( std::int32_t ) { ++calls; return false; };
    tree.Query( bounds, stop );
    check( calls == 0, "empty tree callback" );
    for( int i = 0; i < 8; ++i ) { (void)tree.CreateProxy( bounds, i ); }
    for( const bool rebuild : { false, true } )
    {
        if( rebuild ) { (void)tree.Rebuild( true ); }
        calls = 0;
        tree.Query( bounds, stop );
        check( calls == 1, "tree callback false did not stop query" );
        int seen = 0;
        tree.Query( bounds, [&]( std::int32_t proxy ) { seen |= 1 << tree.GetProxyShapeIndex( proxy ); ++calls; return true; } );
        check( calls == 9 && seen == 255, "tree query missed or repeated leaf" );
    }
}

#pragma endregion

} // namespace

int main()
{
    checkContactQueries();
    checkSensorQueries();
    checkSensorFilters();
    checkVisitorReuse();
    checkSensorSwap();
    checkTreeCallback();
}
