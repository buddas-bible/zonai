#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <memory>

#include "dynamics/world.h"
#include "geometry/circle2.h"

using namespace zonai;

namespace
{

// assert는 Release에서 제거되므로 수명과 list 검사는 항상 실행함.
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
    simulation.UpdateCollisions(
        []( const contactData& )
        {
        } );
}

#pragma region HandleLifetime

void checkOwnership()
{
    world first{};
    world second{};
    const bodyId firstBody = first.CreateBody( bodyType::Dynamic );
    const bodyId secondBody = second.CreateBody( bodyType::Dynamic );
    check( firstBody.index1 == secondBody.index1 && firstBody.generation == secondBody.generation, "ownership fixture" );
    check( !first.IsValid( secondBody ) && !second.IsValid( firstBody ), "foreign body accepted" );
    const shapeId firstShape = first.CreateShape( firstBody, circle2{ {}, 1.0f } );
    const shapeId secondShape = second.CreateShape( secondBody, circle2{ {}, 1.0f } );
    check( !first.IsValid( secondShape ) && !second.IsValid( firstShape ), "foreign shape accepted" );
    const bodyId firstStatic = first.CreateBody();
    const bodyId secondStatic = second.CreateBody();
    ( void )first.CreateShape( firstStatic, circle2{ {}, 1.0f } );
    ( void )second.CreateShape( secondStatic, circle2{ {}, 1.0f } );
    update( first );
    update( second );
    std::array<contactData, 1> firstContacts{}, secondContacts{};
    check( first.GetBodyContactData( firstBody, firstContacts ) == 1 && second.GetBodyContactData( secondBody, secondContacts ) == 1, "contact fixture" );
    check( !first.IsValid( secondContacts[0].id ) && !second.IsValid( firstContacts[0].id ), "foreign contact accepted" );
}

void checkWorldReuse()
{
    alignas( world ) std::byte storage[sizeof( world )];
    world* first = std::construct_at( reinterpret_cast<world*>( storage ) );
    const bodyId oldBody = first->CreateBody( bodyType::Dynamic );
    const shapeId oldShape = first->CreateShape( oldBody, circle2{ {}, 1.0f } );
    const bodyId staticBody = first->CreateBody();
    ( void )first->CreateShape( staticBody, circle2{ {}, 1.0f } );
    update( *first );
    std::array<contactData, 1> contacts{};
    check( first->GetBodyContactData( oldBody, contacts ) == 1, "world reuse contact fixture" );
    const contactId oldContact = contacts[0].id;
    std::destroy_at( first );
    world* second = std::construct_at( reinterpret_cast<world*>( storage ) );
    const bodyId newBody = second->CreateBody( bodyType::Dynamic );
    ( void )second->CreateShape( newBody, circle2{ {}, 1.0f } );
    const bodyId newStatic = second->CreateBody();
    ( void )second->CreateShape( newStatic, circle2{ {}, 1.0f } );
    update( *second );
    check( second->GetBodyContactData( newBody, contacts ) == 1, "new world contact fixture" );
    check( oldBody.index1 == newBody.index1 && oldContact.index1 == contacts[0].id.index1, "world reuse slots" );
    check( !second->IsValid( oldBody ) && !second->IsValid( oldShape ) && !second->IsValid( oldContact ), "previous world handle resurrected" );
    std::destroy_at( second );
}

#pragma endregion HandleLifetime

#pragma region ShapeAndContactLifecycle

void checkLists()
{
    // 같은 geometry 세 개로 양쪽 body의 contact edge와 shape list를 함께 검사함.
    world simulation{};
    const bodyId dynamicBody = simulation.CreateBody( bodyType::Dynamic );
    const bodyId staticBody = simulation.CreateBody();
    ( void )simulation.CreateShape( staticBody, circle2{ {}, 1.0f } );
    std::array<shapeId, 3> shapes{};
    for( shapeId& id : shapes )
        id = simulation.CreateShape( dynamicBody, circle2{ {}, 1.0f } );
    update( simulation );
    std::array<contactData, 3> contacts{};
    check( simulation.GetBodyContactData( dynamicBody, contacts ) == 3, "three contacts" );
    const auto oldContacts = contacts;
    // 삽입 순서와 관계없이 head/middle/tail을 각각 제거하는 모든 순서를 검사함.
    const float initialMass = simulation.GetBodyMass( dynamicBody );
    simulation.DestroyShape( shapes[1] );
    check( simulation.GetShape( shapes[2] ).nextShapeId == shapes[0].index1 - 1, "middle shape next" );
    check( simulation.GetShape( shapes[0] ).prevShapeId == shapes[2].index1 - 1, "middle shape prev" );
    check( simulation.GetBody( dynamicBody ).shapeCount == 2 && simulation.GetBody( staticBody ).contactCount == 2, "middle unlink counts" );
    check( simulation.GetBodyMass( dynamicBody ) < initialMass, "mass after unlink" );
    simulation.DestroyShape( shapes[2] );
    simulation.DestroyShape( shapes[0] );
    check( simulation.GetBody( dynamicBody ).headShapeId == body::NULL_INDEX, "empty shape head" );
    check( simulation.GetBody( dynamicBody ).headContactKey == body::NULL_INDEX && simulation.GetBody( staticBody ).headContactKey == body::NULL_INDEX, "empty contact heads" );
    check( simulation.GetContactCount() == 0, "contacts after all shapes removed" );
    for( const auto& data : oldContacts )
        check( !simulation.IsValid( data.id ), "destroyed contact valid" );
    const shapeId reusedShape = simulation.CreateShape( dynamicBody, circle2{ {}, 1.0f } );
    check( reusedShape.index1 == shapes[0].index1 && reusedShape.generation != shapes[0].generation, "shape generation reuse" );
    update( simulation );
    check( simulation.GetBodyContactData( dynamicBody, contacts ) == 1, "pair can be recreated" );
    const contactId reusedContact = contacts[0].id;
    for( const auto& data : oldContacts )
        check( !simulation.IsValid( data.id ), "old contact resurrected" );
    check( contacts[0].manifold.points[0].normalImpulse == 0.0f && contacts[0].manifold.points[0].tangentImpulse == 0.0f, "new contact impulse reset" );
    simulation.SetBodyLinearVelocity( dynamicBody, { 3.0f, 4.0f } );
    simulation.ApplyForceToCenter( dynamicBody, { 10.0f, 20.0f } );
    simulation.DestroyBody( dynamicBody );
    check( !simulation.IsValid( dynamicBody ) && !simulation.IsValid( reusedShape ) && !simulation.IsValid( reusedContact ), "body cascade invalidation" );
    check( simulation.GetBodyCount() == 1 && simulation.GetShapeCount() == 1 && simulation.GetContactCount() == 0, "cascade counts" );
    check( simulation.GetBroadPhase().GetTree( bodyType::Dynamic ).GetProxyCount() == 0, "cascade proxies" );
    const bodyId reusedBody = simulation.CreateBody( bodyType::Dynamic );
    check( reusedBody.index1 == dynamicBody.index1 && reusedBody.generation != dynamicBody.generation, "body generation reuse" );
    check( LengthSquared( simulation.GetBodyLinearVelocity( reusedBody ) ) == 0.0f && LengthSquared( simulation.GetBodyTransform( reusedBody ).position ) == 0.0f, "body state reset" );
    simulation.SetGravity( {} );
    simulation.Step( 0.01f );
    check( LengthSquared( simulation.GetBodyLinearVelocity( reusedBody ) ) == 0.0f, "old body force retained" );
}

void checkFilterAndQueries()
{
    world simulation{};
    simulation.SetContactRecycleDistance( 0.0f );
    const bodyId staticBody = simulation.CreateBody();
    ( void )simulation.CreateShape( staticBody, circle2{ {}, 1.0f } );
    const bodyId dynamicBody = simulation.CreateBody( bodyType::Dynamic, { { 2.01f, 0.0f }, {} } );
    const shapeId dynamicShape = simulation.CreateShape( dynamicBody, circle2{ {}, 1.0f } );
    update( simulation );
    std::array<contactData, 1> contacts{};
    // Box2D의 touching flag는 pointCount > 0이므로 양의 separation도 public query에 포함함.
    check( simulation.GetBodyContactData( dynamicBody, contacts ) == 1 && contacts[0].manifold.points[0].separation > 0.0f, "speculative body query" );
    check( simulation.GetShapeContactData( dynamicShape, contacts ) == 1, "speculative shape query" );
    simulation.SetBodyTransform( dynamicBody, { { 2.06f, 0.0f }, {} } );
    update( simulation );
    check( simulation.GetContactCount() == 1 && simulation.GetBodyContactData( dynamicBody, contacts ) == 0 && simulation.GetShapeContactData( dynamicShape, contacts ) == 0, "zero-point persistent query" );
    simulation.SetBodyTransform( dynamicBody, { { 2.01f, 0.0f }, {} } );
    update( simulation );
    check( simulation.GetBodyContactData( dynamicBody, contacts ) == 1, "speculative transition" );
    const contactId oldContact = contacts[0].id;
    collisionFilter filter{};
    filter.maskBits = 0;
    simulation.SetShapeFilter( dynamicShape, filter );
    check( !simulation.IsValid( oldContact ) && simulation.GetContactCount() == 0, "filter removes contact immediately" );
    update( simulation );
    check( simulation.GetContactCount() == 0, "blocked pair recreated" );
    simulation.SetShapeFilter( dynamicShape, {} );
    update( simulation );
    check( simulation.GetShapeContactData( dynamicShape, contacts ) == 1, "stationary pair refilter" );
    check( !simulation.IsValid( oldContact ), "contact generation reuse" );
    simulation.SetBodyTransform( dynamicBody, { { 20.0f, 0.0f }, {} } );
    update( simulation );
    check( simulation.GetContactCount() == 0 && simulation.GetBodyContactData( dynamicBody, contacts ) == 0, "transform removes pair" );
    simulation.SetBodyTransform( dynamicBody, {} );
    update( simulation );
    check( simulation.GetBodyContactData( dynamicBody, contacts ) == 1, "transform recreates pair" );
}

void checkEdgeRemovalOrders()
{
    std::array<int, 3> order{ 0, 1, 2 };
    do
    {
        world simulation{};
        const bodyId dynamicBody = simulation.CreateBody( bodyType::Dynamic );
        const bodyId staticBody = simulation.CreateBody();
        const shapeId staticShape = simulation.CreateShape( staticBody, circle2{ {}, 1.0f } );
        for( int i = 0; i < 3; ++i )
            ( void )simulation.CreateShape( dynamicBody, circle2{ {}, 1.0f } );
        update( simulation );
        std::array<contactData, 3> contacts{};
        check( simulation.GetBodyContactData( dynamicBody, contacts ) == 3, "edge order fixture" );
        for( int removed = 0; removed < 3; ++removed )
        {
            const contactData& data = contacts[order[removed]];
            simulation.DestroyShape( data.shapeA == staticShape ? data.shapeB : data.shapeA );
            const int remaining = 2 - removed;
            check( simulation.GetBody( dynamicBody ).contactCount == remaining && simulation.GetBody( staticBody ).contactCount == remaining, "both edge counts" );
            std::array<contactData, 3> survivors{};
            check( simulation.GetBodyContactData( dynamicBody, survivors ) == remaining && simulation.GetBodyContactData( staticBody, survivors ) == remaining, "both edge lists" );
            for( int i = removed + 1; i < 3; ++i )
                check( simulation.IsValid( contacts[order[i]].id ), "unrelated contact removed" );
        }
    } while( std::next_permutation( order.begin(), order.end() ) );
}

void checkImpulseReset()
{
    world simulation{};
    simulation.SetGravity( {} );
    const bodyId staticBody = simulation.CreateBody();
    ( void )simulation.CreateShape( staticBody, circle2{ {}, 1.0f } );
    const bodyId dynamicBody = simulation.CreateBody( bodyType::Dynamic, { { 1.99f, 0.0f }, {} } );
    const shapeId oldShape = simulation.CreateShape( dynamicBody, circle2{ {}, 1.0f } );
    simulation.SetBodyLinearVelocity( dynamicBody, { -2.0f, 0.0f } );
    simulation.Step( 0.001f );
    std::array<contactData, 1> contacts{};
    check( simulation.GetBodyContactData( dynamicBody, contacts ) == 1 && contacts[0].manifold.points[0].normalImpulse > 0.0f, "nonzero impulse fixture" );
    const contactId oldContact = contacts[0].id;
    simulation.DestroyShape( oldShape );
    ( void )simulation.CreateShape( dynamicBody, circle2{ {}, 1.0f } );
    update( simulation );
    check( simulation.GetBodyContactData( dynamicBody, contacts ) == 1 && contacts[0].id.index1 == oldContact.index1, "contact slot reused" );
    check( !simulation.IsValid( oldContact ) && contacts[0].manifold.points[0].normalImpulse == 0.0f, "old impulse leaked into new contact" );
}

void checkSensorHandles()
{
    world simulation{};
    simulation.SetGravity( {} );
    const bodyId staticBody = simulation.CreateBody();
    const shapeId sensor = simulation.CreateSensorShape( staticBody, circle2{ {}, 2.0f } );
    const bodyId dynamicBody = simulation.CreateBody( bodyType::Dynamic );
    const shapeId visitor = simulation.CreateShape( dynamicBody, circle2{ {}, 1.0f } );
    simulation.SetShapeSensorEventsEnabled( sensor, true );
    simulation.SetShapeSensorEventsEnabled( visitor, true );
    simulation.Step( 0.001f );
    std::array<shapeId, 1> overlaps{};
    check( simulation.GetShapeSensorData( sensor, overlaps ) == 1 && overlaps[0] == visitor, "sensor query handle identity" );
    check( simulation.GetSensorBeginEvents().size() == 1 && simulation.GetSensorBeginEvents()[0].visitorShapeId == visitor, "sensor begin handle identity" );
    simulation.DestroyShape( visitor );
    simulation.Step( 0.001f );
    check( simulation.GetSensorEndEvents().size() == 1 && simulation.GetSensorEndEvents()[0].visitorShapeId == visitor && !simulation.IsValid( visitor ), "sensor history retains old identity" );
    check( simulation.GetShapeSensorData( sensor, overlaps ) == 0, "destroyed visitor overlap removed" );
}

void checkOrphanChainSegmentUpdate()
{
    world simulation{};
    const bodyId terrainBody = simulation.CreateBody();

    const chainSegment2 original{
        { 2.0f, 0.0f },
        { { 1.0f, 0.0f }, { -1.0f, 0.0f } },
        { -2.0f, 0.0f }
    };
    const shapeId terrainShape = simulation.CreateShape( terrainBody, original, {}, 0.0f );
    simulation.SetShapeFriction( terrainShape, 0.25f );
    simulation.SetShapeRestitution( terrainShape, 0.4f );

    const bodyId circleBody = simulation.CreateBody( bodyType::Dynamic, { { 0.0f, 0.5f }, {} } );
    const shapeId circleShape = simulation.CreateShape( circleBody, circle2{ {}, 0.5f } );

    update( simulation );
    std::array<contactData, 1> contacts{};
    check( simulation.GetShapeContactData( circleShape, contacts ) == 1, "orphan chain initial contact" );
    const contactId oldContact = contacts[0].id;

    const chainSegment2 moved{
        { 2.0f, -2.0f },
        { { 1.0f, -2.0f }, { -1.0f, -2.0f } },
        { -2.0f, -2.0f }
    };

    simulation.SetChainSegment( terrainShape, moved );

    check( simulation.IsValid( terrainShape ), "orphan chain handle changed" );
    check( !simulation.IsValid( oldContact ) && simulation.GetContactCount() == 0, "orphan chain update kept stale contact" );
    check( simulation.GetShapeFriction( terrainShape ) == 0.25f && simulation.GetShapeRestitution( terrainShape ) == 0.4f, "orphan chain update changed material" );

    const chainSegment2& stored = std::get<chainSegment2>( simulation.GetShape( terrainShape ).geometry );
    check( stored.ghost1.x == moved.ghost1.x && stored.ghost1.y == moved.ghost1.y && stored.segment.a.y == -2.0f && stored.segment.b.y == -2.0f && stored.ghost2.x == moved.ghost2.x && stored.ghost2.y == moved.ghost2.y, "orphan chain geometry not replaced" );

    const aabb2& movedAABB = simulation.GetShapeAABB( terrainShape );
    check( movedAABB.max.y < -1.9f && movedAABB.min.y < -2.0f, "orphan chain AABB not updated" );

    simulation.SetBodyTransform( circleBody, { { 0.0f, -1.5f }, {} } );
    update( simulation );
    check( simulation.GetShapeContactData( circleShape, contacts ) == 1, "orphan chain moved contact not recreated" );
    check( contacts[0].shapeA == terrainShape || contacts[0].shapeB == terrainShape, "orphan chain moved contact lost shape identity" );
    check( !simulation.IsValid( oldContact ), "orphan chain old contact resurrected" );

    // Geometry가 body local center에서 멀어지면 CCD / sleep에서 쓰는 maxExtent도 즉시 갱신되어야 함.
    world extentSimulation{};
    bodyDef extentBodyDefinition{};
    extentBodyDefinition.type = bodyType::Dynamic;
    extentBodyDefinition.gravityScale = 0.0f;
    extentBodyDefinition.angularVelocity = 0.01f;
    extentBodyDefinition.sleepThreshold = 0.05f;
    const bodyId extentBody = extentSimulation.CreateBody( extentBodyDefinition );

    const chainSegment2 nearCenter{
        { 0.2f, 0.0f },
        { { 0.1f, 0.0f }, { -0.1f, 0.0f } },
        { -0.2f, 0.0f }
    };
    const shapeId extentShape = extentSimulation.CreateShape( extentBody, nearCenter, {}, 0.0f );

    const chainSegment2 farFromCenter{
        { 11.2f, 0.0f },
        { { 11.1f, 0.0f }, { 10.9f, 0.0f } },
        { 10.8f, 0.0f }
    };
    extentSimulation.SetChainSegment( extentShape, farFromCenter );

    for( int i = 0; i < 40; ++i )
    {
        extentSimulation.Step( 1.0f / 60.0f, 1 );
    }

    check( extentSimulation.IsBodyAwake( extentBody ), "orphan chain update kept stale body extent" );

    const std::array<vec2, 2> points{ vec2{ 1.0f, 2.0f }, vec2{ -1.0f, 2.0f } };
    chainDef definition{};
    definition.points = points;
    definition.ghost1 = { 2.0f, 2.0f };
    definition.ghost2 = { -2.0f, 2.0f };
    const chainId parent = simulation.CreateChain( terrainBody, definition );
    std::array<shapeId, 1> children{};
    check( simulation.GetChainSegments( parent, children ) == 1, "parent chain child fixture" );

#ifdef NDEBUG
    const chainSegment2 before = std::get<chainSegment2>( simulation.GetShape( children[0] ).geometry );
    simulation.SetChainSegment( children[0], moved );
    const chainSegment2 after = std::get<chainSegment2>( simulation.GetShape( children[0] ).geometry );
    check( after.ghost1.x == before.ghost1.x && after.ghost1.y == before.ghost1.y && after.segment.a.x == before.segment.a.x && after.segment.a.y == before.segment.a.y && after.segment.b.x == before.segment.b.x && after.segment.b.y == before.segment.b.y && after.ghost2.x == before.ghost2.x && after.ghost2.y == before.ghost2.y, "parent chain child geometry mutated" );
#endif
}

void checkChainLifecycle()
{
    world simulation{};
    const bodyId terrainBody = simulation.CreateBody();

    // 현재 Box2D처럼 open chain은 실제 points와 시작/끝 ghost point를 별도로 받음.
    // 실제 point N개는 N-1개의 collision segment를 생성함.
    const std::array<vec2, 3> openPoints{ vec2{ 1.0f, 0.0f }, vec2{ 0.0f, 0.0f }, vec2{ -1.0f, 0.0f } };
    chainDef openDef{};
    openDef.points = openPoints;
    openDef.ghost1 = { 2.0f, 0.0f };
    openDef.ghost2 = { -2.0f, 0.0f };

    const chainId openChain = simulation.CreateChain( terrainBody, openDef );
    check( simulation.IsValid( openChain ), "open chain invalid" );
    check( simulation.GetChainCount() == 1, "open chain count" );
    check( simulation.GetChainSegmentCount( openChain ) == 2, "open chain segment count" );

    std::array<shapeId, 2> openSegments{};
    check( simulation.GetChainSegments( openChain, openSegments ) == 2, "open chain segment query" );
    for( shapeId segment : openSegments )
    {
        check( simulation.IsValid( segment ), "open chain child invalid" );
        check( simulation.GetShapeParentChain( segment ) == openChain, "open chain parent link" );
    }
    check( simulation.GetShapeCount() == 2 && simulation.GetBody( terrainBody ).shapeCount == 2, "open chain child shape count" );

    // point1 -> point2의 오른쪽이 앞면이므로 이 방향에서는 위쪽 circle만 접촉함.
    const bodyId circleBody = simulation.CreateBody( bodyType::Dynamic, { { 0.5f, 0.5f }, {} } );
    const shapeId circleShape = simulation.CreateShape( circleBody, circle2{ {}, 0.5f } );
    update( simulation );
    std::array<contactData, 1> contacts{};
    check( simulation.GetShapeContactData( circleShape, contacts ) == 1, "open chain world contact" );

    simulation.DestroyChain( openChain );
    check( !simulation.IsValid( openChain ) && !simulation.IsValid( openSegments[0] ) && !simulation.IsValid( openSegments[1] ), "destroy chain invalidation" );
    check( simulation.GetChainCount() == 0 && simulation.GetShapeCount() == 1 && simulation.GetContactCount() == 0, "destroy chain cascade counts" );

    const chainId reusedChain = simulation.CreateChain( terrainBody, openDef );
    check( reusedChain.index1 == openChain.index1 && reusedChain.generation != openChain.generation, "chain generation reuse" );

    std::array<shapeId, 2> reusedSegments{};
    check( simulation.GetChainSegments( reusedChain, reusedSegments ) == 2, "reused chain segment query" );

    // Body 파괴는 parent chain과 내부 segment shape를 함께 제거해야 함.
    simulation.DestroyBody( terrainBody );
    check( !simulation.IsValid( reusedChain ) && !simulation.IsValid( reusedSegments[0] ) && !simulation.IsValid( reusedSegments[1] ), "body chain cascade invalidation" );
    check( simulation.GetChainCount() == 0 && simulation.GetShapeCount() == 1, "body chain cascade counts" );

    // Loop chain은 모든 point 사이를 닫아 N개의 실제 segment를 생성함.
    const bodyId loopBody = simulation.CreateBody();
    const std::array<vec2, 4> loopPoints{ vec2{ -2.0f, -1.0f }, vec2{ -2.0f, 1.0f }, vec2{ 2.0f, 1.0f }, vec2{ 2.0f, -1.0f } };
    chainDef loopDef{};
    loopDef.points = loopPoints;
    loopDef.isLoop = true;

    const chainId loopChain = simulation.CreateChain( loopBody, loopDef );
    check( simulation.GetChainSegmentCount( loopChain ) == 4, "loop chain segment count" );
    std::array<shapeId, 4> loopSegments{};
    check( simulation.GetChainSegments( loopChain, loopSegments ) == 4, "loop chain segment query" );
    for( shapeId segment : loopSegments )
    {
        check( simulation.IsValid( segment ), "loop child invalid" );
        check( simulation.GetShapeParentChain( segment ) == loopChain, "loop parent link" );
    }
}

#pragma endregion ShapeAndContactLifecycle

} // namespace

void checkCapsuleCreationBoundary()
{
    world simulation{};
    const bodyId owner = simulation.CreateBody();
    for( float length : { 0.0f, LINEAR_SLOP * 0.5f, LINEAR_SLOP } )
    {
        const capsule2 capsule{ {}, { length, 0.0f }, 1.0f };
        check( !simulation.IsValid( simulation.CreateShape( owner, capsule ) ), "degenerate capsule accepted" );
        check( !simulation.IsValid( simulation.CreateSensorShape( owner, capsule ) ), "degenerate sensor capsule accepted" );
        check( simulation.GetShapeCount() == 0 && simulation.GetBody( owner ).shapeCount == 0, "rejected capsule changed shape lists" );
    }
    const capsule2 capsule{ {}, { LINEAR_SLOP * 2.0f, 0.0f }, 1.0f };
    const shapeId solid = simulation.CreateShape( owner, capsule );
    const shapeId sensor = simulation.CreateSensorShape( owner, capsule );
    check( simulation.IsValid( solid ) && simulation.IsValid( sensor ), "valid capsule rejected" );
    check( simulation.GetShapeCount() == 2 && simulation.GetShape( sensor ).sensorIndex >= 0, "valid capsule creation state" );
    simulation.DestroyBody( owner );
    check( simulation.GetShapeCount() == 0, "capsule cleanup" );
}

void checkFixedRotation()
{
    world simulation{};
    simulation.SetGravity( {} );
    bodyDef definition{};
    definition.type = bodyType::Dynamic;
    definition.fixedRotation = true;
    definition.angularVelocity = 3.0f;
    const bodyId fixed = simulation.CreateBody( definition );
    const shapeId shape = simulation.CreateShape( fixed, circle2{ {}, 0.5f } );
    simulation.SetShapeDensity( shape, 2.0f );
    simulation.SetBodyAngularVelocity( fixed, 4.0f );
    simulation.ApplyAngularImpulse( fixed, 5.0f );
    simulation.ApplyTorque( fixed, 6.0f );
    simulation.ApplyForce( fixed, { 1.0f, 0.0f }, { 0.0f, 1.0f } );
    simulation.Step( 1.0f / 60.0f, 4 );
    const auto rotation = simulation.GetBodyTransform( fixed ).rotation;
    check( rotation.c == 1.0f && rotation.s == 0.0f && simulation.GetBodyAngularVelocity( fixed ) == 0.0f, "fixed rotation must survive mass updates and physical angular inputs" );
    simulation.DestroyBody( fixed );
    definition.fixedRotation = false;
    const bodyId free = simulation.CreateBody( definition );
    (void)simulation.CreateShape( free, circle2{ {}, 0.5f } );
    simulation.Step( 1.0f / 60.0f, 4 );
    check( simulation.GetBodyAngularVelocity( free ) > 0.0f && simulation.GetBodyTransform( free ).rotation.s > 0.0f, "reused body slot must allow default free rotation" );
}

int main()
{
    checkFixedRotation();
    checkCapsuleCreationBoundary();
    checkOwnership();
    checkWorldReuse();
    checkLists();
    checkFilterAndQueries();
    checkEdgeRemovalOrders();
    checkImpulseReset();
    checkSensorHandles();
    checkOrphanChainSegmentUpdate();
    checkChainLifecycle();
}
