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
    simulation.UpdateCollisions( []( const contactData& ) {} );
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
    (void)first.CreateShape( firstStatic, circle2{ {}, 1.0f } );
    (void)second.CreateShape( secondStatic, circle2{ {}, 1.0f } );
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
    (void)first->CreateShape( staticBody, circle2{ {}, 1.0f } );
    update( *first );
    std::array<contactData, 1> contacts{};
    check( first->GetBodyContactData( oldBody, contacts ) == 1, "world reuse contact fixture" );
    const contactId oldContact = contacts[0].id;
    std::destroy_at( first );
    world* second = std::construct_at( reinterpret_cast<world*>( storage ) );
    const bodyId newBody = second->CreateBody( bodyType::Dynamic );
    (void)second->CreateShape( newBody, circle2{ {}, 1.0f } );
    const bodyId newStatic = second->CreateBody();
    (void)second->CreateShape( newStatic, circle2{ {}, 1.0f } );
    update( *second );
    check( second->GetBodyContactData( newBody, contacts ) == 1, "new world contact fixture" );
    check( oldBody.index1 == newBody.index1 && oldContact.index1 == contacts[0].id.index1, "world reuse slots" );
    check( !second->IsValid( oldBody ) && !second->IsValid( oldShape ) && !second->IsValid( oldContact ), "previous world handle resurrected" );
    std::destroy_at( second );
}

#pragma endregion

#pragma region ShapeAndContactLifecycle

void checkLists()
{
    // 같은 geometry 세 개로 양쪽 body의 contact edge와 shape list를 함께 검사함.
    world simulation{};
    const bodyId dynamicBody = simulation.CreateBody( bodyType::Dynamic );
    const bodyId staticBody = simulation.CreateBody();
    (void)simulation.CreateShape( staticBody, circle2{ {}, 1.0f } );
    std::array<shapeId, 3> shapes{};
    for( shapeId& id : shapes ) id = simulation.CreateShape( dynamicBody, circle2{ {}, 1.0f } );
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
    for( const auto& data : oldContacts ) check( !simulation.IsValid( data.id ), "destroyed contact valid" );
    const shapeId reusedShape = simulation.CreateShape( dynamicBody, circle2{ {}, 1.0f } );
    check( reusedShape.index1 == shapes[0].index1 && reusedShape.generation != shapes[0].generation, "shape generation reuse" );
    update( simulation );
    check( simulation.GetBodyContactData( dynamicBody, contacts ) == 1, "pair can be recreated" );
    const contactId reusedContact = contacts[0].id;
    for( const auto& data : oldContacts ) check( !simulation.IsValid( data.id ), "old contact resurrected" );
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
    (void)simulation.CreateShape( staticBody, circle2{ {}, 1.0f } );
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
        for( int i = 0; i < 3; ++i ) (void)simulation.CreateShape( dynamicBody, circle2{ {}, 1.0f } );
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
            for( int i = removed + 1; i < 3; ++i ) check( simulation.IsValid( contacts[order[i]].id ), "unrelated contact removed" );
        }
    } while( std::next_permutation( order.begin(), order.end() ) );
}

void checkImpulseReset()
{
    world simulation{};
    simulation.SetGravity( {} );
    const bodyId staticBody = simulation.CreateBody();
    (void)simulation.CreateShape( staticBody, circle2{ {}, 1.0f } );
    const bodyId dynamicBody = simulation.CreateBody( bodyType::Dynamic, { { 1.99f, 0.0f }, {} } );
    const shapeId oldShape = simulation.CreateShape( dynamicBody, circle2{ {}, 1.0f } );
    simulation.SetBodyLinearVelocity( dynamicBody, { -2.0f, 0.0f } );
    simulation.Step( 0.001f );
    std::array<contactData, 1> contacts{};
    check( simulation.GetBodyContactData( dynamicBody, contacts ) == 1 && contacts[0].manifold.points[0].normalImpulse > 0.0f, "nonzero impulse fixture" );
    const contactId oldContact = contacts[0].id;
    simulation.DestroyShape( oldShape );
    (void)simulation.CreateShape( dynamicBody, circle2{ {}, 1.0f } );
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

#pragma endregion

} // namespace

int main()
{
    checkOwnership();
    checkWorldReuse();
    checkLists();
    checkFilterAndQueries();
    checkEdgeRemovalOrders();
    checkImpulseReset();
    checkSensorHandles();
}
