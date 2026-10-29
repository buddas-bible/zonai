#include <array>
#include <cstdio>
#include <cstdlib>

#include "dynamics/world.h"

using namespace zonai;

namespace
{

void check( bool ok, const char* message )
{
    if( ok ) return;
    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}

template <typename Event>
bool containsPair( std::span<const Event> events, shapeId a, shapeId b )
{
    for( const Event& event : events )
        if( ( event.shapeA == a && event.shapeB == b ) ||
            ( event.shapeA == b && event.shapeB == a ) ) return true;
    return false;
}

void checkLifecycleAndSpeculative()
{
    world simulation{};
    const bodyId staticBody = simulation.CreateBody();
    const shapeId ground = simulation.CreateShape( staticBody, circle2{ {}, 1.0f } );
    const bodyId dynamicBody = simulation.CreateBody( bodyType::Dynamic, { { 1.5f, 0.0f }, {} } );
    const shapeId moving = simulation.CreateShape( dynamicBody, circle2{ {}, 1.0f } );

    simulation.Step( 0.0f );
    check( simulation.GetContactBeginEvents().empty() && simulation.GetContactEndEvents().empty(),
        "Contact events must be opt-in by default" );

    simulation.SetShapeContactEventsEnabled( ground, true );
    check( simulation.AreShapeContactEventsEnabled( ground ) &&
        !simulation.AreShapeContactEventsEnabled( moving ),
        "Either participating Shape can enable Contact events" );
    simulation.Step( 0.0f );
    check( simulation.GetContactBeginEvents().size() == 1 &&
        containsPair( simulation.GetContactBeginEvents(), ground, moving ) &&
        simulation.GetContactEndEvents().empty(),
        "Enabling on one Shape must report the existing touching pair as Begin" );

    const contactBeginEvent2 copiedBegin = simulation.GetContactBeginEvents()[0];
    simulation.Step( 0.0f );
    check( simulation.GetContactBeginEvents().empty() && simulation.GetContactEndEvents().empty() &&
        containsPair( std::span{ &copiedBegin, 1 }, ground, moving ),
        "Persistent touching must not repeat Begin; copied event is stable" );

    // Contact 자체는 유지돼도 solver가 만든 positive separation은 Begin/End의 touching이 아님.
    simulation.SetBodyTransform( dynamicBody, { { 2.005f, 0.0f }, {} } );
    simulation.Step( 0.0f );
    std::array<contactData, 1> contacts{};
    check( simulation.GetShapeContactData( ground, contacts ) == 1 &&
        contacts[0].manifold.pointCount == 1 &&
        contacts[0].manifold.points[0].separation > 0.0f,
        "Speculative manifold fixture must keep a positive-separation Contact" );
    check( simulation.GetContactBeginEvents().empty() &&
        simulation.GetContactEndEvents().size() == 1 &&
        containsPair( simulation.GetContactEndEvents(), ground, moving ),
        "Moving into speculative-only contact must emit End, not Begin" );

    simulation.Step( 0.0f );
    check( simulation.GetContactBeginEvents().empty() && simulation.GetContactEndEvents().empty(),
        "Speculative-only state must not repeat End" );

    simulation.SetBodyTransform( dynamicBody, { { 1.5f, 0.0f }, {} } );
    simulation.Step( 0.0f );
    check( simulation.GetContactBeginEvents().size() == 1 &&
        containsPair( simulation.GetContactBeginEvents(), ground, moving ),
        "Re-entering geometric contact must emit a new Begin" );

    simulation.SetShapeContactEventsEnabled( ground, false );
    simulation.Step( 0.0f );
    check( simulation.GetContactEndEvents().size() == 1 &&
        containsPair( simulation.GetContactEndEvents(), ground, moving ),
        "Opting out of the last subscribed Shape must end the published touch" );

    simulation.SetShapeContactEventsEnabled( moving, true );
    simulation.Step( 0.0f );
    check( simulation.GetContactBeginEvents().size() == 1 &&
        containsPair( simulation.GetContactBeginEvents(), ground, moving ),
        "Subscription on the other Shape must also report Begin" );

    simulation.SetShapeFilter( moving, collisionFilter{ 1, 0, 0 } );
    check( simulation.GetContactBeginEvents().size() == 1,
        "Contact mutation must not invalidate the previously published event span" );
    simulation.Step( 0.0f );
    check( simulation.GetContactEndEvents().size() == 1 &&
        containsPair( simulation.GetContactEndEvents(), ground, moving ) &&
        simulation.GetContactBeginEvents().empty(),
        "Refilter must keep a pending End for the destroyed Contact" );

    simulation.SetShapeFilter( moving, collisionFilter{} );
    simulation.Step( 0.0f );
    check( simulation.GetContactBeginEvents().size() == 1 &&
        containsPair( simulation.GetContactBeginEvents(), ground, moving ),
        "Restoring filter must rediscover a stationary touching pair" );
}

void checkShapeAndBodyReuse()
{
    world simulation{};
    const bodyId fixed = simulation.CreateBody();
    const shapeId original = simulation.CreateShape( fixed, circle2{ {}, 1.0f } );
    const bodyId mover = simulation.CreateBody( bodyType::Dynamic, { { 1.5f, 0.0f }, {} } );
    const shapeId visitor = simulation.CreateShape( mover, circle2{ {}, 1.0f } );
    simulation.SetShapeContactEventsEnabled( original, true );
    simulation.Step( 0.0f );
    check( simulation.GetContactBeginEvents().size() == 1, "Initial pair must begin" );

    simulation.DestroyShape( original );
    const shapeId replacement = simulation.CreateShape( fixed, circle2{ {}, 1.0f } );
    simulation.SetShapeContactEventsEnabled( replacement, true );
    check( original.index1 == replacement.index1 && original.generation != replacement.generation &&
        !simulation.IsValid( original ), "Shape slot reuse must increment generation" );
    simulation.Step( 0.0f );

    check( simulation.GetContactEndEvents().size() == 1 &&
        containsPair( simulation.GetContactEndEvents(), original, visitor ) &&
        !simulation.IsValid( original ),
        "End event must preserve the destroyed Shape generation" );
    check( simulation.GetContactBeginEvents().size() == 1 &&
        containsPair( simulation.GetContactBeginEvents(), replacement, visitor ),
        "Recycled Shape slot must be a new Begin rather than a continuation" );

    simulation.DestroyBody( mover );
    simulation.Step( 0.0f );
    check( simulation.GetContactEndEvents().size() == 1 &&
        containsPair( simulation.GetContactEndEvents(), replacement, visitor ) &&
        !simulation.IsValid( visitor ),
        "DestroyBody must preserve a pending End for the removed visitor" );

    simulation.Step( 0.0f );
    check( simulation.GetContactBeginEvents().empty() && simulation.GetContactEndEvents().empty(),
        "Destruction End must only appear once" );
}

void checkSensorIsolation()
{
    world simulation{};
    const bodyId owner = simulation.CreateBody();
    const shapeId sensor = simulation.CreateSensorShape( owner, circle2{ {}, 1.0f } );
    const bodyId visitorBody = simulation.CreateBody( bodyType::Dynamic, { { 1.0f, 0.0f }, {} } );
    const shapeId visitor = simulation.CreateShape( visitorBody, circle2{ {}, 1.0f } );
    simulation.SetShapeContactEventsEnabled( sensor, true );
    simulation.SetShapeContactEventsEnabled( visitor, true );
    simulation.Step( 0.0f );
    check( simulation.GetContactBeginEvents().empty() && simulation.GetContactEndEvents().empty() &&
        simulation.GetContactCount() == 0,
        "Sensor overlap must never create solid Contact events" );
}

} // namespace

int main()
{
    checkLifecycleAndSpeculative();
    checkShapeAndBodyReuse();
    checkSensorIsolation();
}
