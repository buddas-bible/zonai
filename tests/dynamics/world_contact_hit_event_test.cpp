#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "dynamics/world.h"

using namespace zonai;

namespace
{

void check( bool condition, const char* message )
{
    if( condition ) return;

    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}

struct hitFixture
{
    world simulation{};
    bodyId movingBody{};
    shapeId staticShape{};
    shapeId movingShape{};

    hitFixture()
    {
        simulation.SetGravity( {} );

        const bodyId staticBody = simulation.CreateBody();
        staticShape = simulation.CreateShape( staticBody, circle2{ {}, 1.0f } );

        movingBody = simulation.CreateBody( bodyType::Dynamic, { { 1.5f, 0.0f }, {} } );
        movingShape = simulation.CreateShape( movingBody, circle2{ {}, 1.0f } );
    }

    void collide( float speed )
    {
        simulation.SetBodyTransform( movingBody, { { 1.5f, 0.0f }, {} } );
        simulation.SetBodyLinearVelocity( movingBody, { -speed, 0.0f } );
        simulation.Step( 1.0f / 60.0f );
    }
};

void checkHitOptInAndImpulse()
{
    hitFixture scene{};
    world& simulation = scene.simulation;

    check( !simulation.areShapeHitEventsEnabled( scene.staticShape ) &&
        !simulation.areShapeHitEventsEnabled( scene.movingShape ) &&
        simulation.getContactHitSpeedThreshold() == 1.0f,
        "Hit events must be opt-in with a documented default speed threshold" );

    scene.collide( 5.0f );
    check( simulation.getContactHitEvents().empty(), "Disabled Hit events must not be published" );

    simulation.setShapeHitEventsEnabled( scene.staticShape, true );
    check( simulation.areShapeHitEventsEnabled( scene.staticShape ) &&
        !simulation.areShapeHitEventsEnabled( scene.movingShape ),
        "Either shape must be able to subscribe independently" );

    scene.collide( 5.0f );
    check( simulation.getContactHitEvents().size() == 1,
        "A high-speed collision must publish one strongest Hit per Contact" );

    const contactHitEvent2 event = simulation.getContactHitEvents()[0];
    check( simulation.IsValid( event.contact ) &&
        ( event.shapeA == scene.staticShape && event.shapeB == scene.movingShape ||
            event.shapeA == scene.movingShape && event.shapeB == scene.staticShape ),
        "Hit must reference the live Contact and its two Shape handles" );
    check( std::isfinite( event.point.x ) && std::isfinite( event.point.y ) &&
        std::isfinite( event.normal.x ) && std::isfinite( event.normal.y ) &&
        std::fabs( LengthSquared( event.normal ) - 1.0f ) < 0.01f,
        "Hit must report a world-space point and unit normal" );
    check( event.approachSpeed > 4.5f && event.approachSpeed < 5.5f &&
        event.normalImpulse > 0.0f,
        "Hit speed and normal impulse must come from the actual Contact constraint" );
    check( simulation.GetContactBeginEvents().empty() &&
        simulation.GetContactEndEvents().empty(),
        "Hit subscription must not implicitly enable Begin/End events" );

    simulation.Step( 0.0f );
    check( simulation.getContactHitEvents().empty() &&
        event.approachSpeed > 4.5f && event.normalImpulse > 0.0f,
        "Step(0) must clear transient Hit output while copied events remain valid" );

    simulation.setShapeHitEventsEnabled( scene.staticShape, false );
    simulation.setShapeHitEventsEnabled( scene.movingShape, true );
    scene.collide( 5.0f );
    check( simulation.getContactHitEvents().size() == 1,
        "Subscription on the other Shape must also report a high-speed Hit" );

    simulation.DestroyBody( scene.movingBody );
    check( simulation.getContactHitEvents().size() == 1,
        "World mutations between Steps must not rewrite the last published event" );
    simulation.Step( 0.0f );
    check( simulation.getContactHitEvents().empty() && !simulation.IsValid( event.contact ),
        "Destroyed Contacts must not be reported by later Steps" );
}

void checkHitThreshold()
{
    hitFixture scene{};
    world& simulation = scene.simulation;
    simulation.setShapeHitEventsEnabled( scene.movingShape, true );

    simulation.setContactHitSpeedThreshold( 10.0f );
    check( simulation.getContactHitSpeedThreshold() == 10.0f, "Hit speed threshold setter/getter" );
    scene.collide( 5.0f );
    check( simulation.getContactHitEvents().empty(), "Below-threshold collision must not report Hit" );

    simulation.setContactHitSpeedThreshold( 1.0f );
    scene.collide( 0.25f );
    check( simulation.getContactHitEvents().empty(), "Resting and low-speed contacts must not report Hit" );

    scene.collide( 5.0f );
    check( simulation.getContactHitEvents().size() == 1,
        "A later high-speed impact on the same persistent pair must report Hit" );

    simulation.setShapeHitEventsEnabled( scene.movingShape, false );
    scene.collide( 5.0f );
    check( simulation.getContactHitEvents().empty(), "Disabling Hit events must stop reports" );
}

void checkSensorIsolation()
{
    world simulation{};
    simulation.SetGravity( {} );

    const bodyId staticBody = simulation.CreateBody();
    const shapeId sensor = simulation.CreateSensorShape( staticBody, circle2{ {}, 1.0f } );
    const bodyId body = simulation.CreateBody( bodyType::Dynamic, { { 1.5f, 0.0f }, {} } );
    const shapeId moving = simulation.CreateShape( body, circle2{ {}, 1.0f } );

    simulation.setShapeHitEventsEnabled( sensor, true );
    simulation.setShapeHitEventsEnabled( moving, true );
    simulation.SetBodyLinearVelocity( body, { -5.0f, 0.0f } );
    simulation.Step( 1.0f / 60.0f );

    check( simulation.getContactHitEvents().empty() && simulation.GetContactCount() == 0,
        "Sensor overlap must not generate solid Contact hit events" );
}

} // namespace

int main()
{
    checkHitOptInAndImpulse();
    checkHitThreshold();
    checkSensorIsolation();
}
