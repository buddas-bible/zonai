#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numbers>

#include "dynamics/world.h"
#include "geometry/circle2.h"
#include "geometry/polygon2.h"

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
    simulation.UpdateCollisions(
        []( const contactData& )
        {
        } );
}

contactData getContact( world& simulation, bodyId owner )
{
    std::array<contactData, 1> output{};
    check( simulation.GetBodyContactData( owner, output ) == 1, "active contact fixture" );

    return output[0];
}

#pragma region RecyclingEligibility

void checkDistanceThreshold()
{
    world simulation{};
    const bodyId fixed = simulation.CreateBody();
    ( void )simulation.CreateShape( fixed, circle2{ {}, 1.0f } );
    const bodyId moving = simulation.CreateBody( bodyType::Dynamic, { { 2.0f, 0.0f }, {} } );
    ( void )simulation.CreateShape( moving, circle2{ {}, 1.0f } );
    simulation.SetContactRecycleDistance( 0.03125f );
    update( simulation );
    const contactId id = getContact( simulation, moving ).id;
    check( simulation.GetRecycledContactCount() == 0, "new contact recycled uninitialized cache" );
    for( const float offset : { 0.015625f, 0.0234375f } )
    {
        simulation.SetBodyTransform( moving, { { 2.0f + offset, 0.0f }, {} } );
        update( simulation );
        const contactData data = simulation.GetContactData( id );
        check( simulation.GetRecycledContactCount() == 1 && data.manifold.pointCount == 1, "small cumulative motion not recycled" );
        check( std::fabs( data.manifold.points[0].separation - offset ) < 1e-6f, "recycled separation drifted from fresh anchors" );
        check( std::fabs( data.manifold.points[0].point.x - ( 1.0f + 0.5f * offset ) ) < 1e-6f, "recycled midpoint not updated" );
    }
    simulation.SetBodyTransform( moving, { { 2.03125f, 0.0f }, {} } );
    update( simulation );
    check( simulation.GetRecycledContactCount() == 0 && simulation.GetContactData( id ).manifold.pointCount == 0, "threshold equality or accumulated motion recycled" );
    simulation.SetContactRecycleDistance( 0.0f );
    update( simulation );
    check( simulation.GetRecycledContactCount() == 0, "zero distance did not disable recycling" );
}

void checkEmptyTolerance()
{
    world simulation{};
    const bodyId fixed = simulation.CreateBody();
    ( void )simulation.CreateShape( fixed, circle2{ {}, 1.0f } );
    const bodyId moving = simulation.CreateBody( bodyType::Dynamic, { { 2.0625f, 0.0f }, {} } );
    ( void )simulation.CreateShape( moving, circle2{ {}, 1.0f } );
    simulation.SetContactRecycleDistance( 0.25f );
    update( simulation );
    check( simulation.GetContactCount() == 1 && simulation.GetBodyContactCapacity( moving ) == 1, "empty persistent pair fixture" );
    simulation.SetBodyTransform( moving, { { 2.046875f, 0.0f }, {} } );
    update( simulation );
    check( simulation.GetRecycledContactCount() == 1, "small empty manifold motion not recycled" );
    simulation.SetBodyTransform( moving, { { 2.0f, 0.0f }, {} } );
    update( simulation );
    check( simulation.GetRecycledContactCount() == 0 && getContact( simulation, moving ).manifold.pointCount == 1, "empty manifold ignored speculative tolerance" );
}

void checkRotationGuards()
{
    for( const bool rotateTogether : { false, true } )
    {
        world simulation{};
        const bodyId fixed = simulation.CreateBody();
        ( void )simulation.CreateShape( fixed, circle2{ {}, 1.0f } );
        const bodyId moving = simulation.CreateBody( bodyType::Dynamic, { { 2.0f, 0.0f }, {} } );
        ( void )simulation.CreateShape( moving, circle2{ {}, 1.0f } );
        update( simulation );
        const rot2 small = rot2::FromRadians( 0.01f );
        if( rotateTogether )
        {
            simulation.SetBodyTransform( fixed, { {}, small } );
        }
        simulation.SetBodyTransform( moving, { rotateTogether ? Rotate( small, { 2.0f, 0.0f } ) : vec2{ 2.0f, 0.0f }, small } );
        update( simulation );
        check( simulation.GetRecycledContactCount() == 1, "small body rotation not recycled" );
        const rot2 large = rot2::FromRadians( rotateTogether ? 0.25f : 0.06f );
        if( rotateTogether )
        {
            simulation.SetBodyTransform( fixed, { {}, large } );
        }
        simulation.SetBodyTransform( moving, { rotateTogether ? Rotate( large, { 2.0f, 0.0f } ) : vec2{ 2.0f, 0.0f }, large } );
        update( simulation );
        check( simulation.GetRecycledContactCount() == 0, "absolute or extent-scaled rotation guard missed" );
    }
}

void checkCreationSettings()
{
    for( const bool initiallyEnabled : { false, true } )
    {
        world simulation{};
        const bodyId fixed = simulation.CreateBody();
        ( void )simulation.CreateShape( fixed, circle2{ {}, 1.0f } );
        const bodyId moving = simulation.CreateBody( bodyType::Dynamic, { { 2.0f, 0.0f }, {} } );
        simulation.SetBodyContactRecyclingEnabled( moving, initiallyEnabled );
        const shapeId visitor = simulation.CreateShape( moving, circle2{ {}, 1.0f } );
        update( simulation );
        const contactId old = getContact( simulation, moving ).id;
        simulation.SetBodyContactRecyclingEnabled( moving, !initiallyEnabled );
        update( simulation );
        check( simulation.GetRecycledContactCount() == ( initiallyEnabled ? 1u : 0u ), "body recycling toggle altered existing pair" );
        simulation.SetShapeFilter( visitor, { 1, 0, 0 } );
        simulation.SetShapeFilter( visitor, {} );
        update( simulation );
        check( !simulation.IsValid( old ) && getContact( simulation, moving ).id.generation != old.generation, "refilter retained old cache identity" );
        update( simulation );
        check( simulation.GetRecycledContactCount() == ( initiallyEnabled ? 0u : 1u ), "new pair ignored updated recycling setting" );
    }
}

void checkFastExclusion()
{
    world simulation{};
    simulation.SetGravity( {} );
    const bodyId fixed = simulation.CreateBody();
    ( void )simulation.CreateShape( fixed, circle2{ {}, 1.0f } );
    const bodyId moving = simulation.CreateBody( bodyType::Dynamic, { { -4.0f, 0.0f }, {} } );
    ( void )simulation.CreateShape( moving, circle2{ {}, 1.0f } );
    update( simulation );
    simulation.SetContactRecycleDistance( 100.0f );
    simulation.SetBodyLinearVelocity( moving, { 100.0f, 0.0f } );
    simulation.Step( 0.02f );
    check( simulation.IsBodyFast( moving ), "fast contact fixture" );
    check( simulation.GetBodyContactCapacity( moving ) == 1, "fast persistent contact fixture" );
    update( simulation );
    check( simulation.GetRecycledContactCount() == 0, "fast body recycled contact" );
}

#pragma endregion RecyclingEligibility
#pragma region ImpulsePersistence

void checkImpulsePersistence()
{
    world simulation{};
    simulation.SetGravity( {} );
    const bodyId fixed = simulation.CreateBody();
    const shapeId ground = simulation.CreateShape( fixed, MakeBox( { 2.0f, 0.5f } ) );
    const bodyId moving = simulation.CreateBody( bodyType::Dynamic, { { 0.0f, 1.49f }, {} } );
    const shapeId ball = simulation.CreateShape( moving, circle2{ {}, 1.0f } );
    simulation.SetBodyLinearVelocity( moving, { 0.5f, -2.0f } );
    simulation.Step( 0.001f );
    const contactData initial = getContact( simulation, moving );
    check( initial.manifold.points[0].normalImpulse > 0.0f && std::fabs( initial.manifold.points[0].tangentImpulse ) > 0.0f, "nonzero cached impulses fixture" );
    update( simulation );
    const contactData recycled = getContact( simulation, moving );
    check( simulation.GetRecycledContactCount() == 1 && recycled.manifold.points[0].normalImpulse == initial.manifold.points[0].normalImpulse && recycled.manifold.points[0].tangentImpulse == initial.manifold.points[0].tangentImpulse, "recycling discarded impulses" );
    simulation.SetContactRecycleDistance( 0.0f );
    update( simulation );
    const contactData fresh = getContact( simulation, moving );
    check( fresh.manifold.points[0].id == initial.manifold.points[0].id && fresh.manifold.points[0].normalImpulse == initial.manifold.points[0].normalImpulse && fresh.manifold.points[0].tangentImpulse == initial.manifold.points[0].tangentImpulse, "fresh matching feature discarded impulses" );
    simulation.SetShapeFilter( ball, { 1, 0, 0 } );
    simulation.SetShapeFilter( ball, {} );
    update( simulation );
    const contactData recreated = getContact( simulation, moving );
    check( !simulation.IsValid( initial.id ) && recreated.manifold.points[0].normalImpulse == 0.0f, "new contact inherited old cache" );
    check( recreated.shapeA == ground || recreated.shapeB == ground, "cache pair fixture" );
}

void checkFeatureReset()
{
    world simulation{};
    simulation.SetGravity( {} );
    simulation.SetContactRecycleDistance( 0.0f );
    const bodyId fixed = simulation.CreateBody();
    ( void )simulation.CreateShape( fixed, MakeBox( { 2.0f, 0.5f } ) );
    const bodyId moving = simulation.CreateBody( bodyType::Dynamic, { { 0.0f, 1.49f }, {} } );
    ( void )simulation.CreateShape( moving, MakeBox( { 1.0f, 1.0f } ) );
    simulation.SetBodyLinearVelocity( moving, { 0.0f, -2.0f } );
    simulation.Step( 0.001f );
    const contactData old = getContact( simulation, moving );
    check( old.manifold.pointCount == 2 && old.manifold.points[0].normalImpulse > 0.0f && old.manifold.points[1].normalImpulse > 0.0f, "two nonzero polygon impulses fixture" );
    simulation.SetBodyTransform( moving, { { 2.99f, 0.0f }, {} } );
    update( simulation );
    const contactData changed = getContact( simulation, moving );
    check( changed.id == old.id && changed.manifold.pointCount == 2, "polygon feature transition fixture" );
    for( int i = 0; i < 2; ++i )
    {
        check( changed.manifold.points[i].id != old.manifold.points[0].id && changed.manifold.points[i].id != old.manifold.points[1].id, "polygon feature IDs unchanged" );
        check( changed.manifold.points[i].normalImpulse == 0.0f && changed.manifold.points[i].tangentImpulse == 0.0f, "unmatched feature inherited impulses" );
    }
}

void checkMassRefresh()
{
    std::array<float, 2> speeds{};
    std::array<float, 2> impulses{};
    int index = 0;
    for( const bool recycling : { false, true } )
    {
        world simulation{};
        simulation.SetGravity( {} );
        simulation.SetContactRecycleDistance( recycling ? 0.05f : 0.0f );
        const bodyId fixed = simulation.CreateBody();
        ( void )simulation.CreateShape( fixed, circle2{ {}, 1.0f } );
        const bodyId moving = simulation.CreateBody( bodyType::Dynamic, { { 2.0f, 0.0f }, {} } );
        const shapeId visitor = simulation.CreateShape( moving, circle2{ {}, 1.0f } );
        update( simulation );
        simulation.SetShapeDensity( visitor, 4.0f );
        simulation.SetBodyLinearVelocity( moving, { -2.0f, 0.0f } );
        simulation.Step( 0.001f );
        speeds[index] = simulation.GetBodyLinearVelocity( moving ).x;
        impulses[index++] = getContact( simulation, moving ).manifold.points[0].normalImpulse;
    }
    check( std::fabs( speeds[0] ) < 1e-4f && std::fabs( speeds[1] - speeds[0] ) < 1e-4f, "recycled solver used stale inverse mass" );
    // Unit circle at density 4 has mass 4*pi; stopping speed 2 requires impulse 8*pi.
    check( std::fabs( impulses[0] - 8.0f * std::numbers::pi_v<float> ) < 1e-3f && std::fabs( impulses[1] - impulses[0] ) < 1e-3f, "recycled contact retained stale effective mass" );
}

#pragma endregion ImpulsePersistence

} // namespace

int main()
{
    checkDistanceThreshold();
    checkEmptyTolerance();
    checkRotationGuards();
    checkCreationSettings();
    checkFastExclusion();
    checkImpulsePersistence();
    checkFeatureReset();
    checkMassRefresh();
}
