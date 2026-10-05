#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>

#include "dynamics/world.h"

using namespace zonai;

namespace
{
void check( bool condition, const char* message )
{
    if( !condition ) { std::fprintf( stderr, "%s\n", message ); std::exit( EXIT_FAILURE ); }
}
bodyId ball( world& simulation, vec2 position, bodyType type = bodyType::Dynamic )
{
    const auto id = simulation.CreateBody( type, { position, {} } );
    (void)simulation.CreateShape( id, circle2{ {}, 0.25f } );
    return id;
}
distanceJointDef definition( bodyId a, bodyId b, float length = 2.0f, bool collide = false )
{
    distanceJointDef value{}; value.bodyA = a; value.bodyB = b; value.length = length; value.collideConnected = collide; return value;
}

#pragma region LifetimeAndFiltering
void checkLifetime()
{
    world simulation{}, other{};
    const auto a = ball( simulation, {} ), b = ball( simulation, { 2.0f, 0.0f } );
    std::array<jointId, 3> joints{};
    for( auto& id : joints ) id = simulation.createDistanceJoint( definition( a, b ) );
    check( simulation.getJointCount() == 3 && simulation.GetBody( a ).jointCount == 3 && simulation.GetBody( b ).jointCount == 3, "joint counts" );
    check( !simulation.IsValid( jointId{} ) && !other.IsValid( joints[0] ), "joint handle ownership" );
    simulation.destroyJoint( joints[1] );
    check( simulation.GetBody( a ).jointCount == 2 && simulation.GetBody( b ).jointCount == 2, "middle joint unlink" );
    const auto reused = simulation.createDistanceJoint( definition( a, b ) );
    check( reused.index1 == joints[1].index1 && reused.generation != joints[1].generation && !simulation.IsValid( joints[1] ), "joint slot generation" );
    simulation.destroyJoint( joints[2] ); simulation.destroyJoint( joints[0] );
    simulation.DestroyBody( a );
    check( simulation.getJointCount() == 0 && !simulation.IsValid( reused ) && simulation.GetBody( b ).headJointKey == -1 && simulation.GetBody( b ).jointCount == 0, "body destruction removes joints" );
    const auto c = ball( simulation, {} );
    const auto fresh = simulation.createDistanceJoint( definition( c, b ) );
    check( simulation.IsValid( fresh ) && !simulation.IsValid( reused ), "reused body joint slots" );
    std::array<bodyId, 32> neighbors{};
    for( int i = 0; i < 32; ++i )
    {
        neighbors[i] = ball( simulation, { 10.0f + 2.0f * i, 0.0f } );
        (void)simulation.createDistanceJoint( definition( b, neighbors[i] ) );
    }
    check( simulation.IsValid( fresh ) && simulation.GetBody( b ).jointCount == 33, "joint links survive storage growth" );
    for( const auto id : neighbors ) simulation.DestroyBody( id );
    check( simulation.GetBody( b ).jointCount == 1 && simulation.getJointCount() == 1, "grown joint links unlink" );

    alignas( world ) std::byte storage[sizeof( world )];
    auto* first = std::construct_at( reinterpret_cast<world*>( storage ) );
    const auto old = first->createDistanceJoint( definition( ball( *first, {} ), ball( *first, { 2.0f, 0.0f } ) ) );
    std::destroy_at( first );
    auto* second = std::construct_at( reinterpret_cast<world*>( storage ) );
    const auto next = second->createDistanceJoint( definition( ball( *second, {} ), ball( *second, { 2.0f, 0.0f } ) ) );
    check( old.index1 == next.index1 && !second->IsValid( old ), "old world joint resurrected" );
    std::destroy_at( second );
}

void checkFilters()
{
    world simulation{}; simulation.SetGravity( {} );
    const auto a = ball( simulation, {} ), b = ball( simulation, { 0.3f, 0.0f } );
    simulation.Step( 0.0f ); check( simulation.GetContactCount() == 1, "existing contact fixture" );
    const auto allowed = simulation.createDistanceJoint( definition( a, b, 0.3f, true ) );
    check( simulation.GetContactCount() == 1, "allowed joint preserves contact" );
    const auto blocked = simulation.createDistanceJoint( definition( a, b, 0.3f ) );
    check( simulation.GetContactCount() == 0, "blocking joint removes existing contact" );
    const auto secondBlocker = simulation.createDistanceJoint( definition( a, b, 0.3f ) );
    simulation.Step( 0.0f ); check( simulation.GetContactCount() == 0, "blocked pair recreated" );
    simulation.destroyJoint( blocked ); simulation.Step( 0.0f );
    check( simulation.GetContactCount() == 0, "another blocker still applies" );
    simulation.destroyJoint( secondBlocker ); simulation.Step( 0.0f );
    check( simulation.GetContactCount() == 1, "stationary pair rediscovered after unblocking" );
    simulation.destroyJoint( allowed );

    // Joint filter는 positive groupIndex보다 우선하지만 discrete sensor overlap에는 적용하지 않음.
    collisionFilter filter{}; filter.groupIndex = 1;
    const auto shapeA = simulation.CreateShape( a, circle2{ {}, 0.25f }, filter );
    const auto shapeB = simulation.CreateShape( b, circle2{ {}, 0.25f }, filter );
    (void)shapeA; (void)shapeB;
    const auto joint = simulation.createDistanceJoint( definition( a, b, 0.3f ) );
    const auto sensor = simulation.CreateSensorShape( a, circle2{ {}, 1.0f } );
    simulation.SetShapeSensorEventsEnabled( sensor, true );
    simulation.SetShapeSensorEventsEnabled( shapeB, true );
    simulation.Step( 0.0f );
    check( simulation.GetContactCount() == 0 && simulation.GetShapeSensorCapacity( sensor ) > 0, "joint blocking and discrete sensor policy" );
    simulation.destroyJoint( joint );
}
#pragma endregion

#pragma region IslandsAndSleep
void checkGraph()
{
    world simulation{}; simulation.SetGravity( {} ); simulation.SetContinuousEnabled( false );
    const auto a = ball( simulation, {} ), b = ball( simulation, { 2.0f, 0.0f } ), c = ball( simulation, { 4.0f, 0.0f } );
    (void)simulation.createDistanceJoint( definition( a, b ) );
    (void)simulation.createDistanceJoint( definition( b, c ) );
    for( int i = 0; i < 40; ++i ) simulation.Step( 1.0f / 60.0f, 4 );
    check( !simulation.IsBodyAwake( a ) && !simulation.IsBodyAwake( b ) && !simulation.IsBodyAwake( c ), "joint island sleeps" );
    simulation.ApplyLinearImpulseToCenter( a, { 1.0f, 0.0f } );
    check( simulation.IsBodyAwake( a ) && simulation.IsBodyAwake( b ) && simulation.IsBodyAwake( c ), "joint wake graph" );
    simulation.SetBodyAwake( b, false );
    check( !simulation.IsBodyAwake( a ) && !simulation.IsBodyAwake( c ), "explicit joint island sleep" );
    simulation.SetBodyTransform( c, simulation.GetBodyTransform( c ) );
    check( simulation.IsBodyAwake( a ), "pose wakes connected joints" );

    world anchored{}; anchored.SetGravity( {} );
    const auto ground = anchored.CreateBody();
    const auto left = ball( anchored, { -2.0f, 0.0f } ), right = ball( anchored, { 2.0f, 0.0f } );
    const auto leftJoint = anchored.createDistanceJoint( definition( ground, left ) );
    (void)anchored.createDistanceJoint( definition( ground, right ) );
    anchored.SetBodyAwake( left, false ); anchored.SetBodyAwake( right, false );
    anchored.SetBodyAwake( left, true );
    check( !anchored.IsBodyAwake( right ), "static anchor must not bridge islands" );
    const auto extra = anchored.createDistanceJoint( definition( ground, left ) );
    check( !anchored.IsBodyAwake( right ), "static joint creation wakes unrelated island" );
    anchored.destroyJoint( extra );
    check( !anchored.IsBodyAwake( right ), "static joint destruction wakes unrelated island" );
    anchored.destroyJoint( leftJoint ); check( anchored.IsBodyAwake( left ), "joint destruction wakes endpoint" );
    anchored.SetBodyTransform( ground, {} ); check( anchored.IsBodyAwake( right ), "static anchor pose wakes neighbors" );

    std::array<body, 4> bodies{};
    for( int i = 0; i < 4; ++i ) { bodies[i].bodyId = i; bodies[i].type = bodyType::Dynamic; }
    bodies[0].type = bodyType::Static;
    std::array<distanceJointSim2, 3> joints{};
    for( int i = 0; i < 3; ++i ) { joints[i].jointId = i; joints[i].bodyIdA = 0; joints[i].bodyIdB = i + 1; }
    const auto graph = BuildIslands( bodies, {}, joints );
    check( graph.islands.size() == 3 && graph.jointIds.size() == 3 && graph.bodyIds.size() == 3, "static joints assigned once to distinct islands" );
    for( const auto& island : graph.islands ) check( island.jointCount == 1, "joint island range" );
    bodies[1].awake = false;
    check( BuildIslands( bodies, {}, joints ).jointIds.size() == 2, "sleeping joint excluded" );
    bodies[1].awake = true; joints[1].bodyIdA = 1;
    check( BuildIslands( bodies, {}, joints ).islands.size() == 2, "dynamic joint joins islands" );

    world mixed{}; mixed.SetGravity( {} );
    const auto contactA = ball( mixed, {} ), contactB = ball( mixed, { 0.45f, 0.0f } ), jointC = ball( mixed, { 2.45f, 0.0f } );
    (void)mixed.createDistanceJoint( definition( contactB, jointC ) ); mixed.Step( 0.0f );
    check( mixed.GetContactCount() == 1, "mixed graph contact fixture" );
    mixed.SetBodyAwake( jointC, false );
    check( !mixed.IsBodyAwake( contactA ), "mixed graph sleep traverses joint and contact" );
    mixed.SetBodyAwake( contactA, true );
    check( mixed.IsBodyAwake( jointC ), "mixed graph wake traverses contact and joint" );
}
#pragma endregion

#pragma region SolverIntegration
void checkMotion( int subSteps )
{
    world simulation{}; simulation.SetContinuousEnabled( false ); simulation.SetSleepingEnabled( false );
    const auto anchor = simulation.CreateBody();
    const auto bob = ball( simulation, { 0.0f, -2.0f } );
    const auto joint = simulation.createDistanceJoint( definition( anchor, bob ) );
    simulation.ApplyLinearImpulseToCenter( bob, { 0.1f, 0.0f } );
    for( int i = 0; i < 180; ++i ) simulation.Step( 1.0f / 60.0f, subSteps );
    const auto data = simulation.getDistanceJointData( joint );
    check( std::abs( data.currentLength - 2.0f ) < 0.03f && IsFinite( data.anchorB ), "pendulum fixed distance" );
    simulation.Step( 1.0f / 120.0f, subSteps + 1 );
    check( IsFinite( simulation.GetBodyLinearVelocity( bob ) ), "changed time step finite" );

    world moving{}; moving.SetGravity( {} ); moving.SetContinuousEnabled( false );
    const auto kinematic = ball( moving, {}, bodyType::Kinematic ), follower = ball( moving, { 2.0f, 0.0f } );
    (void)moving.createDistanceJoint( definition( kinematic, follower ) );
    moving.SetBodyLinearVelocity( kinematic, { 0.5f, 0.0f } );
    for( int i = 0; i < 60; ++i ) moving.Step( 1.0f / 60.0f, subSteps );
    check( std::abs( moving.GetBodyTransform( follower ).position.x - moving.GetBodyTransform( kinematic ).position.x - 2.0f ) < 0.02f, "kinematic anchor follows" );
    check( moving.GetBodyLinearVelocity( kinematic ).x == 0.5f, "kinematic velocity not changed" );
}

void checkZeroStepAndCom()
{
    world simulation{}; simulation.SetGravity( {} ); simulation.SetContinuousEnabled( false );
    const auto a = simulation.CreateBody(), b = simulation.CreateBody( bodyType::Dynamic, { { 2.0f, 0.0f }, {} } );
    const auto shape = simulation.CreateShape( b, circle2{ { 0.0f, 0.5f }, 0.25f } );
    auto def = definition( a, b ); def.localAnchorB = { 0.0f, 0.0f };
    const auto joint = simulation.createDistanceJoint( def );
    check( simulation.GetBodyLocalCenter( b ).y == 0.5f, "off center fixture" );
    simulation.ApplyForceToCenter( b, { 0.0f, 1.0f } );
    simulation.Step( 0.0f );
    check( simulation.GetBodyTransform( b ).position.x == 2.0f && LengthSquared( simulation.GetBodyLinearVelocity( b ) ) == 0.0f, "zero step skips joint solver" );
    simulation.Step( 1.0f / 60.0f, 4 );
    check( simulation.GetBodyLinearVelocity( b ).y > 0.0f, "zero step preserves force" );
    simulation.SetShapeDensity( shape, 2.0f );
    simulation.SetBodyTransform( b, { { 2.5f, 0.0f }, {} } );
    for( int i = 0; i < 120; ++i ) simulation.Step( 1.0f / 60.0f, 4 );
    check( std::abs( simulation.getDistanceJointData( joint ).currentLength - 2.0f ) < 0.02f, "mass/pose change joint recovers" );
    simulation.DestroyShape( shape );
    (void)simulation.CreateShape( b, circle2{ { 0.0f, -0.5f }, 0.25f } );
    simulation.Step( 1.0f / 60.0f, 4 );
    check( IsFinite( simulation.getDistanceJointData( joint ).anchorB ), "COM replacement finite" );

    world coincident{}; coincident.SetGravity( {} );
    const auto origin = coincident.CreateBody(), duplicate = ball( coincident, {} );
    const auto shortJoint = coincident.createDistanceJoint( definition( origin, duplicate, 0.001f ) );
    check( coincident.getDistanceJointData( shortJoint ).length == LINEAR_SLOP, "target length clamped to linear slop" );
    for( int i = 0; i < 10; ++i ) coincident.Step( 1.0f / 60.0f, 4 );
    check( IsFinite( coincident.GetBodyTransform( duplicate ).position ), "coincident world anchors finite" );
}

void checkImpulseReset( int change )
{
    world cached{}, fresh{};
    cached.SetContinuousEnabled( false ); fresh.SetContinuousEnabled( false );
    cached.SetSleepingEnabled( false ); fresh.SetSleepingEnabled( false );
    const auto anchorA = cached.CreateBody(), anchorB = fresh.CreateBody();
    const auto secondA = cached.CreateBody( bodyType::Static, { { 2.0f, 0.0f }, {} } );
    const auto secondB = fresh.CreateBody( bodyType::Static, { { 2.0f, 0.0f }, {} } );
    const auto bobA = ball( cached, { 0.0f, -2.0f } ), bobB = ball( fresh, { 0.0f, -2.0f } );
    const auto shapeA = cached.CreateShape( bobA, circle2{ { 0.0f, 0.25f }, 0.25f } );
    const auto shapeB = fresh.CreateShape( bobB, circle2{ { 0.0f, 0.25f }, 0.25f } );
    (void)cached.createDistanceJoint( definition( anchorA, bobA ) );
    const auto old = fresh.createDistanceJoint( definition( anchorB, bobB ) );
    (void)cached.createDistanceJoint( definition( secondA, bobA, std::sqrt( 8.0f ) ) );
    const auto oldSecond = fresh.createDistanceJoint( definition( secondB, bobB, std::sqrt( 8.0f ) ) );
    for( int i = 0; i < 20; ++i ) { cached.Step( 1.0f / 60.0f, 4 ); fresh.Step( 1.0f / 60.0f, 4 ); }
    if( change == 0 )
    {
        cached.SetShapeDensity( shapeA, 3.0f ); fresh.SetShapeDensity( shapeB, 3.0f );
    }
    else if( change == 1 )
    {
        cached.SetBodyTransform( bobA, { { 0.5f, -2.25f }, {} } );
        fresh.SetBodyTransform( bobB, { { 0.5f, -2.25f }, {} } );
    }
    // 재생성한 Joint는 impulse가 0임. 기존 Joint도 mass/pose/h 변경 후 같은 결과여야 함.
    // Free-list 재사용 순서도 맞춰 solver의 stable slot 순서를 유지함.
    fresh.destroyJoint( oldSecond ); fresh.destroyJoint( old );
    (void)fresh.createDistanceJoint( definition( anchorB, bobB ) );
    (void)fresh.createDistanceJoint( definition( secondB, bobB, std::sqrt( 8.0f ) ) );
    const float timeStep = change == 2 ? 1.0f / 120.0f : 1.0f / 60.0f;
    cached.Step( timeStep, 4 ); fresh.Step( timeStep, 4 );
    check( LengthSquared( cached.GetBodyTransform( bobA ).position - fresh.GetBodyTransform( bobB ).position ) == 0.0f, "stale joint impulse after mass/pose/h change" );
    check( LengthSquared( cached.GetBodyLinearVelocity( bobA ) - fresh.GetBodyLinearVelocity( bobB ) ) == 0.0f, "stale joint velocity after mass/pose/h change" );
}

void checkContinuousFilter( bool collide, bool sensor )
{
    world simulation{}; simulation.SetGravity( {} ); simulation.SetSleepingEnabled( false );
    const auto obstacle = simulation.CreateBody();
    const auto fast = simulation.CreateBody( bodyType::Dynamic, { { -2.0f, 0.0f }, {} } );
    const auto fastShape = simulation.CreateShape( fast, circle2{ {}, 0.25f } );
    const auto obstacleShape = sensor ? simulation.CreateSensorShape( obstacle, circle2{ {}, 0.25f } ) : simulation.CreateShape( obstacle, circle2{ {}, 0.25f } );
    auto def = definition( obstacle, fast, 2.0f, collide ); def.localAnchorA = { -2.0f, 2.0f };
    const auto joint = simulation.createDistanceJoint( def );
    if( sensor )
    {
        simulation.SetShapeSensorEventsEnabled( obstacleShape, true );
        simulation.SetShapeSensorEventsEnabled( fastShape, true );
    }
    simulation.SetBodyLinearVelocity( fast, { 200.0f, 0.0f } );
    simulation.Step( 0.02f );
    check( IsFinite( simulation.GetBodyTransform( fast ).position ) && IsFinite( simulation.GetBodyLinearVelocity( fast ) ), "CCD joint finite" );
    if( sensor ) check( !simulation.GetSensorBeginEvents().empty() == collide, "continuous sensor respects joint filter" );
    else
    {
        check( simulation.HadBodyTimeOfImpact( fast ) == collide, "solid CCD respects joint filter" );
        simulation.DestroyShape( obstacleShape ); simulation.SetContinuousEnabled( false );
        simulation.SetBodyLinearVelocity( fast, {} );
        for( int i = 0; i < 180; ++i ) simulation.Step( 1.0f / 60.0f, 4 );
        check( std::abs( simulation.getDistanceJointData( joint ).currentLength - 2.0f ) < 0.02f, "distance recovers after CCD clipping" );
    }
}
#pragma endregion
}

int main()
{
    checkLifetime(); checkFilters(); checkGraph();
    checkMotion( 1 ); checkMotion( 4 ); checkZeroStepAndCom();
    for( int i = 0; i < 3; ++i ) checkImpulseReset( i );
    for( const bool collide : { false, true } ) for( const bool sensor : { false, true } ) checkContinuousFilter( collide, sensor );
    return EXIT_SUCCESS;
}
