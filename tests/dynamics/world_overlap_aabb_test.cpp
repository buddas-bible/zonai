#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "dynamics/world.h"
#include "geometry/circle2.h"

using namespace zonai;

namespace
{

void check( bool condition, const char* message )
{
    if( condition ) return;

    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}

std::vector<shapeId> collect( const world& simulation, const aabb2& bounds, queryFilter filter = {} )
{
    std::vector<shapeId> result;

    simulation.overlapAABB( bounds, filter,
        [&]( shapeId id )
        {
            check( simulation.IsValid( id ), "query returned an invalid shape handle" );
            result.push_back( id );
            return true;
        } );

    return result;
}

bool contains( const std::vector<shapeId>& result, shapeId id )
{
    return std::find( result.begin(), result.end(), id ) != result.end();
}

void checkWorldCandidates()
{
    world simulation{};

    const bodyId fixed = simulation.CreateBody();
    const shapeId staticShape = simulation.CreateShape( fixed, circle2{ {}, 0.4f } );
    const shapeId secondStatic = simulation.CreateShape( fixed, circle2{ { 0.2f, 0.0f }, 0.2f } );

    const bodyId moving = simulation.CreateBody( bodyType::Dynamic, { { 0.8f, 0.0f }, {} } );
    const shapeId dynamicShape = simulation.CreateShape( moving, circle2{ {}, 0.4f } );

    const bodyId kinematic = simulation.CreateBody( bodyType::Kinematic, { { -0.8f, 0.0f }, {} } );
    const shapeId sensor = simulation.CreateSensorShape( kinematic, circle2{ {}, 0.4f } );

    const bodyId farAway = simulation.CreateBody( bodyType::Static, { { 30.0f, 0.0f }, {} } );
    const shapeId farShape = simulation.CreateShape( farAway, circle2{ {}, 0.4f } );

    const aabb2 bounds{ { -1.5f, -1.5f }, { 1.5f, 1.5f } };
    const std::vector<shapeId> result = collect( simulation, bounds );

    check( result.size() == 4, "world AABB query should report each nearby shape exactly once" );
    check( contains( result, staticShape ) && contains( result, secondStatic ) &&
           contains( result, dynamicShape ) && contains( result, sensor ), "query omitted a body type, sensor, or second shape" );
    check( !contains( result, farShape ), "query reported a distant shape" );
    check( collect( simulation, { { 40.0f, 40.0f }, { 41.0f, 41.0f } } ).empty(), "empty region should have no candidates" );

    // 다른 물체의 speculative bounds와 겹치지 않는 별도 World에서 fat 오탐만 검사함.
    world isolated{};
    const bodyId owner = isolated.CreateBody();
    const shapeId onlyShape = isolated.CreateShape( owner, circle2{ {}, 0.4f } );

    const aabb2 actual = isolated.GetShapeAABB( onlyShape );
    const aabb2 fat = isolated.GetShapeFatAABB( onlyShape );
    check( fat.max.x > actual.max.x, "fat AABB must extend beyond speculative AABB" );

    const float x = ( actual.max.x + fat.max.x ) * 0.5f;
    check( collect( isolated, { { x, -0.001f }, { x + 0.0001f, 0.001f } } ).empty(),
           "fat proxy overlap alone must not be reported" );
}

void checkFiltering()
{
    world simulation{};
    const bodyId fixed = simulation.CreateBody();

    const shapeId allowed = simulation.CreateShape( fixed, circle2{ {}, 0.5f }, collisionFilter{ 2, 4, -7 } );
    const shapeId wrongCategory = simulation.CreateShape( fixed, circle2{ {}, 0.5f }, collisionFilter{ 8, 4, -7 } );
    const shapeId blockedMask = simulation.CreateShape( fixed, circle2{ {}, 0.5f }, collisionFilter{ 2, 0, 7 } );
    const shapeId sensor = simulation.CreateSensorShape( fixed, circle2{ {}, 0.5f }, collisionFilter{ 2, 4, 0 } );
    const aabb2 region{ { -1.0f, -1.0f }, { 1.0f, 1.0f } };

    collisionMatrix matrix{};
    matrix.setPair( 1, 2, false );
    simulation.setCollisionMatrix( matrix );

    const auto hits = collect( simulation, region, queryFilter{ 4, 2 } );
    check( hits.size() == 2 && contains( hits, allowed ) && contains( hits, sensor ),
           "query category/mask must check both directions, ignore group and collision matrix, and include sensors" );
    check( !contains( hits, wrongCategory ) && !contains( hits, blockedMask ),
           "query returned a shape excluded by category or mask" );
    check( collect( simulation, region ).empty(), "default query still respects the shape's mask bits" );
    check( collect( simulation, region, queryFilter{ 0, ~std::uint64_t{ 0 } } ).empty(),
           "zero query category must reject every shape" );
}

void checkLifecycle()
{
    world simulation{};
    const bodyId moving = simulation.CreateBody( bodyType::Dynamic );
    const shapeId oldShape = simulation.CreateShape( moving, circle2{ {}, 0.3f } );
    const aabb2 origin{ { -1.0f, -1.0f }, { 1.0f, 1.0f } };
    const aabb2 destination{ { 19.0f, -1.0f }, { 21.0f, 1.0f } };

    check( contains( collect( simulation, origin ), oldShape ), "new proxy should be queryable before Step" );
    simulation.SetBodyTransform( moving, { { 20.0f, 0.0f }, {} } );
    check( collect( simulation, origin ).empty(), "moving a proxy must remove its previous query position" );
    check( contains( collect( simulation, destination ), oldShape ), "moving a proxy must update query results immediately" );

    simulation.DestroyShape( oldShape );
    check( collect( simulation, destination ).empty(), "destroyed shape must disappear from queries" );

    const shapeId replacement = simulation.CreateShape( moving, circle2{ {}, 0.3f } );
    check( oldShape != replacement && !simulation.IsValid( oldShape ), "recycled shape must not reuse old handle" );
    check( contains( collect( simulation, destination ), replacement ), "recycled slot must return its new handle" );

    world otherWorld{};
    const bodyId otherBody = otherWorld.CreateBody();
    const shapeId otherShape = otherWorld.CreateShape( otherBody, circle2{ {}, 0.3f } );
    check( !otherWorld.IsValid( replacement ) && !simulation.IsValid( otherShape ), "query handles must belong to their world" );

    simulation.DestroyBody( moving );
    check( collect( simulation, destination ).empty(), "destroying an owning body must clear the query" );
}

void checkCallbacksAndInvalidBounds()
{
    world simulation{};
    const bodyId fixed = simulation.CreateBody();
    ( void )simulation.CreateShape( fixed, circle2{ {}, 0.5f } );
    ( void )simulation.CreateShape( fixed, circle2{ { 0.2f, 0.0f }, 0.5f } );

    const world& readonly = simulation;
    const aabb2 region{ { -1.0f, -1.0f }, { 1.0f, 1.0f } };
    int calls = 0;
    auto moveOnly = [token = std::make_unique<int>( 0 ), &calls]( shapeId ) mutable
    {
        ++*token;
        ++calls;
        return false;
    };

    readonly.overlapAABB( region, std::move( moveOnly ) );
    check( calls == 1, "false callback should stop traversal without copying a move-only callable" );

    const auto noResult = [&]( shapeId )
    {
        check( false, "invalid AABB must not call visitor" );
        return true;
    };

    const float infinity = std::numeric_limits<float>::infinity();
    const float nan = std::numeric_limits<float>::quiet_NaN();

    readonly.overlapAABB( { { 1.0f, 1.0f }, { -1.0f, -1.0f } }, noResult );
    readonly.overlapAABB( { { nan, 0.0f }, { 1.0f, 1.0f } }, noResult );
    readonly.overlapAABB( { { 0.0f, 0.0f }, { infinity, 1.0f } }, noResult );
    check( readonly.GetShapeCount() == 2, "spatial query must not change world storage" );
}

} // namespace

int main()
{
    checkWorldCandidates();
    checkFiltering();
    checkLifecycle();
    checkCallbacksAndInvalidBounds();
}
