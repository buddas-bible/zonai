#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <vector>

#include "dynamics/world.h"
#include "geometry/polygon2.h"

using namespace zonai;

namespace
{

void check( bool ok, const char* message )
{
    if( ok ) return;

    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}

void near( float actual, float expected, const char* message )
{
    check( std::fabs( actual - expected ) < 0.003f, message );
}

worldShapeCastInput2 movingCircle( vec2 start = {}, vec2 translation = { 10.0f, 0.0f }, float radius = 0.5f )
{
    worldShapeCastInput2 input{};
    input.geometry = circle2{ {}, radius };
    input.transform.position = start;
    input.translation = translation;
    return input;
}

void checkBodyTypesAndClosest()
{
    world simulation{};

    const bodyId fixed = simulation.CreateBody( bodyType::Static, { { 2.0f, 0.0f }, {} } );
    const shapeId staticShape = simulation.CreateShape( fixed, circle2{ {}, 0.5f } );

    const bodyId moving = simulation.CreateBody( bodyType::Dynamic, { { 5.0f, 0.0f }, {} } );
    const shapeId dynamicShape = simulation.CreateShape( moving, circle2{ {}, 0.5f } );

    const bodyId kinematic = simulation.CreateBody( bodyType::Kinematic, { { 8.0f, 0.0f }, {} } );
    const shapeId sensor = simulation.CreateSensorShape( kinematic, circle2{ {}, 0.5f } );

    const auto input = movingCircle();
    std::vector<shapeId> found;

    simulation.castShape( input,
        [&]( shapeId id, vec2 point, vec2 normal, float fraction )
        {
            check( simulation.IsValid( id ), "World query must report active handles" );
            check( IsFinite( point ) && IsFinite( normal ) && fraction >= 0.0f && fraction < 1.0f,
                "World query must report finite hit data" );
            found.push_back( id );
            return 1.0f;
        } );

    check( found.size() == 3, "World ShapeCast must visit static, dynamic and kinematic Sensor targets" );
    check( std::find( found.begin(), found.end(), staticShape ) != found.end() &&
        std::find( found.begin(), found.end(), dynamicShape ) != found.end() &&
        std::find( found.begin(), found.end(), sensor ) != found.end(),
        "World ShapeCast missed a target kind" );

    const auto closest = simulation.castShapeClosest( input );
    check( closest.hit && closest.shape == staticShape, "Closest ShapeCast must select earliest collision" );
    near( closest.fraction, 0.1005f, "Circle versus circle fraction" );
    near( closest.point.x, 1.5f, "World point on target circle" );
    near( closest.normal.x, -1.0f, "World normal points from target toward moving circle" );

    int stopped = 0;
    simulation.castShape( input, [&]( shapeId, vec2, vec2, float )
    {
        ++stopped;
        return 0.0f;
    } );
    check( stopped == 1, "Zero callback must stop all trees" );

    const shapeId initial = simulation.CreateShape( fixed, circle2{ { -2.0f, 0.0f }, 0.5f } );
    int initialCount = 0;
    simulation.castShape( input, [&]( shapeId id, vec2, vec2 normal, float fraction )
    {
        if( id == initial )
        {
            ++initialCount;
            check( fraction == 0.0f && LengthSquared( normal ) == 0.0f,
                "Initial overlap reports zero normal and fraction" );
        }
        return 1.0f;
    } );
    check( initialCount == 1 && simulation.castShapeClosest( input ).shape == staticShape,
        "Closest query skips initial overlap but general query reports it" );

    simulation.DestroyShape( initial );
    simulation.SetBodyTransform( fixed, { { 30.0f, 0.0f }, {} } );
    check( simulation.castShapeClosest( input ).shape == dynamicShape,
        "Body teleport changes spatial query immediately" );

    simulation.DestroyShape( dynamicShape );
    check( simulation.castShapeClosest( input ).shape == sensor, "Destroyed target must disappear" );

    simulation.DestroyBody( kinematic );
    check( !simulation.castShapeClosest( input ).hit, "Destroyed owner clears ShapeCast candidates" );

    world unrelated{};
    const bodyId id = unrelated.CreateBody();
    ( void )unrelated.CreateShape( id, circle2{ {}, 0.5f } );
    check( !unrelated.IsValid( staticShape ), "ShapeCast handle must remain World scoped" );
}

void checkFilteringAndCallbacks()
{
    world simulation{};
    const bodyId fixed = simulation.CreateBody( bodyType::Static, { { 3.0f, 0.0f }, {} } );
    const shapeId included = simulation.CreateShape( fixed, circle2{ {}, 0.5f }, collisionFilter{ 2, 4, -7 } );
    ( void )simulation.CreateShape( fixed, circle2{ {}, 0.5f }, collisionFilter{ 8, 4, -7 } );
    ( void )simulation.CreateShape( fixed, circle2{ {}, 0.5f }, collisionFilter{ 2, 0, 7 } );

    collisionMatrix matrix{};
    matrix.setPair( 1, 2, false );
    simulation.setCollisionMatrix( matrix );

    const world& readonly = simulation;
    int count = 0;
    auto moveOnly = [state = std::make_unique<int>( 0 ), &count]( shapeId id, vec2, vec2, float ) mutable
    {
        ++*state;
        ++count;
        check( id.index1 != 0, "Callback must receive live shape handle" );
        return 1.0f;
    };

    readonly.castShape( movingCircle(), queryFilter{ 4, 2 }, std::move( moveOnly ) );
    check( count == 1, "Query filter checks both masks independently of group/matrix" );
    check( readonly.castShapeClosest( movingCircle(), queryFilter{ 4, 2 } ).shape == included,
        "Filtered closest cast chooses included Shape" );
    check( !readonly.castShapeClosest( movingCircle() ).hit, "Default filter respects target mask" );

    int ignored = 0;
    readonly.castShape( movingCircle(), queryFilter{ 4, 2 }, [&]( shapeId, vec2, vec2, float )
    {
        ++ignored;
        return -1.0f;
    } );
    check( ignored == 1, "-1 ignores a hit without stopping" );

    const auto shouldNotCall = []( shapeId, vec2, vec2, float )
    {
        check( false, "Invalid ShapeCast must not invoke callback" );
        return 1.0f;
    };

    auto invalid = movingCircle();
    invalid.translation = {};
    readonly.castShape( invalid, shouldNotCall );

    invalid = movingCircle();
    invalid.translation.x = std::numeric_limits<float>::infinity();
    readonly.castShape( invalid, shouldNotCall );

    invalid = movingCircle();
    invalid.transform.position.x = std::numeric_limits<float>::quiet_NaN();
    readonly.castShape( invalid, shouldNotCall );

    invalid = movingCircle();
    invalid.geometry = {};
    readonly.castShape( invalid, shouldNotCall );

    invalid = movingCircle();
    invalid.geometry = circle2{ {}, -1.0f };
    readonly.castShape( invalid, shouldNotCall );
}

void checkSweepGeometryAndNormals()
{
    // Query Circle vs rotated Body-local circle tests the target transform.
    world circleWorld{};
    const bodyId rotated = circleWorld.CreateBody( bodyType::Static,
        { { 4.0f, 1.0f }, rot2::FromRadians( 1.57079632679f ) } );
    const shapeId shifted = circleWorld.CreateShape( rotated, circle2{ { 1.0f, 0.0f }, 0.5f } );
    const auto hit = circleWorld.castShapeClosest( movingCircle( { 0.0f, 2.0f }, { 8.0f, 0.0f } ) );

    check( hit.hit && hit.shape == shifted, "Body transform must rotate the local shape" );
    near( hit.normal.x, -1.0f, "Rotated local target normal transformed into World" );
    near( hit.fraction, 0.375625f, "Translated circle-to-circle fraction" );

    // Query Capsule vs Convex Polygon uses existing ShapeCast proxy/solver.
    world polygonWorld{};
    const bodyId boxBody = polygonWorld.CreateBody( bodyType::Static, { { 4.0f, 0.0f }, {} } );
    const shapeId box = polygonWorld.CreateShape( boxBody, MakeBox( { 0.75f, 1.0f } ) );

    worldShapeCastInput2 capsule{};
    capsule.geometry = capsule2{ { 0.0f, -0.4f }, { 0.0f, 0.4f }, 0.25f };
    capsule.translation = { 6.0f, 0.0f };

    const auto polygonHit = polygonWorld.castShapeClosest( capsule );
    check( polygonHit.hit && polygonHit.shape == box, "Capsule sweep hits convex Polygon" );
    check( polygonHit.fraction > 0.0f && polygonHit.fraction < 1.0f &&
        polygonHit.normal.x < -0.9f, "Capsule/Polygon hit has a usable fraction and normal" );

    // Static chain target must reject approach from its back side.
    world terrain{};
    const bodyId terrainBody = terrain.CreateBody();
    const shapeId chain = terrain.CreateShape( terrainBody, chainSegment2{
        { -4.0f, 0.0f }, { { -2.0f, 0.0f }, { 2.0f, 0.0f } }, { 4.0f, 0.0f } } );

    const auto front = terrain.castShapeClosest( movingCircle( { 0.0f, -3.0f }, { 0.0f, 6.0f } ) );
    check( front.hit && front.shape == chain && front.normal.y < -0.9f,
        "Chain front is the edge's right side" );

    const auto back = terrain.castShapeClosest( movingCircle( { 0.0f, 3.0f }, { 0.0f, -6.0f } ) );
    check( !back.hit, "Chain backface must be rejected" );
}

void checkClippingAndEncroachment()
{
    world simulation{};
    const bodyId target = simulation.CreateBody( bodyType::Static, { { 5.0f, 0.0f }, {} } );
    const shapeId other = simulation.CreateShape( target, circle2{ {}, 1.0f } );

    const auto input = movingCircle( {}, { 10.0f, 0.0f }, 1.0f );
    check( simulation.castShapeClosest( input ).shape == other, "Full moving Shape hits a target" );

    const auto partial = movingCircle( {}, { 2.0f, 0.0f }, 1.0f );
    check( !simulation.castShapeClosest( partial ).hit, "Sweep ending before target must miss" );

    world close{};
    const bodyId overlap = close.CreateBody( bodyType::Static, { { 1.99f, 0.0f }, {} } );
    ( void )close.CreateShape( overlap, circle2{ {}, 1.0f } );
    auto nearInput = movingCircle( {}, { 1.0f, 0.0f }, 1.0f );

    int startOverlap = 0;
    close.castShape( nearInput, [&]( shapeId, vec2, vec2, float fraction )
    {
        ++startOverlap;
        check( fraction == 0.0f, "Default ShapeCast initial near overlap has fraction zero" );
        return 1.0f;
    } );
    check( startOverlap == 1, "Initial near overlap must be reported" );

    nearInput.canEncroach = true;
    const auto encroaching = close.castShapeClosest( nearInput );
    check( encroaching.hit && encroaching.fraction > 0.0f && encroaching.fraction < 0.02f,
        "canEncroach advances an initially close but separated pair" );
}

} // namespace

int main()
{
    checkBodyTypesAndClosest();
    checkFilteringAndCallbacks();
    checkSweepGeometryAndNormals();
    checkClippingAndEncroachment();
}
