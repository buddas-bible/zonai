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
    check( std::fabs( actual - expected ) < 0.002f, message );
}

void checkGeometry()
{
    const transform2 identity{};

    const auto circle = RayCastShapeGeometry( circle2{ {}, 1.0f }, identity, { -3.0f, 0.0f }, { 6.0f, 0.0f } );
    check( circle.hit, "circle ray should hit" );
    near( circle.fraction, 1.0f / 3.0f, "circle fraction" );
    near( circle.point.x, -1.0f, "circle hit point" );
    near( circle.normal.x, -1.0f, "circle outward normal" );
    check( !RayCastShapeGeometry( circle2{ {}, 1.0f }, identity, {}, { 3.0f, 0.0f } ).hit, "inside solid circle must miss" );
    check( !RayCastShapeGeometry( circle2{ {}, 1.0f }, identity, { -3.0f, 2.0f }, { 6.0f, 0.0f } ).hit, "circle ray must miss off axis" );
    check( !RayCastShapeGeometry( circle2{ {}, 1.0f }, identity, { -3.0f, 0.0f }, { 6.0f, 0.0f }, 0.2f ).hit, "circle clipping limit" );

    const segment2 segment{ { -2.0f, 0.0f }, { 2.0f, 0.0f } };
    const auto above = RayCastShapeGeometry( segment, identity, { 0.0f, 2.0f }, { 0.0f, -4.0f } );
    const auto below = RayCastShapeGeometry( segment, identity, { 0.0f, -2.0f }, { 0.0f, 4.0f } );
    check( above.hit && below.hit, "two sided segment must hit from both sides" );
    near( above.normal.y, 1.0f, "segment above outward normal" );
    near( below.normal.y, -1.0f, "segment below outward normal" );

    const chainSegment2 chain{ { -4.0f, 0.0f }, segment, { 4.0f, 0.0f } };
    check( !RayCastShapeGeometry( chain, identity, { 0.0f, 2.0f }, { 0.0f, -4.0f } ).hit,
           "chain backface must miss" );
    check( RayCastShapeGeometry( chain, identity, { 0.0f, -2.0f }, { 0.0f, 4.0f } ).hit,
           "chain front must hit" );

    const capsule2 capsule{ { -1.0f, 0.0f }, { 1.0f, 0.0f }, 0.5f };
    const auto side = RayCastShapeGeometry( capsule, identity, { 0.0f, 2.0f }, { 0.0f, -4.0f } );
    const auto cap = RayCastShapeGeometry( capsule, identity, { -3.0f, 0.0f }, { 6.0f, 0.0f } );
    check( side.hit && cap.hit, "capsule side and cap should hit" );
    near( side.point.y, 0.5f, "capsule side point" );
    near( side.normal.y, 1.0f, "capsule side normal" );
    near( cap.point.x, -1.5f, "capsule rounded cap point" );
    check( !RayCastShapeGeometry( capsule, identity, {}, { 3.0f, 0.0f } ).hit, "inside capsule must miss" );

    const polygon2 box = MakeBox( { 1.0f, 1.0f } );
    const auto polygon = RayCastShapeGeometry( box, identity, { -3.0f, 0.0f }, { 6.0f, 0.0f } );
    check( polygon.hit, "polygon face ray should hit" );
    near( polygon.fraction, 1.0f / 3.0f, "polygon fraction" );
    near( polygon.normal.x, -1.0f, "polygon normal" );
    check( !RayCastShapeGeometry( box, identity, {}, { 5.0f, 0.0f } ).hit, "inside polygon must miss" );
    check( !RayCastShapeGeometry( box, identity, { 2.0f, 2.0f }, { 4.0f, 0.0f } ).hit, "polygon ray should miss" );

    polygon2 rounded = box;
    rounded.radius = 0.25f;
    const auto roundedSide = RayCastShapeGeometry( rounded, identity, { -3.0f, 0.0f }, { 6.0f, 0.0f } );
    check( roundedSide.hit, "rounded polygon side hit" );
    near( roundedSide.point.x, -1.25f, "rounded polygon side contact" );
    const auto roundedCorner = RayCastShapeGeometry( rounded, identity, { -2.0f, -2.0f }, { 2.0f, 2.0f } );
    check( roundedCorner.hit, "rounded polygon corner hit" );
    near( Length( roundedCorner.normal ), 1.0f, "rounded polygon corner normal unit length" );

    const transform2 rotated{ { 4.0f, 1.0f }, rot2::FromRadians( 1.57079632679f ) };
    const auto moved = RayCastShapeGeometry( circle2{ { 1.0f, 0.0f }, 0.5f }, rotated,
        { 1.0f, 2.0f }, { 6.0f, 0.0f } );
    check( moved.hit, "shape ray must use body transform" );
    near( moved.point.x, 3.5f, "transformed circle point" );
}

void checkWorldQueries()
{
    world simulation{};
    const bodyId staticBody = simulation.CreateBody( bodyType::Static, { { 2.0f, 0.0f }, {} } );
    const shapeId first = simulation.CreateShape( staticBody, circle2{ {}, 0.5f } );
    const bodyId dynamicBody = simulation.CreateBody( bodyType::Dynamic, { { 5.0f, 0.0f }, {} } );
    const shapeId second = simulation.CreateShape( dynamicBody, circle2{ {}, 0.5f } );
    const bodyId kinematicBody = simulation.CreateBody( bodyType::Kinematic, { { 8.0f, 0.0f }, {} } );
    const shapeId sensor = simulation.CreateSensorShape( kinematicBody, circle2{ {}, 0.5f } );

    const vec2 origin{ 0.0f, 0.0f }, translation{ 10.0f, 0.0f };
    std::vector<shapeId> hits;

    simulation.castRay( origin, translation, [&]( shapeId id, vec2 point, vec2 normal, float fraction )
    {
        check( simulation.IsValid( id ) && point.x > 0.0f && normal.x < 0.0f && fraction > 0.0f,
               "world hit should return valid ID, point, normal, and fraction" );
        hits.push_back( id );
        return 1.0f;
    } );

    check( hits.size() == 3, "castRay should include static/dynamic/kinematic and sensors" );
    check( std::find( hits.begin(), hits.end(), first ) != hits.end() &&
           std::find( hits.begin(), hits.end(), second ) != hits.end() &&
           std::find( hits.begin(), hits.end(), sensor ) != hits.end(), "world cast missed a body type" );

    const auto closest = simulation.castRayClosest( origin, translation );
    check( closest.hit && closest.shape == first, "closest ray hit should find first intersection" );
    near( closest.fraction, 0.15f, "closest hit fraction" );
    near( closest.normal.x, -1.0f, "closest hit normal" );

    int count = 0;
    simulation.castRay( origin, translation, [&]( shapeId, vec2, vec2, float )
    {
        ++count;
        return 0.0f;
    } );
    check( count == 1, "zero result must terminate ray across all three trees" );

    int clipHits = 0;
    simulation.castRay( origin, translation, [&]( shapeId, vec2, vec2, float fraction )
    {
        ++clipHits;
        return fraction;
    } );
    check( clipHits >= 1, "clipped ray must report a hit" );

    simulation.SetBodyTransform( staticBody, { { 30.0f, 0.0f }, {} } );
    check( simulation.castRayClosest( origin, translation ).shape == second,
           "transformed body should update ray tree immediately" );
    simulation.DestroyShape( second );
    check( simulation.castRayClosest( origin, translation ).shape == sensor, "destroyed shape must disappear" );
    simulation.DestroyBody( kinematicBody );
    check( !simulation.castRayClosest( origin, translation ).hit, "destroyed owner removes its proxy" );

    world another{};
    const auto other = another.CreateBody();
    ( void )another.CreateShape( other, circle2{ {}, 0.5f } );
    check( !another.IsValid( first ), "ray handle cannot belong to another world" );
}

void checkFiltersAndCallbacks()
{
    world simulation{};
    const bodyId body = simulation.CreateBody( bodyType::Static, { { 2.0f, 0.0f }, {} } );
    const shapeId accepted = simulation.CreateShape( body, circle2{ {}, 0.5f }, collisionFilter{ 2, 4, -9 } );
    ( void )simulation.CreateShape( body, circle2{ {}, 0.5f }, collisionFilter{ 8, 4, -9 } );
    ( void )simulation.CreateShape( body, circle2{ {}, 0.5f }, collisionFilter{ 2, 0, 9 } );

    collisionMatrix matrix{};
    matrix.setPair( 1, 2, false );
    simulation.setCollisionMatrix( matrix );

    int calls = 0;
    const world& readonly = simulation;
    auto callback = [state = std::make_unique<int>( 0 ), &calls]( shapeId id, vec2, vec2, float ) mutable
    {
        ++*state;
        ++calls;
        check( id.index1 != 0, "move-only callback should receive live shape" );
        return 1.0f;
    };
    readonly.castRay( { 0.0f, 0.0f }, { 4.0f, 0.0f }, queryFilter{ 4, 2 }, std::move( callback ) );
    check( calls == 1, "query filter should use both masks, independent of group/matrix" );
    check( readonly.castRayClosest( { 0.0f, 0.0f }, { 4.0f, 0.0f }, queryFilter{ 4, 2 } ).shape == accepted,
        "closest hit with filter should choose accepted shape" );
    check( !readonly.castRayClosest( { 0.0f, 0.0f }, { 4.0f, 0.0f } ).hit, "default category 1 query respects shape mask" );

    int ignored = 0;
    readonly.castRay( { 0.0f, 0.0f }, { 4.0f, 0.0f }, queryFilter{ 4, 2 },
        [&]( shapeId, vec2, vec2, float )
        {
            ++ignored;
            return -1.0f;
        } );
    check( ignored == 1, "-1 callback ignores hit without terminating" );

    const auto failOnCall = []( shapeId, vec2, vec2, float )
    {
        check( false, "invalid ray must never call callback" );
        return 1.0f;
    };

    const float nan = std::numeric_limits<float>::quiet_NaN();
    readonly.castRay( { nan, 0.0f }, { 4.0f, 0.0f }, failOnCall );
    readonly.castRay( { 0.0f, 0.0f }, {}, failOnCall );
    readonly.castRay( { 0.0f, 0.0f }, { std::numeric_limits<float>::infinity(), 0.0f }, failOnCall );
}

} // namespace

int main()
{
    checkGeometry();
    checkWorldQueries();
    checkFiltersAndCallbacks();
}
