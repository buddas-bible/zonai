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

template <typename Geometry>
std::vector<shapeId> overlapping( const world& simulation, const Geometry& geometry,
    const transform2& transform = {}, queryFilter filter = {} )
{
    std::vector<shapeId> results;

    simulation.overlapShape( shapeGeometry{ geometry }, transform, filter,
        [&]( shapeId id )
        {
            check( simulation.IsValid( id ), "OverlapShape must return a live World handle" );
            results.push_back( id );
            return true;
        } );

    return results;
}

void checkExactGeometry()
{
    world simulation{};
    const bodyId body = simulation.CreateBody();
    const shapeId circle = simulation.CreateShape( body, circle2{ {}, 0.5f } );

    // 두 AABB는 겹치지만 중심 간 거리는 두 원의 radius 합보다 큼.
    const transform2 diagonal{ { 0.9f, 0.9f }, {} };
    int aabbCount = 0;

    simulation.overlapAABB( ComputeShapeAABB( circle2{ {}, 0.5f }, diagonal ),
        [&]( shapeId )
        {
            ++aabbCount;
            return true;
        } );
    check( aabbCount == 1, "AABB query should include the diagonal candidate" );
    check( overlapping( simulation, circle2{ {}, 0.5f }, diagonal ).empty(),
        "Exact shape overlap must reject AABB-only diagonal false positives" );

    check( overlapping( simulation, circle2{ {}, 0.5f }, { { 0.7f, 0.7f }, {} } ).size() == 1,
        "Overlapping circle surfaces must be reported" );
    check( overlapping( simulation, circle2{ {}, 0.5f }, { { 1.0f, 0.0f }, {} } ).size() == 1,
        "Exactly touching circle surfaces must count as overlap" );
    check( overlapping( simulation, circle2{ {}, 0.5f }, { { 1.01f, 0.0f }, {} } ).empty(),
        "Separated circles must not be reported" );

    // Rectangle AABB의 빈 모서리와 회전된 Body local circle.
    const bodyId rotated = simulation.CreateBody( bodyType::Static,
        { { 4.0f, 1.0f }, rot2::FromRadians( 1.57079632679f ) } );
    const shapeId moved = simulation.CreateShape( rotated, circle2{ { 1.0f, 0.0f }, 0.35f } );
    const auto movedHits = overlapping( simulation, circle2{ {}, 0.2f }, { { 4.0f, 2.0f }, {} } );
    check( movedHits.size() == 1 && movedHits.front() == moved,
        "OverlapShape must apply the target Body local/world transform" );

    const bodyId boxBody = simulation.CreateBody( bodyType::Kinematic, { { 7.0f, 1.0f }, rot2::FromRadians( 0.78539816339f ) } );
    const shapeId box = simulation.CreateShape( boxBody, MakeBox( { 0.8f, 0.5f } ) );
    const auto boxHits = overlapping( simulation, capsule2{ { -0.3f, 0.0f }, { 0.3f, 0.0f }, 0.25f }, { { 7.0f, 1.0f }, {} } );
    check( boxHits.size() == 1 && boxHits.front() == box,
        "Capsule query must use GJK against rotated convex polygon" );

    const auto polygonHits = overlapping( simulation, MakeBox( { 0.4f, 0.4f } ), { { 0.0f, 0.0f }, {} } );
    check( polygonHits.size() == 1 && polygonHits.front() == circle,
        "Convex polygon query must overlap a circular target" );

    // GJK는 tangent contact를 1e-7 수준의 양수 거리로 반환할 수 있음.
    // Sensor와 같은 10*float epsilon 안의 오차는 겹침으로 취급함.
    {
        world tangentWorld{};
        const bodyId target = tangentWorld.CreateBody();
        const shapeId rectangle = tangentWorld.CreateShape( target, MakeBox( { 1.0f, 1.0f } ) );

        const auto tangent = overlapping( tangentWorld, circle2{ {}, 0.5f }, { { 1.5f, 0.45f }, {} } );
        check( tangent.size() == 1 && tangent.front() == rectangle,
            "Polygon/circle tangency must tolerate numerical GJK residual distance" );

        check( overlapping( tangentWorld, circle2{ {}, 0.5f },
            { { 1.5001f, 0.45f }, {} } ).empty(),
            "Small but meaningful separation must not be reported as overlap" );
    }

    simulation.SetBodyTransform( boxBody, { { 20.0f, 1.0f }, {} } );
    check( overlapping( simulation, capsule2{ { -0.3f, 0.0f }, { 0.3f, 0.0f }, 0.25f },
        { { 7.0f, 1.0f }, {} } ).empty(),
        "Transform updates remove stale query hits immediately" );
    simulation.DestroyBody( rotated );
    check( overlapping( simulation, circle2{ {}, 0.2f }, { { 4.0f, 2.0f }, {} } ).empty(),
        "Destroyed Body must remove its query shapes" );
}

void checkBodyTypesAndSensors()
{
    world simulation{};

    const bodyId fixed = simulation.CreateBody( bodyType::Static, { { -2.0f, 0.0f }, {} } );
    const shapeId staticShape = simulation.CreateShape( fixed, circle2{ {}, 0.5f } );
    const shapeId extra = simulation.CreateShape( fixed, circle2{ { 0.0f, 0.5f }, 0.3f } );
    const bodyId moving = simulation.CreateBody( bodyType::Dynamic );
    const shapeId dynamicShape = simulation.CreateShape( moving, MakeBox( { 0.4f, 0.4f } ) );
    const bodyId kinematic = simulation.CreateBody( bodyType::Kinematic, { { 2.0f, 0.0f }, {} } );
    const shapeId sensor = simulation.CreateSensorShape( kinematic, circle2{ {}, 0.5f } );

    const capsule2 capsule{ { -3.0f, 0.0f }, { 3.0f, 0.0f }, 0.4f };
    const auto hits = overlapping( simulation, capsule );
    check( hits.size() == 4, "Static/Dynamic/Kinematic/Sensor + multiple shapes must be queried once each" );

    for( const shapeId id : { staticShape, extra, dynamicShape, sensor } )
        check( std::count( hits.begin(), hits.end(), id ) == 1, "Each shape must appear exactly once" );

    int stopCount = 0;
    simulation.overlapShape( capsule, {},
        [&]( shapeId )
        {
            ++stopCount;
            return false;
        } );
    check( stopCount == 1, "Callback false must stop across all body type trees" );

    const world& readonly = simulation;
    int calls = 0;
    auto moveOnly = [guard = std::make_unique<int>( 0 ), &calls]( shapeId id ) mutable
    {
        ++*guard;
        ++calls;
        check( id.index1 != 0, "Move-only query callback must receive live ID" );
        return true;
    };
    readonly.overlapShape( capsule, {}, std::move( moveOnly ) );
    check( calls == 4, "Const World accepts move-only callable without copying it" );

    simulation.DestroyShape( dynamicShape );
    check( overlapping( simulation, capsule ).size() == 3, "Destroyed shape must disappear from overlap" );
    simulation.DestroyBody( kinematic );
    check( overlapping( simulation, capsule ).size() == 2, "Destroyed sensor owner must disappear" );

    world unrelated{};
    const bodyId otherBody = unrelated.CreateBody();
    ( void )unrelated.CreateShape( otherBody, circle2{ {}, 0.5f } );
    check( !unrelated.IsValid( staticShape ), "World shape handle token must remain isolated" );
}

void checkFilters()
{
    world simulation{};
    const bodyId fixed = simulation.CreateBody();
    const shapeId included = simulation.CreateShape( fixed, circle2{ {}, 0.5f }, collisionFilter{ 2, 4, -9 } );
    ( void )simulation.CreateShape( fixed, circle2{ {}, 0.5f }, collisionFilter{ 8, 4, -9 } );
    ( void )simulation.CreateShape( fixed, circle2{ {}, 0.5f }, collisionFilter{ 2, 0, 9 } );

    collisionMatrix matrix{};
    matrix.setPair( 1, 2, false );
    simulation.setCollisionMatrix( matrix );

    const auto hits = overlapping( simulation, circle2{ {}, 0.5f }, {}, queryFilter{ 4, 2 } );
    check( hits.size() == 1 && hits.front() == included,
        "Query category/mask must be checked both ways, independent of group and matrix" );
    check( overlapping( simulation, circle2{ {}, 0.5f } ).empty(),
        "Default query category should be excluded by target mask" );
}

void checkSegmentsAndInvalidInputs()
{
    world simulation{};
    const bodyId fixed = simulation.CreateBody();
    const chainSegment2 chain{ { -3.0f, 0.0f }, { { -2.0f, 0.0f }, { 2.0f, 0.0f } }, { 3.0f, 0.0f } };
    const shapeId chainId = simulation.CreateShape( fixed, chain );
    check( simulation.IsValid( chainId ), "Chain segment target must be valid" );

    const auto chainHits = overlapping( simulation, circle2{ {}, 0.25f }, { { 0.0f, 0.1f }, {} } );
    check( chainHits.size() == 1 && chainHits.front() == chainId,
        "Overlap uses the owned Chain edge as a symmetric distance proxy" );
    check( overlapping( simulation, segment2{ { -1.0f, 0.0f }, { 1.0f, 0.0f } } ).size() == 1,
        "Segment query can overlap a Chain edge" );
    check( overlapping( simulation, circle2{ {}, 0.25f }, { { 0.0f, 0.7f }, {} } ).empty(),
        "Separated geometry next to Chain does not count as overlap" );

    const auto shouldNeverRun = []( shapeId )
    {
        check( false, "Invalid OverlapShape input must never call callback" );
        return true;
    };

    const float invalidNumber = std::numeric_limits<float>::quiet_NaN();
    simulation.overlapShape( shapeGeometry{}, {}, shouldNeverRun );
    simulation.overlapShape( circle2{ {}, -1.0f }, {}, shouldNeverRun );
    simulation.overlapShape( circle2{ { invalidNumber, 0.0f }, 1.0f }, {}, shouldNeverRun );
    simulation.overlapShape( circle2{ {}, 0.5f }, { { invalidNumber, 0.0f }, {} }, shouldNeverRun );
    simulation.overlapShape( circle2{ {}, 0.5f },
        { {}, { std::numeric_limits<float>::infinity(), 0.0f } }, shouldNeverRun );
    polygon2 invalidPolygon{};
    simulation.overlapShape( invalidPolygon, {}, shouldNeverRun );

    // Two circles at identical locations must work even when GJK has no separating direction.
    const shapeId coincident = simulation.CreateShape( fixed, circle2{ {}, 0.3f } );
    check( overlapping( simulation, circle2{ {}, 0.3f } ).size() == 2,
        "GJK must recognize coincident convex shapes and keep multiple handles" );
    simulation.DestroyShape( coincident );
}


void checkPointOverlap()
{
    world simulation{};
    const bodyId fixed = simulation.CreateBody( bodyType::Static );
    const shapeId circle = simulation.CreateShape( fixed, circle2{ {}, 1.0f } );
    // AABB 후보에 포함되더라도 면적이 없는 선분/Chain은 점 안쪽 판정이 없음.
    ( void )simulation.CreateShape( fixed, segment2{ { -2.0f, 0.0f }, { 2.0f, 0.0f } } );
    ( void )simulation.CreateShape( fixed, chainSegment2{
        { -3.0f, 0.0f }, { { -2.0f, 0.0f }, { 2.0f, 0.0f } }, { 3.0f, 0.0f } } );

    const bodyId moving = simulation.CreateBody( bodyType::Dynamic,
        { { 3.0f, 0.0f }, rot2::FromRadians( 0.78539816339f ) } );
    const shapeId polygon = simulation.CreateShape( moving, MakeBox( { 1.0f, 0.4f } ) );
    const bodyId kinematic = simulation.CreateBody( bodyType::Kinematic, { { 6.0f, 0.0f }, {} } );
    const shapeId sensor = simulation.CreateSensorShape( kinematic,
        capsule2{ { -0.5f, 0.0f }, { 0.5f, 0.0f }, 0.25f } );

    const auto at = [&]( vec2 point, queryFilter filter = {} )
    {
        std::vector<shapeId> result;
        simulation.overlapPoint( point, filter,
            [&]( shapeId id )
            {
                check( simulation.IsValid( id ), "OverlapPoint must return live shape handles" );
                result.push_back( id );
                return true;
            } );
        return result;
    };

    check( at( { 0.0f, 0.0f } ) == std::vector<shapeId>{ circle },
        "Point inside circle must ignore coincident zero-area Segment and Chain" );
    check( at( { 0.95f, 0.95f } ).empty(),
        "OverlapPoint must reject a circle AABB corner outside the actual geometry" );

    const vec2 insideBox = TransformPoint( simulation.GetBodyTransform( moving ), { 0.9f, 0.0f } );
    check( at( insideBox ) == std::vector<shapeId>{ polygon },
        "OverlapPoint must transform rotated Dynamic polygon into local space" );
    check( at( { 3.8f, 0.0f } ).empty(),
        "OverlapPoint must reject a point inside the rotated polygon AABB but outside its face" );
    check( at( { 6.0f, 0.0f } ) == std::vector<shapeId>{ sensor },
        "OverlapPoint must include Kinematic sensors" );
    check( at( { 6.0f, 0.26f } ).empty(),
        "OverlapPoint must reject points just outside Capsule radius" );

    const bodyId second = simulation.CreateBody( bodyType::Dynamic );
    const shapeId overlap = simulation.CreateShape( second, circle2{ {}, 0.3f } );
    const auto both = at( { 0.0f, 0.0f } );
    check( both.size() == 2 && std::count( both.begin(), both.end(), circle ) == 1 &&
        std::count( both.begin(), both.end(), overlap ) == 1,
        "Point overlap must visit overlapping shapes in different Body trees exactly once" );

    int count = 0;
    simulation.overlapPoint( { 0.0f, 0.0f }, [&]( shapeId )
    {
        ++count;
        return false;
    } );
    check( count == 1, "OverlapPoint callback false must stop querying all Body trees" );

    const world& readonly = simulation;
    int moveOnlyCalls = 0;
    readonly.overlapPoint( { 6.0f, 0.0f },
        [guard = std::make_unique<int>( 0 ), &moveOnlyCalls]( shapeId ) mutable
        {
            ++*guard;
            ++moveOnlyCalls;
            return true;
        } );
    check( moveOnlyCalls == 1, "Const World point query accepts a move-only callback" );

    const float invalid = std::numeric_limits<float>::quiet_NaN();
    simulation.overlapPoint( { invalid, 0.0f }, []( shapeId )
    {
        check( false, "Nonfinite query point must not call callback" );
        return true;
    } );

    simulation.SetBodyTransform( moving, { { 10.0f, 0.0f }, {} } );
    check( at( insideBox ).empty(), "Moving a Body must immediately update its point query candidates" );
    simulation.DestroyBody( kinematic );
    check( at( { 6.0f, 0.0f } ).empty(), "Destroyed sensor Body must disappear from point overlap" );
    simulation.DestroyShape( overlap );
    check( at( { 0.0f, 0.0f } ) == std::vector<shapeId>{ circle },
        "Destroyed Dynamic shape must immediately disappear from point overlap" );

    world filtered{};
    const bodyId owner = filtered.CreateBody();
    const shapeId accepted = filtered.CreateShape( owner, circle2{ {}, 1.0f }, collisionFilter{ 2, 4, -3 } );
    ( void )filtered.CreateShape( owner, circle2{ {}, 1.0f }, collisionFilter{ 8, 4, -3 } );
    ( void )filtered.CreateShape( owner, circle2{ {}, 1.0f }, collisionFilter{ 2, 0, -3 } );
    collisionMatrix matrix{};
    matrix.setPair( 2, 4, false );
    filtered.setCollisionMatrix( matrix );

    std::vector<shapeId> filteredHits;
    filtered.overlapPoint( {}, queryFilter{ 4, 2 }, [&]( shapeId id )
    {
        filteredHits.push_back( id );
        return true;
    } );
    check( filteredHits == std::vector<shapeId>{ accepted },
        "OverlapPoint applies reciprocal category/mask, not groupIndex or collisionMatrix" );
    int defaultHits = 0;
    filtered.overlapPoint( {}, [&]( shapeId )
    {
        ++defaultHits;
        return true;
    } );
    check( defaultHits == 0, "Default point QueryFilter must be checked against target mask" );
}

} // namespace

int main()
{
    checkExactGeometry();
    checkBodyTypesAndSensors();
    checkFilters();
    checkSegmentsAndInvalidInputs();
    checkPointOverlap();
}
