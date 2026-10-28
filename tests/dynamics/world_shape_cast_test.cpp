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

void checkRotatingSweep()
{
    constexpr float quarterTurn = 1.5707963267948966f;
    world simulation{};
    const bodyId fixed = simulation.CreateBody( bodyType::Static, { { 1.4f, 1.4f }, {} } );
    const shapeId target = simulation.CreateShape( fixed, circle2{ {}, 0.45f } );

    // 양 끝에서는 겹치지 않지만 중간 자세에서 로컬 오프셋 원이 목표를 통과함.
    worldShapeSweepInput2 input{};
    input.geometry = circle2{ { 2.0f, 0.0f }, 0.3f };
    input.rotationRadians = quarterTurn;

    const auto hit = simulation.castShapeSweepClosest( input );
    check( hit.hit && hit.shape == target, "Rotation-only World sweep must detect an intermediate collision" );
    check( hit.fraction > 0.05f && hit.fraction < 0.95f &&
        IsFinite( hit.point ) && IsFinite( hit.normal ) && LengthSquared( hit.normal ) > 0.5f,
        "Rotating sweep must provide finite world hit information" );

    int callbackCount = 0;
    simulation.castShapeSweep( input, [&]( shapeId id, vec2, vec2, float fraction )
    {
        check( id == target && fraction > 0.0f, "Rotational callback reports a live target" );
        ++callbackCount;
        return 0.0f;
    } );
    check( callbackCount == 1, "Rotational callback returning zero stops the query" );

    // TOI 입력의 반대 회전 방향과 translation 동시 적용.
    world negative{};
    const bodyId negativeBody = negative.CreateBody( bodyType::Static, { { 1.4f, -1.4f }, {} } );
    const shapeId negativeShape = negative.CreateShape( negativeBody, circle2{ {}, 0.45f } );
    input.rotationRadians = -quarterTurn;
    check( negative.castShapeSweepClosest( input ).shape == negativeShape,
        "Negative rotation must trace the opposite arc" );

    world translated{};
    const bodyId translatedBody = translated.CreateBody( bodyType::Static, { { 2.0f, 0.0f }, {} } );
    const shapeId translatedShape = translated.CreateShape( translatedBody, circle2{ {}, 0.5f } );
    input.geometry = circle2{ { 0.8f, 0.0f }, 0.25f };
    input.transform.position = { -3.0f, 0.0f };
    input.translation = { 5.0f, 0.0f };
    input.rotationRadians = 0.5f;
    const auto mixed = translated.castShapeSweepClosest( input );
    check( mixed.hit && mixed.shape == translatedShape && mixed.fraction > 0.0f && mixed.fraction < 1.0f,
        "Mixed translation and rotation must use TOI" );

    // 회전 0은 기존 선형 ShapeCast를 사용하며 결과 계약이 같음.
    input.rotationRadians = 0.0f;
    input.geometry = circle2{ {}, 0.5f };
    const auto linear = translated.castShapeSweepClosest( input );
    const auto original = translated.castShapeClosest( { input.geometry, input.transform, input.translation, false } );
    check( linear.hit && linear.shape == original.shape && linear.fraction == original.fraction,
        "Zero angular sweep must preserve the existing linear API" );

    // 초기 겹침은 일반 query만 fraction 0으로 보고하며 closest에서 무시함.
    world overlapped{};
    const bodyId overlappingBody = overlapped.CreateBody( bodyType::Static, { { 2.0f, 0.0f }, {} } );
    const shapeId overlapShape = overlapped.CreateShape( overlappingBody, circle2{ {}, 0.4f } );
    input = {};
    input.geometry = circle2{ { 2.0f, 0.0f }, 0.4f };
    input.rotationRadians = quarterTurn;
    int startCount = 0;
    overlapped.castShapeSweep( input, [&]( shapeId id, vec2 point, vec2 normal, float fraction )
    {
        check( id == overlapShape && fraction == 0.0f && LengthSquared( normal ) == 0.0f &&
            IsFinite( point ), "Initial rotating overlap must report zero normal and fraction" );
        ++startCount;
        return -1.0f;
    } );
    check( startCount == 1 && !overlapped.castShapeSweepClosest( input ).hit,
        "Rotating closest sweep ignores initial overlap" );

    // Query Filter, Sensor 및 move-only callback을 그대로 지원함.
    world filtered{};
    const bodyId filteredBody = filtered.CreateBody( bodyType::Kinematic, { { 1.4f, 1.4f }, {} } );
    const shapeId included = filtered.CreateSensorShape( filteredBody, circle2{ {}, 0.45f },
        collisionFilter{ 2, 4, 0 } );
    input = {};
    input.geometry = circle2{ { 2.0f, 0.0f }, 0.3f };
    input.rotationRadians = quarterTurn;
    int filteredCount = 0;
    filtered.castShapeSweep( input, queryFilter{ 4, 2 },
        [owned = std::make_unique<int>( 0 ), &filteredCount]( shapeId id, vec2, vec2, float ) mutable
        {
            ++*owned;
            ++filteredCount;
            check( id.index1 != 0, "Rotating query handle must be valid" );
            return 1.0f;
        } );
    check( filteredCount == 1 && filtered.castShapeSweepClosest( input, queryFilter{ 4, 2 } ).shape == included,
        "Rotating query includes sensor and honors query filter" );
    check( !filtered.castShapeSweepClosest( input ).hit, "Default filter excludes masked rotating query target" );
    filtered.DestroyBody( filteredBody );
    check( !filtered.castShapeSweepClosest( input, queryFilter{ 4, 2 } ).hit,
        "Destroyed rotating query target must disappear" );

    // CCD one-sided 조기 종료 전에 시작 겹침을 보고해야 함 (앞면에서 멀어지는 회전).
    world chainWorld{};
    const bodyId chainBody = chainWorld.CreateBody();
    const shapeId edge = chainWorld.CreateShape( chainBody, chainSegment2{
        { -4.0f, 0.0f }, { { -2.0f, 0.0f }, { 2.0f, 0.0f } }, { 4.0f, 0.0f } } );
    worldShapeSweepInput2 chainInput{};
    chainInput.geometry = circle2{ {}, 0.4f };
    chainInput.transform.position = { 0.0f, -0.2f };
    chainInput.translation = { 0.0f, -1.0f };
    chainInput.rotationRadians = 0.5f;
    int chainCallbacks = 0;
    chainWorld.castShapeSweep( chainInput, [&]( shapeId id, vec2 point, vec2 normal, float fraction )
    {
        check( id == edge && fraction == 0.0f && LengthSquared( normal ) == 0.0f &&
            IsFinite( point ), "Moving-away initial front-chain overlap must remain reported" );
        ++chainCallbacks;
        return 1.0f;
    } );
    check( chainCallbacks == 1 && !chainWorld.castShapeSweepClosest( chainInput ).hit,
        "Front-chain overlap must be ignored only by closest sweep" );
    chainInput.transform.position.y = 0.2f;
    chainInput.translation.y = 1.0f;
    check( !chainWorld.castShapeSweepClosest( chainInput ).hit,
        "Rotating chain sweep must reject an initial back-face overlap" );
    int backCallbacks = 0;
    chainWorld.castShapeSweep( chainInput, [&]( shapeId, vec2, vec2, float )
    {
        ++backCallbacks;
        return 1.0f;
    } );
    check( backCallbacks == 0, "Rotating query must not report chain back faces" );

    // 짧은 접근은 CCD의 minimum-motion heuristic 대상이지만 공간 조회는 hit을 반환해야 함.
    // 중심에 있는 원은 회전해도 같은 geometry이므로 선형과 회전 결과가 일치해야 함.
    chainInput.transform.position.y = -0.42f;
    chainInput.translation.y = 0.05f;
    chainInput.rotationRadians = 0.5f;
    const auto shortRotation = chainWorld.castShapeSweepClosest( chainInput );
    const auto shortLinear = chainWorld.castShapeClosest(
        { chainInput.geometry, chainInput.transform, chainInput.translation, false } );
    check( shortRotation.hit && shortRotation.shape == edge &&
        shortLinear.hit && shortLinear.shape == edge,
        "Short front-chain approach must hit regardless of centered circle rotation" );
    check( shortRotation.fraction > 0.0f && shortRotation.fraction < 1.0f &&
        std::fabs( shortRotation.fraction - shortLinear.fraction ) < 0.05f &&
        Dot( shortRotation.normal, vec2{ 0.0f, -1.0f } ) > 0.9f,
        "Short rotating chain hit must preserve a valid fraction and front normal" );

    chainInput.transform.position.y = 0.42f;
    chainInput.translation.y = -0.05f;
    check( !chainWorld.castShapeSweepClosest( chainInput ).hit,
        "Short back-chain approach must remain one-sided after bypassing CCD heuristic" );

    const auto invalidCallback = []( shapeId, vec2, vec2, float )
    {
        check( false, "Invalid rotating ShapeCast must not invoke callback" );
        return 1.0f;
    };

    input = {};
    input.geometry = circle2{ { 2.0f, 0.0f }, 0.3f };
    input.rotationRadians = 2.0f;
    simulation.castShapeSweep( input, invalidCallback );
    input.rotationRadians = std::numeric_limits<float>::quiet_NaN();
    simulation.castShapeSweep( input, invalidCallback );
    input.rotationRadians = 0.0f;
    simulation.castShapeSweep( input, invalidCallback );
    input.rotationRadians = quarterTurn;
    input.transform.rotation = { 0.0f, 0.0f };
    simulation.castShapeSweep( input, invalidCallback );
    input.transform.rotation = {};
    input.geometry = circle2{ {}, -1.0f };
    simulation.castShapeSweep( input, invalidCallback );

    // MakeShapeProxy는 polygon vertexCount를 assert하므로 반드시 그 전에 거절해야 함.
    input.geometry = polygon2{};
    simulation.castShapeSweep( input, invalidCallback );
    input.rotationRadians = 0.0f;
    input.translation = { 1.0f, 0.0f };
    simulation.castShapeSweep( input, invalidCallback );
    polygon2 malformed{};
    malformed.vertexCount = MAX_POLYGON_VERTICES + 1;
    input.geometry = malformed;
    input.rotationRadians = quarterTurn;
    simulation.castShapeSweep( input, invalidCallback );
}

} // namespace

int main()
{
    checkBodyTypesAndClosest();
    checkFilteringAndCallbacks();
    checkSweepGeometryAndNormals();
    checkClippingAndEncroachment();
    checkRotatingSweep();
}
