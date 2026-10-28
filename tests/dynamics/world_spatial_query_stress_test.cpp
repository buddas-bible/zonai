#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <optional>
#include <vector>

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

// 재현 가능한 입력을 생성하며 표준 라이브러리 random 구현에 의존하지 않음.
struct randomSequence
{
    std::uint32_t state = 0xC0FFEE42u;

    std::uint32_t next()
    {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return state;
    }

    float between( float low, float high )
    {
        return low + ( high - low ) * static_cast<float>( next() & 0x00FFFFFFu ) / 16777216.0f;
    }

    vec2 point( float low, float high )
    {
        return { between( low, high ), between( low, high ) };
    }
};

struct target
{
    bodyId body{};
    shapeId shape{};
    vec2 localCenter{};
    float radius = 0.0f;
    bool live = true;
};

struct rayHit
{
    shapeId shape{};
    float fraction = 0.0f;
};

bool has( const std::vector<shapeId>& ids, shapeId id )
{
    return std::find( ids.begin(), ids.end(), id ) != ids.end();
}

const rayHit* findRay( const std::vector<rayHit>& hits, shapeId id )
{
    const auto it = std::find_if( hits.begin(), hits.end(), [id]( const rayHit& hit )
    {
        return hit.shape == id;
    } );
    return it == hits.end() ? nullptr : &*it;
}

vec2 centerOf( const world& simulation, const target& object )
{
    return TransformPoint( simulation.GetBodyTransform( object.body ), object.localCenter );
}

// World RayCast와 독립된 circle/ray 2차방정식 해. World와 같은 초기 겹침=0 규약을 사용함.
std::optional<float> rayCircleFraction( vec2 origin, vec2 translation, vec2 center, float radius )
{
    const double dx = static_cast<double>( origin.x ) - center.x;
    const double dy = static_cast<double>( origin.y ) - center.y;
    const double vx = translation.x;
    const double vy = translation.y;
    const double a = vx * vx + vy * vy;
    const double b = dx * vx + dy * vy;
    const double c = dx * dx + dy * dy - static_cast<double>( radius ) * radius;

    if( c <= 0.0 ) return 0.0f;

    const double discriminant = b * b - a * c;
    if( discriminant < 0.0 ) return std::nullopt;

    const double fraction = ( -b - std::sqrt( discriminant ) ) / a;
    if( fraction < 0.0 || fraction > 1.0 ) return std::nullopt;

    return static_cast<float>( fraction );
}

void verifyOverlapQueries( const world& simulation, const std::vector<target>& targets,
    randomSequence& random, int phase, std::size_t& queryCount )
{
    constexpr std::array filters{
        queryFilter{},
        queryFilter{ 1, 1 },
        queryFilter{ 1, 2 },
        queryFilter{ 4, 3 }
    };

    for( int sample = 0; sample < 180; ++sample )
    {
        const queryFilter filter = filters[static_cast<std::size_t>( sample ) % filters.size()];
        const vec2 position = random.point( -12.0f, 12.0f );
        const float radius = random.between( 0.15f, 0.85f );
        const shapeGeometry geometry = circle2{ {}, radius };
        const transform2 transform{ position, {} };
        const aabb2 bounds = ComputeShapeAABB( geometry, transform );

        std::vector<shapeId> broad;
        std::vector<shapeId> exact;
        simulation.overlapAABB( bounds, filter, [&]( shapeId id )
        {
            check( simulation.IsValid( id ) && !has( broad, id ), "AABB query returned duplicate/stale handle" );
            broad.push_back( id );
            return true;
        } );

        simulation.overlapShape( geometry, transform, filter, [&]( shapeId id )
        {
            check( simulation.IsValid( id ) && !has( exact, id ), "Shape query returned duplicate/stale handle" );
            exact.push_back( id );
            return true;
        } );

        std::size_t expectedBroad = 0;
        std::size_t expectedExact = 0;

        for( const target& object : targets )
        {
            if( !object.live ) continue;
            const bool allowed = ShouldQueryShape( filter, simulation.GetShape( object.shape ).filter );
            const bool broadCandidate = allowed && Overlaps( bounds, simulation.GetShapeAABB( object.shape ) );
            if( broadCandidate ) ++expectedBroad;

            if( has( broad, object.shape ) != broadCandidate )
            {
                std::fprintf( stderr, "AABB oracle mismatch: phase=%d sample=%d center=(%.5f,%.5f)\n",
                    phase, sample, position.x, position.y );
                check( false, "AABB candidate differs from stored speculative bounds" );
            }

            const vec2 difference = centerOf( simulation, object ) - position;
            const float sumRadius = radius + object.radius;
            const float distanceSquared = LengthSquared( difference );
            const bool analyticOverlap = allowed && distanceSquared <= sumRadius * sumRadius;
            const bool reported = has( exact, object.shape );

            // 접선 직전/직후의 극소 float 오차만 독립 oracle 비교에서 제외함.
            const bool ambiguous = std::fabs( std::sqrt( distanceSquared ) - sumRadius ) < 0.0002f;
            if( !ambiguous && reported != analyticOverlap )
            {
                std::fprintf( stderr,
                    "Shape oracle mismatch: phase=%d sample=%d query=(%.5f,%.5f) target=(%.5f,%.5f) qr=%.5f tr=%.5f reported=%d expected=%d\n",
                    phase, sample, position.x, position.y,
                    centerOf( simulation, object ).x, centerOf( simulation, object ).y,
                    radius, object.radius, static_cast<int>( reported ), static_cast<int>( analyticOverlap ) );
                check( false, "GJK Shape overlap differs from circle distance oracle" );
            }

            if( reported )
            {
                ++expectedExact;
                check( broadCandidate, "Exact overlap must be a subset of AABB candidates" );
            }
        }

        check( broad.size() == expectedBroad && exact.size() == expectedExact,
            "World spatial queries must report each live shape exactly once" );

        if( !exact.empty() )
        {
            int calls = 0;
            simulation.overlapShape( geometry, transform, filter, [&]( shapeId )
            {
                ++calls;
                return false;
            } );
            check( calls == 1, "A false overlap callback must stop across all Body trees" );
        }

        queryCount += 2;
    }
}

void verifyRayQueries( const world& simulation, const std::vector<target>& targets,
    randomSequence& random, int phase, std::size_t& queryCount )
{
    constexpr std::array filters{
        queryFilter{}, queryFilter{ 1, 1 }, queryFilter{ 1, 2 }, queryFilter{ 4, 3 }
    };

    for( int sample = 0; sample < 80; ++sample )
    {
        const queryFilter filter = filters[static_cast<std::size_t>( sample ) % filters.size()];
        const vec2 origin = random.point( -12.0f, 12.0f );
        const vec2 destination = random.point( -12.0f, 12.0f );
        const vec2 translation = destination - origin;
        if( LengthSquared( translation ) < 1.0f ) continue;

        std::vector<rayHit> found;

        simulation.castRay( origin, translation, filter,
            [&]( shapeId id, vec2 point, vec2 normal, float fraction )
            {
                check( simulation.IsValid( id ) && findRay( found, id ) == nullptr,
                    "Ray callback returned duplicate/stale shape" );
                check( IsFinite( point ) && IsFinite( normal ) && fraction >= 0.0f && fraction <= 1.0f,
                    "Ray callback must provide finite contact data" );
                check( LengthSquared( point - ( origin + translation * fraction ) ) < 0.0001f,
                    "Ray callback point must lie on the input segment" );
                found.push_back( { id, fraction } );
                return 1.0f;
            } );

        bool expectedClosest = false;
        shapeId closestId{};
        float closestFraction = 1.0f;
        std::size_t expectedCount = 0;

        for( const target& object : targets )
        {
            if( !object.live ) continue;
            const bool allowed = ShouldQueryShape( filter, simulation.GetShape( object.shape ).filter );
            const auto oracle = allowed ? rayCircleFraction( origin, translation, centerOf( simulation, object ), object.radius ) : std::nullopt;
            const rayHit* actual = findRay( found, object.shape );

            if( oracle )
            {
                ++expectedCount;
                if( *oracle > 0.0f && ( !expectedClosest || *oracle < closestFraction ) )
                {
                    expectedClosest = true;
                    closestFraction = *oracle;
                    closestId = object.shape;
                }
            }

            if( oracle.has_value() != ( actual != nullptr ) ||
                ( oracle && actual && std::fabs( *oracle - actual->fraction ) > 0.001f ) )
            {
                std::fprintf( stderr, "Ray oracle mismatch: phase=%d sample=%d id=%u\n",
                    phase, sample, object.shape.index1 );
                check( false, "RayCast differs from independent circle intersection" );
            }
        }

        check( found.size() == expectedCount, "Ray callbacks must visit each actual circle hit once" );

        const worldRayCastResult2 closest = simulation.castRayClosest( origin, translation, filter );
        check( closest.hit == expectedClosest, "Ray closest miss/hit differs from oracle" );
        if( expectedClosest )
        {
            if( closest.shape != closestId || std::fabs( closest.fraction - closestFraction ) > 0.001f )
            {
                std::fprintf( stderr, "Ray closest mismatch: phase=%d sample=%d\n", phase, sample );
                check( false, "RayClosest must choose nearest positive intersection" );
            }
        }

        queryCount += 2;
    }
}

void verifyShapeCastConsistency( const world& simulation, randomSequence& random, std::size_t& queryCount )
{
    for( int i = 0; i < 45; ++i )
    {
        const vec2 start = random.point( -12.0f, 12.0f );
        const vec2 end = random.point( -12.0f, 12.0f );
        const vec2 translation = end - start;
        if( LengthSquared( translation ) < 1.0f ) continue;

        worldShapeCastInput2 input{};
        input.geometry = circle2{ {}, 0.23f };
        input.transform.position = start;
        input.translation = translation;
        const queryFilter filter = ( i % 2 == 0 ) ? queryFilter{} : queryFilter{ 1, 2 };

        const worldShapeCastResult2 hit = simulation.castShapeClosest( input, filter );
        if( hit.hit )
        {
            check( simulation.IsValid( hit.shape ) && hit.fraction > 0.0f && hit.fraction < 1.0f &&
                IsFinite( hit.normal ) && IsFinite( hit.point ),
                "ShapeCast closest must return finite, live collision data" );

            const transform2 atHit{ start + translation * hit.fraction, {} };
            bool foundTarget = false;
            simulation.overlapShape( input.geometry, atHit, filter, [&]( shapeId id )
            {
                if( id == hit.shape ) foundTarget = true;
                return true;
            } );

            check( foundTarget, "ShapeCast hit position must overlap its reported target" );
        }

        queryCount += 2;
    }
}

} // namespace

int main()
{
    world simulation{};
    simulation.SetGravity( {} );

    randomSequence random{};
    std::vector<target> targets;
    targets.reserve( 108 );

    // 세 Tree에 고르게 분산하며 각 Body의 회전과 local center도 다르게 구성함.
    for( int i = 0; i < 108; ++i )
    {
        const bodyType type = static_cast<bodyType>( i % static_cast<int>( bodyType::Count ) );
        const float x = ( static_cast<float>( i % 12 ) - 5.5f ) * 1.8f + random.between( -0.1f, 0.1f );
        const float y = ( static_cast<float>( i / 12 ) - 4.0f ) * 1.8f + random.between( -0.1f, 0.1f );
        const float angle = random.between( -3.0f, 3.0f );
        const bodyId body = simulation.CreateBody( type, { { x, y }, rot2::FromRadians( angle ) } );

        const vec2 localCenter = random.point( -0.2f, 0.2f );
        const float radius = random.between( 0.18f, 0.48f );
        const std::uint64_t category = i % 4 == 0 ? 2 : 1;
        const std::uint64_t mask = i % 6 == 0 ? 4 : ~std::uint64_t{ 0 };
        const collisionFilter filter{ category, mask, i % 9 == 0 ? -5 : 0 };

        const shapeId shape = i % 9 == 0
            ? simulation.CreateSensorShape( body, circle2{ localCenter, radius }, filter )
            : simulation.CreateShape( body, circle2{ localCenter, radius }, filter );

        check( simulation.IsValid( body ) && simulation.IsValid( shape ), "Stress fixture creation failed" );
        targets.push_back( { body, shape, localCenter, radius, true } );
    }

    std::size_t queryCount = 0;
    const auto startTime = std::chrono::steady_clock::now();

    for( int phase = 0; phase < 3; ++phase )
    {
        if( phase == 1 )
        {
            for( std::size_t i = 0; i < targets.size(); ++i )
            {
                target& object = targets[i];
                if( i % 4 == 0 )
                {
                    simulation.SetBodyTransform( object.body,
                        { random.point( -10.0f, 10.0f ), rot2::FromRadians( random.between( -3.0f, 3.0f ) ) } );
                }

                if( i % 10 == 0 )
                    simulation.SetShapeFilter( object.shape, collisionFilter{ 2, ~std::uint64_t{ 0 }, -9 } );
            }
        }
        else if( phase == 2 )
        {
            for( std::size_t i = 0; i < targets.size(); ++i )
            {
                target& object = targets[i];
                if( i % 9 != 0 ) continue;

                const shapeId old = object.shape;
                simulation.DestroyShape( old );
                check( !simulation.IsValid( old ), "Destroyed handle must be invalid" );

                if( i % 18 == 0 )
                {
                    object.shape = simulation.CreateShape( object.body,
                        circle2{ object.localCenter, object.radius }, collisionFilter{ 1, ~std::uint64_t{ 0 }, 0 } );
                    check( simulation.IsValid( object.shape ) && !simulation.IsValid( old ),
                        "Reused storage must reject stale generation" );
                }
                else
                {
                    simulation.DestroyBody( object.body );
                    object.live = false;
                    check( !simulation.IsValid( object.body ), "Destroyed owner must be invalid" );
                }
            }
        }

        verifyOverlapQueries( simulation, targets, random, phase, queryCount );
        verifyRayQueries( simulation, targets, random, phase, queryCount );
        verifyShapeCastConsistency( simulation, random, queryCount );
    }

    const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - startTime ).count();

    // 회귀 실패는 oracle 불일치로만 판정. 실행 시간은 플랫폼/부하에 따라 달라 참고값으로 남김.
    std::printf( "Spatial query differential: %zu targets, %zu queries, %lld us (informational)\n",
        targets.size(), queryCount, static_cast<long long>( elapsed ) );
}
