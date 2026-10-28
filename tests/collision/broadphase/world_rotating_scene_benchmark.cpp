#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

#include "dynamics/world.h"
#include "geometry/polygon2.h"

using namespace zonai;

namespace
{

constexpr int GRID_SIZE = 18;
constexpr int QUERY_COUNT = 32;
constexpr int MEASURE_ROUNDS = 7;
constexpr int REPEATS = 4;

volatile std::uint64_t benchmarkSink = 0;

void check( bool condition, const char* message )
{
    if( condition ) return;
    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}

struct randomSequence
{
    std::uint32_t state = 0xA7195D42u;

    std::uint32_t next()
    {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return state;
    }

    float between( float min, float max )
    {
        return min + ( max - min ) *
            static_cast<float>( next() >> 8 ) * ( 1.0f / 16777216.0f );
    }
};

struct trial
{
    worldShapeSweepInput2 input{};
    aabb2 candidateBounds{};
};

struct scenario
{
    const char* name = "";
    std::array<trial, QUERY_COUNT> queries{};
};

aabb2 makeCandidateBounds( const worldShapeSweepInput2& input )
{
    const shapeProxy2 proxy = MakeShapeProxy( input.geometry );
    double radius = proxy.radius;

    for( int i = 0; i < proxy.count; ++i )
        radius = std::max( radius,
            std::hypot( static_cast<double>( proxy.points[i].x ),
                static_cast<double>( proxy.points[i].y ) ) + proxy.radius );

    // Extra margin keeps the candidate-only comparison conservative.
    const float extent = static_cast<float>( radius ) + 0.01f;
    const vec2 start = input.transform.position;
    const vec2 end = start + input.translation;
    return { { std::min( start.x, end.x ) - extent, std::min( start.y, end.y ) - extent },
        { std::max( start.x, end.x ) + extent, std::max( start.y, end.y ) + extent } };
}

struct fixture
{
    world simulation{};
    std::array<scenario, 4> scenarios{};

    fixture()
    {
        simulation.SetGravity( {} );
        randomSequence random{};

        for( int y = 0; y < GRID_SIZE; ++y )
        {
            for( int x = 0; x < GRID_SIZE; ++x )
            {
                const int index = y * GRID_SIZE + x;
                const bodyType type = static_cast<bodyType>(
                    index % static_cast<int>( bodyType::Count ) );
                const vec2 center{
                    ( x - ( GRID_SIZE - 1 ) * 0.5f ) * 1.65f +
                        random.between( -0.07f, 0.07f ),
                    ( y - ( GRID_SIZE - 1 ) * 0.5f ) * 1.65f +
                        random.between( -0.07f, 0.07f )
                };
                const transform2 pose{ center,
                    rot2::FromRadians( random.between( -0.6f, 0.6f ) ) };
                const bodyId body = simulation.CreateBody( type, pose );

                shapeGeometry geometry{};
                if( index % 3 == 0 ) geometry = circle2{ {}, 0.38f };
                else if( index % 3 == 1 ) geometry = MakeBox( { 0.4f, 0.28f } );
                else geometry = capsule2{ { -0.34f, 0.0f }, { 0.34f, 0.0f }, 0.21f };

                if( index % 13 == 0 )
                    ( void )simulation.CreateSensorShape( body, geometry,
                        collisionFilter{ 2, ~std::uint64_t{ 0 }, 0 } );
                else
                    ( void )simulation.CreateShape( body, geometry );
            }
        }

        check( simulation.GetShapeCount() == GRID_SIZE * GRID_SIZE,
            "Dense benchmark fixture must contain all 324 shapes" );

        scenarios[0].name = "long_cast";
        scenarios[1].name = "short_cast";
        scenarios[2].name = "far_miss";
        scenarios[3].name = "pure_rotation";

        for( int i = 0; i < QUERY_COUNT; ++i )
        {
            const float jitter = random.between( -0.5f, 0.5f );

            worldShapeSweepInput2 longCast{};
            longCast.geometry = circle2{ { 0.6f, 0.0f }, 0.32f };
            longCast.transform.position = { -16.0f, -13.0f + ( i % 15 ) * 1.75f + jitter };
            longCast.translation = { 32.0f, 0.0f };
            longCast.rotationRadians = ( i % 2 ? -0.7f : 0.7f );
            scenarios[0].queries[i].input = longCast;

            worldShapeSweepInput2 shortCast{};
            shortCast.geometry = MakeBox( { 0.3f, 0.2f } );
            shortCast.transform.position = {
                random.between( -12.0f, 12.0f ),
                random.between( -12.0f, 12.0f ) };
            shortCast.translation = { 2.0f, 1.1f };
            shortCast.rotationRadians = 0.65f;
            scenarios[1].queries[i].input = shortCast;

            worldShapeSweepInput2 miss{};
            miss.geometry = circle2{ { 0.4f, 0.0f }, 0.3f };
            miss.transform.position = { -16.0f, 23.0f + jitter };
            miss.translation = { 32.0f, 0.0f };
            miss.rotationRadians = -0.8f;
            scenarios[2].queries[i].input = miss;

            worldShapeSweepInput2 rotating{};
            polygon2 box = MakeBox( { 0.55f, 0.12f } );
            for( int vertex = 0; vertex < box.vertexCount; ++vertex )
                box.vertices[vertex] += vec2{ 1.5f, 0.0f };
            box.centroid = { 1.5f, 0.0f };
            rotating.geometry = box;
            rotating.transform.position = { random.between( -11.0f, 11.0f ),
                random.between( -11.0f, 11.0f ) };
            rotating.rotationRadians = i % 2 ? -1.3f : 1.3f;
            scenarios[3].queries[i].input = rotating;
        }

        for( scenario& scene : scenarios )
            for( trial& query : scene.queries )
                query.candidateBounds = makeCandidateBounds( query.input );
    }
};

struct totals
{
    std::uint64_t treeCandidates = 0;
    std::uint64_t narrowphaseCandidates = 0;
    std::uint64_t clippedCandidates = 0;
    std::uint64_t distancePrunedCandidates = 0;
    std::uint64_t toiCalls = 0;
    std::uint64_t toiBaseGjk = 0;
    std::uint64_t fallbackAttempts = 0;
    std::uint64_t fallbackGjk = 0;
    std::uint64_t fallbackHits = 0;
    std::uint64_t hits = 0;
    std::uint64_t candidateOnlyHits = 0;

    void add( const worldShapeSweepStats2& stats )
    {
        treeCandidates += stats.treeCandidates;
        narrowphaseCandidates += stats.narrowphaseCandidates;
        clippedCandidates += stats.clippedCandidates;
        distancePrunedCandidates += stats.distancePrunedCandidates;
        toiCalls += stats.toiCalls;
        toiBaseGjk += stats.toi.distanceCalls;
        fallbackAttempts += stats.toi.transientAttempts;
        fallbackGjk += stats.toi.transientDistanceCalls;
        fallbackHits += stats.toi.transientHits;
        hits += stats.hits;
    }
};

totals verify( const fixture& data, const scenario& scene )
{
    totals total{};

    for( const trial& query : scene.queries )
    {
        worldShapeSweepStats2 stats{};
        const auto measured = data.simulation.castShapeSweepClosest( query.input, {}, &stats );
        const auto original = data.simulation.castShapeSweepClosest( query.input );

        check( measured.hit == original.hit && measured.shape == original.shape &&
            measured.fraction == original.fraction &&
            ( !measured.hit || ( LengthSquared( measured.point - original.point ) == 0.0f &&
                LengthSquared( measured.normal - original.normal ) == 0.0f ) ),
            "Optional World sweep instrumentation changed the first hit" );
        check( stats.treeCandidates >= stats.narrowphaseCandidates +
                stats.clippedCandidates + stats.distancePrunedCandidates &&
            stats.narrowphaseCandidates >= stats.toiCalls &&
            stats.toi.distanceCalls >= stats.toiCalls &&
            stats.toi.transientAttempts <= stats.toiCalls &&
            stats.toi.transientHits <= stats.toi.transientAttempts &&
            stats.toi.transientDistanceCalls >= stats.toi.transientHits,
            "World broadphase or TOI stage counters are inconsistent" );

        std::uint64_t candidateCount = 0;
        data.simulation.overlapAABB( query.candidateBounds, queryFilter{},
            [&]( shapeId )
            {
                ++candidateCount;
                return true;
            } );
        check( candidateCount >= static_cast<std::uint64_t>( stats.narrowphaseCandidates ),
            "Conservative candidate-only AABB query must include all narrowphase candidates" );
        total.candidateOnlyHits += candidateCount;
        total.add( stats );
    }
    return total;
}

enum class method { fullWorld, candidateOnly };

std::uint64_t runBatch( const fixture& data, const scenario& scene, method mode )
{
    std::uint64_t checksum = 0;
    for( int repeat = 0; repeat < REPEATS; ++repeat )
    {
        for( const trial& query : scene.queries )
        {
            if( mode == method::fullWorld )
            {
                const auto result = data.simulation.castShapeSweepClosest( query.input );
                checksum += result.hit
                    ? static_cast<std::uint64_t>( result.shape.index1 ) +
                        static_cast<std::uint64_t>( result.fraction * 100000.0f )
                    : 1;
            }
            else
            {
                data.simulation.overlapAABB( query.candidateBounds, queryFilter{},
                    [&]( shapeId id )
                    {
                        checksum += static_cast<std::uint64_t>( id.index1 );
                        return true;
                    } );
            }
        }
    }
    return checksum;
}

double medianNanoseconds( const fixture& data, const scenario& scene, method mode )
{
    std::array<double, MEASURE_ROUNDS> measurements{};
    for( int round = 0; round < MEASURE_ROUNDS; ++round )
    {
        const auto start = std::chrono::steady_clock::now();
        const std::uint64_t checksum = runBatch( data, scene, mode );
        const auto end = std::chrono::steady_clock::now();
        benchmarkSink = benchmarkSink + checksum;
        measurements[round] = static_cast<double>(
            std::chrono::duration_cast<std::chrono::nanoseconds>( end - start ).count() ) /
            ( QUERY_COUNT * REPEATS );
    }
    std::sort( measurements.begin(), measurements.end() );
    return measurements[MEASURE_ROUNDS / 2];
}

} // namespace

int main( int argc, char** argv )
{
    bool verifyOnly = false, csv = false;
    for( int i = 1; i < argc; ++i )
    {
        if( std::strcmp( argv[i], "--verify" ) == 0 ) verifyOnly = true;
        else if( std::strcmp( argv[i], "--csv" ) == 0 ) csv = true;
        else
        {
            std::fprintf( stderr, "Unknown argument: %s\n", argv[i] );
            return EXIT_FAILURE;
        }
    }

    const fixture data{};
    std::array<totals, 4> results{};

    for( int i = 0; i < static_cast<int>( data.scenarios.size() ); ++i )
        results[i] = verify( data, data.scenarios[i] );

    check( results[0].toiCalls > 0 && results[1].toiCalls > 0 &&
        results[2].toiCalls == 0 && results[2].treeCandidates == 0 &&
        results[3].toiCalls > 0,
        "Dense World corpus must cover moving/rotating TOI and broadphase-only misses" );

    if( verifyOnly ) return EXIT_SUCCESS;

    if( csv )
        std::printf( "scenario,shapes,queries,tree_candidates,narrowphase_candidates,"
            "clipped_candidates,distance_pruned,toi_calls,base_gjk,fallback_attempts,fallback_gjk,fallback_hits,"
            "candidate_only_hits,world_ns_per_query,candidate_only_ns_per_query\n" );

    for( int i = 0; i < static_cast<int>( data.scenarios.size() ); ++i )
    {
        const scenario& scene = data.scenarios[i];
        const totals& stats = results[i];
        ( void )runBatch( data, scene, method::fullWorld );
        ( void )runBatch( data, scene, method::candidateOnly );
        const double worldNs = medianNanoseconds( data, scene, method::fullWorld );
        const double candidatesNs = medianNanoseconds( data, scene, method::candidateOnly );
        std::printf( "%s,%d,%d,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%.2f,%.2f\n",
            scene.name, GRID_SIZE * GRID_SIZE, QUERY_COUNT,
            static_cast<unsigned long long>( stats.treeCandidates ),
            static_cast<unsigned long long>( stats.narrowphaseCandidates ),
            static_cast<unsigned long long>( stats.clippedCandidates ),
            static_cast<unsigned long long>( stats.distancePrunedCandidates ),
            static_cast<unsigned long long>( stats.toiCalls ),
            static_cast<unsigned long long>( stats.toiBaseGjk ),
            static_cast<unsigned long long>( stats.fallbackAttempts ),
            static_cast<unsigned long long>( stats.fallbackGjk ),
            static_cast<unsigned long long>( stats.fallbackHits ),
            static_cast<unsigned long long>( stats.candidateOnlyHits ),
            worldNs, candidatesNs );
    }
}
