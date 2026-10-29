#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "collision/chainSegmentCastOwnership2.h"
#include "geometry/polygon2.h"

using namespace zonai;

namespace
{

constexpr int CASES = 32;
constexpr int ROUNDS = 7;
constexpr int REPEATS = 4;

volatile std::uint64_t benchmarkSink = 0;

void check( bool value, const char* message )
{
    if( value ) return;
    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}

struct trial
{
    sweep2 sweep{};
};

struct fixture
{
    chainSegment2 chain{
        { -4.0f, 0.0f }, { { -2.0f, 0.0f }, { 0.0f, 0.0f } }, { 2.0f, 0.0f }
    };
    polygon2 polygon = MakeBox( { 0.5f, 0.05f } );
    transform2 target{ { 0.6f, -1.4f }, {} };
    shapeGeometry geometry{};
    shapeProxy2 proxy{};
    std::array<std::array<trial, CASES>, 3> scenarios{};

    fixture()
    {
        for( int i = 0; i < polygon.vertexCount; ++i )
            polygon.vertices[i].y += 1.0f;
        polygon.centroid.y += 1.0f;
        geometry = polygon;
        proxy = MakeShapeProxy( geometry );

        for( int i = 0; i < CASES; ++i )
        {
            const float jitter = 0.00015f * static_cast<float>( i % 8 );
            const float pivot = -1.14f + jitter;
            const vec2 stationary = TransformPoint( target, { -1.0f, pivot } );
            scenarios[0][i].sweep = { {}, stationary, stationary,
                rot2::FromRadians( -0.74f ), rot2::FromRadians( 0.74f ) };

            const float fastY = -1.07f + 0.003f * static_cast<float>( i % 16 );
            const vec2 start = TransformPoint( target, { -1001.0f, fastY } );
            const vec2 end = TransformPoint( target, { 999.0f, fastY } );
            scenarios[1][i].sweep = { {}, start, end,
                rot2::FromRadians( -0.74f ), rot2::FromRadians( 0.74f ) };

            const float clearY = -2.2f - 0.01f * static_cast<float>( i );
            const vec2 clear = TransformPoint( target, { -1.0f, clearY } );
            scenarios[2][i].sweep = { {}, clear, clear,
                rot2::FromRadians( -0.74f ), rot2::FromRadians( 0.74f ) };
        }
    }
};

struct totals
{
    int calls = 0;
    int hits = 0;
    int intervals = 0;
    int distanceCalls = 0;
    int manifoldCalls = 0;
    int maxIntervals = 0;
    int maxDepth = 0;
    int exhausted = 0;
};

totals verify( const fixture& data, const std::array<trial, CASES>& scenario )
{
    totals result{};
    for( const trial& item : scenario )
    {
        chainSweepRetryStats2 stats{};
        const chainSweepContact2 measured = FindOwnedChainSweepContact(
            data.chain, data.geometry, data.proxy, data.target,
            item.sweep, 0.0f, 1.0f, &stats );
        const chainSweepContact2 plain = FindOwnedChainSweepContact(
            data.chain, data.geometry, data.proxy, data.target,
            item.sweep, 0.0f, 1.0f );
        check( plain.hit == measured.hit &&
            ( !plain.hit || ( plain.fraction == measured.fraction &&
                LengthSquared( plain.point - measured.point ) == 0.0f &&
                LengthSquared( plain.normal - measured.normal ) == 0.0f ) ),
            "Chain retry benchmark instrumentation changed the hit" );
        check( stats.intervals <= 1024 && stats.distanceCalls <= stats.intervals &&
            stats.manifoldCalls <= stats.intervals + 24 && !stats.exhausted,
            "Chain retry benchmark exceeded its visit or manifold budget" );
        if( measured.hit )
        {
            const transform2 inChain = InverseMul( data.target,
                GetSweepTransform( item.sweep, measured.fraction ) );
            simplexCache2 cache{};
            const localManifold2 manifold = CollideChainSegmentPolygon(
                data.chain, data.polygon, inChain, cache );
            check( manifold.pointCount > 0 &&
                Dot( manifold.normal, { 0.0f, -1.0f } ) > 0.0f,
                "Chain retry benchmark returned an unowned contact" );
            ++result.hits;
        }
        ++result.calls;
        result.intervals += stats.intervals;
        result.distanceCalls += stats.distanceCalls;
        result.manifoldCalls += stats.manifoldCalls;
        result.maxIntervals = std::max( result.maxIntervals, stats.intervals );
        result.maxDepth = std::max( result.maxDepth, stats.depthLimit );
        result.exhausted += stats.exhausted ? 1 : 0;
    }
    return result;
}

std::uint64_t batch( const fixture& data, const std::array<trial, CASES>& scenario )
{
    std::uint64_t sum = 0;
    for( int r = 0; r < REPEATS; ++r )
        for( const trial& item : scenario )
        {
            const chainSweepContact2 contact = FindOwnedChainSweepContact(
                data.chain, data.geometry, data.proxy, data.target,
                item.sweep, 0.0f, 1.0f );
            sum += contact.hit ?
                static_cast<std::uint64_t>( contact.fraction * 1000000.0f ) + 1 : 0;
        }
    return sum;
}

double medianNanoseconds( const fixture& data, const std::array<trial, CASES>& scenario )
{
    std::array<double, ROUNDS> times{};
    for( int round = 0; round < ROUNDS; ++round )
    {
        const auto start = std::chrono::steady_clock::now();
        const std::uint64_t sum = batch( data, scenario );
        const auto finish = std::chrono::steady_clock::now();
        benchmarkSink = benchmarkSink + sum;
        times[round] = static_cast<double>(
            std::chrono::duration_cast<std::chrono::nanoseconds>( finish - start ).count() ) /
            ( CASES * REPEATS );
    }
    std::sort( times.begin(), times.end() );
    return times[ROUNDS / 2];
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
            std::fprintf( stderr, "Unknown benchmark option: %s\n", argv[i] );
            return EXIT_FAILURE;
        }
    }

    const fixture data{};
    const std::array<const char*, 3> labels{
        "rotating_two_windows", "fast_2000m_grazing", "separated_front"
    };
    std::array<totals, 3> results{};
    for( int i = 0; i < 3; ++i )
        results[i] = verify( data, data.scenarios[i] );

    check( results[0].hits > 0 && results[1].hits > 0 && results[2].hits == 0 &&
        results[0].maxDepth == 9 && results[1].maxDepth >= 12,
        "Chain retry benchmark lacks the intended contact or adaptive-depth coverage" );
    if( verifyOnly ) return EXIT_SUCCESS;

    if( csv ) std::printf( "scenario,calls,hits,intervals,gjk,manifold,max_intervals,"
        "max_depth,exhausted,median_ns_per_query\n" );

    for( int i = 0; i < 3; ++i )
    {
        ( void )batch( data, data.scenarios[i] );
        const double ns = medianNanoseconds( data, data.scenarios[i] );
        const totals& values = results[i];
        std::printf( "%s,%d,%d,%d,%d,%d,%d,%d,%d,%.2f\n",
            labels[i], values.calls, values.hits, values.intervals,
            values.distanceCalls, values.manifoldCalls,
            values.maxIntervals, values.maxDepth, values.exhausted, ns );
    }
}
