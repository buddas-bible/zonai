#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "collision/broadphase/dynamicTree.h"

using namespace zonai;

namespace
{

constexpr int TARGET_COLUMNS = 32;
constexpr int TARGET_ROWS = 32;
constexpr int QUERY_COUNT = 128;
constexpr int BENCHMARK_ROUNDS = 9;
constexpr int BENCHMARK_REPEATS = 12;

volatile std::uint64_t benchmarkSink = 0;

void check( bool condition, const char* message )
{
    if( condition ) return;
    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}

struct randomSequence
{
    std::uint32_t state = 0xB0CA574u;

    std::uint32_t next()
    {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return state;
    }

    float between( float low, float high )
    {
        return low + ( high - low ) * static_cast<float>( next() & 0xFFFFFFu ) / 16777216.0f;
    }
};

struct sweep
{
    aabb2 start{};
    vec2 translation{};
    float fraction = 1.0f;
};

struct scenario
{
    const char* name = "";
    std::array<sweep, QUERY_COUNT> queries{};
};

struct candidateTotals
{
    std::uint64_t query = 0;
    std::uint64_t box = 0;
};

enum class castMode
{
    sweptQuery,
    boxCast
};

aabb2 sweptBounds( const sweep& input )
{
    const vec2 clipped = input.translation * input.fraction;
    return Union( input.start, { input.start.min + clipped, input.start.max + clipped } );
}

// Tree 구현과 독립적인 두 축의 겹침 허용 시간 구간.
bool sweepsOver( const sweep& input, const aabb2& target )
{
    float minTime = 0.0f;
    float maxTime = input.fraction;

    for( int axis = 0; axis < 2; ++axis )
    {
        const float aMin = axis == 0 ? input.start.min.x : input.start.min.y;
        const float aMax = axis == 0 ? input.start.max.x : input.start.max.y;
        const float bMin = axis == 0 ? target.min.x : target.min.y;
        const float bMax = axis == 0 ? target.max.x : target.max.y;
        const float velocity = axis == 0 ? input.translation.x : input.translation.y;

        if( velocity == 0.0f )
        {
            if( aMax < bMin || bMax < aMin ) return false;
            continue;
        }

        const float enter = ( bMin - aMax ) / velocity;
        const float leave = ( bMax - aMin ) / velocity;
        minTime = std::max( minTime, std::min( enter, leave ) );
        maxTime = std::min( maxTime, std::max( enter, leave ) );
        if( minTime > maxTime ) return false;
    }

    return minTime <= maxTime;
}

sweep makeSweep( vec2 start, vec2 halfExtent, vec2 translation, float fraction = 1.0f )
{
    return { { start - halfExtent, start + halfExtent }, translation, fraction };
}

std::array<scenario, 5> makeScenarios()
{
    randomSequence random{};
    std::array<scenario, 5> scenarios{};
    scenarios[0].name = "diagonal";
    scenarios[1].name = "horizontal";
    scenarios[2].name = "short";
    scenarios[3].name = "wide";
    scenarios[4].name = "clipped";

    for( int i = 0; i < QUERY_COUNT; ++i )
    {
        const float dx = random.between( -3.0f, 3.0f );
        const float dy = random.between( -3.0f, 3.0f );

        scenarios[0].queries[i] = makeSweep(
            { -25.0f + dx, -25.0f + dy }, { 0.2f, 0.2f }, { 50.0f, 50.0f } );

        scenarios[1].queries[i] = makeSweep(
            { -25.0f, random.between( -22.0f, 22.0f ) }, { 0.2f, 0.2f }, { 50.0f, 0.0f } );

        scenarios[2].queries[i] = makeSweep(
            { random.between( -22.0f, 22.0f ), random.between( -22.0f, 22.0f ) },
            { 0.3f, 0.3f }, { random.between( -3.0f, 3.0f ), random.between( -3.0f, 3.0f ) } );

        scenarios[3].queries[i] = makeSweep(
            { -25.0f + dx, -25.0f + dy }, { 3.0f, 2.0f }, { 50.0f, 50.0f } );

        scenarios[4].queries[i] = makeSweep(
            { -25.0f + dx, -25.0f + dy }, { 0.2f, 0.2f }, { 50.0f, 50.0f }, 0.25f );
    }

    return scenarios;
}

void collectCandidates( const dynamicTree& tree, const sweep& input, castMode mode,
    std::vector<std::uint8_t>& seen, std::uint64_t& count )
{
    std::fill( seen.begin(), seen.end(), 0 );
    count = 0;

    auto visit = [&]( std::int32_t proxyId )
    {
        const std::int32_t index = tree.GetProxyShapeIndex( proxyId );
        check( index >= 0 && static_cast<std::size_t>( index ) < seen.size(), "Invalid proxy shape index" );
        check( seen[static_cast<std::size_t>( index )] == 0, "Repeated candidate" );
        seen[static_cast<std::size_t>( index )] = 1;
        ++count;
    };

    if( mode == castMode::sweptQuery )
    {
        tree.Query( sweptBounds( input ), [&]( std::int32_t proxyId )
        {
            visit( proxyId );
            return true;
        } );
    }
    else
    {
        float fraction = input.fraction;
        tree.BoxCast( input.start, input.translation, fraction, [&]( std::int32_t proxyId, float )
        {
            visit( proxyId );
            return -1.0f;
        } );

        check( fraction == input.fraction, "Ignore callback must not clip BoxCast" );
    }
}

candidateTotals verifyScenario( const dynamicTree& tree, const std::vector<aabb2>& targets, const scenario& scene )
{
    std::vector<std::uint8_t> querySeen( targets.size() );
    std::vector<std::uint8_t> boxSeen( targets.size() );
    candidateTotals totals{};

    for( const sweep& input : scene.queries )
    {
        std::uint64_t queryCount = 0;
        std::uint64_t boxCount = 0;
        collectCandidates( tree, input, castMode::sweptQuery, querySeen, queryCount );
        collectCandidates( tree, input, castMode::boxCast, boxSeen, boxCount );

        for( std::size_t i = 0; i < targets.size(); ++i )
        {
            const bool expectedQuery = Overlaps( sweptBounds( input ), targets[i] );
            const bool expectedBox = sweepsOver( input, targets[i] );

            if( static_cast<bool>( querySeen[i] ) != expectedQuery ||
                static_cast<bool>( boxSeen[i] ) != expectedBox )
            {
                std::fprintf( stderr, "Candidate mismatch: %s target=%zu\n", scene.name, i );
                check( false, "BroadPhase differential oracle mismatch" );
            }

            check( !boxSeen[i] || querySeen[i], "BoxCast candidates must be a subset of swept Query" );
        }

        check( boxCount <= queryCount, "BoxCast reported more candidates than Query" );
        totals.query += queryCount;
        totals.box += boxCount;
    }

    return totals;
}

std::uint64_t runBatch( const dynamicTree& tree, const scenario& scene, castMode mode )
{
    std::uint64_t sum = 0;

    for( int repeat = 0; repeat < BENCHMARK_REPEATS; ++repeat )
    {
        for( const sweep& input : scene.queries )
        {
            if( mode == castMode::sweptQuery )
            {
                tree.Query( sweptBounds( input ), [&]( std::int32_t id )
                {
                    sum += static_cast<std::uint64_t>( id ) + 1;
                    return true;
                } );
            }
            else
            {
                float fraction = input.fraction;
                tree.BoxCast( input.start, input.translation, fraction, [&]( std::int32_t id, float )
                {
                    sum += static_cast<std::uint64_t>( id ) + 1;
                    return -1.0f;
                } );
            }
        }
    }

    return sum;
}

double measureBatch( const dynamicTree& tree, const scenario& scene, castMode mode )
{
    const auto begin = std::chrono::steady_clock::now();
    const std::uint64_t sum = runBatch( tree, scene, mode );
    const auto end = std::chrono::steady_clock::now();

    benchmarkSink = benchmarkSink + sum;
    const auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>( end - begin ).count();
    return static_cast<double>( nanos ) / static_cast<double>( QUERY_COUNT * BENCHMARK_REPEATS );
}

double median( std::array<double, BENCHMARK_ROUNDS> values )
{
    std::sort( values.begin(), values.end() );
    return values[values.size() / 2];
}

void printResults( const dynamicTree& tree, const std::array<scenario, 5>& scenarios,
    const std::array<candidateTotals, 5>& totals, bool csv )
{
    if( csv )
    {
        std::printf( "scenario,queries,swept_query_candidates,box_cast_candidates,reduction_percent,swept_query_ns_per_cast,box_cast_ns_per_cast,box_over_query_ratio\n" );
    }
    else
    {
        std::printf( "DynamicTree candidate benchmark: %zu proxies, %d queries/scenario, %d rounds x %d repeats\n",
            tree.GetProxyCount(), QUERY_COUNT, BENCHMARK_ROUNDS, BENCHMARK_REPEATS );
        std::printf( "Scenario        Query    BoxCast    Saved    Query ns/cast   Box ns/cast   Time ratio\n" );
    }

    for( std::size_t index = 0; index < scenarios.size(); ++index )
    {
        const scenario& scene = scenarios[index];
        // Warm-up uses the same query set. Alternate order each round to reduce measurement drift.
        benchmarkSink = benchmarkSink + runBatch( tree, scene, castMode::sweptQuery );
        benchmarkSink = benchmarkSink + runBatch( tree, scene, castMode::boxCast );

        std::array<double, BENCHMARK_ROUNDS> querySamples{};
        std::array<double, BENCHMARK_ROUNDS> boxSamples{};

        for( int round = 0; round < BENCHMARK_ROUNDS; ++round )
        {
            if( round % 2 == 0 )
            {
                querySamples[round] = measureBatch( tree, scene, castMode::sweptQuery );
                boxSamples[round] = measureBatch( tree, scene, castMode::boxCast );
            }
            else
            {
                boxSamples[round] = measureBatch( tree, scene, castMode::boxCast );
                querySamples[round] = measureBatch( tree, scene, castMode::sweptQuery );
            }
        }

        const double queryNanos = median( querySamples );
        const double boxNanos = median( boxSamples );
        const candidateTotals counts = totals[index];
        const double saved = counts.query == 0 ? 0.0 :
            100.0 * static_cast<double>( counts.query - counts.box ) / static_cast<double>( counts.query );
        const double ratio = queryNanos == 0.0 ? 0.0 : boxNanos / queryNanos;

        if( csv )
        {
            std::printf( "%s,%d,%llu,%llu,%.2f,%.2f,%.2f,%.3f\n", scene.name, QUERY_COUNT,
                static_cast<unsigned long long>( counts.query ),
                static_cast<unsigned long long>( counts.box ), saved, queryNanos, boxNanos, ratio );
        }
        else
        {
            std::printf( "%-12s %8llu %10llu %6.1f%% %15.1f %13.1f %12.3f\n", scene.name,
                static_cast<unsigned long long>( counts.query ),
                static_cast<unsigned long long>( counts.box ), saved, queryNanos, boxNanos, ratio );
        }
    }
}

}

int main( int argc, char** argv )
{
    bool verifyOnly = false;
    bool csv = false;

    for( int i = 1; i < argc; ++i )
    {
        if( std::strcmp( argv[i], "--verify" ) == 0 ) verifyOnly = true;
        else if( std::strcmp( argv[i], "--csv" ) == 0 ) csv = true;
        else
        {
            std::fprintf( stderr, "Usage: dynamicTreeBoxCastBenchmark [--verify] [--csv]\n" );
            return EXIT_FAILURE;
        }
    }

    dynamicTree tree{};
    randomSequence random{};
    std::vector<aabb2> targets;
    targets.reserve( TARGET_COLUMNS * TARGET_ROWS );

    for( int y = 0; y < TARGET_ROWS; ++y )
    {
        for( int x = 0; x < TARGET_COLUMNS; ++x )
        {
            const vec2 center{ ( static_cast<float>( x ) - 15.5f ) * 1.5f + random.between( -0.1f, 0.1f ),
                ( static_cast<float>( y ) - 15.5f ) * 1.5f + random.between( -0.1f, 0.1f ) };
            const vec2 half{ random.between( 0.2f, 0.55f ), random.between( 0.2f, 0.55f ) };
            const aabb2 bounds{ center - half, center + half };
            const std::int32_t shapeIndex = static_cast<std::int32_t>( targets.size() );

            targets.push_back( bounds );
            ( void )tree.CreateProxy( bounds, shapeIndex );
        }
    }

    // Tree topology is fixed for both methods. No rebuild cost is included in the timing.
    ( void )tree.Rebuild( true );
    check( tree.Validate(), "Benchmark tree must be valid" );

    const std::array<scenario, 5> scenarios = makeScenarios();
    std::array<candidateTotals, 5> totals{};

    for( std::size_t i = 0; i < scenarios.size(); ++i )
        totals[i] = verifyScenario( tree, targets, scenarios[i] );

    check( totals[0].box < totals[0].query, "Diagonal path should eliminate bounding-rectangle false positives" );

    if( verifyOnly )
    {
        std::printf( "BoxCast benchmark oracle verified: %zu targets, %zu sweeps\n",
            targets.size(), scenarios.size() * QUERY_COUNT );
        return EXIT_SUCCESS;
    }

    printResults( tree, scenarios, totals, csv );
    return EXIT_SUCCESS;
}
