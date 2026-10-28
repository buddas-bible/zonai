#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "collision/constants.h"
#include "collision/shapeCast2.h"
#include "dynamics/world.h"
#include "geometry/polygon2.h"

using namespace zonai;

namespace
{

constexpr int gridSize = 20;
constexpr int queriesPerScene = 64;
constexpr int measuredRounds = 7;
constexpr int repeatsPerRound = 4;

volatile std::uint64_t benchmarkSink = 0;

void check( bool condition, const char* message )
{
    if( condition ) return;
    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}

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
};

struct target
{
    shapeGeometry geometry{};
    transform2 transform{};
    aabb2 speculativeBounds{};
    collisionFilter filter{};
    shapeId id{};
};

struct castTrial
{
    worldShapeCastInput2 input{};
    queryFilter filter{};
};

struct scenario
{
    const char* name = "";
    std::array<castTrial, queriesPerScene> trials{};
};

enum class castMethod
{
    sweptQuery,
    boxCast,
    actualWorld
};

struct closestHit
{
    int targetIndex = -1;
    float fraction = 1.0f;
    bool hit = false;
};

struct castMetrics
{
    std::uint64_t treeCandidates = 0;
    std::uint64_t acceptedCandidates = 0;
    std::uint64_t shapeCastCalls = 0;
    std::uint64_t distanceCalls = 0;
    std::uint64_t shapeCastHits = 0;
    std::uint64_t closestHits = 0;

    castMetrics& operator+=( const castMetrics& other )
    {
        treeCandidates += other.treeCandidates;
        acceptedCandidates += other.acceptedCandidates;
        shapeCastCalls += other.shapeCastCalls;
        distanceCalls += other.distanceCalls;
        shapeCastHits += other.shapeCastHits;
        closestHits += other.closestHits;
        return *this;
    }
};

struct fixture
{
    world simulation{};
    std::array<dynamicTree, static_cast<int>( bodyType::Count )> trees{};
    std::vector<target> targets{};

    fixture()
    {
        simulation.SetGravity( {} );
        targets.reserve( gridSize * gridSize );

        randomSequence random{};

        for( int y = 0; y < gridSize; ++y )
        {
            for( int x = 0; x < gridSize; ++x )
            {
                const int index = y * gridSize + x;
                const bodyType type = static_cast<bodyType>( index % static_cast<int>( bodyType::Count ) );
                const vec2 center{
                    ( static_cast<float>( x ) - 9.5f ) * 1.6f + random.between( -0.06f, 0.06f ),
                    ( static_cast<float>( y ) - 9.5f ) * 1.6f + random.between( -0.06f, 0.06f )
                };
                const transform2 transform{ center, rot2::FromRadians( random.between( -1.5f, 1.5f ) ) };

                shapeGeometry geometry{};
                if( index % 3 == 0 ) geometry = circle2{ {}, random.between( 0.2f, 0.43f ) };
                else if( index % 3 == 1 ) geometry = capsule2{ { -0.28f, 0.0f }, { 0.28f, 0.0f }, 0.22f };
                else geometry = MakeBox( { 0.32f, 0.28f } );

                const collisionFilter filter{
                    index % 5 == 0 ? std::uint64_t{ 2 } : ( index % 7 == 0 ? std::uint64_t{ 4 } : std::uint64_t{ 1 } ),
                    index % 9 == 0 ? std::uint64_t{ 3 } : ~std::uint64_t{ 0 },
                    index % 13 == 0 ? -3 : 0
                };

                const bodyId body = simulation.CreateBody( type, transform );
                const shapeId id = index % 11 == 0
                    ? simulation.CreateSensorShape( body, geometry, filter )
                    : simulation.CreateShape( body, geometry, filter );

                check( simulation.IsValid( id ), "Failed to create benchmark target" );

                const aabb2 speculativeBounds = simulation.GetShapeAABB( id );
                const aabb2 fatBounds = simulation.GetShapeFatAABB( id );
                check( ContainsAABB( fatBounds, speculativeBounds ), "Fat AABB must contain speculative AABB" );

                const int shapeIndex = static_cast<int>( targets.size() );
                targets.push_back( { geometry, transform, speculativeBounds, filter, id } );
                trees[static_cast<int>( type )].CreateProxy( fatBounds, shapeIndex );
            }
        }

        for( dynamicTree& tree : trees )
        {
            ( void )tree.Rebuild( true );
            check( tree.Validate(), "Benchmark DynamicTree invalid after rebuild" );
        }
    }
};

std::array<scenario, 6> makeScenarios()
{
    std::array<scenario, 6> scenes{};
    scenes[0].name = "diagonal";
    scenes[1].name = "horizontal";
    scenes[2].name = "short";
    scenes[3].name = "wide-box";
    scenes[4].name = "capsule";
    scenes[5].name = "miss";

    randomSequence random{};

    for( int i = 0; i < queriesPerScene; ++i )
    {
        const float jitterX = random.between( -0.45f, 0.45f );
        const float jitterY = random.between( -0.45f, 0.45f );

        for( scenario& scene : scenes )
        {
            castTrial& trial = scene.trials[i];
            trial.filter = i % 4 == 0 ? queryFilter{} :
                ( i % 4 == 1 ? queryFilter{ 1, 1 } :
                  i % 4 == 2 ? queryFilter{ 1, 2 } : queryFilter{ 4, ~std::uint64_t{ 0 } } );
        }

        scenes[0].trials[i].input = { circle2{ {}, 0.38f }, { { -17.0f + jitterX, -17.0f + jitterY }, {} }, { 34.0f, 34.0f }, false };

        scenes[1].trials[i].input = { circle2{ {}, 0.38f },
            { { -17.0f, random.between( -14.0f, 14.0f ) }, {} }, { 34.0f, 0.0f }, false };

        scenes[2].trials[i].input = { circle2{ {}, 0.3f },
            { { random.between( -14.0f, 14.0f ), random.between( -14.0f, 14.0f ) }, {} },
            { random.between( -1.5f, 1.5f ), random.between( -1.5f, 1.5f ) }, false };

        scenes[3].trials[i].input = { MakeBox( { 1.5f, 0.9f } ),
            { { -17.0f + jitterX, -17.0f + jitterY }, rot2::FromRadians( random.between( -0.4f, 0.4f ) ) },
            { 34.0f, 34.0f }, false };

        scenes[4].trials[i].input = { capsule2{ { -0.7f, 0.0f }, { 0.7f, 0.0f }, 0.3f },
            { { -17.0f + jitterX, -17.0f + jitterY }, rot2::FromRadians( 0.8f ) },
            { 34.0f, 34.0f }, i % 5 == 0 };

        scenes[5].trials[i].input = { circle2{ {}, 0.38f },
            { { -17.0f, 19.0f + jitterY }, {} }, { 34.0f, 0.0f }, false };
    }

    return scenes;
}

closestHit castComparison( const fixture& data, const castTrial& trial, castMethod method, castMetrics& metrics )
{
    const worldShapeCastInput2& input = trial.input;
    const aabb2 startBounds = ComputeShapeAABB( input.geometry, input.transform );
    const aabb2 endBounds{ startBounds.min + input.translation, startBounds.max + input.translation };
    const aabb2 fullSweep = Union( startBounds, endBounds );
    const shapeProxy2 movingProxy = MakeShapeProxy( input.geometry );

    closestHit closest{};
    float maxFraction = 1.0f;

    for( int treeIndex = 0; treeIndex < static_cast<int>( data.trees.size() ); ++treeIndex )
    {
        const dynamicTree& tree = data.trees[treeIndex];

        auto candidate = [&]( std::int32_t proxyId, float limit ) -> float
        {
            ++metrics.treeCandidates;
            const target& object = data.targets[static_cast<std::size_t>( tree.GetProxyShapeIndex( proxyId ) )];

            if( !ShouldQueryShape( trial.filter, object.filter ) ) return -1.0f;

            const aabb2 clippedEnd{ startBounds.min + input.translation * limit,
                startBounds.max + input.translation * limit };
            if( !Overlaps( Union( startBounds, clippedEnd ), object.speculativeBounds ) ) return -1.0f;

            ++metrics.acceptedCandidates;

            shapeCastInput2 cast{};
            cast.proxyA = MakeShapeProxy( object.geometry );
            cast.proxyB = movingProxy;
            cast.transform = InverseMul( object.transform, input.transform );
            cast.translationB = InverseTransformVector( object.transform, input.translation );
            cast.maxFraction = limit;
            cast.canEncroach = input.canEncroach;

            ++metrics.shapeCastCalls;
            const castOutput2 hit = ShapeCast( cast );
            metrics.distanceCalls += static_cast<std::uint64_t>( hit.iterations );

            if( !hit.hit ) return -1.0f;
            ++metrics.shapeCastHits;

            const vec2 point = TransformPoint( object.transform, hit.point );
            const vec2 normal = TransformVector( object.transform, hit.normal );
            if( !IsFinite( point ) || !IsFinite( normal ) ) return -1.0f;

            // World::castShapeClosest는 초기 겹침을 무시한다.
            if( hit.fraction == 0.0f ) return -1.0f;

            if( !closest.hit || hit.fraction < closest.fraction )
                closest = { tree.GetProxyShapeIndex( proxyId ), hit.fraction, true };

            return closest.fraction;
        };

        if( method == castMethod::sweptQuery )
        {
            tree.Query( fullSweep, [&]( std::int32_t proxyId )
            {
                const float result = candidate( proxyId, maxFraction );
                if( result == 0.0f )
                {
                    maxFraction = 0.0f;
                    return false;
                }

                if( std::isfinite( result ) && result > 0.0f && result < maxFraction ) maxFraction = result;
                return true;
            } );
        }
        else
        {
            tree.BoxCast( startBounds, input.translation, maxFraction, candidate );
        }

        if( maxFraction == 0.0f ) break;
    }

    if( closest.hit ) ++metrics.closestHits;
    return closest;
}

void compareHits( const fixture& data, const closestHit& query, const closestHit& box,
    const worldShapeCastResult2& actual, const char* sceneName, int trialIndex )
{
    if( query.hit != box.hit || query.hit != actual.hit )
    {
        std::fprintf( stderr, "%s trial %d: query=%d box=%d world=%d\n",
            sceneName, trialIndex, query.hit, box.hit, actual.hit );
        check( false, "Closest ShapeCast hit/miss mismatch" );
    }

    if( !query.hit ) return;

    // 서로 다른 Tree 방문 순서에서 거의 같은 시간에 접촉한 도형은 동률로 취급함.
    if( std::fabs( query.fraction - box.fraction ) > 0.001f ||
        std::fabs( box.fraction - actual.fraction ) > 0.001f )
    {
        std::fprintf( stderr, "%s trial %d: query=%f box=%f world=%f\n",
            sceneName, trialIndex, query.fraction, box.fraction, actual.fraction );
        check( false, "Closest ShapeCast fraction mismatch" );
    }

    check( data.simulation.IsValid( actual.shape ), "World returned stale shapeId" );

    // 근접 동률에서는 Tree 순회 순서에 따라 다른 Shape가 같은 fraction으로 보고될 수 있음.
}

struct sceneStats
{
    castMetrics query{};
    castMetrics box{};
    std::uint64_t actualHits = 0;
};

sceneStats verifyScene( const fixture& data, const scenario& scene )
{
    sceneStats stats{};

    for( int i = 0; i < queriesPerScene; ++i )
    {
        const castTrial& trial = scene.trials[i];
        castMetrics q{};
        castMetrics b{};
        const closestHit query = castComparison( data, trial, castMethod::sweptQuery, q );
        const closestHit box = castComparison( data, trial, castMethod::boxCast, b );
        const worldShapeCastResult2 actual = data.simulation.castShapeClosest( trial.input, trial.filter );

        compareHits( data, query, box, actual, scene.name, i );
        stats.query += q;
        stats.box += b;
        if( actual.hit ) ++stats.actualHits;
    }

    check( stats.query.closestHits == stats.box.closestHits && stats.box.closestHits == stats.actualHits,
        "Closest hit counts must agree" );
    return stats;
}

std::uint64_t runBatch( const fixture& data, const scenario& scene, castMethod method )
{
    std::uint64_t sum = 0;

    for( int repeat = 0; repeat < repeatsPerRound; ++repeat )
    {
        for( const castTrial& trial : scene.trials )
        {
            if( method == castMethod::actualWorld )
            {
                const worldShapeCastResult2 result = data.simulation.castShapeClosest( trial.input, trial.filter );
                if( result.hit )
                    sum += static_cast<std::uint64_t>( result.shape.index1 ) +
                        static_cast<std::uint64_t>( result.fraction * 100000.0f );
                continue;
            }

            castMetrics metrics{};
            const closestHit hit = castComparison( data, trial, method, metrics );
            sum += metrics.treeCandidates + metrics.shapeCastCalls + metrics.distanceCalls;
            if( hit.hit ) sum += static_cast<std::uint64_t>( hit.targetIndex + 1 ) +
                static_cast<std::uint64_t>( hit.fraction * 100000.0f );
        }
    }

    return sum;
}

double measureBatch( const fixture& data, const scenario& scene, castMethod method )
{
    const auto begin = std::chrono::steady_clock::now();
    const std::uint64_t sum = runBatch( data, scene, method );
    const auto end = std::chrono::steady_clock::now();
    benchmarkSink = benchmarkSink + sum;

    return static_cast<double>( std::chrono::duration_cast<std::chrono::nanoseconds>( end - begin ).count() )
        / static_cast<double>( queriesPerScene * repeatsPerRound );
}

double median( std::array<double, measuredRounds> samples )
{
    std::sort( samples.begin(), samples.end() );
    return samples[samples.size() / 2];
}

void printMeasurement( const fixture& data, const std::array<scenario, 6>& scenes,
    const std::array<sceneStats, 6>& totals, bool csv )
{
    if( csv )
        std::printf( "scenario,casts,closest_hits,query_leaves,box_leaves,query_shape_cast_calls,box_shape_cast_calls,query_distance_calls,box_distance_calls,query_ns,box_ns,world_ns,box_over_query_ratio\n" );
    else
    {
        std::printf( "World-like ShapeCast comparison: %zu shapes, %d casts/scenario, %d rounds x %d repeats\n",
            data.targets.size(), queriesPerScene, measuredRounds, repeatsPerRound );
        std::printf( "Scene         Q leaves B leaves    Q casts B casts Q distance B distance   Query ns    Box ns  World ns  Box/Query\n" );
    }

    for( std::size_t i = 0; i < scenes.size(); ++i )
    {
        const scenario& scene = scenes[i];
        const sceneStats& stats = totals[i];

        for( castMethod method : { castMethod::sweptQuery, castMethod::boxCast, castMethod::actualWorld } )
            benchmarkSink = benchmarkSink + runBatch( data, scene, method );

        std::array<double, measuredRounds> querySamples{}, boxSamples{}, worldSamples{};

        for( int round = 0; round < measuredRounds; ++round )
        {
            if( round % 2 == 0 )
            {
                querySamples[round] = measureBatch( data, scene, castMethod::sweptQuery );
                boxSamples[round] = measureBatch( data, scene, castMethod::boxCast );
                worldSamples[round] = measureBatch( data, scene, castMethod::actualWorld );
            }
            else
            {
                worldSamples[round] = measureBatch( data, scene, castMethod::actualWorld );
                boxSamples[round] = measureBatch( data, scene, castMethod::boxCast );
                querySamples[round] = measureBatch( data, scene, castMethod::sweptQuery );
            }
        }

        const double q = median( querySamples );
        const double b = median( boxSamples );
        const double w = median( worldSamples );

        if( csv )
        {
            std::printf( "%s,%d,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%.2f,%.2f,%.2f,%.3f\n",
                scene.name, queriesPerScene, static_cast<unsigned long long>( stats.actualHits ),
                static_cast<unsigned long long>( stats.query.treeCandidates ),
                static_cast<unsigned long long>( stats.box.treeCandidates ),
                static_cast<unsigned long long>( stats.query.shapeCastCalls ),
                static_cast<unsigned long long>( stats.box.shapeCastCalls ),
                static_cast<unsigned long long>( stats.query.distanceCalls ),
                static_cast<unsigned long long>( stats.box.distanceCalls ),
                q, b, w, q == 0.0 ? 0.0 : b / q );
        }
        else
        {
            std::printf( "%-12s %8llu %8llu %8llu %6llu %12llu %12llu %10.1f %9.1f %9.1f %10.3f\n",
                scene.name,
                static_cast<unsigned long long>( stats.query.treeCandidates ),
                static_cast<unsigned long long>( stats.box.treeCandidates ),
                static_cast<unsigned long long>( stats.query.shapeCastCalls ),
                static_cast<unsigned long long>( stats.box.shapeCastCalls ),
                static_cast<unsigned long long>( stats.query.distanceCalls ),
                static_cast<unsigned long long>( stats.box.distanceCalls ),
                q, b, w, q == 0.0 ? 0.0 : b / q );
        }
    }
}

} // namespace

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
            std::fprintf( stderr, "Usage: worldShapeCastBenchmark [--verify] [--csv]\n" );
            return EXIT_FAILURE;
        }
    }

    const fixture data{};
    const std::array<scenario, 6> scenes = makeScenarios();
    std::array<sceneStats, 6> totals{};

    for( std::size_t i = 0; i < scenes.size(); ++i )
        totals[i] = verifyScene( data, scenes[i] );

    if( verifyOnly )
    {
        std::printf( "World ShapeCast differential verified: %zu shapes, %zu casts, 3 paths\n",
            data.targets.size(), scenes.size() * queriesPerScene );
        return EXIT_SUCCESS;
    }

    printMeasurement( data, scenes, totals, csv );
    return EXIT_SUCCESS;
}
