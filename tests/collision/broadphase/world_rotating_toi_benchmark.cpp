#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "collision/timeOfImpact2.h"
#include "dynamics/world.h"
#include "geometry/polygon2.h"

using namespace zonai;

namespace
{

constexpr int trialCount = 1200;
constexpr int measuredRounds = 7;
constexpr int batchRepeats = 3;

volatile std::uint64_t benchmarkSink = 0;

void check( bool condition, const char* message )
{
    if( condition ) return;
    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}

struct randomSequence
{
    std::uint32_t state = 0x57B0A923u;

    std::uint32_t next()
    {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return state;
    }

    float between( float low, float high )
    {
        return low + ( high - low ) *
            static_cast<float>( next() >> 8 ) * ( 1.0f / 16777216.0f );
    }
};

struct trial
{
    toiInput2 input{};
    worldShapeSweepInput2 worldInput{};
    vec2 target{};
    float targetRadius = 0.33f;
    bool thin = false;
};

vec2 centerAt( const worldShapeSweepInput2& input, float offset, float fraction )
{
    const rot2 from = input.transform.rotation;
    const rot2 to = from * rot2::FromRadians( input.rotationRadians );
    const double t = fraction;
    const double c = ( 1.0 - t ) * from.c + t * to.c;
    const double s = ( 1.0 - t ) * from.s + t * to.s;
    const double length = std::hypot( c, s );
    check( length > 0.0, "Invalid sweep rotation" );
    return { static_cast<float>( input.transform.position.x + input.translation.x * t +
        c / length * offset ),
        static_cast<float>( input.transform.position.y + input.translation.y * t +
        s / length * offset ) };
}

// The same seed, draw order, and geometry as worldRotatingBoxSweepOracleTests.
std::vector<trial> makeTrials()
{
    std::vector<trial> trials;
    trials.reserve( trialCount );
    randomSequence random{};

    for( int index = 0; index < trialCount; ++index )
    {
        trial item{};
        const int localIndex = index % 600;
        const bool translation = localIndex % 3 == 1;
        const bool far = localIndex % 3 == 2;
        item.thin = index >= 600;
        item.targetRadius = item.thin ? 0.04f : 0.33f;
        const float halfWidth = item.thin ? 0.45f : 0.52f;
        const float halfHeight = item.thin ? 0.015f : 0.12f;
        const float offset = random.between( 1.4f, 3.1f );

        polygon2 box = MakeBox( { halfWidth, halfHeight } );
        for( int vertex = 0; vertex < box.vertexCount; ++vertex )
            box.vertices[vertex] += vec2{ offset, 0.0f };
        box.centroid = { offset, 0.0f };
        item.worldInput.geometry = box;
        item.worldInput.transform.position = {
            random.between( -3.0f, 3.0f ), random.between( -3.0f, 3.0f )
        };
        item.worldInput.transform.rotation = rot2::FromRadians(
            random.between( -0.8f, 0.8f ) );
        if( translation )
            item.worldInput.translation = {
                random.between( -1.5f, 1.5f ), random.between( -1.5f, 1.5f )
            };
        const float angle = random.between( 0.45f, 1.55f );
        item.worldInput.rotationRadians = index % 2 == 0 ? angle : -angle;

        const vec2 middle = centerAt( item.worldInput, offset,
            random.between( 0.30f, 0.70f ) );
        if( far )
            item.target = middle + vec2{ 8.0f, 8.0f };
        else
            item.target = middle + vec2{
                random.between( item.thin ? -0.008f : -0.10f,
                    item.thin ? 0.008f : 0.10f ),
                random.between( item.thin ? -0.008f : -0.10f,
                    item.thin ? 0.008f : 0.10f )
            };

        item.input.proxyA = MakeShapeProxy( circle2{ {}, item.targetRadius } );
        item.input.proxyB = MakeShapeProxy( item.worldInput.geometry );
        item.input.sweepA = { {}, item.target, item.target, {}, {} };
        item.input.sweepB = { {}, item.worldInput.transform.position,
            item.worldInput.transform.position + item.worldInput.translation,
            item.worldInput.transform.rotation,
            item.worldInput.transform.rotation *
                rot2::FromRadians( item.worldInput.rotationRadians ) };
        trials.push_back( item );
    }

    return trials;
}

bool isHit( toiState2 state )
{
    return state == toiState2::Hit || state == toiState2::Overlapped;
}

struct totals
{
    std::uint64_t totalCalls = 0;
    std::uint64_t baseDistanceCalls = 0;
    std::uint64_t transientAttempts = 0;
    std::uint64_t transientDistanceCalls = 0;
    std::uint64_t transientHits = 0;
    std::uint64_t transientStepLimit = 0;

    void add( const toiDiagnostics2& stats )
    {
        ++totalCalls;
        baseDistanceCalls += stats.distanceCalls;
        transientAttempts += stats.transientAttempts;
        transientDistanceCalls += stats.transientDistanceCalls;
        transientHits += stats.transientHits;
        transientStepLimit += stats.transientStepLimit;
    }
};

void verify( const std::vector<trial>& trials, totals& results )
{
    int recovered = 0;
    for( std::size_t index = 0; index < trials.size(); ++index )
    {
        const trial& item = trials[index];

        toiInput2 base = item.input;
        base.detectTransientContacts = false;
        const toiOutput2 legacy = TimeOfImpact( base );

        toiInput2 improved = item.input;
        improved.detectTransientContacts = true;
        toiDiagnostics2 stats{};
        const toiOutput2 checked = TimeOfImpact( improved, &stats );
        const toiOutput2 plain = TimeOfImpact( improved );
        check( plain.state == checked.state && plain.fraction == checked.fraction,
            "Optional profiling must not change TOI output" );
        check( stats.distanceCalls >= 1 && stats.transientAttempts <= 1 &&
            stats.transientHits <= stats.transientAttempts &&
            stats.transientDistanceCalls >= stats.transientHits,
            "TOI diagnostic counters are inconsistent" );
        if( isHit( legacy.state ) )
            check( legacy.state == checked.state && legacy.fraction == checked.fraction,
                "Transient recheck must not change an existing TOI hit" );
        if( !isHit( legacy.state ) && isHit( checked.state ) ) ++recovered;
        results.add( stats );
    }

    check( results.totalCalls == trialCount && results.transientAttempts >= 100,
        "Corpus must exercise optional TOI recheck substantially" );
    check( recovered > 0 && results.transientHits > 0,
        "Corpus must reproduce a transient contact recovered by recheck" );

    // The same exact unrounded seed input as issue #221.
    const trial& counterexample = trials[664];
    toiInput2 baseline = counterexample.input;
    baseline.detectTransientContacts = false;
    check( !isHit( TimeOfImpact( baseline ).state ),
        "Issue #221 baseline no longer reproduces; update benchmark fixture" );
    baseline.detectTransientContacts = true;
    const toiOutput2 corrected = TimeOfImpact( baseline );
    check( corrected.state == toiState2::Hit &&
        corrected.fraction > 0.0f && corrected.fraction < 1.0f,
        "Transient recheck must recover issue #221" );

    world simulation{};
    const bodyId body = simulation.CreateBody( bodyType::Static,
        { counterexample.target, {} } );
    const shapeId target = simulation.CreateShape( body,
        circle2{ {}, counterexample.targetRadius } );
    const worldShapeCastResult2 result = simulation.castShapeSweepClosest(
        counterexample.worldInput );
    check( result.hit && result.shape == target &&
        std::fabs( result.fraction - corrected.fraction ) < 0.002f,
        "World sweep and direct TOI recovered contact must agree" );
}

std::uint64_t runBatch( const std::vector<trial>& trials, bool recheck )
{
    std::uint64_t total = 0;
    for( int repeat = 0; repeat < batchRepeats; ++repeat )
    {
        for( const trial& item : trials )
        {
            toiInput2 input = item.input;
            input.detectTransientContacts = recheck;
            const toiOutput2 result = TimeOfImpact( input );
            total += static_cast<std::uint64_t>( result.state ) +
                static_cast<std::uint64_t>( result.fraction * 100000.0f );
        }
    }
    return total;
}

double medianNanoseconds( const std::vector<trial>& trials, bool recheck )
{
    std::array<double, measuredRounds> elapsed{};
    for( int round = 0; round < measuredRounds; ++round )
    {
        const auto start = std::chrono::steady_clock::now();
        const std::uint64_t sum = runBatch( trials, recheck );
        const auto finish = std::chrono::steady_clock::now();
        benchmarkSink = benchmarkSink + sum;
        elapsed[round] = static_cast<double>(
            std::chrono::duration_cast<std::chrono::nanoseconds>( finish - start ).count() ) /
            ( trials.size() * batchRepeats );
    }
    std::sort( elapsed.begin(), elapsed.end() );
    return elapsed[measuredRounds / 2];
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
            std::fprintf( stderr, "Unknown argument: %s\n", argv[i] );
            return EXIT_FAILURE;
        }
    }

    const std::vector<trial> trials = makeTrials();
    totals stats{};
    verify( trials, stats );

    if( verifyOnly ) return EXIT_SUCCESS;

    // Warm up both paths. Timings are informative only, never pass/fail thresholds.
    ( void )runBatch( trials, false );
    ( void )runBatch( trials, true );
    const double normalNs = medianNanoseconds( trials, false );
    const double guardedNs = medianNanoseconds( trials, true );

    if( csv )
        std::printf( "case,calls,recheck_attempts,recheck_hits,recheck_gjk,base_gjk,"
            "recheck_limit,legacy_ns_per_call,guarded_ns_per_call,guarded_over_legacy\n" );

    std::printf( "%s,%llu,%llu,%llu,%llu,%llu,%llu,%.2f,%.2f,%.3f\n",
        csv ? "rotating_box_corpus" : "rotating_box_corpus",
        static_cast<unsigned long long>( stats.totalCalls ),
        static_cast<unsigned long long>( stats.transientAttempts ),
        static_cast<unsigned long long>( stats.transientHits ),
        static_cast<unsigned long long>( stats.transientDistanceCalls ),
        static_cast<unsigned long long>( stats.baseDistanceCalls ),
        static_cast<unsigned long long>( stats.transientStepLimit ),
        normalNs, guardedNs, normalNs > 0.0 ? guardedNs / normalNs : 0.0 );

    return EXIT_SUCCESS;
}
