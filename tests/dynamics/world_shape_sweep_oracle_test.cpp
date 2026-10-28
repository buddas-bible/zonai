#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

#include "dynamics/world.h"

using namespace zonai;

namespace
{

// Deterministic and independent analytic circle-versus-circle sweep sampling.
// The oracle intentionally does not call ShapeCast, TimeOfImpact or ShapeDistance.
constexpr int SAMPLE_COUNT = 512;
constexpr float TARGET_RADIUS = 0.33f;

void check( bool condition, const char* message )
{
    if( condition ) return;
    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}

struct randomSequence
{
    std::uint32_t state = 0xA91F35D7u;

    std::uint32_t next()
    {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return state;
    }

    float between( float minimum, float maximum )
    {
        return minimum + ( maximum - minimum ) *
            static_cast<float>( next() >> 8 ) * ( 1.0f / 16777216.0f );
    }
};

struct orbitCase
{
    worldShapeSweepInput2 input{};
    vec2 target{};
    const char* category = "";
};

struct oracleResult
{
    double startGap = 0.0;
    double endGap = 0.0;
    double minimumGap = std::numeric_limits<double>::infinity();
    float firstHitFraction = 1.0f;
};

struct auditTotals
{
    int cases = 0;
    int robustMidHits = 0;
    int missedMidHits = 0;
    int definiteMisses = 0;
    int reportedHits = 0;
    int uncertain = 0;
};

// Reproduce normalized sin/cos interpolation in double, independently of engine GetSweepTransform.
// Fraction is the engine's nlerp fraction, NOT an assumption of constant angular velocity.
vec2 centerAt( const worldShapeSweepInput2& input, float fraction )
{
    const circle2& circle = std::get<circle2>( input.geometry );
    const rot2 from = input.transform.rotation;
    const rot2 to = from * rot2::FromRadians( input.rotationRadians );
    const double t = fraction;
    const double oneMinusT = 1.0 - t;
    double cosine = oneMinusT * from.c + t * to.c;
    double sine = oneMinusT * from.s + t * to.s;
    const double length = std::hypot( cosine, sine );
    check( length > 0.0, "Oracle rotation interpolation is invalid" );
    cosine /= length;
    sine /= length;

    const double x = static_cast<double>( input.transform.position.x ) +
        static_cast<double>( input.translation.x ) * t +
        cosine * circle.center.x - sine * circle.center.y;
    const double y = static_cast<double>( input.transform.position.y ) +
        static_cast<double>( input.translation.y ) * t +
        sine * circle.center.x + cosine * circle.center.y;
    return { static_cast<float>( x ), static_cast<float>( y ) };
}

double signedGap( const orbitCase& sample, float fraction )
{
    const vec2 center = centerAt( sample.input, fraction );
    const circle2& circle = std::get<circle2>( sample.input.geometry );
    return std::hypot( static_cast<double>( center.x ) - sample.target.x,
        static_cast<double>( center.y ) - sample.target.y ) -
        static_cast<double>( circle.radius + TARGET_RADIUS );
}

oracleResult sampleOracle( const orbitCase& sample )
{
    oracleResult result{};
    result.startGap = signedGap( sample, 0.0f );
    result.endGap = signedGap( sample, 1.0f );
    bool found = false;

    for( int i = 0; i <= SAMPLE_COUNT; ++i )
    {
        const float fraction = static_cast<float>( i ) / SAMPLE_COUNT;
        const double gap = signedGap( sample, fraction );
        result.minimumGap = std::min( result.minimumGap, gap );

        if( gap <= 0.0 && !found )
        {
            result.firstHitFraction = fraction;
            found = true;
        }
    }

    return result;
}

orbitCase makeCase( randomSequence& random, int index )
{
    orbitCase sample{};
    const bool translation = index % 3 == 1;
    const bool definiteMiss = index % 3 == 2;

    sample.category = definiteMiss ? "far" : translation ? "combined" : "rotation";
    sample.input.geometry = circle2{ { random.between( 1.4f, 3.1f ), 0.0f }, 0.18f };
    sample.input.transform.position = {
        random.between( -3.0f, 3.0f ), random.between( -3.0f, 3.0f )
    };
    sample.input.transform.rotation = rot2::FromRadians( random.between( -0.8f, 0.8f ) );

    if( translation ) sample.input.translation = {
        random.between( -1.5f, 1.5f ), random.between( -1.5f, 1.5f )
    };

    const float angle = random.between( 0.45f, 1.55f );
    sample.input.rotationRadians = index % 2 == 0 ? angle : -angle;

    const vec2 nearest = centerAt( sample.input, random.between( 0.30f, 0.70f ) );
    if( definiteMiss )
        sample.target = nearest + vec2{ 8.0f, 8.0f };
    else
        sample.target = nearest + vec2{
            random.between( -0.12f, 0.12f ), random.between( -0.12f, 0.12f )
        };

    return sample;
}

void runAudit( bool csv, bool strict )
{
    world simulation{};
    const bodyId targetBody = simulation.CreateBody( bodyType::Static );
    const shapeId targetShape = simulation.CreateShape( targetBody, circle2{ {}, TARGET_RADIUS } );
    randomSequence random{};
    auditTotals totals{};
    int printedMisses = 0;

    if( csv )
        std::printf( "case,kind,start_gap,end_gap,min_gap,sampled_first,toi_hit,toi_fraction,robust_mid\n" );

    // The same seed and case index always reproduce the same geometry and motion.
    for( int i = 0; i < 600; ++i )
    {
        const orbitCase sample = makeCase( random, i );
        simulation.SetBodyTransform( targetBody, { sample.target, {} } );
        const oracleResult oracle = sampleOracle( sample );
        const worldShapeCastResult2 hit = simulation.castShapeSweepClosest( sample.input );
        const bool robustMid = oracle.startGap > 0.04 && oracle.endGap > 0.04 &&
            oracle.minimumGap < -0.04 && oracle.firstHitFraction > 0.0f &&
            oracle.firstHitFraction < 1.0f;
        const bool definiteMiss = oracle.minimumGap > 0.08;

        ++totals.cases;
        totals.robustMidHits += static_cast<int>( robustMid );
        totals.definiteMisses += static_cast<int>( definiteMiss );
        totals.reportedHits += static_cast<int>( hit.hit );

        if( hit.hit )
        {
            check( hit.shape == targetShape && hit.fraction > 0.0f && hit.fraction < 1.0f,
                "TOI returned a stale shape or invalid closest fraction" );
            check( IsFinite( hit.point ) && IsFinite( hit.normal ),
                "TOI returned non-finite hit data" );
            // Contact can be reported slightly before true surface intersection due to LINEAR_SLOP.
            // Reject only clearly impossible positives, not marginal glancing cases.
            check( signedGap( sample, hit.fraction ) < 0.06,
                "TOI reported a hit far outside the analytic circle surfaces" );
        }

        if( definiteMiss ) check( !hit.hit, "TOI reported a hit on a definitively separated path" );

        if( robustMid && !hit.hit )
        {
            ++totals.missedMidHits;
            if( !csv && printedMisses < 5 )
            {
                ++printedMisses;
                std::printf( "miss: case=%d kind=%s angle=%.5f start=(%.5f,%.5f) "
                    "travel=(%.5f,%.5f) target=(%.5f,%.5f) min_gap=%.5f first=%.5f\n",
                    i, sample.category, sample.input.rotationRadians,
                    sample.input.transform.position.x, sample.input.transform.position.y,
                    sample.input.translation.x, sample.input.translation.y,
                    sample.target.x, sample.target.y, oracle.minimumGap, oracle.firstHitFraction );
            }
        }

        if( !robustMid && !definiteMiss ) ++totals.uncertain;
        if( csv )
            std::printf( "%d,%s,%.7f,%.7f,%.7f,%.7f,%d,%.7f,%d\n", i, sample.category,
                oracle.startGap, oracle.endGap, oracle.minimumGap, oracle.firstHitFraction,
                static_cast<int>( hit.hit ), hit.hit ? hit.fraction : -1.0f,
                static_cast<int>( robustMid ) );
    }

    check( totals.cases == 600 && totals.definiteMisses >= 190 &&
        totals.robustMidHits >= 50,
        "Oracle fixtures did not exercise enough robust mid-sweep hits and definite misses" );

    if( !csv )
        std::printf( "Rotating World sweep oracle: cases=%d robust_middle=%d "
            "toi_missed=%d definite_miss=%d reported_hit=%d ambiguous=%d\n",
            totals.cases, totals.robustMidHits, totals.missedMidHits,
            totals.definiteMisses, totals.reportedHits, totals.uncertain );

    if( strict )
        check( totals.missedMidHits == 0,
            "Strict audit: TOI missed robust intermediate circle contacts" );
}

} // namespace

int main( int argc, char** argv )
{
    bool csv = false;
    bool strict = false;

    for( int i = 1; i < argc; ++i )
    {
        if( std::strcmp( argv[i], "--csv" ) == 0 ) csv = true;
        else if( std::strcmp( argv[i], "--strict" ) == 0 ) strict = true;
        else
        {
            std::fprintf( stderr, "Unknown argument: %s\n", argv[i] );
            return EXIT_FAILURE;
        }
    }

    runAudit( csv, strict );
}
