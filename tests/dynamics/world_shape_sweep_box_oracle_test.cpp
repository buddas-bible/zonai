#include <algorithm>
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

// Independent analytic point-to-oriented-box distance versus a fixed Circle.
// Do not use GJK, ShapeDistance, ShapeCast or TimeOfImpact for the sampled reference.
constexpr int SAMPLE_COUNT = 512;
constexpr float TARGET_RADIUS = 0.33f;
constexpr float HALF_WIDTH = 0.52f;
constexpr float HALF_HEIGHT = 0.12f;

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
    float localOffset = 0.0f;
    const char* category = "";
};

struct sampleResult
{
    double startGap = 0.0;
    double endGap = 0.0;
    double minimumGap = std::numeric_limits<double>::infinity();
    float firstHitFraction = 1.0f;
};

struct sampledPose
{
    double x = 0.0;
    double y = 0.0;
    double c = 1.0;
    double s = 0.0;
};

// Match the engine's nlerp fraction definition, but evaluate its math independently.
sampledPose poseAt( const orbitCase& sample, float fraction )
{
    const auto& input = sample.input;
    const rot2 from = input.transform.rotation;
    const rot2 to = from * rot2::FromRadians( input.rotationRadians );
    const double t = fraction, inverseT = 1.0 - t;
    const double cosine = inverseT * from.c + t * to.c;
    const double sine = inverseT * from.s + t * to.s;
    const double norm = std::hypot( cosine, sine );
    check( norm > 0.0, "Box oracle received invalid normalized rotation" );

    return { input.transform.position.x + static_cast<double>( input.translation.x ) * t,
        input.transform.position.y + static_cast<double>( input.translation.y ) * t,
        cosine / norm, sine / norm };
}

vec2 boxCenterAt( const orbitCase& sample, float fraction )
{
    const sampledPose pose = poseAt( sample, fraction );
    return { static_cast<float>( pose.x + pose.c * sample.localOffset ),
        static_cast<float>( pose.y + pose.s * sample.localOffset ) };
}

double signedGap( const orbitCase& sample, float fraction )
{
    const sampledPose pose = poseAt( sample, fraction );
    const double relativeX = static_cast<double>( sample.target.x ) - pose.x;
    const double relativeY = static_cast<double>( sample.target.y ) - pose.y;
    const double localX = pose.c * relativeX + pose.s * relativeY - sample.localOffset;
    const double localY = -pose.s * relativeX + pose.c * relativeY;

    const double outsideX = std::max( std::fabs( localX ) - HALF_WIDTH, 0.0 );
    const double outsideY = std::max( std::fabs( localY ) - HALF_HEIGHT, 0.0 );
    return std::hypot( outsideX, outsideY ) - TARGET_RADIUS;
}

sampleResult sampleOracle( const orbitCase& sample )
{
    sampleResult result{};
    result.startGap = signedGap( sample, 0.0f );
    result.endGap = signedGap( sample, 1.0f );
    bool found = false;

    for( int i = 0; i <= SAMPLE_COUNT; ++i )
    {
        const float fraction = static_cast<float>( i ) / SAMPLE_COUNT;
        const double gap = signedGap( sample, fraction );
        result.minimumGap = std::min( result.minimumGap, gap );
        if( !found && gap <= 0.0 )
        {
            found = true;
            result.firstHitFraction = fraction;
        }
    }

    return result;
}

orbitCase makeCase( randomSequence& random, int index )
{
    orbitCase sample{};
    const bool translation = index % 3 == 1;
    const bool far = index % 3 == 2;
    sample.category = far ? "far" : translation ? "combined" : "rotation";

    sample.localOffset = random.between( 1.4f, 3.1f );
    polygon2 box = MakeBox( { HALF_WIDTH, HALF_HEIGHT } );
    for( int vertex = 0; vertex < box.vertexCount; ++vertex )
        box.vertices[vertex] += vec2{ sample.localOffset, 0.0f };
    box.centroid = { sample.localOffset, 0.0f };
    sample.input.geometry = box;
    sample.input.transform.position = {
        random.between( -3.0f, 3.0f ), random.between( -3.0f, 3.0f )
    };
    sample.input.transform.rotation = rot2::FromRadians( random.between( -0.8f, 0.8f ) );
    if( translation )
        sample.input.translation = {
            random.between( -1.5f, 1.5f ), random.between( -1.5f, 1.5f )
        };
    const float angle = random.between( 0.45f, 1.55f );
    sample.input.rotationRadians = index % 2 == 0 ? angle : -angle;

    const vec2 middle = boxCenterAt( sample, random.between( 0.30f, 0.70f ) );
    if( far )
        sample.target = middle + vec2{ 8.0f, 8.0f };
    else
        sample.target = middle + vec2{
            random.between( -0.10f, 0.10f ), random.between( -0.10f, 0.10f )
        };
    return sample;
}

void runAudit( bool csv, bool strict )
{
    world simulation{};
    const bodyId body = simulation.CreateBody( bodyType::Static );
    const shapeId target = simulation.CreateShape( body, circle2{ {}, TARGET_RADIUS } );
    randomSequence random{};

    int robustMiddle = 0;
    int misses = 0;
    int definiteMisses = 0;
    int reportedHits = 0;
    int printed = 0;

    if( csv )
        std::printf( "case,kind,start_gap,end_gap,min_gap,sampled_first,toi_hit,toi_fraction,robust_mid\n" );

    for( int i = 0; i < 600; ++i )
    {
        const orbitCase sample = makeCase( random, i );
        simulation.SetBodyTransform( body, { sample.target, {} } );
        const sampleResult reference = sampleOracle( sample );
        const worldShapeCastResult2 hit = simulation.castShapeSweepClosest( sample.input );

        const bool robust = reference.startGap > 0.04 && reference.endGap > 0.04 &&
            reference.minimumGap < -0.04 && reference.firstHitFraction > 0.0f &&
            reference.firstHitFraction < 1.0f;
        const bool separated = reference.minimumGap > 0.08;

        robustMiddle += static_cast<int>( robust );
        definiteMisses += static_cast<int>( separated );
        reportedHits += static_cast<int>( hit.hit );

        if( hit.hit )
        {
            check( hit.shape == target && hit.fraction > 0.0f && hit.fraction < 1.0f &&
                IsFinite( hit.point ) && IsFinite( hit.normal ),
                "Rotating Box TOI returned an invalid world hit" );
            check( signedGap( sample, hit.fraction ) < 0.06,
                "Rotating Box TOI reported a hit far outside analytic surfaces" );
        }
        if( separated ) check( !hit.hit, "Rotating Box TOI reported a definite miss as a hit" );

        if( robust && !hit.hit )
        {
            ++misses;
            if( !csv && printed < 5 )
            {
                ++printed;
                std::printf( "box miss: case=%d kind=%s angle=%.5f start=(%.5f,%.5f) "
                    "translation=(%.5f,%.5f) target=(%.5f,%.5f) offset=%.5f "
                    "min_gap=%.5f first=%.5f\n", i, sample.category,
                    sample.input.rotationRadians, sample.input.transform.position.x,
                    sample.input.transform.position.y, sample.input.translation.x,
                    sample.input.translation.y, sample.target.x, sample.target.y,
                    sample.localOffset, reference.minimumGap, reference.firstHitFraction );
            }
        }

        if( csv )
            std::printf( "%d,%s,%.7f,%.7f,%.7f,%.7f,%d,%.7f,%d\n", i, sample.category,
                reference.startGap, reference.endGap, reference.minimumGap,
                reference.firstHitFraction, static_cast<int>( hit.hit ),
                hit.hit ? hit.fraction : -1.0f, static_cast<int>( robust ) );
    }

    check( robustMiddle >= 50 && definiteMisses >= 190,
        "Box oracle corpus lacks robust transient contacts or definite misses" );

    if( !csv )
        std::printf( "Rotating Box World sweep oracle: cases=600 robust_middle=%d "
            "toi_missed=%d definite_miss=%d reported_hit=%d\n",
            robustMiddle, misses, definiteMisses, reportedHits );

    if( strict ) check( misses == 0,
        "Strict Box oracle: TOI missed robust intermediate contacts" );
}

} // namespace

int main( int argc, char** argv )
{
    bool csv = false, strict = false;
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
