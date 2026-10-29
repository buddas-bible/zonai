#include <algorithm>
#include <array>
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

// 독립 기하 oracle: 정적 원과 회전 Capsule/삼각형 사이의 간격.
// ShapeDistance/GJK/TimeOfImpact/ShapeCast 및 엔진 sweep 변환은 사용하지 않음.
constexpr int SAMPLE_COUNT = 512;
constexpr int CASE_COUNT = 900;
constexpr double TARGET_RADIUS = 0.18;

void check( bool condition, const char* message )
{
    if( condition ) return;
    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}

struct randomSequence
{
    std::uint32_t state = 0x6ACD309Bu;

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

enum class movingKind { capsule, triangle };

struct sweepCase
{
    worldShapeSweepInput2 input{};
    vec2 target{};
    vec2 localCenter{};
    movingKind kind = movingKind::capsule;
    bool translated = false;
    bool far = false;
};

struct sampledPose
{
    double x = 0.0;
    double y = 0.0;
    double c = 1.0;
    double s = 0.0;
};

struct oracleResult
{
    double startGap = 0.0;
    double endGap = 0.0;
    double minimumGap = std::numeric_limits<double>::infinity();
    float firstHitFraction = 1.0f;
};

// 월드와 같은 nlerp 분율 정의만 공유하고 double 산술로 독립 계산함.
sampledPose poseAt( const sweepCase& sample, float fraction )
{
    const auto& input = sample.input;
    const rot2 start = input.transform.rotation;
    const double angle = input.rotationRadians;
    const double endC = static_cast<double>( start.c ) * std::cos( angle ) -
        static_cast<double>( start.s ) * std::sin( angle );
    const double endS = static_cast<double>( start.s ) * std::cos( angle ) +
        static_cast<double>( start.c ) * std::sin( angle );
    const double t = fraction;
    const double c = ( 1.0 - t ) * start.c + t * endC;
    const double s = ( 1.0 - t ) * start.s + t * endS;
    const double magnitude = std::hypot( c, s );
    check( magnitude > 0.0, "Invalid independent nlerp rotation" );

    return { input.transform.position.x + t * input.translation.x,
        input.transform.position.y + t * input.translation.y,
        c / magnitude, s / magnitude };
}

vec2 centerAt( const sweepCase& sample, float fraction )
{
    const sampledPose pose = poseAt( sample, fraction );
    return { static_cast<float>( pose.x + pose.c * sample.localCenter.x -
        pose.s * sample.localCenter.y ),
        static_cast<float>( pose.y + pose.s * sample.localCenter.x +
        pose.c * sample.localCenter.y ) };
}

double segmentDistanceSquared( double px, double py, vec2 start, vec2 end )
{
    const double dx = static_cast<double>( end.x ) - start.x;
    const double dy = static_cast<double>( end.y ) - start.y;
    const double len2 = dx * dx + dy * dy;
    const double t = len2 > 0.0 ? std::clamp(
        ( ( px - start.x ) * dx + ( py - start.y ) * dy ) / len2, 0.0, 1.0 ) : 0.0;
    const double deltaX = px - ( start.x + t * dx );
    const double deltaY = py - ( start.y + t * dy );
    return deltaX * deltaX + deltaY * deltaY;
}

double signedGap( const sweepCase& sample, float fraction )
{
    const sampledPose pose = poseAt( sample, fraction );
    const double dx = static_cast<double>( sample.target.x ) - pose.x;
    const double dy = static_cast<double>( sample.target.y ) - pose.y;
    const double localX = pose.c * dx + pose.s * dy;
    const double localY = -pose.s * dx + pose.c * dy;

    if( sample.kind == movingKind::capsule )
    {
        const capsule2& capsule = std::get<capsule2>( sample.input.geometry );
        return std::sqrt( segmentDistanceSquared(
            localX, localY, capsule.center1, capsule.center2 ) ) -
            ( capsule.radius + TARGET_RADIUS );
    }

    const polygon2& triangle = std::get<polygon2>( sample.input.geometry );
    double closestSquared = std::numeric_limits<double>::infinity();
    bool inside = true;
    for( int i = 0; i < triangle.vertexCount; ++i )
    {
        const vec2 a = triangle.vertices[i];
        const vec2 b = triangle.vertices[( i + 1 ) % triangle.vertexCount];
        closestSquared = std::min( closestSquared,
            segmentDistanceSquared( localX, localY, a, b ) );
        const double cross = ( static_cast<double>( b.x ) - a.x ) * ( localY - a.y ) -
            ( static_cast<double>( b.y ) - a.y ) * ( localX - a.x );
        if( cross < 0.0 ) inside = false;
    }
    // CCW 삼각형 안쪽은 음의 signed distance. 원 반경은 그 값에서 추가로 뺌.
    const double signedDistance = std::sqrt( closestSquared ) * ( inside ? -1.0 : 1.0 );
    return signedDistance - TARGET_RADIUS;
}

oracleResult sampleOracle( const sweepCase& sample )
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
        if( !found && gap <= 0.0 )
        {
            found = true;
            result.firstHitFraction = fraction;
        }
    }
    return result;
}

sweepCase makeCase( randomSequence& random, int index )
{
    sweepCase sample{};
    sample.kind = index < CASE_COUNT / 2 ? movingKind::capsule : movingKind::triangle;
    sample.translated = index % 3 == 1;
    sample.far = index % 3 == 2;

    const vec2 offset{ random.between( 1.7f, 2.8f ), random.between( -0.25f, 0.25f ) };
    if( sample.kind == movingKind::capsule )
    {
        sample.input.geometry = capsule2{
            offset + vec2{ -0.38f, -0.12f },
            offset + vec2{ 0.38f, 0.12f }, 0.12f };
        sample.localCenter = offset;
    }
    else
    {
        const std::array<vec2, 3> vertices{
            offset + vec2{ -0.36f, -0.30f },
            offset + vec2{ 0.48f, -0.18f },
            offset + vec2{ 0.02f, 0.42f }
        };
        const polygon2 triangle = MakePolygon( vertices );
        check( triangle.vertexCount == 3, "Triangle oracle geometry must be valid" );
        sample.localCenter = triangle.centroid;
        sample.input.geometry = triangle;
    }

    sample.input.transform.position = {
        random.between( -3.0f, 3.0f ), random.between( -3.0f, 3.0f ) };
    sample.input.transform.rotation = rot2::FromRadians( random.between( -1.2f, 1.2f ) );
    if( sample.translated ) sample.input.translation = {
        random.between( -0.8f, 0.8f ), random.between( -0.8f, 0.8f ) };
    const float angle = random.between( 0.60f, 1.52f );
    sample.input.rotationRadians = index % 2 == 0 ? angle : -angle;

    const vec2 middle = centerAt( sample, random.between( 0.28f, 0.72f ) );
    if( sample.far )
        sample.target = middle + vec2{ 7.0f, 7.0f };
    else
        sample.target = middle + vec2{
            random.between( -0.065f, 0.065f ), random.between( -0.065f, 0.065f ) };
    return sample;
}

void runAudit( bool csv, bool strict )
{
    world simulation{};
    const bodyId targetBody = simulation.CreateBody( bodyType::Static );
    const shapeId targetShape = simulation.CreateShape( targetBody,
        circle2{ {}, static_cast<float>( TARGET_RADIUS ) } );
    randomSequence random{};
    int robustCapsules = 0, robustTriangles = 0, robustCombined = 0, robustNegative = 0;
    int missedCapsules = 0, missedTriangles = 0, definiteMisses = 0, reportedHits = 0;
    int printedMisses = 0;

    if( csv )
        std::printf( "case,shape,mode,start_gap,end_gap,min_gap,sampled_first,toi_hit,toi_fraction,robust_mid\n" );

    for( int i = 0; i < CASE_COUNT; ++i )
    {
        const sweepCase sample = makeCase( random, i );
        simulation.SetBodyTransform( targetBody, { sample.target, {} } );
        const oracleResult reference = sampleOracle( sample );
        const worldShapeCastResult2 hit = simulation.castShapeSweepClosest( sample.input );
        const bool robust = reference.startGap > 0.04 && reference.endGap > 0.04 &&
            reference.minimumGap < -0.04 && reference.firstHitFraction > 0.0f &&
            reference.firstHitFraction < 1.0f;
        const bool definiteMiss = reference.minimumGap > 0.08;
        const bool capsule = sample.kind == movingKind::capsule;
        robustCapsules += static_cast<int>( robust && capsule );
        robustTriangles += static_cast<int>( robust && !capsule );
        robustCombined += static_cast<int>( robust && sample.translated );
        robustNegative += static_cast<int>( robust && sample.input.rotationRadians < 0.0f );
        definiteMisses += static_cast<int>( definiteMiss );
        reportedHits += static_cast<int>( hit.hit );

        if( hit.hit )
        {
            check( hit.shape == targetShape && hit.fraction > 0.0f && hit.fraction < 1.0f &&
                IsFinite( hit.point ) && IsFinite( hit.normal ),
                "Rotating Capsule/Triangle TOI returned invalid World hit" );
            check( signedGap( sample, hit.fraction ) < 0.06,
                "Rotating Capsule/Triangle hit is far outside independent surfaces" );
        }
        if( definiteMiss ) check( !hit.hit,
            "Rotating Capsule/Triangle TOI hit a definitely separated path" );

        if( robust && !hit.hit )
        {
            missedCapsules += static_cast<int>( capsule );
            missedTriangles += static_cast<int>( !capsule );
            if( !csv && printedMisses++ < 5 )
                std::printf( "miss: case=%d shape=%s mode=%s angle=%.5f "
                    "start=(%.5f,%.5f) translation=(%.5f,%.5f) target=(%.5f,%.5f) "
                    "minimum_gap=%.5f first=%.5f\n", i, capsule ? "capsule" : "triangle",
                    sample.far ? "far" : sample.translated ? "combined" : "rotation",
                    sample.input.rotationRadians, sample.input.transform.position.x,
                    sample.input.transform.position.y, sample.input.translation.x,
                    sample.input.translation.y, sample.target.x, sample.target.y,
                    reference.minimumGap, reference.firstHitFraction );
        }
        if( csv )
            std::printf( "%d,%s,%s,%.7f,%.7f,%.7f,%.7f,%d,%.7f,%d\n",
                i, capsule ? "capsule" : "triangle",
                sample.far ? "far" : sample.translated ? "combined" : "rotation",
                reference.startGap, reference.endGap, reference.minimumGap,
                reference.firstHitFraction, static_cast<int>( hit.hit ),
                hit.hit ? hit.fraction : -1.0f, static_cast<int>( robust ) );
    }

    check( robustCapsules >= 30 && robustTriangles >= 30 &&
        robustCombined >= 20 && robustNegative >= 20 && definiteMisses >= 280,
        "Capsule/Triangle oracle corpus lacks robust contacts or definite misses" );
    if( !csv )
        std::printf( "Rotating Capsule/Triangle World oracle: cases=%d "
            "capsule_robust=%d capsule_missed=%d triangle_robust=%d triangle_missed=%d "
            "combined_robust=%d negative_robust=%d definite_miss=%d reported_hit=%d\n",
            CASE_COUNT, robustCapsules, missedCapsules, robustTriangles, missedTriangles,
            robustCombined, robustNegative, definiteMisses, reportedHits );
    if( strict ) check( missedCapsules == 0 && missedTriangles == 0,
        "Strict Capsule/Triangle oracle missed robust intermediate contacts" );
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
