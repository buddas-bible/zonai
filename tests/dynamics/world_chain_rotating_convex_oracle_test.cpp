#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

#include "dynamics/world.h"
#include "geometry/polygon2.h"

using namespace zonai;

namespace
{

// Analytic front-plane support test for a pair of collinear, one-sided Chain edges.
// Does not call GJK, ShapeDistance, TimeOfImpact, narrowphase, or GetSweepTransform.
constexpr int SAMPLE_COUNT = 512;

void check( bool value, const char* message )
{
    if( value ) return;
    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}

enum class probeKind { capsule, box, triangle };
enum class trialKind { transient, farFront, farBack };

struct trial
{
    worldShapeSweepInput2 input{};
    vec2 localOrigin{};
    vec2 localTranslation{};
    float startAngle = 0.0f;
    float relativeAngle = 0.0f;
    probeKind probe = probeKind::capsule;
    trialKind kind = trialKind::transient;
    float offset = 0.0f;
    float peak = 0.0f;
};

shapeGeometry makeGeometry( probeKind kind, float offset )
{
    if( kind == probeKind::capsule )
        return capsule2{ { -0.14f, offset }, { 0.14f, offset }, 0.12f };

    if( kind == probeKind::box )
    {
        polygon2 box = MakeBox( { 0.13f, 0.12f } );
        for( int i = 0; i < box.vertexCount; ++i ) box.vertices[i].y += offset;
        box.centroid.y += offset;
        return box;
    }

    const std::array<vec2, 3> vertices{
        vec2{ -0.15f, offset - 0.11f },
        vec2{ 0.16f, offset - 0.10f },
        vec2{ 0.0f, offset + 0.16f }
    };
    return MakePolygon( vertices );
}

struct localPose
{
    double x = 0.0, y = 0.0;
    double c = 1.0, s = 0.0;
};

localPose poseAt( const trial& sample, float fraction )
{
    const double start = sample.startAngle;
    const double finish = start + sample.relativeAngle;
    const double t = fraction;
    const double c = ( 1.0 - t ) * std::cos( start ) + t * std::cos( finish );
    const double s = ( 1.0 - t ) * std::sin( start ) + t * std::sin( finish );
    const double scale = std::hypot( c, s );
    check( scale > 0.0, "Invalid nlerp oracle rotation" );
    return {
        static_cast<double>( sample.localOrigin.x ) + t * sample.localTranslation.x,
        static_cast<double>( sample.localOrigin.y ) + t * sample.localTranslation.y,
        c / scale, s / scale
    };
}

struct frame
{
    double gap = 0.0;
    double centroidY = 0.0;
    double left = std::numeric_limits<double>::infinity();
    double right = -std::numeric_limits<double>::infinity();
};

frame evaluate( const trial& sample, float fraction )
{
    const localPose pose = poseAt( sample, fraction );
    double highestY = -std::numeric_limits<double>::infinity();
    frame output{};
    double localCentroidX = 0.0;
    double localCentroidY = sample.offset;
    double radius = 0.0;

    const auto accumulate = [&]( vec2 vertex )
    {
        const double x = pose.x + pose.c * vertex.x - pose.s * vertex.y;
        const double y = pose.y + pose.s * vertex.x + pose.c * vertex.y;
        output.left = std::min( output.left, x );
        output.right = std::max( output.right, x );
        highestY = std::max( highestY, y );
    };

    if( const capsule2* capsule = std::get_if<capsule2>( &sample.input.geometry ) )
    {
        accumulate( capsule->center1 );
        accumulate( capsule->center2 );
        radius = capsule->radius;
        localCentroidX = 0.5 * ( capsule->center1.x + capsule->center2.x );
        localCentroidY = 0.5 * ( capsule->center1.y + capsule->center2.y );
    }
    else
    {
        const polygon2& polygon = std::get<polygon2>( sample.input.geometry );
        for( int i = 0; i < polygon.vertexCount; ++i ) accumulate( polygon.vertices[i] );
        radius = polygon.radius;
        localCentroidX = polygon.centroid.x;
        localCentroidY = polygon.centroid.y;
    }

    output.gap = -highestY - radius;
    output.centroidY = pose.y + pose.s * localCentroidX + pose.c * localCentroidY;
    output.left -= radius;
    output.right += radius;
    return output;
}

struct referenceResult
{
    double startGap = 0.0;
    double endGap = 0.0;
    double minimumGap = std::numeric_limits<double>::infinity();
    double highestCentroid = -std::numeric_limits<double>::infinity();
    bool withinChainSpan = true;
    float firstPenetrating = 1.0f;
};

referenceResult samplePath( const trial& sample )
{
    referenceResult result{};
    result.startGap = evaluate( sample, 0.0f ).gap;
    result.endGap = evaluate( sample, 1.0f ).gap;
    bool found = false;

    for( int index = 0; index <= SAMPLE_COUNT; ++index )
    {
        const float fraction = static_cast<float>( index ) / SAMPLE_COUNT;
        const frame pose = evaluate( sample, fraction );
        result.minimumGap = std::min( result.minimumGap, pose.gap );
        result.highestCentroid = std::max( result.highestCentroid, pose.centroidY );
        // Both chain segments together form the continuous [-2,2] front surface.
        // The support plane test is exact when the complete shape lies within that span.
        result.withinChainSpan &= pose.left > -1.9 && pose.right < 1.9;
        if( !found && pose.gap < 0.0 )
        {
            found = true;
            result.firstPenetrating = fraction;
        }
    }
    return result;
}

trial makeTrial( probeKind kind, trialKind mode, float offset, float angle,
    float peak, float centerX, int sign, float groundAngle, world& simulation )
{
    trial sample{};
    sample.probe = kind;
    sample.kind = mode;
    sample.offset = offset;
    sample.peak = peak;
    sample.relativeAngle = sign * angle;
    sample.startAngle = -0.5f * sample.relativeAngle;
    sample.localOrigin = { centerX, -offset + peak };
    if( mode == trialKind::farFront ) sample.localOrigin.y -= 2.0f;
    if( mode == trialKind::farBack ) sample.localOrigin.y += 2.0f;

    // Combined translation and rotation, including either sign of displacement.
    sample.localTranslation = { 0.02f * sign, 0.012f * sign };
    sample.input.geometry = makeGeometry( kind, offset );
    const transform2 ground{ { 0.7f, -1.2f }, rot2::FromRadians( groundAngle ) };
    sample.input.transform.position = TransformPoint( ground, sample.localOrigin );
    sample.input.transform.rotation = ground.rotation * rot2::FromRadians( sample.startAngle );
    sample.input.translation = TransformVector( ground, sample.localTranslation );
    sample.input.rotationRadians = sample.relativeAngle;
    ( void )simulation;
    return sample;
}

void runAudit( bool csv, bool strict )
{
    world simulation{};
    const transform2 ground{ { 0.7f, -1.2f }, rot2::FromRadians( 0.38f ) };
    const bodyId groundBody = simulation.CreateBody( bodyType::Static, ground );
    const std::array<vec2, 3> vertices{ vec2{ -2.0f, 0.0f },
        vec2{ 0.0f, 0.0f }, vec2{ 2.0f, 0.0f } };
    chainDef definition{};
    definition.points = vertices;
    definition.ghost1 = { -4.0f, 0.0f };
    definition.ghost2 = { 4.0f, 0.0f };
    const chainId parent = simulation.CreateChain( groundBody, definition );
    std::array<shapeId, 2> edges{};
    check( simulation.GetChainSegments( parent, edges ) == edges.size(),
        "Transient oracle must use both adjacent Chain child handles" );

    int cases = 0, robust = 0, misses = 0, definiteMisses = 0, falseHits = 0;
    int seamRobust = 0, negativeRobust = 0;
    std::array<int, 3> shapeRobust{}, shapeMissed{};
    int printed = 0;

    if( csv )
        std::printf( "case,probe,mode,offset,angle,peak,x,start_gap,end_gap,min_gap,centroid_y,robust,hit,fraction\n" );

    for( const probeKind kind : { probeKind::capsule, probeKind::box, probeKind::triangle } )
    for( const float offset : { 0.8f, 0.98f } )
    for( const float angle : { 1.32f, 1.5f } )
    for( const float peak : { -0.055f, -0.080f, -0.105f } )
    for( const float x : { -1.0f, -0.28f, 0.0f, 0.28f, 1.0f } )
    for( const int sign : { -1, 1 } )
    for( const trialKind mode : { trialKind::transient, trialKind::farFront, trialKind::farBack } )
    {
        const trial sample = makeTrial( kind, mode, offset, angle, peak, x, sign, 0.38f, simulation );
        const referenceResult oracle = samplePath( sample );
        const worldShapeCastResult2 hit = simulation.castShapeSweepClosest( sample.input );
        const bool robustContact = mode == trialKind::transient &&
            oracle.withinChainSpan && oracle.startGap > 0.035 &&
            oracle.endGap > 0.035 && oracle.minimumGap < -0.040 &&
            oracle.highestCentroid < -0.015 &&
            oracle.firstPenetrating > 0.0f && oracle.firstPenetrating < 1.0f;
        const bool definiteMiss = oracle.minimumGap > 0.10;
        ++cases;

        if( hit.hit )
        {
            check( ( hit.shape == edges[0] || hit.shape == edges[1] ) &&
                hit.fraction > 0.0f && hit.fraction < 1.0f &&
                IsFinite( hit.point ) && IsFinite( hit.normal ),
                "Rotating Chain oracle returned invalid child hit" );
            // Analytic signed clearance of the infinite front line, not a GJK distance.
            check( evaluate( sample, hit.fraction ).gap < 0.07,
                "Rotating Chain oracle hit too far away from the geometric front" );
        }

        if( robustContact )
        {
            ++robust;
            ++shapeRobust[static_cast<int>( kind )];
            if( std::fabs( x ) < 0.3f ) ++seamRobust;
            if( sign < 0 ) ++negativeRobust;
            if( !hit.hit )
            {
                ++misses;
                ++shapeMissed[static_cast<int>( kind )];
                if( !csv && printed++ < 8 )
                    std::printf( "miss: case=%d kind=%d offset=%.4f angle=%.4f "
                        "peak=%.4f x=%.4f sign=%d start=%.5f end=%.5f min=%.5f\n",
                        cases, static_cast<int>( kind ), offset, angle, peak, x, sign,
                        oracle.startGap, oracle.endGap, oracle.minimumGap );
            }
        }
        if( definiteMiss )
        {
            ++definiteMisses;
            if( hit.hit )
            {
                ++falseHits;
                check( false, "Rotating Chain oracle reported a definitely separated sweep" );
            }
        }
        if( csv )
            std::printf( "%d,%d,%d,%.5f,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f,%d,%d,%.6f\n",
                cases, static_cast<int>( kind ), static_cast<int>( mode ),
                offset, angle, peak, x, oracle.startGap, oracle.endGap,
                oracle.minimumGap, oracle.highestCentroid, static_cast<int>( robustContact ),
                static_cast<int>( hit.hit ), hit.hit ? hit.fraction : -1.0f );
    }

    check( cases == 1080 && robust >= 40 && seamRobust >= 12 &&
        negativeRobust >= 20 && definiteMisses >= 700 &&
        std::all_of( shapeRobust.begin(), shapeRobust.end(),
            []( int count ) { return count >= 10; } ),
        "Chain sweep oracle has insufficient independently classified cases" );
    if( !csv )
        std::printf( "Chain rotating convex oracle: cases=%d robust=%d misses=%d "
            "capsule=%d/%d box=%d/%d triangle=%d/%d seam=%d negative=%d "
            "definite_miss=%d false_hit=%d\n",
            cases, robust, misses,
            shapeRobust[0], shapeMissed[0], shapeRobust[1], shapeMissed[1],
            shapeRobust[2], shapeMissed[2], seamRobust, negativeRobust,
            definiteMisses, falseHits );
    if( strict ) check( misses == 0,
        "Rotating Chain Capsule/Polygon TOI omitted robust intermediate front contacts" );
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
            std::fprintf( stderr, "Unknown Chain oracle argument: %s\n", argv[i] );
            return EXIT_FAILURE;
        }
    }
    runAudit( csv, strict );
}
