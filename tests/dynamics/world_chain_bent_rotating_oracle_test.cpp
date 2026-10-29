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

// 독립 기하 oracle: finite Chain edge와 moving convex core의 실제 거리.
// GJK/ShapeDistance/TOI/Manifold/GetSweepTransform을 정답 계산에 사용하지 않음.
constexpr int SAMPLES = 512;
constexpr double MIN_PENETRATION = 0.035;
constexpr double MIN_ENDPOINT_GAP = 0.045;
constexpr double EDGE_MARGIN = 0.18;

void check( bool ok, const char* message )
{
    if( ok ) return;
    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}

struct vector2
{
    double x = 0.0;
    double y = 0.0;
};

vector2 operator+( vector2 a, vector2 b ) { return { a.x + b.x, a.y + b.y }; }
vector2 operator-( vector2 a, vector2 b ) { return { a.x - b.x, a.y - b.y }; }
vector2 operator*( vector2 a, double s ) { return { a.x * s, a.y * s }; }
double dot( vector2 a, vector2 b ) { return a.x * b.x + a.y * b.y; }
double cross( vector2 a, vector2 b ) { return a.x * b.y - a.y * b.x; }
double length( vector2 a ) { return std::hypot( a.x, a.y ); }
vector2 rotate( vector2 a, double c, double s )
{
    return { c * a.x - s * a.y, s * a.x + c * a.y };
}

struct edge
{
    vector2 a{}, b{}, axis{}, front{};
    double length = 0.0;
};

edge makeEdge( vector2 a, vector2 b )
{
    const vector2 delta = b - a;
    const double distance = length( delta );
    check( distance > 0.1, "Invalid independent Chain edge" );
    const vector2 axis = delta * ( 1.0 / distance );
    return { a, b, axis, { axis.y, -axis.x }, distance };
}

double pointSegmentDistance2( vector2 p, vector2 a, vector2 b )
{
    const vector2 span = b - a;
    const double size2 = dot( span, span );
    const double t = size2 > 0.0 ? std::clamp( dot( p - a, span ) / size2, 0.0, 1.0 ) : 0.0;
    const vector2 delta = p - ( a + span * t );
    return dot( delta, delta );
}

bool segmentsCross( vector2 a, vector2 b, vector2 c, vector2 d )
{
    const vector2 u = b - a, v = d - c;
    const double denominator = cross( u, v );
    if( std::fabs( denominator ) <= 1e-14 ) return false;
    const vector2 r = c - a;
    const double t = cross( r, v ) / denominator;
    const double w = cross( r, u ) / denominator;
    return t >= 0.0 && t <= 1.0 && w >= 0.0 && w <= 1.0;
}

double segmentDistance2( vector2 a, vector2 b, vector2 c, vector2 d )
{
    if( segmentsCross( a, b, c, d ) ) return 0.0;
    return std::min( { pointSegmentDistance2( a, c, d ),
        pointSegmentDistance2( b, c, d ), pointSegmentDistance2( c, a, b ),
        pointSegmentDistance2( d, a, b ) } );
}

enum class shapeType { capsule, box, triangle };
enum class pathType { transient, clearFront, clearBack };

struct caseInput
{
    worldShapeSweepInput2 worldInput{};
    std::array<vector2, 4> core{};
    int count = 0;
    double radius = 0.0;
    vector2 localCentroid{};
    vector2 origin{}, translation{};
    double startAngle = 0.0, turn = 0.0;
    shapeType type = shapeType::capsule;
    pathType path = pathType::transient;
    int edgeIndex = 0;
    double bend = 0.0;
    double localX = 0.0;
};

caseInput makeCase( shapeType type, pathType path, double offset,
    double peak, double x, int direction, int edgeIndex,
    double bend, const transform2& ground, const std::array<edge, 2>& segments )
{
    caseInput sample{};
    sample.type = type;
    sample.path = path;
    sample.edgeIndex = edgeIndex;
    sample.bend = bend;
    sample.localX = x;
    sample.turn = direction * 1.48;
    const edge& target = segments[edgeIndex];
    const double baseAngle = std::atan2( target.axis.y, target.axis.x );
    sample.startAngle = baseAngle - 0.5 * sample.turn;
    sample.origin = target.a + target.axis * x + target.front * ( offset - peak );
    // target.front points downward, so offset-peak puts the pivot below the edge.
    if( path == pathType::clearFront ) sample.origin = sample.origin + target.front * 1.5;
    if( path == pathType::clearBack ) sample.origin = sample.origin - target.front * 1.8;
    sample.translation = target.axis * ( 0.018 * direction ) +
        target.front * ( 0.012 * direction );

    if( type == shapeType::capsule )
    {
        sample.core[0] = { -0.14, offset };
        sample.core[1] = { 0.14, offset };
        sample.count = 2;
        sample.radius = 0.12;
        sample.localCentroid = { 0.0, offset };
        sample.worldInput.geometry = capsule2{
            { -0.14f, static_cast<float>( offset ) },
            { 0.14f, static_cast<float>( offset ) }, 0.12f };
    }
    else if( type == shapeType::box )
    {
        sample.core = { vector2{ -0.13, offset - 0.12 },
            vector2{ 0.13, offset - 0.12 }, vector2{ 0.13, offset + 0.12 },
            vector2{ -0.13, offset + 0.12 } };
        sample.count = 4;
        sample.localCentroid = { 0.0, offset };
        polygon2 polygon = MakeBox( { 0.13f, 0.12f } );
        for( int i = 0; i < polygon.vertexCount; ++i )
            polygon.vertices[i].y += static_cast<float>( offset );
        polygon.centroid.y += static_cast<float>( offset );
        sample.worldInput.geometry = polygon;
    }
    else
    {
        sample.core = { vector2{ -0.15, offset - 0.11 },
            vector2{ 0.16, offset - 0.10 }, vector2{ 0.0, offset + 0.16 },
            vector2{} };
        sample.count = 3;
        sample.localCentroid = { 0.003333333333, offset - 0.016666666667 };
        const std::array<vec2, 3> vertices{
            vec2{ -0.15f, static_cast<float>( offset - 0.11 ) },
            vec2{ 0.16f, static_cast<float>( offset - 0.10 ) },
            vec2{ 0.0f, static_cast<float>( offset + 0.16 ) } };
        sample.worldInput.geometry = MakePolygon( vertices );
    }

    const vec2 position = TransformPoint( ground,
        { static_cast<float>( sample.origin.x ), static_cast<float>( sample.origin.y ) } );
    const vec2 translation = TransformVector( ground,
        { static_cast<float>( sample.translation.x ),
          static_cast<float>( sample.translation.y ) } );
    sample.worldInput.transform = {
        position, ground.rotation * rot2::FromRadians( static_cast<float>( sample.startAngle ) ) };
    sample.worldInput.translation = translation;
    sample.worldInput.rotationRadians = static_cast<float>( sample.turn );
    return sample;
}

struct frame
{
    std::array<vector2, 4> vertices{};
    int count = 0;
    double radius = 0.0;
    vector2 centroid{};
};

frame evaluate( const caseInput& sample, float fraction )
{
    const double t = fraction;
    const double startC = std::cos( sample.startAngle );
    const double startS = std::sin( sample.startAngle );
    const double endC = std::cos( sample.startAngle + sample.turn );
    const double endS = std::sin( sample.startAngle + sample.turn );
    const double c = ( 1.0 - t ) * startC + t * endC;
    const double s = ( 1.0 - t ) * startS + t * endS;
    const double inv = 1.0 / std::hypot( c, s );
    const vector2 p = sample.origin + sample.translation * t;

    frame pose{};
    pose.count = sample.count;
    pose.radius = sample.radius;
    for( int i = 0; i < sample.count; ++i )
        pose.vertices[i] = p + rotate( sample.core[i], c * inv, s * inv );
    pose.centroid = p + rotate( sample.localCentroid, c * inv, s * inv );
    return pose;
}

double coreSegmentDistance( const frame& shape, const edge& terrain )
{
    double minimum = std::numeric_limits<double>::infinity();
    const int sides = shape.count == 2 ? 1 : shape.count;
    for( int i = 0; i < sides; ++i )
    {
        const int next = shape.count == 2 ? 1 : ( i + 1 ) % shape.count;
        minimum = std::min( minimum,
            segmentDistance2( shape.vertices[i], shape.vertices[next], terrain.a, terrain.b ) );
    }
    // Entire Chain edge inside a convex polygon also means overlap.
    if( shape.count > 2 )
    {
        const auto inside = [&]( vector2 point )
        {
            for( int i = 0; i < shape.count; ++i )
                if( cross( shape.vertices[( i + 1 ) % shape.count] - shape.vertices[i],
                    point - shape.vertices[i] ) < 0.0 ) return false;
            return true;
        };
        if( inside( terrain.a ) || inside( terrain.b ) ) return 0.0;
    }
    return std::sqrt( minimum ) - shape.radius;
}

bool deepFrontCrossing( const frame& shape, const edge& terrain )
{
    const double centroidFront = dot( shape.centroid - terrain.a, terrain.front );
    if( centroidFront <= 0.045 ) return false;

    // Circle-rounded Capsule: closest point distance less than its radius.
    if( shape.count == 2 )
    {
        if( coreSegmentDistance( shape, terrain ) > -MIN_PENETRATION ) return false;
        // The closest candidate contact must lie away from both ghost endpoints.
        for( const vector2 vertex : { shape.vertices[0], shape.vertices[1] } )
        {
            const double projection = dot( vertex - terrain.a, terrain.axis );
            const double signedGap = dot( vertex - terrain.a, terrain.front );
            if( projection > EDGE_MARGIN && projection < terrain.length - EDGE_MARGIN &&
                signedGap < shape.radius - MIN_PENETRATION )
                return true;
        }
        return false;
    }

    // Polygon: a core edge visibly crosses the Chain's front plane well inside
    // the actual finite edge. A plane extension or ghost-only touch cannot qualify.
    double minFront = std::numeric_limits<double>::infinity();
    double maxFront = -std::numeric_limits<double>::infinity();
    for( int i = 0; i < shape.count; ++i )
    {
        const double gap = dot( shape.vertices[i] - terrain.a, terrain.front );
        minFront = std::min( minFront, gap );
        maxFront = std::max( maxFront, gap );
    }
    if( minFront > -MIN_PENETRATION || maxFront < 0.045 ) return false;

    for( int i = 0; i < shape.count; ++i )
    {
        const vector2 a = shape.vertices[i];
        const vector2 b = shape.vertices[( i + 1 ) % shape.count];
        const double da = dot( a - terrain.a, terrain.front );
        const double db = dot( b - terrain.a, terrain.front );
        if( ( da > 0.0 && db > 0.0 ) || ( da < 0.0 && db < 0.0 ) ||
            std::fabs( da - db ) < 1e-12 ) continue;

        const vector2 contact = a + ( b - a ) * ( da / ( da - db ) );
        const double distance = dot( contact - terrain.a, terrain.axis );
        if( distance > EDGE_MARGIN && distance < terrain.length - EDGE_MARGIN )
            return true;
    }
    return false;
}

struct result
{
    bool robust = false;
    bool definiteMiss = false;
    bool nearSeam = false;
    float firstCrossing = 1.0f;
    double startGap = 0.0, endGap = 0.0;
};

result classify( const caseInput& sample, const std::array<edge, 2>& terrain )
{
    result reference{};
    const frame first = evaluate( sample, 0.0f );
    const frame last = evaluate( sample, 1.0f );
    reference.startGap = std::min( coreSegmentDistance( first, terrain[0] ),
        coreSegmentDistance( first, terrain[1] ) );
    reference.endGap = std::min( coreSegmentDistance( last, terrain[0] ),
        coreSegmentDistance( last, terrain[1] ) );

    bool contact = false;
    bool clearlySeparated = true;
    for( int j = 0; j <= SAMPLES; ++j )
    {
        const float fraction = static_cast<float>( j ) / SAMPLES;
        const frame shape = evaluate( sample, fraction );
        const double gap = std::min( coreSegmentDistance( shape, terrain[0] ),
            coreSegmentDistance( shape, terrain[1] ) );
        clearlySeparated &= gap > 0.09;
        if( !contact && deepFrontCrossing( shape, terrain[sample.edgeIndex] ) )
        {
            reference.firstCrossing = fraction;
            contact = true;
        }
    }

    reference.robust = sample.path == pathType::transient &&
        reference.startGap > MIN_ENDPOINT_GAP &&
        reference.endGap > MIN_ENDPOINT_GAP && contact;
    reference.nearSeam = sample.localX > 1.35 || sample.localX < 0.65;
    reference.definiteMiss = clearlySeparated;
    return reference;
}

void run( bool strict, bool csv )
{
    int cases = 0, robust = 0, missed = 0, definiteMiss = 0, falseHits = 0;
    int convexRobust = 0, concaveRobust = 0, seamRobust = 0, seamMissed = 0;
    std::array<int, 3> perShape{}, shapeMissed{};
    int printed = 0;

    if( csv ) std::printf( "case,bend,shape,mode,edge,x,offset,peak,rotation,start_gap,end_gap,robust,hit,fraction\n" );

    for( const float bend : { -0.6f, 0.6f } )
    for( const float worldAngle : { 0.0f, 0.37f } )
    {
        world simulation{};
        const transform2 ground{ { 0.8f, -1.1f }, rot2::FromRadians( worldAngle ) };
        const bodyId body = simulation.CreateBody( bodyType::Static, ground );
        const std::array<vec2, 3> vertices{ vec2{ -2.0f, 0.0f },
            vec2{ 0.0f, 0.0f }, vec2{ 2.0f, bend } };
        chainDef definition{};
        definition.points = vertices;
        definition.ghost1 = { -4.0f, 0.0f };
        definition.ghost2 = { 4.0f, 2.0f * bend };
        const chainId chain = simulation.CreateChain( body, definition );
        std::array<shapeId, 2> handles{};
        check( simulation.GetChainSegments( chain, handles ) == handles.size(),
            "Bent Chain oracle expects exactly two child edges" );

        const std::array<edge, 2> terrain{
            makeEdge( { -2.0, 0.0 }, { 0.0, 0.0 } ),
            makeEdge( { 0.0, 0.0 }, { 2.0, bend } ) };

        for( const shapeType type : { shapeType::capsule, shapeType::box, shapeType::triangle } )
        for( const int edgeIndex : { 0, 1 } )
        for( const double offset : { 0.8, 1.0 } )
        for( const double peak : { -0.055, -0.085 } )
        for( const double x : { 0.42, 0.75, 1.0, 1.25, 1.58 } )
        for( const int direction : { -1, 1 } )
        for( const pathType path : { pathType::transient, pathType::clearFront, pathType::clearBack } )
        {
            const caseInput sample = makeCase( type, path, offset, peak, x,
                direction, edgeIndex, bend, ground, terrain );
            const result oracle = classify( sample, terrain );
            const worldShapeCastResult2 hit = simulation.castShapeSweepClosest( sample.worldInput );
            ++cases;

            if( hit.hit )
            {
                check( ( hit.shape == handles[0] || hit.shape == handles[1] ) &&
                    hit.fraction >= 0.0f && hit.fraction < 1.0f &&
                    IsFinite( hit.point ) && IsFinite( hit.normal ),
                    "Bent Chain rotational cast returned an invalid child hit" );
            }
            if( oracle.robust )
            {
                ++robust;
                ++perShape[static_cast<int>( type )];
                if( bend > 0.0f ) ++convexRobust;
                else ++concaveRobust;
                if( oracle.nearSeam ) ++seamRobust;
                if( !hit.hit )
                {
                    ++missed;
                    ++shapeMissed[static_cast<int>( type )];
                    if( oracle.nearSeam ) ++seamMissed;
                    if( !csv && printed++ < 8 )
                        std::printf( "bent miss: bend=%.3f type=%d edge=%d x=%.3f offset=%.3f peak=%.3f dir=%d start=%.5f end=%.5f first=%.5f\n",
                            bend, static_cast<int>( type ), edgeIndex, x, offset, peak, direction,
                            oracle.startGap, oracle.endGap, oracle.firstCrossing );
                }
            }
            if( oracle.definiteMiss )
            {
                ++definiteMiss;
                if( hit.hit ) ++falseHits;
            }
            if( csv )
                std::printf( "%d,%.3f,%d,%d,%d,%.3f,%.3f,%.3f,%d,%.6f,%.6f,%d,%d,%.6f\n",
                    cases, bend, static_cast<int>( type ), static_cast<int>( path ),
                    edgeIndex, x, offset, peak, direction,
                    oracle.startGap, oracle.endGap, oracle.robust, hit.hit,
                    hit.hit ? hit.fraction : -1.0f );
        }
    }

    check( cases == 2880 && robust >= 40 && concaveRobust >= 10 &&
        convexRobust >= 10 && seamRobust >= 5 && definiteMiss >= 800 &&
        std::all_of( perShape.begin(), perShape.end(),
            []( int n ) { return n >= 8; } ),
        "Bent Chain oracle lacks adequate independently classified coverage" );

    if( !csv )
        std::printf( "Bent Chain rotating oracle: cases=%d robust=%d missed=%d "
            "capsule=%d/%d box=%d/%d triangle=%d/%d convex=%d concave=%d "
            "near_seam=%d seam_missed=%d definite_miss=%d false_hit=%d\n",
            cases, robust, missed, perShape[0], shapeMissed[0],
            perShape[1], shapeMissed[1], perShape[2], shapeMissed[2],
            convexRobust, concaveRobust, seamRobust, seamMissed, definiteMiss, falseHits );
    if( strict )
    {
        check( missed == 0, "Bent Chain cast omitted an independently confirmed front contact" );
        check( falseHits == 0, "Bent Chain cast hit a definitely separated convex shape" );
    }
}

} // namespace

int main( int argc, char** argv )
{
    bool strict = false, csv = false;
    for( int i = 1; i < argc; ++i )
    {
        if( std::strcmp( argv[i], "--strict" ) == 0 ) strict = true;
        else if( std::strcmp( argv[i], "--csv" ) == 0 ) csv = true;
        else
        {
            std::fprintf( stderr, "Unknown bent Chain oracle argument: %s\n", argv[i] );
            return EXIT_FAILURE;
        }
    }
    run( strict, csv );
}
