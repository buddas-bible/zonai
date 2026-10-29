#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <cstdlib>

#include "collision/chainSegmentCastOwnership2.h"
#include "geometry/polygon2.h"

using namespace zonai;

namespace
{

void check( bool value, const char* message )
{
    if( value ) return;
    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}

bool owns( const chainSegment2& chain, const polygon2& polygon,
    const transform2& target, const sweep2& sweep, float fraction )
{
    simplexCache2 cache{};
    const localManifold2 manifold = CollideChainSegmentPolygon( chain, polygon,
        InverseMul( target, GetSweepTransform( sweep, fraction ) ), cache );
    const vec2 face = Normalize( Cross( chain.segment.b - chain.segment.a, 1.0f ) );
    return manifold.pointCount > 0 && Dot( manifold.normal, face ) > 0.0f;
}

struct reference
{
    float first = -1.0f;
    int count = 0;
    int windows = 0;
    int firstSamples = 0;
};

reference sample( const chainSegment2& chain, const polygon2& polygon,
    const transform2& target, const sweep2& sweep, int samples )
{
    reference result{};
    bool previous = false;
    for( int j = 0; j <= samples; ++j )
    {
        const float t = static_cast<float>( j ) / samples;
        const bool hit = owns( chain, polygon, target, sweep, t );
        if( hit && !previous )
        {
            ++result.windows;
            if( result.first < 0.0f ) result.first = t;
        }
        if( hit )
        {
            ++result.count;
            if( result.windows == 1 ) ++result.firstSamples;
        }
        previous = hit;
    }
    return result;
}

struct summary
{
    int cases = 0;
    int robust = 0;
    int narrow = 0;
    int missed = 0;
    int late = 0;
    int exhausted = 0;
    int maxIntervals = 0;
    int maxManifold = 0;
};

void audit( bool strict )
{
    const chainSegment2 chain{
        { -4.0f, 0.0f }, { { -2.0f, 0.0f }, { 0.0f, 0.0f } }, { 2.0f, 0.0f } };
    polygon2 polygon = MakeBox( { 0.5f, 0.05f } );
    for( int j = 0; j < polygon.vertexCount; ++j ) polygon.vertices[j].y += 1.0f;
    polygon.centroid.y += 1.0f;
    const shapeGeometry geometry = polygon;
    const shapeProxy2 proxy = MakeShapeProxy( geometry );

    summary results{};
    int printed = 0;

    for( const float groundAngle : { 0.0f, 0.41f } )
    {
        const transform2 ground{ { 0.6f, -1.4f },
            rot2::FromRadians( groundAngle ) };
        for( const int mode : { 0, 1 } )
        for( int i = 0; i < ( mode == 0 ? 81 : 45 ); ++i )
        {
            const float pivotY = mode == 0 ? -1.176f - 0.0003f * i :
                -1.145f + 0.0003f * i;
            const vec2 start = TransformPoint( ground,
                mode == 0 ? vec2{ -1.0f, pivotY } : vec2{ -25.0f, pivotY } );
            const vec2 end = TransformPoint( ground,
                mode == 0 ? vec2{ -1.0f, pivotY } : vec2{ 25.0f, pivotY } );
            const sweep2 sweep{ {}, start, end,
                ground.rotation * rot2::FromRadians( -0.74f ),
                ground.rotation * rot2::FromRadians( 0.74f ) };

            constexpr int SAMPLES = 16384;
            const reference expected = sample( chain, polygon, ground, sweep, SAMPLES );
            chainSweepRetryStats2 stats{};
            const chainSweepContact2 actual = FindOwnedChainSweepContact(
                chain, geometry, proxy, ground, sweep, 0.0f, 1.0f, &stats );

            ++results.cases;
            results.maxIntervals = std::max( results.maxIntervals, stats.intervals );
            results.maxManifold = std::max( results.maxManifold, stats.manifoldCalls );
            results.exhausted += stats.exhausted ? 1 : 0;

            // Sampling oracle 양성은 최소 4번의 실제 narrowphase hit로 확인.
            const bool robust = expected.first > 0.0f &&
                expected.first < 0.9f && expected.firstSamples >= 4;
            if( !robust ) continue;
            ++results.robust;
            if( expected.firstSamples <= SAMPLES / 512 ) ++results.narrow;
            if( !actual.hit )
            {
                ++results.missed;
                if( printed++ < 12 ) std::printf(
                    "miss mode=%d angle=%.2f pivot=%.6f width=%d "
                    "first=%.7f intervals=%d exhausted=%d\n",
                    mode, groundAngle, pivotY, expected.firstSamples, expected.first,
                    stats.intervals, stats.exhausted );
                continue;
            }
            const float error = actual.fraction - expected.first;
            if( error > 0.002f )
            {
                ++results.late;
                if( printed++ < 12 ) std::printf(
                    "late mode=%d angle=%.2f pivot=%.6f width=%d "
                    "first=%.7f result=%.7f intervals=%d\n",
                    mode, groundAngle, pivotY, expected.firstSamples, expected.first,
                    actual.fraction, stats.intervals );
            }
            check( owns( chain, polygon, ground, sweep, actual.fraction ),
                "Chain retry stress returned a non-owned manifold" );
            check( stats.intervals <= 1024 &&
                stats.distanceCalls <= stats.intervals &&
                stats.manifoldCalls <= stats.intervals + 24,
                "Chain retry stress exceeded its deterministic work budget" );
        }
    }

    std::printf( "Chain retry stress: cases=%d robust=%d narrow=%d missed=%d "
        "late=%d exhausted=%d max_intervals=%d max_manifolds=%d\n",
        results.cases, results.robust, results.narrow, results.missed,
        results.late, results.exhausted, results.maxIntervals, results.maxManifold );
    check( results.robust >= 40, "Insufficient Chain retry stress contact coverage" );
    if( strict )
        check( results.missed == 0 && results.late == 0,
            "Chain retry missed a narrow or high speed front contact" );
}

} // namespace

int main( int argc, char** argv )
{
    if( argc > 2 || ( argc == 2 && std::strcmp( argv[1], "--strict" ) != 0 ) )
    {
        std::fprintf( stderr, "Usage: worldChainRetryStressTests [--strict]\n" );
        return EXIT_FAILURE;
    }
    audit( argc == 2 );
}
