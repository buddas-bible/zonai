#include <algorithm>
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
    int fastRobust = 0;
    int fastNarrow = 0;
    int missed = 0;
    int late = 0;
    int exhausted = 0;
    int maxIntervals = 0;
    int maxManifold = 0;
    int maxDepth = 0;
    float maxDelay = 0.0f;
    float maxEarly = 0.0f;
    int measuredPairs = 0;
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
                -0.98f - 0.003f * i;
            const vec2 start = TransformPoint( ground,
                mode == 0 ? vec2{ -1.0f, pivotY } : vec2{ -1001.0f, pivotY } );
            const vec2 end = TransformPoint( ground,
                mode == 0 ? vec2{ -1.0f, pivotY } : vec2{ 999.0f, pivotY } );
            const sweep2 sweep{ {}, start, end,
                ground.rotation * rot2::FromRadians( -0.74f ),
                ground.rotation * rot2::FromRadians( 0.74f ) };

            constexpr int SAMPLES = 16384;
            const reference expected = sample( chain, polygon, ground, sweep, SAMPLES );
            chainSweepRetryStats2 stats{};
            const chainSweepContact2 actual = FindOwnedChainSweepContact(
                chain, geometry, proxy, ground, sweep, 0.0f, 1.0f, &stats );
            const chainSweepContact2 unmeasured = FindOwnedChainSweepContact(
                chain, geometry, proxy, ground, sweep, 0.0f, 1.0f );
            check( actual.hit == unmeasured.hit &&
                ( !actual.hit || ( actual.fraction == unmeasured.fraction &&
                    LengthSquared( actual.point - unmeasured.point ) == 0.0f &&
                    LengthSquared( actual.normal - unmeasured.normal ) == 0.0f ) ),
                "Chain retry diagnostics changed the result" );

            ++results.cases;
            results.maxIntervals = std::max( results.maxIntervals, stats.intervals );
            results.maxManifold = std::max( results.maxManifold, stats.manifoldCalls );
            results.maxDepth = std::max( results.maxDepth, stats.depthLimit );
            results.exhausted += stats.exhausted ? 1 : 0;
            check( stats.depthLimit >= ( mode == 1 ? 12 : 9 ),
                "High-speed sweep did not receive adaptive Chain retry depth" );
            check( stats.intervals <= 1024 &&
                stats.distanceCalls <= stats.intervals &&
                stats.manifoldCalls <= stats.intervals + 24,
                "Chain retry stress exceeded its deterministic work budget" );

            // Sampling oracle 양성은 최소 4번의 실제 narrowphase hit로 확인.
            const bool robust = expected.first > 0.0f &&
                expected.first < 0.9f && expected.firstSamples >= 4;
            if( !robust ) continue;
            ++results.robust;
            if( mode == 1 ) ++results.fastRobust;
            if( expected.firstSamples <= SAMPLES / 512 )
            {
                ++results.narrow;
                if( mode == 1 ) ++results.fastNarrow;
            }
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
            results.maxDelay = std::max( results.maxDelay, error );
            results.maxEarly = std::max( results.maxEarly, -error );
            ++results.measuredPairs;
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

        }
    }

    // 접촉이 사라지는 pivot 경계를 이분 탐색해 depth-9 leaf보다
    // 좁은 실제 narrowphase 접촉 창을 수동 수치 선택 없이 재현함.
    for( const float groundAngle : { 0.0f, 0.41f } )
    for( const float phase : { 0.0f, 0.0027f, 0.0054f } )
    {
        const transform2 ground{ { 0.6f, -1.4f },
            rot2::FromRadians( groundAngle ) };
        const auto makeSweep = [&]( float pivotY )
        {
            const vec2 pivot = TransformPoint( ground, { -1.0f, pivotY } );
            return sweep2{ {}, pivot, pivot,
                ground.rotation * rot2::FromRadians( -0.74f + phase ),
                ground.rotation * rot2::FromRadians( 0.74f + phase ) };
        };

        float low = -1.19f, high = -1.1828f;
        constexpr int FINE_SAMPLES = 32768;
        check( sample( chain, polygon, ground, makeSweep( low ), FINE_SAMPLES ).count == 0 &&
            sample( chain, polygon, ground, makeSweep( high ), FINE_SAMPLES ).count > 0,
            "Grazing pivot bracket did not straddle the contact boundary" );
        for( int i = 0; i < 22; ++i )
        {
            const float middle = low + 0.5f * ( high - low );
            if( middle <= low || middle >= high ) break;
            if( sample( chain, polygon, ground, makeSweep( middle ), FINE_SAMPLES ).count > 0 )
                high = middle;
            else low = middle;
        }
        std::printf( "Grazing boundary: angle=%.2f phase=%.4f threshold=%.8f\n",
            groundAngle, phase, high );
        for( int i = 1; i <= 12; ++i )
        {
            const float pivotY = high + 0.0000005f * i;
            const sweep2 moving = makeSweep( pivotY );
            const reference expected = sample( chain, polygon, ground, moving, FINE_SAMPLES );
            chainSweepRetryStats2 stats{};
            const chainSweepContact2 actual = FindOwnedChainSweepContact(
                chain, geometry, proxy, ground, moving, 0.0f, 1.0f, &stats );
            ++results.cases;
            results.maxIntervals = std::max( results.maxIntervals, stats.intervals );
            results.maxManifold = std::max( results.maxManifold, stats.manifoldCalls );
            results.maxDepth = std::max( results.maxDepth, stats.depthLimit );
            results.exhausted += stats.exhausted ? 1 : 0;
            check( stats.intervals <= 1024 &&
                stats.distanceCalls <= stats.intervals &&
                stats.manifoldCalls <= stats.intervals + 24,
                "Grazing Chain retry exceeded its deterministic work budget" );
            if( expected.firstSamples < 4 || expected.first <= 0.0f ) continue;
            ++results.robust;
            if( expected.firstSamples <= FINE_SAMPLES / 512 ) ++results.narrow;
            if( !actual.hit )
            {
                ++results.missed;
                if( printed++ < 12 ) std::printf(
                    "grazing miss: angle=%.2f phase=%.4f pivot=%.8f width=%d first=%.7f intervals=%d exhausted=%d\n",
                    groundAngle, phase, pivotY, expected.firstSamples, expected.first,
                    stats.intervals, stats.exhausted );
            }
            else
            {
                const float error = actual.fraction - expected.first;
                results.maxDelay = std::max( results.maxDelay, error );
                results.maxEarly = std::max( results.maxEarly, -error );
                ++results.measuredPairs;
                if( error > 0.002f )
                {
                    ++results.late;
                    if( printed++ < 12 ) std::printf(
                        "grazing late: angle=%.2f phase=%.4f pivot=%.8f width=%d first=%.7f result=%.7f intervals=%d\n",
                        groundAngle, phase, pivotY, expected.firstSamples, expected.first,
                        actual.fraction, stats.intervals );
                }
            }
            if( actual.hit ) check( owns( chain, polygon, ground, moving, actual.fraction ),
                "Grazing Chain retry returned a non-owned manifold" );
        }
    }

    std::printf( "Chain retry stress: cases=%d robust=%d narrow=%d fast_robust=%d "
        "fast_narrow=%d missed=%d late=%d exhausted=%d "
        "max_intervals=%d max_manifolds=%d max_depth=%d "
        "fraction_pairs=%d max_late=%.7f max_early=%.7f\n",
        results.cases, results.robust, results.narrow, results.fastRobust,
        results.fastNarrow, results.missed, results.late, results.exhausted,
        results.maxIntervals, results.maxManifold, results.maxDepth,
        results.measuredPairs, results.maxDelay, results.maxEarly );
    check( results.cases == 324 && results.robust >= 100 && results.narrow >= 40 &&
        results.fastRobust >= 20 && results.fastNarrow >= 20,
        "Insufficient Chain retry stress contact coverage" );
    if( strict )
    {
        check( results.missed == 0 && results.late == 0,
            "Chain retry missed a narrow or high speed front contact" );
        check( results.exhausted == 0,
            "Chain retry exhausted its search budget on a covered contact" );
        check( results.measuredPairs == results.robust &&
            results.maxDelay <= 0.00025f && results.maxEarly <= 0.00025f,
            "Chain retry first-owned fraction differs too far from the sampled reference" );
    }
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
