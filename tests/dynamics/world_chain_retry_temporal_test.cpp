#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "collision/chainSegmentCastOwnership2.h"
#include "dynamics/world.h"
#include "geometry/polygon2.h"

using namespace zonai;

namespace
{

void check( bool condition, const char* message )
{
    if( condition ) return;
    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}

bool ownedAt( const chainSegment2& chain, const polygon2& geometry,
    const transform2& target, const sweep2& moving, float fraction )
{
    const transform2 relative = InverseMul( target,
        GetSweepTransform( moving, fraction ) );
    simplexCache2 cache{};
    const localManifold2 manifold =
        CollideChainSegmentPolygon( chain, geometry, relative, cache );
    const vec2 face = Normalize( Cross( chain.segment.b - chain.segment.a, 1.0f ) );
    return manifold.pointCount > 0 && Dot( manifold.normal, face ) > 0.0f;
}

void audit( float angle, float pivotY, bool shortWindow )
{
    const chainSegment2 chain{
        { -4.0f, 0.0f }, { { -2.0f, 0.0f }, { 0.0f, 0.0f } }, { 2.0f, 0.0f } };
    polygon2 geometry = MakeBox( { 0.5f, 0.05f } );
    for( int i = 0; i < geometry.vertexCount; ++i )
        geometry.vertices[i].y += 1.0f;
    geometry.centroid.y += 1.0f;

    const transform2 target{ { 0.6f, -1.4f }, rot2::FromRadians( angle ) };
    const vec2 origin = TransformPoint( target, { -1.0f, pivotY } );
    const sweep2 moving{
        {}, origin, origin,
        target.rotation * rot2::FromRadians( -0.74f ),
        target.rotation * rot2::FromRadians( 0.74f )
    };
    const shapeGeometry shape = geometry;
    const shapeProxy2 proxy = MakeShapeProxy( shape );

    // Generic narrowphase의 '첫 owned 접촉'만 sampling reference로 사용함.
    // 시간순 탐색 자체를 검증하므로 TOI/World/retry를 oracle에 재사용하지 않음.
    constexpr int SAMPLES = 16384;
    float first = -1.0f;
    int windows = 0;
    int firstWidthSamples = 0;
    bool wasOwned = false;
    for( int index = 0; index <= SAMPLES; ++index )
    {
        const float fraction = static_cast<float>( index ) / SAMPLES;
        const bool owned = ownedAt( chain, geometry, target, moving, fraction );
        if( owned && !wasOwned )
        {
            ++windows;
            if( first < 0.0f ) first = fraction;
        }
        if( owned && windows == 1 ) ++firstWidthSamples;
        wasOwned = owned;
    }

    check( windows == 2 && first > 0.0f && first < 0.5f,
        "The fixture must contain two separate owned contact windows" );
    if( shortWindow )
        check( firstWidthSamples < SAMPLES / 12,
            "The narrow transient window is wider than intended" );

    chainSweepRetryStats2 stats{};
    const chainSweepContact2 recovered = FindOwnedChainSweepContact(
        chain, shape, proxy, target, moving, 0.0f, 1.0f, &stats );
    check( recovered.hit && ownedAt( chain, geometry, target, moving, recovered.fraction ),
        "Ordered retry returned a non-owned manifold" );
    check( std::fabs( recovered.fraction - first ) < 0.006f,
        "Ordered retry returned the later window instead of the earliest" );
    check( IsFinite( recovered.point ) && IsFinite( recovered.normal ),
        "Ordered retry produced nonfinite contact data" );
    check( stats.intervals <= 1024 && stats.distanceCalls <= stats.intervals &&
        stats.manifoldCalls <= stats.intervals + 24 && !stats.exhausted,
        "Ordered retry exceeded the bounded work budget" );

    std::printf( "Chain retry: ground=%.2f pivot=%.5f windows=%d first=%.6f "
        "recovered=%.6f width=%d intervals=%d distances=%d manifolds=%d\n",
        angle, pivotY, windows, first, recovered.fraction, firstWidthSamples,
        stats.intervals, stats.distanceCalls, stats.manifoldCalls );
}

} // namespace

int main()
{
    for( const float angle : { 0.0f, 0.41f } )
    {
        audit( angle, -1.14f, false );
        audit( angle, -1.1828f, true );
    }
}
