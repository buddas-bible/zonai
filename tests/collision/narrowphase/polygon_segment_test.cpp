#include <cassert>
#include <cmath>

#include "collision/distance2.h"
#include "collision/narrowphase/collide.h"

using namespace zonai;

bool NearlyEqual( float a, float b, float epsilon = 1e-5f )
{
    return std::fabs( a - b ) <= epsilon;
}

bool NearlyEqual( const vec2& a, const vec2& b, float epsilon = 1e-5f )
{
    return NearlyEqual( a.x, b.x, epsilon ) && NearlyEqual( a.y, b.y, epsilon );
}

int main()
{
    {
        // polygon과 segment가 떨어진 경우
        const polygon2 polygon = MakeBox( { 1.0f, 1.0f } );
        const segment2 segment{ { -0.5f, 0.0f }, { 0.5f, 0.0f } };

        transform2 segmentTransform{};
        segmentTransform.position = { 0.0f, 1.1f };

        const localManifold2 manifold = CollidePolygonSegment( polygon, segment, segmentTransform );

        assert( manifold.pointCount == 0 );
    }

    {
        // polygon face와 segment가 정확히 접촉하면 두 접촉점을 만든다.
        const polygon2 polygon = MakeBox( { 1.0f, 1.0f } );
        const segment2 segment{ { -0.5f, 0.0f }, { 0.5f, 0.0f } };

        transform2 segmentTransform{};
        segmentTransform.position = { 0.0f, 1.0f };

        const localManifold2 manifold = CollidePolygonSegment( polygon, segment, segmentTransform );

        assert( manifold.pointCount == 2 );
        assert( NearlyEqual( manifold.normal, { 0.0f, 1.0f } ) );
        assert( NearlyEqual( manifold.points[0].separation, 0.0f ) );
        assert( NearlyEqual( manifold.points[1].separation, 0.0f ) );
    }

    {
        // segment transform의 회전을 적용해야 한다.
        const polygon2 polygon = MakeBox( { 1.0f, 1.0f } );
        const segment2 segment{ { -0.5f, 0.0f }, { 0.5f, 0.0f } };

        transform2 segmentTransform{};
        segmentTransform.position = { 1.0f, 0.0f };
        segmentTransform.rotation = rot2::FromRadians( 3.1415926535f * 0.5f );

        const localManifold2 manifold = CollidePolygonSegment( polygon, segment, segmentTransform );

        assert( manifold.pointCount == 2 );
        assert( NearlyEqual( manifold.normal, { 1.0f, 0.0f } ) );
        assert( NearlyEqual( manifold.points[0].separation, 0.0f ) );
        assert( NearlyEqual( manifold.points[1].separation, 0.0f ) );
    }

    const chainSegment2 chain{
        { 2.0f, 0.0f },
        { { 1.0f, 0.0f }, { -1.0f, 0.0f } },
        { -2.0f, 0.0f }
    };

    {
        // Chain 앞면에 box face가 닿으면 segment가 reference edge가 되어 두 점을 만든다.
        const polygon2 polygon = MakeBox( { 0.5f, 0.5f } );
        transform2 polygonTransform{};
        polygonTransform.position = { 0.0f, 0.5f };
        simplexCache2 cache{};

        const localManifold2 manifold = CollideChainSegmentPolygon( chain, polygon, polygonTransform, cache );

        assert( manifold.pointCount == 2 );
        assert( NearlyEqual( manifold.normal, { 0.0f, 1.0f } ) );
        assert( NearlyEqual( manifold.points[0].separation, 0.0f ) );
        assert( NearlyEqual( manifold.points[1].separation, 0.0f ) );
    }

    {
        // 같은 polygon이 뒷면에서 겹쳐도 one-sided chain은 contact를 만들지 않는다.
        const polygon2 polygon = MakeBox( { 0.5f, 0.5f } );
        transform2 polygonTransform{};
        polygonTransform.position = { 0.0f, -0.25f };
        simplexCache2 cache{};

        const localManifold2 manifold = CollideChainSegmentPolygon( chain, polygon, polygonTransform, cache );

        assert( manifold.pointCount == 0 );
    }

    {
        // Rounded core가 point1과 겹쳐도 실제 영역이 이전 edge 쪽이면 ghost1이 현재 edge의 endpoint normal을 막는다.
        polygon2 polygon = MakeBox( { 0.05f, 0.05f } );
        polygon.radius = 0.3f;
        transform2 polygonTransform{};
        polygonTransform.position = { 1.2f, 0.2f };
        simplexCache2 cache{};

        const localManifold2 manifold = CollideChainSegmentPolygon( chain, polygon, polygonTransform, cache );

        assert( manifold.pointCount == 0 );
    }

    return 0;
}
