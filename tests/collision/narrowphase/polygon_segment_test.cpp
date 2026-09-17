#include <cassert>
#include <cmath>

#include "collision/narrowphase/collide.h"

using namespace zonai;

bool NearlyEqual(
    float a,
    float b,
    float epsilon = 1e-5f )
{
    return std::fabs( a - b ) <= epsilon;
}

bool NearlyEqual(
    const vec2& a,
    const vec2& b,
    float epsilon = 1e-5f )
{
    return NearlyEqual( a.x, b.x, epsilon ) &&
        NearlyEqual( a.y, b.y, epsilon );
}

int main()
{
    {
        // polygon과 segment가 떨어진 경우
        const polygon2 polygon = MakeBox( { 1.0f, 1.0f } );
        const segment2 segment{ { -0.5f, 0.0f }, { 0.5f, 0.0f } };

        transform2 segmentTransform{};
        segmentTransform.position = { 0.0f, 1.1f };

        const localManifold2 manifold =
            CollidePolygonSegment( polygon, segment, segmentTransform );

        assert( manifold.pointCount == 0 );
    }

    {
        // polygon face와 segment가 정확히 접촉하면 두 접촉점을 만든다.
        const polygon2 polygon = MakeBox( { 1.0f, 1.0f } );
        const segment2 segment{ { -0.5f, 0.0f }, { 0.5f, 0.0f } };

        transform2 segmentTransform{};
        segmentTransform.position = { 0.0f, 1.0f };

        const localManifold2 manifold =
            CollidePolygonSegment( polygon, segment, segmentTransform );

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
        segmentTransform.rotation =
            rot2::FromRadians( 3.1415926535f * 0.5f );

        const localManifold2 manifold =
            CollidePolygonSegment( polygon, segment, segmentTransform );

        assert( manifold.pointCount == 2 );
        assert( NearlyEqual( manifold.normal, { 1.0f, 0.0f } ) );
        assert( NearlyEqual( manifold.points[0].separation, 0.0f ) );
        assert( NearlyEqual( manifold.points[1].separation, 0.0f ) );
    }

    return 0;
}
