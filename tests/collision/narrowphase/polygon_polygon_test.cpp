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

bool HasPoint(
    const localManifold2& manifold,
    const vec2& point,
    float epsilon = 1e-5f )
{
    for( std::size_t i = 0; i < manifold.pointCount; ++i )
    {
        if( NearlyEqual( manifold.points[i].point, point, epsilon ) )
        {
            return true;
        }
    }

    return false;
}

int main()
{
    {
        // 서로 떨어진 두 box
        const polygon2 a = MakeBox( { 1.0f, 1.0f } );
        const polygon2 b = MakeBox( { 1.0f, 1.0f } );

        transform2 transformB{};
        transformB.position = { 0.0f, 2.1f };

        const localManifold2 manifold =
            CollidePolygons( a, b, transformB );

        assert( manifold.pointCount == 0 );
    }

    {
        // face끼리 정확히 접촉하면 두 접촉점을 만든다.
        const polygon2 a = MakeBox( { 1.0f, 1.0f } );
        const polygon2 b = MakeBox( { 1.0f, 1.0f } );

        transform2 transformB{};
        transformB.position = { 0.0f, 2.0f };

        const localManifold2 manifold =
            CollidePolygons( a, b, transformB );

        assert( manifold.pointCount == 2 );
        assert( NearlyEqual( manifold.normal, { 0.0f, 1.0f } ) );
        assert( NearlyEqual( manifold.points[0].separation, 0.0f ) );
        assert( NearlyEqual( manifold.points[1].separation, 0.0f ) );
        assert( HasPoint( manifold, { -1.0f, 1.0f } ) );
        assert( HasPoint( manifold, { 1.0f, 1.0f } ) );
    }

    {
        // face끼리 침투한 경우
        const polygon2 a = MakeBox( { 1.0f, 1.0f } );
        const polygon2 b = MakeBox( { 1.0f, 1.0f } );

        transform2 transformB{};
        transformB.position = { 0.0f, 1.5f };

        const localManifold2 manifold =
            CollidePolygons( a, b, transformB );

        assert( manifold.pointCount == 2 );
        assert( NearlyEqual( manifold.normal, { 0.0f, 1.0f } ) );
        assert( NearlyEqual( manifold.points[0].separation, -0.5f ) );
        assert( NearlyEqual( manifold.points[1].separation, -0.5f ) );
    }

    {
        // polygon B의 회전과 이동을 적용해야 한다.
        const polygon2 a = MakeBox( { 1.0f, 1.0f } );
        const polygon2 b = MakeBox( { 1.0f, 0.5f } );

        transform2 transformB{};
        transformB.position = { 1.5f, 0.0f };
        transformB.rotation = rot2::FromRadians( 3.1415926535f * 0.5f );

        const localManifold2 manifold =
            CollidePolygons( a, b, transformB );

        assert( manifold.pointCount == 2 );
        assert( NearlyEqual( manifold.normal, { 1.0f, 0.0f } ) );
        assert( NearlyEqual( manifold.points[0].separation, 0.0f ) );
        assert( NearlyEqual( manifold.points[1].separation, 0.0f ) );
        assert( HasPoint( manifold, { 1.0f, -1.0f } ) );
        assert( HasPoint( manifold, { 1.0f, 1.0f } ) );
    }

    {
        // 두 polygon의 radius를 모두 접촉 반경에 포함해야 한다.
        polygon2 a = MakeBox( { 1.0f, 1.0f } );
        polygon2 b = MakeBox( { 1.0f, 1.0f } );
        a.radius = 0.25f;
        b.radius = 0.25f;

        transform2 transformB{};
        transformB.position = { 0.0f, 2.5f };

        const localManifold2 manifold =
            CollidePolygons( a, b, transformB );

        assert( manifold.pointCount == 2 );
        assert( NearlyEqual( manifold.normal, { 0.0f, 1.0f } ) );
        assert( NearlyEqual( manifold.points[0].separation, 0.0f ) );
        assert( NearlyEqual( manifold.points[1].separation, 0.0f ) );
        assert( HasPoint( manifold, { -1.0f, 1.25f } ) );
        assert( HasPoint( manifold, { 1.0f, 1.25f } ) );
    }

    {
        // 회전된 polygon의 edge가 reference feature가 될 수 있다.
        const polygon2 a = MakeBox( { 1.0f, 1.0f } );
        const polygon2 b = MakeBox( { 0.5f, 0.5f } );

        transform2 transformB{};
        transformB.position = { 0.0f, 1.6f };
        transformB.rotation = rot2::FromRadians( 3.1415926535f * 0.25f );

        const localManifold2 manifold =
            CollidePolygons( a, b, transformB );

        assert( manifold.pointCount > 0 );
        assert( manifold.pointCount <= MAX_LOCAL_MANIFOLD_POINTS );
        assert( NearlyEqual( Length( manifold.normal ), 1.0f, 1e-4f ) );

        for( std::size_t i = 0; i < manifold.pointCount; ++i )
        {
            assert( manifold.points[i].separation <= 0.0f );
        }
    }

    return 0;
}
