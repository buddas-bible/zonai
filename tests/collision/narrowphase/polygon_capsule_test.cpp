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
    const polygon2 box = MakeBox( { 1.0f, 1.0f } );

    {
        // box 위에서 떨어진 capsule
        capsule2 capsule{};
        capsule.center1 = { -1.0f, 0.0f };
        capsule.center2 = { 1.0f, 0.0f };
        capsule.radius = 0.5f;

        transform2 transform{};
        transform.position = { 0.0f, 1.6f };

        const localManifold2 manifold =
            CollidePolygonCapsule( box, capsule, transform );

        assert( manifold.pointCount == 0 );
    }

    {
        // capsule의 긴 면이 box의 위쪽 face에 정확히 닿는다.
        capsule2 capsule{};
        capsule.center1 = { -1.0f, 0.0f };
        capsule.center2 = { 1.0f, 0.0f };
        capsule.radius = 0.5f;

        transform2 transform{};
        transform.position = { 0.0f, 1.5f };

        const localManifold2 manifold =
            CollidePolygonCapsule( box, capsule, transform );

        assert( manifold.pointCount == 2 );
        assert( NearlyEqual( manifold.normal, { 0.0f, 1.0f } ) );
        assert( NearlyEqual( manifold.points[0].separation, 0.0f ) );
        assert( NearlyEqual( manifold.points[1].separation, 0.0f ) );
        assert( HasPoint( manifold, { -1.0f, 1.0f } ) );
        assert( HasPoint( manifold, { 1.0f, 1.0f } ) );
    }

    {
        // capsule이 box의 위쪽 face를 침투한다.
        capsule2 capsule{};
        capsule.center1 = { -1.0f, 0.0f };
        capsule.center2 = { 1.0f, 0.0f };
        capsule.radius = 0.5f;

        transform2 transform{};
        transform.position = { 0.0f, 1.25f };

        const localManifold2 manifold =
            CollidePolygonCapsule( box, capsule, transform );

        assert( manifold.pointCount == 2 );
        assert( NearlyEqual( manifold.normal, { 0.0f, 1.0f } ) );
        assert( NearlyEqual( manifold.points[0].separation, -0.25f ) );
        assert( NearlyEqual( manifold.points[1].separation, -0.25f ) );
    }

    {
        // capsule transform의 회전과 이동을 적용한다.
        capsule2 capsule{};
        capsule.center1 = { -1.0f, 0.0f };
        capsule.center2 = { 1.0f, 0.0f };
        capsule.radius = 0.5f;

        transform2 transform{};
        transform.position = { 1.5f, 0.0f };
        transform.rotation = rot2::FromRadians( 3.1415926535f * 0.5f );

        const localManifold2 manifold =
            CollidePolygonCapsule( box, capsule, transform );

        assert( manifold.pointCount == 2 );
        assert( NearlyEqual( manifold.normal, { 1.0f, 0.0f } ) );
        assert( NearlyEqual( manifold.points[0].separation, 0.0f ) );
        assert( NearlyEqual( manifold.points[1].separation, 0.0f ) );
        assert( HasPoint( manifold, { 1.0f, -1.0f } ) );
        assert( HasPoint( manifold, { 1.0f, 1.0f } ) );
    }

    {
        // capsule의 둥근 끝점이 box corner에 닿는다.
        capsule2 capsule{};
        capsule.center1 = { 0.0f, 0.0f };
        capsule.center2 = { 1.0f, 0.0f };
        capsule.radius = 0.5f;

        transform2 transform{};
        transform.position =
        {
            1.0f + 0.5f / std::sqrt( 2.0f ),
            1.0f + 0.5f / std::sqrt( 2.0f )
        };
        transform.rotation = rot2::FromRadians( 3.1415926535f * 0.25f );

        const localManifold2 manifold =
            CollidePolygonCapsule( box, capsule, transform );

        assert( manifold.pointCount == 1 );
        assert( NearlyEqual(
            manifold.normal,
            { 1.0f / std::sqrt( 2.0f ), 1.0f / std::sqrt( 2.0f ) },
            1e-4f
        ) );
        assert( NearlyEqual( manifold.points[0].point, { 1.0f, 1.0f }, 1e-4f ) );
        assert( NearlyEqual( manifold.points[0].separation, 0.0f, 1e-4f ) );
    }

    return 0;
}
