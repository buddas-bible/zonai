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
        // 서로 떨어진 평행 capsule
        capsule2 a{};
        a.center1 = { -1.0f, 0.0f };
        a.center2 = { 1.0f, 0.0f };
        a.radius = 0.5f;

        capsule2 b = a;

        transform2 transformB{};
        transformB.position = { 0.0f, 1.1f };

        const localManifold2 manifold =
            CollideCapsules( a, b, transformB );

        assert( manifold.pointCount == 0 );
    }

    {
        // 끝점끼리 접촉하는 경우
        capsule2 a{};
        a.center1 = { -1.0f, 0.0f };
        a.center2 = { 1.0f, 0.0f };
        a.radius = 0.5f;

        capsule2 b{};
        b.center1 = { 2.0f, 0.0f };
        b.center2 = { 4.0f, 0.0f };
        b.radius = 0.5f;

        const localManifold2 manifold =
            CollideCapsules( a, b, {} );

        assert( manifold.pointCount == 1 );
        assert( NearlyEqual( manifold.normal, { 1.0f, 0.0f } ) );
        assert( NearlyEqual( manifold.points[0].point, { 1.5f, 0.0f } ) );
        assert( NearlyEqual( manifold.points[0].separation, 0.0f ) );
    }

    {
        // 평행한 두 capsule이 옆면 전체로 접촉하는 경우
        capsule2 a{};
        a.center1 = { -1.0f, 0.0f };
        a.center2 = { 1.0f, 0.0f };
        a.radius = 0.5f;

        capsule2 b = a;

        transform2 transformB{};
        transformB.position = { 0.0f, 1.0f };

        const localManifold2 manifold =
            CollideCapsules( a, b, transformB );

        assert( manifold.pointCount == 2 );
        assert( NearlyEqual( manifold.normal, { 0.0f, 1.0f } ) );
        assert( NearlyEqual( manifold.points[0].separation, 0.0f ) );
        assert( NearlyEqual( manifold.points[1].separation, 0.0f ) );

        const vec2 point0 = manifold.points[0].point;
        const vec2 point1 = manifold.points[1].point;

        assert(
            ( NearlyEqual( point0, { -1.0f, 0.5f } ) &&
              NearlyEqual( point1, { 1.0f, 0.5f } ) ) ||
            ( NearlyEqual( point1, { -1.0f, 0.5f } ) &&
              NearlyEqual( point0, { 1.0f, 0.5f } ) )
        );
    }

    {
        // 평행한 두 capsule이 침투하는 경우
        capsule2 a{};
        a.center1 = { -1.0f, 0.0f };
        a.center2 = { 1.0f, 0.0f };
        a.radius = 0.5f;

        capsule2 b = a;

        transform2 transformB{};
        transformB.position = { 0.0f, 0.75f };

        const localManifold2 manifold =
            CollideCapsules( a, b, transformB );

        assert( manifold.pointCount == 2 );
        assert( NearlyEqual( manifold.normal, { 0.0f, 1.0f } ) );
        assert( NearlyEqual( manifold.points[0].separation, -0.25f ) );
        assert( NearlyEqual( manifold.points[1].separation, -0.25f ) );
    }

    {
        // capsule B의 transform을 적용해야 한다.
        capsule2 a{};
        a.center1 = { -1.0f, 0.0f };
        a.center2 = { 1.0f, 0.0f };
        a.radius = 0.5f;

        capsule2 b{};
        b.center1 = { -1.0f, 0.0f };
        b.center2 = { 1.0f, 0.0f };
        b.radius = 0.5f;

        transform2 transformB{};
        transformB.position = { 1.6f, 0.0f };
        transformB.rotation = rot2::FromRadians( 3.1415926535f * 0.5f );

        const localManifold2 manifold =
            CollideCapsules( a, b, transformB );

        assert( manifold.pointCount == 1 );
        assert( NearlyEqual( manifold.normal, { 1.0f, 0.0f } ) );
        assert( NearlyEqual( manifold.points[0].separation, -0.4f ) );
    }

    return 0;
}
