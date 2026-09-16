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
        // 두 원이 서로 겹치지 않는 경우
        circle2 a{};
        a.radius = 1.0f;

        circle2 b{};
        b.radius = 1.0f;

        transform2 transformB{};
        transformB.position = { 3.0f, 0.0f };

        const localManifold2 manifold =
            zonai::CollideCircles(
                a,
                b,
                transformB
            );

        assert( manifold.pointCount == 0 );
    }

    {
        // 두 원이 접촉하는 경우
        circle2 a{};
        a.radius = 1.0f;

        circle2 b{};
        b.radius = 1.0f;

        transform2 transformB{};
        transformB.position = { 2.0f, 0.0f };

        const localManifold2 manifold =
            zonai::CollideCircles(
                a,
                b,
                transformB
            );

        assert( manifold.pointCount == 1 );

        assert( NearlyEqual(
            manifold.normal,
            { 1.0f, 0.0f }
        ) );

        assert( NearlyEqual(
            manifold.points[0].point,
            { 1.0f, 0.0f }
        ) );

        assert( NearlyEqual(
            manifold.points[0].separation,
            0.0f
        ) );
    }

    {
        // 두 원이 겹치는 경우
        circle2 a{};
        a.radius = 1.0f;

        circle2 b{};
        b.radius = 1.0f;

        transform2 transformB{};
        transformB.position = { 1.5f, 0.0f };

        const localManifold2 manifold =
            zonai::CollideCircles(
                a,
                b,
                transformB
            );

        assert( manifold.pointCount == 1 );

        assert( NearlyEqual(
            manifold.points[0].separation,
            -0.5f
        ) );
    }

    {
        // axis 내부
        capsule2 capsule{};
        capsule.center1 = { -1.0f, 0.0f };
        capsule.center2 = { 1.0f, 0.0f };
        capsule.radius = 0.5f;

        circle2 circle{};
        circle.radius = 0.5f;

        transform2 circleTransform{};
        circleTransform.position =
        { 0.0f, 1.0f };

        const localManifold2 manifold =
            zonai::CollideCapsuleCircle(
                capsule,
                circle,
                circleTransform
            );

        assert( manifold.pointCount == 1 );

        assert( NearlyEqual(
            manifold.normal,
            { 0.0f, 1.0f }
        ) );

        assert( NearlyEqual(
            manifold.points[0].separation,
            0.0f
        ) );
    }

    {
        // center1 영역
        capsule2 capsule{};
        capsule.center1 = { -1.0f, 0.0f };
        capsule.center2 = { 1.0f, 0.0f };
        capsule.radius = 0.5f;

        circle2 circle{};
        circle.radius = 0.5f;

        transform2 circleTransform{};
        circleTransform.position =
        { -2.0f, 0.0f };

        const localManifold2 manifold =
            zonai::CollideCapsuleCircle(
                capsule,
                circle,
                circleTransform
            );

        assert( manifold.pointCount == 1 );

        assert( NearlyEqual(
            manifold.normal,
            { -1.0f, 0.0f }
        ) );
    }

    {
        // center2 영역
        capsule2 capsule{};
        capsule.center1 = { -1.0f, 0.0f };
        capsule.center2 = { 1.0f, 0.0f };
        capsule.radius = 0.5f;

        circle2 circle{};
        circle.radius = 0.5f;

        transform2 circleTransform{};
        circleTransform.position =
        { 2.0f, 0.0f };

        const localManifold2 manifold =
            zonai::CollideCapsuleCircle(
                capsule,
                circle,
                circleTransform
            );

        assert( manifold.pointCount == 1 );

        assert( NearlyEqual(
            manifold.normal,
            { 1.0f, 0.0f }
        ) );
    }

    {
        // polygon의 face와 circle이 떨어져 있는 경우
        const polygon2 polygon = MakeBox( { 1.0f, 1.0f } );

        circle2 circle{};
        circle.radius = 0.5f;

        transform2 circleTransform{};
        circleTransform.position = { 0.0f, 1.6f };

        const localManifold2 manifold =
            zonai::CollidePolygonCircle(
                polygon,
                circle,
                circleTransform
            );

        assert( manifold.pointCount == 0 );
    }

    {
        // polygon의 face와 circle이 접촉하는 경우
        const polygon2 polygon = MakeBox( { 1.0f, 1.0f } );

        circle2 circle{};
        circle.radius = 0.5f;

        transform2 circleTransform{};
        circleTransform.position = { 0.0f, 1.5f };

        const localManifold2 manifold =
            zonai::CollidePolygonCircle(
                polygon,
                circle,
                circleTransform
            );

        assert( manifold.pointCount == 1 );
        assert( NearlyEqual( manifold.normal, { 0.0f, 1.0f } ) );
        assert( NearlyEqual( manifold.points[0].point, { 0.0f, 1.0f } ) );
        assert( NearlyEqual( manifold.points[0].separation, 0.0f ) );
    }

    {
        // polygon의 face와 circle이 겹치는 경우
        const polygon2 polygon = MakeBox( { 1.0f, 1.0f } );

        circle2 circle{};
        circle.radius = 0.5f;

        transform2 circleTransform{};
        circleTransform.position = { 0.0f, 1.25f };

        const localManifold2 manifold =
            zonai::CollidePolygonCircle(
                polygon,
                circle,
                circleTransform
            );

        assert( manifold.pointCount == 1 );
        assert( NearlyEqual( manifold.normal, { 0.0f, 1.0f } ) );
        assert( NearlyEqual( manifold.points[0].separation, -0.25f ) );
    }

    {
        // reference edge의 vertex1 영역에서 접촉하는 경우
        const polygon2 polygon = MakeBox( { 1.0f, 1.0f } );

        circle2 circle{};
        circle.radius = 0.5f;

        transform2 circleTransform{};
        circleTransform.position = { 1.4f, -1.3f };

        const localManifold2 manifold =
            zonai::CollidePolygonCircle(
                polygon,
                circle,
                circleTransform
            );

        assert( manifold.pointCount == 1 );
        assert( NearlyEqual( manifold.normal, { 0.8f, -0.6f } ) );
        assert( NearlyEqual( manifold.points[0].point, { 1.0f, -1.0f } ) );
        assert( NearlyEqual( manifold.points[0].separation, 0.0f ) );
    }

    {
        // reference edge의 vertex2 영역에서 접촉하는 경우
        const polygon2 polygon = MakeBox( { 1.0f, 1.0f } );

        circle2 circle{};
        circle.radius = 0.5f;

        transform2 circleTransform{};
        circleTransform.position = { 1.4f, 1.3f };

        const localManifold2 manifold =
            zonai::CollidePolygonCircle(
                polygon,
                circle,
                circleTransform
            );

        assert( manifold.pointCount == 1 );
        assert( NearlyEqual( manifold.normal, { 0.8f, 0.6f } ) );
        assert( NearlyEqual( manifold.points[0].point, { 1.0f, 1.0f } ) );
        assert( NearlyEqual( manifold.points[0].separation, 0.0f ) );
    }

    {
        // circle transform의 회전과 이동을 적용해야 한다.
        const polygon2 polygon = MakeBox( { 1.0f, 1.0f } );

        circle2 circle{};
        circle.center = { 1.0f, 0.0f };
        circle.radius = 0.5f;

        transform2 circleTransform{};
        circleTransform.position = { 0.0f, 0.5f };
        circleTransform.rotation = rot2::FromRadians( 3.1415926535f * 0.5f );

        const localManifold2 manifold =
            zonai::CollidePolygonCircle(
                polygon,
                circle,
                circleTransform
            );

        assert( manifold.pointCount == 1 );
        assert( NearlyEqual( manifold.normal, { 0.0f, 1.0f } ) );
        assert( NearlyEqual( manifold.points[0].point, { 0.0f, 1.0f } ) );
        assert( NearlyEqual( manifold.points[0].separation, 0.0f ) );
    }

    {
        // polygon radius와 circle radius를 모두 접촉 반경에 포함해야 한다.
        polygon2 polygon = MakeBox( { 1.0f, 1.0f } );
        polygon.radius = 0.25f;

        circle2 circle{};
        circle.radius = 0.5f;

        transform2 circleTransform{};
        circleTransform.position = { 0.0f, 1.75f };

        const localManifold2 manifold =
            zonai::CollidePolygonCircle(
                polygon,
                circle,
                circleTransform
            );

        assert( manifold.pointCount == 1 );
        assert( NearlyEqual( manifold.normal, { 0.0f, 1.0f } ) );
        assert( NearlyEqual( manifold.points[0].point, { 0.0f, 1.25f } ) );
        assert( NearlyEqual( manifold.points[0].separation, 0.0f ) );
    }

    return 0;
}
