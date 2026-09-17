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
        // segment와 circle이 떨어져 있는 경우
        const segment2 segment{ { -1.0f, 0.0f }, { 1.0f, 0.0f } };

        circle2 circle{};
        circle.radius = 0.5f;

        transform2 circleTransform{};
        circleTransform.position = { 0.0f, 1.0f };

        const localManifold2 manifold =
            zonai::CollideSegmentCircle(
                segment,
                circle,
                circleTransform
            );

        assert( manifold.pointCount == 0 );
    }

    {
        // segment의 내부 영역과 circle이 접촉하는 경우
        const segment2 segment{ { -1.0f, 0.0f }, { 1.0f, 0.0f } };

        circle2 circle{};
        circle.radius = 0.5f;

        transform2 circleTransform{};
        circleTransform.position = { 0.0f, 0.5f };

        const localManifold2 manifold =
            zonai::CollideSegmentCircle(
                segment,
                circle,
                circleTransform
            );

        assert( manifold.pointCount == 1 );
        assert( NearlyEqual( manifold.normal, { 0.0f, 1.0f } ) );
        assert( NearlyEqual( manifold.points[0].point, { 0.0f, 0.0f } ) );
        assert( NearlyEqual( manifold.points[0].separation, 0.0f ) );
    }

    {
        // segment endpoint와 circle이 접촉하는 경우
        const segment2 segment{ { -1.0f, 0.0f }, { 1.0f, 0.0f } };

        circle2 circle{};
        circle.radius = 0.5f;

        transform2 circleTransform{};
        circleTransform.position = { 1.3f, 0.4f };

        const localManifold2 manifold =
            zonai::CollideSegmentCircle(
                segment,
                circle,
                circleTransform
            );

        assert( manifold.pointCount == 1 );
        assert( NearlyEqual( manifold.normal, { 0.6f, 0.8f } ) );
        assert( NearlyEqual( manifold.points[0].point, { 1.0f, 0.0f } ) );
        assert( NearlyEqual( manifold.points[0].separation, 0.0f ) );
    }

    {
        // segment의 내부 영역과 circle이 겹치는 경우
        const segment2 segment{ { -1.0f, 0.0f }, { 1.0f, 0.0f } };

        circle2 circle{};
        circle.radius = 0.5f;

        transform2 circleTransform{};
        circleTransform.position = { 0.0f, 0.25f };

        const localManifold2 manifold =
            zonai::CollideSegmentCircle(
                segment,
                circle,
                circleTransform
            );

        assert( manifold.pointCount == 1 );
        assert( NearlyEqual( manifold.normal, { 0.0f, 1.0f } ) );
        assert( NearlyEqual( manifold.points[0].separation, -0.25f ) );
    }

    return 0;
}
