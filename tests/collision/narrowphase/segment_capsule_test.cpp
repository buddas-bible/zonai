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
        // 서로 떨어진 segment와 capsule
        const segment2 segment{ { -1.0f, 0.0f }, { 1.0f, 0.0f } };

        capsule2 capsule{};
        capsule.center1 = { -1.0f, 0.0f };
        capsule.center2 = { 1.0f, 0.0f };
        capsule.radius = 0.5f;

        transform2 capsuleTransform{};
        capsuleTransform.position = { 0.0f, 0.6f };

        const localManifold2 manifold =
            CollideSegmentCapsule( segment, capsule, capsuleTransform );

        assert( manifold.pointCount == 0 );
    }

    {
        // capsule의 옆면이 segment에 정확히 접촉하면 두 접촉점을 만든다.
        const segment2 segment{ { -1.0f, 0.0f }, { 1.0f, 0.0f } };

        capsule2 capsule{};
        capsule.center1 = { -1.0f, 0.0f };
        capsule.center2 = { 1.0f, 0.0f };
        capsule.radius = 0.5f;

        transform2 capsuleTransform{};
        capsuleTransform.position = { 0.0f, 0.5f };

        const localManifold2 manifold =
            CollideSegmentCapsule( segment, capsule, capsuleTransform );

        assert( manifold.pointCount == 2 );
        assert( NearlyEqual( manifold.normal, { 0.0f, 1.0f } ) );
        assert( NearlyEqual( manifold.points[0].separation, 0.0f ) );
        assert( NearlyEqual( manifold.points[1].separation, 0.0f ) );
    }

    {
        // capsule의 둥근 끝이 segment 끝점에 접촉하는 경우
        const segment2 segment{ { -1.0f, 0.0f }, { 1.0f, 0.0f } };

        capsule2 capsule{};
        capsule.center1 = { 1.5f, 0.0f };
        capsule.center2 = { 2.5f, 0.0f };
        capsule.radius = 0.5f;

        const localManifold2 manifold =
            CollideSegmentCapsule( segment, capsule, {} );

        assert( manifold.pointCount == 1 );
        assert( NearlyEqual( manifold.normal, { 1.0f, 0.0f } ) );
        assert( NearlyEqual( manifold.points[0].point, { 1.0f, 0.0f } ) );
        assert( NearlyEqual( manifold.points[0].separation, 0.0f ) );
    }

    return 0;
}
