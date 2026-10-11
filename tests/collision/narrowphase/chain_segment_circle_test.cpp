#include <cassert>
#include <cmath>

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
    const chainSegment2 chain{
        { 2.0f, 0.0f },
        { { 1.0f, 0.0f }, { -1.0f, 0.0f } },
        { -2.0f, 0.0f }
    };

    circle2 circle{};
    circle.radius = 0.5f;

    {
        // point1 -> point2의 오른쪽이 앞면이다. 이 방향에서는 위쪽이 terrain의 충돌 면이다.
        transform2 circleTransform{};
        circleTransform.position = { 0.0f, 0.5f };

        const localManifold2 manifold = CollideChainSegmentCircle( chain, circle, circleTransform );

        assert( manifold.pointCount == 1 );
        assert( NearlyEqual( manifold.normal, { 0.0f, 1.0f } ) );
        assert( NearlyEqual( manifold.points[0].point, { 0.0f, 0.0f } ) );
        assert( NearlyEqual( manifold.points[0].separation, 0.0f ) );
    }

    {
        // Chain segment는 one-sided이므로 뒷면에서 겹쳐도 contact를 만들지 않는다.
        transform2 circleTransform{};
        circleTransform.position = { 0.0f, -0.25f };

        const localManifold2 manifold = CollideChainSegmentCircle( chain, circle, circleTransform );

        assert( manifold.pointCount == 0 );
    }

    {
        // point1 바깥은 이전 edge의 Voronoi 영역이다. ghost1이 이 endpoint의 가짜 normal을 막아야 한다.
        transform2 circleTransform{};
        circleTransform.position = { 1.2f, 0.2f };

        const localManifold2 manifold = CollideChainSegmentCircle( chain, circle, circleTransform );

        assert( manifold.pointCount == 0 );
    }

    {
        // point2 바깥은 다음 edge가 담당하므로 ghost2가 현재 endpoint contact를 막아야 한다.
        transform2 circleTransform{};
        circleTransform.position = { -1.2f, 0.2f };

        const localManifold2 manifold = CollideChainSegmentCircle( chain, circle, circleTransform );

        assert( manifold.pointCount == 0 );
    }

    return 0;
}
