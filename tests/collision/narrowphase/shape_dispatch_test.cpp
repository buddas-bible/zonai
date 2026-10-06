#include <cassert>
#include <cmath>

#include "collision/narrowphase/collide.h"

using namespace zonai;

namespace
{

bool NearlyEqual( float a, float b, float epsilon = 1e-5f )
{
    return std::fabs( a - b ) <= epsilon;
}

bool NearlyEqual( const vec2& a, const vec2& b, float epsilon = 1e-5f )
{
    return NearlyEqual( a.x, b.x, epsilon ) && NearlyEqual( a.y, b.y, epsilon );
}

} // namespace

int main()
{
    const shapeGeometry circle = circle2{ {}, 0.5f };

    const shapeGeometry polygon = MakeBox( { 1.0f, 1.0f } );

    const shapeGeometry segment = segment2{ { -1.0f, 0.0f }, { 1.0f, 0.0f } };

    assert( CanCollideShapes( circle, polygon ) );
    assert( CanCollideShapes( polygon, circle ) );
    assert( !CanCollideShapes( segment, segment ) );

    {
        // canonical 순서: polygon A, circle B.
        const transform2 transformA{};
        transform2 transformB{};
        transformB.position = { 0.0f, 1.5f };

        const localManifold2 manifold = CollideShapes( polygon, transformA, circle, transformB );

        assert( manifold.pointCount == 1 );
        assert( NearlyEqual( manifold.normal, { 0.0f, 1.0f } ) );
        assert( NearlyEqual( manifold.points[0].point, { 0.0f, 1.0f } ) );
        assert( NearlyEqual( manifold.points[0].separation, 0.0f ) );
    }

    {
        // 입력 순서가 뒤집혀도 결과는 원래 shape A(circle)의 local space여야 함.
        transform2 transformA{};
        transformA.position = { 0.0f, 1.5f };

        const transform2 transformB{};

        const localManifold2 manifold = CollideShapes( circle, transformA, polygon, transformB );

        assert( manifold.pointCount == 1 );

        // Circle A에서 Polygon B를 향하므로 아래쪽이 normal 방향임.
        assert( NearlyEqual( manifold.normal, { 0.0f, -1.0f } ) );

        // world contact (0, 1)은 Circle A local에서는 (0, -0.5)임.
        assert( NearlyEqual( manifold.points[0].point, { 0.0f, -0.5f } ) );
        assert( NearlyEqual( manifold.points[0].separation, 0.0f ) );
    }

    {
        // A 자체가 회전/이동되어 있어도 relative transform으로 B를 A local에 맞춰야 함.
        transform2 transformA{};
        transformA.position = { 2.0f, 3.0f };
        transformA.rotation = rot2::FromRadians( 3.1415926535f * 0.5f );

        transform2 transformB{};
        transformB.position = { 0.5f, 3.0f };

        const localManifold2 manifold = CollideShapes( polygon, transformA, circle, transformB );

        assert( manifold.pointCount == 1 );
        assert( NearlyEqual( manifold.normal, { 0.0f, 1.0f }, 1e-4f ) );
        assert( NearlyEqual( manifold.points[0].point, { 0.0f, 1.0f }, 1e-4f ) );
        assert( NearlyEqual( manifold.points[0].separation, 0.0f, 1e-4f ) );
    }

    {
        const localManifold2 manifold = CollideShapes( segment, {}, segment, {} );

        assert( manifold.pointCount == 0 );
    }

    return 0;
}
