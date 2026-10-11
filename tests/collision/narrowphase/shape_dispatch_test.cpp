#include <cassert>
#include <cmath>

#include "collision/distance2.h"
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
    const shapeGeometry capsule = capsule2{ { -0.5f, 0.0f }, { 0.5f, 0.0f }, 0.5f };
    const shapeGeometry polygon = MakeBox( { 1.0f, 1.0f } );
    const shapeGeometry segment = segment2{ { -1.0f, 0.0f }, { 1.0f, 0.0f } };
    const shapeGeometry chain = chainSegment2{ { 2.0f, 0.0f }, { { 1.0f, 0.0f }, { -1.0f, 0.0f } }, { -2.0f, 0.0f } };

    assert( CanCollideShapes( circle, polygon ) );
    assert( CanCollideShapes( polygon, circle ) );
    assert( !CanCollideShapes( segment, segment ) );
    assert( CanCollideShapes( chain, circle ) );
    assert( CanCollideShapes( circle, chain ) );
    assert( CanCollideShapes( chain, capsule ) );
    assert( CanCollideShapes( chain, polygon ) );
    assert( !CanCollideShapes( chain, segment ) );
    assert( !CanCollideShapes( segment, chain ) );
    assert( !CanCollideShapes( chain, chain ) );

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
        assert( NearlyEqual( manifold.normal, { 0.0f, -1.0f } ) );
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

    {
        // Chain-circle은 cache를 쓰지 않아도 같은 dispatch API를 공유함.
        transform2 circleTransform{};
        circleTransform.position = { 0.0f, 0.5f };
        simplexCache2 cache{};

        const localManifold2 manifold = CollideShapes( chain, {}, circle, circleTransform, cache );

        assert( manifold.pointCount == 1 );
        assert( NearlyEqual( manifold.normal, { 0.0f, 1.0f } ) );
    }

    {
        // Reverse dispatch는 원래 A(circle) local space로 normal과 point를 되돌림.
        transform2 circleTransform{};
        circleTransform.position = { 0.0f, 0.5f };
        simplexCache2 cache{};

        const localManifold2 manifold = CollideShapes( circle, circleTransform, chain, {}, cache );

        assert( manifold.pointCount == 1 );
        assert( NearlyEqual( manifold.normal, { 0.0f, -1.0f } ) );
        assert( NearlyEqual( manifold.points[0].point, { 0.0f, -0.5f } ) );
    }

    {
        // Chain-polygon은 Contact가 보관한 simplex cache를 같은 dispatch를 통해 갱신함.
        transform2 polygonTransform{};
        polygonTransform.position = { 0.0f, 1.0f };
        simplexCache2 cache{};

        const localManifold2 manifold = CollideShapes( chain, {}, polygon, polygonTransform, cache );

        assert( manifold.pointCount == 2 );
        assert( NearlyEqual( manifold.normal, { 0.0f, 1.0f } ) );
        assert( cache.count > 0 );
    }

    {
        transform2 capsuleTransform{};
        capsuleTransform.position = { 0.0f, 0.5f };
        simplexCache2 cache{};

        const localManifold2 manifold = CollideShapes( chain, {}, capsule, capsuleTransform, cache );

        assert( manifold.pointCount == 2 );
        assert( NearlyEqual( manifold.normal, { 0.0f, 1.0f } ) );
        assert( cache.count > 0 );
    }

    return 0;
}
