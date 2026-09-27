#include "collision/narrowphase/collide.h"

#include <type_traits>
#include <variant>

namespace zonai
{
namespace
{

// canonical collider를 B/A 순서로 호출한 결과를 원래 A의 local space로 되돌림.
localManifold2 FlipManifoldToA(
    localManifold2 manifold,
    const transform2& transformA,
    const transform2& transformB )
{
    if( manifold.pointCount == 0 )
    {
        return manifold;
    }

    // canonical 호출의 normal은 B -> A이므로 원래 A -> B 방향으로 뒤집음.
    const vec2 worldNormal =
        TransformVector( transformB, manifold.normal );

    manifold.normal =
        InverseTransformVector( transformA, -worldNormal );

    for( int i = 0; i < manifold.pointCount; ++i )
    {
        const vec2 worldPoint =
            TransformPoint( transformB, manifold.points[i].point );

        manifold.points[i].point =
            InverseTransformPoint( transformA, worldPoint );
    }

    return manifold;
}

template <typename GeometryA, typename GeometryB>
constexpr bool IsSupportedPair()
{
    using A = std::remove_cvref_t<GeometryA>;
    using B = std::remove_cvref_t<GeometryB>;

    if constexpr(
        std::is_same_v<A, std::monostate> ||
        std::is_same_v<B, std::monostate> )
    {
        return false;
    }
    else if constexpr(
        std::is_same_v<A, segment2> &&
        std::is_same_v<B, segment2> )
    {
        // Box2D와 동일하게 segment-segment contact는 지원하지 않음.
        return false;
    }
    else
    {
        return true;
    }
}

} // namespace

bool CanCollideShapes(
    const ShapeGeometry& geometryA,
    const ShapeGeometry& geometryB )
{
    return std::visit(
        []( const auto& a, const auto& b )
        {
            return IsSupportedPair<
                decltype( a ),
                decltype( b )
            >();
        },
        geometryA,
        geometryB
    );
}

localManifold2 CollideShapes(
    const ShapeGeometry& geometryA,
    const transform2& transformA,
    const ShapeGeometry& geometryB,
    const transform2& transformB )
{
    // 기존 collider들은 A local space에서 계산하므로 B를 A local space로 변환함.
    const transform2 transformBToA =
        InverseMul( transformA, transformB );

    // 뒤집힌 canonical 호출에 필요한 A -> B 상대 transform.
    const transform2 transformAToB =
        InverseMul( transformB, transformA );

    return std::visit(
        [&]( const auto& a, const auto& b ) -> localManifold2
        {
            using A = std::remove_cvref_t<decltype( a )>;
            using B = std::remove_cvref_t<decltype( b )>;

            if constexpr(
                std::is_same_v<A, std::monostate> ||
                std::is_same_v<B, std::monostate> )
            {
                return {};
            }
            else if constexpr(
                std::is_same_v<A, circle2> &&
                std::is_same_v<B, circle2> )
            {
                return CollideCircles( a, b, transformBToA );
            }
            else if constexpr(
                std::is_same_v<A, capsule2> &&
                std::is_same_v<B, circle2> )
            {
                return CollideCapsuleCircle( a, b, transformBToA );
            }
            else if constexpr(
                std::is_same_v<A, circle2> &&
                std::is_same_v<B, capsule2> )
            {
                return FlipManifoldToA(
                    CollideCapsuleCircle( b, a, transformAToB ),
                    transformA,
                    transformB
                );
            }
            else if constexpr(
                std::is_same_v<A, capsule2> &&
                std::is_same_v<B, capsule2> )
            {
                return CollideCapsules( a, b, transformBToA );
            }
            else if constexpr(
                std::is_same_v<A, polygon2> &&
                std::is_same_v<B, circle2> )
            {
                return CollidePolygonCircle( a, b, transformBToA );
            }
            else if constexpr(
                std::is_same_v<A, circle2> &&
                std::is_same_v<B, polygon2> )
            {
                return FlipManifoldToA(
                    CollidePolygonCircle( b, a, transformAToB ),
                    transformA,
                    transformB
                );
            }
            else if constexpr(
                std::is_same_v<A, polygon2> &&
                std::is_same_v<B, capsule2> )
            {
                return CollidePolygonCapsule( a, b, transformBToA );
            }
            else if constexpr(
                std::is_same_v<A, capsule2> &&
                std::is_same_v<B, polygon2> )
            {
                return FlipManifoldToA(
                    CollidePolygonCapsule( b, a, transformAToB ),
                    transformA,
                    transformB
                );
            }
            else if constexpr(
                std::is_same_v<A, polygon2> &&
                std::is_same_v<B, polygon2> )
            {
                return CollidePolygons( a, b, transformBToA );
            }
            else if constexpr(
                std::is_same_v<A, segment2> &&
                std::is_same_v<B, circle2> )
            {
                return CollideSegmentCircle( a, b, transformBToA );
            }
            else if constexpr(
                std::is_same_v<A, circle2> &&
                std::is_same_v<B, segment2> )
            {
                return FlipManifoldToA(
                    CollideSegmentCircle( b, a, transformAToB ),
                    transformA,
                    transformB
                );
            }
            else if constexpr(
                std::is_same_v<A, segment2> &&
                std::is_same_v<B, capsule2> )
            {
                return CollideSegmentCapsule( a, b, transformBToA );
            }
            else if constexpr(
                std::is_same_v<A, capsule2> &&
                std::is_same_v<B, segment2> )
            {
                return FlipManifoldToA(
                    CollideSegmentCapsule( b, a, transformAToB ),
                    transformA,
                    transformB
                );
            }
            else if constexpr(
                std::is_same_v<A, polygon2> &&
                std::is_same_v<B, segment2> )
            {
                return CollidePolygonSegment( a, b, transformBToA );
            }
            else if constexpr(
                std::is_same_v<A, segment2> &&
                std::is_same_v<B, polygon2> )
            {
                return FlipManifoldToA(
                    CollidePolygonSegment( b, a, transformAToB ),
                    transformA,
                    transformB
                );
            }
            else
            {
                // segment-segment는 Box2D와 동일하게 contact를 만들지 않음.
                return {};
            }
        },
        geometryA,
        geometryB
    );
}

} // namespace zonai
