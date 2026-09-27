#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "collision/broadphase/broadPhase.h"
#include "collision/narrowphase/collide.h"
#include "collision/shape.h"
#include "dynamics/body.h"

namespace zonai
{

template <typename Callback>
concept WorldCollisionCallback =
    requires(
        Callback& callback,
        std::int32_t shapeIdA,
        std::int32_t shapeIdB,
        const localManifold2& manifold )
    {
        { callback( shapeIdA, shapeIdB, manifold ) } -> std::same_as<void>;
    };

class World
{
public:
    // Body를 World storage 끝에 생성하고 해당 Body index를 반환함.
    [[nodiscard]] std::int32_t CreateBody(
        BodyType type = BodyType::Static,
        transform2 transform = {} );

    // local geometry를 Body에 연결하고 BroadPhase proxy까지 생성함.
    [[nodiscard]] std::int32_t CreateShape(
        std::int32_t bodyId,
        ShapeGeometry geometry,
        Filter filter = {} );

    // Body transform을 변경하고 연결된 모든 Shape proxy의 world AABB를 함께 갱신함.
    void SetBodyTransform(
        std::int32_t bodyId,
        transform2 transform );

    // BroadPhase의 새 후보를 NarrowPhase까지 검사해 실제 접촉 중인 pair만 callback으로 전달함.
    // manifold는 shapeIdA가 속한 Body의 local space 기준임.
    template <WorldCollisionCallback Callback>
    void UpdateCollisions( Callback&& callback )
    {
        broadPhase_.UpdatePairs(
            std::span<const Shape>{ shapes_.data(), shapes_.size() },
            [this, &callback](
                std::int32_t shapeIdA,
                std::int32_t shapeIdB )
            {
                assert( shapeIdA >= 0 );
                assert( shapeIdB >= 0 );
                assert( static_cast<std::size_t>( shapeIdA ) < shapes_.size() );
                assert( static_cast<std::size_t>( shapeIdB ) < shapes_.size() );

                const Shape& shapeA = shapes_[shapeIdA];
                const Shape& shapeB = shapes_[shapeIdB];

                assert( shapeA.bodyId >= 0 );
                assert( shapeB.bodyId >= 0 );
                assert( static_cast<std::size_t>( shapeA.bodyId ) < bodies_.size() );
                assert( static_cast<std::size_t>( shapeB.bodyId ) < bodies_.size() );

                if( !CanCollideShapes( shapeA.geometry, shapeB.geometry ) )
                {
                    return;
                }

                const Body& bodyA = bodies_[shapeA.bodyId];
                const Body& bodyB = bodies_[shapeB.bodyId];

                const localManifold2 manifold =
                    CollideShapes(
                        shapeA.geometry,
                        bodyA.transform,
                        shapeB.geometry,
                        bodyB.transform
                    );

                if( manifold.pointCount == 0 )
                {
                    return;
                }

                callback( shapeIdA, shapeIdB, manifold );
            }
        );
    }

    [[nodiscard]] const Body& GetBody( std::int32_t bodyId ) const;
    [[nodiscard]] const Shape& GetShape( std::int32_t shapeId ) const;

    [[nodiscard]] const BroadPhase& GetBroadPhase() const noexcept
    {
        return broadPhase_;
    }

    [[nodiscard]] std::size_t GetBodyCount() const noexcept
    {
        return bodies_.size();
    }

    [[nodiscard]] std::size_t GetShapeCount() const noexcept
    {
        return shapes_.size();
    }

private:
    // 현재 단계에서는 Body를 연속 배열로 보관함.
    // DestroyBody / id 재사용은 추후 Body id pool을 추가할 때 구현함.
    std::vector<Body> bodies_;

    // Shape도 World가 연속 storage로 소유하고 bodyId / shapeId로 서로 연결함.
    std::vector<Shape> shapes_;

    // 모든 Shape의 broad-phase proxy를 body type별 DynamicTree에 관리함.
    BroadPhase broadPhase_;
};

} // namespace zonai
