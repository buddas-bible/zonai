#pragma once

#include <cassert>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "collision/broadphase/broadPhase.h"
#include "collision/narrowphase/collide.h"
#include "collision/narrowphase/contact2.h"
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

    // 기존 Contact를 갱신하고 BroadPhase의 새 AABB pair는 persistent Contact로 생성함.
    // callback은 현재 실제 접촉점이 존재하는 Contact만 받음.
    template <WorldCollisionCallback Callback>
    void UpdateCollisions( Callback&& callback )
    {
        // 기존 Contact는 BroadPhase에서 다시 후보로 나오지 않으므로 직접 갱신함.
        std::size_t contactIndex = 0;

        while( contactIndex < contacts_.size() )
        {
            contact2& contact = contacts_[contactIndex];

            assert( contact.shapeIdA >= 0 );
            assert( contact.shapeIdB >= 0 );
            assert( static_cast<std::size_t>( contact.shapeIdA ) < shapes_.size() );
            assert( static_cast<std::size_t>( contact.shapeIdB ) < shapes_.size() );

            const Shape& shapeA = shapes_[contact.shapeIdA];
            const Shape& shapeB = shapes_[contact.shapeIdB];

            assert( shapeA.proxyKey != Shape::NULL_INDEX );
            assert( shapeB.proxyKey != Shape::NULL_INDEX );

            const aabb2& aabbA =
                broadPhase_
                    .GetTree( GetProxyType( shapeA.proxyKey ) )
                    .GetProxyAABB( GetProxyId( shapeA.proxyKey ) );

            const aabb2& aabbB =
                broadPhase_
                    .GetTree( GetProxyType( shapeB.proxyKey ) )
                    .GetProxyAABB( GetProxyId( shapeB.proxyKey ) );

            // AABB pair 자체가 끝났으면 Contact와 pairSet을 함께 제거함.
            if( !Overlaps( aabbA, aabbB ) )
            {
                const ShapePairKey pairKey =
                    MakeShapePairKey( contact.shapeIdA, contact.shapeIdB );

                const bool removed = broadPhase_.RemovePair( pairKey );
                assert( removed );

                contacts_[contactIndex] = contacts_.back();
                contacts_.pop_back();
                continue;
            }

            assert( shapeA.bodyId >= 0 );
            assert( shapeB.bodyId >= 0 );
            assert( static_cast<std::size_t>( shapeA.bodyId ) < bodies_.size() );
            assert( static_cast<std::size_t>( shapeB.bodyId ) < bodies_.size() );

            const Body& bodyA = bodies_[shapeA.bodyId];
            const Body& bodyB = bodies_[shapeB.bodyId];

            contact.manifold =
                CollideShapes(
                    shapeA.geometry,
                    bodyA.transform,
                    shapeB.geometry,
                    bodyB.transform
                );

            if( contact.manifold.pointCount > 0 )
            {
                callback(
                    contact.shapeIdA,
                    contact.shapeIdB,
                    contact.manifold
                );
            }

            ++contactIndex;
        }

        // 기존 pair는 pairSet이 걸러주므로 여기에는 새 AABB pair만 들어옴.
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
                assert( CanCollideShapes( shapeA.geometry, shapeB.geometry ) );

                const Body& bodyA = bodies_[shapeA.bodyId];
                const Body& bodyB = bodies_[shapeB.bodyId];

                contact2 contact{};
                contact.shapeIdA = shapeIdA;
                contact.shapeIdB = shapeIdB;
                contact.manifold =
                    CollideShapes(
                        shapeA.geometry,
                        bodyA.transform,
                        shapeB.geometry,
                        bodyB.transform
                    );

                const ShapePairKey pairKey =
                    MakeShapePairKey( shapeIdA, shapeIdB );

                // HashSet::Add는 새 key면 false, 이미 있으면 true를 반환함.
                const bool alreadyExists = broadPhase_.AddPair( pairKey );
                assert( !alreadyExists );

                contacts_.push_back( contact );

                if( contact.manifold.pointCount > 0 )
                {
                    callback( shapeIdA, shapeIdB, contact.manifold );
                }
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

    [[nodiscard]] std::size_t GetContactCount() const noexcept
    {
        return contacts_.size();
    }

    [[nodiscard]] const contact2& GetContact( std::size_t contactIndex ) const
    {
        assert( contactIndex < contacts_.size() );
        return contacts_[contactIndex];
    }

private:
    // 현재 단계에서는 Body를 연속 배열로 보관함.
    // DestroyBody / id 재사용은 추후 Body id pool을 추가할 때 구현함.
    std::vector<Body> bodies_;

    // Shape도 World가 연속 storage로 소유하고 bodyId / shapeId로 서로 연결함.
    std::vector<Shape> shapes_;

    // BroadPhase AABB pair가 유지되는 동안 persistent Contact를 보관함.
    std::vector<contact2> contacts_;

    // 모든 Shape의 broad-phase proxy를 body type별 DynamicTree에 관리함.
    BroadPhase broadPhase_;
};

} // namespace zonai
