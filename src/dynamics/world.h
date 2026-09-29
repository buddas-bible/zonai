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
#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"
#include "dynamics/contactData.h"
#include "dynamics/id.h"

namespace zonai
{

template <typename Callback>
concept WorldCollisionCallback =
    requires(
        Callback& callback,
        const ContactData& contactData )
    {
        { callback( contactData ) } -> std::same_as<void>;
    };

class World
{
public:
    // 새 Body를 만들고 slot index + generation으로 구성된 외부 handle을 반환함.
    [[nodiscard]] BodyId CreateBody(
        BodyType type = BodyType::Static,
        transform2 transform = {} );

    // handle이 가리키는 Body와 연결된 Contact / Shape / proxy를 모두 정리함.
    void DestroyBody( BodyId bodyId );

    // local geometry를 Body에 연결하고 Shape handle을 반환함.
    [[nodiscard]] ShapeId CreateShape(
        BodyId bodyId,
        ShapeGeometry geometry,
        Filter filter = {},
        float density = 1.0f );

    // handle이 가리키는 Shape의 Contact / proxy / Body list 연결을 정리함.
    void DestroyShape( ShapeId shapeId );

    // Shape density를 변경하고 owning Dynamic Body의 mass data를 다시 계산함.
    void SetShapeDensity( ShapeId shapeId, float density );
    [[nodiscard]] float GetShapeDensity( ShapeId shapeId ) const;

    // Body transform을 변경하고 연결된 모든 Shape proxy의 world AABB를 함께 갱신함.
    void SetBodyTransform(
        BodyId bodyId,
        transform2 transform );

    // Body의 simulation storage에 보관된 현재 world transform을 반환함.
    [[nodiscard]] transform2 GetBodyTransform( BodyId bodyId ) const;

    // 정적 Body는 움직이지 않으므로 setter를 무시함.
    void SetBodyLinearVelocity(
        BodyId bodyId,
        vec2 linearVelocity );

    [[nodiscard]] vec2 GetBodyLinearVelocity( BodyId bodyId ) const;

    // 정적 Body는 움직이지 않으므로 setter를 무시함.
    void SetBodyAngularVelocity(
        BodyId bodyId,
        float angularVelocity );

    [[nodiscard]] float GetBodyAngularVelocity( BodyId bodyId ) const;

    [[nodiscard]] float GetBodyMass( BodyId bodyId ) const;
    [[nodiscard]] float GetBodyRotationalInertia( BodyId bodyId ) const;
    [[nodiscard]] vec2 GetBodyLocalCenter( BodyId bodyId ) const;

    // World 전체 Dynamic Body에 적용되는 중력 가속도.
    void SetGravity( vec2 gravity );
    [[nodiscard]] vec2 GetGravity() const noexcept;

    // Dynamic Body에 world-space 힘을 누적함.
    // point가 center of mass에서 벗어나 있으면 torque도 함께 누적됨.
    void ApplyForce(
        BodyId bodyId,
        vec2 force,
        vec2 point );

    // center of mass에 힘을 가해 회전 없이 선가속도만 만듦.
    void ApplyForceToCenter(
        BodyId bodyId,
        vec2 force );

    // Dynamic Body에 z축 torque를 누적함.
    void ApplyTorque(
        BodyId bodyId,
        float torque );

    // 아직 Step에서 소비되지 않은 누적 force / torque를 직접 제거함.
    void ClearForces( BodyId bodyId );

    // force / gravity로 velocity를 갱신한 뒤 non-static Body를 적분하고
    // proxy / Contact 상태를 갱신함. constraint solver는 아직 포함하지 않음.
    void Step( float timeStep );

    // null / 범위 / generation / 활성 slot을 모두 확인함.
    [[nodiscard]] bool IsValid( BodyId bodyId ) const noexcept;
    [[nodiscard]] bool IsValid( ShapeId shapeId ) const noexcept;
    [[nodiscard]] bool IsValid( ContactId contactId ) const noexcept;

    // 기존 Contact를 갱신하고 BroadPhase의 새 AABB pair는 persistent Contact로 생성함.
    // callback은 현재 실제 접촉점이 존재하는 Contact만 받음.
    template <WorldCollisionCallback Callback>
    void UpdateCollisions( Callback&& callback )
    {
        // 기존 Contact는 BroadPhase에서 다시 후보로 나오지 않으므로 stable slot을 직접 갱신함.
        for( std::int32_t contactId = 0;
             contactId < static_cast<std::int32_t>( contacts_.size() );
             ++contactId )
        {
            contact2& contact = contacts_[contactId];

            // free-list에 들어간 slot은 현재 활성 Contact가 아님.
            if( contact.contactId == contact2::NULL_INDEX )
            {
                continue;
            }

            assert( contact.contactId == contactId );
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
                DestroyContact( contactId );
                continue;
            }

            assert( shapeA.bodyId >= 0 );
            assert( shapeB.bodyId >= 0 );
            assert( static_cast<std::size_t>( shapeA.bodyId ) < bodies_.size() );
            assert( static_cast<std::size_t>( shapeB.bodyId ) < bodies_.size() );

            const BodySim& bodySimA = bodySims_[shapeA.bodyId];
            const BodySim& bodySimB = bodySims_[shapeB.bodyId];

            assert( bodySimA.bodyId == shapeA.bodyId );
            assert( bodySimB.bodyId == shapeB.bodyId );

            contact.manifold =
                CollideShapes(
                    shapeA.geometry,
                    bodySimA.transform,
                    shapeB.geometry,
                    bodySimB.transform
                );

            if( contact.manifold.pointCount > 0 )
            {
                callback( MakeContactData( contactId ) );
            }

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
                if( !CanCollideShapes( shapeA.geometry, shapeB.geometry ) )
                {
                    return;
                }

                const BodySim& bodySimA = bodySims_[shapeA.bodyId];
                const BodySim& bodySimB = bodySims_[shapeB.bodyId];

                assert( bodySimA.bodyId == shapeA.bodyId );
                assert( bodySimB.bodyId == shapeB.bodyId );

                const localManifold2 manifold =
                    CollideShapes(
                        shapeA.geometry,
                        bodySimA.transform,
                        shapeB.geometry,
                        bodySimB.transform
                    );

                const std::int32_t contactId =
                    CreateContact( shapeIdA, shapeIdB, manifold );

                const contact2& contact = contacts_[contactId];

                if( contact.manifold.pointCount > 0 )
                {
                    callback( MakeContactData( contactId ) );
                }
            }
        );
    }

    [[nodiscard]] const Body& GetBody( BodyId bodyId ) const;
    [[nodiscard]] const Shape& GetShape( ShapeId shapeId ) const;

    // 내부 Contact를 public snapshot으로 변환해 반환함.
    [[nodiscard]] ContactData GetContactData( ContactId contactId ) const;

    // Body에 연결된 Contact 전체 개수. 실제 touching Contact 수보다 클 수 있음.
    [[nodiscard]] std::size_t GetBodyContactCapacity( BodyId bodyId ) const;

    // Body에 연결된 touching Contact만 output에 채우고 실제 작성 개수를 반환함.
    [[nodiscard]] std::size_t GetBodyContactData(
        BodyId bodyId,
        std::span<ContactData> output ) const;

    // Shape가 속한 Body의 Contact 개수이므로 보수적인 capacity임.
    [[nodiscard]] std::size_t GetShapeContactCapacity( ShapeId shapeId ) const;

    // 이 Shape가 실제로 참여한 touching Contact만 output에 채움.
    [[nodiscard]] std::size_t GetShapeContactData(
        ShapeId shapeId,
        std::span<ContactData> output ) const;

    [[nodiscard]] const BroadPhase& GetBroadPhase() const noexcept
    {
        return broadPhase_;
    }

    [[nodiscard]] std::size_t GetBodyCount() const noexcept
    {
        return bodyCount_;
    }

    [[nodiscard]] std::size_t GetShapeCount() const noexcept
    {
        return shapeCount_;
    }

    [[nodiscard]] std::size_t GetContactCount() const noexcept
    {
        return contactCount_;
    }

private:
    [[nodiscard]] std::int32_t GetBodyIndex( BodyId bodyId ) const;
    [[nodiscard]] std::int32_t GetShapeIndex( ShapeId shapeId ) const;
    [[nodiscard]] std::int32_t GetContactIndex( ContactId contactId ) const;

    [[nodiscard]] BodyId MakeBodyId( std::int32_t bodyIndex ) const;
    [[nodiscard]] ShapeId MakeShapeId( std::int32_t shapeIndex ) const;
    [[nodiscard]] ContactId MakeContactId( std::int32_t contactIndex ) const;
    [[nodiscard]] ContactData MakeContactData( std::int32_t contactIndex ) const;

    void DestroyBodyByIndex( std::int32_t bodyIndex );
    void DestroyShapeByIndex( std::int32_t shapeIndex );

    // BodySim transform을 기준으로 이 Body의 모든 Shape proxy를 BroadPhase에 동기화함.
    void SyncBodyProxies( std::int32_t bodyIndex );

    // 연결된 Shape들의 density / geometry를 합산해 Dynamic Body의 mass data를 갱신함.
    void UpdateBodyMassData( std::int32_t bodyIndex );

    // stable Contact slot을 할당하고 두 Body의 intrusive contact list에 연결함.
    [[nodiscard]] std::int32_t CreateContact(
        std::int32_t shapeIdA,
        std::int32_t shapeIdB,
        const localManifold2& manifold );

    // Body contact list / pairSet에서 해제한 뒤 slot을 free-list로 반환함.
    void DestroyContact( std::int32_t contactId );

    // bodyId가 변하지 않는 stable slot storage.
    std::vector<Body> bodies_;

    // solver set 도입 전까지 Body와 같은 stable slot index로 보관하는 simulation 데이터.
    std::vector<BodySim> bodySims_;

    // solver set 도입 전까지 Body와 같은 stable slot index로 보관하는 운동 상태.
    std::vector<BodyState> bodyStates_;

    // 제거된 Body slot 재사용을 위한 free-list head.
    std::int32_t bodyFreeList_ = Body::NULL_INDEX;

    // bodies_.size()와 별개인 현재 활성 Body 개수.
    std::size_t bodyCount_ = 0;

    // shapeId가 변하지 않는 stable slot storage.
    std::vector<Shape> shapes_;

    // 제거된 Shape slot 재사용을 위한 free-list head.
    std::int32_t shapeFreeList_ = Shape::NULL_INDEX;

    // shapes_.size()와 별개인 현재 활성 Shape 개수.
    std::size_t shapeCount_ = 0;

    // contactId가 변하지 않는 stable slot storage.
    std::vector<contact2> contacts_;

    // 제거된 Contact slot을 재사용하기 위한 free-list head.
    std::int32_t contactFreeList_ = contact2::NULL_INDEX;

    // contacts_.size()와 별개인 현재 활성 Contact 개수.
    std::size_t contactCount_ = 0;

    // 모든 Dynamic Body에 적용되는 world-space 중력 가속도.
    vec2 gravity_{ 0.0f, -10.0f };

    // 모든 Shape의 broad-phase proxy를 body type별 DynamicTree에 관리함.
    BroadPhase broadPhase_;
};

} // namespace zonai
