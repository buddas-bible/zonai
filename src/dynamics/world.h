#pragma once

#include <cassert>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "collision/broadphase/broadPhase.h"
#include "collision/constants.h"
#include "collision/narrowphase/collide.h"
#include "collision/narrowphase/contact2.h"
#include "collision/shape.h"
#include "dynamics/body.h"
#include "dynamics/bodyDef.h"
#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"
#include "dynamics/contactConstraint2.h"
#include "dynamics/contactData.h"
#include "dynamics/contactSim2.h"
#include "dynamics/constants.h"
#include "dynamics/id.h"
#include "dynamics/island2.h"
#include "dynamics/sensor2.h"

namespace zonai
{

template <typename Callback>
concept worldCollisionCallback =
    requires(
        Callback& callback,
        const contactData& contactData )
    {
        { callback( contactData ) } -> std::same_as<void>;
    };

class world
{
public:
    // definition의 초기 simulation 설정으로 새 body를 만듦.
    [[nodiscard]] bodyId CreateBody( const bodyDef& definition );

    // 기존 간단한 생성 경로. 내부에서 bodyDef를 만들어 같은 생성 로직을 사용함.
    [[nodiscard]] bodyId CreateBody( bodyType type = bodyType::Static, transform2 transform = {} );

    // handle이 가리키는 body와 연결된 Contact / shape / proxy를 모두 정리함.
    void DestroyBody( bodyId bodyId );


    // local geometry를 body에 연결하고 shape handle을 반환함.
    [[nodiscard]] shapeId CreateShape( bodyId bodyId, shapeGeometry geometry, collisionFilter filter = {}, float density = 1.0f );

    // Contact를 만들지 않고 overlap만 추적하는 sensor shape를 생성함.
    [[nodiscard]] shapeId CreateSensorShape(
        bodyId bodyId,
        shapeGeometry geometry,
        collisionFilter filter = {},
        float density = 0.0f );

    // handle이 가리키는 shape의 Contact / proxy / body list 연결을 정리함.
    void DestroyShape( shapeId shapeId );


    // shape density를 변경하고 owning Dynamic body의 mass data를 다시 계산함.
    void SetShapeDensity( shapeId shapeId, float density );
    [[nodiscard]] float GetShapeDensity( shapeId shapeId ) const;

    void SetShapeFriction( shapeId shapeId, float friction );
    [[nodiscard]] float GetShapeFriction( shapeId shapeId ) const;

    void SetShapeRestitution( shapeId shapeId, float restitution );
    [[nodiscard]] float GetShapeRestitution( shapeId shapeId ) const;

    // runtime filter 변경은 기존 Contact를 즉시 제거하고
    // 다음 collision update에서 이 shape의 broad-phase pair를 다시 탐색함.
    void SetShapeFilter( shapeId shapeId, collisionFilter filter );
    [[nodiscard]] collisionFilter GetShapeFilter( shapeId shapeId ) const;

    [[nodiscard]] bool IsShapeSensor( shapeId shapeId ) const;

    // Sensor event는 sensor/visitor 양쪽 shape가 모두 켜져 있어야 생성됨.
    // 변경 결과는 다음 Step의 sensor update부터 반영됨.
    void SetShapeSensorEventsEnabled( shapeId shapeId, bool enabled );
    [[nodiscard]] bool AreShapeSensorEventsEnabled( shapeId shapeId ) const;

    // sensor가 현재 추적 중인 visitor 개수. sensor가 아니면 0.
    [[nodiscard]] std::size_t GetShapeSensorCapacity( shapeId shapeId ) const;

    // sensor overlap handle을 output에 채우고 실제 작성 개수를 반환함.
    [[nodiscard]] std::size_t GetShapeSensorData(
        shapeId sensorShapeId,
        std::span<shapeId> output ) const;

    // 가장 최근 Step 끝에서 생성된 transient sensor begin / end events.
    [[nodiscard]] std::span<const sensorBeginEvent2> GetSensorBeginEvents() const noexcept
    {
        return sensorBeginEvents_;
    }

    [[nodiscard]] std::span<const sensorEndEvent2> GetSensorEndEvents() const noexcept
    {
        return sensorEndEvents_;
    }

    // 현재 shape의 speculative AABB와 broad-phase fat AABB를 반환함.
    [[nodiscard]] const aabb2& GetShapeAABB( shapeId shapeId ) const;
    [[nodiscard]] const aabb2& GetShapeFatAABB( shapeId shapeId ) const;

    // body transform을 변경하고 연결된 모든 shape proxy의 world AABB를 함께 갱신함.
    void SetBodyTransform( bodyId bodyId, transform2 transform );

    // body의 simulation storage에 보관된 현재 world transform을 반환함.
    [[nodiscard]] transform2 GetBodyTransform( bodyId bodyId ) const;


    void SetBodyLinearVelocity( bodyId bodyId, vec2 linearVelocity );
    [[nodiscard]] vec2 GetBodyLinearVelocity( bodyId bodyId ) const;

    void SetBodyAngularVelocity( bodyId bodyId, float angularVelocity );
    [[nodiscard]] float GetBodyAngularVelocity( bodyId bodyId ) const;

    void SetBodyLinearDamping( bodyId bodyId, float damping );
    [[nodiscard]] float GetBodyLinearDamping( bodyId bodyId ) const;

    void SetBodyAngularDamping( bodyId bodyId, float damping );
    [[nodiscard]] float GetBodyAngularDamping( bodyId bodyId ) const;

    void SetBodyGravityScale( bodyId bodyId, float scale );
    [[nodiscard]] float GetBodyGravityScale( bodyId bodyId ) const;

    void SetBodyFastRotationAllowed( bodyId bodyId, bool allowed );
    [[nodiscard]] bool IsBodyFastRotationAllowed( bodyId bodyId ) const;


    void SetBodyAwake( bodyId bodyId, bool awake );
    [[nodiscard]] bool IsBodyAwake( bodyId bodyId ) const;

    void SetBodySleepEnabled( bodyId bodyId, bool enabled );
    [[nodiscard]] bool IsBodySleepEnabled( bodyId bodyId ) const;

    void SetBodySleepThreshold( bodyId bodyId, float threshold );
    [[nodiscard]] float GetBodySleepThreshold( bodyId bodyId ) const;

    void SetBodySafetyFactor( bodyId bodyId, float safetyFactor );
    [[nodiscard]] float GetBodySafetyFactor( bodyId bodyId ) const;

    void SetBodyBullet( bodyId bodyId, bool bullet );
    [[nodiscard]] bool IsBodyBullet( bodyId bodyId ) const;

    // 이 설정을 바꿔도 이미 만들어진 Contact의 recycling 허용 여부는 바뀌지 않음.
    void SetBodyContactRecyclingEnabled( bodyId bodyId, bool enabled );
    [[nodiscard]] bool IsBodyContactRecyclingEnabled( bodyId bodyId ) const;

    // 마지막 Step에서 continuous collision 후보로 분류됐는지 반환함.
    [[nodiscard]] bool IsBodyFast( bodyId bodyId ) const;

    // 마지막 Step에서 실제 TOI로 이동이 잘렸는지 반환함.
    [[nodiscard]] bool HadBodyTimeOfImpact( bodyId bodyId ) const;


    [[nodiscard]] float GetBodyMass( bodyId bodyId ) const;
    [[nodiscard]] float GetBodyRotationalInertia( bodyId bodyId ) const;
    [[nodiscard]] vec2 GetBodyLocalCenter( bodyId bodyId ) const;


    // world 전체 Dynamic body에 적용되는 중력 가속도.
    void SetGravity( vec2 gravity );
    [[nodiscard]] vec2 GetGravity() const noexcept;

    // position integration 전에 적용하는 world 최대 선속도.
    void SetMaximumLinearSpeed( float speed );
    [[nodiscard]] float GetMaximumLinearSpeed() const noexcept;

    void SetSleepingEnabled( bool enabled );
    [[nodiscard]] bool IsSleepingEnabled() const noexcept;

    void SetContinuousEnabled( bool enabled ) noexcept;
    [[nodiscard]] bool IsContinuousEnabled() const noexcept;

    void SetContactRecycleDistance( float distance );
    [[nodiscard]] float GetContactRecycleDistance() const noexcept;

    // 가장 최근 UpdateCollisions에서 narrowphase를 건너뛰고 재활용한 Contact 수.
    [[nodiscard]] std::size_t GetRecycledContactCount() const noexcept
    {
        return recycledContactCount_;
    }


    // Dynamic body에 world-space 힘을 누적함.
    // point가 center of mass에서 벗어나 있으면 torque도 함께 누적됨.
    void ApplyForce( bodyId bodyId, vec2 force, vec2 point );

    // center of mass에 힘을 가해 회전 없이 선가속도만 만듦.
    void ApplyForceToCenter( bodyId bodyId, vec2 force );

    // Dynamic body에 z축 torque를 누적함.
    void ApplyTorque( bodyId bodyId, float torque );

    // 아직 Step에서 소비되지 않은 누적 force / torque를 직접 제거함.
    void ClearForces( bodyId bodyId );

    // Dynamic body에 world-space linear impulse를 즉시 적용함.
    // point가 center of mass에서 벗어나 있으면 angular velocity도 함께 바뀜.
    void ApplyLinearImpulse( bodyId bodyId, vec2 impulse, vec2 point );

    // center of mass에 linear impulse를 적용해 선속도만 즉시 변경함.
    void ApplyLinearImpulseToCenter( bodyId bodyId, vec2 impulse );

    // Dynamic body의 angular velocity를 즉시 변경하는 z축 angular impulse.
    void ApplyAngularImpulse( bodyId bodyId, float impulse );


    // Contact constraint를 준비한 뒤 sub-step마다 force / gravity, solve, position integration을 수행함.
    void Step( float timeStep, int subStepCount = 1 );

    // null / 범위 / generation / 활성 slot을 모두 확인함.
    [[nodiscard]] bool IsValid( bodyId bodyId ) const noexcept;
    [[nodiscard]] bool IsValid( shapeId shapeId ) const noexcept;
    [[nodiscard]] bool IsValid( contactId contactId ) const noexcept;

    // 기존 Contact를 갱신하고 broadPhase의 새 AABB pair는 persistent Contact로 생성함.
    // callback은 현재 실제 접촉점이 존재하는 Contact만 받음.
    template <worldCollisionCallback Callback>
    void UpdateCollisions( Callback&& callback )
    {
        recycledContactCount_ = 0;

        // 기존 Contact는 broadPhase에서 다시 후보로 나오지 않으므로 stable slot을 직접 갱신함.
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

            const shape& shapeA = shapes_[contact.shapeIdA];
            const shape& shapeB = shapes_[contact.shapeIdB];

            assert( shapeA.proxyKey != shape::NULL_INDEX );
            assert( shapeB.proxyKey != shape::NULL_INDEX );

            assert( fatAABBs_.size() == shapes_.size() );

            const aabb2& fatAABBA =
                fatAABBs_[contact.shapeIdA];

            const aabb2& fatAABBB =
                fatAABBs_[contact.shapeIdB];

            // persistent broad-phase bounds가 완전히 분리됐을 때 Contact를 제거함.
            if( !Overlaps( fatAABBA, fatAABBB ) )
            {
                DestroyContact( contactId );
                continue;
            }

            assert( shapeA.bodyId >= 0 );
            assert( shapeB.bodyId >= 0 );
            assert( static_cast<std::size_t>( shapeA.bodyId ) < bodies_.size() );
            assert( static_cast<std::size_t>( shapeB.bodyId ) < bodies_.size() );

            const bodySim& bodySimA = bodySims_[shapeA.bodyId];
            const bodySim& bodySimB = bodySims_[shapeB.bodyId];

            assert( bodySimA.bodyId == shapeA.bodyId );
            assert( bodySimB.bodyId == shapeB.bodyId );

            if( TryRecycleContact( contactId ) )
            {
                const contactSim2& contactSim =
                    contactSims_[contactId];

                if( IsTouchingManifold( contactSim.manifold ) )
                {
                    callback( MakeContactData( contactId ) );
                }

                continue;
            }

            const localManifold2 manifold =
                CollideShapes(
                    shapeA.geometry,
                    bodySimA.transform,
                    shapeB.geometry,
                    bodySimB.transform
                );

            UpdateContactSim(
                contactId,
                manifold
            );

            const contactSim2& contactSim =
                contactSims_[contactId];

            if( IsTouchingManifold( contactSim.manifold ) )
            {
                callback( MakeContactData( contactId ) );
            }

        }

        // 기존 pair는 pairSet이 걸러주므로 여기에는 새 AABB pair만 들어옴.
        broadPhase_.UpdatePairs(
            std::span<const shape>{ shapes_.data(), shapes_.size() },
            [this, &callback](
                std::int32_t shapeIdA,
                std::int32_t shapeIdB )
            {
                assert( shapeIdA >= 0 );
                assert( shapeIdB >= 0 );
                assert( static_cast<std::size_t>( shapeIdA ) < shapes_.size() );
                assert( static_cast<std::size_t>( shapeIdB ) < shapes_.size() );

                const shape& shapeA = shapes_[shapeIdA];
                const shape& shapeB = shapes_[shapeIdB];

                assert( shapeA.bodyId >= 0 );
                assert( shapeB.bodyId >= 0 );
                assert( static_cast<std::size_t>( shapeA.bodyId ) < bodies_.size() );
                assert( static_cast<std::size_t>( shapeB.bodyId ) < bodies_.size() );

                // 같은 body에 연결된 shape끼리는 self collision을 만들지 않음.
                if( shapeA.bodyId == shapeB.bodyId )
                {
                    return;
                }

                if( !CanCollideShapes( shapeA.geometry, shapeB.geometry ) )
                {
                    return;
                }

                const bodySim& bodySimA = bodySims_[shapeA.bodyId];
                const bodySim& bodySimB = bodySims_[shapeB.bodyId];

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

                const contactSim2& contactSim =
                    contactSims_[contactId];

                if( IsTouchingManifold( contactSim.manifold ) )
                {
                    callback( MakeContactData( contactId ) );
                }
            }
        );
    }

    [[nodiscard]] const body& GetBody( bodyId bodyId ) const;
    [[nodiscard]] const shape& GetShape( shapeId shapeId ) const;

    // 내부 Contact를 public snapshot으로 변환해 반환함.
    [[nodiscard]] contactData GetContactData( contactId contactId ) const;

    // body에 연결된 Contact 전체 개수. 실제 touching Contact 수보다 클 수 있음.
    [[nodiscard]] std::size_t GetBodyContactCapacity( bodyId bodyId ) const;

    // body에 연결된 touching Contact만 output에 채우고 실제 작성 개수를 반환함.
    [[nodiscard]] std::size_t GetBodyContactData( bodyId bodyId, std::span<contactData> output ) const;

    // shape가 속한 body의 Contact 개수이므로 보수적인 capacity임.
    [[nodiscard]] std::size_t GetShapeContactCapacity( shapeId shapeId ) const;

    // 이 shape가 실제로 참여한 touching Contact만 output에 채움.
    [[nodiscard]] std::size_t GetShapeContactData( shapeId shapeId, std::span<contactData> output ) const;

    [[nodiscard]] const broadPhase& GetBroadPhase() const noexcept
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
    [[nodiscard]] std::int32_t GetBodyIndex( bodyId bodyId ) const;
    [[nodiscard]] std::int32_t GetShapeIndex( shapeId shapeId ) const;
    [[nodiscard]] std::int32_t GetContactIndex( contactId contactId ) const;

    [[nodiscard]] bodyId MakeBodyId( std::int32_t bodyIndex ) const;
    [[nodiscard]] shapeId MakeShapeId( std::int32_t shapeIndex ) const;
    [[nodiscard]] contactId MakeContactId( std::int32_t contactIndex ) const;
    [[nodiscard]] contactData MakeContactData( std::int32_t contactIndex ) const;

    void DestroyBodyByIndex( std::int32_t bodyIndex );
    void DestroyShapeByIndex( std::int32_t shapeIndex );

    // bodySim transform을 기준으로 이 body의 모든 shape proxy를 broadPhase에 동기화함.
    void SyncBodyProxies( std::int32_t bodyIndex );

    // 아직 commit되지 않은 transform을 기준으로 speculative / fat bounds와 tree proxy를 갱신함.
    void UpdateBodyProxyBounds(
        std::int32_t bodyIndex,
        const transform2& transform );

    // 연결된 shape들의 density / geometry를 합산해 Dynamic body의 mass data를 갱신함.
    void UpdateBodyMassData( std::int32_t bodyIndex );

    // 모든 sensor가 세 broad-phase tree를 query해 overlap과 begin/end event를 갱신함.
    void UpdateSensors();

    // sensor shape 파괴 시 dense sensor storage를 정리하고 기존 overlap end event를 예약함.
    void DestroySensorByShapeIndex( std::int32_t shapeIndex );

    // 현재 solver-active Contact graph를 따라 연결된 body 전체를 깨움.
    // Static body가 시작점이면 연결된 non-static body만 깨움.
    void WakeBodyByIndex( std::int32_t bodyIndex );

    // 명시적으로 body 하나를 sleep시키면 현재 연결된 island 전체를 함께 sleep시킴.
    void SleepBodyByIndex( std::int32_t bodyIndex );

    // active Contact가 awake / sleeping 경계를 가로지르지 않도록 wake 상태를 전파함.
    void WakeSleepingBodiesFromContacts();

    // solver가 끝난 island의 motion이 threshold 아래에 충분히 오래 머물렀는지 판정함.
    void UpdateIslandSleepStates(
        const islandGraph2& islandGraph,
        float timeStep );

    // fast body의 swept path를 검사하고 가장 이른 TOI에서 delta transform을 잘라냄.
    void SolveContinuousBody(
        std::int32_t bodyIndex,
        float timeStep );

    // stable Contact slot을 할당하고 두 body의 intrusive contact list에 연결함.
    [[nodiscard]] std::int32_t CreateContact(
        std::int32_t shapeIdA, std::int32_t shapeIdB,
        const localManifold2& manifold );

    // 상대 transform 변화가 충분히 작으면 cached local anchor로 manifold를 갱신함.
    [[nodiscard]] bool TryRecycleContact(
        std::int32_t contactId );

    // 현재 bodySim / narrow-phase manifold를 solver용 ContactSim에 동기화하고
    // 다음 recycling에 사용할 anchor / transform cache를 다시 저장함.
    void UpdateContactSim(
        std::int32_t contactId,
        const localManifold2& manifold );

    // 한 island의 solver-active Contact를 이번 step의 transient constraint로 변환함.
    [[nodiscard]] std::vector<contactConstraint2> PrepareContactConstraints(
        std::span<const std::int32_t> contactIds,
        float timeStep );

    // 이전 step의 cached impulse를 body velocity에 먼저 적용함.
    void WarmStartContacts(
        std::span<contactConstraint2> constraints );

    // normal constraint를 반복해서 풂.
    // useBias=true는 penetration push, false는 적분 후 velocity relaxation임.
    void SolveContactConstraints(
        std::span<contactConstraint2> constraints,
        bool useBias );

    // 충돌 전 접근 속도가 충분히 빠른 Contact에 restitution impulse를 적용함.
    void ApplyRestitutionContacts(
        std::span<contactConstraint2> constraints );

    // 최종 누적 impulse를 persistent ContactSim으로 되돌림.
    void StoreContactConstraintImpulses(
        std::span<const contactConstraint2> constraints );

    // body contact list / pairSet에서 해제한 뒤 slot을 free-list로 반환함.
    void DestroyContact( std::int32_t contactId );

    /// body
    // bodyId가 변하지 않는 stable slot storage.
    std::vector<body> bodies_;

    // solver set 도입 전까지 body와 같은 stable slot index로 보관하는 simulation 데이터.
    std::vector<bodySim> bodySims_;

    // solver set 도입 전까지 body와 같은 stable slot index로 보관하는 운동 상태.
    std::vector<bodyState> bodyStates_;

    // 제거된 body slot 재사용을 위한 free-list head.
    std::int32_t bodyFreeList_ = body::NULL_INDEX;

    // bodies_.size()와 별개인 현재 활성 body 개수.
    std::size_t bodyCount_ = 0;


    /// shape
    // shapeId가 변하지 않는 stable slot storage.
    std::vector<shape> shapes_;

    // shape와 같은 stable slot index로 보관하는 broad-phase fat AABB.
    // tree proxy bounds와 동일한 값을 유지하되 Contact lifetime에서도 직접 사용함.
    std::vector<aabb2> fatAABBs_;

    // Sensor는 Contact와 독립된 dense storage에서 overlap을 추적함.
    std::vector<sensor2> sensors_;
    std::vector<sensorBeginEvent2> sensorBeginEvents_;
    std::vector<sensorEndEvent2> sensorEndEvents_;

    // sensor 자체가 파괴될 때 생긴 end event는 다음 sensor update까지 보존함.
    std::vector<sensorEndEvent2> pendingSensorEndEvents_;

    // 제거된 shape slot 재사용을 위한 free-list head.
    std::int32_t shapeFreeList_ = shape::NULL_INDEX;

    // shapes_.size()와 별개인 현재 활성 shape 개수.
    std::size_t shapeCount_ = 0;


    /// contact
    // contactId가 변하지 않는 stable slot storage.
    // 수명 / shape 연결 / body intrusive list 같은 cold data를 보관함.
    std::vector<contact2> contacts_;

    // solver set 도입 전까지 Contact와 같은 stable slot index로 보관하는 hot data.
    std::vector<contactSim2> contactSims_;

    // 제거된 Contact slot을 재사용하기 위한 free-list head.
    std::int32_t contactFreeList_ = contact2::NULL_INDEX;

    // contacts_.size()와 별개인 현재 활성 Contact 개수.
    std::size_t contactCount_ = 0;

    // 모든 Dynamic body에 적용되는 world-space 중력 가속도.
    vec2 gravity_{ 0.0f, -10.0f };

    // 지나치게 큰 이동으로 solver가 불안정해지는 것을 막는 world 최대 선속도.
    float maximumLinearSpeed_ = DEFAULT_MAX_LINEAR_SPEED;

    // false면 모든 non-static body를 계속 awake 상태로 유지함.
    bool sleepingEnabled_ = true;

    // false면 fast body를 분류만 하고 TOI pass는 실행하지 않음.
    bool continuousEnabled_ = true;

    // 작은 상대 이동에서 persistent manifold를 재활용할 최대 거리.
    float contactRecycleDistance_ = CONTACT_RECYCLE_DISTANCE;

    // 가장 최근 collision update에서 재활용된 Contact 수.
    std::size_t recycledContactCount_ = 0;

    // 모든 shape의 broad-phase proxy를 body type별 DynamicTree에 관리함.
    broadPhase broadPhase_;
};

} // namespace zonai
