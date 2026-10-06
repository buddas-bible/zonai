#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>
#include <variant>

#include "collision/broadphase/broadPhase.h"
#include "collision/constants.h"
#include "collision/shape.h"
#include "dynamics/body.h"
#include "dynamics/bodyDef.h"
#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"
#include "dynamics/contact2.h"
#include "dynamics/contactConstraint2.h"
#include "dynamics/contactData.h"
#include "dynamics/contactSim2.h"
#include "dynamics/constants.h"
#include "dynamics/distanceJoint2.h"
#include "dynamics/distanceJointConstraint2.h"
#include "dynamics/joint2.h"
#include "dynamics/mouseJoint2.h"
#include "dynamics/mouseJointConstraint2.h"
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
    // handle 입력은 IsValid가 true인 값이어야 함. 삭제/slot 재사용 후에는 다시 확인함.
#pragma region WorldLifetime

    world();

    // world identity를 복제하지 않도록 값 복사/이동을 금지함.
    world( const world& ) = delete;
    world& operator=( const world& ) = delete;

    world( world&& ) = delete;
    world& operator=( world&& ) = delete;

#pragma endregion

#pragma region BodyLifecycle

    // definition의 초기 simulation 설정으로 새 body를 만듦.
    [[nodiscard]] bodyId CreateBody( const bodyDef& definition );

    // 기존 간단한 생성 경로. 내부에서 bodyDef를 만들어 같은 생성 로직을 사용함.
    [[nodiscard]] bodyId CreateBody( bodyType type = bodyType::Static, transform2 transform = {} );

    // handle이 가리키는 body와 연결된 Joint / Contact / shape / proxy를 모두 정리함.
    void DestroyBody( bodyId bodyId );


#pragma endregion

#pragma region ShapeLifecycle

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


#pragma endregion

#pragma region JointLifecycle

    // 유효한 서로 다른 Body 두 개 중 하나 이상이 Dynamic이어야 함.
    // Local anchor는 Body origin 기준, length는 양수이며 LINEAR_SLOP 이상으로 제한함.
    [[nodiscard]] jointId createDistanceJoint( const distanceJointDef& definition );
    // 같은 설정은 유지하고, mode/계수 변경은 cached impulse를 비우고 non-static component를 깨움.
    void setDistanceJointSpring( jointId id, bool enableSpring, float hertz, float dampingRatio );
    // 유한한 비음수 min <= max를 stable minimum으로 제한하고 cache/wake를 갱신함.
    void setDistanceJointLimit( jointId id, bool enableLimit, float minLength, float maxLength );
    // Static A / Dynamic B. target은 world 좌표, hertz/damping/maxForce는 유한한 비음수임.
    [[nodiscard]] jointId createMouseJoint( const mouseJointDef& definition );
    void setMouseJointTarget( jointId id, vec2 target );
    void setMouseJointTuning( jointId id, float hertz, float dampingRatio, float maxForce );
    [[nodiscard]] mouseJointData getMouseJointData( jointId id ) const;
    void destroyJoint( jointId id );
    [[nodiscard]] distanceJointData getDistanceJointData( jointId id ) const;
    [[nodiscard]] std::size_t getJointCount() const noexcept { return jointCount_; }

#pragma endregion

#pragma region ShapeProperties

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
    // 공통 category 규칙 변경은 Contact를 무효화하고 정지 proxy도 다시 검사함.
    void setCollisionMatrix( const collisionMatrix& matrix );
    [[nodiscard]] const collisionMatrix& getCollisionMatrix() const noexcept { return collisionMatrix_; }

#pragma endregion

#pragma region SensorQueries

    [[nodiscard]] bool IsShapeSensor( shapeId shapeId ) const;

    // Sensor event는 sensor/visitor 양쪽 shape가 모두 켜져 있어야 생성됨.
    // 변경 결과는 다음 Step의 sensor update부터 반영됨.
    void SetShapeSensorEventsEnabled( shapeId shapeId, bool enabled );
    [[nodiscard]] bool AreShapeSensorEventsEnabled( shapeId shapeId ) const;

    // 마지막 Step에서 sensor가 추적한 visitor 개수. sensor가 아니면 0.
    [[nodiscard]] std::size_t GetShapeSensorCapacity( shapeId shapeId ) const;

    // 마지막 Step의 overlap handle을 output 크기까지만 채우고 실제 작성 개수를 반환함.
    // Box2D처럼 파괴된 visitor의 과거 handle도 포함할 수 있으므로 사용 전에 IsValid로 확인함.
    [[nodiscard]] std::size_t GetShapeSensorData(
        shapeId sensorShapeId,
        std::span<shapeId> output ) const;

    // 가장 최근 Step 끝에서 생성된 transient sensor begin / end events.
    // span은 다음 Step 또는 world 파괴 전까지 유효함. 보존하려면 event를 복사함.
    // Box2D처럼 sensor/visitor가 이미 파괴됐을 수 있으므로 handle은 IsValid로 확인함.
    [[nodiscard]] std::span<const sensorBeginEvent2> GetSensorBeginEvents() const noexcept
    {
        return sensorBeginEvents_;
    }

    [[nodiscard]] std::span<const sensorEndEvent2> GetSensorEndEvents() const noexcept
    {
        return sensorEndEvents_;
    }

#pragma endregion

#pragma region ShapeBounds

    // 현재 shape의 speculative AABB와 broad-phase fat AABB를 반환함.
    // 내부 storage 참조이므로 shape 생성/파괴 또는 world 파괴를 넘겨 보관하지 않음.
    // Point를 Body local space로 옮겨 실제 geometry에 검사함. Segment는 면적이 없어 false임.
    [[nodiscard]] bool testShapePoint( shapeId id, vec2 point ) const;
    [[nodiscard]] const aabb2& GetShapeAABB( shapeId shapeId ) const;
    [[nodiscard]] const aabb2& GetShapeFatAABB( shapeId shapeId ) const;

#pragma endregion

#pragma region BodyProperties

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


#pragma endregion

#pragma region WorldSettings

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


#pragma endregion

#pragma region ForcesAndImpulses

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


#pragma endregion

#pragma region SimulationStep

    // Contact / Joint constraint를 준비한 뒤 sub-step마다 force / gravity, solve, position integration을 수행함.
    void Step( float timeStep, int subStepCount = 1 );

#pragma endregion

#pragma region HandleValidity

    // world lifetime / null / 범위 / generation / 활성 slot을 모두 확인함.
    [[nodiscard]] bool IsValid( bodyId bodyId ) const noexcept;
    [[nodiscard]] bool IsValid( shapeId shapeId ) const noexcept;
    [[nodiscard]] bool IsValid( contactId contactId ) const noexcept;
    [[nodiscard]] bool IsValid( jointId id ) const noexcept;

#pragma endregion

#pragma region CollisionCallbacks

    // 기존 Contact를 갱신하고 broadPhase의 새 AABB pair는 persistent Contact로 생성함.
    // callback은 현재 실제 접촉점이 존재하는 Contact만 받음.
    // callback 중 world 변경이나 Step / UpdateCollisions 재진입은 허용하지 않음.
    template <worldCollisionCallback Callback>
    void UpdateCollisions( Callback&& callback )
    {
        recycledContactCount_ = 0;

        // 기존 Contact를 갱신한 직후 알림. 새 pair 생성 뒤로 callback을 미루지 않음.
        for( std::int32_t contactId = 0; contactId < static_cast<std::int32_t>( contacts_.size() ); ++contactId )
        {
            if( updateExistingContact( contactId ) )
            {
                callback( MakeContactData( contactId ) );
            }
        }

        // pairSet이 기존 pair를 걸러줌. callable을 복사하거나 임시 event buffer를 만들지 않음.
        broadPhase_.UpdatePairs( std::span<const shape>{ shapes_ }, [this, &callback]( std::int32_t shapeIdA, std::int32_t shapeIdB )
        {
            const std::int32_t contactId = createContactForPair( shapeIdA, shapeIdB );
            if( contactId != contact2::NULL_INDEX && IsTouchingManifold( contactSims_[contactId].manifold ) )
            {
                callback( MakeContactData( contactId ) );
            }
        } );
    }

#pragma endregion

#pragma region DiagnosticQueries

    // 내부 record를 읽는 진단용 API. body/shape 생성·파괴 또는 world 파괴 전까지만 참조함.
    // 변경 후에도 보존할 데이터는 복사하고, entity 식별에는 stable handle을 사용함.
    [[nodiscard]] const body& GetBody( bodyId bodyId ) const;
    [[nodiscard]] const shape& GetShape( shapeId shapeId ) const;

    // 내부 Contact를 public snapshot으로 변환해 반환함.
    [[nodiscard]] contactData GetContactData( contactId contactId ) const;

    // body에 연결된 Contact 전체 개수. 실제 touching Contact 수보다 클 수 있음.
    [[nodiscard]] std::size_t GetBodyContactCapacity( bodyId bodyId ) const;

    // Box2D처럼 pointCount > 0인 Contact를 speculative point까지 output에 채움.
    // output 크기까지만 작성하며 반환 개수 이후 원소는 변경하지 않음.
    [[nodiscard]] std::size_t GetBodyContactData( bodyId bodyId, std::span<contactData> output ) const;

    // shape가 속한 body의 Contact 개수이므로 보수적인 capacity임. sensor는 0.
    [[nodiscard]] std::size_t GetShapeContactCapacity( shapeId shapeId ) const;

    // 이 shape가 참여한 pointCount > 0 Contact를 speculative point까지 output에 채움.
    [[nodiscard]] std::size_t GetShapeContactData( shapeId shapeId, std::span<contactData> output ) const;

    // world가 소유하는 읽기 전용 진단 view. tree/proxy 참조를 world 변경 너머로 보관하지 않음.
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

#pragma endregion

private:
    // Public handle이 world 수명을 구분할 때 사용하는 opaque token.
    std::uint64_t worldToken_ = 0;

#pragma region InternalHandles

    [[nodiscard]] std::int32_t GetBodyIndex( bodyId bodyId ) const;
    [[nodiscard]] std::int32_t GetShapeIndex( shapeId shapeId ) const;
    [[nodiscard]] std::int32_t GetContactIndex( contactId contactId ) const;
    [[nodiscard]] std::int32_t getJointIndex( jointId id ) const;

    [[nodiscard]] bodyId MakeBodyId( std::int32_t bodyIndex ) const;
    [[nodiscard]] shapeId MakeShapeId( std::int32_t shapeIndex ) const;
    [[nodiscard]] contactId MakeContactId( std::int32_t contactIndex ) const;
    [[nodiscard]] contactData MakeContactData( std::int32_t contactIndex ) const;
    [[nodiscard]] jointId makeJointId( std::int32_t jointIndex ) const;

#pragma endregion

#pragma region JointStorage

    [[nodiscard]] std::int32_t allocateJoint( std::int32_t bodyIndexA, std::int32_t bodyIndexB, bool collideConnected );
    void destroyJointByIndex( std::int32_t jointIndex, bool touchProxies );
    void resetJointImpulses( std::int32_t bodyIndex );
    [[nodiscard]] bool shouldBodiesCollide( std::int32_t bodyIndexA, std::int32_t bodyIndexB ) const;
    [[nodiscard]] bool shouldShapeFiltersCollide( const collisionFilter& a, const collisionFilter& b ) const;

#pragma endregion

#pragma region BodyShapeStorage

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

#pragma endregion

#pragma region SensorUpdate

    // 모든 sensor가 세 broad-phase tree를 query해 overlap과 begin/end event를 갱신함.
    void UpdateSensors();

    // sensor shape 파괴 시 dense sensor storage를 정리하고 기존 overlap end event를 예약함.
    void DestroySensorByShapeIndex( std::int32_t shapeIndex );

#pragma endregion

#pragma region SleepAndWake

    // 현재 solver-active Contact / Joint graph를 따라 연결된 body 전체를 깨움.
    // Static body가 시작점이면 연결된 non-static body만 깨움.
    void WakeBodyByIndex( std::int32_t bodyIndex );

    // 명시적으로 body 하나를 sleep시키면 현재 연결된 island 전체를 함께 sleep시킴.
    void SleepBodyByIndex( std::int32_t bodyIndex );

    // Contact / Joint가 awake / sleeping 경계를 가로지르지 않도록 wake 상태를 전파함.
    void wakeSleepingBodiesFromConstraints();

    // solver가 끝난 island의 motion이 threshold 아래에 충분히 오래 머물렀는지 판정함.
    void UpdateIslandSleepStates(
        const islandGraph2& islandGraph,
        float timeStep );

#pragma endregion

#pragma region ContinuousCollision

    // fast body의 swept path를 검사하고 가장 이른 TOI에서 delta transform을 잘라냄.
    void SolveContinuousBody(
        std::int32_t bodyIndex,
        float timeStep );

#pragma endregion

#pragma region ContactUpdate

    // callback과 무관한 갱신은 cpp에서 처리하고 실제 접촉 여부만 template에 반환함.
    [[nodiscard]] bool updateExistingContact( std::int32_t contactId );
    [[nodiscard]] std::int32_t createContactForPair( std::int32_t shapeIdA, std::int32_t shapeIdB );

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

#pragma endregion

#pragma region JointSolver

    using jointConstraint = std::variant<distanceJointConstraint2, mouseJointConstraint2>;
    void warmStartJoints( std::span<jointConstraint> constraints );
    void solveJoints( std::span<jointConstraint> constraints, bool useBias );

#pragma endregion

#pragma region ContactSolver

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

#pragma endregion

#pragma region ContactDestruction

    // body contact list / pairSet에서 해제한 뒤 slot을 free-list로 반환함.
    void DestroyContact( std::int32_t contactId );

#pragma endregion

#pragma region Storage

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

    // Joint cold / hot 데이터는 동일한 stable slot과 free-list를 공유함.
    std::vector<joint2> joints_;
    std::vector<std::variant<distanceJointSim2, mouseJointSim2>> jointSims_;
    std::int32_t jointFreeList_ = -1;
    std::size_t jointCount_ = 0;

    // 모든 Dynamic body에 적용되는 world-space 중력 가속도.
    collisionMatrix collisionMatrix_{};
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
#pragma endregion
};

} // namespace zonai
