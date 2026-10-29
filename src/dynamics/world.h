#pragma once

#include <algorithm>
#include <concepts>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <vector>
#include <variant>
#include <utility>

#include "collision/broadphase/broadPhase.h"
#include "collision/constants.h"
#include "collision/distance2.h"
#include "collision/chainSegmentCastOwnership2.h"
#include "collision/rayCast2.h"
#include "collision/shapeCast2.h"
#include "collision/rotatingSweepBounds2.h"
#include "collision/timeOfImpact2.h"
#include "dynamics/body.h"
#include "dynamics/bodyDef.h"
#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"
#include "dynamics/chain2.h"
#include "dynamics/chainDef.h"
#include "dynamics/contact2.h"
#include "dynamics/contactConstraint2.h"
#include "dynamics/contactData.h"
#include "dynamics/contactSim2.h"
#include "dynamics/constants.h"
#include "dynamics/joints/distanceJoint2.h"
#include "dynamics/joints/distanceJointConstraint2.h"
#include "dynamics/joints/filterJoint2.h"
#include "dynamics/joints/filterJointConstraint2.h"
#include "dynamics/joints/joint2.h"
#include "dynamics/joints/motorJoint2.h"
#include "dynamics/joints/motorJointConstraint2.h"
#include "dynamics/joints/moverJoint2.h"
#include "dynamics/joints/moverJointConstraint2.h"
#include "dynamics/joints/pogoJoint2.h"
#include "dynamics/joints/pogoJointConstraint2.h"
#include "dynamics/joints/mouseJoint2.h"
#include "dynamics/joints/mouseJointConstraint2.h"
#include "dynamics/joints/prismaticJoint2.h"
#include "dynamics/joints/splineRailJoint2.h"
#include "dynamics/joints/prismaticJointConstraint2.h"
#include "dynamics/joints/splineRailJointConstraint2.h"
#include "dynamics/joints/revoluteJoint2.h"
#include "dynamics/joints/revoluteJointConstraint2.h"
#include "dynamics/joints/weldJoint2.h"
#include "dynamics/joints/weldJointConstraint2.h"
#include "dynamics/joints/wheelJoint2.h"
#include "dynamics/joints/wheelJointConstraint2.h"
#include "dynamics/id.h"
#include "dynamics/island2.h"
#include "dynamics/sensor2.h"
#include "dynamics/shape.h"

namespace zonai
{

// Callback: shapeId, world hit point, outward world normal, fraction of input translation.
template <typename Callback>
concept worldRayCastCallback = requires( Callback& callback, shapeId id, vec2 point, vec2 normal, float fraction ) {
    { callback( id, point, normal, fraction ) } -> std::convertible_to<float>;
};

struct worldRayCastResult2
{
    shapeId shape{};
    vec2 point{};
    vec2 normal{};
    float fraction = 0.0f;
    bool hit = false;
};

// B 도형을 현재 자세 그대로 world translation 방향으로 선형 sweep함.
struct worldShapeCastInput2
{
    shapeGeometry geometry{};
    transform2 transform{};
    vec2 translation{};
    bool canEncroach = false;
};

struct worldShapeCastResult2
{
    shapeId shape{};
    vec2 point{};
    vec2 normal{};
    float fraction = 0.0f;
    bool hit = false;
};

// 선형 이동과 제한된 각도 회전을 함께 보간하는 TOI 기반 World 조회.
// rotationRadians는 시작 자세에 적용되는 상대 회전량이며 [-pi/2, pi/2] 범위만 허용함.
struct worldShapeSweepInput2
{
    shapeGeometry geometry{};
    transform2 transform{};
    vec2 translation{};
    float rotationRadians = 0.0f;
};

// 선택적 호출 단위 World 회전 조회 계측. 비회전 선형 조회에는 수치를 기록하지 않음.
struct worldShapeSweepStats2
{
    int treeCandidates = 0;
    int narrowphaseCandidates = 0;
    int clippedCandidates = 0;
    int distancePrunedCandidates = 0;
    int toiCalls = 0;
    int hits = 0;
    toiDiagnostics2 toi{};
};

template <typename Callback>
concept worldOverlapCallback = requires( Callback& callback, shapeId id ) {
    { callback( id ) } -> std::convertible_to<bool>;
};

template <typename Callback>
concept worldCollisionCallback = requires( Callback& callback, const contactData& contactData ) {
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

#pragma endregion WorldLifetime

#pragma region BodyLifecycle

    // definition의 초기 simulation 설정으로 새 body를 만듦.
    [[nodiscard]] bodyId CreateBody( const bodyDef& definition );

    // 기존 간단한 생성 경로. 내부에서 bodyDef를 만들어 같은 생성 로직을 사용함.
    [[nodiscard]] bodyId CreateBody( bodyType type = bodyType::Static, transform2 transform = {} );

    // handle이 가리키는 body와 연결된 Joint / Contact / shape / proxy를 모두 정리함.
    void DestroyBody( bodyId bodyId );

#pragma endregion BodyLifecycle

#pragma region ShapeLifecycle

    // local geometry를 body에 연결하고 shape handle을 반환함.
    // capsule 축 길이가 LINEAR_SLOP 이하이면 빈 handle을 반환함 (sensor 생성도 동일).
    [[nodiscard]] shapeId CreateShape( bodyId bodyId, shapeGeometry geometry, collisionFilter filter = {}, float density = 1.0f );

    // Contact를 만들지 않고 overlap만 추적하는 sensor shape를 생성함.
    [[nodiscard]] shapeId CreateSensorShape( bodyId bodyId, shapeGeometry geometry, collisionFilter filter = {}, float density = 0.0f );

    // handle이 가리키는 shape의 Contact / proxy / body list 연결을 정리함.
    // parent chain의 child segment는 DestroyChain으로만 제거함.
    void DestroyShape( shapeId shapeId );

    // parent가 없는 chainSegment2의 geometry와 ghost vertex를 같은 shape handle로 교체함.
    // parent chain child는 adjacency를 parent가 소유하므로 개별 수정하지 않음.
    void SetChainSegment( shapeId shapeId, const chainSegment2& chainSegment );

#pragma endregion ShapeLifecycle

#pragma region ChainLifecycle

    // points를 여러 one-sided chainSegment2 shape로 확장해 하나의 parent chain으로 관리함.
    [[nodiscard]] chainId CreateChain( bodyId bodyId, const chainDef& definition );
    void DestroyChain( chainId chainId );

    // Parent chain의 point 하나를 stable child handle 안에서 바꾸고 인접 ghost adjacency도 함께 다시 연결함.
    void SetChainPoint( chainId chainId, std::size_t pointIndex, vec2 point );

    // 열린 chain을 검증 후 일괄 갱신함. 공통 앞쪽 child는 유지하고 새 child는 기존 끝 edge 설정을 이어받음.
    // 제거한 child handle은 무효화함. 검증 실패하면 geometry를 유지함.
    [[nodiscard]] bool SetChainPoints( chainId chainId, std::span<const vec2> points, vec2 ghost1, vec2 ghost2 );

    [[nodiscard]] std::size_t GetChainSegmentCount( chainId chainId ) const;
    [[nodiscard]] std::size_t GetChainSegments( chainId chainId, std::span<shapeId> output ) const;
    [[nodiscard]] chainId GetShapeParentChain( shapeId shapeId ) const;

#pragma endregion ChainLifecycle

#pragma region JointLifecycle

    // 유효한 서로 다른 Body 두 개 중 하나 이상이 Dynamic이어야 함.
    // Local anchor는 Body origin 기준, length는 양수이며 LINEAR_SLOP 이상으로 제한함.
    [[nodiscard]] jointId createDistanceJoint( const distanceJointDef& definition );
    void setDistanceJointLength( jointId id, float length );

    // 같은 설정은 유지하고, mode/계수 변경은 cached impulse를 비우고 non-static component를 깨움.
    void setDistanceJointSpring( jointId id, bool enableSpring, float hertz, float dampingRatio );

    // 유한한 비음수 min <= max를 stable minimum으로 제한하고 cache/wake를 갱신함.
    void setDistanceJointLimit( jointId id, bool enableLimit, float minLength, float maxLength );

    // 유한한 목표 축속도와 비음수 최대 힘. 변경 시 cache를 비우고 연결된 component를 깨움.
    void setDistanceJointMotor( jointId id, bool enableMotor, float motorSpeed, float maxMotorForce );

    // 두 Body 사이 collision만 선택적으로 차단하는 solver-less Joint. Joint graph / sleep 연결은 유지함.
    [[nodiscard]] jointId createFilterJoint( const filterJointDef& definition );
    [[nodiscard]] filterJointData getFilterJointData( jointId id ) const;

    // 두 Body의 작용점 상대 선속도와 상대 각속도를 직접 제어함. 하나 이상은 Dynamic이어야 함.
    [[nodiscard]] jointId createMotorJoint( const motorJointDef& definition );
    // 선형 목표속도 / 힘 한도 변경은 선형 cache만 비우고 연결된 non-static component를 깨움.
    void setMotorJointLinearVelocity( jointId id, vec2 linearVelocity, float maxVelocityForce );
    // 회전 목표속도 / 토크 한도 변경은 회전 cache만 비우고 연결된 non-static component를 깨움.
    void setMotorJointAngularVelocity( jointId id, float angularVelocity, float maxVelocityTorque );
    // 두 anchor를 복원하는 선형 spring 설정. 변경 시 선형 spring cache만 비우고 component를 깨움.
    void setMotorJointLinearSpring( jointId id, float hertz, float dampingRatio, float maxSpringForce );
    // 기준 상대각도와 회전 spring 설정. 변경 시 회전 spring cache만 비우고 component를 깨움.
    void setMotorJointAngularSpring( jointId id, float referenceAngle, float hertz, float dampingRatio, float maxSpringTorque );
    [[nodiscard]] motorJointData getMotorJointData( jointId id ) const;

    // 두 Body의 COM 상대 선속도만 제어하며 회전은 건드리지 않음. 하나 이상은 Dynamic이어야 함.
    [[nodiscard]] jointId createMoverJoint( const moverJointDef& definition );
    // 목표 상대 선속도 변경은 누적 impulse를 비우고 연결된 non-static component를 깨움.
    void setMoverJointLinearVelocity( jointId id, vec2 linearVelocity );
    // x/y 방향별 최대 힘 변경은 누적 impulse를 비우고 연결된 non-static component를 깨움.
    void setMoverJointMaxVelocityForce( jointId id, vec2 maxVelocityForce );
    [[nodiscard]] moverJointData getMoverJointData( jointId id ) const;

    // Pogo 축으로 spring 길이를 측정하고 contact normal 방향으로 반력을 가하는 character support Joint.
    [[nodiscard]] jointId createPogoJoint( const pogoJointDef& definition );
    // rest length / 주파수 / 감쇠 변경. 같은 값이면 깨우지 않음.
    void setPogoJointSpring( jointId id, float restLength, float hertz, float dampingRatio );
    // 인장/압축 최대 힘을 각각 설정함. 다음 Prepare에서 cached impulse도 새 한도로 제한함.
    void setPogoJointForceLimits( jointId id, float maxTensionForce, float maxCompressionForce );
    [[nodiscard]] pogoJointData getPogoJointData( jointId id ) const;

    // 두 작용점을 일치시키며 상대 회전은 허용함. 서로 다른 Body 중 하나 이상은 Dynamic이어야 함.
    [[nodiscard]] jointId createRevoluteJoint( const revoluteJointDef& definition );
    // 유한한 lower <= upper (rad). ±0.99*pi로 제한하며 변경 시 cache를 비우고 연결된 component를 깨움.
    void setRevoluteJointLimit( jointId id, bool enableLimit, float lowerAngle, float upperAngle );
    // B의 A에 대한 목표 각속도 (rad/s)와 최대 토크 (N*m). 속도 0은 제동함.
    void setRevoluteJointMotor( jointId id, bool enableMotor, float motorSpeed, float maxMotorTorque );
    [[nodiscard]] revoluteJointData getRevoluteJointData( jointId id ) const;

    // A의 단위 로컬 축을 따라 B의 이동만 허용하고 축 수직 이동과 상대 회전을 막음.
    [[nodiscard]] jointId createPrismaticJoint( const prismaticJointDef& definition );
    // 축 방향 목표 위치를 향한 실제 물리 스프링. 0 Hz는 스프링 힘을 끄며 변경 시 coupled cache와 wake를 갱신함.
    void setPrismaticJointSpring( jointId id, bool enableSpring, float hertz, float dampingRatio, float targetTranslation );
    // 유한한 lower <= upper 변위 (m). 변경 시 coupled cache를 비우고 연결된 component를 깨움.
    void setPrismaticJointLimit( jointId id, bool enableLimit, float lowerTranslation, float upperTranslation );
    // B의 A에 대한 목표 축속도 (m/s)와 비음수 최대 힘 (N). 속도 0은 제동함.
    void setPrismaticJointMotor( jointId id, bool enableMotor, float motorSpeed, float maxMotorForce );
    [[nodiscard]] prismaticJointData getPrismaticJointData( jointId id ) const;

    [[nodiscard]] jointId createSplineRailJoint( const splineRailJointDef& definition );
    void setSplineRailJointSpring( jointId id, bool enableSpring, float hertz, float dampingRatio, float targetTranslation );
    void setSplineRailJointLimit( jointId id, bool enableLimit, float lowerTranslation, float upperTranslation );
    void setSplineRailJointMotor( jointId id, bool enableMotor, float motorSpeed, float maxMotorForce );
    void setSplineRailJointRotation( jointId id, splineRailRotationMode mode, float angularHertz, float angularDampingRatio, bool enableAngularSpring = true );
    // 회전 구속만 켜고 끔. 모드·기준·튜닝을 유지하고 변경 시 캐시를 비우고 깨움.
    void setSplineRailJointRotationEnabled( jointId id, bool enabled );
    bool setSplineRailJointPoints( jointId id, std::span<const vec2> points );
    // 현재 이동량을 0으로 바꾸고 목표·리미트를 함께 옮김. 물리 상태·캐시·수면은 유지함.
    // 추적·float 범위 검증 실패나 리미트 폭 손실은 false이며 변경하지 않음.
    bool rebaseSplineRailJoint( jointId id );
    // 현재 자세를 선택한 회전 모드의 기준으로 삼음. 추적 실패 시 변경 없이 거절함.
    bool rebaseSplineRailJointRotation( jointId id );
    // Fixed는 상대 각도, 접선 추종은 접선 기준 각도(rad)를 지정함. 비유한 값은 변경 없이 거절함.
    bool setSplineRailJointRotationReference( jointId id, float angle );
    // 명시적으로 전체 곡선에 재투영함. 실패 시 기존 추적 상태를 유지함.
    bool reprojectSplineRailJoint( jointId id );
    // 지정 구간 안에서만 재투영 후보를 선택함. 이후에는 기존 연결 구간 추적을 사용함.
    bool reprojectSplineRailJoint( jointId id, std::size_t segmentIndex );
    // 앵커와 곡선 투영점의 거리가 유한한 비음수 maxDistance(m) 이내일 때만 재투영함.
    bool reprojectSplineRailJointWithinDistance( jointId id, float maxDistance );
    bool reprojectSplineRailJointWithinDistance( jointId id, std::size_t segmentIndex, float maxDistance );
    [[nodiscard]] splineRailJointData getSplineRailJointData( jointId id ) const;
    [[nodiscard]] std::span<const vec2> getSplineRailJointPoints( jointId id ) const;

    // 두 작용점과 기준 상대각도를 함께 유지해 두 Body의 상대 transform을 고정함.
    [[nodiscard]] jointId createWeldJoint( const weldJointDef& definition );
    // 0 Hz는 선형 hard Weld. 변경 시 선형 cache만 비우고 연결된 non-static component를 깨움.
    void setWeldJointLinearTuning( jointId id, float hertz, float dampingRatio );
    // 0 Hz는 회전 hard Weld. 변경 시 회전 cache만 비우고 연결된 non-static component를 깨움.
    void setWeldJointAngularTuning( jointId id, float hertz, float dampingRatio );
    [[nodiscard]] weldJointData getWeldJointData( jointId id ) const;

    // A의 단위 로컬 축을 따라 B를 이동시키며 상대 회전은 허용함. 하나 이상은 Dynamic이어야 함.
    [[nodiscard]] jointId createWheelJoint( const wheelJointDef& definition );
    // 유한한 비음수 Hz/감쇠. off/0 Hz는 축을 잠그지 않으며 변경 시 cache와 wake를 갱신함.
    void setWheelJointSpring( jointId id, bool enableSpring, float hertz, float dampingRatio );
    // 유한한 lower <= upper 변위 (m). 스프링 off/0 Hz와 무관하게 제한함.
    void setWheelJointLimit( jointId id, bool enableLimit, float lowerTranslation, float upperTranslation );
    // 유한한 상대 각속도와 비음수 토크. 변경 시 coupled cache와 wake를 갱신함.
    void setWheelJointMotor( jointId id, bool enableMotor, float motorSpeed, float maxMotorTorque );
    [[nodiscard]] wheelJointData getWheelJointData( jointId id ) const;

    // Static A / Dynamic B. target은 world 좌표, hertz/damping/maxForce는 유한한 비음수임.
    [[nodiscard]] jointId createMouseJoint( const mouseJointDef& definition );
    void setMouseJointTarget( jointId id, vec2 target );
    void setMouseJointTuning( jointId id, float hertz, float dampingRatio, float maxForce );
    [[nodiscard]] mouseJointData getMouseJointData( jointId id ) const;

    void destroyJoint( jointId id );
    [[nodiscard]] distanceJointData getDistanceJointData( jointId id ) const;

    [[nodiscard]] std::size_t getJointCount() const noexcept { return jointCount_; }

#pragma endregion JointLifecycle

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

#pragma endregion ShapeProperties

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
    [[nodiscard]] std::size_t GetShapeSensorData( shapeId sensorShapeId, std::span<shapeId> output ) const;

    // 가장 최근 Step 끝에서 생성된 transient sensor begin / end events.
    // span은 다음 Step 또는 world 파괴 전까지 유효함. 보존하려면 event를 복사함.
    // Box2D처럼 sensor/visitor가 이미 파괴됐을 수 있으므로 handle은 IsValid로 확인함.
    [[nodiscard]] std::span<const sensorBeginEvent2> GetSensorBeginEvents() const noexcept { return sensorBeginEvents_; }

    [[nodiscard]] std::span<const sensorEndEvent2> GetSensorEndEvents() const noexcept { return sensorEndEvents_; }

#pragma endregion SensorQueries

#pragma region SpatialQueries

    // World 좌표 AABB와 겹치는 활성 Shape 후보를 세 종류의 Tree에서 찾음.
    // 현재 speculative AABB로 fat proxy의 거짓 양성을 걸러내지만 정확한 geometry overlap은 아님.
    // Sensor도 포함함. 호출 순서는 보장하지 않으며 callback이 false면 나머지 Tree도 중단함.
    // Query 중 World/Tree/Shape를 수정하거나 Step으로 재진입하지 않음.
    template <worldOverlapCallback Callback> void overlapAABB( const aabb2& bounds, queryFilter filter, Callback&& callback ) const
    {
        if( !IsValidAABB( bounds ) ) return;

        for( int i = 0; i < static_cast<int>( bodyType::Count ); ++i )
        {
            const dynamicTree& tree = broadPhase_.GetTree( static_cast<bodyType>( i ) );
            bool keepQuerying = true;

            tree.Query( bounds,
                [&]( std::int32_t proxyId )
                {
                    const std::int32_t shapeIndex = tree.GetProxyShapeIndex( proxyId );
                    assert( shapeIndex >= 0 && static_cast<std::size_t>( shapeIndex ) < shapes_.size() );

                    const shape& candidate = shapes_[shapeIndex];

                    if( !Overlaps( bounds, candidate.aabb ) || !ShouldQueryShape( filter, candidate.filter ) ) return true;

                    keepQuerying = static_cast<bool>( callback( MakeShapeId( shapeIndex ) ) );
                    return keepQuerying;
                } );

            if( !keepQuerying ) return;
        }
    }

    // 기본 Query Filter는 category 1, mask 전체를 사용함.
    template <worldOverlapCallback Callback> void overlapAABB( const aabb2& bounds, Callback&& callback ) const
    {
        overlapAABB( bounds, queryFilter{}, std::forward<Callback>( callback ) );
    }

    // World 좌표의 convex geometry와 실제로 겹치는 Shape를 조회함.
    // AABB 후보를 GJK 거리와 radius로 다시 판정하므로 AABB만 겹친 false positive는 제외함.
    // callback false는 세 Tree 모두 즉시 중단. 순서 미보장, Sensor 포함, 조회 중 World 변경 금지.
    // ChainSegment는 owned edge를 양면 거리 proxy로 검사하며 ghost vertex/앞면 소유권은 적용하지 않음.
    template <worldOverlapCallback Callback>
    void overlapShape( const shapeGeometry& geometry, const transform2& transform, queryFilter filter, Callback&& callback ) const
    {
        if( std::holds_alternative<std::monostate>( geometry ) ||
            !IsFinite( transform.position ) ||
            !std::isfinite( transform.rotation.c ) || !std::isfinite( transform.rotation.s ) ) return;

        // MakeShapeProxy의 polygon 사전 조건은 외부 입력에 의한 assert가 일어나지 않도록 먼저 확인.
        if( const polygon2* polygon = std::get_if<polygon2>( &geometry ) )
        {
            if( polygon->vertexCount < 1 || polygon->vertexCount > static_cast<int>( MAX_POLYGON_VERTICES ) ) return;
        }

        const shapeProxy2 queryProxy = MakeShapeProxy( geometry );
        if( queryProxy.count < 1 || queryProxy.count > static_cast<int>( queryProxy.points.size() ) ||
            !std::isfinite( queryProxy.radius ) || queryProxy.radius < 0.0f ) return;

        for( int i = 0; i < queryProxy.count; ++i )
        {
            if( !IsFinite( queryProxy.points[i] ) ) return;
        }

        const aabb2 bounds = ComputeShapeAABB( geometry, transform );
        if( !IsValidAABB( bounds ) ) return;

        // Sensor의 discrete overlap과 동일한 float 오차 허용: tangent geometry의 GJK 잔여 거리.
        constexpr float OVERLAP_EPSILON = 10.0f * std::numeric_limits<float>::epsilon();

        for( int i = 0; i < static_cast<int>( bodyType::Count ); ++i )
        {
            const dynamicTree& tree = broadPhase_.GetTree( static_cast<bodyType>( i ) );
            bool keepQuerying = true;

            tree.Query( bounds,
                [&]( std::int32_t proxyId )
                {
                    const std::int32_t shapeIndex = tree.GetProxyShapeIndex( proxyId );
                    assert( shapeIndex >= 0 && static_cast<std::size_t>( shapeIndex ) < shapes_.size() );

                    const shape& candidate = shapes_[shapeIndex];
                    if( !ShouldQueryShape( filter, candidate.filter ) || !Overlaps( bounds, candidate.aabb ) ) return true;

                    const transform2& targetTransform = bodySims_[candidate.bodyId].transform;
                    distanceInput2 input{};
                    input.proxyA = MakeShapeProxy( candidate.geometry );
                    input.proxyB = queryProxy;
                    input.transform = InverseMul( targetTransform, transform );
                    input.useRadii = true;

                    simplexCache2 cache{};
                    const distanceOutput2 result = ShapeDistance( input, cache );
                    if( !std::isfinite( result.distance ) || result.distance > OVERLAP_EPSILON ) return true;

                    keepQuerying = static_cast<bool>( callback( MakeShapeId( shapeIndex ) ) );
                    return keepQuerying;
                } );

            if( !keepQuerying ) return;
        }
    }

    template <worldOverlapCallback Callback>
    void overlapShape( const shapeGeometry& geometry, const transform2& transform, Callback&& callback ) const
    {
        overlapShape( geometry, transform, queryFilter{}, std::forward<Callback>( callback ) );
    }

    // World 좌표 origin + translation * fraction 경로에서 실제 Shape geometry를 조회함.
    // callback: -1 무시, 0 중단, hit fraction으로 clip, 1 계속. 호출 순서는 거리순이 아님.
    // Sensor도 포함하며 시작점이 solid 내부면 fraction/normal 0으로 전달함. 조회 도중 World 변경 금지.
    template <worldRayCastCallback Callback> void castRay( vec2 origin, vec2 translation, queryFilter filter, Callback&& callback ) const
    {
        if( !IsFinite( origin ) || !IsFinite( translation ) || !( LengthSquared( translation ) > 0.0f ) ) return;

        float maxFraction = 1.0f;

        for( int i = 0; i < static_cast<int>( bodyType::Count ); ++i )
        {
            const dynamicTree& tree = broadPhase_.GetTree( static_cast<bodyType>( i ) );

            tree.RayCast( origin, translation, maxFraction,
                [&]( std::int32_t proxyId, float limit )
                {
                    const std::int32_t shapeIndex = tree.GetProxyShapeIndex( proxyId );
                    assert( shapeIndex >= 0 && static_cast<std::size_t>( shapeIndex ) < shapes_.size() );

                    const shape& candidate = shapes_[shapeIndex];
                    if( !ShouldQueryShape( filter, candidate.filter ) ) return -1.0f;

                    const transform2& transform = bodySims_[candidate.bodyId].transform;
                    const rayCastHit2 result = RayCastShapeGeometry( candidate.geometry, transform, origin, translation, limit );
                    if( !result.hit ) return -1.0f;

                    const float fraction = static_cast<float>( callback( MakeShapeId( shapeIndex ), result.point, result.normal, result.fraction ) );
                    return std::isfinite( fraction ) ? fraction : -1.0f;
                } );

            if( maxFraction == 0.0f ) return;
        }
    }

    template <worldRayCastCallback Callback> void castRay( vec2 origin, vec2 translation, Callback&& callback ) const
    {
        castRay( origin, translation, queryFilter{}, std::forward<Callback>( callback ) );
    }

    // 범용 콜백과 같은 필터를 사용하며 첫 번째가 아닌 가장 가까운 hit을 선택함.
    [[nodiscard]] worldRayCastResult2 castRayClosest( vec2 origin, vec2 translation, queryFilter filter = {} ) const
    {
        worldRayCastResult2 closest{};

        castRay( origin, translation, filter,
            [&]( shapeId id, vec2 point, vec2 normal, float fraction )
            {
                // Box2D의 closest 편의 조회는 최초 overlap을 무시함.
                if( fraction == 0.0f ) return -1.0f;

                if( !closest.hit || fraction < closest.fraction ) closest = { id, point, normal, fraction, true };
                return closest.fraction;
            } );

        return closest;
    }

    // 이동 도형의 sweep AABB로 기존 Tree를 조회한 뒤 GJK ShapeCast로 실제 충돌을 확인함.
    // callback은 -1 무시, 0 종료, (0, fraction]으로 clip, 1 계속. 방문 순서는 거리순이 아님.
    // Sensor를 포함하며 시작 겹침은 fraction=0을 보고함. Query 중 World 변경 금지.
    template <worldRayCastCallback Callback> void castShape( const worldShapeCastInput2& input, queryFilter filter, Callback&& callback ) const
    {
        if( std::holds_alternative<std::monostate>( input.geometry ) ||
            std::holds_alternative<chainSegment2>( input.geometry ) ||
            !IsFinite( input.transform.position ) ||
            !std::isfinite( input.transform.rotation.c ) || !std::isfinite( input.transform.rotation.s ) ||
            !IsFinite( input.translation ) || !( LengthSquared( input.translation ) > 0.0f ) ||
            !std::isfinite( LengthSquared( input.translation ) ) ) return;

        const shapeProxy2 movingProxy = MakeShapeProxy( input.geometry );
        if( movingProxy.count < 1 || movingProxy.count > static_cast<int>( movingProxy.points.size() ) ||
            !std::isfinite( movingProxy.radius ) || movingProxy.radius < 0.0f ) return;

        for( int i = 0; i < movingProxy.count; ++i )
        {
            if( !IsFinite( movingProxy.points[i] ) ) return;
        }

        const aabb2 startBounds = ComputeShapeAABB( input.geometry, input.transform );
        const aabb2 endBounds{ startBounds.min + input.translation, startBounds.max + input.translation };

        if( !IsValidAABB( startBounds ) || !IsValidAABB( endBounds ) ) return;

        float maxFraction = 1.0f;

        for( int i = 0; i < static_cast<int>( bodyType::Count ); ++i )
        {
            const dynamicTree& tree = broadPhase_.GetTree( static_cast<bodyType>( i ) );

            // 넓은 swept AABB Query 대신 Tree가 이동 방향과 현재 clip fraction을 반영하도록 함.
            tree.BoxCast( startBounds, input.translation, maxFraction,
                [&]( std::int32_t proxyId, float limit )
                {
                    const std::int32_t shapeIndex = tree.GetProxyShapeIndex( proxyId );
                    assert( shapeIndex >= 0 && static_cast<std::size_t>( shapeIndex ) < shapes_.size() );

                    const shape& candidate = shapes_[shapeIndex];
                    if( !ShouldQueryShape( filter, candidate.filter ) ) return -1.0f;

                    const aabb2 clippedEnd{ startBounds.min + input.translation * limit,
                        startBounds.max + input.translation * limit };
                    if( !Overlaps( Union( startBounds, clippedEnd ), candidate.aabb ) ) return -1.0f;

                    const transform2& targetTransform = bodySims_[candidate.bodyId].transform;

                    shapeCastInput2 cast{};
                    cast.proxyA = MakeShapeProxy( candidate.geometry );
                    cast.proxyB = movingProxy;
                    cast.transform = InverseMul( targetTransform, input.transform );
                    cast.translationB = InverseTransformVector( targetTransform, input.translation );
                    cast.maxFraction = limit;
                    cast.canEncroach = input.canEncroach;

                    const castOutput2 hit = ShapeCast( cast );
                    if( !hit.hit ) return -1.0f;

                    if( const chainSegment2* chain = std::get_if<chainSegment2>( &candidate.geometry ) )
                    {
                        // World 조회에서도 Chain 좁은 단계의 ghost/Voronoi 및 인접 법선
                        // ownership을 따름. 일반 Segment는 기존 앞면 검사만 유지함.
                        transform2 movingInChain = cast.transform;
                        movingInChain.position += hit.fraction * cast.translationB;
                        const bool hasPrevious =
                            candidate.parentChainId != shape::NULL_INDEX &&
                            ( chains_[candidate.parentChainId].isLoop ||
                              chains_[candidate.parentChainId].shapeIds.front() != shapeIndex );
                        if( !IsChainConvexCastOwned( *chain, input.geometry, movingInChain, hasPrevious ) )
                            return -1.0f;

                        // Chain은 edge의 오른쪽만 앞면. 겹침의 zero normal에는 중심의 초기 측면을 사용함.
                        const vec2 edge = chain->segment.b - chain->segment.a;
                        const vec2 front = Normalize( vec2{ edge.y, -edge.x } );

                        if( hit.fraction == 0.0f )
                        {
                            vec2 center{};

                            for( int point = 0; point < movingProxy.count; ++point )
                                center += TransformPoint( cast.transform, movingProxy.points[point] );

                            center /= static_cast<float>( movingProxy.count );

                            if( Dot( center - chain->segment.a, front ) <= 0.0f ) return -1.0f;
                        }
                        else if( Dot( hit.normal, front ) <= 0.0f ) return -1.0f;
                    }

                    const vec2 point = TransformPoint( targetTransform, hit.point );
                    const vec2 normal = TransformVector( targetTransform, hit.normal );

                    if( !IsFinite( point ) || !IsFinite( normal ) ) return -1.0f;

                    const float fraction = static_cast<float>( callback( MakeShapeId( shapeIndex ), point, normal, hit.fraction ) );
                    return std::isfinite( fraction ) ? fraction : -1.0f;
                } );

            if( maxFraction == 0.0f ) return;
        }
    }

    template <worldRayCastCallback Callback> void castShape( const worldShapeCastInput2& input, Callback&& callback ) const
    {
        castShape( input, queryFilter{}, std::forward<Callback>( callback ) );
    }

    // RayCastClosest와 같이 최초 겹침은 무시하고 가장 가까운 양수 이동 충돌만 반환함.
    [[nodiscard]] worldShapeCastResult2 castShapeClosest( const worldShapeCastInput2& input, queryFilter filter = {} ) const
    {
        worldShapeCastResult2 closest{};

        castShape( input, filter,
            [&]( shapeId id, vec2 point, vec2 normal, float fraction )
            {
                if( fraction == 0.0f ) return -1.0f;

                if( !closest.hit || fraction < closest.fraction ) closest = { id, point, normal, fraction, true };
                return closest.fraction;
            } );

        return closest;
    }

    // 기존 linear ShapeCast는 변경하지 않고 회전이 있는 sweep만 TOI를 사용함.
    // 회전 중의 도형을 전부 포함하는 origin-중심 보수적 AABB로 Tree를 조회함.
    // TOI는 swept separating axis 방식이므로 회전 도중의 일부 접촉을 놓칠 수 있음.
    template <worldRayCastCallback Callback>
    void castShapeSweep( const worldShapeSweepInput2& input, queryFilter filter, Callback&& callback,
        worldShapeSweepStats2* stats = nullptr ) const
    {
        if( stats ) *stats = {};
        if( std::holds_alternative<std::monostate>( input.geometry ) ||
            std::holds_alternative<chainSegment2>( input.geometry ) ||
            !IsFinite( input.transform.position ) || !IsFinite( input.translation ) ||
            !std::isfinite( input.transform.rotation.c ) || !std::isfinite( input.transform.rotation.s ) ||
            !std::isfinite( input.rotationRadians ) ||
            std::fabs( input.rotationRadians ) > 1.5707963267948966f ) return;

        if( const polygon2* polygon = std::get_if<polygon2>( &input.geometry ) )
        {
            if( polygon->vertexCount < 1 ||
                polygon->vertexCount > static_cast<int>( polygon->vertices.size() ) ) return;
        }

        const float rotationLengthSquared = input.transform.rotation.c * input.transform.rotation.c +
            input.transform.rotation.s * input.transform.rotation.s;
        if( !std::isfinite( rotationLengthSquared ) ||
            std::fabs( rotationLengthSquared - 1.0f ) > 0.001f ) return;

        const vec2 endPosition = input.transform.position + input.translation;
        if( !IsFinite( endPosition ) ) return;

        if( input.rotationRadians == 0.0f )
        {
            castShape( { input.geometry, input.transform, input.translation, false },
                filter, std::forward<Callback>( callback ) );
            return;
        }

        const shapeProxy2 movingProxy = MakeShapeProxy( input.geometry );
        if( movingProxy.count < 1 || movingProxy.count > static_cast<int>( movingProxy.points.size() ) ||
            !std::isfinite( movingProxy.radius ) || movingProxy.radius < 0.0f ) return;

        // 로컬 원점에서 가장 먼 support 점과 도형 radius를 이용하면 회전 도중의
        // 모든 자세에서 moving shape가 이 반경의 원 안에 들어감.
        double maxRadius = static_cast<double>( movingProxy.radius );
        double localMinX = std::numeric_limits<double>::infinity();
        double localMinY = localMinX;
        double localMaxX = -localMinX;
        double localMaxY = -localMinX;
        for( int i = 0; i < movingProxy.count; ++i )
        {
            const vec2 point = movingProxy.points[i];
            if( !IsFinite( point ) ) return;
            localMinX = std::min( localMinX, static_cast<double>( point.x ) );
            localMinY = std::min( localMinY, static_cast<double>( point.y ) );
            localMaxX = std::max( localMaxX, static_cast<double>( point.x ) );
            localMaxY = std::max( localMaxY, static_cast<double>( point.y ) );
            const double radius = std::hypot( static_cast<double>( point.x ),
                static_cast<double>( point.y ) ) + movingProxy.radius;
            maxRadius = std::max( maxRadius, radius );
        }
        if( !std::isfinite( maxRadius ) ||
            maxRadius >= static_cast<double>( std::numeric_limits<float>::max() ) ) return;

        // 부동소수점 경계 접촉이 후보 AABB에서 누락되지 않도록 바깥 방향으로 확장함.
        const float boundRadius = std::nextafter( static_cast<float>( maxRadius ),
            std::numeric_limits<float>::infinity() );
        const float negativeInfinity = -std::numeric_limits<float>::infinity();
        const float positiveInfinity = std::numeric_limits<float>::infinity();
        aabb2 sweptBounds{
            { std::nextafter( std::min( input.transform.position.x, endPosition.x ) - boundRadius, negativeInfinity ),
              std::nextafter( std::min( input.transform.position.y, endPosition.y ) - boundRadius, negativeInfinity ) },
            { std::nextafter( std::max( input.transform.position.x, endPosition.x ) + boundRadius, positiveInfinity ),
              std::nextafter( std::max( input.transform.position.y, endPosition.y ) + boundRadius, positiveInfinity ) }
        };
        if( !IsValidAABB( sweptBounds ) ) return;

        const rot2 endRotation = input.transform.rotation * rot2::FromRadians( input.rotationRadians );
        // Local 원점에서 떨어진 도형은 원호의 실제 축별 범위가 외접 원보다 좁을 수 있음.
        // 별도 계산 실패나 경계 값 불일치 시에는 기존 보수적 전체 원 범위를 유지함.
        aabb2 arcBounds{};
        // 로컬 정점 경계가 회전축을 가로지르는 도형은 원형 범위가 이미 충분히 작음.
        // 이 경우 추가 원호 계산은 생략하여 중심 정렬 Box 등의 짧은 조회 비용을 보존함.
        const bool pivotOutsideCoreBounds = localMinX > 0.0 || localMaxX < 0.0 ||
            localMinY > 0.0 || localMaxY < 0.0;
        if( pivotOutsideCoreBounds &&
            TryComputeRotatingSweepBounds( movingProxy, input.transform, endPosition,
                endRotation, input.rotationRadians, arcBounds ) )
        {
            // 두 보수적 경계의 교집합도 보수적임. arc의 float 여유 때문에
            // 원 경계를 조금 넘는 경우에도 더 좁은 부분만 사용할 수 있음.
            const aabb2 reduced{
                { std::max( sweptBounds.min.x, arcBounds.min.x ),
                  std::max( sweptBounds.min.y, arcBounds.min.y ) },
                { std::min( sweptBounds.max.x, arcBounds.max.x ),
                  std::min( sweptBounds.max.y, arcBounds.max.y ) }
            };
            if( IsValidAABB( reduced ) ) sweptBounds = reduced;
        }

        const sweep2 movingSweep{
            {}, input.transform.position, endPosition, input.transform.rotation, endRotation
        };

        float maxFraction = 1.0f;
        float clippedFraction = 1.0f;
        aabb2 clippedBounds = sweptBounds;

        for( int i = 0; i < static_cast<int>( bodyType::Count ); ++i )
        {
            const dynamicTree& tree = broadPhase_.GetTree( static_cast<bodyType>( i ) );

            // 회전 도형의 AABB는 직선으로 평행이동하지 않으므로 BoxCast가 아닌 Query를 사용함.
            tree.Query( sweptBounds, [&]( std::int32_t proxyId )
            {
                if( stats ) ++stats->treeCandidates;
                const std::int32_t shapeIndex = tree.GetProxyShapeIndex( proxyId );
                assert( shapeIndex >= 0 && static_cast<std::size_t>( shapeIndex ) < shapes_.size() );

                const shape& candidate = shapes_[shapeIndex];
                if( !ShouldQueryShape( filter, candidate.filter ) ||
                    !Overlaps( sweptBounds, candidate.aabb ) ) return true;

                // Tree.Query 자체는 기존 전체 bounds를 순회하지만, callback이 가장 가까운
                // hit으로 fraction을 줄였으면 이후 후보는 그 이전 구간에 들어와야 함.
                // 회전 정점은 원점 반경 안에 있으므로 짧아진 origin 구간만 검사해도 안전함.
                if( maxFraction < 1.0f )
                {
                    if( clippedFraction != maxFraction )
                    {
                        const double time = maxFraction;
                        const double inverseTime = 1.0 - time;
                        const double x = inverseTime * input.transform.position.x +
                            time * endPosition.x;
                        const double y = inverseTime * input.transform.position.y +
                            time * endPosition.y;

                        // GetSweepTransform은 float 곱셈/덧셈을 사용하므로 원점 좌표와
                        // 원점 반경에 따른 rounding margin을 주어 경계 접촉을 보존함.
                        constexpr double EPSILON = std::numeric_limits<float>::epsilon();
                        const double padX = 8.0 * EPSILON *
                            ( std::fabs( input.transform.position.x ) +
                              std::fabs( endPosition.x ) + maxRadius );
                        const double padY = 8.0 * EPSILON *
                            ( std::fabs( input.transform.position.y ) +
                              std::fabs( endPosition.y ) + maxRadius );
                        const double minX = std::min( static_cast<double>( input.transform.position.x ), x ) -
                            maxRadius - padX;
                        const double minY = std::min( static_cast<double>( input.transform.position.y ), y ) -
                            maxRadius - padY;
                        const double maxX = std::max( static_cast<double>( input.transform.position.x ), x ) +
                            maxRadius + padX;
                        const double maxY = std::max( static_cast<double>( input.transform.position.y ), y ) +
                            maxRadius + padY;

                        const aabb2 bounds{
                            { std::nextafter( static_cast<float>( minX ), negativeInfinity ),
                              std::nextafter( static_cast<float>( minY ), negativeInfinity ) },
                            { std::nextafter( static_cast<float>( maxX ), positiveInfinity ),
                              std::nextafter( static_cast<float>( maxY ), positiveInfinity ) }
                        };
                        // 유효하지 않은 극단 입력에서는 최적화를 생략하고 원래 조회를 유지.
                        clippedBounds = IsValidAABB( bounds ) ? bounds : sweptBounds;
                        clippedFraction = maxFraction;
                    }

                    if( !Overlaps( clippedBounds, candidate.aabb ) )
                    {
                        if( stats ) ++stats->clippedCandidates;
                        return true;
                    }
                }
                const transform2& targetTransform = bodySims_[candidate.bodyId].transform;
                toiInput2 toi{};
                toi.proxyA = MakeShapeProxy( candidate.geometry );

                // 원점 이동 경로와 타깃 원점 사이의 최소 거리로 충돌 불가능한
                // 도형을 TOI 전에 제외함. 각 proxy의 모든 support 점은
                // 로컬 원점을 중심으로 한 circumscribed disk 안에 들어감.
                // 회전 궤적이 복잡해도 이 두 원의 거리보다 가까워질 수 없음.
                double targetRadiusSquared = 0.0;
                for( int p = 0; p < toi.proxyA.count; ++p )
                {
                    const vec2 point = toi.proxyA.points[p];
                    const double x = point.x;
                    const double y = point.y;
                    targetRadiusSquared = std::max( targetRadiusSquared, x * x + y * y );
                }
                double targetRadius = std::sqrt( targetRadiusSquared ) + toi.proxyA.radius;

                const double startX = input.transform.position.x;
                const double startY = input.transform.position.y;
                const double targetX = targetTransform.position.x;
                const double targetY = targetTransform.position.y;
                const double dx = static_cast<double>( endPosition.x ) - startX;
                const double dy = static_cast<double>( endPosition.y ) - startY;
                const double lengthSquared = dx * dx + dy * dy;
                double nearestTime = 0.0;
                if( lengthSquared > 0.0 )
                    nearestTime = std::clamp(
                        ( ( targetX - startX ) * dx + ( targetY - startY ) * dy ) /
                        lengthSquared, 0.0, static_cast<double>( maxFraction ) );

                const double deltaX = targetX - ( startX + nearestTime * dx );
                const double deltaY = targetY - ( startY + nearestTime * dy );
                const double targetRotationLength = std::hypot(
                    static_cast<double>( targetTransform.rotation.c ),
                    static_cast<double>( targetTransform.rotation.s ) );
                // World의 local transform이 정확히 단위 회전이 아니어도
                // 보수적이어야 하므로 타깃 반경에 최대 회전 스케일을 반영함.
                targetRadius *= std::max( 1.0, targetRotationLength );
                constexpr double DISTANCE_EPSILON = std::numeric_limits<float>::epsilon();
                const double margin = 16.0 * DISTANCE_EPSILON *
                    ( std::fabs( startX ) + std::fabs( startY ) +
                      std::fabs( endPosition.x ) + std::fabs( endPosition.y ) +
                      std::fabs( targetX ) + std::fabs( targetY ) +
                      maxRadius + targetRadius + 1.0 );
                // TOI는 core 거리 target+tolerance까지 Hit으로 받아들이므로
                // 실제 도형 radius보다 큰 접촉 허용치를 반경에 더해야 함.
                // 그렇지 않으면 작은 Circle의 정상적인 TOI hit을 조기에 버릴 수 있음.
                const double combinedRadius =
                    static_cast<double>( movingProxy.radius ) + toi.proxyA.radius;
                const double contactDistance =
                    std::max( static_cast<double>( LINEAR_SLOP ),
                        combinedRadius - LINEAR_SLOP ) + 0.25 * LINEAR_SLOP;
                const double contactAllowance =
                    std::max( 0.0, contactDistance - combinedRadius );
                const double diskRadius = maxRadius + targetRadius +
                    contactAllowance + margin;
                if( std::isfinite( diskRadius ) && std::isfinite( deltaX ) &&
                    std::isfinite( deltaY ) &&
                    deltaX * deltaX + deltaY * deltaY > diskRadius * diskRadius )
                {
                    if( stats ) ++stats->distancePrunedCandidates;
                    return true;
                }

                if( stats ) ++stats->narrowphaseCandidates;
                toi.proxyB = movingProxy;
                toi.sweepA = { {}, targetTransform.position, targetTransform.position,
                    targetTransform.rotation, targetTransform.rotation };
                toi.sweepB = movingSweep;
                toi.maxFraction = maxFraction;
                toi.detectTransientContacts = true;

                // CCD의 one-sided 조기 종료보다 앞서 시작 시점의 앞면 겹침을 확인함.
                // 처음부터 맞닿아 멀어지는 회전 sweep도 기존 castShape처럼 fraction 0을 보고해야 함.
                toiOutput2 hit{};
                bool initialChainOverlap = false;
                if( const chainSegment2* chain = std::get_if<chainSegment2>( &candidate.geometry ) )
                {
                    const vec2 edge = chain->segment.b - chain->segment.a;
                    const vec2 front = Rotate( targetTransform.rotation, Normalize( vec2{ edge.y, -edge.x } ) );
                    vec2 centroid{};
                    for( int p = 0; p < movingProxy.count; ++p )
                        centroid += TransformPoint( input.transform, movingProxy.points[p] );
                    centroid /= static_cast<float>( movingProxy.count );

                    // World 공간 조회에서는 CCD의 작은 접근 속도 생략 조건을 적용하지 않음.
                    // 시작점의 앞면 여부와 최종 normal로 단면 충돌을 직접 제한함.
                    if( Dot( centroid - TransformPoint( targetTransform, chain->segment.a ), front ) <= 0.0f )
                        return true;
                    toi.proxyA.oneSided = false;

                    distanceInput2 distance{};
                    distance.proxyA = toi.proxyA;
                    distance.proxyB = movingProxy;
                    distance.transform = InverseMul( targetTransform, input.transform );
                    distance.useRadii = true;
                    simplexCache2 cache{};
                    const distanceOutput2 nearest = ShapeDistance( distance, cache );
                    constexpr float OVERLAP_EPSILON = 10.0f * std::numeric_limits<float>::epsilon();

                    if( std::isfinite( nearest.distance ) && nearest.distance <= OVERLAP_EPSILON )
                    {
                        initialChainOverlap = true;
                        hit.state = toiState2::Overlapped;
                        hit.fraction = 0.0f;
                        hit.point = TransformPoint( targetTransform, 0.5f * ( nearest.pointA + nearest.pointB ) );
                    }
                }

                if( !initialChainOverlap )
                {
                    if( stats )
                    {
                        ++stats->toiCalls;
                        toiDiagnostics2 detail{};
                        hit = TimeOfImpact( toi, &detail );
                        stats->toi.distanceCalls += detail.distanceCalls;
                        stats->toi.transientAttempts += detail.transientAttempts;
                        stats->toi.transientDistanceCalls += detail.transientDistanceCalls;
                        stats->toi.transientHits += detail.transientHits;
                        stats->toi.transientStepLimit += detail.transientStepLimit;
                    }
                    else hit = TimeOfImpact( toi );
                }
                if( hit.state != toiState2::Hit && hit.state != toiState2::Overlapped ) return true;
                if( !std::isfinite( hit.fraction ) ||
                    hit.fraction < 0.0f || hit.fraction >= maxFraction ) return true;

                vec2 point = hit.point;
                vec2 normal = hit.normal;

                if( hit.fraction == 0.0f )
                {
                    if( !initialChainOverlap )
                    {
                        distanceInput2 distance{};
                        distance.proxyA = toi.proxyA;
                        distance.proxyB = movingProxy;
                        distance.transform = InverseMul( targetTransform, input.transform );
                        distance.useRadii = true;
                        simplexCache2 cache{};
                        const distanceOutput2 nearest = ShapeDistance( distance, cache );
                        point = TransformPoint( targetTransform, 0.5f * ( nearest.pointA + nearest.pointB ) );
                    }
                    normal = {};
                }

                if( const chainSegment2* chain = std::get_if<chainSegment2>( &candidate.geometry ) )
                {
                    // World 회전 TOI도 실제 fraction의 Capsule/Polygon 자세를 사용해
                    // Chain narrowphase의 접합부 법선 소유권과 동일하게 판단함.
                    const transform2 movingAtHit = GetSweepTransform( movingSweep, hit.fraction );
                    const transform2 movingInChain = InverseMul( targetTransform, movingAtHit );
                    const bool hasPrevious =
                        candidate.parentChainId != shape::NULL_INDEX &&
                        ( chains_[candidate.parentChainId].isLoop ||
                          chains_[candidate.parentChainId].shapeIds.front() != shapeIndex );
                    if( !IsChainConvexCastOwned( *chain, input.geometry, movingInChain, hasPrevious ) )
                        return true;

                    const vec2 edge = chain->segment.b - chain->segment.a;
                    const vec2 front = Rotate( targetTransform.rotation, Normalize( vec2{ edge.y, -edge.x } ) );

                    if( hit.fraction == 0.0f )
                    {
                        vec2 centroid{};
                        for( int p = 0; p < movingProxy.count; ++p )
                            centroid += TransformPoint( input.transform, movingProxy.points[p] );
                        centroid /= static_cast<float>( movingProxy.count );
                        if( Dot( centroid - TransformPoint( targetTransform, chain->segment.a ), front ) <= 0.0f )
                            return true;
                    }
                    else if( Dot( normal, front ) <= 0.0f ) return true;
                }

                if( !IsFinite( point ) || !IsFinite( normal ) ) return true;
                if( stats ) ++stats->hits;
                const float fraction = static_cast<float>(
                    callback( MakeShapeId( shapeIndex ), point, normal, hit.fraction ) );

                if( fraction == 0.0f )
                {
                    maxFraction = 0.0f;
                    return false;
                }

                if( std::isfinite( fraction ) && fraction > 0.0f && fraction < maxFraction )
                    maxFraction = fraction;
                return true;
            } );

            if( maxFraction == 0.0f ) return;
        }
    }

    template <worldRayCastCallback Callback>
    void castShapeSweep( const worldShapeSweepInput2& input, Callback&& callback ) const
    {
        castShapeSweep( input, queryFilter{}, std::forward<Callback>( callback ) );
    }

    [[nodiscard]] worldShapeCastResult2 castShapeSweepClosest(
        const worldShapeSweepInput2& input, queryFilter filter = {},
        worldShapeSweepStats2* stats = nullptr ) const
    {
        worldShapeCastResult2 closest{};
        castShapeSweep( input, filter, [&]( shapeId id, vec2 point, vec2 normal, float fraction )
        {
            if( fraction == 0.0f ) return -1.0f;
            if( !closest.hit || fraction < closest.fraction )
                closest = { id, point, normal, fraction, true };
            return closest.fraction;
        }, stats );
        return closest;
    }

#pragma endregion SpatialQueries

#pragma region ShapeBounds

    // 현재 shape의 speculative AABB와 broad-phase fat AABB를 반환함.
    // 내부 storage 참조이므로 shape 생성/파괴 또는 world 파괴를 넘겨 보관하지 않음.
    // Point를 Body local space로 옮겨 실제 geometry에 검사함. Segment는 면적이 없어 false임.
    [[nodiscard]] bool testShapePoint( shapeId id, vec2 point ) const;
    [[nodiscard]] const aabb2& GetShapeAABB( shapeId shapeId ) const;
    [[nodiscard]] const aabb2& GetShapeFatAABB( shapeId shapeId ) const;

#pragma endregion ShapeBounds

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

#pragma endregion BodyProperties

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
    [[nodiscard]] std::size_t GetRecycledContactCount() const noexcept { return recycledContactCount_; }

#pragma endregion WorldSettings

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

#pragma endregion ForcesAndImpulses

#pragma region SimulationStep

    // Contact / Joint constraint를 준비한 뒤 sub-step마다 force / gravity, solve, position integration을 수행함.
    void Step( float timeStep, int subStepCount = 1 );

#pragma endregion SimulationStep

#pragma region HandleValidity

    // world lifetime / null / 범위 / generation / 활성 slot을 모두 확인함.
    [[nodiscard]] bool IsValid( bodyId bodyId ) const noexcept;
    [[nodiscard]] bool IsValid( chainId chainId ) const noexcept;
    [[nodiscard]] bool IsValid( shapeId shapeId ) const noexcept;
    [[nodiscard]] bool IsValid( contactId contactId ) const noexcept;
    [[nodiscard]] bool IsValid( jointId id ) const noexcept;

#pragma endregion HandleValidity

#pragma region CollisionCallbacks

    // 기존 Contact를 갱신하고 broadPhase의 새 AABB pair는 persistent Contact로 생성함.
    // callback은 현재 실제 접촉점이 존재하는 Contact만 받음.
    // callback 중 world 변경이나 Step / UpdateCollisions 재진입은 허용하지 않음.
    template <worldCollisionCallback Callback> void UpdateCollisions( Callback&& callback )
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
        broadPhase_.UpdatePairs( std::span<const shape>{ shapes_ },
            [this, &callback]( std::int32_t shapeIdA, std::int32_t shapeIdB )
            {
                const std::int32_t contactId = createContactForPair( shapeIdA, shapeIdB );
                if( contactId != contact2::NULL_INDEX && IsTouchingManifold( contactSims_[contactId].manifold ) )
                {
                    callback( MakeContactData( contactId ) );
                }
            } );
    }

#pragma endregion CollisionCallbacks

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
    [[nodiscard]] const broadPhase& GetBroadPhase() const noexcept { return broadPhase_; }

    [[nodiscard]] std::size_t GetBodyCount() const noexcept { return bodyCount_; }

    [[nodiscard]] std::size_t GetChainCount() const noexcept { return chainCount_; }

    [[nodiscard]] std::size_t GetShapeCount() const noexcept { return shapeCount_; }

    [[nodiscard]] std::size_t GetContactCount() const noexcept { return contactCount_; }

#pragma endregion DiagnosticQueries

private:
    // Public handle이 world 수명을 구분할 때 사용하는 opaque token.
    std::uint64_t worldToken_ = 0;

#pragma region InternalHandles

    [[nodiscard]] std::int32_t GetBodyIndex( bodyId bodyId ) const;
    [[nodiscard]] std::int32_t GetChainIndex( chainId chainId ) const;
    [[nodiscard]] std::int32_t GetShapeIndex( shapeId shapeId ) const;
    [[nodiscard]] std::int32_t GetContactIndex( contactId contactId ) const;
    [[nodiscard]] std::int32_t getJointIndex( jointId id ) const;

    [[nodiscard]] bodyId MakeBodyId( std::int32_t bodyIndex ) const;
    [[nodiscard]] chainId MakeChainId( std::int32_t chainIndex ) const;
    [[nodiscard]] shapeId MakeShapeId( std::int32_t shapeIndex ) const;
    [[nodiscard]] contactId MakeContactId( std::int32_t contactIndex ) const;
    [[nodiscard]] contactData MakeContactData( std::int32_t contactIndex ) const;
    [[nodiscard]] jointId makeJointId( std::int32_t jointIndex ) const;

#pragma endregion InternalHandles

#pragma region JointStorage

    [[nodiscard]] std::int32_t allocateJoint( std::int32_t bodyIndexA, std::int32_t bodyIndexB, bool collideConnected );
    void destroyJointByIndex( std::int32_t jointIndex, bool touchProxies );
    void resetJointImpulses( std::int32_t bodyIndex );
    void resetSplineRailJoint( jointId id );
    bool applySplineRailProjection( jointId id, const splineRailFrame2& frame, std::optional<float> maxDistance = std::nullopt );
    [[nodiscard]] bool shouldBodiesCollide( std::int32_t bodyIndexA, std::int32_t bodyIndexB ) const;
    [[nodiscard]] bool shouldShapeFiltersCollide( const collisionFilter& a, const collisionFilter& b ) const;

#pragma endregion JointStorage

#pragma region BodyShapeStorage

    // public shape 경로와 chain batch가 같은 slot/proxy/link 로직을 사용함.
    // chain batch는 body 전체 wake와 mass/extents 갱신을 바깥에서 한 번만 수행함.
    [[nodiscard]] std::int32_t CreateShapeByIndex( std::int32_t bodyIndex, shapeGeometry geometry, collisionFilter filter, float density, bool deferBodyUpdates );

    void DestroyBodyByIndex( std::int32_t bodyIndex );
    void DestroyChainByIndex( std::int32_t chainIndex, bool deferBodyUpdates = false );
    void DestroyShapeByIndex( std::int32_t shapeIndex, bool deferBodyUpdates = false, bool destroyContacts = true );

    // 검증된 chain segment geometry를 같은 stable shape slot에 반영하고 contact / proxy bounds를 갱신함.
    void UpdateChainSegmentByIndex( std::int32_t shapeIndex, const chainSegment2& chainSegment );

    // bodySim transform을 기준으로 이 body의 모든 shape proxy를 broadPhase에 동기화함.
    void SyncBodyProxies( std::int32_t bodyIndex );

    // 아직 commit되지 않은 transform을 기준으로 speculative / fat bounds와 tree proxy를 갱신함.
    void UpdateBodyProxyBounds( std::int32_t bodyIndex, const transform2& transform );

    // 현재 local center 기준으로 모든 shape의 CCD / sleep extent만 다시 합산함.
    // 질량과 COM은 바꾸지 않음.
    void UpdateBodyShapeExtents( std::int32_t bodyIndex );

    // 연결된 shape들의 density / geometry를 합산해 Dynamic body의 mass data를 갱신함.
    void UpdateBodyMassData( std::int32_t bodyIndex );

#pragma endregion BodyShapeStorage

#pragma region SensorUpdate

    // 모든 sensor가 세 broad-phase tree를 query해 overlap과 begin/end event를 갱신함.
    void UpdateSensors();

    // sensor shape 파괴 시 dense sensor storage를 정리하고 기존 overlap end event를 예약함.
    void DestroySensorByShapeIndex( std::int32_t shapeIndex );

#pragma endregion SensorUpdate

#pragma region SleepAndWake

    // 현재 solver-active Contact / Joint graph를 따라 연결된 body 전체를 깨움.
    // Static body가 시작점이면 연결된 non-static body만 깨움.
    void WakeBodyByIndex( std::int32_t bodyIndex );

    // 명시적으로 body 하나를 sleep시키면 현재 연결된 island 전체를 함께 sleep시킴.
    void SleepBodyByIndex( std::int32_t bodyIndex );

    // Contact / Joint가 awake / sleeping 경계를 가로지르지 않도록 wake 상태를 전파함.
    void wakeSleepingBodiesFromConstraints();

    // solver가 끝난 island의 motion이 threshold 아래에 충분히 오래 머물렀는지 판정함.
    void UpdateIslandSleepStates( const islandGraph2& islandGraph, float timeStep );

#pragma endregion SleepAndWake

#pragma region ContinuousCollision

    // fast body의 swept path를 검사하고 가장 이른 TOI에서 delta transform을 잘라냄.
    void SolveContinuousBody( std::int32_t bodyIndex, float timeStep );

#pragma endregion ContinuousCollision

#pragma region ContactUpdate

    // callback과 무관한 갱신은 cpp에서 처리하고 실제 접촉 여부만 template에 반환함.
    [[nodiscard]] bool updateExistingContact( std::int32_t contactId );
    [[nodiscard]] std::int32_t createContactForPair( std::int32_t shapeIdA, std::int32_t shapeIdB );

    // stable Contact slot을 할당하고 두 body의 intrusive contact list에 연결함.
    [[nodiscard]] std::int32_t CreateContact( std::int32_t shapeIdA, std::int32_t shapeIdB, const localManifold2& manifold );

    // 상대 transform 변화가 충분히 작으면 cached local anchor로 manifold를 갱신함.
    [[nodiscard]] bool TryRecycleContact( std::int32_t contactId );

    // 현재 bodySim / narrow-phase manifold를 solver용 ContactSim에 동기화하고
    // 다음 recycling에 사용할 anchor / transform cache를 다시 저장함.
    void UpdateContactSim( std::int32_t contactId, const localManifold2& manifold );

#pragma endregion ContactUpdate

#pragma region JointSolver

    using jointConstraint = std::variant<distanceJointConstraint2, filterJointConstraint2, motorJointConstraint2, moverJointConstraint2, pogoJointConstraint2, mouseJointConstraint2, prismaticJointConstraint2, splineRailJointConstraint2, revoluteJointConstraint2, weldJointConstraint2, wheelJointConstraint2>;
    void warmStartJoints( std::span<jointConstraint> constraints );
    void solveJoints( std::span<jointConstraint> constraints, bool useBias );

#pragma endregion JointSolver

#pragma region ContactSolver

    // 한 island의 solver-active Contact를 이번 step의 transient constraint로 변환함.
    [[nodiscard]] std::vector<contactConstraint2> PrepareContactConstraints( std::span<const std::int32_t> contactIds, float timeStep );

    // 이전 step의 cached impulse를 body velocity에 먼저 적용함.
    void WarmStartContacts( std::span<contactConstraint2> constraints );

    // normal constraint를 반복해서 풂.
    // useBias=true는 penetration push, false는 적분 후 velocity relaxation임.
    void SolveContactConstraints( std::span<contactConstraint2> constraints, bool useBias );

    // 충돌 전 접근 속도가 충분히 빠른 Contact에 restitution impulse를 적용함.
    void ApplyRestitutionContacts( std::span<contactConstraint2> constraints );

    // 최종 누적 impulse를 persistent ContactSim으로 되돌림.
    void StoreContactConstraintImpulses( std::span<const contactConstraint2> constraints );

#pragma endregion ContactSolver

#pragma region ContactDestruction

    // body contact list / pairSet에서 해제한 뒤 slot을 free-list로 반환함.
    // Chain batch처럼 연결 component를 이미 깨운 경로는 wake를 생략할 수 있음.
    void DestroyContact( std::int32_t contactId, bool wakeBodies = true );

#pragma endregion ContactDestruction

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

    /// chain
    // parent chain도 shape/body와 같은 stable slot + generation 규칙을 사용함.
    std::vector<chain2> chains_;
    std::int32_t chainFreeList_ = chain2::NULL_INDEX;
    std::size_t chainCount_ = 0;

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
    std::vector<std::variant<distanceJointSim2, filterJointSim2, motorJointSim2, moverJointSim2, pogoJointSim2, mouseJointSim2, prismaticJointSim2, splineRailJointSim2, revoluteJointSim2, weldJointSim2, wheelJointSim2>> jointSims_;
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
#pragma endregion Storage
};

} // namespace zonai
