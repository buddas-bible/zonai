#include "dynamics/world.h"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cmath>
#include <limits>
#include <numbers>
#include <utility>

#include "collision/constants.h"
#include "collision/distance2.h"
#include "collision/narrowphase/collide.h"
#include "collision/shapeProxy2.h"
#include "collision/sweep2.h"
#include "collision/timeOfImpact2.h"
#include "dynamics/bodyShape.h"
#include "dynamics/constants.h"

namespace zonai
{

namespace
{

std::uint64_t allocateWorldToken() noexcept
{
    static std::atomic<std::uint64_t> nextToken{ 1 };

    for( ;; )
    {
        const std::uint64_t token = nextToken.fetch_add( 1, std::memory_order_relaxed );

        if( token != 0 ) return token;
    }
}

} // namespace

#pragma region WorldLifetime

world::world() : worldToken_( allocateWorldToken() )
{
}

#pragma endregion WorldLifetime

#pragma region BodyLifecycle

bodyId world::CreateBody( const bodyDef& definition )
{
    assert( std::isfinite( definition.transform.position.x ) );
    assert( std::isfinite( definition.transform.position.y ) );
    assert( std::isfinite( definition.transform.rotation.c ) );
    assert( std::isfinite( definition.transform.rotation.s ) );

    assert( std::isfinite( definition.linearVelocity.x ) );
    assert( std::isfinite( definition.linearVelocity.y ) );
    assert( std::isfinite( definition.angularVelocity ) );

    assert( std::isfinite( definition.linearDamping ) );
    assert( std::isfinite( definition.angularDamping ) );
    assert( definition.linearDamping >= 0.0f );
    assert( definition.angularDamping >= 0.0f );

    assert( std::isfinite( definition.gravityScale ) );
    assert( std::isfinite( definition.sleepThreshold ) );
    assert( definition.sleepThreshold >= 0.0f );
    assert( std::isfinite( definition.safetyFactor ) );
    assert( definition.safetyFactor >= 0.0f );

    std::int32_t bodyIndex = body::NULL_INDEX;

    if( bodyFreeList_ != body::NULL_INDEX )
    {
        bodyIndex = bodyFreeList_;

        assert( bodyIndex >= 0 );
        assert( static_cast<std::size_t>( bodyIndex ) < bodies_.size() );

        body& freeBody = bodies_[bodyIndex];

        assert( freeBody.bodyId == body::NULL_INDEX );
        assert( freeBody.headShapeId == body::NULL_INDEX );
        assert( freeBody.headContactKey == body::NULL_INDEX );
        assert( freeBody.headJointKey == body::NULL_INDEX && freeBody.jointCount == 0 );
        assert( bodySims_.size() == bodies_.size() );
        assert( bodyStates_.size() == bodies_.size() );
        assert( bodySims_[bodyIndex].bodyId == bodySim::NULL_INDEX );

        bodyFreeList_ = freeBody.nextFreeId;
    }
    else
    {
        assert( bodies_.size() < static_cast<std::size_t>( std::numeric_limits<std::int32_t>::max() - 1 ) );

        bodyIndex = static_cast<std::int32_t>( bodies_.size() );
        bodies_.push_back( {} );
        bodySims_.push_back( {} );
        bodyStates_.push_back( {} );
    }

    body& body = bodies_[bodyIndex];

    const std::uint16_t generation = static_cast<std::uint16_t>( body.generation + 1u );

    body = {};
    body.bodyId = bodyIndex;
    body.generation = generation;
    body.type = definition.type;
    body.enableSleep = definition.enableSleep;
    body.sleepThreshold = definition.sleepThreshold;
    body.safetyFactor = definition.safetyFactor;
    body.awake = definition.type != bodyType::Static && ( definition.isAwake || !definition.enableSleep );

    assert( bodySims_.size() == bodies_.size() );

    bodySim& bodySim = bodySims_[bodyIndex];
    bodySim = {};
    bodySim.bodyId = bodyIndex;
    bodySim.transform = definition.transform;
    bodySim.center = definition.transform.position;
    bodySim.linearDamping = definition.linearDamping;
    bodySim.angularDamping = definition.angularDamping;
    bodySim.gravityScale = definition.gravityScale;
    bodySim.enableContactRecycling = definition.enableContactRecycling;
    bodySim.isBullet = definition.isBullet;
    bodySim.allowFastRotation = definition.allowFastRotation;

    assert( bodyStates_.size() == bodies_.size() );

    bodyState& bodyState = bodyStates_[bodyIndex];
    bodyState = {};

    if( definition.type != bodyType::Static )
    {
        bodyState.linearVelocity = definition.linearVelocity;
        bodyState.angularVelocity = definition.angularVelocity;
    }

    ++bodyCount_;

    return MakeBodyId( bodyIndex );
}

bodyId world::CreateBody( bodyType type, transform2 transform )
{
    bodyDef definition{};
    definition.type = type;
    definition.transform = transform;

    return CreateBody( definition );
}

void world::DestroyBody( bodyId bodyId )
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    DestroyBodyByIndex( bodyIndex );
}

#pragma endregion BodyLifecycle

#pragma region ShapeLifecycle

shapeId world::CreateShape( bodyId bodyId, shapeGeometry geometry, collisionFilter filter, float density )
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    assert( !std::holds_alternative<std::monostate>( geometry ) );
    assert( std::isfinite( density ) );
    assert( density >= 0.0f );

    std::int32_t shapeIndex = shape::NULL_INDEX;

    if( shapeFreeList_ != shape::NULL_INDEX )
    {
        shapeIndex = shapeFreeList_;

        assert( shapeIndex >= 0 );
        assert( static_cast<std::size_t>( shapeIndex ) < shapes_.size() );

        shape& freeShape = shapes_[shapeIndex];

        assert( freeShape.bodyId == shape::NULL_INDEX );
        assert( std::holds_alternative<std::monostate>( freeShape.geometry ) );

        shapeFreeList_ = freeShape.nextFreeId;
    }
    else
    {
        assert( shapes_.size() < static_cast<std::size_t>( std::numeric_limits<std::int32_t>::max() - 1 ) );

        shapeIndex = static_cast<std::int32_t>( shapes_.size() );
        shapes_.push_back( {} );
        fatAABBs_.push_back( {} );
    }

    assert( fatAABBs_.size() == shapes_.size() );

    shape& storedShape = shapes_[shapeIndex];

    const std::uint16_t generation = static_cast<std::uint16_t>( storedShape.generation + 1u );

    storedShape = {};
    storedShape.generation = generation;
    storedShape.geometry = std::move( geometry );
    storedShape.density = density;
    storedShape.filter = filter;
    storedShape.aabbMargin = ComputeShapeAABBMargin( storedShape.geometry );

    body& body = bodies_[bodyIndex];

    WakeBodyByIndex( bodyIndex );

    assert( bodySims_.size() == bodies_.size() );

    const bodySim& bodySim = bodySims_[bodyIndex];

    assert( bodySim.bodyId == bodyIndex );

    const aabb2 tightAABB = ComputeShapeAABB( storedShape.geometry, bodySim.transform );

    storedShape.aabb = ExpandAABB( tightAABB, SPECULATIVE_DISTANCE );

    // Static은 TOI tolerance 때문에 speculative distance를 한 번 더 사용하고,
    // moving body는 shape 크기 기반 margin으로 persistent fat bounds를 만듦.
    const float fatMargin = body.type == bodyType::Static ? SPECULATIVE_DISTANCE : storedShape.aabbMargin;

    aabb2& fatAABB = fatAABBs_[shapeIndex];

    fatAABB = ExpandAABB( storedShape.aabb, fatMargin );

    storedShape.proxyKey = broadPhase_.CreateProxy( body.type, fatAABB, shapeIndex, true );

    LinkShape( body, bodyIndex, shapes_, shapeIndex );
    ++shapeCount_;

    UpdateBodyMassData( bodyIndex );

    return MakeShapeId( shapeIndex );
}

shapeId world::CreateSensorShape( bodyId bodyId, shapeGeometry geometry, collisionFilter filter, float density )
{
    const shapeId sensorShapeId = CreateShape( bodyId, std::move( geometry ), filter, density );

    const std::int32_t shapeIndex = GetShapeIndex( sensorShapeId );

    shape& sensorShape = shapes_[shapeIndex];

    assert( sensorShape.sensorIndex == shape::NULL_INDEX );

    sensorShape.sensorIndex = static_cast<std::int32_t>( sensors_.size() );

    sensor2 sensor{};
    sensor.shapeIndex = shapeIndex;

    sensors_.push_back( std::move( sensor ) );

    return sensorShapeId;
}

void world::DestroyShape( shapeId shapeId )
{
    const std::int32_t shapeIndex = GetShapeIndex( shapeId );

    DestroyShapeByIndex( shapeIndex );
}

#pragma endregion ShapeLifecycle

#pragma region JointLifecycle

jointId world::createDistanceJoint( const distanceJointDef& definition )
{
    const std::int32_t bodyIndexA = GetBodyIndex( definition.bodyA );
    const std::int32_t bodyIndexB = GetBodyIndex( definition.bodyB );
    assert( bodyIndexA != bodyIndexB );
    assert( bodies_[bodyIndexA].type == bodyType::Dynamic || bodies_[bodyIndexB].type == bodyType::Dynamic );
    assert( IsFinite( definition.localAnchorA ) && IsFinite( definition.localAnchorB ) );
    assert( std::isfinite( definition.length ) && definition.length > 0.0f );
    assert( std::isfinite( definition.hertz ) && definition.hertz >= 0.0f );
    assert( std::isfinite( definition.dampingRatio ) && definition.dampingRatio >= 0.0f );
    assert( std::isfinite( definition.minLength ) && std::isfinite( definition.maxLength ) && definition.minLength >= 0.0f && definition.minLength <= definition.maxLength );

    assert( std::isfinite( definition.motorSpeed ) );
    assert( std::isfinite( definition.maxMotorForce ) && definition.maxMotorForce >= 0.0f );

    const std::int32_t index = allocateJoint( bodyIndexA, bodyIndexB, definition.collideConnected );
    jointSims_[index] = distanceJointSim2{};
    distanceJointSim2& sim = std::get<distanceJointSim2>( jointSims_[index] );
    sim = {};
    sim.jointId = index;
    sim.bodyIdA = bodyIndexA;
    sim.bodyIdB = bodyIndexB;
    sim.localAnchorA = definition.localAnchorA;
    sim.localAnchorB = definition.localAnchorB;
    sim.length = std::max( LINEAR_SLOP, definition.length );
    sim.enableSpring = definition.enableSpring;
    sim.hertz = definition.hertz;
    sim.dampingRatio = definition.dampingRatio;
    sim.enableLimit = definition.enableLimit;
    sim.minLength = std::max( LINEAR_SLOP, definition.minLength );
    sim.maxLength = std::max( sim.minLength, definition.maxLength );
    sim.enableMotor = definition.enableMotor;
    sim.motorSpeed = definition.motorSpeed;
    sim.maxMotorForce = definition.maxMotorForce;

    return makeJointId( index );
}

void world::setDistanceJointSpring( jointId id, bool enableSpring, float hertz, float dampingRatio )
{
    assert( std::isfinite( hertz ) && hertz >= 0.0f && std::isfinite( dampingRatio ) && dampingRatio >= 0.0f );

    auto& joint = std::get<distanceJointSim2>( jointSims_[getJointIndex( id )] );
    if( joint.enableSpring == enableSpring && joint.hertz == hertz && joint.dampingRatio == dampingRatio ) return;

    joint.enableSpring = enableSpring;
    joint.hertz = hertz;
    joint.dampingRatio = dampingRatio;
    joint.impulse = 0.0f;
    joint.upperImpulse = 0.0f;
    joint.lowerImpulse = 0.0f;
    joint.motorImpulse = 0.0f;

    // 계수가 바뀌면 연결된 비정적 물체를 깨워 새 설정으로 계산함.
    if( bodies_[joint.bodyIdA].type != bodyType::Static )
    {
        WakeBodyByIndex( joint.bodyIdA );
    }
    if( bodies_[joint.bodyIdB].type != bodyType::Static )
    {
        WakeBodyByIndex( joint.bodyIdB );
    }
}

void world::setDistanceJointLimit( jointId id, bool enableLimit, float minLength, float maxLength )
{
    assert( std::isfinite( minLength ) && std::isfinite( maxLength ) && minLength >= 0.0f && minLength <= maxLength );

    minLength = std::max( LINEAR_SLOP, minLength );
    maxLength = std::max( minLength, maxLength );
    auto& joint = std::get<distanceJointSim2>( jointSims_[getJointIndex( id )] );
    if( joint.enableLimit == enableLimit && joint.minLength == minLength && joint.maxLength == maxLength ) return;

    joint.enableLimit = enableLimit;
    joint.minLength = minLength;
    joint.maxLength = maxLength;
    // 거리 범위 변경은 스프링과 고정 거리 제약의 전환도 일으킬 수 있어 모든 누적 임펄스를 비움.
    joint.upperImpulse = 0.0f;
    joint.lowerImpulse = 0.0f;
    joint.impulse = 0.0f;
    joint.motorImpulse = 0.0f;

    // 계수가 바뀌면 연결된 비정적 물체를 깨워 새 설정으로 계산함.
    if( bodies_[joint.bodyIdA].type != bodyType::Static )
    {
        WakeBodyByIndex( joint.bodyIdA );
    }
    if( bodies_[joint.bodyIdB].type != bodyType::Static )
    {
        WakeBodyByIndex( joint.bodyIdB );
    }
}

void world::setDistanceJointMotor( jointId id, bool enableMotor, float motorSpeed, float maxMotorForce )
{
    assert( std::isfinite( motorSpeed ) );
    assert( std::isfinite( maxMotorForce ) && maxMotorForce >= 0.0f );

    auto& joint = std::get<distanceJointSim2>( jointSims_[getJointIndex( id )] );
    if( joint.enableMotor == enableMotor && joint.motorSpeed == motorSpeed && joint.maxMotorForce == maxMotorForce ) return;

    joint.enableMotor = enableMotor;
    joint.motorSpeed = motorSpeed;
    joint.maxMotorForce = maxMotorForce;

    // 모터와 스프링·limit이 같은 축에서 힘을 주므로 설정 변경 시 함께 누적한 값을 비움.
    joint.impulse = 0.0f;
    joint.lowerImpulse = 0.0f;
    joint.upperImpulse = 0.0f;
    joint.motorImpulse = 0.0f;

    if( bodies_[joint.bodyIdA].type != bodyType::Static )
    {
        WakeBodyByIndex( joint.bodyIdA );
    }
    if( bodies_[joint.bodyIdB].type != bodyType::Static )
    {
        WakeBodyByIndex( joint.bodyIdB );
    }
}

jointId world::createRevoluteJoint( const revoluteJointDef& definition )
{
    const std::int32_t bodyIndexA = GetBodyIndex( definition.bodyA );
    const std::int32_t bodyIndexB = GetBodyIndex( definition.bodyB );
    assert( bodyIndexA != bodyIndexB );
    assert( bodies_[bodyIndexA].type == bodyType::Dynamic || bodies_[bodyIndexB].type == bodyType::Dynamic );
    assert( IsFinite( definition.localAnchorA ) && IsFinite( definition.localAnchorB ) );
    assert( std::isfinite( definition.referenceAngle ) && std::isfinite( definition.lowerAngle ) && std::isfinite( definition.upperAngle ) && definition.lowerAngle <= definition.upperAngle );
    assert( std::isfinite( definition.motorSpeed ) );
    assert( std::isfinite( definition.maxMotorTorque ) && definition.maxMotorTorque >= 0.0f );

    const std::int32_t index = allocateJoint( bodyIndexA, bodyIndexB, definition.collideConnected );
    revoluteJointSim2 sim{};
    sim.jointId = index;
    sim.bodyIdA = bodyIndexA;
    sim.bodyIdB = bodyIndexB;
    sim.localAnchorA = definition.localAnchorA;
    sim.localAnchorB = definition.localAnchorB;
    sim.referenceAngle = definition.referenceAngle;
    sim.enableLimit = definition.enableLimit;
    sim.lowerAngle = std::clamp( definition.lowerAngle, -0.99f * std::numbers::pi_v<float>, 0.99f * std::numbers::pi_v<float> );
    sim.upperAngle = std::clamp( definition.upperAngle, -0.99f * std::numbers::pi_v<float>, 0.99f * std::numbers::pi_v<float> );
    sim.enableMotor = definition.enableMotor;
    sim.motorSpeed = definition.motorSpeed;
    sim.maxMotorTorque = definition.maxMotorTorque;
    jointSims_[index] = sim;

    return makeJointId( index );
}

void world::setRevoluteJointLimit( jointId id, bool enableLimit, float lowerAngle, float upperAngle )
{
    assert( std::isfinite( lowerAngle ) && std::isfinite( upperAngle ) && lowerAngle <= upperAngle );

    lowerAngle = std::clamp( lowerAngle, -0.99f * std::numbers::pi_v<float>, 0.99f * std::numbers::pi_v<float> );
    upperAngle = std::clamp( upperAngle, -0.99f * std::numbers::pi_v<float>, 0.99f * std::numbers::pi_v<float> );
    auto& joint = std::get<revoluteJointSim2>( jointSims_[getJointIndex( id )] );
    if( joint.enableLimit == enableLimit && joint.lowerAngle == lowerAngle && joint.upperAngle == upperAngle ) return;

    joint.enableLimit = enableLimit;
    joint.lowerAngle = lowerAngle;
    joint.upperAngle = upperAngle;
    // 새로운 각도 경계는 연결점 임펄스와도 결합하므로 이전 해를 모두 비움.
    joint.impulse = {};
    joint.lowerImpulse = 0.0f;
    joint.upperImpulse = 0.0f;
    joint.motorImpulse = 0.0f;

    if( bodies_[joint.bodyIdA].type != bodyType::Static )
    {
        WakeBodyByIndex( joint.bodyIdA );
    }
    if( bodies_[joint.bodyIdB].type != bodyType::Static )
    {
        WakeBodyByIndex( joint.bodyIdB );
    }
}

void world::setRevoluteJointMotor( jointId id, bool enableMotor, float motorSpeed, float maxMotorTorque )
{
    assert( std::isfinite( motorSpeed ) );
    assert( std::isfinite( maxMotorTorque ) && maxMotorTorque >= 0.0f );

    auto& joint = std::get<revoluteJointSim2>( jointSims_[getJointIndex( id )] );
    if( joint.enableMotor == enableMotor && joint.motorSpeed == motorSpeed && joint.maxMotorTorque == maxMotorTorque ) return;

    joint.enableMotor = enableMotor;
    joint.motorSpeed = motorSpeed;
    joint.maxMotorTorque = maxMotorTorque;

    // 모터·각도 제한·연결점은 서로 속도를 바꾸므로 설정 변경 시 함께 누적한 값을 비움.
    joint.impulse = {};
    joint.lowerImpulse = 0.0f;
    joint.upperImpulse = 0.0f;
    joint.motorImpulse = 0.0f;

    if( bodies_[joint.bodyIdA].type != bodyType::Static )
    {
        WakeBodyByIndex( joint.bodyIdA );
    }
    if( bodies_[joint.bodyIdB].type != bodyType::Static )
    {
        WakeBodyByIndex( joint.bodyIdB );
    }
}

revoluteJointData world::getRevoluteJointData( jointId id ) const
{
    const std::int32_t index = getJointIndex( id );
    const revoluteJointSim2& sim = std::get<revoluteJointSim2>( jointSims_[index] );
    revoluteJointData data{};
    data.bodyA = MakeBodyId( sim.bodyIdA );
    data.bodyB = MakeBodyId( sim.bodyIdB );
    data.anchorA = TransformPoint( bodySims_[sim.bodyIdA].transform, sim.localAnchorA );
    data.anchorB = TransformPoint( bodySims_[sim.bodyIdB].transform, sim.localAnchorB );

    const rot2 relativeRotation = Inverse( bodySims_[sim.bodyIdA].transform.rotation * rot2::FromRadians( sim.referenceAngle ) ) * bodySims_[sim.bodyIdB].transform.rotation;
    data.currentAngle = std::atan2( relativeRotation.s, relativeRotation.c );
    data.force = sim.subStepTime > 0.0f ? sim.impulse / sim.subStepTime : vec2{};
    data.collideConnected = joints_[index].collideConnected;
    data.referenceAngle = sim.referenceAngle;
    data.enableLimit = sim.enableLimit;
    data.lowerAngle = sim.lowerAngle;
    data.upperAngle = sim.upperAngle;
    data.torque = sim.subStepTime > 0.0f ? ( sim.lowerImpulse - sim.upperImpulse ) / sim.subStepTime : 0.0f;
    data.enableMotor = sim.enableMotor;
    data.motorSpeed = sim.motorSpeed;
    data.maxMotorTorque = sim.maxMotorTorque;
    data.motorTorque = sim.subStepTime > 0.0f ? sim.motorImpulse / sim.subStepTime : 0.0f;

    return data;
}

jointId world::createPrismaticJoint( const prismaticJointDef& definition )
{
    const std::int32_t bodyIndexA = GetBodyIndex( definition.bodyA );
    const std::int32_t bodyIndexB = GetBodyIndex( definition.bodyB );
    assert( bodyIndexA != bodyIndexB );
    assert( bodies_[bodyIndexA].type == bodyType::Dynamic || bodies_[bodyIndexB].type == bodyType::Dynamic );
    assert( IsFinite( definition.localAnchorA ) && IsFinite( definition.localAnchorB ) );
    assert( IsFinite( definition.localAxisA ) && std::abs( LengthSquared( definition.localAxisA ) - 1.0f ) < 0.0001f );
    assert( std::isfinite( definition.referenceAngle ) );
    assert( std::isfinite( definition.hertz ) && definition.hertz >= 0.0f );
    assert( std::isfinite( definition.dampingRatio ) && definition.dampingRatio >= 0.0f );
    assert( std::isfinite( definition.targetTranslation ) );
    assert( std::isfinite( definition.lowerTranslation ) && std::isfinite( definition.upperTranslation ) && definition.lowerTranslation <= definition.upperTranslation );
    assert( std::isfinite( definition.motorSpeed ) );
    assert( std::isfinite( definition.maxMotorForce ) && definition.maxMotorForce >= 0.0f );

    const std::int32_t index = allocateJoint( bodyIndexA, bodyIndexB, definition.collideConnected );
    prismaticJointSim2 sim{};
    sim.jointId = index;
    sim.bodyIdA = bodyIndexA;
    sim.bodyIdB = bodyIndexB;
    sim.localAnchorA = definition.localAnchorA;
    sim.localAnchorB = definition.localAnchorB;
    sim.localAxisA = definition.localAxisA;
    sim.referenceAngle = definition.referenceAngle;
    sim.enableSpring = definition.enableSpring;
    sim.hertz = definition.hertz;
    sim.dampingRatio = definition.dampingRatio;
    sim.targetTranslation = definition.targetTranslation;
    sim.enableLimit = definition.enableLimit;
    sim.lowerTranslation = definition.lowerTranslation;
    sim.upperTranslation = definition.upperTranslation;
    sim.enableMotor = definition.enableMotor;
    sim.motorSpeed = definition.motorSpeed;
    sim.maxMotorForce = definition.maxMotorForce;
    jointSims_[index] = sim;

    return makeJointId( index );
}

void world::setPrismaticJointSpring( jointId id, bool enableSpring, float hertz, float dampingRatio, float targetTranslation )
{
    assert( std::isfinite( hertz ) && hertz >= 0.0f );
    assert( std::isfinite( dampingRatio ) && dampingRatio >= 0.0f );
    assert( std::isfinite( targetTranslation ) );

    auto& joint = std::get<prismaticJointSim2>( jointSims_[getJointIndex( id )] );
    if( joint.enableSpring == enableSpring && joint.hertz == hertz && joint.dampingRatio == dampingRatio && joint.targetTranslation == targetTranslation ) return;

    joint.enableSpring = enableSpring;
    joint.hertz = hertz;
    joint.dampingRatio = dampingRatio;
    joint.targetTranslation = targetTranslation;
    joint.impulse = {};
    joint.springImpulse = 0.0f;
    joint.lowerImpulse = 0.0f;
    joint.upperImpulse = 0.0f;
    joint.motorImpulse = 0.0f;

    if( bodies_[joint.bodyIdA].type != bodyType::Static )
    {
        WakeBodyByIndex( joint.bodyIdA );
    }
    if( bodies_[joint.bodyIdB].type != bodyType::Static )
    {
        WakeBodyByIndex( joint.bodyIdB );
    }
}

void world::setPrismaticJointLimit( jointId id, bool enableLimit, float lowerTranslation, float upperTranslation )
{
    assert( std::isfinite( lowerTranslation ) && std::isfinite( upperTranslation ) && lowerTranslation <= upperTranslation );

    auto& joint = std::get<prismaticJointSim2>( jointSims_[getJointIndex( id )] );
    if( joint.enableLimit == enableLimit && joint.lowerTranslation == lowerTranslation && joint.upperTranslation == upperTranslation ) return;

    joint.enableLimit = enableLimit;
    joint.lowerTranslation = lowerTranslation;
    joint.upperTranslation = upperTranslation;
    joint.impulse = {};
    joint.springImpulse = 0.0f;
    joint.lowerImpulse = 0.0f;
    joint.upperImpulse = 0.0f;

    if( bodies_[joint.bodyIdA].type != bodyType::Static )
    {
        WakeBodyByIndex( joint.bodyIdA );
    }
    if( bodies_[joint.bodyIdB].type != bodyType::Static )
    {
        WakeBodyByIndex( joint.bodyIdB );
    }
}

void world::setPrismaticJointMotor( jointId id, bool enableMotor, float motorSpeed, float maxMotorForce )
{
    assert( std::isfinite( motorSpeed ) );
    assert( std::isfinite( maxMotorForce ) && maxMotorForce >= 0.0f );

    auto& joint = std::get<prismaticJointSim2>( jointSims_[getJointIndex( id )] );
    if( joint.enableMotor == enableMotor && joint.motorSpeed == motorSpeed && joint.maxMotorForce == maxMotorForce ) return;

    joint.enableMotor = enableMotor;
    joint.motorSpeed = motorSpeed;
    joint.maxMotorForce = maxMotorForce;
    joint.impulse = {};
    joint.springImpulse = 0.0f;
    joint.lowerImpulse = 0.0f;
    joint.upperImpulse = 0.0f;
    joint.motorImpulse = 0.0f;

    if( bodies_[joint.bodyIdA].type != bodyType::Static )
    {
        WakeBodyByIndex( joint.bodyIdA );
    }
    if( bodies_[joint.bodyIdB].type != bodyType::Static )
    {
        WakeBodyByIndex( joint.bodyIdB );
    }
}

prismaticJointData world::getPrismaticJointData( jointId id ) const
{
    const std::int32_t index = getJointIndex( id );
    const prismaticJointSim2& sim = std::get<prismaticJointSim2>( jointSims_[index] );
    prismaticJointData data{};
    data.bodyA = MakeBodyId( sim.bodyIdA );
    data.bodyB = MakeBodyId( sim.bodyIdB );
    data.anchorA = TransformPoint( bodySims_[sim.bodyIdA].transform, sim.localAnchorA );
    data.anchorB = TransformPoint( bodySims_[sim.bodyIdB].transform, sim.localAnchorB );
    data.axis = Rotate( bodySims_[sim.bodyIdA].transform.rotation, sim.localAxisA );
    const vec2 perpendicular = Cross( 1.0f, data.axis );
    const vec2 d = data.anchorB - data.anchorA;
    data.currentTranslation = Dot( data.axis, d );
    data.lateralError = Dot( perpendicular, d );
    const rot2 relativeRotation = Inverse( bodySims_[sim.bodyIdA].transform.rotation * rot2::FromRadians( sim.referenceAngle ) ) * bodySims_[sim.bodyIdB].transform.rotation;
    data.currentAngle = std::atan2( relativeRotation.s, relativeRotation.c );
    const float axialImpulse = sim.springImpulse + sim.motorImpulse + sim.lowerImpulse - sim.upperImpulse;
    data.force = sim.subStepTime > 0.0f ? ( sim.impulse.x * perpendicular + axialImpulse * data.axis ) / sim.subStepTime : vec2{};
    data.torque = sim.subStepTime > 0.0f ? sim.impulse.y / sim.subStepTime : 0.0f;
    data.enableSpring = sim.enableSpring;
    data.hertz = sim.hertz;
    data.dampingRatio = sim.dampingRatio;
    data.targetTranslation = sim.targetTranslation;
    data.springForce = sim.subStepTime > 0.0f ? sim.springImpulse / sim.subStepTime : 0.0f;
    data.enableLimit = sim.enableLimit;
    data.lowerTranslation = sim.lowerTranslation;
    data.upperTranslation = sim.upperTranslation;
    data.enableMotor = sim.enableMotor;
    data.motorSpeed = sim.motorSpeed;
    data.maxMotorForce = sim.maxMotorForce;
    data.motorForce = sim.subStepTime > 0.0f ? sim.motorImpulse / sim.subStepTime : 0.0f;
    data.collideConnected = joints_[index].collideConnected;

    return data;
}

jointId world::createWheelJoint( const wheelJointDef& definition )
{
    const std::int32_t bodyIndexA = GetBodyIndex( definition.bodyA );
    const std::int32_t bodyIndexB = GetBodyIndex( definition.bodyB );
    assert( bodyIndexA != bodyIndexB );
    assert( bodies_[bodyIndexA].type == bodyType::Dynamic || bodies_[bodyIndexB].type == bodyType::Dynamic );
    assert( IsFinite( definition.localAnchorA ) && IsFinite( definition.localAnchorB ) );
    assert( IsFinite( definition.localAxisA ) && std::abs( LengthSquared( definition.localAxisA ) - 1.0f ) < 0.0001f );
    assert( std::isfinite( definition.hertz ) && definition.hertz >= 0.0f && std::isfinite( definition.dampingRatio ) && definition.dampingRatio >= 0.0f );
    assert( std::isfinite( definition.lowerTranslation ) && std::isfinite( definition.upperTranslation ) && definition.lowerTranslation <= definition.upperTranslation );
    assert( std::isfinite( definition.motorSpeed ) && std::isfinite( definition.maxMotorTorque ) && definition.maxMotorTorque >= 0.0f );

    const std::int32_t index = allocateJoint( bodyIndexA, bodyIndexB, definition.collideConnected );
    wheelJointSim2 sim{};
    sim.jointId = index;
    sim.bodyIdA = bodyIndexA;
    sim.bodyIdB = bodyIndexB;
    sim.localAnchorA = definition.localAnchorA;
    sim.localAnchorB = definition.localAnchorB;
    sim.localAxisA = definition.localAxisA;
    sim.enableSpring = definition.enableSpring;
    sim.hertz = definition.hertz;
    sim.dampingRatio = definition.dampingRatio;
    sim.enableLimit = definition.enableLimit;
    sim.lowerTranslation = definition.lowerTranslation;
    sim.upperTranslation = definition.upperTranslation;
    sim.enableMotor = definition.enableMotor;
    sim.motorSpeed = definition.motorSpeed;
    sim.maxMotorTorque = definition.maxMotorTorque;
    jointSims_[index] = sim;

    return makeJointId( index );
}

void world::setWheelJointSpring( jointId id, bool enableSpring, float hertz, float dampingRatio )
{
    assert( std::isfinite( hertz ) && hertz >= 0.0f && std::isfinite( dampingRatio ) && dampingRatio >= 0.0f );

    auto& joint = std::get<wheelJointSim2>( jointSims_[getJointIndex( id )] );
    if( joint.enableSpring == enableSpring && joint.hertz == hertz && joint.dampingRatio == dampingRatio ) return;

    joint.enableSpring = enableSpring;
    joint.hertz = hertz;
    joint.dampingRatio = dampingRatio;
    // 모터·스프링·제한·수직 제약이 결합하므로 이전 해를 함께 비움.
    joint.impulse = 0.0f;
    joint.springImpulse = 0.0f;
    joint.lowerImpulse = 0.0f;
    joint.upperImpulse = 0.0f;
    joint.motorImpulse = 0.0f;

    if( bodies_[joint.bodyIdA].type != bodyType::Static )
    {
        WakeBodyByIndex( joint.bodyIdA );
    }
    if( bodies_[joint.bodyIdB].type != bodyType::Static )
    {
        WakeBodyByIndex( joint.bodyIdB );
    }
}

void world::setWheelJointLimit( jointId id, bool enableLimit, float lowerTranslation, float upperTranslation )
{
    assert( std::isfinite( lowerTranslation ) && std::isfinite( upperTranslation ) && lowerTranslation <= upperTranslation );

    auto& joint = std::get<wheelJointSim2>( jointSims_[getJointIndex( id )] );
    if( joint.enableLimit == enableLimit && joint.lowerTranslation == lowerTranslation && joint.upperTranslation == upperTranslation ) return;

    joint.enableLimit = enableLimit;
    joint.lowerTranslation = lowerTranslation;
    joint.upperTranslation = upperTranslation;
    // 새 경계에는 이전 모터·스프링·제한·수직 제약의 결합된 해를 적용하지 않음.
    joint.impulse = 0.0f;
    joint.springImpulse = 0.0f;
    joint.lowerImpulse = 0.0f;
    joint.upperImpulse = 0.0f;
    joint.motorImpulse = 0.0f;

    if( bodies_[joint.bodyIdA].type != bodyType::Static )
    {
        WakeBodyByIndex( joint.bodyIdA );
    }
    if( bodies_[joint.bodyIdB].type != bodyType::Static )
    {
        WakeBodyByIndex( joint.bodyIdB );
    }
}

void world::setWheelJointMotor( jointId id, bool enableMotor, float motorSpeed, float maxMotorTorque )
{
    assert( std::isfinite( motorSpeed ) && std::isfinite( maxMotorTorque ) && maxMotorTorque >= 0.0f );

    auto& joint = std::get<wheelJointSim2>( jointSims_[getJointIndex( id )] );
    if( joint.enableMotor == enableMotor && joint.motorSpeed == motorSpeed && joint.maxMotorTorque == maxMotorTorque ) return;

    joint.enableMotor = enableMotor;
    joint.motorSpeed = motorSpeed;
    joint.maxMotorTorque = maxMotorTorque;
    // 중심 밖 연결점에서는 회전이 스프링·제한·수직 제약과 결합함.
    joint.impulse = 0.0f;
    joint.springImpulse = 0.0f;
    joint.lowerImpulse = 0.0f;
    joint.upperImpulse = 0.0f;
    joint.motorImpulse = 0.0f;

    if( bodies_[joint.bodyIdA].type != bodyType::Static )
    {
        WakeBodyByIndex( joint.bodyIdA );
    }
    if( bodies_[joint.bodyIdB].type != bodyType::Static )
    {
        WakeBodyByIndex( joint.bodyIdB );
    }
}

wheelJointData world::getWheelJointData( jointId id ) const
{
    const std::int32_t index = getJointIndex( id );
    const wheelJointSim2& sim = std::get<wheelJointSim2>( jointSims_[index] );
    wheelJointData data{};
    data.bodyA = MakeBodyId( sim.bodyIdA );
    data.bodyB = MakeBodyId( sim.bodyIdB );
    data.anchorA = TransformPoint( bodySims_[sim.bodyIdA].transform, sim.localAnchorA );
    data.anchorB = TransformPoint( bodySims_[sim.bodyIdB].transform, sim.localAnchorB );
    data.axis = Rotate( bodySims_[sim.bodyIdA].transform.rotation, sim.localAxisA );
    const vec2 perpendicular = Cross( 1.0f, data.axis );
    const vec2 d = data.anchorB - data.anchorA;
    data.currentTranslation = Dot( data.axis, d );
    data.lateralError = Dot( perpendicular, d );
    const float axialImpulse = sim.springImpulse + sim.lowerImpulse - sim.upperImpulse;
    data.force = sim.subStepTime > 0.0f ? ( sim.impulse * perpendicular + axialImpulse * data.axis ) / sim.subStepTime : vec2{};
    data.springForce = sim.subStepTime > 0.0f ? sim.springImpulse / sim.subStepTime : 0.0f;
    data.limitForce = sim.subStepTime > 0.0f ? ( sim.lowerImpulse - sim.upperImpulse ) / sim.subStepTime : 0.0f;
    data.motorTorque = sim.subStepTime > 0.0f ? sim.motorImpulse / sim.subStepTime : 0.0f;
    data.collideConnected = joints_[index].collideConnected;
    data.enableSpring = sim.enableSpring;
    data.hertz = sim.hertz;
    data.dampingRatio = sim.dampingRatio;
    data.enableLimit = sim.enableLimit;
    data.lowerTranslation = sim.lowerTranslation;
    data.upperTranslation = sim.upperTranslation;
    data.enableMotor = sim.enableMotor;
    data.motorSpeed = sim.motorSpeed;
    data.maxMotorTorque = sim.maxMotorTorque;

    return data;
}

jointId world::createMouseJoint( const mouseJointDef& definition )
{
    const std::int32_t bodyIndexA = GetBodyIndex( definition.bodyA );
    const std::int32_t bodyIndexB = GetBodyIndex( definition.bodyB );
    assert( bodies_[bodyIndexA].type == bodyType::Static && bodies_[bodyIndexB].type == bodyType::Dynamic );
    assert( IsFinite( definition.target ) );
    assert( std::isfinite( definition.hertz ) && definition.hertz >= 0.0f );
    assert( std::isfinite( definition.dampingRatio ) && definition.dampingRatio >= 0.0f );
    assert( std::isfinite( definition.maxForce ) && definition.maxForce >= 0.0f );

    // 바닥과의 접촉은 유지함. 정적 A에 반작용을 주지 않고 월드 목표점을 따라감.
    const std::int32_t index = allocateJoint( bodyIndexA, bodyIndexB, true );
    mouseJointSim2 sim{};
    sim.jointId = index;
    sim.bodyIdA = bodyIndexA;
    sim.bodyIdB = bodyIndexB;
    sim.target = definition.target;
    sim.localAnchorB = InverseTransformPoint( bodySims_[bodyIndexB].transform, definition.target );
    sim.hertz = definition.hertz;
    sim.dampingRatio = definition.dampingRatio;
    sim.maxForce = definition.maxForce;
    jointSims_[index] = sim;

    return makeJointId( index );
}

void world::setMouseJointTarget( jointId id, vec2 target )
{
    assert( IsFinite( target ) );

    auto& joint = std::get<mouseJointSim2>( jointSims_[getJointIndex( id )] );
    if( joint.target.x == target.x && joint.target.y == target.y ) return;

    joint.target = target;
    WakeBodyByIndex( joint.bodyIdB );
}

void world::setMouseJointTuning( jointId id, float hertz, float dampingRatio, float maxForce )
{
    assert( std::isfinite( hertz ) && hertz >= 0.0f && std::isfinite( dampingRatio ) && dampingRatio >= 0.0f );
    assert( std::isfinite( maxForce ) && maxForce >= 0.0f );

    auto& joint = std::get<mouseJointSim2>( jointSims_[getJointIndex( id )] );
    if( joint.hertz == hertz && joint.dampingRatio == dampingRatio && joint.maxForce == maxForce ) return;

    joint.hertz = hertz;
    joint.dampingRatio = dampingRatio;
    joint.maxForce = maxForce;
    joint.impulse = {};
    WakeBodyByIndex( joint.bodyIdB );
}

mouseJointData world::getMouseJointData( jointId id ) const
{
    const auto& joint = std::get<mouseJointSim2>( jointSims_[getJointIndex( id )] );
    mouseJointData data{};
    data.bodyA = MakeBodyId( joint.bodyIdA );
    data.bodyB = MakeBodyId( joint.bodyIdB );
    data.target = joint.target;
    data.anchorB = TransformPoint( bodySims_[joint.bodyIdB].transform, joint.localAnchorB );
    data.force = joint.subStepTime > 0.0f ? joint.impulse / joint.subStepTime : vec2{};
    data.hertz = joint.hertz;
    data.dampingRatio = joint.dampingRatio;
    data.maxForce = joint.maxForce;

    return data;
}

void world::destroyJoint( jointId id )
{
    destroyJointByIndex( getJointIndex( id ), true );
}

distanceJointData world::getDistanceJointData( jointId id ) const
{
    const std::int32_t index = getJointIndex( id );
    const distanceJointSim2& sim = std::get<distanceJointSim2>( jointSims_[index] );
    distanceJointData data{};
    data.bodyA = MakeBodyId( sim.bodyIdA );
    data.bodyB = MakeBodyId( sim.bodyIdB );
    data.anchorA = TransformPoint( bodySims_[sim.bodyIdA].transform, sim.localAnchorA );
    data.anchorB = TransformPoint( bodySims_[sim.bodyIdB].transform, sim.localAnchorB );
    data.length = sim.length;
    data.currentLength = Length( data.anchorB - data.anchorA );
    data.collideConnected = joints_[index].collideConnected;
    data.enableSpring = sim.enableSpring;
    data.hertz = sim.hertz;
    data.dampingRatio = sim.dampingRatio;
    data.axialForce = sim.subStepTime > 0.0f ? ( sim.impulse + sim.lowerImpulse - sim.upperImpulse + sim.motorImpulse ) / sim.subStepTime : 0.0f;
    data.enableLimit = sim.enableLimit;
    data.minLength = sim.minLength;
    data.maxLength = sim.maxLength;
    data.enableMotor = sim.enableMotor;
    data.motorSpeed = sim.motorSpeed;
    data.maxMotorForce = sim.maxMotorForce;
    data.motorForce = sim.subStepTime > 0.0f ? sim.motorImpulse / sim.subStepTime : 0.0f;

    return data;
}

#pragma endregion JointLifecycle

#pragma region ShapeProperties

void world::SetShapeDensity( shapeId shapeId, float density )
{
    assert( std::isfinite( density ) );
    assert( density >= 0.0f );

    const std::int32_t shapeIndex = GetShapeIndex( shapeId );
    shape& shape = shapes_[shapeIndex];

    shape.density = density;

    assert( shape.bodyId != shape::NULL_INDEX );

    WakeBodyByIndex( shape.bodyId );
    UpdateBodyMassData( shape.bodyId );
}

float world::GetShapeDensity( shapeId shapeId ) const
{
    const std::int32_t shapeIndex = GetShapeIndex( shapeId );

    return shapes_[shapeIndex].density;
}

void world::SetShapeFriction( shapeId shapeId, float friction )
{
    assert( std::isfinite( friction ) );
    assert( friction >= 0.0f );

    const std::int32_t shapeIndex = GetShapeIndex( shapeId );

    shapes_[shapeIndex].friction = friction;
}

float world::GetShapeFriction( shapeId shapeId ) const
{
    const std::int32_t shapeIndex = GetShapeIndex( shapeId );

    return shapes_[shapeIndex].friction;
}

void world::SetShapeRestitution( shapeId shapeId, float restitution )
{
    assert( std::isfinite( restitution ) );
    assert( restitution >= 0.0f );

    const std::int32_t shapeIndex = GetShapeIndex( shapeId );

    shapes_[shapeIndex].restitution = restitution;
}

float world::GetShapeRestitution( shapeId shapeId ) const
{
    const std::int32_t shapeIndex = GetShapeIndex( shapeId );

    return shapes_[shapeIndex].restitution;
}

void world::SetShapeFilter( shapeId shapeId, collisionFilter filter )
{
    const std::int32_t shapeIndex = GetShapeIndex( shapeId );

    shape& storedShape = shapes_[shapeIndex];

    const collisionFilter& oldFilter = storedShape.filter;

    if( oldFilter.categoryBits == filter.categoryBits && oldFilter.maskBits == filter.maskBits && oldFilter.groupIndex == filter.groupIndex ) return;

    assert( storedShape.bodyId != shape::NULL_INDEX );
    assert( storedShape.proxyKey != shape::NULL_INDEX );

    const std::int32_t bodyIndex = storedShape.bodyId;

    body& owner = bodies_[bodyIndex];

    // Box2D처럼 filter가 바뀐 shape가 참여하는 기존 Contact는 즉시 제거함.
    // 같은 body의 다른 shape Contact는 유지해야 하므로 body list를 훑으며 shape id를 검사함.
    std::int32_t contactKey = owner.headContactKey;

    while( contactKey != body::NULL_INDEX )
    {
        const std::int32_t contactId = GetContactId( contactKey );

        const std::int32_t edgeIndex = GetContactEdgeIndex( contactKey );

        assert( contactId >= 0 );
        assert( static_cast<std::size_t>( contactId ) < contacts_.size() );

        contact2& contact = contacts_[contactId];

        assert( contact.contactId == contactId );

        // DestroyContact가 intrusive list를 수정하므로 다음 key를 먼저 보존함.
        contactKey = contact.edges[edgeIndex].nextKey;

        if( contact.shapeIdA == shapeIndex || contact.shapeIdB == shapeIndex )
        {
            DestroyContact( contactId );
        }
    }

    storedShape.filter = filter;

    // Zonai tree는 categoryBits를 node sorting data로 저장하지 않으므로
    // Box2D처럼 category 변경 시 proxy를 재생성할 필요는 없음.
    // AABB는 그대로 유지하고 moved path만 표시해 다음 UpdatePairs가
    // 새 filter로 이 shape의 후보를 다시 검사하도록 함.
    broadPhase_.TouchProxy( storedShape.proxyKey );
}

collisionFilter world::GetShapeFilter( shapeId shapeId ) const
{
    const std::int32_t shapeIndex = GetShapeIndex( shapeId );

    return shapes_[shapeIndex].filter;
}

void world::setCollisionMatrix( const collisionMatrix& matrix )
{
    if( matrix == collisionMatrix_ ) return;

    collisionMatrix_ = matrix;
    // 공통 규칙 변경은 모든 category 쌍에 영향을 줌. 기존 cache/pair를 버리고 정지 shape도 다시 찾음.
    for( std::int32_t index = 0; index < static_cast<std::int32_t>( contacts_.size() ); ++index )
    {
        if( contacts_[index].contactId != contact2::NULL_INDEX )
        {
            DestroyContact( index );
        }
    }
    for( const shape& value : shapes_ )
    {
        if( value.bodyId != shape::NULL_INDEX )
        {
            broadPhase_.TouchProxy( value.proxyKey );
        }
    }
    for( body& value : bodies_ )
    {
        if( value.bodyId != body::NULL_INDEX && value.type != bodyType::Static )
        {
            value.awake = true;
            value.sleepTime = 0.0f;
        }
    }
}

#pragma endregion ShapeProperties

#pragma region SensorQueries

bool world::IsShapeSensor( shapeId shapeId ) const
{
    const std::int32_t shapeIndex = GetShapeIndex( shapeId );

    return shapes_[shapeIndex].sensorIndex != shape::NULL_INDEX;
}

void world::SetShapeSensorEventsEnabled( shapeId shapeId, bool enabled )
{
    const std::int32_t shapeIndex = GetShapeIndex( shapeId );

    shapes_[shapeIndex].enableSensorEvents = enabled;
}

bool world::AreShapeSensorEventsEnabled( shapeId shapeId ) const
{
    const std::int32_t shapeIndex = GetShapeIndex( shapeId );

    return shapes_[shapeIndex].enableSensorEvents;
}

std::size_t world::GetShapeSensorCapacity( shapeId shapeId ) const
{
    const std::int32_t shapeIndex = GetShapeIndex( shapeId );

    const shape& sensorShape = shapes_[shapeIndex];

    if( sensorShape.sensorIndex == shape::NULL_INDEX ) return 0;

    assert( sensorShape.sensorIndex >= 0 );
    assert( static_cast<std::size_t>( sensorShape.sensorIndex ) < sensors_.size() );

    return sensors_[sensorShape.sensorIndex].overlaps.size();
}

std::size_t world::GetShapeSensorData( shapeId sensorShapeId, std::span<shapeId> output ) const
{
    const std::int32_t shapeIndex = GetShapeIndex( sensorShapeId );

    const shape& sensorShape = shapes_[shapeIndex];

    if( sensorShape.sensorIndex == shape::NULL_INDEX ) return 0;

    const sensor2& sensor = sensors_[sensorShape.sensorIndex];

    const std::size_t count = std::min( output.size(), sensor.overlaps.size() );

    for( std::size_t i = 0; i < count; ++i )
    {
        const sensorVisitor2& visitor = sensor.overlaps[i];

        output[i] = { visitor.shapeIndex + 1, visitor.generation, worldToken_ };
    }

    return count;
}

#pragma endregion SensorQueries

#pragma region ShapeBounds

bool world::testShapePoint( shapeId id, vec2 point ) const
{
    assert( IsFinite( point ) );

    const std::int32_t shapeIndex = GetShapeIndex( id );
    const shape& value = shapes_[shapeIndex];
    const vec2 localPoint = InverseTransformPoint( bodySims_[value.bodyId].transform, point );

    return std::visit(
        [&]( const auto& geometry ) -> bool
        {
            using geometryType = std::remove_cvref_t<decltype( geometry )>;

            if constexpr( std::is_same_v<geometryType, circle2> || std::is_same_v<geometryType, capsule2> )
            {
                return Contains( geometry, localPoint );
            }
            else if constexpr( std::is_same_v<geometryType, polygon2> )
            {
                bool inside = geometry.vertexCount >= 3;
                float distanceSquared = std::numeric_limits<float>::max();

                for( int i = 0; i < geometry.vertexCount; ++i )
                {
                    const vec2 a = geometry.vertices[i], b = geometry.vertices[( i + 1 ) % geometry.vertexCount];

                    if( Dot( geometry.normals[i], localPoint - a ) > 0.0f )
                        inside = false;

                    distanceSquared = std::min( distanceSquared, DistanceSquared( segment2{ a, b }, localPoint ) );
                }

                // Rounded core는 꼭짓점 주변의 circle 거리까지 검사함. 확장된 halfspace만으로는 모서리를 과하게 pick함.
                return inside || distanceSquared <= geometry.radius * geometry.radius;
            }
            else
            {
                return false;
            }
        }, value.geometry );
}

const aabb2& world::GetShapeAABB( shapeId shapeId ) const
{
    const std::int32_t shapeIndex = GetShapeIndex( shapeId );

    return shapes_[shapeIndex].aabb;
}

const aabb2& world::GetShapeFatAABB( shapeId shapeId ) const
{
    assert( fatAABBs_.size() == shapes_.size() );

    const std::int32_t shapeIndex = GetShapeIndex( shapeId );

    return fatAABBs_[shapeIndex];
}

#pragma endregion ShapeBounds

#pragma region BodyProperties

void world::SetBodyTransform( bodyId bodyId, transform2 transform )
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    // transform이 NaN / infinity를 포함하면 tree AABB까지 오염되므로 입구에서 차단함.
    assert( std::isfinite( transform.position.x ) );
    assert( std::isfinite( transform.position.y ) );
    assert( std::isfinite( transform.rotation.c ) );
    assert( std::isfinite( transform.rotation.s ) );

    WakeBodyByIndex( bodyIndex );
    resetJointImpulses( bodyIndex );

    assert( bodySims_.size() == bodies_.size() );

    bodySim& bodySim = bodySims_[bodyIndex];

    assert( bodySim.bodyId == bodyIndex );

    bodySim.transform = transform;
    bodySim.center = TransformPoint( bodySim.transform, bodySim.localCenter );

    SyncBodyProxies( bodyIndex );
}

transform2 world::GetBodyTransform( bodyId bodyId ) const
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    assert( bodySims_.size() == bodies_.size() );

    const bodySim& bodySim = bodySims_[bodyIndex];

    assert( bodySim.bodyId == bodyIndex );

    return bodySim.transform;
}

void world::SetBodyLinearVelocity( bodyId bodyId, vec2 linearVelocity )
{
    assert( std::isfinite( linearVelocity.x ) );
    assert( std::isfinite( linearVelocity.y ) );

    const std::int32_t bodyIndex = GetBodyIndex( bodyId );
    const body& body = bodies_[bodyIndex];

    // 정적 body는 움직이지 않으므로 setter를 무시함.
    if( body.type == bodyType::Static ) return;

    if( linearVelocity.x != 0.0f || linearVelocity.y != 0.0f )
    {
        WakeBodyByIndex( bodyIndex );
    }

    assert( bodyStates_.size() == bodies_.size() );

    bodyStates_[bodyIndex].linearVelocity = linearVelocity;
}

vec2 world::GetBodyLinearVelocity( bodyId bodyId ) const
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    assert( bodyStates_.size() == bodies_.size() );

    // Static body의 state도 항상 zero로 유지되므로 그대로 반환할 수 있음.
    return bodyStates_[bodyIndex].linearVelocity;
}

void world::SetBodyAngularVelocity( bodyId bodyId, float angularVelocity )
{
    assert( std::isfinite( angularVelocity ) );

    const std::int32_t bodyIndex = GetBodyIndex( bodyId );
    const body& body = bodies_[bodyIndex];

    // 정적 body는 움직이지 않으므로 setter를 무시함.
    if( body.type == bodyType::Static ) return;

    if( angularVelocity != 0.0f )
    {
        WakeBodyByIndex( bodyIndex );
    }

    assert( bodyStates_.size() == bodies_.size() );

    bodyStates_[bodyIndex].angularVelocity = angularVelocity;
}

float world::GetBodyAngularVelocity( bodyId bodyId ) const
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    assert( bodyStates_.size() == bodies_.size() );

    return bodyStates_[bodyIndex].angularVelocity;
}

void world::SetBodyLinearDamping( bodyId bodyId, float damping )
{
    assert( std::isfinite( damping ) );
    assert( damping >= 0.0f );

    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    assert( bodySims_.size() == bodies_.size() );

    bodySims_[bodyIndex].linearDamping = damping;
}

float world::GetBodyLinearDamping( bodyId bodyId ) const
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    assert( bodySims_.size() == bodies_.size() );

    return bodySims_[bodyIndex].linearDamping;
}

void world::SetBodyAngularDamping( bodyId bodyId, float damping )
{
    assert( std::isfinite( damping ) );
    assert( damping >= 0.0f );

    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    assert( bodySims_.size() == bodies_.size() );

    bodySims_[bodyIndex].angularDamping = damping;
}

float world::GetBodyAngularDamping( bodyId bodyId ) const
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    assert( bodySims_.size() == bodies_.size() );

    return bodySims_[bodyIndex].angularDamping;
}

void world::SetBodyGravityScale( bodyId bodyId, float scale )
{
    assert( std::isfinite( scale ) );

    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    assert( bodySims_.size() == bodies_.size() );

    bodySims_[bodyIndex].gravityScale = scale;
}

float world::GetBodyGravityScale( bodyId bodyId ) const
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    assert( bodySims_.size() == bodies_.size() );

    return bodySims_[bodyIndex].gravityScale;
}

void world::SetBodyFastRotationAllowed( bodyId bodyId, bool allowed )
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    assert( bodySims_.size() == bodies_.size() );

    bodySims_[bodyIndex].allowFastRotation = allowed;
}

bool world::IsBodyFastRotationAllowed( bodyId bodyId ) const
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    assert( bodySims_.size() == bodies_.size() );

    return bodySims_[bodyIndex].allowFastRotation;
}

void world::SetBodyAwake( bodyId bodyId, bool awake )
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    const body& body = bodies_[bodyIndex];

    if( body.type == bodyType::Static ) return;

    if( awake )
    {
        WakeBodyByIndex( bodyIndex );
    }
    else
    {
        SleepBodyByIndex( bodyIndex );
    }
}

bool world::IsBodyAwake( bodyId bodyId ) const
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    const body& body = bodies_[bodyIndex];

    return body.type != bodyType::Static && body.awake;
}

void world::SetBodySleepEnabled( bodyId bodyId, bool enabled )
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    body& body = bodies_[bodyIndex];

    if( body.type == bodyType::Static ) return;

    if( body.enableSleep == enabled ) return;

    body.enableSleep = enabled;
    body.sleepTime = 0.0f;

    if( !enabled )
    {
        WakeBodyByIndex( bodyIndex );
    }
}

bool world::IsBodySleepEnabled( bodyId bodyId ) const
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    const body& body = bodies_[bodyIndex];

    return body.enableSleep;
}

void world::SetBodySleepThreshold( bodyId bodyId, float threshold )
{
    assert( std::isfinite( threshold ) );
    assert( threshold >= 0.0f );

    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    body& body = bodies_[bodyIndex];

    body.sleepThreshold = threshold;
    body.sleepTime = 0.0f;
}

float world::GetBodySleepThreshold( bodyId bodyId ) const
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    return bodies_[bodyIndex].sleepThreshold;
}

void world::SetBodySafetyFactor( bodyId bodyId, float safetyFactor )
{
    assert( std::isfinite( safetyFactor ) );
    assert( safetyFactor >= 0.0f );

    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    bodies_[bodyIndex].safetyFactor = safetyFactor;
}

float world::GetBodySafetyFactor( bodyId bodyId ) const
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    return bodies_[bodyIndex].safetyFactor;
}

void world::SetBodyBullet( bodyId bodyId, bool bullet )
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    assert( bodySims_.size() == bodies_.size() );

    bodySims_[bodyIndex].isBullet = bullet;
}

bool world::IsBodyBullet( bodyId bodyId ) const
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    assert( bodySims_.size() == bodies_.size() );

    return bodySims_[bodyIndex].isBullet;
}

void world::SetBodyContactRecyclingEnabled( bodyId bodyId, bool enabled )
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    assert( bodySims_.size() == bodies_.size() );

    bodySims_[bodyIndex].enableContactRecycling = enabled;
}

bool world::IsBodyContactRecyclingEnabled( bodyId bodyId ) const
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    assert( bodySims_.size() == bodies_.size() );

    return bodySims_[bodyIndex].enableContactRecycling;
}

bool world::IsBodyFast( bodyId bodyId ) const
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    assert( bodySims_.size() == bodies_.size() );

    return bodySims_[bodyIndex].isFast;
}

bool world::HadBodyTimeOfImpact( bodyId bodyId ) const
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    assert( bodySims_.size() == bodies_.size() );

    return bodySims_[bodyIndex].hadTimeOfImpact;
}

float world::GetBodyMass( bodyId bodyId ) const
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    return bodies_[bodyIndex].mass;
}

float world::GetBodyRotationalInertia( bodyId bodyId ) const
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    return bodies_[bodyIndex].inertia;
}

vec2 world::GetBodyLocalCenter( bodyId bodyId ) const
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    assert( bodySims_.size() == bodies_.size() );

    return bodySims_[bodyIndex].localCenter;
}

#pragma endregion BodyProperties

#pragma region WorldSettings

void world::SetGravity( vec2 gravity )
{
    assert( std::isfinite( gravity.x ) );
    assert( std::isfinite( gravity.y ) );

    if( gravity.x == gravity_.x && gravity.y == gravity_.y ) return;

    gravity_ = gravity;

    // 중력장이 달라지면 sleeping Dynamic body도 다시 반응해야 함.
    for( body& body : bodies_ )
    {
        if( body.bodyId == body::NULL_INDEX || body.type != bodyType::Dynamic ) continue;

        body.awake = true;
        body.sleepTime = 0.0f;
    }
}

vec2 world::GetGravity() const noexcept
{
    return gravity_;
}

void world::SetMaximumLinearSpeed( float speed )
{
    assert( std::isfinite( speed ) );
    assert( speed > 0.0f );

    maximumLinearSpeed_ = speed;
}

float world::GetMaximumLinearSpeed() const noexcept
{
    return maximumLinearSpeed_;
}

void world::SetSleepingEnabled( bool enabled )
{
    if( sleepingEnabled_ == enabled ) return;

    sleepingEnabled_ = enabled;

    if( enabled ) return;

    // World sleep을 끄는 순간 모든 non-static body를 다시 solver에 참여시킴.
    for( body& body : bodies_ )
    {
        if( body.bodyId == body::NULL_INDEX || body.type == bodyType::Static )
            continue;

        body.awake = true;
        body.sleepTime = 0.0f;
    }
}

bool world::IsSleepingEnabled() const noexcept
{
    return sleepingEnabled_;
}

void world::SetContinuousEnabled( bool enabled ) noexcept
{
    continuousEnabled_ = enabled;
}

bool world::IsContinuousEnabled() const noexcept
{
    return continuousEnabled_;
}

void world::SetContactRecycleDistance( float distance )
{
    assert( std::isfinite( distance ) );
    assert( distance >= 0.0f );

    contactRecycleDistance_ = distance;
}

float world::GetContactRecycleDistance() const noexcept
{
    return contactRecycleDistance_;
}

#pragma endregion WorldSettings

#pragma region ForcesAndImpulses

void world::ApplyForce( bodyId bodyId, vec2 force, vec2 point )
{
    assert( std::isfinite( force.x ) );
    assert( std::isfinite( force.y ) );
    assert( std::isfinite( point.x ) );
    assert( std::isfinite( point.y ) );

    const std::int32_t bodyIndex = GetBodyIndex( bodyId );
    const body& body = bodies_[bodyIndex];

    // Static / Kinematic body는 외력으로 속도가 바뀌지 않음.
    if( body.type != bodyType::Dynamic ) return;

    if( force.x != 0.0f || force.y != 0.0f )
    {
        WakeBodyByIndex( bodyIndex );
    }

    assert( bodySims_.size() == bodies_.size() );

    bodySim& bodySim = bodySims_[bodyIndex];

    // F_total += F
    bodySim.force += force;

    // center of mass 기준 torque:
    //
    //     tau = r x F
    //     r   = point - center
    bodySim.torque += Cross( point - bodySim.center, force );
}

void world::ApplyForceToCenter( bodyId bodyId, vec2 force )
{
    assert( std::isfinite( force.x ) );
    assert( std::isfinite( force.y ) );

    const std::int32_t bodyIndex = GetBodyIndex( bodyId );
    const body& body = bodies_[bodyIndex];

    if( body.type != bodyType::Dynamic ) return;

    if( force.x != 0.0f || force.y != 0.0f )
    {
        WakeBodyByIndex( bodyIndex );
    }

    assert( bodySims_.size() == bodies_.size() );

    bodySims_[bodyIndex].force += force;
}

void world::ApplyTorque( bodyId bodyId, float torque )
{
    assert( std::isfinite( torque ) );

    const std::int32_t bodyIndex = GetBodyIndex( bodyId );
    const body& body = bodies_[bodyIndex];

    if( body.type != bodyType::Dynamic ) return;

    if( torque != 0.0f )
    {
        WakeBodyByIndex( bodyIndex );
    }

    assert( bodySims_.size() == bodies_.size() );

    bodySims_[bodyIndex].torque += torque;
}

void world::ClearForces( bodyId bodyId )
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    assert( bodySims_.size() == bodies_.size() );

    bodySim& bodySim = bodySims_[bodyIndex];
    bodySim.force = {};
    bodySim.torque = 0.0f;
}

void world::ApplyLinearImpulse( bodyId bodyId, vec2 impulse, vec2 point )
{
    assert( std::isfinite( impulse.x ) );
    assert( std::isfinite( impulse.y ) );
    assert( std::isfinite( point.x ) );
    assert( std::isfinite( point.y ) );

    const std::int32_t bodyIndex = GetBodyIndex( bodyId );
    const body& body = bodies_[bodyIndex];

    // Static / Kinematic body는 impulse로 속도가 바뀌지 않음.
    if( body.type != bodyType::Dynamic ) return;

    if( impulse.x != 0.0f || impulse.y != 0.0f )
    {
        WakeBodyByIndex( bodyIndex );
    }

    assert( bodySims_.size() == bodies_.size() );
    assert( bodyStates_.size() == bodies_.size() );

    const bodySim& bodySim = bodySims_[bodyIndex];
    bodyState& bodyState = bodyStates_[bodyIndex];

    /*
    * Linear impulse
    *
    * impulse J는 힘을 시간에 대해 적분한 값:
    *
    *     J = integral( F dt )
    *
    * 선운동량 변화:
    *
    *     J = DeltaP = M * DeltaV
    *
    * 따라서:
    *
    *     DeltaV = J / M
    *            = J * invMass
    *
    * Force와 달리 이미 시간이 적분된 값이므로
    * timeStep을 다시 곱하지 않고 즉시 velocity를 변경함.
    */
    bodyState.linearVelocity += impulse * bodySim.invMass;

    /*
    * center of mass에서 벗어난 위치에 impulse가 들어오면
    * angular impulse도 함께 발생함.
    *
    *     r = point - center
    *
    *     L = r x J
    *
    *     DeltaW = L / I
    *            = invInertia * ( r x J )
    */
    const vec2 r = point - bodySim.center;

    bodyState.angularVelocity += bodySim.invInertia * Cross( r, impulse );
}

void world::ApplyLinearImpulseToCenter( bodyId bodyId, vec2 impulse )
{
    assert( std::isfinite( impulse.x ) );
    assert( std::isfinite( impulse.y ) );

    const std::int32_t bodyIndex = GetBodyIndex( bodyId );
    const body& body = bodies_[bodyIndex];

    if( body.type != bodyType::Dynamic ) return;

    if( impulse.x != 0.0f || impulse.y != 0.0f )
    {
        WakeBodyByIndex( bodyIndex );
    }

    assert( bodySims_.size() == bodies_.size() );
    assert( bodyStates_.size() == bodies_.size() );

    const bodySim& bodySim = bodySims_[bodyIndex];
    bodyState& bodyState = bodyStates_[bodyIndex];

    // center에 적용하므로 r=0이고 angular impulse는 발생하지 않음.
    //
    //     DeltaV = J * invMass
    bodyState.linearVelocity += impulse * bodySim.invMass;
}

void world::ApplyAngularImpulse( bodyId bodyId, float impulse )
{
    assert( std::isfinite( impulse ) );

    const std::int32_t bodyIndex = GetBodyIndex( bodyId );
    const body& body = bodies_[bodyIndex];

    if( body.type != bodyType::Dynamic ) return;

    if( impulse != 0.0f )
    {
        WakeBodyByIndex( bodyIndex );
    }

    assert( bodySims_.size() == bodies_.size() );
    assert( bodyStates_.size() == bodies_.size() );

    const bodySim& bodySim = bodySims_[bodyIndex];
    bodyState& bodyState = bodyStates_[bodyIndex];

    /*
    * Angular impulse L은 각운동량의 변화량:
    *
    *     L = DeltaAngularMomentum
    *       = I * DeltaW
    *
    * 따라서:
    *
    *     DeltaW = L / I
    *            = L * invInertia
    */
    bodyState.angularVelocity += impulse * bodySim.invInertia;
}

#pragma endregion ForcesAndImpulses

#pragma region SimulationStep

void world::Step( float timeStep, int subStepCount )
{
    assert( std::isfinite( timeStep ) );
    assert( timeStep >= 0.0f );
    assert( subStepCount > 0 );

    assert( bodySims_.size() == bodies_.size() );
    assert( bodyStates_.size() == bodies_.size() );

    // fast / TOI 표시는 현재 Step에서 다시 계산하는 transient 상태임.
    for( bodySim& sim : bodySims_ )
    {
        sim.isFast = false;
        sim.hadTimeOfImpact = false;
    }

    if( timeStep > 0.0f )
    {
        /*
        * Step 순서
        *
        * 1. 현재 transform의 Contact manifold 갱신
        * 2. solver-active Contact / Joint로 island 구성
        * 3. Contact / Joint constraint 준비
        * 4. 각 sub-step에서:
        *    - force / gravity / damping으로 velocity 갱신
        *    - warm start
        *    - penetration / speculative solve
        *    - velocity 제한 후 delta transform 적분
        *    - bias 없는 relax
        * 5. restitution
        * 6. 최종 impulse 저장
        * 7. delta transform을 bodySim에 반영
        *
        * Contact / Joint는 sub-step 전에 한 번만 준비하고,
        * 각 sub-step의 이동은 bodyState delta transform에 누적함.
        */
        const float subStepTime = timeStep / static_cast<float>( subStepCount );

        const float maxLinearSpeedSquared = maximumLinearSpeed_ * maximumLinearSpeed_;

        // 전체 Step에서 MAX_ROTATION 이상 회전하지 못하게 함.
        // sub-step 수가 바뀌어도 한 Step의 최대 회전량은 동일함.
        const float maxAngularSpeed = MAX_ROTATION / timeStep;
        const float maxAngularSpeedSquared = maxAngularSpeed * maxAngularSpeed;

        // -----------------------------------------------------
        // 1. Update current contacts
        // -----------------------------------------------------
        UpdateCollisions(
            []( const contactData& )
            {
            } );

        wakeSleepingBodiesFromConstraints();

        // -----------------------------------------------------
        // 2. Build islands
        // -----------------------------------------------------
        const islandGraph2 islandGraph = BuildIslands( bodies_, contactSims_, joints_ );

        // -----------------------------------------------------
        // 3. Prepare contact constraints
        // -----------------------------------------------------
        // restitution용 relativeNormalVelocity도 이 시점에서 저장되므로
        // 이번 step의 force / gravity가 아직 섞이지 않은 충돌 전 속도를 사용함.
        std::vector<contactConstraint2> contactConstraints = PrepareContactConstraints( islandGraph.contactIds, subStepTime );

        assert( contactConstraints.size() == islandGraph.contactIds.size() );

        std::vector<jointConstraint> jointConstraints;
        jointConstraints.reserve( islandGraph.jointIds.size() );
        for( const std::int32_t index : islandGraph.jointIds )
        {
            jointConstraints.push_back( std::visit(
                [&]( const auto& joint ) -> jointConstraint
                {
                    if constexpr( std::is_same_v<std::remove_cvref_t<decltype( joint )>, distanceJointSim2> )
                    {
                        return prepareDistanceJointConstraint( joint, bodySims_[joint.bodyIdA], bodySims_[joint.bodyIdB], subStepTime );
                    }
                    else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( joint )>, prismaticJointSim2> )
                    {
                        return preparePrismaticJointConstraint( joint, bodySims_[joint.bodyIdA], bodySims_[joint.bodyIdB], subStepTime );
                    }
                    else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( joint )>, revoluteJointSim2> )
                    {
                        return prepareRevoluteJointConstraint( joint, bodySims_[joint.bodyIdA], bodySims_[joint.bodyIdB], subStepTime );
                    }
                    else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( joint )>, wheelJointSim2> )
                    {
                        return prepareWheelJointConstraint( joint, bodySims_[joint.bodyIdA], bodySims_[joint.bodyIdB], subStepTime );
                    }
                    else
                    {
                        return prepareMouseJointConstraint( joint, bodySims_[joint.bodyIdB], subStepTime );
                    }
                },
                jointSims_[index] ) );
        }

        for( int subStepIndex = 0; subStepIndex < subStepCount; ++subStepIndex )
        {
            ( void )subStepIndex;

            // -------------------------------------------------
            // 4-1. Integrate velocities
            // -------------------------------------------------
            for( std::int32_t bodyIndex = 0; bodyIndex < static_cast<std::int32_t>( bodies_.size() ); ++bodyIndex )
            {
                const body& body = bodies_[bodyIndex];

                if( body.bodyId == body::NULL_INDEX || body.type == bodyType::Static || !body.awake ) continue;

                bodySim& bodySim = bodySims_[bodyIndex];
                bodyState& bodyState = bodyStates_[bodyIndex];

                assert( body.bodyId == bodyIndex );
                assert( bodySim.bodyId == bodyIndex );

                /*
                * Box2D와 같은 Pade 근사 damping:
                *
                *     v2 = v1 / ( 1 + c * h )
                *
                * 현재 velocity에 damping을 적용한 뒤
                * 이번 sub-step의 force / gravity 변화량을 더함.
                */
                const float linearDamping = 1.0f / ( 1.0f + subStepTime * bodySim.linearDamping );

                const float angularDamping = 1.0f / ( 1.0f + subStepTime * bodySim.angularDamping );

                vec2 linearVelocityDelta{};
                float angularVelocityDelta = 0.0f;

                if( body.type == bodyType::Dynamic )
                {
                    linearVelocityDelta = ( bodySim.force * bodySim.invMass + gravity_ * bodySim.gravityScale ) * subStepTime;

                    angularVelocityDelta = subStepTime * bodySim.invInertia * bodySim.torque;
                }

                bodyState.linearVelocity = bodyState.linearVelocity * linearDamping + linearVelocityDelta;

                bodyState.angularVelocity = bodyState.angularVelocity * angularDamping + angularVelocityDelta;
            }

            // -------------------------------------------------
            // 4-2. Warm start
            // -------------------------------------------------
            for( const island2& island : islandGraph.islands )
            {
                const std::span<contactConstraint2> constraints = std::span<contactConstraint2>{ contactConstraints }.subspan( island.contactStart, island.contactCount );

                warmStartJoints( std::span<jointConstraint>{ jointConstraints }.subspan( island.jointStart, island.jointCount ) );
                WarmStartContacts( constraints );
            }

            // -------------------------------------------------
            // 4-3. Solve with penetration / speculative bias
            // -------------------------------------------------
            for( const island2& island : islandGraph.islands )
            {
                const std::span<contactConstraint2> constraints = std::span<contactConstraint2>{ contactConstraints }.subspan( island.contactStart, island.contactCount );

                solveJoints( std::span<jointConstraint>{ jointConstraints }.subspan( island.jointStart, island.jointCount ), true );
                SolveContactConstraints( constraints, true );
            }

            // -------------------------------------------------
            // 4-4. Integrate island position deltas
            // -------------------------------------------------
            for( const island2& island : islandGraph.islands )
            {
                const std::span<const std::int32_t> bodyIds = std::span<const std::int32_t>{ islandGraph.bodyIds }.subspan( island.bodyStart, island.bodyCount );

                for( const std::int32_t bodyIndex : bodyIds )
                {
                    const body& body = bodies_[bodyIndex];
                    const bodySim& bodySim = bodySims_[bodyIndex];
                    bodyState& bodyState = bodyStates_[bodyIndex];

                    assert( body.bodyId == bodyIndex );
                    assert( body.type != bodyType::Static );
                    assert( bodySim.bodyId == bodyIndex );

                    const float linearSpeedSquared = LengthSquared( bodyState.linearVelocity );

                    if( linearSpeedSquared > maxLinearSpeedSquared )
                    {
                        const float ratio = maximumLinearSpeed_ / std::sqrt( linearSpeedSquared );
                        bodyState.linearVelocity *= ratio;
                    }

                    const float angularSpeedSquared = bodyState.angularVelocity * bodyState.angularVelocity;

                    if( angularSpeedSquared > maxAngularSpeedSquared && !bodySim.allowFastRotation )
                    {
                        const float ratio = maxAngularSpeed / std::abs( bodyState.angularVelocity );
                        bodyState.angularVelocity *= ratio;
                    }

                    bodyState.deltaPosition += bodyState.linearVelocity * subStepTime;

                    const float deltaAngle = bodyState.angularVelocity * subStepTime;

                    if( deltaAngle != 0.0f )
                    {
                        bodyState.deltaRotation = rot2::FromRadians( deltaAngle ) * bodyState.deltaRotation;
                    }
                }
            }

            // -------------------------------------------------
            // 4-5. Relax contact velocities
            // -------------------------------------------------
            for( const island2& island : islandGraph.islands )
            {
                const std::span<contactConstraint2> constraints = std::span<contactConstraint2>{ contactConstraints }.subspan( island.contactStart, island.contactCount );

                solveJoints( std::span<jointConstraint>{ jointConstraints }.subspan( island.jointStart, island.jointCount ), false );
                SolveContactConstraints( constraints, false );
            }
        }

        // -----------------------------------------------------
        // 5. Apply restitution
        // -----------------------------------------------------
        for( const island2& island : islandGraph.islands )
        {
            const std::span<contactConstraint2> constraints = std::span<contactConstraint2>{ contactConstraints }.subspan( island.contactStart, island.contactCount );

            ApplyRestitutionContacts( constraints );
        }

        // -----------------------------------------------------
        // 6. Classify fast bodies for continuous collision
        // -----------------------------------------------------
        for( const island2& island : islandGraph.islands )
        {
            const std::span<const std::int32_t> bodyIds = std::span<const std::int32_t>{ islandGraph.bodyIds }.subspan( island.bodyStart, island.bodyCount );

            for( const std::int32_t bodyIndex : bodyIds )
            {
                const body& body = bodies_[bodyIndex];
                bodySim& sim = bodySims_[bodyIndex];
                const bodyState& state = bodyStates_[bodyIndex];

                if( body.type != bodyType::Dynamic || body.shapeCount == 0 ) continue;

                const float maxVelocity = Length( state.linearVelocity ) + std::fabs( state.angularVelocity ) * sim.maxExtent;

                const float maxDeltaPosition = Length( state.deltaPosition ) + std::fabs( state.deltaRotation.s ) * sim.maxExtent;

                const float maxMotion = std::max( maxDeltaPosition, maxVelocity * timeStep );

                sim.isFast = maxMotion > body.safetyFactor * sim.minExtent;
            }
        }

        // -----------------------------------------------------
        // 7. Continuous collision
        // -----------------------------------------------------
        if( continuousEnabled_ )
        {
            // 일반 fast body는 먼저 static geometry에 대해서만 CCD를 수행함.
            // 이 결과로 delta transform이 잘릴 수 있으므로 proxy finalize보다 먼저 처리함.
            for( const island2& island : islandGraph.islands )
            {
                const std::span<const std::int32_t> bodyIds = std::span<const std::int32_t>{ islandGraph.bodyIds }.subspan( island.bodyStart, island.bodyCount );

                for( const std::int32_t bodyIndex : bodyIds )
                {
                    const bodySim& sim = bodySims_[bodyIndex];

                    if( sim.isFast && !sim.isBullet )
                    {
                        SolveContinuousBody( bodyIndex, timeStep );
                    }
                }
            }

            // Bullet을 처리하기 전에 non-bullet world의 이번 Step 최종 bounds를
            // Dynamic Tree에 반영함. Bullet은 이 tree를 query해 moving target도 찾음.
            for( const std::int32_t bodyIndex : islandGraph.bodyIds )
            {
                const bodySim& sim = bodySims_[bodyIndex];

                const bodyState& state = bodyStates_[bodyIndex];

                if( sim.isFast && sim.isBullet ) continue;

                const sweep2 pendingSweep{ sim.localCenter, sim.center, sim.center + state.deltaPosition, sim.transform.rotation, state.deltaRotation * sim.transform.rotation };

                UpdateBodyProxyBounds( bodyIndex, GetSweepTransform( pendingSweep, 1.0f ) );
            }

            // Bullet은 refit된 static / kinematic / dynamic tree를 query함.
            // 다른 bullet은 순서 의존성을 피하기 위해 candidate에서 제외함.
            for( const island2& island : islandGraph.islands )
            {
                const std::span<const std::int32_t> bodyIds = std::span<const std::int32_t>{ islandGraph.bodyIds }.subspan( island.bodyStart, island.bodyCount );

                for( const std::int32_t bodyIndex : bodyIds )
                {
                    const bodySim& sim = bodySims_[bodyIndex];

                    if( sim.isFast && sim.isBullet )
                    {
                        SolveContinuousBody( bodyIndex, timeStep );

                        const bodyState& state = bodyStates_[bodyIndex];

                        const sweep2 pendingSweep{ sim.localCenter, sim.center, sim.center + state.deltaPosition, sim.transform.rotation, state.deltaRotation * sim.transform.rotation };

                        UpdateBodyProxyBounds( bodyIndex, GetSweepTransform( pendingSweep, 1.0f ) );
                    }
                }
            }
        }

        // -----------------------------------------------------
        // 8. Store impulses
        // -----------------------------------------------------
        for( const jointConstraint& value : jointConstraints )
        {
            std::visit(
                [&]( const auto& constraint )
                {
                    using constraintType = std::remove_cvref_t<decltype( constraint )>;
                    using simType = std::conditional_t<std::is_same_v<constraintType, distanceJointConstraint2>, distanceJointSim2, std::conditional_t<std::is_same_v<constraintType, prismaticJointConstraint2>, prismaticJointSim2, std::conditional_t<std::is_same_v<constraintType, revoluteJointConstraint2>, revoluteJointSim2, std::conditional_t<std::is_same_v<constraintType, wheelJointConstraint2>, wheelJointSim2, mouseJointSim2>>>>;
                    auto& joint = std::get<simType>( jointSims_[constraint.jointId] );
                    joint.impulse = constraint.impulse;
                    joint.subStepTime = subStepTime;
                    if constexpr( std::is_same_v<simType, distanceJointSim2> || std::is_same_v<simType, revoluteJointSim2> )
                    {
                        joint.lowerImpulse = constraint.lowerImpulse;
                        joint.upperImpulse = constraint.upperImpulse;
                        joint.motorImpulse = constraint.motorImpulse;
                    }
                    if constexpr( std::is_same_v<simType, prismaticJointSim2> )
                    {
                        joint.springImpulse = constraint.springImpulse;
                        joint.lowerImpulse = constraint.lowerImpulse;
                        joint.upperImpulse = constraint.upperImpulse;
                        joint.motorImpulse = constraint.motorImpulse;
                    }
                    if constexpr( std::is_same_v<simType, wheelJointSim2> )
                    {
                        joint.springImpulse = constraint.springImpulse;
                        joint.lowerImpulse = constraint.lowerImpulse;
                        joint.upperImpulse = constraint.upperImpulse;
                        joint.motorImpulse = constraint.motorImpulse;
                    }
                },
                value );
        }

        StoreContactConstraintImpulses( contactConstraints );

        // -----------------------------------------------------
        // 9. Commit island position deltas
        // -----------------------------------------------------
        for( const island2& island : islandGraph.islands )
        {
            const std::span<const std::int32_t> bodyIds = std::span<const std::int32_t>{ islandGraph.bodyIds }.subspan( island.bodyStart, island.bodyCount );

            for( const std::int32_t bodyIndex : bodyIds )
            {
                bodySim& bodySim = bodySims_[bodyIndex];
                const bodyState& bodyState = bodyStates_[bodyIndex];

                const sweep2 sweep{ bodySim.localCenter, bodySim.center, bodySim.center + bodyState.deltaPosition, bodySim.transform.rotation, bodyState.deltaRotation * bodySim.transform.rotation };

                bodySim.center = sweep.c2;
                bodySim.transform = GetSweepTransform( sweep, 1.0f );

                SyncBodyProxies( bodyIndex );
            }
        }

        UpdateIslandSleepStates( islandGraph, timeStep );

        // force / torque는 모든 sub-step에서 같은 외력으로 사용한 뒤 한 번만 소비함.
        for( std::int32_t bodyIndex = 0; bodyIndex < static_cast<std::int32_t>( bodies_.size() ); ++bodyIndex )
        {
            const body& body = bodies_[bodyIndex];

            if( body.bodyId == body::NULL_INDEX || body.type != bodyType::Dynamic ) continue;

            bodySims_[bodyIndex].force = {};
            bodySims_[bodyIndex].torque = 0.0f;
        }

        // delta transform은 다음 Step으로 넘기지 않음.
        for( const std::int32_t bodyIndex : islandGraph.bodyIds )
        {
            bodyStates_[bodyIndex].deltaPosition = {};
            bodyStates_[bodyIndex].deltaRotation = {};
        }
    }

    // 이동 후 새 broadPhase pair와 manifold를 만들어 다음 Step의 solver가 사용할
    // Contact 상태를 최신 transform 기준으로 준비함.
    UpdateCollisions(
        []( const contactData& )
        {
        } );

    // Sensor는 Contact와 독립적으로 최종 transform 기준 overlap을 갱신함.
    UpdateSensors();
}

#pragma endregion SimulationStep

#pragma region HandleValidity

bool world::IsValid( bodyId bodyId ) const noexcept
{
    if( bodyId.index1 <= 0 || bodyId.worldToken != worldToken_ ) return false;

    const std::int32_t bodyIndex = bodyId.index1 - 1;

    if( static_cast<std::size_t>( bodyIndex ) >= bodies_.size() ) return false;

    const body& body = bodies_[bodyIndex];

    return body.bodyId == bodyIndex && body.generation == bodyId.generation;
}

bool world::IsValid( shapeId shapeId ) const noexcept
{
    if( shapeId.index1 <= 0 || shapeId.worldToken != worldToken_ ) return false;

    const std::int32_t shapeIndex = shapeId.index1 - 1;

    if( static_cast<std::size_t>( shapeIndex ) >= shapes_.size() ) return false;

    const shape& shape = shapes_[shapeIndex];

    return shape.bodyId != shape::NULL_INDEX && shape.generation == shapeId.generation;
}

bool world::IsValid( contactId contactId ) const noexcept
{
    if( contactId.index1 <= 0 || contactId.worldToken != worldToken_ ) return false;

    const std::int32_t contactIndex = contactId.index1 - 1;

    if( static_cast<std::size_t>( contactIndex ) >= contacts_.size() ) return false;

    const contact2& contact = contacts_[contactIndex];

    return contact.contactId == contactIndex && contact.generation == contactId.generation;
}

bool world::IsValid( jointId id ) const noexcept
{
    if( id.index1 <= 0 || id.worldToken != worldToken_ ) return false;

    const std::int32_t index = id.index1 - 1;

    return static_cast<std::size_t>( index ) < joints_.size() && joints_[index].jointId == index && joints_[index].generation == id.generation;
}

#pragma endregion HandleValidity

#pragma region DiagnosticQueries

const body& world::GetBody( bodyId bodyId ) const
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    return bodies_[bodyIndex];
}

const shape& world::GetShape( shapeId shapeId ) const
{
    const std::int32_t shapeIndex = GetShapeIndex( shapeId );

    return shapes_[shapeIndex];
}

contactData world::GetContactData( contactId contactId ) const
{
    return MakeContactData( GetContactIndex( contactId ) );
}

std::size_t world::GetBodyContactCapacity( bodyId bodyId ) const
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    const body& body = bodies_[bodyIndex];

    // Box2D와 같이 빠르고 보수적으로 body의 전체 Contact 수를 반환함.
    return static_cast<std::size_t>( body.contactCount );
}

std::size_t world::GetBodyContactData( bodyId bodyId, std::span<contactData> output ) const
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    const body& body = bodies_[bodyIndex];

    std::int32_t contactKey = body.headContactKey;
    std::size_t count = 0;

    while( contactKey != body::NULL_INDEX && count < output.size() )
    {
        const std::int32_t contactId = GetContactId( contactKey );
        const std::int32_t edgeIndex = GetContactEdgeIndex( contactKey );

        assert( contactId >= 0 );
        assert( static_cast<std::size_t>( contactId ) < contacts_.size() );

        const contact2& contact = contacts_[contactId];

        assert( contact.contactId == contactId );
        assert( edgeIndex == 0 || edgeIndex == 1 );
        assert( contactSims_.size() == contacts_.size() );

        const contactSim2& contactSim = contactSims_[contactId];

        assert( contactSim.contactId == contactId );

        // pointCount가 있으면 speculative point도 solver-active Contact로 public query에 노출함.
        if( contactSim.manifold.pointCount > 0 )
        {
            output[count] = MakeContactData( contactId );
            ++count;
        }

        contactKey = contact.edges[edgeIndex].nextKey;
    }

    return count;
}

std::size_t world::GetShapeContactCapacity( shapeId shapeId ) const
{
    const std::int32_t shapeIndex = GetShapeIndex( shapeId );

    const shape& shape = shapes_[shapeIndex];

    // Sensor는 Contact를 만들지 않으므로 overlap 정보는 Sensor API에서 별도로 조회함.
    if( shape.sensorIndex != shape::NULL_INDEX ) return 0;

    assert( shape.bodyId >= 0 );
    assert( static_cast<std::size_t>( shape.bodyId ) < bodies_.size() );

    const body& body = bodies_[shape.bodyId];

    // 같은 body의 다른 shape Contact도 포함하므로 실제 필요량보다 클 수 있음.
    return static_cast<std::size_t>( body.contactCount );
}

std::size_t world::GetShapeContactData( shapeId shapeId, std::span<contactData> output ) const
{
    const std::int32_t shapeIndex = GetShapeIndex( shapeId );

    const shape& shape = shapes_[shapeIndex];

    if( shape.sensorIndex != shape::NULL_INDEX ) return 0;

    assert( shape.bodyId >= 0 );
    assert( static_cast<std::size_t>( shape.bodyId ) < bodies_.size() );

    const body& body = bodies_[shape.bodyId];

    std::int32_t contactKey = body.headContactKey;
    std::size_t count = 0;

    while( contactKey != body::NULL_INDEX && count < output.size() )
    {
        const std::int32_t contactId = GetContactId( contactKey );
        const std::int32_t edgeIndex = GetContactEdgeIndex( contactKey );

        assert( contactId >= 0 );
        assert( static_cast<std::size_t>( contactId ) < contacts_.size() );

        const contact2& contact = contacts_[contactId];

        assert( contact.contactId == contactId );
        assert( edgeIndex == 0 || edgeIndex == 1 );
        assert( contactSims_.size() == contacts_.size() );

        const contactSim2& contactSim = contactSims_[contactId];

        assert( contactSim.contactId == contactId );

        const bool involvesShape = contact.shapeIdA == shapeIndex || contact.shapeIdB == shapeIndex;

        if( involvesShape && contactSim.manifold.pointCount > 0 )
        {
            output[count] = MakeContactData( contactId );
            ++count;
        }

        contactKey = contact.edges[edgeIndex].nextKey;
    }

    return count;
}

#pragma endregion DiagnosticQueries

#pragma region InternalHandles

std::int32_t world::GetBodyIndex( bodyId bodyId ) const
{
    assert( IsValid( bodyId ) );

    return bodyId.index1 - 1;
}

std::int32_t world::GetShapeIndex( shapeId shapeId ) const
{
    assert( IsValid( shapeId ) );

    return shapeId.index1 - 1;
}

std::int32_t world::GetContactIndex( contactId contactId ) const
{
    assert( IsValid( contactId ) );

    return contactId.index1 - 1;
}

std::int32_t world::getJointIndex( jointId id ) const
{
    assert( IsValid( id ) );

    return id.index1 - 1;
}

bodyId world::MakeBodyId( std::int32_t bodyIndex ) const
{
    assert( bodyIndex >= 0 );
    assert( static_cast<std::size_t>( bodyIndex ) < bodies_.size() );

    const body& body = bodies_[bodyIndex];

    assert( body.bodyId == bodyIndex );

    return { bodyIndex + 1, body.generation, worldToken_ };
}

shapeId world::MakeShapeId( std::int32_t shapeIndex ) const
{
    assert( shapeIndex >= 0 );
    assert( static_cast<std::size_t>( shapeIndex ) < shapes_.size() );

    const shape& shape = shapes_[shapeIndex];

    assert( shape.bodyId != shape::NULL_INDEX );

    return { shapeIndex + 1, shape.generation, worldToken_ };
}

contactId world::MakeContactId( std::int32_t contactIndex ) const
{
    assert( contactIndex >= 0 );
    assert( static_cast<std::size_t>( contactIndex ) < contacts_.size() );

    const contact2& contact = contacts_[contactIndex];

    assert( contact.contactId == contactIndex );

    return { contactIndex + 1, contact.generation, worldToken_ };
}

contactData world::MakeContactData( std::int32_t contactIndex ) const
{
    assert( contactIndex >= 0 );
    assert( static_cast<std::size_t>( contactIndex ) < contacts_.size() );

    const contact2& contact = contacts_[contactIndex];

    assert( contact.contactId == contactIndex );
    assert( contactSims_.size() == contacts_.size() );

    const contactSim2& contactSim = contactSims_[contactIndex];

    assert( contactSim.contactId == contactIndex );
    assert( contact.shapeIdA >= 0 );
    assert( contact.shapeIdB >= 0 );
    assert( static_cast<std::size_t>( contact.shapeIdA ) < shapes_.size() );
    assert( static_cast<std::size_t>( contact.shapeIdB ) < shapes_.size() );

    const shape& shapeA = shapes_[contact.shapeIdA];

    assert( shapeA.bodyId >= 0 );
    assert( static_cast<std::size_t>( shapeA.bodyId ) < bodies_.size() );

    assert( bodySims_.size() == bodies_.size() );

    const bodySim& bodySimA = bodySims_[shapeA.bodyId];

    assert( bodySimA.bodyId == shapeA.bodyId );

    contactData data{};
    data.id = MakeContactId( contactIndex );
    data.shapeA = MakeShapeId( contact.shapeIdA );
    data.shapeB = MakeShapeId( contact.shapeIdB );
    data.manifold = ToWorldManifold( contactSim.manifold, bodySimA.transform );

    // world-space geometry와 함께 마지막 solver impulse도 public snapshot에 복사함.
    for( int i = 0; i < data.manifold.pointCount; ++i )
    {
        data.manifold.points[i].normalImpulse = contactSim.impulses[i].normalImpulse;

        data.manifold.points[i].tangentImpulse = contactSim.impulses[i].tangentImpulse;
    }

    return data;
}

jointId world::makeJointId( std::int32_t jointIndex ) const
{
    assert( joints_[jointIndex].jointId == jointIndex );

    return { jointIndex + 1, joints_[jointIndex].generation, worldToken_ };
}

#pragma endregion InternalHandles

#pragma region JointStorage

std::int32_t world::allocateJoint( std::int32_t bodyIndexA, std::int32_t bodyIndexB, bool collideConnected )
{
    if( !collideConnected )
    {
        // Box2D처럼 이미 존재하는 Contact도 제거함. 삭제 전에 next key를 보존함.
        const body& owner = bodies_[bodies_[bodyIndexA].contactCount < bodies_[bodyIndexB].contactCount ? bodyIndexA : bodyIndexB];
        std::int32_t key = owner.headContactKey;
        while( key != body::NULL_INDEX )
        {
            const std::int32_t index = GetContactId( key );
            const contact2& contact = contacts_[index];
            key = contact.edges[GetContactEdgeIndex( key )].nextKey;
            if( ( contact.edges[0].bodyId == bodyIndexA && contact.edges[1].bodyId == bodyIndexB ) || ( contact.edges[0].bodyId == bodyIndexB && contact.edges[1].bodyId == bodyIndexA ) )
            {
                DestroyContact( index );
            }
        }
    }

    std::int32_t index = jointFreeList_;
    if( index != -1 )
    {
        jointFreeList_ = joints_[index].nextFree;
    }
    else
    {
        // Edge key의 signed shift와 index1이 모두 표현 가능한 범위를 유지함.
        assert( joints_.size() <= static_cast<std::size_t>( std::numeric_limits<std::int32_t>::max() / 2 ) );

        index = static_cast<std::int32_t>( joints_.size() );
        joints_.push_back( {} );
        jointSims_.push_back( {} );
    }
    joint2& joint = joints_[index];
    const auto generation = static_cast<std::uint16_t>( joint.generation + 1u );
    joint = {};
    joint.jointId = index;
    joint.generation = generation;
    joint.collideConnected = collideConnected;
    joint.edges[0].bodyId = bodyIndexA;
    joint.edges[1].bodyId = bodyIndexB;
    for( int edgeIndex = 0; edgeIndex < 2; ++edgeIndex )
    {
        jointEdge2& edge = joint.edges[edgeIndex];
        body& endpoint = bodies_[edge.bodyId];
        const std::int32_t key = ( index << 1 ) | edgeIndex;
        edge.nextKey = endpoint.headJointKey;
        if( edge.nextKey != -1 )
        {
            joints_[edge.nextKey >> 1].edges[edge.nextKey & 1].prevKey = key;
        }
        endpoint.headJointKey = key;
        ++endpoint.jointCount;
    }
    ++jointCount_;
    // Static은 공유 endpoint여도 이웃 Island를 깨우는 경유점이 아님.
    if( bodies_[bodyIndexA].type != bodyType::Static )
    {
        WakeBodyByIndex( bodyIndexA );
    }
    if( bodies_[bodyIndexB].type != bodyType::Static )
    {
        WakeBodyByIndex( bodyIndexB );
    }
    return index;
}

void world::destroyJointByIndex( std::int32_t jointIndex, bool touchProxies )
{
    joint2& joint = joints_[jointIndex];

    assert( joint.jointId == jointIndex && jointCount_ > 0 );

    const std::int32_t bodyIndexA = joint.edges[0].bodyId;
    const std::int32_t bodyIndexB = joint.edges[1].bodyId;
    const bool blocked = !joint.collideConnected;
    for( const jointEdge2& edge : joint.edges )
    {
        body& endpoint = bodies_[edge.bodyId];
        if( edge.prevKey != -1 )
        {
            joints_[edge.prevKey >> 1].edges[edge.prevKey & 1].nextKey = edge.nextKey;
        }
        else
        {
            endpoint.headJointKey = edge.nextKey;
        }
        if( edge.nextKey != -1 )
        {
            joints_[edge.nextKey >> 1].edges[edge.nextKey & 1].prevKey = edge.prevKey;
        }
        --endpoint.jointCount;
        if( blocked && touchProxies )
        {
            // 움직임이 없는 쌍도 다음 UpdatePairs에서 재검사함. Static proxy도 포함함.
            for( std::int32_t shapeIndex = endpoint.headShapeId; shapeIndex != -1; shapeIndex = shapes_[shapeIndex].nextShapeId )
            {
                broadPhase_.TouchProxy( shapes_[shapeIndex].proxyKey );
            }
        }
    }
    const auto generation = joint.generation;
    joint = {};
    joint.generation = generation;
    joint.nextFree = jointFreeList_;
    jointFreeList_ = jointIndex;
    jointSims_[jointIndex] = {};
    --jointCount_;
    // 먼저 edge를 분리해야 남은 component별로 wake가 전파됨.
    // Static pose 변경용 wake 경로를 쓰면 공통 anchor의 다른 Island까지 깨우게 됨.
    // Joint 수명 변경은 non-static endpoint의 component만 깨움.
    if( bodies_[bodyIndexA].type != bodyType::Static )
    {
        WakeBodyByIndex( bodyIndexA );
    }
    if( bodies_[bodyIndexB].type != bodyType::Static )
    {
        WakeBodyByIndex( bodyIndexB );
    }
}

void world::resetJointImpulses( std::int32_t bodyIndex )
{
    for( std::int32_t key = bodies_[bodyIndex].headJointKey; key != -1; key = joints_[key >> 1].edges[key & 1].nextKey )
    {
        std::visit(
            []( auto& joint )
            {
                joint.impulse = {};
                using simType = std::remove_cvref_t<decltype( joint )>;
                if constexpr( std::is_same_v<simType, distanceJointSim2> || std::is_same_v<simType, revoluteJointSim2> )
                {
                    joint.lowerImpulse = 0.0f;
                    joint.upperImpulse = 0.0f;
                    joint.motorImpulse = 0.0f;
                }
                if constexpr( std::is_same_v<simType, prismaticJointSim2> )
                {
                    joint.springImpulse = 0.0f;
                    joint.lowerImpulse = 0.0f;
                    joint.upperImpulse = 0.0f;
                    joint.motorImpulse = 0.0f;
                }
                if constexpr( std::is_same_v<simType, wheelJointSim2> )
                {
                    joint.springImpulse = 0.0f;
                    joint.lowerImpulse = 0.0f;
                    joint.upperImpulse = 0.0f;
                    joint.motorImpulse = 0.0f;
                }
            },
            jointSims_[key >> 1] );
    }
}

bool world::shouldBodiesCollide( std::int32_t bodyIndexA, std::int32_t bodyIndexB ) const
{
    if( bodyIndexA == bodyIndexB ) return false;

    const body& bodyA = bodies_[bodyIndexA];
    const body& bodyB = bodies_[bodyIndexB];
    if( bodyA.type != bodyType::Dynamic && bodyB.type != bodyType::Dynamic ) return false;

    const body& owner = bodyA.jointCount < bodyB.jointCount ? bodyA : bodyB;
    for( std::int32_t key = owner.headJointKey; key != -1; key = joints_[key >> 1].edges[key & 1].nextKey )
    {
        const joint2& joint = joints_[key >> 1];
        const std::int32_t other = joint.edges[( key & 1 ) ^ 1].bodyId;
        if( !joint.collideConnected && ( other == bodyIndexA || other == bodyIndexB ) ) return false;
    }
    return true;
}

bool world::shouldShapeFiltersCollide( const collisionFilter& a, const collisionFilter& b ) const
{
    // 프로젝트 허용 범위 안에서만 기존 group/mask가 동작함. Positive group도 공통 금지를 우회하지 않음.
    return collisionMatrix_.allows( a.categoryBits, b.categoryBits ) && ShouldShapesCollide( a, b );
}

#pragma endregion JointStorage

#pragma region BodyShapeStorage

void world::DestroyBodyByIndex( std::int32_t bodyIndex )
{
    assert( bodyIndex >= 0 );
    assert( static_cast<std::size_t>( bodyIndex ) < bodies_.size() );
    assert( bodyCount_ > 0 );

    body& body = bodies_[bodyIndex];

    assert( body.bodyId == bodyIndex );

    // Body가 살아 있는 동안 Joint edge를 먼저 정리하고, 이후 Contact / Shape를 제거함.
    while( body.headJointKey != body::NULL_INDEX )
    {
        destroyJointByIndex( body.headJointKey >> 1, false );
    }
    assert( body.jointCount == 0 );

    // Box2D처럼 먼저 이 body에 연결된 모든 Contact를 제거함.
    while( body.headContactKey != body::NULL_INDEX )
    {
        const std::int32_t contactId = GetContactId( body.headContactKey );

        assert( contactId >= 0 );
        assert( static_cast<std::size_t>( contactId ) < contacts_.size() );

        DestroyContact( contactId );
    }

    assert( body.contactCount == 0 );

    // shape를 하나씩 제거하면 각 proxy와 shape slot도 함께 정리됨.
    while( body.headShapeId != body::NULL_INDEX )
    {
        DestroyShapeByIndex( body.headShapeId );
    }

    assert( body.shapeCount == 0 );
    assert( body.headShapeId == body::NULL_INDEX );
    assert( body.headContactKey == body::NULL_INDEX );

    const std::uint16_t generation = body.generation;

    assert( bodySims_.size() == bodies_.size() );

    bodySim& bodySim = bodySims_[bodyIndex];

    assert( bodySim.bodyId == bodyIndex );

    // simulation 데이터도 함께 비워 재사용 slot에 이전 transform이 남지 않게 함.
    bodySim = {};

    assert( bodyStates_.size() == bodies_.size() );

    // 운동 상태도 함께 초기화해 재사용 slot에 이전 body 속도가 남지 않게 함.
    bodyStates_[bodyIndex] = {};

    // generation은 보존하고 slot만 free-list에 반환함.
    body = {};
    body.generation = generation;
    body.nextFreeId = bodyFreeList_;
    bodyFreeList_ = bodyIndex;

    --bodyCount_;
}

void world::DestroyShapeByIndex( std::int32_t shapeIndex )
{
    assert( shapeIndex >= 0 );
    assert( static_cast<std::size_t>( shapeIndex ) < shapes_.size() );
    assert( shapeCount_ > 0 );

    shape& shape = shapes_[shapeIndex];

    assert( shape.bodyId != shape::NULL_INDEX );
    assert( shape.proxyKey != shape::NULL_INDEX );
    assert( !std::holds_alternative<std::monostate>( shape.geometry ) );

    if( shape.sensorIndex != shape::NULL_INDEX )
    {
        DestroySensorByShapeIndex( shapeIndex );
    }

    const std::int32_t bodyIndex = shape.bodyId;
    body& body = bodies_[bodyIndex];

    WakeBodyByIndex( bodyIndex );

    // Box2D처럼 먼저 body의 shape list에서 분리함.
    UnlinkShape( body, bodyIndex, shapes_, shapeIndex );

    // 더 이상 broadPhase 후보가 되지 않도록 proxy를 제거함.
    broadPhase_.DestroyProxy( shape.proxyKey );
    shape.proxyKey = shape::NULL_INDEX;

    // 이 body의 Contact list에서 삭제 shape가 관여한 Contact만 제거함.
    std::int32_t contactKey = body.headContactKey;

    while( contactKey != body::NULL_INDEX )
    {
        const std::int32_t contactId = GetContactId( contactKey );
        const std::int32_t edgeIndex = GetContactEdgeIndex( contactKey );

        assert( contactId >= 0 );
        assert( static_cast<std::size_t>( contactId ) < contacts_.size() );

        contact2& contact = contacts_[contactId];

        assert( contact.contactId == contactId );

        // DestroyContact가 list를 수정하므로 다음 key를 먼저 저장함.
        contactKey = contact.edges[edgeIndex].nextKey;

        if( contact.shapeIdA == shapeIndex || contact.shapeIdB == shapeIndex )
        {
            DestroyContact( contactId );
        }
    }

    UpdateBodyMassData( bodyIndex );

    const std::uint16_t generation = shape.generation;

    // generation은 보존하고 slot만 free-list에 반환함.
    shape = {};
    fatAABBs_[shapeIndex] = {};
    shape.generation = generation;
    shape.nextFreeId = shapeFreeList_;
    shapeFreeList_ = shapeIndex;

    --shapeCount_;
}

void world::SyncBodyProxies( std::int32_t bodyIndex )
{
    assert( bodyIndex >= 0 );
    assert( static_cast<std::size_t>( bodyIndex ) < bodies_.size() );
    assert( bodySims_.size() == bodies_.size() );

    const bodySim& bodySim = bodySims_[bodyIndex];

    assert( bodySim.bodyId == bodyIndex );

    UpdateBodyProxyBounds( bodyIndex, bodySim.transform );
}

void world::UpdateBodyProxyBounds( std::int32_t bodyIndex, const transform2& transform )
{
    assert( bodyIndex >= 0 );
    assert( static_cast<std::size_t>( bodyIndex ) < bodies_.size() );
    assert( fatAABBs_.size() == shapes_.size() );

    const body& body = bodies_[bodyIndex];

    assert( body.bodyId == bodyIndex );

    std::int32_t shapeId = body.headShapeId;

    std::int32_t visitedCount = 0;

    while( shapeId != body::NULL_INDEX )
    {
        assert( shapeId >= 0 );
        assert( static_cast<std::size_t>( shapeId ) < shapes_.size() );
        assert( visitedCount < body.shapeCount );

        shape& shape = shapes_[shapeId];

        assert( shape.bodyId == bodyIndex );

        const aabb2 tightAABB = ComputeShapeAABB( shape.geometry, transform );

        shape.aabb = ExpandAABB( tightAABB, SPECULATIVE_DISTANCE );

        aabb2& fatAABB = fatAABBs_[shapeId];

        if( !ContainsAABB( fatAABB, shape.aabb ) )
        {
            const float fatMargin = body.type == bodyType::Static ? SPECULATIVE_DISTANCE : shape.aabbMargin;

            const aabb2 newFatAABB = ExpandAABB( shape.aabb, fatMargin );

            fatAABB = newFatAABB;

            if( shape.proxyKey != shape::NULL_INDEX )
            {
                const dynamicTree& tree = broadPhase_.GetTree( GetProxyType( shape.proxyKey ) );

                const aabb2& treeAABB = tree.GetProxyAABB( GetProxyId( shape.proxyKey ) );

                // 회전이나 geometry 변화처럼 새 fat bounds가 기존 tree bounds를
                // 완전히 포함한다면 topology를 건드리지 않고 bounds만 확장함.
                if( ContainsAABB( newFatAABB, treeAABB ) )
                {
                    broadPhase_.EnlargeProxy( shape.proxyKey, newFatAABB );
                }
                else
                {
                    // translation처럼 기존 bounds를 포함하지 않는 이동은
                    // 현재 tree 구현에서는 leaf를 새 위치로 재배치함.
                    broadPhase_.MoveProxy( shape.proxyKey, newFatAABB );
                }
            }
        }

        shapeId = shape.nextShapeId;

        ++visitedCount;
    }

    assert( visitedCount == body.shapeCount );
}

void world::UpdateBodyMassData( std::int32_t bodyIndex )
{
    assert( bodyIndex >= 0 );
    assert( static_cast<std::size_t>( bodyIndex ) < bodies_.size() );
    assert( bodySims_.size() == bodies_.size() );

    body& body = bodies_[bodyIndex];
    bodySim& bodySim = bodySims_[bodyIndex];

    assert( body.bodyId == bodyIndex );
    assert( bodySim.bodyId == bodyIndex );

    /*
    * body 질량 특성 계산
    *
    * 각 shape i가 다음 값을 가진다고 하면:
    *
    *     mi : shape 질량
    *     ci : shape의 local center of mass
    *     Ii : ci를 지나는 z축 기준 shape 회전 관성
    *
    * body 전체 질량:
    *
    *     M = sum( mi )
    *
    * body 전체 local center of mass:
    *
    *     C = sum( mi * ci ) / M
    *
    * body center C 기준 회전 관성:
    *
    *     I = sum( Ii + mi * |ci - C|^2 )
    *
    * 마지막 식의 mi * |ci - C|^2가 평행축 정리로 추가되는 항임.
    */

    // COM / mass 변경 전 Jacobian으로 만든 warm-start impulse는 버림.
    resetJointImpulses( bodyIndex );

    const vec2 oldCenter = bodySim.center;

    body.mass = 0.0f;
    body.inertia = 0.0f;

    bodySim.invMass = 0.0f;
    bodySim.invInertia = 0.0f;
    bodySim.localCenter = {};
    bodySim.minExtent = std::numeric_limits<float>::max();
    bodySim.maxExtent = 0.0f;

    const auto updateExtents = [&]()
    {
        std::int32_t shapeIndex = body.headShapeId;
        std::int32_t visitedCount = 0;

        while( shapeIndex != body::NULL_INDEX )
        {
            assert( shapeIndex >= 0 );
            assert( static_cast<std::size_t>( shapeIndex ) < shapes_.size() );
            assert( visitedCount < body.shapeCount );

            const shape& shape = shapes_[shapeIndex];

            assert( shape.bodyId == bodyIndex );

            const shapeExtent2 extent = ComputeShapeExtent( shape.geometry, bodySim.localCenter );

            bodySim.minExtent = std::min( bodySim.minExtent, extent.minExtent );

            bodySim.maxExtent = std::max( bodySim.maxExtent, extent.maxExtent );

            shapeIndex = shape.nextShapeId;
            ++visitedCount;
        }

        assert( visitedCount == body.shapeCount );
    };

    // Static / Kinematic body는 외력이나 impulse로 가속되지 않으므로
    // solver 관점에서 무한 질량으로 취급하고 inverse mass / inertia를 0으로 둠.
    if( body.type != bodyType::Dynamic )
    {
        bodySim.center = bodySim.transform.position;
        updateExtents();
        return;
    }

    // 두 번째 관성 계산에서 shape별 mass / center / inertia가 다시 필요하므로
    // 첫 순회 결과를 임시 배열에 저장함.
    std::vector<massData2> masses;
    masses.reserve( static_cast<std::size_t>( body.shapeCount ) );

    // sum( mi * ci )
    vec2 weightedCenter{};

    std::int32_t shapeIndex = body.headShapeId;
    std::int32_t visitedCount = 0;

    /*
    * 첫 번째 순회
    *
    *     M = sum( mi )
    *     centerNumerator = sum( mi * ci )
    *
    * 을 계산함.
    */
    while( shapeIndex != body::NULL_INDEX )
    {
        assert( shapeIndex >= 0 );
        assert( static_cast<std::size_t>( shapeIndex ) < shapes_.size() );
        assert( visitedCount < body.shapeCount );

        const shape& shape = shapes_[shapeIndex];

        assert( shape.bodyId == bodyIndex );

        const massData2 massData = ComputeShapeMass( shape );

        body.mass += massData.mass;
        weightedCenter += massData.center * massData.mass;

        masses.push_back( massData );

        shapeIndex = shape.nextShapeId;
        ++visitedCount;
    }

    assert( visitedCount == body.shapeCount );

    if( body.mass > 0.0f )
    {
        // Solver에서는 나눗셈을 반복하지 않도록 역질량을 미리 저장함.
        //
        //     invMass = 1 / M
        bodySim.invMass = 1.0f / body.mass;

        //     C = sum( mi * ci ) / M
        //       = weightedCenter * invMass
        bodySim.localCenter = weightedCenter * bodySim.invMass;
    }

    /*
    * 두 번째 순회
    *
    * 각 shape의 회전 관성 Ii는 자기 center ci 기준이므로
    * body center C 기준으로 바로 더할 수 없음.
    *
    * 평행축 정리:
    *
    *     I_shifted = Ii + mi * d^2
    *
    *     d = |ci - C|
    *
    * 를 적용한 뒤 모든 shape의 관성을 합산함.
    */
    for( const massData2& massData : masses )
    {
        if( massData.mass == 0.0f ) continue;

        const vec2 offset = bodySim.localCenter - massData.center;

        body.inertia += massData.rotationalInertia + massData.mass * LengthSquared( offset );
    }

    assert( body.inertia >= 0.0f );

    if( body.inertia > 0.0f )
    {
        // Constraint solver에서 angular impulse를 곱셈으로 적용하기 위해
        // 역관성도 미리 계산해 보관함.
        //
        //     invInertia = 1 / I
        bodySim.invInertia = 1.0f / body.inertia;
    }

    updateExtents();

    // local center of mass C를 현재 body transform으로 world space에 옮김.
    bodySim.center = TransformPoint( bodySim.transform, bodySim.localCenter );

    // center of mass가 이동해도 body origin의 순간 속도가 갑자기 변하지 않도록
    // v_new = v_old + w x ( C_new - C_old ) 로 COM 선속도를 보정함.
    assert( bodyStates_.size() == bodies_.size() );

    bodyState& bodyState = bodyStates_[bodyIndex];
    bodyState.linearVelocity += Cross( bodyState.angularVelocity, bodySim.center - oldCenter );
}

#pragma endregion BodyShapeStorage

#pragma region SensorUpdate

void world::UpdateSensors()
{
    sensorBeginEvents_.clear();
    sensorEndEvents_.clear();

    if( !pendingSensorEndEvents_.empty() )
    {
        sensorEndEvents_.insert( sensorEndEvents_.end(), pendingSensorEndEvents_.begin(), pendingSensorEndEvents_.end() );

        pendingSensorEndEvents_.clear();
    }

    if( sensors_.empty() ) return;

    const auto makeVisitorId = [this]( const sensorVisitor2& visitor )
    {
        return shapeId{ visitor.shapeIndex + 1, visitor.generation, worldToken_ };
    };

    constexpr float OVERLAP_EPSILON = 10.0f * std::numeric_limits<float>::epsilon();

    for( sensor2& sensor : sensors_ )
    {
        assert( sensor.shapeIndex >= 0 );
        assert( static_cast<std::size_t>( sensor.shapeIndex ) < shapes_.size() );

        shape& sensorShape = shapes_[sensor.shapeIndex];

        assert( sensorShape.sensorIndex >= 0 );

        // Sensor shape 자체가 이벤트를 끄면 기존 overlap은 아래 diff에서 모두 End로 빠짐.
        if( !sensorShape.enableSensorEvents )
        {
            sensor.hits.clear();

            const shapeId sensorShapeId = MakeShapeId( sensor.shapeIndex );

            for( const sensorVisitor2& oldVisitor : sensor.overlaps )
            {
                sensorEndEvents_.push_back( { sensorShapeId, { oldVisitor.shapeIndex + 1, oldVisitor.generation, worldToken_ } } );
            }

            sensor.overlaps.clear();

            continue;
        }

        const bodySim& sensorBodySim = bodySims_[sensorShape.bodyId];

        std::vector<sensorVisitor2> newOverlaps;
        newOverlaps.reserve( sensor.overlaps.size() + sensor.hits.size() );

        // Continuous pass에서 지나간 visitor도 이번 Step의 overlap state에 한 번 포함함.
        // 최종 위치에서도 실제로 겹치면 아래 tree query 결과와 중복되지만 정렬 후 제거됨.
        newOverlaps.insert( newOverlaps.end(), sensor.hits.begin(), sensor.hits.end() );

        sensor.hits.clear();

        const shapeProxy2 sensorProxy = MakeShapeProxy( sensorShape.geometry );

        const auto queryTree = [&]( bodyType type )
        {
            const dynamicTree& tree = broadPhase_.GetTree( type );

            tree.Query( sensorShape.aabb,
                [&]( std::int32_t proxyId )
                {
                    const std::int32_t otherShapeIndex = tree.GetProxyShapeIndex( proxyId );

                    if( otherShapeIndex == sensor.shapeIndex ) return true;

                    assert( otherShapeIndex >= 0 );
                    assert( static_cast<std::size_t>( otherShapeIndex ) < shapes_.size() );

                    const shape& otherShape = shapes_[otherShapeIndex];

                    if( otherShape.bodyId == shape::NULL_INDEX || otherShape.bodyId == sensorShape.bodyId ) return true;

                    if( !otherShape.enableSensorEvents ) return true;

                    if( !shouldShapeFiltersCollide( sensorShape.filter, otherShape.filter ) ) return true;

                    const bodySim& otherBodySim = bodySims_[otherShape.bodyId];

                    distanceInput2 input{};
                    input.proxyA = sensorProxy;

                    input.proxyB = MakeShapeProxy( otherShape.geometry );

                    input.transform = InverseMul( sensorBodySim.transform, otherBodySim.transform );

                    input.useRadii = true;

                    simplexCache2 cache{};

                    const distanceOutput2 distance = ShapeDistance( input, cache );

                    if( distance.distance <= OVERLAP_EPSILON )
                    {
                        newOverlaps.push_back( { otherShapeIndex, otherShape.generation } );
                    }

                    return true;
                } );
        };

        queryTree( bodyType::Static );

        queryTree( bodyType::Kinematic );

        queryTree( bodyType::Dynamic );

        std::sort( newOverlaps.begin(), newOverlaps.end(),
            []( const sensorVisitor2& a, const sensorVisitor2& b )
            {
                if( a.shapeIndex != b.shapeIndex ) return a.shapeIndex < b.shapeIndex;

                return a.generation < b.generation;
            } );

        newOverlaps.erase( std::unique( newOverlaps.begin(), newOverlaps.end() ), newOverlaps.end() );

        const shapeId sensorShapeId = MakeShapeId( sensor.shapeIndex );

        std::size_t oldIndex = 0;
        std::size_t newIndex = 0;

        while( oldIndex < sensor.overlaps.size() && newIndex < newOverlaps.size() )
        {
            const sensorVisitor2& oldVisitor = sensor.overlaps[oldIndex];

            const sensorVisitor2& newVisitor = newOverlaps[newIndex];

            if( oldVisitor.shapeIndex == newVisitor.shapeIndex && oldVisitor.generation == newVisitor.generation )
            {
                ++oldIndex;
                ++newIndex;
                continue;
            }

            const bool oldComesFirst = oldVisitor.shapeIndex < newVisitor.shapeIndex || ( oldVisitor.shapeIndex == newVisitor.shapeIndex && oldVisitor.generation < newVisitor.generation );

            if( oldComesFirst )
            {
                sensorEndEvents_.push_back( { sensorShapeId, makeVisitorId( oldVisitor ) } );

                ++oldIndex;
            }
            else
            {
                sensorBeginEvents_.push_back( { sensorShapeId, makeVisitorId( newVisitor ) } );

                ++newIndex;
            }
        }

        while( oldIndex < sensor.overlaps.size() )
        {
            sensorEndEvents_.push_back( { sensorShapeId, makeVisitorId( sensor.overlaps[oldIndex] ) } );

            ++oldIndex;
        }

        while( newIndex < newOverlaps.size() )
        {
            sensorBeginEvents_.push_back( { sensorShapeId, makeVisitorId( newOverlaps[newIndex] ) } );

            ++newIndex;
        }

        sensor.overlaps = std::move( newOverlaps );
    }
}

void world::DestroySensorByShapeIndex( std::int32_t shapeIndex )
{
    assert( shapeIndex >= 0 );
    assert( static_cast<std::size_t>( shapeIndex ) < shapes_.size() );

    shape& sensorShape = shapes_[shapeIndex];

    assert( sensorShape.sensorIndex != shape::NULL_INDEX );

    const std::int32_t sensorIndex = sensorShape.sensorIndex;

    assert( sensorIndex >= 0 );
    assert( static_cast<std::size_t>( sensorIndex ) < sensors_.size() );

    const shapeId sensorShapeId = MakeShapeId( shapeIndex );

    const sensor2& sensor = sensors_[sensorIndex];

    for( const sensorVisitor2& visitor : sensor.overlaps )
    {
        pendingSensorEndEvents_.push_back( { sensorShapeId, { visitor.shapeIndex + 1, visitor.generation, worldToken_ } } );
    }

    const std::int32_t lastSensorIndex = static_cast<std::int32_t>( sensors_.size() - 1 );

    if( sensorIndex != lastSensorIndex )
    {
        sensors_[sensorIndex] = std::move( sensors_[lastSensorIndex] );

        const std::int32_t movedShapeIndex = sensors_[sensorIndex].shapeIndex;

        assert( movedShapeIndex >= 0 );
        assert( static_cast<std::size_t>( movedShapeIndex ) < shapes_.size() );

        shapes_[movedShapeIndex].sensorIndex = sensorIndex;
    }

    sensors_.pop_back();

    sensorShape.sensorIndex = shape::NULL_INDEX;
}

#pragma endregion SensorUpdate

#pragma region SleepAndWake

void world::WakeBodyByIndex( std::int32_t bodyIndex )
{
    assert( bodyIndex >= 0 );
    assert( static_cast<std::size_t>( bodyIndex ) < bodies_.size() );

    const body& startBody = bodies_[bodyIndex];

    if( startBody.bodyId == body::NULL_INDEX ) return;

    std::vector<std::uint8_t> visited( bodies_.size(), 0 );

    std::vector<std::int32_t> stack;

    const auto pushBody = [&]( std::int32_t candidateId )
    {
        assert( candidateId >= 0 );
        assert( static_cast<std::size_t>( candidateId ) < bodies_.size() );

        const body& candidate = bodies_[candidateId];

        if( candidate.bodyId == body::NULL_INDEX || candidate.type == bodyType::Static || visited[candidateId] != 0 ) return;

        visited[candidateId] = 1;
        stack.push_back( candidateId );
    };

    const auto pushJointNeighbors = [&]( const body& endpoint )
    {
        for( std::int32_t key = endpoint.headJointKey; key != -1; key = joints_[key >> 1].edges[key & 1].nextKey )
        {
            pushBody( joints_[key >> 1].edges[( key & 1 ) ^ 1].bodyId );
        }
    };

    if( startBody.type == bodyType::Static )
    {
        pushJointNeighbors( startBody );
        std::int32_t contactKey = startBody.headContactKey;

        while( contactKey != body::NULL_INDEX )
        {
            const std::int32_t contactId = GetContactId( contactKey );

            const std::int32_t edgeIndex = GetContactEdgeIndex( contactKey );

            assert( contactId >= 0 );
            assert( static_cast<std::size_t>( contactId ) < contacts_.size() );

            const contact2& contact = contacts_[contactId];

            assert( contact.contactId == contactId );

            contactKey = contact.edges[edgeIndex].nextKey;

            const contactSim2& contactSim = contactSims_[contactId];

            if( contactSim.manifold.pointCount == 0 ) continue;

            pushBody( contact.edges[edgeIndex ^ 1].bodyId );
        }
    }
    else
    {
        pushBody( bodyIndex );
    }

    while( !stack.empty() )
    {
        const std::int32_t currentBodyId = stack.back();

        stack.pop_back();

        body& currentBody = bodies_[currentBodyId];

        currentBody.awake = true;
        currentBody.sleepTime = 0.0f;
        pushJointNeighbors( currentBody );

        std::int32_t contactKey = currentBody.headContactKey;

        while( contactKey != body::NULL_INDEX )
        {
            const std::int32_t contactId = GetContactId( contactKey );

            const std::int32_t edgeIndex = GetContactEdgeIndex( contactKey );

            assert( contactId >= 0 );
            assert( static_cast<std::size_t>( contactId ) < contacts_.size() );

            const contact2& contact = contacts_[contactId];

            assert( contact.contactId == contactId );

            contactKey = contact.edges[edgeIndex].nextKey;

            const contactSim2& contactSim = contactSims_[contactId];

            if( contactSim.manifold.pointCount == 0 ) continue;

            pushBody( contact.edges[edgeIndex ^ 1].bodyId );
        }
    }
}

void world::SleepBodyByIndex( std::int32_t bodyIndex )
{
    assert( bodyIndex >= 0 );
    assert( static_cast<std::size_t>( bodyIndex ) < bodies_.size() );

    const body& startBody = bodies_[bodyIndex];

    if( startBody.bodyId == body::NULL_INDEX || startBody.type == bodyType::Static ) return;

    std::vector<std::uint8_t> visited( bodies_.size(), 0 );

    std::vector<std::int32_t> stack;
    std::vector<std::int32_t> connectedBodies;

    visited[bodyIndex] = 1;
    stack.push_back( bodyIndex );

    while( !stack.empty() )
    {
        const std::int32_t currentBodyId = stack.back();

        stack.pop_back();

        connectedBodies.push_back( currentBodyId );

        const body& currentBody = bodies_[currentBodyId];

        for( std::int32_t key = currentBody.headJointKey; key != -1; key = joints_[key >> 1].edges[key & 1].nextKey )
        {
            const std::int32_t other = joints_[key >> 1].edges[( key & 1 ) ^ 1].bodyId;
            if( bodies_[other].type != bodyType::Static && visited[other] == 0 )
            {
                visited[other] = 1;
                stack.push_back( other );
            }
        }

        std::int32_t contactKey = currentBody.headContactKey;

        while( contactKey != body::NULL_INDEX )
        {
            const std::int32_t contactId = GetContactId( contactKey );

            const std::int32_t edgeIndex = GetContactEdgeIndex( contactKey );

            assert( contactId >= 0 );
            assert( static_cast<std::size_t>( contactId ) < contacts_.size() );

            const contact2& contact = contacts_[contactId];

            contactKey = contact.edges[edgeIndex].nextKey;

            const contactSim2& contactSim = contactSims_[contactId];

            if( contactSim.manifold.pointCount == 0 ) continue;

            const std::int32_t otherBodyId = contact.edges[edgeIndex ^ 1].bodyId;

            assert( otherBodyId >= 0 );
            assert( static_cast<std::size_t>( otherBodyId ) < bodies_.size() );

            const body& otherBody = bodies_[otherBodyId];

            if( otherBody.type == bodyType::Static || visited[otherBodyId] != 0 ) continue;

            visited[otherBodyId] = 1;
            stack.push_back( otherBodyId );
        }
    }

    for( const std::int32_t connectedBodyId : connectedBodies )
    {
        body& connectedBody = bodies_[connectedBodyId];

        connectedBody.awake = false;
        connectedBody.sleepTime = 0.0f;

        bodyStates_[connectedBodyId] = {};

        bodySims_[connectedBodyId].force = {};
        bodySims_[connectedBodyId].torque = 0.0f;
    }
}

void world::wakeSleepingBodiesFromConstraints()
{
    for( const contactSim2& contactSim : contactSims_ )
    {
        if( contactSim.contactId == contactSim2::NULL_INDEX || contactSim.manifold.pointCount == 0 ) continue;

        body& bodyA = bodies_[contactSim.bodyIdA];

        body& bodyB = bodies_[contactSim.bodyIdB];

        const bool bodyAAwake = bodyA.type != bodyType::Static && bodyA.awake;

        const bool bodyBAwake = bodyB.type != bodyType::Static && bodyB.awake;

        if( bodyAAwake && bodyB.type != bodyType::Static && !bodyB.awake )
        {
            WakeBodyByIndex( contactSim.bodyIdB );
        }
        else if( bodyBAwake && bodyA.type != bodyType::Static && !bodyA.awake )
        {
            WakeBodyByIndex( contactSim.bodyIdA );
        }
    }
    for( const joint2& joint : joints_ )
    {
        if( joint.jointId == -1 ) continue;

        const body& a = bodies_[joint.edges[0].bodyId];
        const body& b = bodies_[joint.edges[1].bodyId];
        if( a.type != bodyType::Static && b.type != bodyType::Static && a.awake != b.awake )
        {
            WakeBodyByIndex( a.awake ? joint.edges[1].bodyId : joint.edges[0].bodyId );
        }
    }
}

void world::UpdateIslandSleepStates( const islandGraph2& islandGraph, float timeStep )
{
    assert( std::isfinite( timeStep ) );
    assert( timeStep > 0.0f );

    if( !sleepingEnabled_ ) return;

    for( const island2& island : islandGraph.islands )
    {
        const std::span<const std::int32_t> bodyIds = std::span<const std::int32_t>{ islandGraph.bodyIds }.subspan( island.bodyStart, island.bodyCount );

        bool canSleep = true;

        for( const std::int32_t bodyId : bodyIds )
        {
            body& body = bodies_[bodyId];

            assert( body.awake );
            assert( body.type != bodyType::Static );

            if( !body.enableSleep )
            {
                body.sleepTime = 0.0f;
                canSleep = false;
                continue;
            }

            const bodyState& state = bodyStates_[bodyId];

            const bodySim& sim = bodySims_[bodyId];

            // 가장 먼 점의 속도와 이번 step의 position correction을 함께 확인함.
            const float velocitySpeed = Length( state.linearVelocity ) + std::fabs( state.angularVelocity ) * sim.maxExtent;

            const float deltaDistance = Length( state.deltaPosition ) + std::fabs( state.deltaRotation.s ) * sim.maxExtent;

            const float correctionSpeed = 0.5f * deltaDistance / timeStep;
            const float motionSpeed = std::max( velocitySpeed, correctionSpeed );

            if( motionSpeed > body.sleepThreshold )
            {
                body.sleepTime = 0.0f;
                canSleep = false;
                continue;
            }

            body.sleepTime += timeStep;

            if( body.sleepTime < TIME_TO_SLEEP )
            {
                canSleep = false;
            }
        }

        if( !canSleep ) continue;

        for( const std::int32_t bodyId : bodyIds )
        {
            body& body = bodies_[bodyId];

            body.awake = false;
            body.sleepTime = 0.0f;

            bodyStates_[bodyId] = {};

            bodySims_[bodyId].force = {};
            bodySims_[bodyId].torque = 0.0f;
        }
    }
}

#pragma endregion SleepAndWake

#pragma region ContinuousCollision

void world::SolveContinuousBody( std::int32_t bodyIndex, float timeStep )
{
    assert( bodyIndex >= 0 );
    assert( static_cast<std::size_t>( bodyIndex ) < bodies_.size() );
    assert( std::isfinite( timeStep ) );
    assert( timeStep > 0.0f );

    const body& fastBody = bodies_[bodyIndex];

    bodySim& fastSim = bodySims_[bodyIndex];

    bodyState& fastState = bodyStates_[bodyIndex];

    assert( fastBody.bodyId == bodyIndex );
    assert( fastBody.type == bodyType::Dynamic );
    assert( fastSim.bodyId == bodyIndex );
    assert( fastSim.isFast );

    const sweep2 fastSweep{ fastSim.localCenter, fastSim.center, fastSim.center + fastState.deltaPosition, fastSim.transform.rotation, fastState.deltaRotation * fastSim.transform.rotation };

    // 회전 중 어느 방향을 향하더라도 body 전체가 들어오는 보수적인 swept bounds.
    const float sweepRadius = fastSim.maxExtent + SPECULATIVE_DISTANCE;

    const aabb2 sweptAABB{ { std::min( fastSweep.c1.x, fastSweep.c2.x ) - sweepRadius, std::min( fastSweep.c1.y, fastSweep.c2.y ) - sweepRadius }, { std::max( fastSweep.c1.x, fastSweep.c2.x ) + sweepRadius, std::max( fastSweep.c1.y, fastSweep.c2.y ) + sweepRadius } };

    float hitFraction = 1.0f;

    struct continuousSensorHit2
    {
        std::int32_t sensorShapeIndex = shape::NULL_INDEX;
        sensorVisitor2 visitor{};
        float fraction = 1.0f;
    };

    std::vector<continuousSensorHit2> sensorHits;
    sensorHits.reserve( MAX_CONTINUOUS_SENSOR_HITS );

    const auto testCandidate = [&]( std::int32_t fastShapeIndex, std::int32_t otherShapeIndex )
    {
        assert( fastShapeIndex >= 0 );
        assert( otherShapeIndex >= 0 );
        assert( static_cast<std::size_t>( fastShapeIndex ) < shapes_.size() );
        assert( static_cast<std::size_t>( otherShapeIndex ) < shapes_.size() );

        if( fastShapeIndex == otherShapeIndex ) return;

        const shape& fastShape = shapes_[fastShapeIndex];

        const shape& otherShape = shapes_[otherShapeIndex];

        if( otherShape.bodyId == shape::NULL_INDEX || otherShape.bodyId == bodyIndex ) return;

        if( !shouldShapeFiltersCollide( fastShape.filter, otherShape.filter ) ) return;

        // Box2D continuous candidate는 sensor도 Body/Joint filter를 통과해야 함.
        // Discrete sensor overlap은 UpdateSensors의 shape filter만 사용함.
        if( !shouldBodiesCollide( bodyIndex, otherShape.bodyId ) ) return;

        const bool isSensor = otherShape.sensorIndex != shape::NULL_INDEX;

        if( isSensor && ( !otherShape.enableSensorEvents || !fastShape.enableSensorEvents ) ) return;

        if( !isSensor && !CanCollideShapes( fastShape.geometry, otherShape.geometry ) ) return;

        const body& otherBody = bodies_[otherShape.bodyId];

        const bodySim& otherSim = bodySims_[otherShape.bodyId];

        if( fastSim.isBullet && otherSim.isBullet ) return;

        const bodyState& otherState = bodyStates_[otherShape.bodyId];

        const bool otherMoves = otherBody.type != bodyType::Static;

        const sweep2 otherSweep{ otherSim.localCenter, otherSim.center, otherSim.center + ( otherMoves ? otherState.deltaPosition : vec2{} ), otherSim.transform.rotation, otherMoves ? otherState.deltaRotation * otherSim.transform.rotation : otherSim.transform.rotation };

        toiInput2 input{};
        input.proxyA = MakeShapeProxy( otherShape.geometry );

        input.proxyB = MakeShapeProxy( fastShape.geometry );

        input.sweepA = otherSweep;
        input.sweepB = fastSweep;
        input.maxFraction = hitFraction;

        const toiOutput2 output = TimeOfImpact( input );

        if( isSensor )
        {
            // Sensor는 fast body의 motion을 자르지 않음.
            // 현재 solid TOI보다 앞선 crossing 후보만 작은 fixed budget 안에서 보관함.
            if( output.fraction <= hitFraction && sensorHits.size() < static_cast<std::size_t>( MAX_CONTINUOUS_SENSOR_HITS ) )
            {
                sensorHits.push_back( { otherShapeIndex, { fastShapeIndex, fastShape.generation }, output.fraction } );
            }

            return;
        }

        // fraction 0은 이미 존재하던 overlap / touching Contact가 처리함.
        if( output.state == toiState2::Hit && output.fraction > 0.0f && output.fraction < hitFraction )
        {
            hitFraction = output.fraction;
        }
    };

    std::int32_t fastShapeIndex = fastBody.headShapeId;

    while( fastShapeIndex != body::NULL_INDEX )
    {
        const shape& fastShape = shapes_[fastShapeIndex];

        const std::int32_t nextShapeIndex = fastShape.nextShapeId;

        if( fastShape.sensorIndex == shape::NULL_INDEX )
        {
            // 일반 fast body와 bullet 모두 static tree는 broad-phase query로 좁힘.
            const dynamicTree& staticTree = broadPhase_.GetTree( bodyType::Static );

            staticTree.Query( sweptAABB,
                [&]( std::int32_t proxyId )
                {
                    testCandidate( fastShapeIndex, staticTree.GetProxyShapeIndex( proxyId ) );

                    return true;
                } );

            if( fastSim.isBullet )
            {
                const auto queryMovingTree = [&]( bodyType type )
                {
                    const dynamicTree& tree = broadPhase_.GetTree( type );

                    tree.Query( sweptAABB,
                        [&]( std::int32_t proxyId )
                        {
                            testCandidate( fastShapeIndex, tree.GetProxyShapeIndex( proxyId ) );

                            return true;
                        } );
                };

                queryMovingTree( bodyType::Kinematic );

                queryMovingTree( bodyType::Dynamic );
            }
        }

        fastShapeIndex = nextShapeIndex;
    }

    // 최종 solid hit보다 먼저 지나간 Sensor만 publish함.
    // solid 충돌 뒤쪽의 Sensor는 실제 body가 거기까지 도달하지 않으므로 버림.
    for( const continuousSensorHit2& sensorHit : sensorHits )
    {
        if( sensorHit.fraction >= hitFraction ) continue;

        assert( sensorHit.sensorShapeIndex >= 0 );
        assert( static_cast<std::size_t>( sensorHit.sensorShapeIndex ) < shapes_.size() );

        const shape& sensorShape = shapes_[sensorHit.sensorShapeIndex];

        assert( sensorShape.sensorIndex != shape::NULL_INDEX );

        assert( sensorShape.sensorIndex >= 0 );
        assert( static_cast<std::size_t>( sensorShape.sensorIndex ) < sensors_.size() );

        sensors_[sensorShape.sensorIndex].hits.push_back( sensorHit.visitor );
    }

    if( hitFraction >= 1.0f ) return;

    const transform2 clippedTransform = GetSweepTransform( fastSweep, hitFraction );

    const vec2 clippedCenter = fastSweep.c1 * ( 1.0f - hitFraction ) + fastSweep.c2 * hitFraction;

    fastState.deltaPosition = clippedCenter - fastSim.center;

    fastState.deltaRotation = clippedTransform.rotation * Inverse( fastSim.transform.rotation );

    fastSim.hadTimeOfImpact = true;

    // 이동 시간을 잃은 만큼 이번 Step에서 미리 더해졌던 gravity 성분을 되돌림.
    // Box2D와 같이 다른 force / torque의 time loss는 아직 보정하지 않음.
    const float timeLoss = ( 1.0f - hitFraction ) * timeStep;

    fastState.linearVelocity -= gravity_ * fastSim.gravityScale * timeLoss;
}

#pragma endregion ContinuousCollision

#pragma region ContactUpdate

bool world::updateExistingContact( std::int32_t contactId )
{
    contact2& contact = contacts_[contactId];

    // free-list slot에는 이번 update에서 갱신하거나 알릴 Contact가 없음.
    if( contact.contactId == contact2::NULL_INDEX ) return false;

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

    // Box2D처럼 fat bounds가 분리되기 전까지 zero-point pair도 persistent Contact로 유지함.
    if( !Overlaps( fatAABBs_[contact.shapeIdA], fatAABBs_[contact.shapeIdB] ) )
    {
        DestroyContact( contactId );

        return false;
    }

    assert( shapeA.bodyId >= 0 );
    assert( shapeB.bodyId >= 0 );
    assert( static_cast<std::size_t>( shapeA.bodyId ) < bodies_.size() );
    assert( static_cast<std::size_t>( shapeB.bodyId ) < bodies_.size() );

    const bodySim& bodySimA = bodySims_[shapeA.bodyId];
    const bodySim& bodySimB = bodySims_[shapeB.bodyId];

    assert( bodySimA.bodyId == shapeA.bodyId );
    assert( bodySimB.bodyId == shapeB.bodyId );

    if( !TryRecycleContact( contactId ) )
    {
        const localManifold2 manifold = CollideShapes( shapeA.geometry, bodySimA.transform, shapeB.geometry, bodySimB.transform );
        UpdateContactSim( contactId, manifold );
    }

    // Contact query의 speculative point 포함 규칙과 실제 touching callback을 구분함.
    return IsTouchingManifold( contactSims_[contactId].manifold );
}

std::int32_t world::createContactForPair( std::int32_t shapeIdA, std::int32_t shapeIdB )
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

    if( !shouldShapeFiltersCollide( shapeA.filter, shapeB.filter ) || !shouldBodiesCollide( shapeA.bodyId, shapeB.bodyId ) || !CanCollideShapes( shapeA.geometry, shapeB.geometry ) ) return contact2::NULL_INDEX;

    const bodySim& bodySimA = bodySims_[shapeA.bodyId];
    const bodySim& bodySimB = bodySims_[shapeB.bodyId];

    assert( bodySimA.bodyId == shapeA.bodyId );
    assert( bodySimB.bodyId == shapeB.bodyId );

    const localManifold2 manifold = CollideShapes( shapeA.geometry, bodySimA.transform, shapeB.geometry, bodySimB.transform );

    return CreateContact( shapeIdA, shapeIdB, manifold );
}

std::int32_t world::CreateContact( std::int32_t shapeIdA, std::int32_t shapeIdB, const localManifold2& manifold )
{
    assert( shapeIdA >= 0 );
    assert( shapeIdB >= 0 );
    assert( shapeIdA != shapeIdB );
    assert( static_cast<std::size_t>( shapeIdA ) < shapes_.size() );
    assert( static_cast<std::size_t>( shapeIdB ) < shapes_.size() );

    std::int32_t contactId = contact2::NULL_INDEX;

    if( contactFreeList_ != contact2::NULL_INDEX )
    {
        contactId = contactFreeList_;

        assert( contactId >= 0 );
        assert( static_cast<std::size_t>( contactId ) < contacts_.size() );

        contact2& freeContact = contacts_[contactId];

        assert( freeContact.contactId == contact2::NULL_INDEX );
        assert( contactSims_.size() == contacts_.size() );
        assert( contactSims_[contactId].contactId == contactSim2::NULL_INDEX );

        contactFreeList_ = freeContact.nextFreeId;
    }
    else
    {
        assert( contacts_.size() < static_cast<std::size_t>( std::numeric_limits<std::int32_t>::max() ) );

        contactId = static_cast<std::int32_t>( contacts_.size() );
        contacts_.push_back( {} );
        contactSims_.push_back( {} );
    }

    contact2& contact = contacts_[contactId];

    const std::uint32_t generation = contact.generation + 1u;

    contact = {};
    contact.contactId = contactId;
    contact.generation = generation;
    contact.shapeIdA = shapeIdA;
    contact.shapeIdB = shapeIdB;

    const shape& shapeA = shapes_[shapeIdA];

    const shape& shapeB = shapes_[shapeIdB];

    assert( shapeA.bodyId >= 0 );
    assert( shapeB.bodyId >= 0 );
    assert( static_cast<std::size_t>( shapeA.bodyId ) < bodySims_.size() );
    assert( static_cast<std::size_t>( shapeB.bodyId ) < bodySims_.size() );

    // 최신 Box2D처럼 Contact가 만들어질 때 두 body의 설정을 snapshot함.
    // 이후 body 설정을 바꿔도 기존 Contact에는 영향을 주지 않음.
    contact.enableRecycling = bodySims_[shapeA.bodyId].enableContactRecycling && bodySims_[shapeB.bodyId].enableContactRecycling;

    assert( contactSims_.size() == contacts_.size() );

    UpdateContactSim( contactId, manifold );

    const std::array<std::int32_t, 2> shapeIds =
    {
        shapeIdA,
        shapeIdB
    };

    for( std::int32_t edgeIndex = 0; edgeIndex < 2; ++edgeIndex )
    {
        const shape& shape = shapes_[shapeIds[edgeIndex]];

        assert( shape.bodyId >= 0 );
        assert( static_cast<std::size_t>( shape.bodyId ) < bodies_.size() );

        body& body = bodies_[shape.bodyId];
        contactEdge2& edge = contact.edges[edgeIndex];

        edge.bodyId = shape.bodyId;
        edge.prevKey = contact2::NULL_INDEX;
        edge.nextKey = body.headContactKey;

        const std::int32_t contactKey = MakeContactKey( contactId, edgeIndex );

        if( body.headContactKey != body::NULL_INDEX )
        {
            const std::int32_t headContactId = GetContactId( body.headContactKey );
            const std::int32_t headEdgeIndex = GetContactEdgeIndex( body.headContactKey );

            assert( headContactId >= 0 );
            assert( static_cast<std::size_t>( headContactId ) < contacts_.size() );

            contact2& headContact = contacts_[headContactId];

            assert( headContact.contactId == headContactId );

            headContact.edges[headEdgeIndex].prevKey = contactKey;
        }

        body.headContactKey = contactKey;
        ++body.contactCount;
    }

    const shapePairKey pairKey = MakeShapePairKey( shapeIdA, shapeIdB );

    // hashSet::Add는 새 key면 false, 이미 존재하면 true를 반환함.
    const bool alreadyExists = broadPhase_.AddPair( pairKey );
    assert( !alreadyExists );

    ++contactCount_;

    if( manifold.pointCount > 0 )
    {
        WakeBodyByIndex( contact.edges[0].bodyId );
    }

    return contactId;
}

bool world::TryRecycleContact( std::int32_t contactId )
{
    // Box2D contact recycling 기준: 작은 상대 이동에서는 anchor를 유지해 jitter를 줄임.
    // https://github.com/erincatto/box2d/blob/ac7c751eaeddbabdc1c4d41ae4f3a25d78627790/src/physics_world.c#L530
    assert( contactId >= 0 );
    assert( static_cast<std::size_t>( contactId ) < contacts_.size() );
    assert( contactSims_.size() == contacts_.size() );

    const contact2& contact = contacts_[contactId];

    if( !contact.enableRecycling || contactRecycleDistance_ <= 0.0f ) return false;

    contactSim2& contactSim = contactSims_[contactId];

    if( !contactSim.recycleCacheValid ) return false;

    assert( contactSim.bodyIdA >= 0 );
    assert( contactSim.bodyIdB >= 0 );
    assert( static_cast<std::size_t>( contactSim.bodyIdA ) < bodySims_.size() );
    assert( static_cast<std::size_t>( contactSim.bodyIdB ) < bodySims_.size() );

    const bodySim& bodySimA = bodySims_[contactSim.bodyIdA];

    const bodySim& bodySimB = bodySims_[contactSim.bodyIdB];

    // fast body는 작은 transform 차이처럼 보여도 CCD 경로와 함께 움직일 수 있으므로
    // fresh narrowphase를 사용함.
    if( bodySimA.isFast || bodySimB.isFast ) return false;

    const transform2 currentRelativeTransform = InverseMul( bodySimA.transform, bodySimB.transform );

    const float cosA = bodySimA.transform.rotation.c * contactSim.cachedRotationA.c + bodySimA.transform.rotation.s * contactSim.cachedRotationA.s;

    const float cosB = bodySimB.transform.rotation.c * contactSim.cachedRotationB.c + bodySimB.transform.rotation.s * contactSim.cachedRotationB.s;

    if( std::min( cosA, cosB ) <= CONTACT_RECYCLE_COS_ANGLE ) return false;

    const vec2 relativeTranslation = currentRelativeTransform.position - contactSim.cachedRelativeTransform.position;

    const rot2 relativeRotation = Inverse( currentRelativeTransform.rotation ) * contactSim.cachedRelativeTransform.rotation;

    const float maxExtentA = bodies_[contactSim.bodyIdA].type == bodyType::Static ? 0.0f : bodySimA.maxExtent;

    const float maxExtentB = bodies_[contactSim.bodyIdB].type == bodyType::Static ? 0.0f : bodySimB.maxExtent;

    const float maxExtent = std::max( maxExtentA, maxExtentB );

    // Box2D conservative advancement metric. 작은 회전에서는 sin(theta) ~= theta임.
    // cached pose는 fresh narrowphase에서만 갱신하므로 작은 이동이 반복돼도 누적 오차를 제한함.
    const float relativeMotion = Length( relativeTranslation ) + maxExtent * std::fabs( relativeRotation.s );

    // non-touching Contact는 새 speculative point를 놓치지 않도록 더 엄격하게 검사함.
    const float tolerance = contactSim.manifold.pointCount > 0 ? contactRecycleDistance_ : std::min( contactRecycleDistance_, SPECULATIVE_DISTANCE );

    if( relativeMotion >= tolerance ) return false;

    // mass data는 shape density 변경 등으로 Contact 수명 중에도 바뀔 수 있으므로
    // narrowphase를 건너뛰더라도 최신 값을 반영함.
    contactSim.invMassA = bodySimA.invMass;

    contactSim.invInertiaA = bodySimA.invInertia;

    contactSim.invMassB = bodySimB.invMass;

    contactSim.invInertiaB = bodySimB.invInertia;

    const vec2 worldNormal = TransformVector( bodySimA.transform, contactSim.manifold.normal );

    for( int i = 0; i < contactSim.manifold.pointCount; ++i )
    {
        const vec2 worldPointA = TransformPoint( bodySimA.transform, contactSim.recyclePointA[i] );

        const vec2 worldPointB = TransformPoint( bodySimB.transform, contactSim.recyclePointB[i] );

        const vec2 worldPoint = ( worldPointA + worldPointB ) * 0.5f;

        localManifoldPoint2& point = contactSim.manifold.points[i];

        // public/debug contact point도 현재 두 cached anchor의 중간으로 갱신함.
        point.point = InverseTransformPoint( bodySimA.transform, worldPoint );

        point.separation = contactSim.recycleSeparation[i] + Dot( worldPointB - worldPointA, worldNormal );
    }

    ++recycledContactCount_;

    return true;
}

void world::UpdateContactSim( std::int32_t contactId, const localManifold2& manifold )
{
    assert( contactId >= 0 );
    assert( static_cast<std::size_t>( contactId ) < contacts_.size() );
    assert( contactSims_.size() == contacts_.size() );

    const contact2& contact = contacts_[contactId];

    assert( contact.contactId == contactId );
    assert( contact.shapeIdA >= 0 );
    assert( contact.shapeIdB >= 0 );
    assert( static_cast<std::size_t>( contact.shapeIdA ) < shapes_.size() );
    assert( static_cast<std::size_t>( contact.shapeIdB ) < shapes_.size() );

    const shape& shapeA = shapes_[contact.shapeIdA];
    const shape& shapeB = shapes_[contact.shapeIdB];

    assert( shapeA.bodyId >= 0 );
    assert( shapeB.bodyId >= 0 );
    assert( static_cast<std::size_t>( shapeA.bodyId ) < bodySims_.size() );
    assert( static_cast<std::size_t>( shapeB.bodyId ) < bodySims_.size() );

    const bodySim& bodySimA = bodySims_[shapeA.bodyId];
    const bodySim& bodySimB = bodySims_[shapeB.bodyId];

    assert( bodySimA.bodyId == shapeA.bodyId );
    assert( bodySimB.bodyId == shapeB.bodyId );

    contactSim2& contactSim = contactSims_[contactId];

    // narrow-phase가 새 manifold를 만들기 전에 이전 point id와
    // solver impulse를 보존해 같은 접촉점에 다시 연결함.
    const localManifold2 oldManifold = contactSim.manifold;

    const auto oldImpulses = contactSim.impulses;

    contactSim = {};
    contactSim.contactId = contactId;

    contactSim.bodyIdA = shapeA.bodyId;
    contactSim.bodyIdB = shapeB.bodyId;

    contactSim.shapeIdA = contact.shapeIdA;
    contactSim.shapeIdB = contact.shapeIdB;

    contactSim.invMassA = bodySimA.invMass;
    contactSim.invInertiaA = bodySimA.invInertia;

    contactSim.invMassB = bodySimB.invMass;
    contactSim.invInertiaB = bodySimB.invInertia;

    contactSim.manifold = manifold;

    /*
    * Contact point persistence
    *
    * 새 manifold의 point id가 이전 manifold의 id와 같으면
    * 같은 geometric feature에서 만들어진 접촉점으로 간주함.
    *
    * 이 경우 이전 step의 누적 impulse를 그대로 이어받아
    * 다음 solver의 warm start 초기값으로 사용함.
    */
    for( int i = 0; i < contactSim.manifold.pointCount; ++i )
    {
        const std::uint16_t newId = contactSim.manifold.points[i].id;

        for( int j = 0; j < oldManifold.pointCount; ++j )
        {
            if( oldManifold.points[j].id == newId )
            {
                contactSim.impulses[i] = oldImpulses[j];

                break;
            }
        }
    }

    // fresh narrowphase 결과를 다음 recycling의 기준 pose / anchor로 저장함.
    contactSim.cachedRotationA = bodySimA.transform.rotation;

    contactSim.cachedRotationB = bodySimB.transform.rotation;

    contactSim.cachedRelativeTransform = InverseMul( bodySimA.transform, bodySimB.transform );

    for( int i = 0; i < contactSim.manifold.pointCount; ++i )
    {
        const localManifoldPoint2& point = contactSim.manifold.points[i];

        const vec2 worldPoint = TransformPoint( bodySimA.transform, point.point );

        contactSim.recyclePointA[i] = point.point;

        contactSim.recyclePointB[i] = InverseTransformPoint( bodySimB.transform, worldPoint );

        contactSim.recycleSeparation[i] = point.separation;
    }

    contactSim.recycleCacheValid = true;

    const bool wasSolverActive = oldManifold.pointCount > 0;

    const bool isSolverActive = contactSim.manifold.pointCount > 0;

    if( wasSolverActive != isSolverActive )
    {
        WakeBodyByIndex( contactSim.bodyIdA );
        WakeBodyByIndex( contactSim.bodyIdB );
    }
}

#pragma endregion ContactUpdate

#pragma region JointSolver

void world::warmStartJoints( std::span<jointConstraint> constraints )
{
    for( const jointConstraint& value : constraints )
    {
        std::visit(
            [&]( const auto& constraint )
            {
                if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, distanceJointConstraint2> )
                {
                    warmStartDistanceJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB] );
                }
                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, prismaticJointConstraint2> )
                {
                    warmStartPrismaticJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB] );
                }
                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, revoluteJointConstraint2> )
                {
                    warmStartRevoluteJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB] );
                }
                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, wheelJointConstraint2> )
                {
                    warmStartWheelJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB] );
                }
                else
                {
                    warmStartMouseJointConstraint( constraint, bodyStates_[constraint.bodyIdB] );
                }
            },
            value );
    }
}

void world::solveJoints( std::span<jointConstraint> constraints, bool useBias )
{
    for( jointConstraint& value : constraints )
    {
        std::visit(
            [&]( auto& constraint )
            {
                if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, distanceJointConstraint2> )
                {
                    solveDistanceJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB], useBias );
                }
                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, prismaticJointConstraint2> )
                {
                    solvePrismaticJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB], useBias );
                }
                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, revoluteJointConstraint2> )
                {
                    solveRevoluteJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB], useBias );
                }
                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, wheelJointConstraint2> )
                {
                    solveWheelJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB], useBias );
                }
                else
                {
                    solveMouseJointConstraint( constraint, bodyStates_[constraint.bodyIdB] );
                }
            },
            value );
    }
}

#pragma endregion JointSolver

#pragma region ContactSolver

std::vector<contactConstraint2> world::PrepareContactConstraints( std::span<const std::int32_t> contactIds, float timeStep )
{
    assert( std::isfinite( timeStep ) );
    assert( timeStep > 0.0f );

    /*
    * 현재 Box2D contact softness의 기본값을 시작점으로 사용함.
    * 아직 WorldDef / solver tuning API가 없으므로 내부 상수로 둠.
    *
    * 큰 timeStep에서는 Hertz를 낮춰 한 step에서 지나치게 강한
    * correction을 만들지 않도록 제한함.
    */
    constexpr float CONTACT_HERTZ = 30.0f;
    constexpr float CONTACT_DAMPING_RATIO = 10.0f;
    constexpr float MAX_CONTACT_PUSH_SPEED = 3.0f;

    const float invTimeStep = 1.0f / timeStep;

    const float contactHertz = std::min( CONTACT_HERTZ, 0.125f * invTimeStep );

    const contactSoftness2 contactSoftness = MakeContactSoftness( contactHertz, CONTACT_DAMPING_RATIO, timeStep );

    // Static contact는 지면을 뚫고 내려가는 현상을 줄이기 위해
    // 일반 dynamic contact보다 조금 더 단단하게 풂.
    const contactSoftness2 staticSoftness = MakeContactSoftness( 2.0f * contactHertz, CONTACT_DAMPING_RATIO, timeStep );

    std::vector<contactConstraint2> constraints;
    constraints.reserve( contactIds.size() );

    for( const std::int32_t contactId : contactIds )
    {
        assert( contactId >= 0 );
        assert( static_cast<std::size_t>( contactId ) < contactSims_.size() );

        const contactSim2& contactSim = contactSims_[contactId];

        assert( contactSim.contactId == contactId );
        assert( contactSim.manifold.pointCount > 0 );

        assert( contactSim.bodyIdA >= 0 );
        assert( contactSim.bodyIdB >= 0 );
        assert( static_cast<std::size_t>( contactSim.bodyIdA ) < bodies_.size() );
        assert( static_cast<std::size_t>( contactSim.bodyIdB ) < bodies_.size() );

        // 같은 body의 shape끼리는 Contact를 만들지 않아야 함.
        assert( contactSim.bodyIdA != contactSim.bodyIdB );

        const bodySim& bodySimA = bodySims_[contactSim.bodyIdA];
        const bodySim& bodySimB = bodySims_[contactSim.bodyIdB];

        const bodyState& bodyStateA = bodyStates_[contactSim.bodyIdA];
        const bodyState& bodyStateB = bodyStates_[contactSim.bodyIdB];

        contactConstraint2 constraint =
            PrepareContactConstraint(
                contactSim,
                bodySimA, bodyStateA,
                bodySimB, bodyStateB
            );

        const bool hasStaticBody = bodies_[contactSim.bodyIdA].type == bodyType::Static || bodies_[contactSim.bodyIdB].type == bodyType::Static;

        constraint.softness = hasStaticBody ? staticSoftness : contactSoftness;

        constraint.maxPushSpeed = MAX_CONTACT_PUSH_SPEED;

        constraint.invTimeStep = invTimeStep;

        assert( contactSim.shapeIdA >= 0 );
        assert( contactSim.shapeIdB >= 0 );
        assert( static_cast<std::size_t>( contactSim.shapeIdA ) < shapes_.size() );
        assert( static_cast<std::size_t>( contactSim.shapeIdB ) < shapes_.size() );

        const shape& shapeA = shapes_[contactSim.shapeIdA];

        const shape& shapeB = shapes_[contactSim.shapeIdB];

        // Box2D 기본 mixing rule.
        constraint.friction = std::sqrt( shapeA.friction * shapeB.friction );
        constraint.restitution = std::max( shapeA.restitution, shapeB.restitution );

        constraints.push_back( constraint );
    }

    return constraints;
}

void world::WarmStartContacts( std::span<contactConstraint2> constraints )
{
    for( contactConstraint2& constraint : constraints )
    {
        assert( constraint.bodyIdA >= 0 );
        assert( constraint.bodyIdB >= 0 );

        WarmStartContactConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB] );
    }
}

void world::SolveContactConstraints( std::span<contactConstraint2> constraints, bool useBias )
{
    constexpr int VELOCITY_ITERATIONS = 8;

    /*
    * 접점 하나를 풀 때 속도가 즉시 바뀌고
    * 다음 접점이 그 결과를 사용하므로 sequential impulse가 됨.
    *
    * useBias=true:
    *     penetration correction을 포함한 soft push
    *
    * useBias=false:
    *     position 적분 뒤 correction velocity를 제거하는 rigid relax
    *     + tangent 방향 Coulomb friction
    */
    for( int iteration = 0; iteration < VELOCITY_ITERATIONS; ++iteration )
    {
        for( contactConstraint2& constraint : constraints )
        {
            assert( constraint.bodyIdA >= 0 );
            assert( constraint.bodyIdB >= 0 );

            SolveContactConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB], useBias );
        }
    }
}

void world::ApplyRestitutionContacts( std::span<contactConstraint2> constraints )
{
    constexpr float RESTITUTION_THRESHOLD = 1.0f;
    // Box2D처럼 두 접점의 순차 impulse가 서로 영향을 주므로 restitution을 반복함.
    constexpr int RESTITUTION_ITERATIONS = 2;

    /*
    * normal solve가 이미 penetration / 접근 속도를 처리한 뒤
    * 충돌 전 relativeNormalVelocity와 이번 step의 compression impulse를 기준으로
    * 실제 충돌한 point에만 bounce를 별도 적용함.
    *
    * 낮은 속도 접촉은 threshold 아래에서 restitution을 끄고,
    * restitutionImpulse를 따로 추적해 반복 solve의 에너지 증가를 제한함.
    */
    for( int iteration = 0; iteration < RESTITUTION_ITERATIONS; ++iteration )
    {
        for( contactConstraint2& constraint : constraints )
        {
            if( constraint.restitution <= 0.0f ) continue;

            assert( constraint.bodyIdA >= 0 );
            assert( constraint.bodyIdB >= 0 );

            ApplyRestitutionContactConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB], RESTITUTION_THRESHOLD );
        }
    }
}

void world::StoreContactConstraintImpulses( std::span<const contactConstraint2> constraints )
{
    for( const contactConstraint2& constraint : constraints )
    {
        assert( constraint.contactId >= 0 );
        assert( static_cast<std::size_t>( constraint.contactId ) < contactSims_.size() );

        StoreContactImpulses( constraint, contactSims_[constraint.contactId] );
    }
}

#pragma endregion ContactSolver

#pragma region ContactDestruction

void world::DestroyContact( std::int32_t contactId )
{
    assert( contactId >= 0 );
    assert( static_cast<std::size_t>( contactId ) < contacts_.size() );

    contact2& contact = contacts_[contactId];

    assert( contact.contactId == contactId );
    assert( contactCount_ > 0 );

    const shapePairKey pairKey = MakeShapePairKey( contact.shapeIdA, contact.shapeIdB );

    WakeBodyByIndex( contact.edges[0].bodyId );

    WakeBodyByIndex( contact.edges[1].bodyId );

    for( std::int32_t edgeIndex = 0; edgeIndex < 2; ++edgeIndex )
    {
        const contactEdge2 edge = contact.edges[edgeIndex];

        assert( edge.bodyId >= 0 );
        assert( static_cast<std::size_t>( edge.bodyId ) < bodies_.size() );

        body& body = bodies_[edge.bodyId];
        const std::int32_t contactKey = MakeContactKey( contactId, edgeIndex );

        if( edge.prevKey != contact2::NULL_INDEX )
        {
            contact2& previous = contacts_[GetContactId( edge.prevKey )];

            previous.edges[GetContactEdgeIndex( edge.prevKey )].nextKey = edge.nextKey;
        }

        if( edge.nextKey != contact2::NULL_INDEX )
        {
            contact2& next = contacts_[GetContactId( edge.nextKey )];

            next.edges[GetContactEdgeIndex( edge.nextKey )].prevKey = edge.prevKey;
        }

        if( body.headContactKey == contactKey )
        {
            body.headContactKey = edge.nextKey;
        }

        assert( body.contactCount > 0 );

        --body.contactCount;
    }

    const bool removed = broadPhase_.RemovePair( pairKey );
    assert( removed );

    assert( contactSims_.size() == contacts_.size() );

    contactSims_[contactId] = {};

    const std::uint32_t generation = contact.generation;

    contact = {};
    contact.generation = generation;
    contact.nextFreeId = contactFreeList_;
    contactFreeList_ = contactId;

    --contactCount_;
}

#pragma endregion ContactDestruction

} // namespace zonai
