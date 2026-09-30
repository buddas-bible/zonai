#include "dynamics/world.h"

#include <cassert>
#include <cmath>
#include <limits>
#include <utility>

#include "dynamics/bodyShape.h"

namespace zonai
{

bool World::IsValid( BodyId bodyId ) const noexcept
{
    if( bodyId.index1 <= 0 )
    {
        return false;
    }

    const std::int32_t bodyIndex = bodyId.index1 - 1;

    if( static_cast<std::size_t>( bodyIndex ) >= bodies_.size() )
    {
        return false;
    }

    const Body& body = bodies_[bodyIndex];

    return
        body.bodyId == bodyIndex &&
        body.generation == bodyId.generation;
}

bool World::IsValid( ShapeId shapeId ) const noexcept
{
    if( shapeId.index1 <= 0 )
    {
        return false;
    }

    const std::int32_t shapeIndex = shapeId.index1 - 1;

    if( static_cast<std::size_t>( shapeIndex ) >= shapes_.size() )
    {
        return false;
    }

    const Shape& shape = shapes_[shapeIndex];

    return
        shape.bodyId != Shape::NULL_INDEX &&
        shape.generation == shapeId.generation;
}

bool World::IsValid( ContactId contactId ) const noexcept
{
    if( contactId.index1 <= 0 )
    {
        return false;
    }

    const std::int32_t contactIndex = contactId.index1 - 1;

    if( static_cast<std::size_t>( contactIndex ) >= contacts_.size() )
    {
        return false;
    }

    const contact2& contact = contacts_[contactIndex];

    return
        contact.contactId == contactIndex &&
        contact.generation == contactId.generation;
}


std::int32_t World::GetBodyIndex( BodyId bodyId ) const
{
    assert( IsValid( bodyId ) );
    return bodyId.index1 - 1;
}

std::int32_t World::GetShapeIndex( ShapeId shapeId ) const
{
    assert( IsValid( shapeId ) );
    return shapeId.index1 - 1;
}

std::int32_t World::GetContactIndex( ContactId contactId ) const
{
    assert( IsValid( contactId ) );
    return contactId.index1 - 1;
}


BodyId World::MakeBodyId( std::int32_t bodyIndex ) const
{
    assert( bodyIndex >= 0 );
    assert( static_cast<std::size_t>( bodyIndex ) < bodies_.size() );

    const Body& body = bodies_[bodyIndex];

    assert( body.bodyId == bodyIndex );

    return { bodyIndex + 1, body.generation };
}

ShapeId World::MakeShapeId( std::int32_t shapeIndex ) const
{
    assert( shapeIndex >= 0 );
    assert( static_cast<std::size_t>( shapeIndex ) < shapes_.size() );

    const Shape& shape = shapes_[shapeIndex];

    assert( shape.bodyId != Shape::NULL_INDEX );

    return { shapeIndex + 1, shape.generation };
}

ContactId World::MakeContactId( std::int32_t contactIndex ) const
{
    assert( contactIndex >= 0 );
    assert( static_cast<std::size_t>( contactIndex ) < contacts_.size() );

    const contact2& contact = contacts_[contactIndex];

    assert( contact.contactId == contactIndex );

    return { contactIndex + 1, contact.generation };
}


BodyId World::CreateBody( BodyType type, transform2 transform )
{
    std::int32_t bodyIndex = Body::NULL_INDEX;

    if( bodyFreeList_ != Body::NULL_INDEX )
    {
        bodyIndex = bodyFreeList_;

        assert( bodyIndex >= 0 );
        assert( static_cast<std::size_t>( bodyIndex ) < bodies_.size() );

        Body& freeBody = bodies_[bodyIndex];
        assert( freeBody.bodyId == Body::NULL_INDEX );
        assert( freeBody.headShapeId == Body::NULL_INDEX );
        assert( freeBody.headContactKey == Body::NULL_INDEX );
        assert( bodySims_.size() == bodies_.size() );
        assert( bodyStates_.size() == bodies_.size() );
        assert( bodySims_[bodyIndex].bodyId == BodySim::NULL_INDEX );

        bodyFreeList_ = freeBody.nextFreeId;
    }
    else
    {
        assert(
            bodies_.size() <
            static_cast<std::size_t>( std::numeric_limits<std::int32_t>::max() - 1 )
        );

        bodyIndex = static_cast<std::int32_t>( bodies_.size() );
        bodies_.push_back( {} );
        bodySims_.push_back( {} );
        bodyStates_.push_back( {} );
    }

    Body& body = bodies_[bodyIndex];

    const std::uint16_t generation = static_cast<std::uint16_t>( body.generation + 1u );

    body = {};
    body.bodyId = bodyIndex;
    body.generation = generation;
    body.type = type;

    assert( bodySims_.size() == bodies_.size() );

    BodySim& bodySim = bodySims_[bodyIndex];
    bodySim = {};
    bodySim.bodyId = bodyIndex;
    bodySim.transform = transform;
    bodySim.center = transform.position;

    assert( bodyStates_.size() == bodies_.size() );
    bodyStates_[bodyIndex] = {};

    ++bodyCount_;

    return MakeBodyId( bodyIndex );
}

void World::DestroyBody( BodyId bodyId )
{
    DestroyBodyByIndex( GetBodyIndex( bodyId ) );
}

void World::DestroyBodyByIndex( std::int32_t bodyIndex )
{
    assert( bodyIndex >= 0 );
    assert( static_cast<std::size_t>( bodyIndex ) < bodies_.size() );
    assert( bodyCount_ > 0 );

    Body& body = bodies_[bodyIndex];

    assert( body.bodyId == bodyIndex );

    // Box2D처럼 먼저 이 Body에 연결된 모든 Contact를 제거함.
    while( body.headContactKey != Body::NULL_INDEX )
    {
        const std::int32_t contactId = GetContactId( body.headContactKey );

        assert( contactId >= 0 );
        assert( static_cast<std::size_t>( contactId ) < contacts_.size() );

        DestroyContact( contactId );
    }

    assert( body.contactCount == 0 );

    // Shape를 하나씩 제거하면 각 proxy와 Shape slot도 함께 정리됨.
    while( body.headShapeId != Body::NULL_INDEX )
    {
        DestroyShapeByIndex( body.headShapeId );
    }

    assert( body.shapeCount == 0 );
    assert( body.headShapeId == Body::NULL_INDEX );
    assert( body.headContactKey == Body::NULL_INDEX );

    const std::uint16_t generation = body.generation;

    assert( bodySims_.size() == bodies_.size() );
    BodySim& bodySim = bodySims_[bodyIndex];
    assert( bodySim.bodyId == bodyIndex );

    // simulation 데이터도 함께 비워 재사용 slot에 이전 transform이 남지 않게 함.
    bodySim = {};

    assert( bodyStates_.size() == bodies_.size() );

    // 운동 상태도 함께 초기화해 재사용 slot에 이전 Body 속도가 남지 않게 함.
    bodyStates_[bodyIndex] = {};

    // generation은 보존하고 slot만 free-list에 반환함.
    body = {};
    body.generation = generation;
    body.nextFreeId = bodyFreeList_;
    bodyFreeList_ = bodyIndex;

    --bodyCount_;
}


ShapeId World::CreateShape( BodyId bodyId, ShapeGeometry geometry, Filter filter, float density )
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    assert( !std::holds_alternative<std::monostate>( geometry ) );
    assert( std::isfinite( density ) );
    assert( density >= 0.0f );

    std::int32_t shapeIndex = Shape::NULL_INDEX;

    if( shapeFreeList_ != Shape::NULL_INDEX )
    {
        shapeIndex = shapeFreeList_;

        assert( shapeIndex >= 0 );
        assert( static_cast<std::size_t>( shapeIndex ) < shapes_.size() );

        Shape& freeShape = shapes_[shapeIndex];
        assert( freeShape.bodyId == Shape::NULL_INDEX );
        assert( std::holds_alternative<std::monostate>( freeShape.geometry ) );

        shapeFreeList_ = freeShape.nextFreeId;
    }
    else
    {
        assert(
            shapes_.size() <
            static_cast<std::size_t>( std::numeric_limits<std::int32_t>::max() - 1 )
        );

        shapeIndex = static_cast<std::int32_t>( shapes_.size() );
        shapes_.push_back( {} );
    }

    Shape& storedShape = shapes_[shapeIndex];

    const std::uint16_t generation =
        static_cast<std::uint16_t>( storedShape.generation + 1u );

    storedShape = {};
    storedShape.generation = generation;
    storedShape.geometry = std::move( geometry );
    storedShape.density = density;
    storedShape.filter = filter;

    Body& body = bodies_[bodyIndex];

    assert( bodySims_.size() == bodies_.size() );
    const BodySim& bodySim = bodySims_[bodyIndex];
    assert( bodySim.bodyId == bodyIndex );

    const aabb2 worldAABB =
        ComputeShapeAABB( storedShape.geometry, bodySim.transform );

    // Box2D의 기본 Shape 생성처럼 static Shape도 즉시 pair 탐색 대상이 되게 함.
    storedShape.proxyKey =
        broadPhase_.CreateProxy(
            body.type,
            worldAABB,
            shapeIndex,
            true
        );

    LinkShape( body, bodyIndex, shapes_, shapeIndex );
    ++shapeCount_;

    UpdateBodyMassData( bodyIndex );

    return MakeShapeId( shapeIndex );
}

void World::DestroyShape( ShapeId shapeId )
{
    DestroyShapeByIndex( GetShapeIndex( shapeId ) );
}

void World::SetShapeDensity( ShapeId shapeId, float density )
{
    assert( std::isfinite( density ) );
    assert( density >= 0.0f );

    const std::int32_t shapeIndex = GetShapeIndex( shapeId );
    Shape& shape = shapes_[shapeIndex];

    shape.density = density;

    assert( shape.bodyId != Shape::NULL_INDEX );
    UpdateBodyMassData( shape.bodyId );
}

float World::GetShapeDensity( ShapeId shapeId ) const
{
    return shapes_[GetShapeIndex( shapeId )].density;
}

void World::DestroyShapeByIndex( std::int32_t shapeIndex )
{
    assert( shapeIndex >= 0 );
    assert( static_cast<std::size_t>( shapeIndex ) < shapes_.size() );
    assert( shapeCount_ > 0 );

    Shape& shape = shapes_[shapeIndex];

    assert( shape.bodyId != Shape::NULL_INDEX );
    assert( shape.proxyKey != Shape::NULL_INDEX );
    assert( !std::holds_alternative<std::monostate>( shape.geometry ) );

    // Sensor 저장소는 아직 구현하지 않았으므로 현재는 일반 collision Shape만 제거함.
    assert( shape.sensorIndex == Shape::NULL_INDEX );

    const std::int32_t bodyIndex = shape.bodyId;
    Body& body = bodies_[bodyIndex];

    // Box2D처럼 먼저 Body의 Shape list에서 분리함.
    UnlinkShape( body, bodyIndex, shapes_, shapeIndex );

    // 더 이상 BroadPhase 후보가 되지 않도록 proxy를 제거함.
    broadPhase_.DestroyProxy( shape.proxyKey );
    shape.proxyKey = Shape::NULL_INDEX;

    // 이 Body의 Contact list에서 삭제 Shape가 관여한 Contact만 제거함.
    std::int32_t contactKey = body.headContactKey;

    while( contactKey != Body::NULL_INDEX )
    {
        const std::int32_t contactId = GetContactId( contactKey );
        const std::int32_t edgeIndex = GetContactEdgeIndex( contactKey );

        assert( contactId >= 0 );
        assert( static_cast<std::size_t>( contactId ) < contacts_.size() );

        contact2& contact = contacts_[contactId];
        assert( contact.contactId == contactId );

        // DestroyContact가 list를 수정하므로 다음 key를 먼저 저장함.
        contactKey = contact.edges[edgeIndex].nextKey;

        if( contact.shapeIdA == shapeIndex ||
            contact.shapeIdB == shapeIndex )
        {
            DestroyContact( contactId );
        }
    }

    UpdateBodyMassData( bodyIndex );

    const std::uint16_t generation = shape.generation;

    // generation은 보존하고 slot만 free-list에 반환함.
    shape = {};
    shape.generation = generation;
    shape.nextFreeId = shapeFreeList_;
    shapeFreeList_ = shapeIndex;

    --shapeCount_;
}


void World::SetBodyTransform( BodyId bodyId, transform2 transform )
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    // transform이 NaN / infinity를 포함하면 tree AABB까지 오염되므로 입구에서 차단함.
    assert( std::isfinite( transform.position.x ) );
    assert( std::isfinite( transform.position.y ) );
    assert( std::isfinite( transform.rotation.c ) );
    assert( std::isfinite( transform.rotation.s ) );

    assert( bodySims_.size() == bodies_.size() );

    BodySim& bodySim = bodySims_[bodyIndex];
    assert( bodySim.bodyId == bodyIndex );

    bodySim.transform = transform;
    bodySim.center = TransformPoint( bodySim.transform, bodySim.localCenter );

    SyncBodyProxies( bodyIndex );
}

void World::SyncBodyProxies( std::int32_t bodyIndex )
{
    assert( bodyIndex >= 0 );
    assert( static_cast<std::size_t>( bodyIndex ) < bodies_.size() );
    assert( bodySims_.size() == bodies_.size() );

    const Body& body = bodies_[bodyIndex];
    const BodySim& bodySim = bodySims_[bodyIndex];

    assert( body.bodyId == bodyIndex );
    assert( bodySim.bodyId == bodyIndex );

    std::int32_t shapeId = body.headShapeId;
    std::int32_t visitedCount = 0;

    while( shapeId != Body::NULL_INDEX )
    {
        assert( shapeId >= 0 );
        assert( static_cast<std::size_t>( shapeId ) < shapes_.size() );
        assert( visitedCount < body.shapeCount );

        Shape& shape = shapes_[shapeId];

        assert( shape.bodyId == bodyIndex );

        const aabb2 worldAABB = ComputeShapeAABB( shape.geometry, bodySim.transform );

        // disabled Body 개념이 들어오면 proxy가 없는 Shape는 그대로 건너뜀.
        if( shape.proxyKey != Shape::NULL_INDEX )
        {
            broadPhase_.MoveProxy( shape.proxyKey, worldAABB );
        }

        shapeId = shape.nextShapeId;
        ++visitedCount;
    }

    assert( visitedCount == body.shapeCount );
}

void World::UpdateBodyMassData( std::int32_t bodyIndex )
{
    assert( bodyIndex >= 0 );
    assert( static_cast<std::size_t>( bodyIndex ) < bodies_.size() );
    assert( bodySims_.size() == bodies_.size() );

    Body& body = bodies_[bodyIndex];
    BodySim& bodySim = bodySims_[bodyIndex];

    assert( body.bodyId == bodyIndex );
    assert( bodySim.bodyId == bodyIndex );

    /*
    * Body 질량 특성 계산
    *
    * 각 Shape i가 다음 값을 가진다고 하면:
    *
    *     mi : Shape 질량
    *     ci : Shape의 local center of mass
    *     Ii : ci를 지나는 z축 기준 Shape 회전 관성
    *
    * Body 전체 질량:
    *
    *     M = sum( mi )
    *
    * Body 전체 local center of mass:
    *
    *     C = sum( mi * ci ) / M
    *
    * Body center C 기준 회전 관성:
    *
    *     I = sum( Ii + mi * |ci - C|^2 )
    *
    * 마지막 식의 mi * |ci - C|^2가 평행축 정리로 추가되는 항임.
    */

    const vec2 oldCenter = bodySim.center;

    body.mass = 0.0f;
    body.inertia = 0.0f;

    bodySim.invMass = 0.0f;
    bodySim.invInertia = 0.0f;
    bodySim.localCenter = {};

    // Static / Kinematic Body는 외력이나 impulse로 가속되지 않으므로
    // solver 관점에서 무한 질량으로 취급하고 inverse mass / inertia를 0으로 둠.
    if( body.type != BodyType::Dynamic )
    {
        bodySim.center = bodySim.transform.position;
        return;
    }

    // 두 번째 관성 계산에서 Shape별 mass / center / inertia가 다시 필요하므로
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
    while( shapeIndex != Body::NULL_INDEX )
    {
        assert( shapeIndex >= 0 );
        assert( static_cast<std::size_t>( shapeIndex ) < shapes_.size() );
        assert( visitedCount < body.shapeCount );

        const Shape& shape = shapes_[shapeIndex];
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
        bodySim.localCenter =
            weightedCenter * bodySim.invMass;
    }

    /*
    * 두 번째 순회
    *
    * 각 Shape의 회전 관성 Ii는 자기 center ci 기준이므로
    * Body center C 기준으로 바로 더할 수 없음.
    *
    * 평행축 정리:
    *
    *     I_shifted = Ii + mi * d^2
    *
    *     d = |ci - C|
    *
    * 를 적용한 뒤 모든 Shape의 관성을 합산함.
    */
    for( const massData2& massData : masses )
    {
        if( massData.mass == 0.0f )
        {
            continue;
        }

        const vec2 offset =
            bodySim.localCenter - massData.center;

        body.inertia +=
            massData.rotationalInertia +
            massData.mass * LengthSquared( offset );
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

    // local center of mass C를 현재 Body transform으로 world space에 옮김.
    bodySim.center =
        TransformPoint(
            bodySim.transform,
            bodySim.localCenter
        );

    // center of mass가 이동해도 Body origin의 순간 속도가 갑자기 변하지 않도록
    // v_new = v_old + w x ( C_new - C_old ) 로 COM 선속도를 보정함.
    assert( bodyStates_.size() == bodies_.size() );

    BodyState& bodyState = bodyStates_[bodyIndex];
    bodyState.linearVelocity +=
        Cross( bodyState.angularVelocity,  bodySim.center - oldCenter );
}

const Body& World::GetBody( BodyId bodyId ) const
{
    return bodies_[GetBodyIndex( bodyId )];
}

transform2 World::GetBodyTransform( BodyId bodyId ) const
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    assert( bodySims_.size() == bodies_.size() );

    const BodySim& bodySim = bodySims_[bodyIndex];
    assert( bodySim.bodyId == bodyIndex );

    return bodySim.transform;
}

void World::SetBodyLinearVelocity( BodyId bodyId, vec2 linearVelocity )
{
    assert( std::isfinite( linearVelocity.x ) );
    assert( std::isfinite( linearVelocity.y ) );

    const std::int32_t bodyIndex = GetBodyIndex( bodyId );
    const Body& body = bodies_[bodyIndex];

    if( body.type == BodyType::Static )
    {
        // 정적 Body는 움직이지 않으므로 setter를 무시함.
        return;
    }

    assert( bodyStates_.size() == bodies_.size() );
    bodyStates_[bodyIndex].linearVelocity = linearVelocity;
}

vec2 World::GetBodyLinearVelocity( BodyId bodyId ) const
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    assert( bodyStates_.size() == bodies_.size() );

    // Static Body의 state도 항상 zero로 유지되므로 그대로 반환할 수 있음.
    return bodyStates_[bodyIndex].linearVelocity;
}

void World::SetBodyAngularVelocity( BodyId bodyId, float angularVelocity )
{
    assert( std::isfinite( angularVelocity ) );

    const std::int32_t bodyIndex = GetBodyIndex( bodyId );
    const Body& body = bodies_[bodyIndex];

    if( body.type == BodyType::Static )
    {
        // 정적 Body는 움직이지 않으므로 setter를 무시함.
        return;
    }

    assert( bodyStates_.size() == bodies_.size() );
    bodyStates_[bodyIndex].angularVelocity = angularVelocity;
}

float World::GetBodyAngularVelocity( BodyId bodyId ) const
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    assert( bodyStates_.size() == bodies_.size() );
    return bodyStates_[bodyIndex].angularVelocity;
}

float World::GetBodyMass( BodyId bodyId ) const
{
    return bodies_[GetBodyIndex( bodyId )].mass;
}

float World::GetBodyRotationalInertia( BodyId bodyId ) const
{
    return bodies_[GetBodyIndex( bodyId )].inertia;
}

vec2 World::GetBodyLocalCenter( BodyId bodyId ) const
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    assert( bodySims_.size() == bodies_.size() );
    return bodySims_[bodyIndex].localCenter;
}

void World::SetGravity( vec2 gravity )
{
    assert( std::isfinite( gravity.x ) );
    assert( std::isfinite( gravity.y ) );

    gravity_ = gravity;
}

vec2 World::GetGravity() const noexcept
{
    return gravity_;
}

void World::ApplyForce( BodyId bodyId, vec2 force, vec2 point )
{
    assert( std::isfinite( force.x ) );
    assert( std::isfinite( force.y ) );
    assert( std::isfinite( point.x ) );
    assert( std::isfinite( point.y ) );

    const std::int32_t bodyIndex = GetBodyIndex( bodyId );
    const Body& body = bodies_[bodyIndex];

    // Static / Kinematic Body는 외력으로 속도가 바뀌지 않음.
    if( body.type != BodyType::Dynamic )
    {
        return;
    }

    assert( bodySims_.size() == bodies_.size() );

    BodySim& bodySim = bodySims_[bodyIndex];

    // F_total += F
    bodySim.force += force;

    // center of mass 기준 torque:
    //
    //     tau = r x F
    //     r   = point - center
    bodySim.torque +=
        Cross(
            point - bodySim.center,
            force
        );
}

void World::ApplyForceToCenter( BodyId bodyId, vec2 force )
{
    assert( std::isfinite( force.x ) );
    assert( std::isfinite( force.y ) );

    const std::int32_t bodyIndex = GetBodyIndex( bodyId );
    const Body& body = bodies_[bodyIndex];

    if( body.type != BodyType::Dynamic )
    {
        return;
    }

    assert( bodySims_.size() == bodies_.size() );
    bodySims_[bodyIndex].force += force;
}

void World::ApplyTorque( BodyId bodyId, float torque )
{
    assert( std::isfinite( torque ) );

    const std::int32_t bodyIndex = GetBodyIndex( bodyId );
    const Body& body = bodies_[bodyIndex];

    if( body.type != BodyType::Dynamic )
    {
        return;
    }

    assert( bodySims_.size() == bodies_.size() );
    bodySims_[bodyIndex].torque += torque;
}

void World::ClearForces( BodyId bodyId )
{
    const std::int32_t bodyIndex = GetBodyIndex( bodyId );

    assert( bodySims_.size() == bodies_.size() );

    BodySim& bodySim = bodySims_[bodyIndex];
    bodySim.force = {};
    bodySim.torque = 0.0f;
}

void World::ApplyLinearImpulse( BodyId bodyId, vec2 impulse, vec2 point )
{
    assert( std::isfinite( impulse.x ) );
    assert( std::isfinite( impulse.y ) );
    assert( std::isfinite( point.x ) );
    assert( std::isfinite( point.y ) );

    const std::int32_t bodyIndex = GetBodyIndex( bodyId );
    const Body& body = bodies_[bodyIndex];

    // Static / Kinematic Body는 impulse로 속도가 바뀌지 않음.
    if( body.type != BodyType::Dynamic )
    {
        return;
    }

    assert( bodySims_.size() == bodies_.size() );
    assert( bodyStates_.size() == bodies_.size() );

    const BodySim& bodySim = bodySims_[bodyIndex];
    BodyState& bodyState = bodyStates_[bodyIndex];

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
    bodyState.linearVelocity +=
        impulse * bodySim.invMass;

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
    const vec2 r =
        point - bodySim.center;

    bodyState.angularVelocity +=
        bodySim.invInertia *
        Cross( r, impulse );
}

void World::ApplyLinearImpulseToCenter(
    BodyId bodyId,
    vec2 impulse )
{
    assert( std::isfinite( impulse.x ) );
    assert( std::isfinite( impulse.y ) );

    const std::int32_t bodyIndex = GetBodyIndex( bodyId );
    const Body& body = bodies_[bodyIndex];

    if( body.type != BodyType::Dynamic )
    {
        return;
    }

    assert( bodySims_.size() == bodies_.size() );
    assert( bodyStates_.size() == bodies_.size() );

    const BodySim& bodySim = bodySims_[bodyIndex];
    BodyState& bodyState = bodyStates_[bodyIndex];

    // center에 적용하므로 r=0이고 angular impulse는 발생하지 않음.
    //
    //     DeltaV = J * invMass
    bodyState.linearVelocity +=
        impulse * bodySim.invMass;
}

void World::ApplyAngularImpulse(
    BodyId bodyId,
    float impulse )
{
    assert( std::isfinite( impulse ) );

    const std::int32_t bodyIndex = GetBodyIndex( bodyId );
    const Body& body = bodies_[bodyIndex];

    if( body.type != BodyType::Dynamic )
    {
        return;
    }

    assert( bodySims_.size() == bodies_.size() );
    assert( bodyStates_.size() == bodies_.size() );

    const BodySim& bodySim = bodySims_[bodyIndex];
    BodyState& bodyState = bodyStates_[bodyIndex];

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
    bodyState.angularVelocity +=
        impulse * bodySim.invInertia;
}

void World::Step( float timeStep )
{
    assert( std::isfinite( timeStep ) );
    assert( timeStep >= 0.0f );

    assert( bodySims_.size() == bodies_.size() );
    assert( bodyStates_.size() == bodies_.size() );

    if( timeStep > 0.0f )
    {
        /*
        * Step 순서
        *
        * 1. force / gravity로 velocity 갱신
        * 2. 현재 transform의 Contact manifold 갱신
        * 3. Contact normal constraint를 풀어 velocity 보정
        * 4. 보정된 velocity로 transform 적분
        * 5. proxy 이동 후 Contact를 다시 갱신
        *
        * Solver를 position 적분보다 먼저 실행해야 이미 생성된 Contact가
        * 다음 frame에 더 깊게 파고드는 것을 막을 수 있음.
        */

        // -----------------------------------------------------
        // 1. Integrate velocities
        // -----------------------------------------------------
        for( std::int32_t bodyIndex = 0;
             bodyIndex < static_cast<std::int32_t>( bodies_.size() );
             ++bodyIndex )
        {
            const Body& body = bodies_[bodyIndex];

            if( body.bodyId == Body::NULL_INDEX ||
                body.type == BodyType::Static )
            {
                continue;
            }

            BodySim& bodySim = bodySims_[bodyIndex];
            BodyState& bodyState = bodyStates_[bodyIndex];

            assert( body.bodyId == bodyIndex );
            assert( bodySim.bodyId == bodyIndex );

            if( body.type == BodyType::Dynamic )
            {
                /*
                * Newton 제2법칙:
                *
                *     F = M * a
                *     a = F * invMass
                *
                *     dv = dt * ( gravity + F * invMass )
                *
                * 회전:
                *
                *     torque = I * angularAcceleration
                *     dw = dt * torque * invInertia
                */
                if( bodySim.invMass > 0.0f )
                {
                    const vec2 acceleration =
                        gravity_ +
                        bodySim.force * bodySim.invMass;

                    bodyState.linearVelocity +=
                        acceleration * timeStep;
                }

                if( bodySim.invInertia > 0.0f )
                {
                    bodyState.angularVelocity +=
                        timeStep *
                        bodySim.invInertia *
                        bodySim.torque;
                }

                // force / torque는 한 step 동안만 누적됨.
                bodySim.force = {};
                bodySim.torque = 0.0f;
            }
        }

        // -----------------------------------------------------
        // 2. Update current contacts
        // -----------------------------------------------------
        UpdateCollisions(
            []( const ContactData& )
            {
            }
        );

        // -----------------------------------------------------
        // 3. Solve normal contact constraints
        // -----------------------------------------------------
        SolveContacts();

        // -----------------------------------------------------
        // 4. Integrate positions
        // -----------------------------------------------------
        for( std::int32_t bodyIndex = 0;
             bodyIndex < static_cast<std::int32_t>( bodies_.size() );
             ++bodyIndex )
        {
            const Body& body = bodies_[bodyIndex];

            if( body.bodyId == Body::NULL_INDEX ||
                body.type == BodyType::Static )
            {
                continue;
            }

            BodySim& bodySim = bodySims_[bodyIndex];
            const BodyState& bodyState = bodyStates_[bodyIndex];

            /*
            * semi-implicit Euler
            *
            * velocity는 force와 Contact Solver에서 먼저 갱신됐고,
            * 여기서는 그 최종 velocity로 COM transform을 적분함.
            *
            *     x(t + dt) = x(t) + v(t + dt) * dt
            */
            bodySim.center +=
                bodyState.linearVelocity * timeStep;

            const float deltaAngle =
                bodyState.angularVelocity * timeStep;

            if( deltaAngle != 0.0f )
            {
                bodySim.transform.rotation =
                    rot2::FromRadians( deltaAngle ) *
                    bodySim.transform.rotation;
            }

            // center = origin + R * localCenter
            //
            // 따라서:
            //
            // origin = center - R * localCenter
            bodySim.transform.position =
                bodySim.center -
                Rotate(
                    bodySim.transform.rotation,
                    bodySim.localCenter
                );

            SyncBodyProxies( bodyIndex );
        }
    }

    // 이동 후 새 BroadPhase pair와 manifold를 만들어 다음 Step의 solver가 사용할
    // Contact 상태를 최신 transform 기준으로 준비함.
    UpdateCollisions(
        []( const ContactData& )
        {
        }
    );
}


const Shape& World::GetShape( ShapeId shapeId ) const
{
    return shapes_[GetShapeIndex( shapeId )];
}

ContactData World::GetContactData( ContactId contactId ) const
{
    return MakeContactData( GetContactIndex( contactId ) );
}

std::size_t World::GetBodyContactCapacity( BodyId bodyId ) const
{
    const Body& body =
        bodies_[GetBodyIndex( bodyId )];

    // Box2D와 같이 빠르고 보수적으로 Body의 전체 Contact 수를 반환함.
    return static_cast<std::size_t>( body.contactCount );
}

std::size_t World::GetBodyContactData(
    BodyId bodyId,
    std::span<ContactData> output ) const
{
    const Body& body =
        bodies_[GetBodyIndex( bodyId )];

    std::int32_t contactKey = body.headContactKey;
    std::size_t count = 0;

    while( contactKey != Body::NULL_INDEX &&
           count < output.size() )
    {
        const std::int32_t contactId =
            GetContactId( contactKey );
        const std::int32_t edgeIndex =
            GetContactEdgeIndex( contactKey );

        assert( contactId >= 0 );
        assert( static_cast<std::size_t>( contactId ) < contacts_.size() );

        const contact2& contact = contacts_[contactId];

        assert( contact.contactId == contactId );
        assert( edgeIndex == 0 || edgeIndex == 1 );
        assert( contactSims_.size() == contacts_.size() );

        const contactSim2& contactSim =
            contactSims_[contactId];

        assert( contactSim.contactId == contactId );

        // Contact는 AABB pair만으로도 존재할 수 있으므로 실제 접촉점이 있는 것만 공개함.
        if( contactSim.manifold.pointCount > 0 )
        {
            output[count] =
                MakeContactData( contactId );
            ++count;
        }

        contactKey =
            contact.edges[edgeIndex].nextKey;
    }

    return count;
}

std::size_t World::GetShapeContactCapacity( ShapeId shapeId ) const
{
    const std::int32_t shapeIndex =
        GetShapeIndex( shapeId );

    const Shape& shape = shapes_[shapeIndex];

    // Sensor Contact query는 Sensor 저장소를 구현할 때 별도로 연결함.
    if( shape.sensorIndex != Shape::NULL_INDEX )
    {
        return 0;
    }

    assert( shape.bodyId >= 0 );
    assert( static_cast<std::size_t>( shape.bodyId ) < bodies_.size() );

    const Body& body = bodies_[shape.bodyId];

    // 같은 Body의 다른 Shape Contact도 포함하므로 실제 필요량보다 클 수 있음.
    return static_cast<std::size_t>( body.contactCount );
}

std::size_t World::GetShapeContactData(
    ShapeId shapeId,
    std::span<ContactData> output ) const
{
    const std::int32_t shapeIndex =
        GetShapeIndex( shapeId );

    const Shape& shape = shapes_[shapeIndex];

    if( shape.sensorIndex != Shape::NULL_INDEX )
    {
        return 0;
    }

    assert( shape.bodyId >= 0 );
    assert( static_cast<std::size_t>( shape.bodyId ) < bodies_.size() );

    const Body& body = bodies_[shape.bodyId];

    std::int32_t contactKey = body.headContactKey;
    std::size_t count = 0;

    while( contactKey != Body::NULL_INDEX &&
           count < output.size() )
    {
        const std::int32_t contactId =
            GetContactId( contactKey );
        const std::int32_t edgeIndex =
            GetContactEdgeIndex( contactKey );

        assert( contactId >= 0 );
        assert( static_cast<std::size_t>( contactId ) < contacts_.size() );

        const contact2& contact = contacts_[contactId];

        assert( contact.contactId == contactId );
        assert( edgeIndex == 0 || edgeIndex == 1 );
        assert( contactSims_.size() == contacts_.size() );

        const contactSim2& contactSim =
            contactSims_[contactId];

        assert( contactSim.contactId == contactId );

        const bool involvesShape =
            contact.shapeIdA == shapeIndex ||
            contact.shapeIdB == shapeIndex;

        if( involvesShape &&
            contactSim.manifold.pointCount > 0 )
        {
            output[count] =
                MakeContactData( contactId );
            ++count;
        }

        contactKey =
            contact.edges[edgeIndex].nextKey;
    }

    return count;
}

ContactData World::MakeContactData( std::int32_t contactIndex ) const
{
    assert( contactIndex >= 0 );
    assert( static_cast<std::size_t>( contactIndex ) < contacts_.size() );

    const contact2& contact = contacts_[contactIndex];

    assert( contact.contactId == contactIndex );
    assert( contactSims_.size() == contacts_.size() );

    const contactSim2& contactSim =
        contactSims_[contactIndex];

    assert( contactSim.contactId == contactIndex );
    assert( contact.shapeIdA >= 0 );
    assert( contact.shapeIdB >= 0 );
    assert( static_cast<std::size_t>( contact.shapeIdA ) < shapes_.size() );
    assert( static_cast<std::size_t>( contact.shapeIdB ) < shapes_.size() );

    const Shape& shapeA = shapes_[contact.shapeIdA];

    assert( shapeA.bodyId >= 0 );
    assert( static_cast<std::size_t>( shapeA.bodyId ) < bodies_.size() );

    assert( bodySims_.size() == bodies_.size() );

    const BodySim& bodySimA = bodySims_[shapeA.bodyId];
    assert( bodySimA.bodyId == shapeA.bodyId );

    ContactData data{};
    data.contactId = MakeContactId( contactIndex );
    data.shapeIdA = MakeShapeId( contact.shapeIdA );
    data.shapeIdB = MakeShapeId( contact.shapeIdB );
    data.manifold =
        ToWorldManifold(
            contactSim.manifold,
            bodySimA.transform
        );

    return data;
}


std::int32_t World::CreateContact(
    std::int32_t shapeIdA,
    std::int32_t shapeIdB,
    const localManifold2& manifold )
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
        assert(
            contactSims_[contactId].contactId ==
            contactSim2::NULL_INDEX
        );

        contactFreeList_ = freeContact.nextFreeId;
    }
    else
    {
        assert(
            contacts_.size() <
            static_cast<std::size_t>( std::numeric_limits<std::int32_t>::max() )
        );

        contactId = static_cast<std::int32_t>( contacts_.size() );
        contacts_.push_back( {} );
        contactSims_.push_back( {} );
    }

    contact2& contact = contacts_[contactId];

    const std::uint32_t generation =
        contact.generation + 1u;

    contact = {};
    contact.contactId = contactId;
    contact.generation = generation;
    contact.shapeIdA = shapeIdA;
    contact.shapeIdB = shapeIdB;

    assert( contactSims_.size() == contacts_.size() );

    UpdateContactSim(
        contactId,
        manifold
    );

    const std::array<std::int32_t, 2> shapeIds =
    {
        shapeIdA,
        shapeIdB
    };

    for( std::int32_t edgeIndex = 0; edgeIndex < 2; ++edgeIndex )
    {
        const Shape& shape = shapes_[shapeIds[edgeIndex]];

        assert( shape.bodyId >= 0 );
        assert( static_cast<std::size_t>( shape.bodyId ) < bodies_.size() );

        Body& body = bodies_[shape.bodyId];
        contactEdge2& edge = contact.edges[edgeIndex];

        edge.bodyId = shape.bodyId;
        edge.prevKey = contact2::NULL_INDEX;
        edge.nextKey = body.headContactKey;

        const std::int32_t contactKey =
            MakeContactKey( contactId, edgeIndex );

        if( body.headContactKey != Body::NULL_INDEX )
        {
            const std::int32_t headContactId =
                GetContactId( body.headContactKey );
            const std::int32_t headEdgeIndex =
                GetContactEdgeIndex( body.headContactKey );

            assert( headContactId >= 0 );
            assert( static_cast<std::size_t>( headContactId ) < contacts_.size() );

            contact2& headContact = contacts_[headContactId];
            assert( headContact.contactId == headContactId );

            headContact.edges[headEdgeIndex].prevKey = contactKey;
        }

        body.headContactKey = contactKey;
        ++body.contactCount;
    }

    const ShapePairKey pairKey =
        MakeShapePairKey( shapeIdA, shapeIdB );

    // HashSet::Add는 새 key면 false, 이미 존재하면 true를 반환함.
    const bool alreadyExists = broadPhase_.AddPair( pairKey );
    assert( !alreadyExists );

    ++contactCount_;

    return contactId;
}

void World::UpdateContactSim(
    std::int32_t contactId,
    const localManifold2& manifold )
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

    const Shape& shapeA = shapes_[contact.shapeIdA];
    const Shape& shapeB = shapes_[contact.shapeIdB];

    assert( shapeA.bodyId >= 0 );
    assert( shapeB.bodyId >= 0 );
    assert( static_cast<std::size_t>( shapeA.bodyId ) < bodySims_.size() );
    assert( static_cast<std::size_t>( shapeB.bodyId ) < bodySims_.size() );

    const BodySim& bodySimA = bodySims_[shapeA.bodyId];
    const BodySim& bodySimB = bodySims_[shapeB.bodyId];

    assert( bodySimA.bodyId == shapeA.bodyId );
    assert( bodySimB.bodyId == shapeB.bodyId );

    contactSim2& contactSim =
        contactSims_[contactId];

    // narrow-phase가 새 manifold를 만들기 전에 이전 point id와
    // solver impulse를 보존해 같은 접촉점에 다시 연결함.
    const localManifold2 oldManifold =
        contactSim.manifold;

    const auto oldImpulses =
        contactSim.impulses;

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
        const std::uint16_t newId =
            contactSim.manifold.points[i].id;

        for( int j = 0; j < oldManifold.pointCount; ++j )
        {
            if( oldManifold.points[j].id == newId )
            {
                contactSim.impulses[i] =
                    oldImpulses[j];

                break;
            }
        }
    }
}

void World::SolveContacts()
{
    constexpr int VELOCITY_ITERATIONS = 8;

    std::vector<contactConstraint2> constraints;
    constraints.reserve( contactCount_ );

    /*
    * Prepare
    *
    * persistent ContactSim 중 실제 manifold point가 있는 Contact만
    * 이번 Step에서 사용할 transient solver constraint로 변환함.
    */
    for( const contactSim2& contactSim : contactSims_ )
    {
        if( contactSim.contactId == contactSim2::NULL_INDEX ||
            contactSim.manifold.pointCount == 0 )
        {
            continue;
        }

        assert( contactSim.bodyIdA >= 0 );
        assert( contactSim.bodyIdB >= 0 );
        assert( static_cast<std::size_t>( contactSim.bodyIdA ) < bodies_.size() );
        assert( static_cast<std::size_t>( contactSim.bodyIdB ) < bodies_.size() );

        // 같은 Body의 Shape끼리는 Contact를 만들지 않아야 함.
        assert( contactSim.bodyIdA != contactSim.bodyIdB );

        const BodySim& bodySimA =
            bodySims_[contactSim.bodyIdA];

        const BodySim& bodySimB =
            bodySims_[contactSim.bodyIdB];

        const BodyState& bodyStateA =
            bodyStates_[contactSim.bodyIdA];

        const BodyState& bodyStateB =
            bodyStates_[contactSim.bodyIdB];

        constraints.push_back(
            PrepareContactConstraint(
                contactSim,
                bodySimA,
                bodyStateA,
                bodySimB,
                bodyStateB
            )
        );
    }

    /*
    * Warm start
    *
    * 이전 step에서 수렴한 누적 impulse를 먼저 적용함.
    * 같은 접촉 상태가 이어질 때 solver가 0부터 다시 시작하지 않게 함.
    */
    for( const contactConstraint2& constraint : constraints )
    {
        WarmStartContactConstraint(
            constraint,
            bodyStates_[constraint.bodyIdA],
            bodyStates_[constraint.bodyIdB]
        );
    }

    /*
    * Sequential impulse
    *
    * Contact 하나를 풀 때 Body velocity가 즉시 바뀌고,
    * 다음 Contact는 그 갱신된 velocity를 사용함.
    *
    * 한 번의 순회로는 여러 Contact가 서로 영향을 주는 값을 충분히 전파하지 못하므로
    * 같은 constraint 목록을 여러 번 반복함.
    */
    for( int iteration = 0;
         iteration < VELOCITY_ITERATIONS;
         ++iteration )
    {
        for( contactConstraint2& constraint : constraints )
        {
            assert( constraint.bodyIdA >= 0 );
            assert( constraint.bodyIdB >= 0 );

            SolveContactConstraint(
                constraint,
                bodyStates_[constraint.bodyIdA],
                bodyStates_[constraint.bodyIdB]
            );
        }
    }

    /*
    * Store impulses
    *
    * 이번 step에서 수렴한 누적 impulse를 ContactSim에 되돌려 저장함.
    * 다음 narrow-phase 갱신 때 point id가 유지되면 이 값이 다시 이어짐.
    */
    for( const contactConstraint2& constraint : constraints )
    {
        assert( constraint.contactId >= 0 );
        assert(
            static_cast<std::size_t>( constraint.contactId ) <
            contactSims_.size()
        );

        StoreContactImpulses(
            constraint,
            contactSims_[constraint.contactId]
        );
    }

}

void World::DestroyContact( std::int32_t contactId )
{
    assert( contactId >= 0 );
    assert( static_cast<std::size_t>( contactId ) < contacts_.size() );

    contact2& contact = contacts_[contactId];

    assert( contact.contactId == contactId );
    assert( contactCount_ > 0 );

    const ShapePairKey pairKey =
        MakeShapePairKey( contact.shapeIdA, contact.shapeIdB );

    for( std::int32_t edgeIndex = 0; edgeIndex < 2; ++edgeIndex )
    {
        const contactEdge2 edge = contact.edges[edgeIndex];

        assert( edge.bodyId >= 0 );
        assert( static_cast<std::size_t>( edge.bodyId ) < bodies_.size() );

        Body& body = bodies_[edge.bodyId];
        const std::int32_t contactKey =
            MakeContactKey( contactId, edgeIndex );

        if( edge.prevKey != contact2::NULL_INDEX )
        {
            contact2& previous =
                contacts_[GetContactId( edge.prevKey )];

            previous.edges[GetContactEdgeIndex( edge.prevKey )].nextKey =
                edge.nextKey;
        }

        if( edge.nextKey != contact2::NULL_INDEX )
        {
            contact2& next =
                contacts_[GetContactId( edge.nextKey )];

            next.edges[GetContactEdgeIndex( edge.nextKey )].prevKey =
                edge.prevKey;
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

} // namespace zonai
