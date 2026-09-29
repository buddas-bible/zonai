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
    }

    Body& body = bodies_[bodyIndex];

    const std::uint16_t generation =
        static_cast<std::uint16_t>( body.generation + 1u );

    body = {};
    body.bodyId = bodyIndex;
    body.generation = generation;
    body.type = type;
    body.transform = transform;

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
        const std::int32_t contactId =
            GetContactId( body.headContactKey );

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

    // generation은 보존하고 slot만 free-list에 반환함.
    body = {};
    body.generation = generation;
    body.nextFreeId = bodyFreeList_;
    bodyFreeList_ = bodyIndex;

    --bodyCount_;
}


ShapeId World::CreateShape(
    BodyId bodyId,
    ShapeGeometry geometry,
    Filter filter )
{
    const std::int32_t bodyIndex =
        GetBodyIndex( bodyId );

    assert( !std::holds_alternative<std::monostate>( geometry ) );

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
    storedShape.filter = filter;

    Body& body = bodies_[bodyIndex];

    const aabb2 worldAABB =
        ComputeShapeAABB( storedShape.geometry, body.transform );

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

    return MakeShapeId( shapeIndex );
}

void World::DestroyShape( ShapeId shapeId )
{
    DestroyShapeByIndex( GetShapeIndex( shapeId ) );
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
        const std::int32_t contactId =
            GetContactId( contactKey );
        const std::int32_t edgeIndex =
            GetContactEdgeIndex( contactKey );

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
    const std::int32_t bodyIndex =
        GetBodyIndex( bodyId );

    // transform이 NaN / infinity를 포함하면 tree AABB까지 오염되므로 입구에서 차단함.
    assert( std::isfinite( transform.position.x ) );
    assert( std::isfinite( transform.position.y ) );
    assert( std::isfinite( transform.rotation.c ) );
    assert( std::isfinite( transform.rotation.s ) );

    Body& body = bodies_[bodyIndex];
    body.transform = transform;

    std::int32_t shapeId = body.headShapeId;
    std::int32_t visitedCount = 0;

    while( shapeId != Body::NULL_INDEX )
    {
        assert( shapeId >= 0 );
        assert( static_cast<std::size_t>( shapeId ) < shapes_.size() );
        assert( visitedCount < body.shapeCount );

        Shape& shape = shapes_[shapeId];

        assert( shape.bodyId == bodyIndex );

        const aabb2 worldAABB = ComputeShapeAABB( shape.geometry, body.transform );

        // disabled Body 개념은 아직 없지만 Box2D와 같은 수명 규칙을 위해
        // proxy가 실제로 존재하는 Shape만 BroadPhase에서 이동시킴.
        if( shape.proxyKey != Shape::NULL_INDEX )
        {
            broadPhase_.MoveProxy( shape.proxyKey, worldAABB );
        }

        shapeId = shape.nextShapeId;
        ++visitedCount;
    }

    // linked list와 Body의 shapeCount가 서로 일치해야 함.
    assert( visitedCount == body.shapeCount );
}

const Body& World::GetBody( BodyId bodyId ) const
{
    return bodies_[GetBodyIndex( bodyId )];
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

        // Contact는 AABB pair만으로도 존재할 수 있으므로 실제 접촉점이 있는 것만 공개함.
        if( contact.manifold.pointCount > 0 )
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

        const bool involvesShape =
            contact.shapeIdA == shapeIndex ||
            contact.shapeIdB == shapeIndex;

        if( involvesShape &&
            contact.manifold.pointCount > 0 )
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
    assert( contact.shapeIdA >= 0 );
    assert( contact.shapeIdB >= 0 );
    assert( static_cast<std::size_t>( contact.shapeIdA ) < shapes_.size() );
    assert( static_cast<std::size_t>( contact.shapeIdB ) < shapes_.size() );

    const Shape& shapeA = shapes_[contact.shapeIdA];

    assert( shapeA.bodyId >= 0 );
    assert( static_cast<std::size_t>( shapeA.bodyId ) < bodies_.size() );

    const Body& bodyA = bodies_[shapeA.bodyId];

    ContactData data{};
    data.contactId = MakeContactId( contactIndex );
    data.shapeIdA = MakeShapeId( contact.shapeIdA );
    data.shapeIdB = MakeShapeId( contact.shapeIdB );
    data.manifold =
        ToWorldManifold(
            contact.manifold,
            bodyA.transform
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
    }

    contact2& contact = contacts_[contactId];

    const std::uint32_t generation =
        contact.generation + 1u;

    contact = {};
    contact.contactId = contactId;
    contact.generation = generation;
    contact.shapeIdA = shapeIdA;
    contact.shapeIdB = shapeIdB;
    contact.manifold = manifold;

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

    const std::uint32_t generation = contact.generation;

    contact = {};
    contact.generation = generation;
    contact.nextFreeId = contactFreeList_;
    contactFreeList_ = contactId;

    --contactCount_;
}

} // namespace zonai
