#include "dynamics/world.h"

#include <cassert>
#include <cmath>
#include <limits>
#include <utility>

#include "dynamics/bodyShape.h"

namespace zonai
{

std::int32_t World::CreateBody( BodyType type, transform2 transform )
{
    std::int32_t bodyId = Body::NULL_INDEX;

    if( bodyFreeList_ != Body::NULL_INDEX )
    {
        bodyId = bodyFreeList_;

        assert( bodyId >= 0 );
        assert( static_cast<std::size_t>( bodyId ) < bodies_.size() );

        Body& freeBody = bodies_[bodyId];
        assert( freeBody.bodyId == Body::NULL_INDEX );
        assert( freeBody.headShapeId == Body::NULL_INDEX );
        assert( freeBody.headContactKey == Body::NULL_INDEX );

        bodyFreeList_ = freeBody.nextFreeId;
        freeBody = {};
    }
    else
    {
        assert(
            bodies_.size() <
            static_cast<std::size_t>( std::numeric_limits<std::int32_t>::max() )
        );

        bodyId = static_cast<std::int32_t>( bodies_.size() );
        bodies_.push_back( {} );
    }

    Body& body = bodies_[bodyId];
    body.bodyId = bodyId;
    body.type = type;
    body.transform = transform;

    ++bodyCount_;

    return bodyId;
}

void World::DestroyBody( std::int32_t bodyId )
{
    assert( bodyId >= 0 );
    assert( static_cast<std::size_t>( bodyId ) < bodies_.size() );
    assert( bodyCount_ > 0 );

    Body& body = bodies_[bodyId];

    assert( body.bodyId == bodyId );

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
        DestroyShape( body.headShapeId );
    }

    assert( body.shapeCount == 0 );
    assert( body.headShapeId == Body::NULL_INDEX );
    assert( body.headContactKey == Body::NULL_INDEX );

    // slot index는 유지하고 free-list에 연결해 다음 CreateBody에서 재사용함.
    body = {};
    body.nextFreeId = bodyFreeList_;
    bodyFreeList_ = bodyId;

    --bodyCount_;
}


std::int32_t World::CreateShape( std::int32_t bodyId, ShapeGeometry geometry, Filter filter )
{
    assert( bodyId >= 0 );
    assert( static_cast<std::size_t>( bodyId ) < bodies_.size() );
    assert( bodies_[bodyId].bodyId == bodyId );
    assert( !std::holds_alternative<std::monostate>( geometry ) );

    std::int32_t shapeId = Shape::NULL_INDEX;

    if( shapeFreeList_ != Shape::NULL_INDEX )
    {
        shapeId = shapeFreeList_;

        assert( shapeId >= 0 );
        assert( static_cast<std::size_t>( shapeId ) < shapes_.size() );

        Shape& freeShape = shapes_[shapeId];
        assert( freeShape.bodyId == Shape::NULL_INDEX );
        assert( std::holds_alternative<std::monostate>( freeShape.geometry ) );

        shapeFreeList_ = freeShape.nextFreeId;
        freeShape = {};
    }
    else
    {
        assert(
            shapes_.size() <
            static_cast<std::size_t>( std::numeric_limits<std::int32_t>::max() )
        );

        shapeId = static_cast<std::int32_t>( shapes_.size() );
        shapes_.push_back( {} );
    }

    Shape& storedShape = shapes_[shapeId];
    storedShape.geometry = std::move( geometry );
    storedShape.filter = filter;

    Body& body = bodies_[bodyId];

    const aabb2 worldAABB =
        ComputeShapeAABB( storedShape.geometry, body.transform );

    // Box2D의 기본 Shape 생성처럼 static Shape도 즉시 pair 탐색 대상이 되게 함.
    storedShape.proxyKey =
        broadPhase_.CreateProxy(
            body.type,
            worldAABB,
            shapeId,
            true
        );

    LinkShape( body, bodyId, shapes_, shapeId );
    ++shapeCount_;

    return shapeId;
}

void World::DestroyShape( std::int32_t shapeId )
{
    assert( shapeId >= 0 );
    assert( static_cast<std::size_t>( shapeId ) < shapes_.size() );
    assert( shapeCount_ > 0 );

    Shape& shape = shapes_[shapeId];

    assert( shape.bodyId != Shape::NULL_INDEX );
    assert( shape.proxyKey != Shape::NULL_INDEX );
    assert( !std::holds_alternative<std::monostate>( shape.geometry ) );

    // Sensor 저장소는 아직 구현하지 않았으므로 현재는 일반 collision Shape만 제거함.
    assert( shape.sensorIndex == Shape::NULL_INDEX );

    const std::int32_t bodyId = shape.bodyId;
    Body& body = bodies_[bodyId];

    // Box2D처럼 먼저 Body의 Shape list에서 분리함.
    UnlinkShape( body, bodyId, shapes_, shapeId );

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

        if( contact.shapeIdA == shapeId ||
            contact.shapeIdB == shapeId )
        {
            DestroyContact( contactId );
        }
    }

    // slot index는 유지하고 free-list에 연결해 다음 CreateShape에서 재사용함.
    shape = {};
    shape.nextFreeId = shapeFreeList_;
    shapeFreeList_ = shapeId;

    --shapeCount_;
}


void World::SetBodyTransform( std::int32_t bodyId, transform2 transform )
{
    assert( bodyId >= 0 );
    assert( static_cast<std::size_t>( bodyId ) < bodies_.size() );
    assert( bodies_[bodyId].bodyId == bodyId );

    // transform이 NaN / infinity를 포함하면 tree AABB까지 오염되므로 입구에서 차단함.
    assert( std::isfinite( transform.position.x ) );
    assert( std::isfinite( transform.position.y ) );
    assert( std::isfinite( transform.rotation.c ) );
    assert( std::isfinite( transform.rotation.s ) );

    Body& body = bodies_[bodyId];
    body.transform = transform;

    std::int32_t shapeId = body.headShapeId;
    std::int32_t visitedCount = 0;

    while( shapeId != Body::NULL_INDEX )
    {
        assert( shapeId >= 0 );
        assert( static_cast<std::size_t>( shapeId ) < shapes_.size() );
        assert( visitedCount < body.shapeCount );

        Shape& shape = shapes_[shapeId];

        assert( shape.bodyId == bodyId );

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

const Body& World::GetBody( std::int32_t bodyId ) const
{
    assert( bodyId >= 0 );
    assert( static_cast<std::size_t>( bodyId ) < bodies_.size() );
    assert( bodies_[bodyId].bodyId == bodyId );

    return bodies_[bodyId];
}


const Shape& World::GetShape( std::int32_t shapeId ) const
{
    assert( shapeId >= 0 );
    assert( static_cast<std::size_t>( shapeId ) < shapes_.size() );

    assert( shapes_[shapeId].bodyId != Shape::NULL_INDEX );
    return shapes_[shapeId];
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
    contact = {};
    contact.contactId = contactId;
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

    contact = {};
    contact.nextFreeId = contactFreeList_;
    contactFreeList_ = contactId;

    --contactCount_;
}

} // namespace zonai
