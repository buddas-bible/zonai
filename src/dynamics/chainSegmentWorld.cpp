#include "dynamics/world.h"

#include <cassert>
#include <variant>

namespace zonai
{

void world::SetChainSegment( shapeId shapeId, const chainSegment2& chainSegment )
{
    const std::int32_t shapeIndex = GetShapeIndex( shapeId );
    shape& storedShape = shapes_[shapeIndex];

    // Parent chain은 이웃 segment와 ghost vertex의 연결 관계를 함께 소유함.
    // Child 하나만 바꾸면 adjacency가 깨지므로 orphan segment만 직접 수정함.
    assert( storedShape.parentChainId == shape::NULL_INDEX );
    if( storedShape.parentChainId != shape::NULL_INDEX ) return;

    assert( std::holds_alternative<chainSegment2>( storedShape.geometry ) );
    if( !std::holds_alternative<chainSegment2>( storedShape.geometry ) ) return;

    const float minimumLengthSquared = LINEAR_SLOP * LINEAR_SLOP;
    const bool isValid = IsFinite( chainSegment.ghost1 ) && IsFinite( chainSegment.segment.a ) && IsFinite( chainSegment.segment.b ) && IsFinite( chainSegment.ghost2 ) &&
                         LengthSquared( chainSegment.segment.a - chainSegment.ghost1 ) > minimumLengthSquared &&
                         LengthSquared( chainSegment.segment.b - chainSegment.segment.a ) > minimumLengthSquared &&
                         LengthSquared( chainSegment.ghost2 - chainSegment.segment.b ) > minimumLengthSquared;

    assert( isValid );
    if( !isValid ) return;

    const std::int32_t bodyIndex = storedShape.bodyId;
    assert( bodyIndex != shape::NULL_INDEX );

    // Geometry가 바뀌면 기존 manifold / warm-start cache는 더 이상 같은 접촉을 뜻하지 않음.
    body& owner = bodies_[bodyIndex];
    std::int32_t contactKey = owner.headContactKey;

    while( contactKey != body::NULL_INDEX )
    {
        const std::int32_t edgeIndex = contactKey & 1;
        const std::int32_t contactId = contactKey >> 1;

        assert( contactId >= 0 );
        assert( static_cast<std::size_t>( contactId ) < contacts_.size() );

        contact2& contact = contacts_[contactId];
        assert( contact.contactId == contactId );

        contactKey = contact.edges[edgeIndex].nextKey;

        if( contact.shapeIdA == shapeIndex || contact.shapeIdB == shapeIndex )
        {
            DestroyContact( contactId );
        }
    }

    storedShape.geometry = chainSegment;
    storedShape.aabbMargin = ComputeShapeAABBMargin( storedShape.geometry );

    const transform2 transform = bodySims_[bodyIndex].transform;
    const aabb2 tightAABB = ComputeShapeAABB( storedShape.geometry, transform );
    storedShape.aabb = ExpandAABB( tightAABB, SPECULATIVE_DISTANCE );

    const float fatMargin = owner.type == bodyType::Static ? SPECULATIVE_DISTANCE : storedShape.aabbMargin;
    const aabb2 newFatAABB = ExpandAABB( storedShape.aabb, fatMargin );
    fatAABBs_[shapeIndex] = newFatAABB;

    // Geometry replacement는 기존 fat bounds 안쪽으로 이동하는 경우에도 새 pair 탐색이 필요함.
    // 따라서 기존 proxy bounds containment와 무관하게 leaf를 새 위치로 재배치함.
    if( storedShape.proxyKey != shape::NULL_INDEX )
    {
        broadPhase_.MoveProxy( storedShape.proxyKey, newFatAABB );
    }

    WakeBodyByIndex( bodyIndex );
}

} // namespace zonai
