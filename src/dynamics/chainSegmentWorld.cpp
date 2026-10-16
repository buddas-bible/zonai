#include "dynamics/world.h"

#include <array>
#include <cassert>
#include <cmath>
#include <variant>

namespace zonai
{

namespace
{

bool IsValidChainSegmentGeometry( const chainSegment2& chainSegment )
{
    const float minimumLengthSquared = LINEAR_SLOP * LINEAR_SLOP;

    return IsFinite( chainSegment.ghost1 ) && IsFinite( chainSegment.segment.a ) && IsFinite( chainSegment.segment.b ) && IsFinite( chainSegment.ghost2 ) &&
           LengthSquared( chainSegment.segment.a - chainSegment.ghost1 ) > minimumLengthSquared &&
           LengthSquared( chainSegment.segment.b - chainSegment.segment.a ) > minimumLengthSquared &&
           LengthSquared( chainSegment.ghost2 - chainSegment.segment.b ) > minimumLengthSquared;
}

} // namespace

void world::UpdateChainSegmentByIndex( std::int32_t shapeIndex, const chainSegment2& chainSegment )
{
    assert( shapeIndex >= 0 && static_cast<std::size_t>( shapeIndex ) < shapes_.size() );
    assert( IsValidChainSegmentGeometry( chainSegment ) );

    shape& storedShape = shapes_[shapeIndex];
    assert( std::holds_alternative<chainSegment2>( storedShape.geometry ) );

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
            // Contact를 끊기 전에 양쪽 body를 깨워 정적 chain 편집도 잠든 visitor에 즉시 반영함.
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
    if( storedShape.proxyKey != shape::NULL_INDEX )
    {
        broadPhase_.MoveProxy( storedShape.proxyKey, newFatAABB );
    }
}

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

    const bool isValid = IsValidChainSegmentGeometry( chainSegment );
    assert( isValid );
    if( !isValid ) return;

    const std::int32_t bodyIndex = storedShape.bodyId;
    UpdateChainSegmentByIndex( shapeIndex, chainSegment );

    // Chain은 질량/COM을 바꾸지 않지만 local center에서의 최대 반경은
    // CCD, sleep, contact recycling 판정에 사용되므로 geometry 교체 직후 다시 합산함.
    UpdateBodyShapeExtents( bodyIndex );
    WakeBodyByIndex( bodyIndex );
}

void world::SetChainPoint( chainId chainId, std::size_t pointIndex, vec2 point )
{
    const std::int32_t chainIndex = GetChainIndex( chainId );
    chain2& chain = chains_[chainIndex];

    assert( IsFinite( point ) );
    if( !IsFinite( point ) ) return;

    const std::size_t segmentCount = chain.shapeIds.size();
    assert( segmentCount > 0 );

    const std::size_t pointCount = chain.isLoop ? segmentCount : segmentCount + 1;
    assert( pointIndex < pointCount );
    if( pointIndex >= pointCount ) return;

    const std::size_t pointShapeIndex = pointIndex < segmentCount ? pointIndex : segmentCount - 1;
    const chainSegment2& pointShape = std::get<chainSegment2>( shapes_[chain.shapeIds[pointShapeIndex]].geometry );
    const vec2 oldPoint = pointIndex < segmentCount ? pointShape.segment.a : pointShape.segment.b;
    if( oldPoint.x == point.x && oldPoint.y == point.y ) return;

    struct pendingUpdate
    {
        std::int32_t shapeIndex = shape::NULL_INDEX;
        chainSegment2 geometry{};
    };

    std::array<pendingUpdate, 4> updates{};
    std::size_t updateCount = 0;
    bool valid = true;

    const auto addUpdate = [&]( std::size_t segmentIndex )
    {
        if( !valid ) return;

        const std::int32_t shapeIndex = chain.shapeIds[segmentIndex];
        for( std::size_t i = 0; i < updateCount; ++i )
        {
            if( updates[i].shapeIndex == shapeIndex ) return;
        }

        shape& storedShape = shapes_[shapeIndex];
        assert( storedShape.parentChainId == chainIndex );
        assert( std::holds_alternative<chainSegment2>( storedShape.geometry ) );
        if( storedShape.parentChainId != chainIndex || !std::holds_alternative<chainSegment2>( storedShape.geometry ) )
        {
            valid = false;
            return;
        }

        chainSegment2 geometry = std::get<chainSegment2>( storedShape.geometry );

        if( chain.isLoop )
        {
            if( ( segmentIndex + segmentCount - 1 ) % segmentCount == pointIndex ) geometry.ghost1 = point;
            if( segmentIndex == pointIndex ) geometry.segment.a = point;
            if( ( segmentIndex + 1 ) % segmentCount == pointIndex ) geometry.segment.b = point;
            if( ( segmentIndex + 2 ) % segmentCount == pointIndex ) geometry.ghost2 = point;
        }
        else
        {
            if( segmentIndex > 0 && segmentIndex - 1 == pointIndex ) geometry.ghost1 = point;
            if( segmentIndex == pointIndex ) geometry.segment.a = point;
            if( segmentIndex + 1 == pointIndex ) geometry.segment.b = point;
            if( segmentIndex + 2 < pointCount && segmentIndex + 2 == pointIndex ) geometry.ghost2 = point;
        }

        if( !IsValidChainSegmentGeometry( geometry ) )
        {
            valid = false;
            return;
        }

        updates[updateCount++] = { shapeIndex, geometry };
    };

    if( chain.isLoop )
    {
        addUpdate( ( pointIndex + segmentCount - 2 ) % segmentCount );
        addUpdate( ( pointIndex + segmentCount - 1 ) % segmentCount );
        addUpdate( pointIndex );
        addUpdate( ( pointIndex + 1 ) % segmentCount );
    }
    else
    {
        if( pointIndex >= 2 ) addUpdate( pointIndex - 2 );
        if( pointIndex >= 1 ) addUpdate( pointIndex - 1 );
        if( pointIndex < segmentCount ) addUpdate( pointIndex );
        if( pointIndex + 1 < segmentCount ) addUpdate( pointIndex + 1 );
    }

    assert( valid );
    if( !valid ) return;

    for( std::size_t i = 0; i < updateCount; ++i )
    {
        UpdateChainSegmentByIndex( updates[i].shapeIndex, updates[i].geometry );
    }

    // 여러 child geometry가 바뀌어도 body 전체 extent와 wake는 batch 끝에서 한 번만 처리함.
    UpdateBodyShapeExtents( chain.bodyId );
    WakeBodyByIndex( chain.bodyId );
}

bool world::SetChainPoints( chainId chainId, std::span<const vec2> points, vec2 ghost1, vec2 ghost2 )
{
    if( !IsValid( chainId ) ) return false;
    chain2& chain = chains_[GetChainIndex( chainId )];
    const std::size_t count = chain.shapeIds.size();
    if( chain.isLoop || count == 0 || points.size() != count + 1 ) return false;

    const auto geometryAt = [&]( std::size_t i )
    {
        return chainSegment2{ i == 0 ? ghost1 : points[i - 1], { points[i], points[i + 1] }, i + 2 == points.size() ? ghost2 : points[i + 2] };
    };
    const auto samePoint = []( vec2 a, vec2 b ) { return a.x == b.x && a.y == b.y; };
    const auto sameGeometry = [&]( const chainSegment2& a, const chainSegment2& b )
    {
        return samePoint( a.ghost1, b.ghost1 ) && samePoint( a.segment.a, b.segment.a ) && samePoint( a.segment.b, b.segment.b ) && samePoint( a.ghost2, b.ghost2 );
    };

    bool changed = false;
    // 전체 geometry를 먼저 검증해 중간 상태의 퇴화 edge나 부분 적용을 피함.
    for( std::size_t i = 0; i < count; ++i )
    {
        const chainSegment2 geometry = geometryAt( i );
        if( !IsValidChainSegmentGeometry( geometry ) || !std::isfinite( LengthSquared( geometry.segment.a - geometry.ghost1 ) ) ||
            !std::isfinite( LengthSquared( geometry.segment.b - geometry.segment.a ) ) || !std::isfinite( LengthSquared( geometry.ghost2 - geometry.segment.b ) ) ) return false;
        changed |= !sameGeometry( std::get<chainSegment2>( shapes_[chain.shapeIds[i]].geometry ), geometry );
    }
    if( !changed ) return true;

    for( std::size_t i = 0; i < count; ++i )
    {
        const chainSegment2 geometry = geometryAt( i );
        if( sameGeometry( std::get<chainSegment2>( shapes_[chain.shapeIds[i]].geometry ), geometry ) ) continue;
        UpdateChainSegmentByIndex( chain.shapeIds[i], geometry );
    }
    UpdateBodyShapeExtents( chain.bodyId );
    WakeBodyByIndex( chain.bodyId );
    return true;
}

} // namespace zonai
