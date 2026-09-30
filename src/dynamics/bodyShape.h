#pragma once

#include <cassert>
#include <cstdint>
#include <span>

#include "collision/shape.h"
#include "dynamics/body.h"

namespace zonai
{

// shape를 body의 doubly linked list head에 연결함.
// bodyId / shapeId는 World 내부 storage의 index이며 slot은 삭제 후 재사용될 수 있음.
inline void LinkShape(
    body& bodyRef, std::int32_t bodyId,
    std::span<shape> shapes, std::int32_t shapeId )
{
    assert( bodyId >= 0 );
    assert( shapeId >= 0 );
    assert( static_cast<std::size_t>( shapeId ) < shapes.size() );

    shape& linkedShape = shapes[shapeId];

    // 이미 다른 body/list에 연결된 shape를 중복 삽입하면 안 됨.
    assert( linkedShape.bodyId == shape::NULL_INDEX );
    assert( linkedShape.prevShapeId == shape::NULL_INDEX );
    assert( linkedShape.nextShapeId == shape::NULL_INDEX );

    if( bodyRef.headShapeId != body::NULL_INDEX )
    {
        assert( bodyRef.headShapeId >= 0 );
        assert( static_cast<std::size_t>( bodyRef.headShapeId ) < shapes.size() );

        shape& oldHead = shapes[bodyRef.headShapeId];
        assert( oldHead.bodyId == bodyId );
        assert( oldHead.prevShapeId == shape::NULL_INDEX );

        oldHead.prevShapeId = shapeId;
    }

    linkedShape.bodyId = bodyId;
    linkedShape.prevShapeId = shape::NULL_INDEX;
    linkedShape.nextShapeId = bodyRef.headShapeId;

    bodyRef.headShapeId = shapeId;
    ++bodyRef.shapeCount;
}

// shape를 body의 doubly linked list에서 제거하고 양쪽 이웃을 다시 연결함.
inline void UnlinkShape(
    body& bodyRef, std::int32_t bodyId,
    std::span<shape> shapes, std::int32_t shapeId )
{
    assert( bodyId >= 0 );
    assert( shapeId >= 0 );
    assert( static_cast<std::size_t>( shapeId ) < shapes.size() );
    assert( bodyRef.shapeCount > 0 );

    shape& linkedShape = shapes[shapeId];

    assert( linkedShape.bodyId == bodyId );

    if( linkedShape.prevShapeId != shape::NULL_INDEX )
    {
        assert( static_cast<std::size_t>( linkedShape.prevShapeId ) < shapes.size() );

        shape& previous = shapes[linkedShape.prevShapeId];
        assert( previous.bodyId == bodyId );
        assert( previous.nextShapeId == shapeId );

        previous.nextShapeId = linkedShape.nextShapeId;
    }

    if( linkedShape.nextShapeId != shape::NULL_INDEX )
    {
        assert( static_cast<std::size_t>( linkedShape.nextShapeId ) < shapes.size() );

        shape& next = shapes[linkedShape.nextShapeId];
        assert( next.bodyId == bodyId );
        assert( next.prevShapeId == shapeId );

        next.prevShapeId = linkedShape.prevShapeId;
    }

    if( bodyRef.headShapeId == shapeId )
    {
        bodyRef.headShapeId = linkedShape.nextShapeId;
    }
    else
    {
        // head가 아닌 shape라면 반드시 이전 shape가 있어야 함.
        assert( linkedShape.prevShapeId != shape::NULL_INDEX );
    }

    linkedShape.bodyId = shape::NULL_INDEX;
    linkedShape.prevShapeId = shape::NULL_INDEX;
    linkedShape.nextShapeId = shape::NULL_INDEX;

    --bodyRef.shapeCount;

    if( bodyRef.shapeCount == 0 )
    {
        assert( bodyRef.headShapeId == body::NULL_INDEX );
    }
}

} // namespace zonai
