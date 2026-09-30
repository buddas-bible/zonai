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
    body& body, std::int32_t bodyId,
    std::span<shape> shapes, std::int32_t shapeId )
{
    assert( bodyId >= 0 );
    assert( shapeId >= 0 );
    assert( static_cast<std::size_t>( shapeId ) < shapes.size() );

    shape& shape = shapes[shapeId];

    // 이미 다른 body/list에 연결된 shape를 중복 삽입하면 안 됨.
    assert( shape.bodyId == shape::NULL_INDEX );
    assert( shape.prevShapeId == shape::NULL_INDEX );
    assert( shape.nextShapeId == shape::NULL_INDEX );

    if( body.headShapeId != body::NULL_INDEX )
    {
        assert( body.headShapeId >= 0 );
        assert( static_cast<std::size_t>( body.headShapeId ) < shapes.size() );

        shape& oldHead = shapes[body.headShapeId];
        assert( oldHead.bodyId == bodyId );
        assert( oldHead.prevShapeId == shape::NULL_INDEX );

        oldHead.prevShapeId = shapeId;
    }

    shape.bodyId = bodyId;
    shape.prevShapeId = shape::NULL_INDEX;
    shape.nextShapeId = body.headShapeId;

    body.headShapeId = shapeId;
    ++body.shapeCount;
}

// shape를 body의 doubly linked list에서 제거하고 양쪽 이웃을 다시 연결함.
inline void UnlinkShape(
    body& body, std::int32_t bodyId,
    std::span<shape> shapes, std::int32_t shapeId )
{
    assert( bodyId >= 0 );
    assert( shapeId >= 0 );
    assert( static_cast<std::size_t>( shapeId ) < shapes.size() );
    assert( body.shapeCount > 0 );

    shape& shape = shapes[shapeId];

    assert( shape.bodyId == bodyId );

    if( shape.prevShapeId != shape::NULL_INDEX )
    {
        assert( static_cast<std::size_t>( shape.prevShapeId ) < shapes.size() );

        shape& previous = shapes[shape.prevShapeId];
        assert( previous.bodyId == bodyId );
        assert( previous.nextShapeId == shapeId );

        previous.nextShapeId = shape.nextShapeId;
    }

    if( shape.nextShapeId != shape::NULL_INDEX )
    {
        assert( static_cast<std::size_t>( shape.nextShapeId ) < shapes.size() );

        shape& next = shapes[shape.nextShapeId];
        assert( next.bodyId == bodyId );
        assert( next.prevShapeId == shapeId );

        next.prevShapeId = shape.prevShapeId;
    }

    if( body.headShapeId == shapeId )
    {
        body.headShapeId = shape.nextShapeId;
    }
    else
    {
        // head가 아닌 shape라면 반드시 이전 shape가 있어야 함.
        assert( shape.prevShapeId != shape::NULL_INDEX );
    }

    shape.bodyId = shape::NULL_INDEX;
    shape.prevShapeId = shape::NULL_INDEX;
    shape.nextShapeId = shape::NULL_INDEX;

    --body.shapeCount;

    if( body.shapeCount == 0 )
    {
        assert( body.headShapeId == body::NULL_INDEX );
    }
}

} // namespace zonai
