#pragma once

#include <cassert>
#include <cstdint>
#include <span>

#include "collision/shape.h"
#include "dynamics/body.h"

namespace zonai
{

// Shape를 Body의 doubly linked list head에 연결함.
// bodyId / shapeId는 World 내부 storage의 index이며 slot은 삭제 후 재사용될 수 있음.
inline void LinkShape(
    Body& body,
    std::int32_t bodyId,
    std::span<Shape> shapes,
    std::int32_t shapeId )
{
    assert( bodyId >= 0 );
    assert( shapeId >= 0 );
    assert( static_cast<std::size_t>( shapeId ) < shapes.size() );

    Shape& shape = shapes[shapeId];

    // 이미 다른 Body/list에 연결된 Shape를 중복 삽입하면 안 됨.
    assert( shape.bodyId == Shape::NULL_INDEX );
    assert( shape.prevShapeId == Shape::NULL_INDEX );
    assert( shape.nextShapeId == Shape::NULL_INDEX );

    if( body.headShapeId != Body::NULL_INDEX )
    {
        assert( body.headShapeId >= 0 );
        assert( static_cast<std::size_t>( body.headShapeId ) < shapes.size() );

        Shape& oldHead = shapes[body.headShapeId];
        assert( oldHead.bodyId == bodyId );
        assert( oldHead.prevShapeId == Shape::NULL_INDEX );

        oldHead.prevShapeId = shapeId;
    }

    shape.bodyId = bodyId;
    shape.prevShapeId = Shape::NULL_INDEX;
    shape.nextShapeId = body.headShapeId;

    body.headShapeId = shapeId;
    ++body.shapeCount;
}

// Shape를 Body의 doubly linked list에서 제거하고 양쪽 이웃을 다시 연결함.
inline void UnlinkShape(
    Body& body,
    std::int32_t bodyId,
    std::span<Shape> shapes,
    std::int32_t shapeId )
{
    assert( bodyId >= 0 );
    assert( shapeId >= 0 );
    assert( static_cast<std::size_t>( shapeId ) < shapes.size() );
    assert( body.shapeCount > 0 );

    Shape& shape = shapes[shapeId];

    assert( shape.bodyId == bodyId );

    if( shape.prevShapeId != Shape::NULL_INDEX )
    {
        assert( static_cast<std::size_t>( shape.prevShapeId ) < shapes.size() );

        Shape& previous = shapes[shape.prevShapeId];
        assert( previous.bodyId == bodyId );
        assert( previous.nextShapeId == shapeId );

        previous.nextShapeId = shape.nextShapeId;
    }

    if( shape.nextShapeId != Shape::NULL_INDEX )
    {
        assert( static_cast<std::size_t>( shape.nextShapeId ) < shapes.size() );

        Shape& next = shapes[shape.nextShapeId];
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
        // head가 아닌 Shape라면 반드시 이전 Shape가 있어야 함.
        assert( shape.prevShapeId != Shape::NULL_INDEX );
    }

    shape.bodyId = Shape::NULL_INDEX;
    shape.prevShapeId = Shape::NULL_INDEX;
    shape.nextShapeId = Shape::NULL_INDEX;

    --body.shapeCount;

    if( body.shapeCount == 0 )
    {
        assert( body.headShapeId == Body::NULL_INDEX );
    }
}

} // namespace zonai
