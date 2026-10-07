#include <array>
#include <cassert>
#include <span>

#include "dynamics/body.h"
#include "dynamics/bodyShape.h"
#include "dynamics/shape.h"

using namespace zonai;

int main()
{
    body body{};
    constexpr std::int32_t bodyId = 3;

    std::array<shape, 3> shapes{};

    LinkShape( body, bodyId, shapes, 0 );

    assert( body.headShapeId == 0 );
    assert( body.shapeCount == 1 );
    assert( shapes[0].bodyId == bodyId );
    assert( shapes[0].prevShapeId == shape::NULL_INDEX );
    assert( shapes[0].nextShapeId == shape::NULL_INDEX );

    // 새 shape는 Box2D처럼 항상 head에 삽입됨.
    LinkShape( body, bodyId, shapes, 1 );
    LinkShape( body, bodyId, shapes, 2 );

    // body -> 2 <-> 1 <-> 0
    assert( body.headShapeId == 2 );
    assert( body.shapeCount == 3 );

    assert( shapes[2].prevShapeId == shape::NULL_INDEX );
    assert( shapes[2].nextShapeId == 1 );

    assert( shapes[1].prevShapeId == 2 );
    assert( shapes[1].nextShapeId == 0 );

    assert( shapes[0].prevShapeId == 1 );
    assert( shapes[0].nextShapeId == shape::NULL_INDEX );

    // 가운데 shape를 제거하면 양쪽 이웃이 직접 연결되어야 함.
    UnlinkShape( body, bodyId, shapes, 1 );

    // body -> 2 <-> 0
    assert( body.headShapeId == 2 );
    assert( body.shapeCount == 2 );

    assert( shapes[2].prevShapeId == shape::NULL_INDEX );
    assert( shapes[2].nextShapeId == 0 );

    assert( shapes[0].prevShapeId == 2 );
    assert( shapes[0].nextShapeId == shape::NULL_INDEX );

    assert( shapes[1].bodyId == shape::NULL_INDEX );
    assert( shapes[1].prevShapeId == shape::NULL_INDEX );
    assert( shapes[1].nextShapeId == shape::NULL_INDEX );

    // head를 제거하면 다음 shape가 새 head가 됨.
    UnlinkShape( body, bodyId, shapes, 2 );

    assert( body.headShapeId == 0 );
    assert( body.shapeCount == 1 );
    assert( shapes[0].prevShapeId == shape::NULL_INDEX );
    assert( shapes[0].nextShapeId == shape::NULL_INDEX );

    // 마지막 shape까지 제거하면 빈 body 상태로 돌아감.
    UnlinkShape( body, bodyId, shapes, 0 );

    assert( body.headShapeId == body::NULL_INDEX );
    assert( body.shapeCount == 0 );

    return 0;
}
