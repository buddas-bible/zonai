#include <cassert>
#include <cstdint>
#include <limits>
#include <variant>

#include "collision/shape.h"

using namespace zonai;

int main()
{
    Shape shape{};

    // 아직 body / sensor에 연결되지 않은 runtime shape의 기본 상태를 확인함.
    assert( shape.bodyId == Shape::NULL_INDEX );
    assert( shape.sensorIndex == Shape::NULL_INDEX );

    // 기본 filter는 Box2D처럼 category 1이 모든 category와 충돌하도록 설정됨.
    assert( shape.filter.categoryBits == 1 );
    assert( shape.filter.maskBits == std::numeric_limits<std::uint64_t>::max() );
    assert( shape.filter.groupIndex == 0 );

    shape.bodyId = 7;
    shape.sensorIndex = 3;
    shape.filter.categoryBits = 0x00000004;
    shape.filter.maskBits = 0x00000002;
    shape.filter.groupIndex = -5;

    assert( shape.bodyId == 7 );
    assert( shape.sensorIndex == 3 );
    assert( shape.filter.categoryBits == 0x00000004 );
    assert( shape.filter.maskBits == 0x00000002 );
    assert( shape.filter.groupIndex == -5 );

    return 0;
}
