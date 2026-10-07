#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>

#include "dynamics/body.h"
#include "dynamics/bodyShape.h"
#include "dynamics/shape.h"

using namespace zonai;

int main()
{
    constexpr float epsilon = 1e-4f;
    constexpr float pi = 3.14159265358979323846f;

    // 아직 body / sensor에 연결되지 않은 runtime shape의 기본 상태를 확인함.
    {
        shape defaultShape{};

        assert( defaultShape.bodyId == shape::NULL_INDEX );
        assert( defaultShape.prevShapeId == shape::NULL_INDEX );
        assert( defaultShape.nextShapeId == shape::NULL_INDEX );
        assert( defaultShape.generation == 0 );
        assert( defaultShape.sensorIndex == shape::NULL_INDEX );
        assert( defaultShape.nextFreeId == shape::NULL_INDEX );
        assert( defaultShape.proxyKey == shape::NULL_INDEX );
        assert( defaultShape.density == 1.0f );

        // 기본 filter는 Box2D처럼 category 1이 모든 category와 충돌하도록 설정됨.
        assert( defaultShape.filter.categoryBits == 1 );
        assert( defaultShape.filter.maskBits == std::numeric_limits<std::uint64_t>::max() );
        assert( defaultShape.filter.groupIndex == 0 );

        defaultShape.bodyId = 7;
        defaultShape.sensorIndex = 3;
        defaultShape.filter.categoryBits = 0x00000004;
        defaultShape.filter.maskBits = 0x00000002;
        defaultShape.filter.groupIndex = -5;

        assert( defaultShape.bodyId == 7 );
        assert( defaultShape.sensorIndex == 3 );
        assert( defaultShape.filter.categoryBits == 0x00000004 );
        assert( defaultShape.filter.maskBits == 0x00000002 );
        assert( defaultShape.filter.groupIndex == -5 );
    }

    // runtime shape의 geometry와 density로 질량 특성을 계산함.
    {
        shape circleShape{};
        circleShape.geometry = circle2{ { 3.0f, -1.0f }, 2.0f };
        circleShape.density = 3.0f;

        const massData2 circleMass = ComputeShapeMass( circleShape );

        assert( std::fabs( circleMass.mass - 12.0f * pi ) < epsilon );
        assert( std::fabs( circleMass.center.x - 3.0f ) < epsilon );
        assert( std::fabs( circleMass.center.y + 1.0f ) < epsilon );
        assert( std::fabs( circleMass.rotationalInertia - 24.0f * pi ) < epsilon );

        shape boxShape{};
        boxShape.geometry = MakeBox( { 2.0f, 1.0f } );
        boxShape.density = 3.0f;

        const massData2 boxMass = ComputeShapeMass( boxShape );

        assert( std::fabs( boxMass.mass - 24.0f ) < epsilon );
        assert( std::fabs( boxMass.center.x ) < epsilon );
        assert( std::fabs( boxMass.center.y ) < epsilon );
        assert( std::fabs( boxMass.rotationalInertia - 40.0f ) < epsilon );

        shape segmentShape{};
        segmentShape.geometry = segment2{ { -2.0f, 0.0f }, { 2.0f, 0.0f } };
        segmentShape.density = 10.0f;

        const massData2 segmentMass = ComputeShapeMass( segmentShape );

        assert( segmentMass.mass == 0.0f );
        assert( segmentMass.rotationalInertia == 0.0f );
    }

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
