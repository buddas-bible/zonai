#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <variant>

#include "collision/shape.h"
#include "collision/shapeProxy2.h"

using namespace zonai;

int main()
{
    shape defaultShape{};

    // 아직 body / sensor에 연결되지 않은 runtime shape의 기본 상태를 확인함.
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

    constexpr float epsilon = 1e-4f;
    constexpr float pi = 3.14159265358979323846f;

    // CCD extent는 local center 기준 최소 두께와 최대 반경을 계산함.
    {
        const shapeExtent2 circleExtent =
            ComputeShapeExtent(
                circle2{ { 3.0f, -1.0f }, 2.0f },
                { 1.0f, -1.0f }
            );

        assert( std::fabs( circleExtent.minExtent - 2.0f ) < epsilon );
        assert( std::fabs( circleExtent.maxExtent - 4.0f ) < epsilon );

        const shapeExtent2 capsuleExtent =
            ComputeShapeExtent(
                capsule2
                {
                    { -1.0f, 0.0f },
                    { 1.0f, 0.0f },
                    0.5f
                },
                {}
            );

        assert( std::fabs( capsuleExtent.minExtent - 0.5f ) < epsilon );
        assert( std::fabs( capsuleExtent.maxExtent - 1.5f ) < epsilon );

        const shapeExtent2 boxExtent =
            ComputeShapeExtent(
                MakeBox( { 2.0f, 1.0f } ),
                {}
            );

        assert( std::fabs( boxExtent.minExtent - 1.0f ) < epsilon );
        assert( std::fabs( boxExtent.maxExtent - std::sqrt( 5.0f ) ) < epsilon );

        const shapeExtent2 segmentExtent =
            ComputeShapeExtent(
                segment2{ { -2.0f, 0.0f }, { 2.0f, 0.0f } },
                {}
            );

        assert( segmentExtent.minExtent == 0.0f );
        assert( std::fabs( segmentExtent.maxExtent - 2.0f ) < epsilon );
    }

    // concrete geometry는 CCD / GJK가 공통으로 사용할 convex proxy로 변환됨.
    {
        const shapeProxy2 circleProxy =
            MakeShapeProxy(
                circle2{ { 2.0f, -1.0f }, 0.75f }
            );

        assert( circleProxy.count == 1 );
        assert( circleProxy.points[0].x == 2.0f );
        assert( circleProxy.points[0].y == -1.0f );
        assert( circleProxy.radius == 0.75f );

        const shapeProxy2 capsuleProxy =
            MakeShapeProxy(
                capsule2
                {
                    { -1.0f, 0.0f },
                    { 1.0f, 0.0f },
                    0.25f
                }
            );

        assert( capsuleProxy.count == 2 );
        assert( capsuleProxy.points[0].x == -1.0f );
        assert( capsuleProxy.points[1].x == 1.0f );
        assert( capsuleProxy.radius == 0.25f );

        const polygon2 box =
            MakeBox( { 2.0f, 1.0f } );

        const shapeProxy2 polygonProxy =
            MakeShapeProxy( box );

        assert( polygonProxy.count == box.vertexCount );
        assert( polygonProxy.radius == box.radius );

        const shapeProxy2 segmentProxy =
            MakeShapeProxy(
                segment2
                {
                    { -2.0f, 0.5f },
                    { 3.0f, 0.5f }
                }
            );

        assert( segmentProxy.count == 2 );
        assert( segmentProxy.points[0].x == -2.0f );
        assert( segmentProxy.points[1].x == 3.0f );
        assert( segmentProxy.radius == 0.0f );
    }

    // 원의 mass / center / center 기준 inertia.
    {
        shape circleShape{};
        circleShape.geometry = circle2{ { 3.0f, -1.0f }, 2.0f };
        circleShape.density = 3.0f;

        const massData2 massData =
            ComputeShapeMass( circleShape );

        assert( std::fabs( massData.mass - 12.0f * pi ) < epsilon );
        assert( std::fabs( massData.center.x - 3.0f ) < epsilon );
        assert( std::fabs( massData.center.y + 1.0f ) < epsilon );
        assert(
            std::fabs(
                massData.rotationalInertia -
                24.0f * pi
            ) < epsilon
        );
    }

    // 4x2 box, density 3 => mass 24, inertia = m(w^2+h^2)/12 = 40.
    {
        shape boxShape{};
        boxShape.geometry = MakeBox( { 2.0f, 1.0f } );
        boxShape.density = 3.0f;

        const massData2 massData =
            ComputeShapeMass( boxShape );

        assert( std::fabs( massData.mass - 24.0f ) < epsilon );
        assert( std::fabs( massData.center.x ) < epsilon );
        assert( std::fabs( massData.center.y ) < epsilon );
        assert( std::fabs( massData.rotationalInertia - 40.0f ) < epsilon );
    }

    // Segment는 면적이 없으므로 density가 있어도 질량은 0임.
    {
        shape segmentShape{};
        segmentShape.geometry =
            segment2{ { -2.0f, 0.0f }, { 2.0f, 0.0f } };
        segmentShape.density = 10.0f;

        const massData2 massData =
            ComputeShapeMass( segmentShape );

        assert( massData.mass == 0.0f );
        assert( massData.rotationalInertia == 0.0f );
    }

    return 0;
}
