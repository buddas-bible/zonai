#include <cassert>
#include <cmath>

#include "collision/constants.h"
#include "collision/shapeGeometry.h"
#include "collision/shapeProxy2.h"

using namespace zonai;

int main()
{
    constexpr float epsilon = 1e-4f;

    // Dynamic Tree fat margin은 shape 크기의 12.5%를 사용하되 5cm를 넘지 않음.
    {
        assert( std::fabs( ComputeShapeAABBMargin( circle2{ {}, 1.0f } ) - MAX_AABB_MARGIN ) < epsilon );
        assert( std::fabs( ComputeShapeAABBMargin( circle2{ {}, 0.2f } ) - 0.025f ) < epsilon );
        assert( std::fabs( ComputeShapeAABBMargin( segment2{ { -0.2f, 0.0f }, { 0.2f, 0.0f } } ) - 0.025f ) < epsilon );
    }

    // CCD extent는 local center 기준 최소 두께와 최대 반경을 계산함.
    {
        const shapeExtent2 circleExtent = ComputeShapeExtent( circle2{ { 3.0f, -1.0f }, 2.0f }, { 1.0f, -1.0f } );

        assert( std::fabs( circleExtent.minExtent - 2.0f ) < epsilon );
        assert( std::fabs( circleExtent.maxExtent - 4.0f ) < epsilon );

        const shapeExtent2 capsuleExtent = ComputeShapeExtent( capsule2{ { -1.0f, 0.0f }, { 1.0f, 0.0f }, 0.5f }, {} );

        assert( std::fabs( capsuleExtent.minExtent - 0.5f ) < epsilon );
        assert( std::fabs( capsuleExtent.maxExtent - 1.5f ) < epsilon );

        const shapeExtent2 boxExtent = ComputeShapeExtent( MakeBox( { 2.0f, 1.0f } ), {} );

        assert( std::fabs( boxExtent.minExtent - 1.0f ) < epsilon );
        assert( std::fabs( boxExtent.maxExtent - std::sqrt( 5.0f ) ) < epsilon );

        const shapeExtent2 segmentExtent = ComputeShapeExtent( segment2{ { -2.0f, 0.0f }, { 2.0f, 0.0f } }, {} );

        assert( segmentExtent.minExtent == 0.0f );
        assert( std::fabs( segmentExtent.maxExtent - 2.0f ) < epsilon );
    }

    // concrete geometry는 CCD / GJK가 공통으로 사용할 convex proxy로 변환됨.
    {
        const shapeProxy2 circleProxy = MakeShapeProxy( circle2{ { 2.0f, -1.0f }, 0.75f } );

        assert( circleProxy.count == 1 );
        assert( circleProxy.points[0].x == 2.0f );
        assert( circleProxy.points[0].y == -1.0f );
        assert( circleProxy.radius == 0.75f );

        const shapeProxy2 capsuleProxy = MakeShapeProxy( capsule2{ { -1.0f, 0.0f }, { 1.0f, 0.0f }, 0.25f } );

        assert( capsuleProxy.count == 2 );
        assert( capsuleProxy.points[0].x == -1.0f );
        assert( capsuleProxy.points[1].x == 1.0f );
        assert( capsuleProxy.radius == 0.25f );

        const polygon2 box = MakeBox( { 2.0f, 1.0f } );
        const shapeProxy2 polygonProxy = MakeShapeProxy( box );

        assert( polygonProxy.count == box.vertexCount );
        assert( polygonProxy.radius == box.radius );

        const shapeProxy2 segmentProxy = MakeShapeProxy( segment2{ { -2.0f, 0.5f }, { 3.0f, 0.5f } } );

        assert( segmentProxy.count == 2 );
        assert( segmentProxy.points[0].x == -2.0f );
        assert( segmentProxy.points[1].x == 3.0f );
        assert( segmentProxy.radius == 0.0f );
    }

    return 0;
}
