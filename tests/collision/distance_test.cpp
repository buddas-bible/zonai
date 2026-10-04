#include <cassert>
#include <cmath>

#include "collision/distance2.h"
#include "geometry/circle2.h"
#include "geometry/polygon2.h"

using namespace zonai;

int main()
{
    constexpr float epsilon = 1e-4f;

    // Box2D와 같은 segment-segment 최근접점 primitive를 제공해야 함.
    {
        const segmentDistanceResult2 result = SegmentDistance(
            { -1.0f, -1.0f },
            { -1.0f, 1.0f },
            { 2.0f, 0.0f },
            { 1.0f, 0.0f }
        );

        assert( std::fabs( result.fraction1 - 0.5f ) < epsilon );
        assert( std::fabs( result.fraction2 - 1.0f ) < epsilon );
        assert( std::fabs( result.closest1.x + 1.0f ) < epsilon );
        assert( std::fabs( result.closest1.y ) < epsilon );
        assert( std::fabs( result.closest2.x - 1.0f ) < epsilon );
        assert( std::fabs( result.closest2.y ) < epsilon );
        assert( std::fabs( result.distanceSquared - 4.0f ) < epsilon );
    }

    // radius를 제외한 두 점 proxy 사이 거리.
    {
        distanceInput2 input{};
        input.proxyA = MakeShapeProxy( circle2{ {}, 1.0f } );
        input.proxyB = MakeShapeProxy( circle2{ {}, 1.0f } );
        input.transform.position = { 5.0f, 0.0f };

        simplexCache2 cache{};
        const distanceOutput2 output = ShapeDistance( input, cache );

        assert( std::fabs( output.distance - 5.0f ) < epsilon );
        assert( std::fabs( output.pointA.x ) < epsilon );
        assert( std::fabs( output.pointB.x - 5.0f ) < epsilon );
        assert( std::fabs( output.normal.x - 1.0f ) < epsilon );
        assert( std::fabs( output.normal.y ) < epsilon );
        assert( cache.count > 0 );
    }

    // circle radius를 포함하면 표면 사이 거리는 center distance - rA - rB.
    {
        distanceInput2 input{};
        input.proxyA = MakeShapeProxy( circle2{ {}, 1.0f } );
        input.proxyB = MakeShapeProxy( circle2{ {}, 2.0f } );
        input.transform.position = { 5.0f, 0.0f };
        input.useRadii = true;

        simplexCache2 cache{};
        const distanceOutput2 output = ShapeDistance( input, cache );

        assert( std::fabs( output.distance - 2.0f ) < epsilon );
        assert( std::fabs( output.pointA.x - 1.0f ) < epsilon );
        assert( std::fabs( output.pointB.x - 3.0f ) < epsilon );
        assert( std::fabs( output.normal.x - 1.0f ) < epsilon );
    }

    // 떨어진 box 사이의 가장 가까운 face 거리를 계산함.
    {
        distanceInput2 input{};
        input.proxyA = MakeShapeProxy( MakeBox( { 1.0f, 1.0f } ) );
        input.proxyB = MakeShapeProxy( MakeBox( { 1.0f, 1.0f } ) );
        input.transform.position = { 4.0f, 0.0f };

        simplexCache2 cache{};
        const distanceOutput2 output = ShapeDistance( input, cache );

        assert( std::fabs( output.distance - 2.0f ) < epsilon );
        assert( std::fabs( output.pointA.x - 1.0f ) < epsilon );
        assert( std::fabs( output.pointB.x - 3.0f ) < epsilon );
    }

    // 겹치는 convex shape는 거리 0을 반환함.
    {
        distanceInput2 input{};
        input.proxyA = MakeShapeProxy( MakeBox( { 1.0f, 1.0f } ) );
        input.proxyB = MakeShapeProxy( MakeBox( { 1.0f, 1.0f } ) );
        input.transform.position = { 1.0f, 0.0f };

        simplexCache2 cache{};
        const distanceOutput2 output = ShapeDistance( input, cache );

        assert( output.distance == 0.0f );
    }

    // relative transform의 회전도 B proxy에 적용됨.
    {
        distanceInput2 input{};
        input.proxyA = MakeShapeProxy( MakeBox( { 1.0f, 1.0f } ) );
        input.proxyB = MakeShapeProxy( MakeBox( { 2.0f, 0.5f } ) );
        input.transform =
        {
            { 0.0f, 4.0f },
            rot2::FromRadians( 0.5f * 3.14159265358979323846f )
        };

        simplexCache2 cache{};
        const distanceOutput2 output = ShapeDistance( input, cache );

        // 회전 후 B의 y 반높이는 2이므로 A top(y=1)과 B bottom(y=2)의 간격은 1.
        assert( std::fabs( output.distance - 1.0f ) < epsilon );
    }

    // cache를 재사용해도 동일한 결과를 유지함.
    {
        distanceInput2 input{};
        input.proxyA = MakeShapeProxy( MakeBox( { 2.0f, 1.0f } ) );
        input.proxyB = MakeShapeProxy( circle2{ {}, 0.5f } );
        input.transform.position = { 5.0f, 0.0f };
        input.useRadii = true;

        simplexCache2 cache{};

        const distanceOutput2 first = ShapeDistance( input, cache );
        const distanceOutput2 second = ShapeDistance( input, cache );

        assert( std::fabs( first.distance - second.distance ) < epsilon );
        assert( std::fabs( first.pointA.x - second.pointA.x ) < epsilon );
        assert( std::fabs( first.pointB.x - second.pointB.x ) < epsilon );
    }

    return 0;
}
