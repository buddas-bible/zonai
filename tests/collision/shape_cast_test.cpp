#include <cassert>
#include <cmath>

#include "collision/shapeCast2.h"
#include "collision/shapeProxy2.h"
#include "geometry/circle2.h"
#include "geometry/polygon2.h"

using namespace zonai;

int main()
{
    constexpr float epsilon = 1e-4f;

    // 반지름 1인 B가 x=5에서 왼쪽으로 10 이동하면 A와 약 30% 지점에서 만남.
    {
        shapeCastInput2 input{};
        input.proxyA = MakeShapeProxy( circle2{ {}, 1.0f } );

        input.proxyB = MakeShapeProxy( circle2{ {}, 1.0f } );

        input.transform.position = { 5.0f, 0.0f };

        input.translationB = { -10.0f, 0.0f };

        const castOutput2 output = ShapeCast( input );

        assert( output.hit );

        // target = radiusA + radiusB - LINEAR_SLOP 이므로
        // 실제 접촉보다 LINEAR_SLOP만큼 이르게 멈춤.
        assert( std::fabs( output.fraction - 0.3005f ) < epsilon );

        assert( std::fabs( output.point.x - 1.0f ) < epsilon );

        assert( std::fabs( output.normal.x - 1.0f ) < epsilon );

        assert( std::fabs( output.normal.y ) < epsilon );
    }

    // 반대 방향으로 이동하면 두 shape의 거리가 늘어나므로 miss.
    {
        shapeCastInput2 input{};
        input.proxyA = MakeShapeProxy( circle2{ {}, 1.0f } );

        input.proxyB = MakeShapeProxy( circle2{ {}, 1.0f } );

        input.transform.position = { 5.0f, 0.0f };

        input.translationB = { 10.0f, 0.0f };

        const castOutput2 output = ShapeCast( input );

        assert( !output.hit );
    }

    // 충돌 지점이 maxFraction 바깥이면 miss.
    {
        shapeCastInput2 input{};
        input.proxyA = MakeShapeProxy( circle2{ {}, 1.0f } );

        input.proxyB = MakeShapeProxy( circle2{ {}, 1.0f } );

        input.transform.position = { 5.0f, 0.0f };

        input.translationB = { -10.0f, 0.0f };

        input.maxFraction = 0.2f;

        const castOutput2 output = ShapeCast( input );

        assert( !output.hit );
    }

    // 시작부터 겹쳐 있으면 fraction 0 hit.
    {
        shapeCastInput2 input{};
        input.proxyA = MakeShapeProxy( circle2{ {}, 1.0f } );

        input.proxyB = MakeShapeProxy( circle2{ {}, 1.0f } );

        input.transform.position = { 1.5f, 0.0f };

        input.translationB = { -1.0f, 0.0f };

        const castOutput2 output = ShapeCast( input );

        assert( output.hit );
        assert( output.fraction == 0.0f );
    }

    // polygon proxy에도 같은 cast 알고리즘을 사용함.
    {
        shapeCastInput2 input{};
        input.proxyA = MakeShapeProxy( MakeBox( { 1.0f, 1.0f } ) );

        input.proxyB = MakeShapeProxy( MakeBox( { 0.5f, 0.5f } ) );

        input.transform.position = { 4.0f, 0.0f };

        input.translationB = { -5.0f, 0.0f };

        const castOutput2 output = ShapeCast( input );

        assert( output.hit );
        assert( output.fraction > 0.0f );
        assert( output.fraction < 1.0f );
        assert( output.normal.x > 0.99f );
    }

    // canEncroach=true면 이미 가까운 rounded shape가 LINEAR_SLOP만큼 더 접근 가능함.
    {
        shapeCastInput2 input{};
        input.proxyA = MakeShapeProxy( circle2{ {}, 1.0f } );

        input.proxyB = MakeShapeProxy( circle2{ {}, 1.0f } );

        input.transform.position = { 1.99f, 0.0f };

        input.translationB = { -1.0f, 0.0f };

        input.canEncroach = true;

        const castOutput2 output = ShapeCast( input );

        assert( output.hit );
        assert( output.fraction > 0.0f );
        assert( output.fraction < 0.02f );
    }

    return 0;
}
