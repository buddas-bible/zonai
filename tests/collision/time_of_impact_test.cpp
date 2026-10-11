#include <cassert>
#include <cmath>

#include "collision/timeOfImpact2.h"
#include "geometry/circle2.h"
#include "geometry/polygon2.h"
#include "geometry/segment2.h"

using namespace zonai;

int main()
{
    constexpr float epsilon = 5e-3f;

    // Box2D reference와 같은 linear sweep: x=2 segment가 x=-2까지 이동하며
    // [-1,1] box의 오른쪽 face에 t=0.5 부근에서 닿음.
    {
        shapeProxy2 proxyA{};
        proxyA.points[0] = { -1.0f, -1.0f };
        proxyA.points[1] = { 1.0f, -1.0f };
        proxyA.points[2] = { 1.0f, 1.0f };
        proxyA.points[3] = { -1.0f, 1.0f };
        proxyA.count = 4;

        shapeProxy2 proxyB{};
        proxyB.points[0] = { 2.0f, -1.0f };
        proxyB.points[1] = { 2.0f, 1.0f };
        proxyB.count = 2;

        toiInput2 input{};
        input.proxyA = proxyA;
        input.proxyB = proxyB;

        input.sweepA = {};
        input.sweepB = { {}, {}, { -2.0f, 0.0f }, {}, {} };

        const toiOutput2 output = TimeOfImpact( input );

        assert( output.state == toiState2::Hit );
        assert( std::fabs( output.fraction - 0.5f ) < epsilon );
    }

    // 서로 멀어지는 circle은 전체 sweep 동안 separated.
    {
        toiInput2 input{};
        input.proxyA = MakeShapeProxy( circle2{ {}, 1.0f } );

        input.proxyB = MakeShapeProxy( circle2{ {}, 1.0f } );

        input.sweepA = {};
        input.sweepB = { {}, { 5.0f, 0.0f }, { 10.0f, 0.0f }, {}, {} };

        const toiOutput2 output = TimeOfImpact( input );

        assert( output.state == toiState2::Separated );
        assert( output.fraction == 1.0f );
    }

    // 시작부터 겹친 convex shape는 Overlapped.
    {
        toiInput2 input{};
        input.proxyA = MakeShapeProxy( MakeBox( { 1.0f, 1.0f } ) );

        input.proxyB = MakeShapeProxy( MakeBox( { 1.0f, 1.0f } ) );

        input.sweepA = {};
        input.sweepB = { {}, { 0.5f, 0.0f }, { 3.0f, 0.0f }, {}, {} };

        const toiOutput2 output = TimeOfImpact( input );

        assert( output.state == toiState2::Overlapped );
        assert( output.fraction == 0.0f );
    }

    // 이동이 없어도 회전 sweep만으로 접촉할 수 있어야 함.
    {
        toiInput2 input{};
        input.proxyA = MakeShapeProxy( circle2{ { 0.0f, 2.5f }, 0.25f } );

        input.proxyB = MakeShapeProxy( MakeBox( { 3.0f, 0.2f } ) );

        input.sweepA = {};

        input.sweepB = { {}, {}, {}, {}, rot2::FromRadians( 0.5f * 3.14159265358979323846f ) };

        const toiOutput2 output = TimeOfImpact( input );

        assert( output.state == toiState2::Hit );
        assert( output.fraction > 0.0f );
        assert( output.fraction < 1.0f );
    }

    // maxFraction 이전에 접촉하지 않으면 separated로 남음.
    {
        toiInput2 input{};
        input.proxyA = MakeShapeProxy( circle2{ {}, 1.0f } );

        input.proxyB = MakeShapeProxy( circle2{ {}, 1.0f } );

        input.sweepA = {};
        input.sweepB = { {}, { 5.0f, 0.0f }, { -5.0f, 0.0f }, {}, {} };

        input.maxFraction = 0.2f;

        const toiOutput2 output = TimeOfImpact( input );

        assert( output.state == toiState2::Separated );
        assert( output.fraction == 0.2f );
    }

    // 같은 월드 형상은 로컬 원점을 옮겨도 one-sided TOI 결과가 같아야 함.
    {
        toiInput2 input{};
        input.proxyA.points[0] = { -5.0f, 0.0f };
        input.proxyA.points[1] = { 5.0f, 0.0f };
        input.proxyA.count = 2;
        input.proxyA.oneSided = true;
        input.proxyB.points[0] = { -1.0f, -1.0f };
        input.proxyB.points[1] = { 1.0f, -1.0f };
        input.proxyB.points[2] = { 0.0f, 2.0f };
        input.proxyB.count = 3;
        input.sweepB.c1 = { 0.0f, -5.0f };
        input.sweepB.c2 = { 0.0f, 5.0f };

        const toiOutput2 reference = TimeOfImpact( input );
        if( reference.state != toiState2::Hit ) return 1;

        for( float offset : { 4128.0f, 4162.0f, 4196.0f } )
        {
            toiInput2 shifted = input;
            shifted.sweepB.localCenter = { offset, offset };
            for( int i = 0; i < shifted.proxyB.count; ++i )
            {
                shifted.proxyB.points[i] += shifted.sweepB.localCenter;
            }

            const toiOutput2 output = TimeOfImpact( shifted );
            if( output.state != toiState2::Hit || std::fabs( output.fraction - reference.fraction ) > epsilon ) return 1;
        }
    }

    return 0;
}
