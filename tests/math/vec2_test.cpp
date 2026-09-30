#include <cassert>
#include <cmath>

#include "math/vec2.h"

using namespace zonai;

namespace
{

bool NearlyEqual(
    float a,
    float b,
    float epsilon = 1e-5f )
{
    return std::fabs( a - b ) <= epsilon;
}

} // namespace

int main()
{
    {
        const vec2 a{ 1.0f, 2.0f };
        const vec2 b{ 3.0f, 4.0f };

        const vec2 result = a + b;

        assert( result.x == 4.0f );
        assert( result.y == 6.0f );
    }

    {
        const vec2 a{ 4.0f, 6.0f };
        const vec2 b{ 1.0f, 2.0f };

        const vec2 result = a - b;

        assert( result.x == 3.0f );
        assert( result.y == 4.0f );
    }

    {
        const vec2 v{ 3.0f, 4.0f };

        assert( NearlyEqual( Length( v ), 5.0f ) );
    }

    {
        const vec2 a{ 1.0f, 2.0f };
        const vec2 b{ 3.0f, 4.0f };

        assert( NearlyEqual( Dot( a, b ), 11.0f ) );
        assert( NearlyEqual( Cross( a, b ), -2.0f ) );
    }

    {
        const vec2 v{ 3.0f, 4.0f };

        const vec2 scalarCrossVector = Cross( 2.0f, v );
        assert( NearlyEqual( scalarCrossVector.x, -8.0f ) );
        assert( NearlyEqual( scalarCrossVector.y, 6.0f ) );

        const vec2 vectorCrossScalar = Cross( v, 2.0f );
        assert( NearlyEqual( vectorCrossScalar.x, 8.0f ) );
        assert( NearlyEqual( vectorCrossScalar.y, -6.0f ) );
    }

    {
        const vec2 v{ 3.0f, 4.0f };

        const vec2 normalized = Normalize( v );

        assert( NearlyEqual( normalized.x, 0.6f ) );
        assert( NearlyEqual( normalized.y, 0.8f ) );
        assert( NearlyEqual( Length( normalized ), 1.0f ) );
    }

    {
        vec2 zero{};

        const vec2 normalized = Normalize( zero );

        assert( normalized.x == 0.0f );
        assert( normalized.y == 0.0f );
    }

    return 0;
}
