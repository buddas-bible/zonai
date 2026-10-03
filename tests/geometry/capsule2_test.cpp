#include <cassert>
#include <cmath>

#include "geometry/capsule2.h"

using namespace zonai;

namespace
{

bool NearlyEqual( float a, float b, float epsilon = 1e-5f )
{
    return std::fabs( a - b ) <= epsilon;
}

bool NearlyEqual( const vec2& a, const vec2& b, float epsilon = 1e-5f )
{
    return NearlyEqual( a.x, b.x, epsilon ) &&
        NearlyEqual( a.y, b.y, epsilon );
}

} // namespace

int main()
{
    const capsule2 capsule{
        { -2.0f, 1.0f },
        {  2.0f, 1.0f },
        0.5f
    };

    const aabb2 bounds = ComputeAABB( capsule );
    assert( NearlyEqual( bounds.min, { -2.5f, 0.5f } ) );
    assert( NearlyEqual( bounds.max, {  2.5f, 1.5f } ) );

    assert( Contains( capsule, { 0.0f, 1.0f } ) );
    assert( Contains( capsule, { -2.5f, 1.0f } ) );
    assert( Contains( capsule, { 2.0f, 1.5f } ) );
    assert( !Contains( capsule, { 0.0f, 1.6f } ) );
    assert( !Contains( capsule, { 2.6f, 1.0f } ) );

    return 0;
}
