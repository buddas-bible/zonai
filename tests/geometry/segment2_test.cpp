#include <cassert>
#include <cmath>

#include "geometry/segment2.h"

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
    const segment2 segment{
        { -2.0f, 1.0f },
        {  2.0f, 1.0f }
    };

    assert( NearlyEqual( Direction( segment ), { 4.0f, 0.0f } ) );
    assert( NearlyEqual( LengthSquared( segment ), 16.0f ) );
    assert( NearlyEqual( Length( segment ), 4.0f ) );

    const aabb2 bounds = ComputeAABB( segment );
    assert( NearlyEqual( bounds.min, { -2.0f, 1.0f } ) );
    assert( NearlyEqual( bounds.max, {  2.0f, 1.0f } ) );

    assert( NearlyEqual( ClosestPoint( segment, { 0.5f, 3.0f } ), { 0.5f, 1.0f } ) );
    assert( NearlyEqual( ClosestPoint( segment, { -4.0f, 1.0f } ), segment.a ) );
    assert( NearlyEqual( ClosestPoint( segment, {  4.0f, 1.0f } ), segment.b ) );
    assert( NearlyEqual( DistanceSquared( segment, { 0.0f, 3.0f } ), 4.0f ) );

    const segment2 pointSegment{
        { 3.0f, -1.0f },
        { 3.0f, -1.0f }
    };

    assert( NearlyEqual( ClosestPoint( pointSegment, { 8.0f, 2.0f } ), pointSegment.a ) );

    return 0;
}
