#include <cassert>
#include <cmath>

#include "collision/broadphase/dynamicTree.h"

using namespace zonai;

bool NearlyEqual(
    float a,
    float b,
    float epsilon = 1e-5f )
{
    return std::fabs( a - b ) <= epsilon;
}

bool NearlyEqual(
    const aabb2& a,
    const aabb2& b,
    float epsilon = 1e-5f )
{
    return
        NearlyEqual( a.min.x, b.min.x, epsilon ) &&
        NearlyEqual( a.min.y, b.min.y, epsilon ) &&
        NearlyEqual( a.max.x, b.max.x, epsilon ) &&
        NearlyEqual( a.max.y, b.max.y, epsilon );
}

int main()
{
    DynamicTree tree{};

    assert( tree.GetProxyCount() == 0 );

    const aabb2 box{
        { -1.0f, -2.0f },
        {  3.0f,  4.0f }
    };

    const int proxyId = tree.CreateProxy( box, 7 );

    assert( proxyId >= 0 );
    assert( tree.GetProxyCount() == 1 );
    assert( tree.GetHeight() == 0 );
    assert( NearlyEqual( tree.GetProxyAABB( proxyId ), box ) );

    return 0;
}
