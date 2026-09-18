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


    {
        DynamicTree pairTree{};

        const aabb2 boxA{
            { -2.0f, -1.0f },
            {  0.0f,  1.0f }
        };

        const aabb2 boxB{
            {  1.0f, -2.0f },
            {  3.0f,  2.0f }
        };

        const int proxyA = pairTree.CreateProxy( boxA, 10 );
        const int proxyB = pairTree.CreateProxy( boxB, 20 );

        assert( proxyA >= 0 );

        if( proxyB < 0 )
        {
            return 1;
        }

        assert( proxyA != proxyB );

        assert( pairTree.GetProxyCount() == 2 );
        assert( pairTree.GetHeight() == 1 );

        assert( NearlyEqual( pairTree.GetProxyAABB( proxyA ), boxA ) );
        assert( NearlyEqual( pairTree.GetProxyAABB( proxyB ), boxB ) );
    }


    {
        DynamicTree sahTree{};

        const aabb2 boxA{
            { 0.0f, 0.0f },
            { 1.0f, 1.0f }
        };

        const aabb2 boxB{
            { 10.0f, 0.0f },
            { 11.0f, 1.0f }
        };

        const aabb2 boxC{
            { 1.0f, 0.0f },
            { 2.0f, 1.0f }
        };

        const int proxyA = sahTree.CreateProxy( boxA, 1 );
        const int proxyB = sahTree.CreateProxy( boxB, 2 );
        const int proxyC = sahTree.CreateProxy( boxC, 3 );

        assert( proxyA >= 0 );
        assert( proxyB >= 0 );

        if( proxyC < 0 )
        {
            return 2;
        }

        assert( sahTree.GetProxyCount() == 3 );
        assert( sahTree.GetHeight() == 2 );

        // C는 B보다 A와 묶일 때 AABB 증가 비용이 훨씬 작다.
        // root=24, (A+C)=6, leaf 3개=12 -> 42 / 24 = 1.75
        assert( NearlyEqual( sahTree.GetAreaRatio(), 1.75f ) );
    }


    {
        DynamicTree removalTree{};

        const aabb2 boxA{
            { 0.0f, 0.0f },
            { 1.0f, 1.0f }
        };

        const aabb2 boxB{
            { 10.0f, 0.0f },
            { 11.0f, 1.0f }
        };

        const aabb2 boxC{
            { 1.0f, 0.0f },
            { 2.0f, 1.0f }
        };

        const int proxyA = removalTree.CreateProxy( boxA, 1 );
        const int proxyB = removalTree.CreateProxy( boxB, 2 );
        const int proxyC = removalTree.CreateProxy( boxC, 3 );

        removalTree.DestroyProxy( proxyC );

        assert( removalTree.GetProxyCount() == 2 );
        assert( removalTree.GetHeight() == 1 );
        assert( NearlyEqual( removalTree.GetProxyAABB( proxyA ), boxA ) );
        assert( NearlyEqual( removalTree.GetProxyAABB( proxyB ), boxB ) );

        removalTree.DestroyProxy( proxyA );

        assert( removalTree.GetProxyCount() == 1 );
        assert( removalTree.GetHeight() == 0 );
        assert( NearlyEqual( removalTree.GetProxyAABB( proxyB ), boxB ) );

        removalTree.DestroyProxy( proxyB );

        assert( removalTree.GetProxyCount() == 0 );
        assert( removalTree.GetHeight() == 0 );
        assert( NearlyEqual( removalTree.GetAreaRatio(), 0.0f ) );
    }

    return 0;
}
