#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <vector>

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


    {
        DynamicTree moveTree{};

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

        const aabb2 movedC{
            { 11.0f, 0.0f },
            { 12.0f, 1.0f }
        };

        const int proxyA = moveTree.CreateProxy( boxA, 1 );
        const int proxyB = moveTree.CreateProxy( boxB, 2 );
        const int proxyC = moveTree.CreateProxy( boxC, 3 );

        moveTree.MoveProxy( proxyC, movedC );

        assert( moveTree.GetProxyCount() == 3 );
        assert( moveTree.GetHeight() == 2 );

        assert( NearlyEqual( moveTree.GetProxyAABB( proxyA ), boxA ) );
        assert( NearlyEqual( moveTree.GetProxyAABB( proxyB ), boxB ) );
        assert( NearlyEqual( moveTree.GetProxyAABB( proxyC ), movedC ) );

        // 이동 후에는 C가 B와 묶이는 것이 가장 작다.
        // root=26, (B+C)=6, leaf 3개=12 -> 44 / 26
        assert(
            NearlyEqual(
                moveTree.GetAreaRatio(),
                44.0f / 26.0f
            )
        );
    }

    {
        DynamicTree queryTree{};

        const aabb2 boxA{
            { 0.0f, 0.0f },
            { 1.0f, 1.0f }
        };

        const aabb2 boxB{
            { 5.0f, 0.0f },
            { 6.0f, 1.0f }
        };

        const aabb2 boxC{
            { 10.0f, 0.0f },
            { 11.0f, 1.0f }
        };

        const std::int32_t proxyA = queryTree.CreateProxy( boxA, 1 );
        const std::int32_t proxyB = queryTree.CreateProxy( boxB, 2 );
        queryTree.CreateProxy( boxC, 3 );

        const aabb2 queryBox{
            { -1.0f, -1.0f },
            {  7.0f,  2.0f }
        };

        std::vector<std::int32_t> hits;

        queryTree.Query(
            queryBox,
            [&]( std::int32_t proxyId )
            {
                hits.push_back( proxyId );
                return true;
            }
        );

        std::sort( hits.begin(), hits.end() );

        std::vector<std::int32_t> expected{
            proxyA,
            proxyB
        };
        std::sort( expected.begin(), expected.end() );

        assert( hits == expected );
    }

    {
        DynamicTree balancedTree{};

        for( int i = 0; i < 8; ++i )
        {
            const float x = static_cast<float>( i );

            balancedTree.CreateProxy(
                {
                    { x, 0.0f },
                    { x + 1.0f, 1.0f }
                },
                i
            );
        }

        // 순차 삽입으로 생기는 편향을 local rotation이 줄여야 한다.
        assert( balancedTree.GetHeight() == 3 );
        assert( balancedTree.Validate() );
    }

    return 0;
}
