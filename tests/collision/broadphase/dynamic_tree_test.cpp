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

        // C는 B보다 A랑 묶이는게 훨씬 쌈.
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

        // C를 옮긴 뒤에는 B랑 묶이는게 제일 쌈.
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
        DynamicTree movedTree{};

        assert( movedTree.HasMoved() == false );

        const aabb2 boxA{
            { 0.0f, 0.0f },
            { 1.0f, 1.0f }
        };

        const aabb2 boxB{
            { 2.0f, 0.0f },
            { 3.0f, 1.0f }
        };

        const aabb2 movedA{
            { 4.0f, 0.0f },
            { 5.0f, 1.0f }
        };

        const std::int32_t proxyA = movedTree.CreateProxy( boxA, 1 );
        movedTree.CreateProxy( boxB, 2 );

        // 일반 DynamicTree API는 Box2D처럼 moved를 표시하지 않음.
        assert( movedTree.HasMoved() == false );

        movedTree.MoveProxy( proxyA, movedA );

        assert( movedTree.HasMoved() == false );

        // BroadPhase가 pair 생성을 요구하는 경로에서는 moved를 표시함.
        movedTree.MoveProxy( proxyA, boxA, true );

        assert( movedTree.HasMoved() );

        movedTree.ClearMoved();

        assert( movedTree.HasMoved() == false );
        assert( movedTree.Validate() );
    }

    {
        DynamicTree rebuildStateTree{};

        assert( rebuildStateTree.NeedsRebuild() == false );

        const aabb2 boxA{
            { 0.0f, 0.0f },
            { 1.0f, 1.0f }
        };

        const aabb2 boxB{
            { 3.0f, 0.0f },
            { 4.0f, 1.0f }
        };

        // 기존 tree 전체를 감싸는 큰 leaf를 넣으면 internal root가 새 pair 아래로 내려가
        // DFS 배열 순서가 깨지므로 moved가 없어도 rebuild가 필요해짐.
        const aabb2 enclosingBox{
            { -10.0f, -10.0f },
            {  10.0f,  10.0f }
        };

        rebuildStateTree.CreateProxy( boxA, 1 );
        rebuildStateTree.CreateProxy( boxB, 2 );

        assert( rebuildStateTree.HasMoved() == false );
        assert( rebuildStateTree.NeedsRebuild() == false );

        rebuildStateTree.CreateProxy( enclosingBox, 3 );

        assert( rebuildStateTree.HasMoved() == false );
        assert( rebuildStateTree.NeedsRebuild() );
        assert( rebuildStateTree.Validate() );
    }

    {
        DynamicTree movedRebuildTree{};

        const aabb2 boxA{
            { 0.0f, 0.0f },
            { 1.0f, 1.0f }
        };

        const aabb2 boxB{
            { 2.0f, 0.0f },
            { 3.0f, 1.0f }
        };

        const aabb2 movedA{
            { 4.0f, 0.0f },
            { 5.0f, 1.0f }
        };

        const std::int32_t proxyA = movedRebuildTree.CreateProxy( boxA, 1 );
        movedRebuildTree.CreateProxy( boxB, 2 );

        assert( movedRebuildTree.NeedsRebuild() == false );

        movedRebuildTree.MoveProxy( proxyA, movedA, true );

        assert( movedRebuildTree.HasMoved() );
        assert( movedRebuildTree.NeedsRebuild() );

        movedRebuildTree.ClearMoved();

        assert( movedRebuildTree.HasMoved() == false );
        assert( movedRebuildTree.NeedsRebuild() == false );
        assert( movedRebuildTree.Validate() );
    }

    {
        DynamicTree rebuildTree{};

        const aabb2 boxA{
            { 0.0f, 0.0f },
            { 1.0f, 1.0f }
        };

        const aabb2 boxB{
            { 2.0f, 0.0f },
            { 3.0f, 1.0f }
        };

        const aabb2 boxC{
            { 10.0f, 0.0f },
            { 11.0f, 1.0f }
        };

        const aabb2 boxD{
            { 12.0f, 0.0f },
            { 13.0f, 1.0f }
        };

        const aabb2 movedA{
            { 0.25f, 0.0f },
            { 1.25f, 1.0f }
        };

        const std::int32_t proxyA = rebuildTree.CreateProxy( boxA, 1 );
        const std::int32_t proxyB = rebuildTree.CreateProxy( boxB, 2 );
        const std::int32_t proxyC = rebuildTree.CreateProxy( boxC, 3 );
        const std::int32_t proxyD = rebuildTree.CreateProxy( boxD, 4 );

        // full rebuild는 모든 proxy를 build leaf로 사용하고 DFS 배열 순서를 복구함.
        assert( rebuildTree.Rebuild( true ) == 4 );
        assert( rebuildTree.NeedsRebuild() == false );
        assert( rebuildTree.Validate() );

        // 변화가 없으면 partial rebuild는 아무 작업도 하지 않음.
        assert( rebuildTree.Rebuild( false ) == 0 );

        rebuildTree.MoveProxy( proxyA, movedA, true );

        assert( rebuildTree.HasMoved() );
        assert( rebuildTree.NeedsRebuild() );

        const std::size_t partialCount = rebuildTree.Rebuild( false );

        // moved branch만 펼치고 untouched subtree는 하나의 build leaf로 유지함.
        assert( partialCount > 0 );
        assert( partialCount < rebuildTree.GetProxyCount() );

        assert( rebuildTree.HasMoved() == false );
        assert( rebuildTree.NeedsRebuild() == false );
        assert( rebuildTree.Validate() );

        // rebuild 뒤에도 stable proxy id와 각 proxy AABB는 유지되어야 함.
        assert( NearlyEqual( rebuildTree.GetProxyAABB( proxyA ), movedA ) );
        assert( NearlyEqual( rebuildTree.GetProxyAABB( proxyB ), boxB ) );
        assert( NearlyEqual( rebuildTree.GetProxyAABB( proxyC ), boxC ) );
        assert( NearlyEqual( rebuildTree.GetProxyAABB( proxyD ), boxD ) );
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

        // 한쪽으로 계속 쌓이는걸 local rotation으로 줄여줌.
        assert( balancedTree.GetHeight() == 3 );
        assert( balancedTree.Validate() );
    }

    return 0;
}
