#include <algorithm>
#include <array>
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

std::uint32_t g_randomState = 12345u;

float RandomFloat( float lower, float upper )
{
    g_randomState = 1664525u * g_randomState + 1013904223u;

    const float unit =
        static_cast<float>( g_randomState >> 8 ) *
        ( 1.0f / 16777216.0f );

    return lower + ( upper - lower ) * unit;
}

aabb2 RandomBox( float maxHalfExtent )
{
    const float x = RandomFloat( -50.0f, 50.0f );
    const float y = RandomFloat( -50.0f, 50.0f );
    const float hx = RandomFloat( 0.25f, maxHalfExtent );
    const float hy = RandomFloat( 0.25f, maxHalfExtent );

    return {
        { x - hx, y - hy },
        { x + hx, y + hy }
    };
}

void CheckQueryAgainstBruteForce(
    const DynamicTree& tree,
    const std::vector<std::int32_t>& proxyIds,
    const std::vector<aabb2>& boxes,
    const aabb2& query )
{
    std::vector<std::int32_t> actual;

    tree.Query(
        query,
        [&]( std::int32_t proxyId )
        {
            actual.push_back( proxyId );
            return true;
        }
    );

    std::sort( actual.begin(), actual.end() );

    std::vector<std::int32_t> expected;

    for( std::size_t i = 0; i < proxyIds.size(); ++i )
    {
        if( Overlaps( boxes[i], query ) )
        {
            expected.push_back( proxyIds[i] );
        }
    }

    std::sort( expected.begin(), expected.end() );
    assert( actual == expected );
}

int main()
{
    {
        // Box2D는 tree node 배열을 64-byte 정렬해 sibling pair가 cache-line 단위로 놓이게 함.
        TreeNodeStorage storage( 4 );

        const std::uintptr_t address =
            reinterpret_cast<std::uintptr_t>( storage.data() );

        assert( address % 64u == 0u );
        assert( sizeof( TreeNode ) == 32 );
    }

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

        std::vector<TreeNodeDebugInfo> debugNodes;

        pairTree.VisitNodes(
            [&]( const TreeNodeDebugInfo& info )
            {
                debugNodes.push_back( info );
            }
        );

        assert( debugNodes.size() == 3 );

        const auto rootIt =
            std::find_if(
                debugNodes.begin(),
                debugNodes.end(),
                []( const TreeNodeDebugInfo& info )
                {
                    return info.isRoot;
                }
            );

        assert( rootIt != debugNodes.end() );
        assert( rootIt->isLeaf == false );
        assert( rootIt->parentIndex == -1 );
        assert( rootIt->childPair >= 0 );
        assert( rootIt->height == 1 );

        std::vector<std::int32_t> debugShapeIndices;

        for( const TreeNodeDebugInfo& info : debugNodes )
        {
            if( info.isLeaf )
            {
                assert( info.proxyId >= 0 );
                assert( info.parentIndex == rootIt->nodeIndex );
                debugShapeIndices.push_back( info.shapeIndex );
            }
        }

        std::sort( debugShapeIndices.begin(), debugShapeIndices.end() );

        const std::vector<std::int32_t> expectedDebugShapeIndices{ 10, 20 };
        assert( debugShapeIndices == expectedDebugShapeIndices );
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

        // SAH 품질 지표는 root와 leaf를 제외한 internal node perimeter만 사용함.
        // root=24, (A+C)=6 -> 6 / 24 = 0.25
        assert( NearlyEqual( sahTree.GetAreaRatio(), 0.25f ) );
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
        // root=26, (B+C)=6 -> 6 / 26
        assert(
            NearlyEqual(
                moveTree.GetAreaRatio(),
                6.0f / 26.0f
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

    {
        // Box2D의 brute-force query 테스트와 같은 방식으로 tree 탐색 결과를 직접 비교함.
        constexpr std::int32_t PROXY_COUNT = 200;

        DynamicTree stressTree{};
        std::vector<std::int32_t> proxyIds;
        std::vector<aabb2> boxes;

        proxyIds.reserve( PROXY_COUNT );
        boxes.reserve( PROXY_COUNT );

        for( std::int32_t i = 0; i < PROXY_COUNT; ++i )
        {
            const aabb2 box = RandomBox( 2.0f );
            proxyIds.push_back( stressTree.CreateProxy( box, i ) );
            boxes.push_back( box );
        }

        assert( stressTree.Validate() );

        for( int i = 0; i < 64; ++i )
        {
            CheckQueryAgainstBruteForce(
                stressTree,
                proxyIds,
                boxes,
                RandomBox( 8.0f )
            );
        }

        // 이동 뒤에도 stable proxy id와 query 결과가 유지되어야 함.
        for( std::int32_t i = 0; i < PROXY_COUNT; i += 2 )
        {
            boxes[i] = RandomBox( 2.0f );
            stressTree.MoveProxy( proxyIds[i], boxes[i] );
        }

        assert( stressTree.Validate() );

        for( int i = 0; i < 64; ++i )
        {
            CheckQueryAgainstBruteForce(
                stressTree,
                proxyIds,
                boxes,
                RandomBox( 8.0f )
            );
        }

        assert( stressTree.Rebuild( true ) == PROXY_COUNT );
        assert( stressTree.Validate() );

        bool hasMovedNode = false;
        stressTree.VisitNodes(
            [&]( const TreeNodeDebugInfo& info )
            {
                hasMovedNode = hasMovedNode || info.isMoved;
            }
        );
        assert( hasMovedNode == false );

        for( int i = 0; i < 64; ++i )
        {
            CheckQueryAgainstBruteForce(
                stressTree,
                proxyIds,
                boxes,
                RandomBox( 8.0f )
            );
        }

        // 삭제로 free pair/proxy list에 hole을 만들고 매 단계 불변조건을 검사함.
        std::int32_t removedCount = 0;

        for( std::int32_t i = 0; i < PROXY_COUNT; i += 3 )
        {
            stressTree.DestroyProxy( proxyIds[i] );
            ++removedCount;
            assert( stressTree.Validate() );
        }

        // 삭제한 수만큼 다시 삽입해 free pair/proxy list를 끝까지 재사용함.
        for( std::int32_t i = 0; i < removedCount; ++i )
        {
            stressTree.CreateProxy( RandomBox( 2.0f ), 1000 + i );
            assert( stressTree.Validate() );
        }

        assert( stressTree.GetProxyCount() == PROXY_COUNT );

        stressTree.Rebuild( true );
        assert( stressTree.Validate() );
    }

    return 0;
}
