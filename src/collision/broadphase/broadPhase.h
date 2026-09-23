#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>

#include "collision/broadphase/dynamicTree.h"
#include "dynamics/bodyType.h"

namespace zonai
{

// 상위 bit에는 proxy id, 하위 2bit에는 BodyType을 저장함.
using ProxyKey = std::int32_t;

// BroadPhase가 찾은 후보 shape pair를 전달하는 callback 규약.
template <typename Callback>
concept BroadPhasePairCallback =
    requires( Callback& callback, std::int32_t shapeIndexA, std::int32_t shapeIndexB )
    {
        { callback( shapeIndexA, shapeIndexB ) } -> std::same_as<void>;
    };

// stable proxy id와 body type을 하나의 key로 묶음.
constexpr ProxyKey MakeProxyKey( std::int32_t proxyId, BodyType type )
{
    constexpr std::uint32_t TYPE_BITS = 2;

    const std::uint32_t id = static_cast<std::uint32_t>( proxyId );
    const std::uint32_t bodyType = static_cast<std::uint32_t>( type );

    return static_cast<ProxyKey>( ( id << TYPE_BITS ) | bodyType );
}

// proxy key에서 DynamicTree가 사용하는 stable proxy id를 꺼냄.
constexpr std::int32_t GetProxyId( ProxyKey key )
{
    constexpr std::uint32_t TYPE_BITS = 2;

    return static_cast<std::int32_t>( static_cast<std::uint32_t>( key ) >> TYPE_BITS );
}

// proxy key에서 해당 proxy가 속한 body type을 꺼냄.
constexpr BodyType GetProxyType( ProxyKey key )
{
    constexpr std::uint32_t TYPE_MASK = 0x3u;

    return static_cast<BodyType>( static_cast<std::uint32_t>( key ) & TYPE_MASK );
}

class BroadPhase
{
public:
    // body type에 맞는 tree에 proxy를 만들고 type 정보가 포함된 key를 반환함.
    ProxyKey CreateProxy( BodyType type, const aabb2& aabb, std::int32_t shapeIndex, bool forcePairCreation = false );

    // key로 tree와 proxy id를 찾아 proxy를 제거함.
    void DestroyProxy( ProxyKey proxyKey );

    // proxy AABB를 갱신하고 새 pair 탐색 대상이 되도록 moved 처리함.
    void MoveProxy( ProxyKey proxyKey, const aabb2& aabb );

    // moved sibling pair를 seed로 dynamic tree 내부의 새 충돌 후보를 찾음.
    // callback으로 전달되는 shape pair는 BroadPhase 후보이며 실제 충돌이 확정된 것은 아님.
    template <BroadPhasePairCallback Callback>
    void FindDynamicSelfPairs( std::span<std::int32_t> movedSiblings, Callback&& callback ) const
    {
        const DynamicTree& tree = GetTree( BodyType::Dynamic );

        // moved node가 포함된 sibling pair만 모아 self collision의 시작점으로 사용함.
        const std::size_t movedCount = GatherMovedSiblings( tree, movedSiblings );

        for( std::size_t i = 0; i < movedCount; ++i )
        {
            const std::int32_t pair = movedSiblings[i];

            // 같은 parent를 공유하는 두 subtree를 서로 충돌시켜 중복 없이 후보를 찾음.
            CollideCrossPairs( tree, tree, tree.nodes_[pair], tree.nodes_[pair + 1], callback );
        }
    }

    DynamicTree& GetTree( BodyType type );
    const DynamicTree& GetTree( BodyType type ) const;

private:
    // 두 subtree에서 다음에 비교할 child pair의 시작 index를 묶어 저장함.
    struct NodeIndexPair
    {
        std::int32_t a = 0;
        std::int32_t b = 0;
    };

    static constexpr std::size_t BODY_TYPE_COUNT = static_cast<std::size_t>( BodyType::Count );

    // 둘 중 하나가 moved이고 AABB가 겹치는지 확인함.
    static bool TestPair( const TreeNode& nodeA, const TreeNode& nodeB );

    // moved node가 포함된 sibling pair의 시작 index를 모음.
    static std::size_t GatherMovedSiblings( const DynamicTree& tree, std::span<std::int32_t> pairIndices );

    // leaf 두 개가 만나면 shape index 순서를 정규화해서 후보 pair를 전달함.
    template <BroadPhasePairCallback Callback>
    static void AddCandidatePair( const TreeNode& nodeA, const TreeNode& nodeB, Callback& callback )
    {
        const std::int32_t shapeIndexA = nodeA.shapeIndex;
        const std::int32_t shapeIndexB = nodeB.shapeIndex;

        if( shapeIndexA < shapeIndexB )
        {
            callback( shapeIndexA, shapeIndexB );
        }
        else
        {
            callback( shapeIndexB, shapeIndexA );
        }
    }

    // leaf 하나와 subtree 하나를 비교하며 겹치는 leaf까지 내려감.
    template <BroadPhasePairCallback Callback>
    static void CollideProxyAndSubtree(
        const TreeNode& proxy,
        const DynamicTree& tree,
        std::int32_t pair,
        Callback& callback )
    {
        // proxy가 moved면 상대 subtree의 모든 겹치는 node가 대상이고,
        // 그렇지 않으면 상대 node가 moved인 경로만 탐색함.
        const std::uint32_t proxyMoved = proxy.flagIndex & DynamicTree::TREE_MOVED_NODE;

        // pair 탐색 중 heap allocation을 피하려고 고정 크기 stack을 사용함.
        std::array<std::int32_t, DynamicTree::TREE_STACK_SIZE> stack{};
        std::size_t stackCount = 0;

        stack[stackCount++] = pair;

        while( stackCount > 0 )
        {
            pair = stack[--stackCount];

            for( std::int32_t i = 0; i < 2; ++i )
            {
                const TreeNode& node = tree.nodes_[pair + i];

                if( ( ( node.flagIndex | proxyMoved ) & DynamicTree::TREE_MOVED_NODE ) == 0 ||
                    !Overlaps( proxy.aabb, node.aabb ) )
                {
                    continue;
                }

                if( DynamicTree::IsLeaf( node ) )
                {
                    // AABB가 겹치는 leaf를 찾았으므로 NarrowPhase 후보로 넘김.
                    AddCandidatePair( proxy, node, callback );
                    continue;
                }

                assert( stackCount < stack.size() );
                stack[stackCount++] = DynamicTree::GetChildPair( node );
            }
        }
    }

    // node 두 개의 상태에 따라 후보 보고, leaf-subtree 탐색, subtree 분할 중 하나를 수행함.
    template <BroadPhasePairCallback Callback>
    static void VisitPair(
        const DynamicTree& treeA,
        const DynamicTree& treeB,
        const TreeNode& nodeA,
        const TreeNode& nodeB,
        std::array<NodeIndexPair, DynamicTree::TREE_STACK_SIZE>& stack,
        std::size_t& stackCount,
        Callback& callback )
    {
        if( !TestPair( nodeA, nodeB ) )
        {
            return;
        }

        const bool leafA = DynamicTree::IsLeaf( nodeA );
        const bool leafB = DynamicTree::IsLeaf( nodeB );

        if( leafA && leafB )
        {
            // 둘 다 leaf면 더 내려갈 곳이 없으므로 후보 pair를 보고함.
            AddCandidatePair( nodeA, nodeB, callback );
        }
        else if( leafA )
        {
            CollideProxyAndSubtree( nodeA, treeB, DynamicTree::GetChildPair( nodeB ), callback );
        }
        else if( leafB )
        {
            CollideProxyAndSubtree( nodeB, treeA, DynamicTree::GetChildPair( nodeA ), callback );
        }
        else
        {
            // 둘 다 internal이면 각 child 조합을 다음 순회에서 비교함.
            assert( stackCount < stack.size() );
            stack[stackCount++] = {
                DynamicTree::GetChildPair( nodeA ),
                DynamicTree::GetChildPair( nodeB )
            };
        }
    }

    // 두 subtree 사이에서 moved 조건과 AABB overlap을 만족하는 leaf pair를 찾음.
    // sibling subtree끼리 비교하면 같은 self pair를 중복 생성하지 않음.
    template <BroadPhasePairCallback Callback>
    static void CollideCrossPairs(
        const DynamicTree& treeA,
        const DynamicTree& treeB,
        const TreeNode& subtreeA,
        const TreeNode& subtreeB,
        Callback& callback )
    {
        // 재귀 대신 고정 크기 stack으로 subtree 조합을 순회함.
        std::array<NodeIndexPair, DynamicTree::TREE_STACK_SIZE> stack{};
        std::size_t stackCount = 0;

        // 처음 두 subtree를 검사하고 internal끼리면 child pair가 stack에 추가됨.
        VisitPair( treeA, treeB, subtreeA, subtreeB, stack, stackCount, callback );

        while( stackCount > 0 )
        {
            const NodeIndexPair pair = stack[--stackCount];

            for( std::int32_t i = 0; i < 2; ++i )
            {
                for( std::int32_t j = 0; j < 2; ++j )
                {
                    VisitPair(
                        treeA,
                        treeB,
                        treeA.nodes_[pair.a + i],
                        treeB.nodes_[pair.b + j],
                        stack,
                        stackCount,
                        callback
                    );
                }
            }
        }
    }

    // body type마다 독립된 DynamicTree를 사용함.
    std::array<DynamicTree, BODY_TYPE_COUNT> trees_{};
};

} // namespace zonai
