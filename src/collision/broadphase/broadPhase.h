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

using ProxyKey = std::int32_t;

template <typename Callback>
concept BroadPhasePairCallback =
    requires( Callback& callback, std::int32_t shapeIndexA, std::int32_t shapeIndexB )
    {
        { callback( shapeIndexA, shapeIndexB ) } -> std::same_as<void>;
    };

constexpr ProxyKey MakeProxyKey( std::int32_t proxyId, BodyType type )
{
    constexpr std::uint32_t TYPE_BITS = 2;

    const std::uint32_t id = static_cast<std::uint32_t>( proxyId );
    const std::uint32_t bodyType = static_cast<std::uint32_t>( type );

    return static_cast<ProxyKey>( ( id << TYPE_BITS ) | bodyType );
}

constexpr std::int32_t GetProxyId( ProxyKey key )
{
    constexpr std::uint32_t TYPE_BITS = 2;

    return static_cast<std::int32_t>( static_cast<std::uint32_t>( key ) >> TYPE_BITS );
}

constexpr BodyType GetProxyType( ProxyKey key )
{
    constexpr std::uint32_t TYPE_MASK = 0x3u;

    return static_cast<BodyType>( static_cast<std::uint32_t>( key ) & TYPE_MASK );
}

class BroadPhase
{
public:
    ProxyKey CreateProxy( BodyType type, const aabb2& aabb, std::int32_t shapeIndex, bool forcePairCreation = false );
    void DestroyProxy( ProxyKey proxyKey );
    void MoveProxy( ProxyKey proxyKey, const aabb2& aabb );

    template <BroadPhasePairCallback Callback>
    void FindDynamicSelfPairs( std::span<std::int32_t> movedSiblings, Callback&& callback ) const
    {
        const DynamicTree& tree = GetTree( BodyType::Dynamic );
        const std::size_t movedCount = GatherMovedSiblings( tree, movedSiblings );

        for( std::size_t i = 0; i < movedCount; ++i )
        {
            const std::int32_t pair = movedSiblings[i];
            CollideCrossPairs( tree, tree, tree.nodes_[pair], tree.nodes_[pair + 1], callback );
        }
    }

    DynamicTree& GetTree( BodyType type );
    const DynamicTree& GetTree( BodyType type ) const;

private:
    struct NodeIndexPair
    {
        std::int32_t a = 0;
        std::int32_t b = 0;
    };

    static constexpr std::size_t BODY_TYPE_COUNT = static_cast<std::size_t>( BodyType::Count );

    static bool TestPair( const TreeNode& nodeA, const TreeNode& nodeB );
    static std::size_t GatherMovedSiblings( const DynamicTree& tree, std::span<std::int32_t> pairIndices );

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

    template <BroadPhasePairCallback Callback>
    static void CollideProxyAndSubtree(
        const TreeNode& proxy,
        const DynamicTree& tree,
        std::int32_t pair,
        Callback& callback )
    {
        const std::uint32_t proxyMoved = proxy.flagIndex & DynamicTree::TREE_MOVED_NODE;
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
                    AddCandidatePair( proxy, node, callback );
                    continue;
                }

                assert( stackCount < stack.size() );
                stack[stackCount++] = DynamicTree::GetChildPair( node );
            }
        }
    }

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
            assert( stackCount < stack.size() );
            stack[stackCount++] = {
                DynamicTree::GetChildPair( nodeA ),
                DynamicTree::GetChildPair( nodeB )
            };
        }
    }

    template <BroadPhasePairCallback Callback>
    static void CollideCrossPairs(
        const DynamicTree& treeA,
        const DynamicTree& treeB,
        const TreeNode& subtreeA,
        const TreeNode& subtreeB,
        Callback& callback )
    {
        std::array<NodeIndexPair, DynamicTree::TREE_STACK_SIZE> stack{};
        std::size_t stackCount = 0;

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
