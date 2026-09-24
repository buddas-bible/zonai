#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <unordered_set>

#include "collision/broadphase/dynamicTree.h"
#include "collision/broadphase/hashSet.h"
#include "dynamics/bodyType.h"

namespace zonai
{

// 상위 bit에는 proxy id, 하위 2bit에는 BodyType을 저장함.
using ProxyKey = std::int32_t;

// 두 shape index를 순서와 무관하게 식별하는 64-bit key.
using ShapePairKey = std::uint64_t;

// 작은 shape index를 상위 32bit에 두어 (A, B)와 (B, A)가 같은 key가 되게 함.
constexpr ShapePairKey MakeShapePairKey( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
{
    const std::uint32_t indexA = static_cast<std::uint32_t>( shapeIndexA );
    const std::uint32_t indexB = static_cast<std::uint32_t>( shapeIndexB );
    const std::uint32_t lower = indexA < indexB ? indexA : indexB;
    const std::uint32_t upper = indexA < indexB ? indexB : indexA;

    return static_cast<ShapePairKey>( lower ) << 32 | static_cast<ShapePairKey>( upper );
}

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
private:
    struct CandidatePair
    {
        std::int32_t shapeIndexA = 0;
        std::int32_t shapeIndexB = 0;
    };

    static constexpr std::size_t CANDIDATE_BATCH_SIZE = 32;

    // leaf pair를 고정 크기 batch에 모은 뒤 기존 Contact pair를 한 번에 걸러냄.
    template <typename Callback>
    struct PairContext
    {
        const HashSet& pairSet;
        Callback& callback;
        std::array<CandidatePair, CANDIDATE_BATCH_SIZE> batch{};
        std::size_t batchCount = 0;

        void Add( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
        {
            CandidatePair& candidate = batch[batchCount++];

            if( shapeIndexA < shapeIndexB )
            {
                candidate = { shapeIndexA, shapeIndexB };
            }
            else
            {
                candidate = { shapeIndexB, shapeIndexA };
            }

            if( batchCount == batch.size() )
            {
                Flush();
            }
        }

        void Flush()
        {
            const std::size_t count = batchCount;
            batchCount = 0;

            for( std::size_t i = 0; i < count; ++i )
            {
                const CandidatePair& candidate = batch[i];
                const ShapePairKey pairKey = MakeShapePairKey( candidate.shapeIndexA, candidate.shapeIndexB );

                // 이미 Contact가 존재하는 pair는 새 BroadPhase 후보로 다시 보고하지 않음.
                if( pairSet.Contains( pairKey ) )
                {
                    continue;
                }

                callback( candidate.shapeIndexA, candidate.shapeIndexB );
            }
        }
    };

public:
    // body type에 맞는 tree에 proxy를 만들고 type 정보가 포함된 key를 반환함.
    ProxyKey CreateProxy( BodyType type, const aabb2& aabb, std::int32_t shapeIndex, bool forcePairCreation = false );

    // key로 tree와 proxy id를 찾아 proxy를 제거함.
    void DestroyProxy( ProxyKey proxyKey );

    // proxy AABB를 갱신하고 새 pair 탐색 대상이 되도록 moved 처리함.
    void MoveProxy( ProxyKey proxyKey, const aabb2& aabb );

    // Contact가 존재하는 shape pair를 추적함. 새 pair면 false, 이미 있으면 true를 반환함.
    bool AddPair( ShapePairKey pairKey );

    // Contact가 사라진 shape pair를 추적 목록에서 제거함.
    bool RemovePair( ShapePairKey pairKey );

    // 이미 Contact가 존재하는 shape pair인지 확인함.
    bool HasPair( ShapePairKey pairKey ) const;

    // 세 BroadPhase 탐색 경로를 한 번에 실행해 새 충돌 후보를 모음.
    // moved flag 소비와 tree rebuild는 별도 단계에서 처리함.
    template <BroadPhasePairCallback Callback>
    void FindPairs( std::span<std::int32_t> movedSiblings, Callback&& callback ) const
    {
        // Box2D처럼 dynamic tree를 static / kinematic tree와 먼저 교차 검사함.
        FindDynamicStaticPairs( callback );
        FindDynamicKinematicPairs( callback );

        // cross pair를 찾은 뒤 dynamic tree 내부의 self pair를 검사함.
        FindDynamicSelfPairs( movedSiblings, callback );
    }

    // moved sibling pair를 seed로 dynamic tree 내부의 새 충돌 후보를 찾음.
    // callback으로 전달되는 shape pair는 BroadPhase 후보이며 실제 충돌이 확정된 것은 아님.
    template <BroadPhasePairCallback Callback>
    void FindDynamicSelfPairs( std::span<std::int32_t> movedSiblings, Callback&& callback ) const
    {
        const DynamicTree& tree = GetTree( BodyType::Dynamic );
        PairContext<Callback> context{ pairSet_, callback };

        // moved node가 포함된 sibling pair만 모아 self collision의 시작점으로 사용함.
        const std::size_t movedCount = GatherMovedSiblings( tree, movedSiblings );

        for( std::size_t i = 0; i < movedCount; ++i )
        {
            const std::int32_t pair = movedSiblings[i];

            // 같은 parent를 공유하는 두 subtree를 서로 충돌시켜 중복 없이 후보를 찾음.
            CollideCrossPairs( tree, tree, tree.nodes_[pair], tree.nodes_[pair + 1], context );
        }

        // 32개 미만으로 남은 마지막 후보들도 처리함.
        context.Flush();
    }

    // dynamic tree와 static tree의 겹치는 subtree seed를 찾아 새 충돌 후보를 보고함.
    template <BroadPhasePairCallback Callback>
    void FindDynamicStaticPairs( Callback&& callback ) const
    {
        const DynamicTree& dynamicTree = GetTree( BodyType::Dynamic );
        const DynamicTree& staticTree = GetTree( BodyType::Static );
        PairContext<Callback> context{ pairSet_, callback };

        std::array<TreeNodePair, CROSS_SEED_COUNT> seeds{};
        const std::size_t seedCount = GatherCrossSeeds( dynamicTree, staticTree, seeds );

        // 병렬 작업 시스템이 생기기 전까지는 seed를 현재 스레드에서 순서대로 처리함.
        for( std::size_t i = 0; i < seedCount; ++i )
        {
            CollideCrossPairs( dynamicTree, staticTree, seeds[i].a, seeds[i].b, context );
        }

        context.Flush();
    }

    // dynamic tree와 kinematic tree의 겹치는 subtree seed를 찾아 새 충돌 후보를 보고함.
    template <BroadPhasePairCallback Callback>
    void FindDynamicKinematicPairs( Callback&& callback ) const
    {
        const DynamicTree& dynamicTree = GetTree( BodyType::Dynamic );
        const DynamicTree& kinematicTree = GetTree( BodyType::Kinematic );
        PairContext<Callback> context{ pairSet_, callback };

        std::array<TreeNodePair, CROSS_SEED_COUNT> seeds{};
        const std::size_t seedCount = GatherCrossSeeds( dynamicTree, kinematicTree, seeds );

        // static cross pair와 같은 탐색 경로를 kinematic tree에도 재사용함.
        for( std::size_t i = 0; i < seedCount; ++i )
        {
            CollideCrossPairs( dynamicTree, kinematicTree, seeds[i].a, seeds[i].b, context );
        }

        context.Flush();
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

    // cross-tree 탐색을 병렬 작업 단위로 나누기 위한 subtree node 쌍.
    struct TreeNodePair
    {
        TreeNode a{};
        TreeNode b{};
    };

    static constexpr std::size_t BODY_TYPE_COUNT = static_cast<std::size_t>( BodyType::Count );
    static constexpr std::size_t CROSS_SEED_COUNT = 64;

    static_assert( ( CROSS_SEED_COUNT & ( CROSS_SEED_COUNT - 1 ) ) == 0 );

    // 둘 중 하나가 moved이고 AABB가 겹치는지 확인함.
    static bool TestPair( const TreeNode& nodeA, const TreeNode& nodeB );

    // moved node가 포함된 sibling pair의 시작 index를 모음.
    static std::size_t GatherMovedSiblings( const DynamicTree& tree, std::span<std::int32_t> pairIndices );

    // 서로 다른 tree의 root부터 BFS로 내려가며 겹치는 subtree 조합을 seed로 모음.
    static std::size_t GatherCrossSeeds(
        const DynamicTree& treeA,
        const DynamicTree& treeB,
        std::span<TreeNodePair> seeds );

    // leaf 두 개가 만나면 shape index 순서를 정규화해서 candidate batch에 추가함.
    template <typename Callback>
    static void AddCandidatePair(
        const TreeNode& nodeA,
        const TreeNode& nodeB,
        PairContext<Callback>& context )
    {
        context.Add( nodeA.shapeIndex, nodeB.shapeIndex );
    }

    // leaf 하나와 subtree 하나를 비교하며 겹치는 leaf까지 내려감.
    template <typename Callback>
    static void CollideProxyAndSubtree(
        const TreeNode& proxy,
        const DynamicTree& tree,
        std::int32_t pair,
        PairContext<Callback>& context )
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
                    AddCandidatePair( proxy, node, context );
                    continue;
                }

                assert( stackCount < stack.size() );
                stack[stackCount++] = DynamicTree::GetChildPair( node );
            }
        }
    }

    // node 두 개의 상태에 따라 후보 보고, leaf-subtree 탐색, subtree 분할 중 하나를 수행함.
    template <typename Callback>
    static void VisitPair(
        const DynamicTree& treeA,
        const DynamicTree& treeB,
        const TreeNode& nodeA,
        const TreeNode& nodeB,
        std::array<NodeIndexPair, DynamicTree::TREE_STACK_SIZE>& stack,
        std::size_t& stackCount,
        PairContext<Callback>& context )
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
            AddCandidatePair( nodeA, nodeB, context );
        }
        else if( leafA )
        {
            CollideProxyAndSubtree( nodeA, treeB, DynamicTree::GetChildPair( nodeB ), context );
        }
        else if( leafB )
        {
            CollideProxyAndSubtree( nodeB, treeA, DynamicTree::GetChildPair( nodeA ), context );
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
    template <typename Callback>
    static void CollideCrossPairs(
        const DynamicTree& treeA,
        const DynamicTree& treeB,
        const TreeNode& subtreeA,
        const TreeNode& subtreeB,
        PairContext<Callback>& context )
    {
        // 재귀 대신 고정 크기 stack으로 subtree 조합을 순회함.
        std::array<NodeIndexPair, DynamicTree::TREE_STACK_SIZE> stack{};
        std::size_t stackCount = 0;

        // 처음 두 subtree를 검사하고 internal끼리면 child pair가 stack에 추가됨.
        VisitPair( treeA, treeB, subtreeA, subtreeB, stack, stackCount, context );

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
                        context
                    );
                }
            }
        }
    }

    // body type마다 독립된 DynamicTree를 사용함.
    std::array<DynamicTree, BODY_TYPE_COUNT> trees_{};

    // 이미 Contact를 가진 shape pair를 저장해 새 후보 생성에서 제외할 수 있게 함.
    HashSet pairSet_{ 32 };
    std::unordered_set<std::uint64_t> pairSet_std_;
};

} // namespace zonai
