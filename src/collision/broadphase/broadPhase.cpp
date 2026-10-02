#include "collision/broadphase/broadPhase.h"

#include <cassert>

namespace zonai
{

proxyKey broadPhase::CreateProxy(
    bodyType type, const aabb2& aabb, std::int32_t shapeIndex, 
    bool forcePairCreation )
{
    // static은 강제 요청이 있을 때만 moved 처리하고 나머지 type은 새 pair 생성을 위해 moved 처리함.
    const bool markMoved = type != bodyType::Static || forcePairCreation;

    dynamicTree& tree = GetTree( type );
    const std::int32_t proxyId = tree.CreateProxy( aabb, shapeIndex, markMoved );

    return MakeProxyKey( proxyId, type );
}

void broadPhase::DestroyProxy( proxyKey proxyKey )
{
    // key 하나로 proxy가 속한 tree와 stable proxy id를 복원함.
    const bodyType type = GetProxyType( proxyKey );
    const std::int32_t proxyId = GetProxyId( proxyKey );

    GetTree( type ).DestroyProxy( proxyId );
}

void broadPhase::MoveProxy( proxyKey proxyKey, const aabb2& aabb )
{
    // key에서 tree와 stable proxy id를 복원한 뒤 해당 proxy만 갱신함.
    const bodyType type = GetProxyType( proxyKey );
    const std::int32_t proxyId = GetProxyId( proxyKey );

    // broadPhase에서 이동된 proxy는 새 pair 생성을 위해 항상 moved 처리함.
    GetTree( type ).MoveProxy( proxyId, aabb, true );
}

void broadPhase::EnlargeProxy(
    proxyKey proxyKey,
    const aabb2& aabb )
{
    const bodyType type =
        GetProxyType( proxyKey );

    const std::int32_t proxyId =
        GetProxyId( proxyKey );

    GetTree( type ).EnlargeProxy(
        proxyId,
        aabb
    );
}

bool broadPhase::AddPair( shapePairKey pairKey )
{
    return pairSet_.Add( pairKey );
}

bool broadPhase::RemovePair( shapePairKey pairKey )
{
    return pairSet_.Remove( pairKey );
}

bool broadPhase::HasPair( shapePairKey pairKey ) const
{
    return pairSet_.Contains( pairKey );
}

bool broadPhase::TestPair( const treeNode& nodeA, const treeNode& nodeB )
{
    // 빈 tree의 root는 leaf bit를 포함한 sentinel이므로 실제 proxy처럼 취급하면 안 됨.
    if( dynamicTree::IsEmptyNode( nodeA ) ||
        dynamicTree::IsEmptyNode( nodeB ) )
    {
        return false;
    }

    // 둘 중 하나라도 moved이고 AABB가 겹칠 때만 새 pair 후보가 될 수 있음.
    if( ( ( nodeA.flagIndex | nodeB.flagIndex ) & dynamicTree::TREE_MOVED_NODE ) == 0 )
    {
        return false;
    }

    return Overlaps( nodeA.aabb, nodeB.aabb );
}

std::span<std::int32_t> broadPhase::PrepareMovedSiblingScratch() const
{
    const dynamicTree& dynamicTree = GetTree( bodyType::Dynamic );
    const std::size_t required = dynamicTree.nodes_.size() / 2;

    movedSiblings_.resize( required );
    return movedSiblings_;
}

std::size_t broadPhase::GatherMovedSiblings( const dynamicTree& tree, std::span<std::int32_t> pairIndices )
{
    // sibling이 항상 2개씩 붙어 있으므로 전체 node 수의 절반이면 출력 공간이 충분함.
    const std::size_t pairCapacity = tree.nodes_.size() / 2;

    if( pairIndices.size() < pairCapacity )
    {
        // PrepareMovedSiblingScratch가 필요한 크기를 보장하지만
        // 이 함수 자체도 release에서 범위를 넘겨 쓰지 않도록 방어함.
        assert( pairIndices.size() >= pairCapacity );
        return 0;
    }

    std::size_t count = 0;

    // root를 제외하고 sibling pair를 순회하며 moved node가 포함된 pair만 모음.
    for( std::int32_t pair = 2; static_cast<std::size_t>( pair ) < tree.nodes_.size(); pair += 2 )
    {
        const treeNode& child1 = tree.nodes_[pair];
        const treeNode& child2 = tree.nodes_[pair + 1];

        if( ( ( child1.flagIndex | child2.flagIndex ) & dynamicTree::TREE_MOVED_NODE ) != 0 )
        {
            pairIndices[count++] = pair;
        }
    }

    return count;
}

std::size_t broadPhase::GatherCrossSeeds(
    const dynamicTree& treeA,
    const dynamicTree& treeB,
    std::span<treeNodePair> seeds )
{
    assert( seeds.size() >= CROSS_SEED_COUNT );

    // root 조합부터 BFS로 내려가며 병렬로 처리하기 좋은 subtree 단위까지 분할함.
    std::array<treeNodePair, 2 * CROSS_SEED_COUNT> queue{};
    constexpr std::size_t QUEUE_MASK = 2 * CROSS_SEED_COUNT - 1;

    std::size_t head = 0;
    std::size_t tail = 0;

    const treeNode& rootA = treeA.nodes_[dynamicTree::ROOT_NODE];
    const treeNode& rootB = treeB.nodes_[dynamicTree::ROOT_NODE];

    if( TestPair( rootA, rootB ) )
    {
        queue[tail++ & QUEUE_MASK] = { rootA, rootB };
    }

    std::size_t seedCount = 0;

    // internal node 둘을 펼치면 최대 4개 조합이 생기므로 여유가 있을 때만 더 분할함.
    while( head < tail && seedCount + ( tail - head ) + 3 < CROSS_SEED_COUNT )
    {
        const treeNodePair pair = queue[head++ & QUEUE_MASK];

        // 한쪽이라도 leaf면 더 균등하게 분할하기 어려우므로 현재 조합을 seed로 확정함.
        if( dynamicTree::IsLeaf( pair.a ) || dynamicTree::IsLeaf( pair.b ) )
        {
            seeds[seedCount++] = pair;
            continue;
        }

        const std::int32_t childPairA = dynamicTree::GetChildPair( pair.a );
        const std::int32_t childPairB = dynamicTree::GetChildPair( pair.b );

        // 두 internal node의 자식 2개씩을 조합해 최대 4개의 겹치는 subtree를 queue에 추가함.
        for( std::int32_t i = 0; i < 2; ++i )
        {
            for( std::int32_t j = 0; j < 2; ++j )
            {
                const treeNode& childA = treeA.nodes_[childPairA + i];
                const treeNode& childB = treeB.nodes_[childPairB + j];

                if( TestPair( childA, childB ) )
                {
                    queue[tail++ & QUEUE_MASK] = { childA, childB };
                }
            }
        }
    }

    // 더 분할하지 못한 queue 항목도 그대로 seed로 넘김.
    while( head < tail )
    {
        seeds[seedCount++] = queue[head++ & QUEUE_MASK];
    }

    assert( seedCount <= CROSS_SEED_COUNT );
    return seedCount;
}

dynamicTree& broadPhase::GetTree( bodyType type )
{
    const std::size_t index = static_cast<std::size_t>( type );

    assert( index < trees_.size() );

    return trees_[index];
}

const dynamicTree& broadPhase::GetTree( bodyType type ) const
{
    const std::size_t index = static_cast<std::size_t>( type );

    assert( index < trees_.size() );

    return trees_[index];
}

} // namespace zonai
