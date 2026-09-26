#include "collision/broadphase/broadPhase.h"

#include <cassert>

namespace zonai
{

ProxyKey BroadPhase::CreateProxy(
    BodyType type, const aabb2& aabb, std::int32_t shapeIndex, 
    bool forcePairCreation )
{
    // static은 강제 요청이 있을 때만 moved 처리하고 나머지 type은 새 pair 생성을 위해 moved 처리함.
    const bool markMoved = type != BodyType::Static || forcePairCreation;

    DynamicTree& tree = GetTree( type );
    const std::int32_t proxyId = tree.CreateProxy( aabb, shapeIndex, markMoved );

    return MakeProxyKey( proxyId, type );
}

void BroadPhase::DestroyProxy( ProxyKey proxyKey )
{
    // key 하나로 proxy가 속한 tree와 stable proxy id를 복원함.
    const BodyType type = GetProxyType( proxyKey );
    const std::int32_t proxyId = GetProxyId( proxyKey );

    GetTree( type ).DestroyProxy( proxyId );
}

void BroadPhase::MoveProxy( ProxyKey proxyKey, const aabb2& aabb )
{
    // key에서 tree와 stable proxy id를 복원한 뒤 해당 proxy만 갱신함.
    const BodyType type = GetProxyType( proxyKey );
    const std::int32_t proxyId = GetProxyId( proxyKey );

    // BroadPhase에서 이동된 proxy는 새 pair 생성을 위해 항상 moved 처리함.
    GetTree( type ).MoveProxy( proxyId, aabb, true );
}

bool BroadPhase::AddPair( ShapePairKey pairKey )
{
    return pairSet_.Add( pairKey );
}

bool BroadPhase::RemovePair( ShapePairKey pairKey )
{
    return pairSet_.Remove( pairKey );
}

bool BroadPhase::HasPair( ShapePairKey pairKey ) const
{
    return pairSet_.Contains( pairKey );
}

bool BroadPhase::TestPair( const TreeNode& nodeA, const TreeNode& nodeB )
{
    // 빈 tree의 root는 leaf bit를 포함한 sentinel이므로 실제 proxy처럼 취급하면 안 됨.
    if( DynamicTree::IsEmptyNode( nodeA ) ||
        DynamicTree::IsEmptyNode( nodeB ) )
    {
        return false;
    }

    // 둘 중 하나라도 moved이고 AABB가 겹칠 때만 새 pair 후보가 될 수 있음.
    if( ( ( nodeA.flagIndex | nodeB.flagIndex ) & DynamicTree::TREE_MOVED_NODE ) == 0 )
    {
        return false;
    }

    return Overlaps( nodeA.aabb, nodeB.aabb );
}

std::size_t BroadPhase::GatherMovedSiblings( const DynamicTree& tree, std::span<std::int32_t> pairIndices )
{
    // sibling이 항상 2개씩 붙어 있으므로 전체 node 수의 절반이면 출력 공간이 충분함.
    const std::size_t pairCapacity = tree.nodes_.size() / 2;

    if( pairIndices.size() < pairCapacity )
    {
        // Box2D는 필요한 크기를 update 시점에 정확히 할당함.
        // Zonai는 caller scratch span을 받으므로 release에서도 overflow를 막아야 함.
        assert( pairIndices.size() >= pairCapacity );
        return 0;
    }

    std::size_t count = 0;

    // root를 제외하고 sibling pair를 순회하며 moved node가 포함된 pair만 모음.
    for( std::int32_t pair = 2; static_cast<std::size_t>( pair ) < tree.nodes_.size(); pair += 2 )
    {
        const TreeNode& child1 = tree.nodes_[pair];
        const TreeNode& child2 = tree.nodes_[pair + 1];

        if( ( ( child1.flagIndex | child2.flagIndex ) & DynamicTree::TREE_MOVED_NODE ) != 0 )
        {
            pairIndices[count++] = pair;
        }
    }

    return count;
}

std::size_t BroadPhase::GatherCrossSeeds(
    const DynamicTree& treeA,
    const DynamicTree& treeB,
    std::span<TreeNodePair> seeds )
{
    assert( seeds.size() >= CROSS_SEED_COUNT );

    // root 조합부터 BFS로 내려가며 병렬로 처리하기 좋은 subtree 단위까지 분할함.
    std::array<TreeNodePair, 2 * CROSS_SEED_COUNT> queue{};
    constexpr std::size_t QUEUE_MASK = 2 * CROSS_SEED_COUNT - 1;

    std::size_t head = 0;
    std::size_t tail = 0;

    const TreeNode& rootA = treeA.nodes_[DynamicTree::ROOT_NODE];
    const TreeNode& rootB = treeB.nodes_[DynamicTree::ROOT_NODE];

    if( TestPair( rootA, rootB ) )
    {
        queue[tail++ & QUEUE_MASK] = { rootA, rootB };
    }

    std::size_t seedCount = 0;

    // internal node 둘을 펼치면 최대 4개 조합이 생기므로 여유가 있을 때만 더 분할함.
    while( head < tail && seedCount + ( tail - head ) + 3 < CROSS_SEED_COUNT )
    {
        const TreeNodePair pair = queue[head++ & QUEUE_MASK];

        // 한쪽이라도 leaf면 더 균등하게 분할하기 어려우므로 현재 조합을 seed로 확정함.
        if( DynamicTree::IsLeaf( pair.a ) || DynamicTree::IsLeaf( pair.b ) )
        {
            seeds[seedCount++] = pair;
            continue;
        }

        const std::int32_t childPairA = DynamicTree::GetChildPair( pair.a );
        const std::int32_t childPairB = DynamicTree::GetChildPair( pair.b );

        // 두 internal node의 자식 2개씩을 조합해 최대 4개의 겹치는 subtree를 queue에 추가함.
        for( std::int32_t i = 0; i < 2; ++i )
        {
            for( std::int32_t j = 0; j < 2; ++j )
            {
                const TreeNode& childA = treeA.nodes_[childPairA + i];
                const TreeNode& childB = treeB.nodes_[childPairB + j];

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

    return seedCount;
}

DynamicTree& BroadPhase::GetTree( BodyType type )
{
    const std::size_t index = static_cast<std::size_t>( type );

    assert( index < trees_.size() );

    return trees_[index];
}

const DynamicTree& BroadPhase::GetTree( BodyType type ) const
{
    const std::size_t index = static_cast<std::size_t>( type );

    assert( index < trees_.size() );

    return trees_[index];
}

} // namespace zonai
