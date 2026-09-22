#include "collision/broadphase/broadPhase.h"

#include <cassert>

namespace zonai
{

ProxyKey BroadPhase::CreateProxy(
    BodyType type,
    const aabb2& aabb,
    std::int32_t shapeIndex,
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
    const BodyType type = GetProxyType( proxyKey );
    const std::int32_t proxyId = GetProxyId( proxyKey );

    GetTree( type ).DestroyProxy( proxyId );
}

void BroadPhase::MoveProxy( ProxyKey proxyKey, const aabb2& aabb )
{
    const BodyType type = GetProxyType( proxyKey );
    const std::int32_t proxyId = GetProxyId( proxyKey );

    // BroadPhase에서 이동된 proxy는 새 pair 생성을 위해 항상 moved 처리함.
    GetTree( type ).MoveProxy( proxyId, aabb, true );
}

std::size_t BroadPhase::GatherMovedSiblings( const DynamicTree& tree, std::span<std::int32_t> pairIndices )
{
    const std::size_t pairCapacity = tree.nodes_.size() / 2;

    assert( pairIndices.size() >= pairCapacity );

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
