#include "collision/broadphase/dynamicTree.h"

#include <algorithm>
#include <cassert>

namespace zonai
{

DynamicTree::DynamicTree()
{
    // index 0은 root, index 1은 항상 비워 둔다.
    // 이후 sibling pair는 2,3 / 4,5 / ... 처럼 배치한다.
    nodes_.resize( 2 );
    parents_.assign( 2, NULL_INDEX );

    nodes_[0] = MakeEmptyNode();
    nodes_[1] = MakeEmptyNode();

    proxies_.resize( INITIAL_PROXY_CAPACITY );

    for( std::size_t i = 0; i + 1 < proxies_.size(); ++i )
    {
        proxies_[i].next = static_cast<std::int32_t>( i + 1 );
    }

    proxies_.back().next = NULL_INDEX;
    proxyFreeList_ = 0;
}

int DynamicTree::CreateProxy( const aabb2& aabb, int shapeIndex )
{
    if( proxyCount_ >= 2 )
    {
        // 세 번째 proxy부터는 best sibling 탐색이 필요하다.
        // 이 경로는 다음 단계에서 구현한다.
        return NULL_INDEX;
    }

    const int proxyId = AllocateProxy();
    const TreeNode newLeaf =
        MakeLeafNode( aabb, proxyId, shapeIndex );

    if( proxyCount_ == 1 )
    {
        // 첫 proxy는 root 자체가 leaf다.
        nodes_[ROOT_NODE] = newLeaf;
        parents_[ROOT_NODE] = NULL_INDEX;
        proxies_[proxyId].node = ROOT_NODE;

        return proxyId;
    }

    // 두 번째 proxy가 들어오면 기존 root leaf와 새 leaf를
    // 첫 sibling pair인 index 2, 3으로 내리고 root를 internal로 바꾼다.
    constexpr std::int32_t childPair = 2;

    nodes_.resize( 4 );
    parents_.resize( 4, NULL_INDEX );

    const TreeNode oldRootLeaf = nodes_[ROOT_NODE];
    const std::int32_t oldProxyId = GetProxyId( oldRootLeaf );

    nodes_[childPair] = oldRootLeaf;
    nodes_[childPair + 1] = newLeaf;

    parents_[childPair] = ROOT_NODE;
    parents_[childPair + 1] = ROOT_NODE;

    proxies_[oldProxyId].node = childPair;
    proxies_[proxyId].node = childPair + 1;

    nodes_[ROOT_NODE] = MakeInternalNode( childPair );
    parents_[ROOT_NODE] = NULL_INDEX;

    return proxyId;
}

std::size_t DynamicTree::GetProxyCount() const
{
    return proxyCount_;
}

int DynamicTree::GetHeight() const
{
    if( proxyCount_ == 0 || IsEmptyNode( nodes_[ROOT_NODE] ) )
    {
        return 0;
    }

    if( IsLeaf( nodes_[ROOT_NODE] ) )
    {
        return 0;
    }

    return nodes_[ROOT_NODE].height;
}

const aabb2& DynamicTree::GetProxyAABB( int proxyId ) const
{
    assert( 0 <= proxyId );
    assert( static_cast<std::size_t>( proxyId ) < proxies_.size() );

    const std::int32_t nodeIndex = proxies_[proxyId].node;

    assert( nodeIndex != NULL_INDEX );

    return nodes_[nodeIndex].aabb;
}

bool DynamicTree::IsLeaf( const TreeNode& node )
{
    return ( node.flagIndex & TREE_LEAF_NODE ) != 0;
}

bool DynamicTree::IsEmptyNode( const TreeNode& node )
{
    return node.flagIndex == TREE_EMPTY_NODE;
}

std::int32_t DynamicTree::GetProxyId( const TreeNode& node )
{
    return static_cast<std::int32_t>(
        node.flagIndex & TREE_NODE_INDEX_MASK
    );
}

TreeNode DynamicTree::MakeEmptyNode()
{
    TreeNode node{};
    node.flagIndex = TREE_EMPTY_NODE;
    node.height = 0;

    return node;
}

TreeNode DynamicTree::MakeLeafNode(
    const aabb2& aabb,
    int proxyId,
    int shapeIndex )
{
    TreeNode node{};

    node.aabb = aabb;
    node.flagIndex =
        static_cast<std::uint32_t>( proxyId ) |
        TREE_LEAF_NODE |
        TREE_MOVED_NODE;
    node.shapeIndex = shapeIndex;

    return node;
}

TreeNode DynamicTree::MakeInternalNode(
    std::int32_t childPair ) const
{
    const TreeNode& child1 = nodes_[childPair];
    const TreeNode& child2 = nodes_[childPair + 1];

    TreeNode node{};

    node.aabb = Union( child1.aabb, child2.aabb );
    node.flagIndex =
        static_cast<std::uint32_t>( childPair ) |
        ( ( child1.flagIndex | child2.flagIndex ) & TREE_MOVED_NODE );
    node.height = 1;

    return node;
}

int DynamicTree::AllocateProxy()
{
    if( proxyFreeList_ == NULL_INDEX )
    {
        const std::size_t oldCapacity = proxies_.size();
        const std::size_t growth =
            std::max<std::size_t>( oldCapacity / 2, 1 );
        const std::size_t newCapacity = oldCapacity + growth;

        proxies_.resize( newCapacity );

        for( std::size_t i = oldCapacity;
             i + 1 < newCapacity;
             ++i )
        {
            proxies_[i].node = NULL_INDEX;
            proxies_[i].next =
                static_cast<std::int32_t>( i + 1 );
        }

        proxies_[newCapacity - 1].node = NULL_INDEX;
        proxies_[newCapacity - 1].next = NULL_INDEX;

        proxyFreeList_ =
            static_cast<std::int32_t>( oldCapacity );
    }

    const int proxyId = proxyFreeList_;
    TreeProxy& proxy = proxies_[proxyId];

    proxyFreeList_ = proxy.next;

    proxy.node = NULL_INDEX;
    proxy.next = NULL_INDEX;

    ++proxyCount_;

    return proxyId;
}

} // namespace zonai
