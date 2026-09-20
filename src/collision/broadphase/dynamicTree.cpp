#include "collision/broadphase/dynamicTree.h"

#include <algorithm>
#include <cassert>
#include <limits>

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

std::int32_t DynamicTree::CreateProxy( const aabb2& aabb, std::int32_t shapeIndex )
{
    const std::int32_t proxyId = AllocateProxy();
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

    InsertLeaf( newLeaf );

    return proxyId;
}

void DynamicTree::DestroyProxy( std::int32_t proxyId )
{
    assert( 0 <= proxyId );
    assert( static_cast<std::size_t>( proxyId ) < proxies_.size() );
    assert( proxies_[proxyId].node != NULL_INDEX );

    const std::int32_t leafIndex =
        proxies_[proxyId].node;

    RemoveLeaf( leafIndex );
    FreeProxy( proxyId );
}

void DynamicTree::MoveProxy(
    std::int32_t proxyId,
    const aabb2& aabb )
{
    assert( 0 <= proxyId );
    assert( static_cast<std::size_t>( proxyId ) < proxies_.size() );
    assert( proxies_[proxyId].node != NULL_INDEX );

    const std::int32_t leafIndex =
        proxies_[proxyId].node;
    const std::int32_t shapeIndex =
        nodes_[leafIndex].shapeIndex;

    RemoveLeaf( leafIndex );

    const TreeNode newLeaf =
        MakeLeafNode( aabb, proxyId, shapeIndex );

    if( proxyCount_ == 1 )
    {
        nodes_[ROOT_NODE] = newLeaf;
        parents_[ROOT_NODE] = NULL_INDEX;
        proxies_[proxyId].node = ROOT_NODE;
        return;
    }

    InsertLeaf( newLeaf );
}

std::size_t DynamicTree::GetProxyCount() const
{
    return proxyCount_;
}

std::int32_t DynamicTree::GetHeight() const
{
    if( proxyCount_ == 0 || IsEmptyNode( nodes_[ROOT_NODE] ) )
    {
        return 0;
    }

    return GetNodeHeight( nodes_[ROOT_NODE] );
}

float DynamicTree::GetAreaRatio() const
{
    if( proxyCount_ == 0 || IsEmptyNode( nodes_[ROOT_NODE] ) )
    {
        return 0.0f;
    }

    const float rootPerimeter =
        Perimeter( nodes_[ROOT_NODE].aabb );

    if( rootPerimeter <= 0.0f )
    {
        return 0.0f;
    }

    float totalPerimeter = 0.0f;

    for( const TreeNode& node : nodes_ )
    {
        if( IsEmptyNode( node ) )
        {
            continue;
        }

        totalPerimeter += Perimeter( node.aabb );
    }

    return totalPerimeter / rootPerimeter;
}

const aabb2& DynamicTree::GetProxyAABB( std::int32_t proxyId ) const
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

std::int32_t DynamicTree::GetChildPair( const TreeNode& node )
{
    return static_cast<std::int32_t>(
        node.flagIndex & TREE_NODE_INDEX_MASK
    );
}

std::int32_t DynamicTree::GetProxyId( const TreeNode& node )
{
    return static_cast<std::int32_t>(
        node.flagIndex & TREE_NODE_INDEX_MASK
    );
}

std::int32_t DynamicTree::GetNodeHeight( const TreeNode& node )
{
    return IsLeaf( node ) ? 0 : node.height;
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
    std::int32_t proxyId,
    std::int32_t shapeIndex )
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
    node.height =
        1 + std::max(
            GetNodeHeight( child1 ),
            GetNodeHeight( child2 )
        );

    return node;
}

std::int32_t DynamicTree::AllocateProxy()
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

    const std::int32_t proxyId = proxyFreeList_;
    TreeProxy& proxy = proxies_[proxyId];

    proxyFreeList_ = proxy.next;

    proxy.node = NULL_INDEX;
    proxy.next = NULL_INDEX;

    ++proxyCount_;

    return proxyId;
}

void DynamicTree::FreeProxy( std::int32_t proxyId )
{
    TreeProxy& proxy = proxies_[proxyId];

    proxy.node = NULL_INDEX;
    proxy.next = proxyFreeList_;

    proxyFreeList_ = proxyId;

    assert( proxyCount_ > 0 );
    --proxyCount_;
}

std::int32_t DynamicTree::AllocateSiblingPair()
{
    if( pairFreeList_ != NULL_INDEX )
    {
        const std::int32_t pair = pairFreeList_;

        pairFreeList_ = parents_[pair];

        nodes_[pair] = MakeEmptyNode();
        nodes_[pair + 1] = MakeEmptyNode();

        parents_[pair] = NULL_INDEX;
        parents_[pair + 1] = NULL_INDEX;

        return pair;
    }

    const std::int32_t pair =
        static_cast<std::int32_t>( nodes_.size() );

    // root 0 + spare 1 때문에 이후 pair 시작점은 항상 짝수다.
    assert( ( pair & 1 ) == 0 );

    nodes_.resize( nodes_.size() + 2 );
    parents_.resize( parents_.size() + 2, NULL_INDEX );

    nodes_[pair] = MakeEmptyNode();
    nodes_[pair + 1] = MakeEmptyNode();

    return pair;
}

void DynamicTree::FreeSiblingPair( std::int32_t pair )
{
    assert( pair >= 2 );
    assert( ( pair & 1 ) == 0 );

    nodes_[pair] = MakeEmptyNode();
    nodes_[pair + 1] = MakeEmptyNode();

    // 비어 있는 pair에서는 첫 슬롯의 parent 필드를
    // free-list의 next index로 재사용한다.
    parents_[pair] = pairFreeList_;
    parents_[pair + 1] = NULL_INDEX;

    pairFreeList_ = pair;
}

std::int32_t DynamicTree::FindBestSibling(
    const aabb2& boxD ) const
{
    std::int32_t nodeIndex = ROOT_NODE;

    if( IsLeaf( nodes_[nodeIndex] ) )
    {
        return nodeIndex;
    }

    const float areaD = Perimeter( boxD );

    aabb2 nodeBox = nodes_[nodeIndex].aabb;
    float areaBase = Perimeter( nodeBox );
    float directCost = Perimeter( Union( nodeBox, boxD ) );
    float inheritedCost = 0.0f;

    std::int32_t bestSibling = nodeIndex;
    float bestCost = directCost;

    for( ;; )
    {
        const std::int32_t child1 =
            GetChildPair( nodes_[nodeIndex] );
        const std::int32_t child2 = child1 + 1;

        const float currentCost =
            directCost + inheritedCost;

        if( currentCost < bestCost )
        {
            bestSibling = nodeIndex;
            bestCost = currentCost;
        }

        // 이 노드의 AABB가 커지는 비용은 아래 어느 자식으로
        // 내려가더라도 공통으로 상속된다.
        inheritedCost += directCost - areaBase;

        const bool leaf1 = IsLeaf( nodes_[child1] );
        const bool leaf2 = IsLeaf( nodes_[child2] );

        const aabb2 box1 = nodes_[child1].aabb;
        const aabb2 box2 = nodes_[child2].aabb;

        const float directCost1 =
            Perimeter( Union( box1, boxD ) );
        const float directCost2 =
            Perimeter( Union( box2, boxD ) );

        float area1 = 0.0f;
        float area2 = 0.0f;

        float lowerCost1 =
            std::numeric_limits<float>::max();
        float lowerCost2 =
            std::numeric_limits<float>::max();

        if( leaf1 )
        {
            const float cost1 =
                directCost1 + inheritedCost;

            if( cost1 < bestCost )
            {
                bestSibling = child1;
                bestCost = cost1;
            }
        }
        else
        {
            area1 = Perimeter( box1 );

            lowerCost1 =
                inheritedCost +
                directCost1 +
                std::min( areaD - area1, 0.0f );
        }

        if( leaf2 )
        {
            const float cost2 =
                directCost2 + inheritedCost;

            if( cost2 < bestCost )
            {
                bestSibling = child2;
                bestCost = cost2;
            }
        }
        else
        {
            area2 = Perimeter( box2 );

            lowerCost2 =
                inheritedCost +
                directCost2 +
                std::min( areaD - area2, 0.0f );
        }

        if( leaf1 && leaf2 )
        {
            break;
        }

        if( bestCost <= lowerCost1 &&
            bestCost <= lowerCost2 )
        {
            break;
        }

        if( lowerCost1 <= lowerCost2 )
        {
            nodeIndex = child1;
            areaBase = area1;
            directCost = directCost1;
        }
        else
        {
            nodeIndex = child2;
            areaBase = area2;
            directCost = directCost2;
        }
    }

    return bestSibling;
}

void DynamicTree::LinkChildren( std::int32_t nodeIndex )
{
    const TreeNode& node = nodes_[nodeIndex];

    if( IsLeaf( node ) )
    {
        const std::int32_t proxyId = GetProxyId( node );
        proxies_[proxyId].node = nodeIndex;
        return;
    }

    const std::int32_t childPair = GetChildPair( node );

    parents_[childPair] = nodeIndex;
    parents_[childPair + 1] = nodeIndex;
}

void DynamicTree::InsertLeaf( const TreeNode& leaf )
{
    const std::int32_t siblingIndex =
        FindBestSibling( leaf.aabb );

    const std::int32_t oldParent =
        parents_[siblingIndex];

    const std::int32_t childPair =
        AllocateSiblingPair();

    // siblingIndex 자리를 새 internal parent로 재사용하고,
    // 기존 sibling은 새 pair의 첫 슬롯으로 이동한다.
    nodes_[childPair] = nodes_[siblingIndex];
    nodes_[childPair + 1] = leaf;

    parents_[childPair] = siblingIndex;
    parents_[childPair + 1] = siblingIndex;

    LinkChildren( childPair );
    LinkChildren( childPair + 1 );

    nodes_[siblingIndex] =
        MakeInternalNode( childPair );
    parents_[siblingIndex] = oldParent;

    RefitAncestors( siblingIndex );
}

void DynamicTree::RemoveLeaf(
    std::int32_t leafIndex )
{
    if( leafIndex == ROOT_NODE )
    {
        nodes_[ROOT_NODE] = MakeEmptyNode();
        parents_[ROOT_NODE] = NULL_INDEX;
        return;
    }

    const std::int32_t parentIndex =
        parents_[leafIndex];

    assert( parentIndex != NULL_INDEX );
    assert( !IsLeaf( nodes_[parentIndex] ) );

    const std::int32_t childPair =
        GetChildPair( nodes_[parentIndex] );

    assert(
        leafIndex == childPair ||
        leafIndex == childPair + 1
    );

    const std::int32_t siblingIndex =
        leafIndex == childPair ?
            childPair + 1 :
            childPair;

    const std::int32_t grandParent =
        parents_[parentIndex];

    // 제거되는 parent 자리로 살아남은 sibling을 승격한다.
    nodes_[parentIndex] = nodes_[siblingIndex];
    parents_[parentIndex] = grandParent;

    // sibling이 leaf면 proxy->node를, internal이면
    // 그 자식들의 parent를 새 위치에 맞게 고친다.
    LinkChildren( parentIndex );

    FreeSiblingPair( childPair );

    RefitAncestors( parentIndex );
}

void DynamicTree::RefitAncestors(
    std::int32_t nodeIndex )
{
    while( nodeIndex != NULL_INDEX )
    {
        TreeNode& node = nodes_[nodeIndex];

        if( !IsLeaf( node ) )
        {
            const std::int32_t childPair =
                GetChildPair( node );

            node = MakeInternalNode( childPair );
        }

        nodeIndex = parents_[nodeIndex];
    }
}

} // namespace zonai
