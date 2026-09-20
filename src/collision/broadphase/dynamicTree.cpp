#include "collision/broadphase/dynamicTree.h"

#include <algorithm>
#include <cassert>
#include <limits>

namespace zonai
{

/*
* DynamicTree 동작 서술
* 
* 노드 삽입
* AABB를 가진 leaf node를 삽입하면, 기존 tree에서 sibling을 SAH로 찾아서 internal node를 만들고 연결함.
* 
* 노드 조정
* internal node의 child pair가 바뀌면, AABB를 다시 계산하고, 필요하면 local rotation을 시도함.
* 
* local rotation
* 중간 노드의 child pair를 바꾸는 rotation을 시도함.
* rotation은 항상 child pair를 바꾸는 것이므로, sibling pair의 index는 연속된 index로 유지됨.
* 
* 노드 삭제
* 리프 노드를 삭제하면 트리에서 제거하고, parent internal node가 자식을 모두 잃으면 그 부모 노드도 제거함.
* 
* 노드 이동
* 리프 노드를 이동하면 트리에서 제거하고, 새 AABB로 다시 삽입함.
*/

// sibling이란?
// internal node의 두 child를 sibling pair라고 부름. sibling pair는 항상 연속된 index로 배치함.

DynamicTree::DynamicTree()
{
    // root는 0번을 고정으로 쓰고 1번은 비워둠.
    // sibling pair는 2,3 / 4,5 / ...처럼 짝수 index부터 연속 배치함.
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
    // 사용할 proxy id를 free-list에서 확보함.
    const std::int32_t proxyId = AllocateProxy();
    const TreeNode newLeaf = MakeLeafNode( aabb, proxyId, shapeIndex );
    
    // 첫 proxy면 internal node 없이 root에 바로 넣음.
    if( proxyCount_ == 1 )
    {
        nodes_[ROOT_NODE] = newLeaf;
        parents_[ROOT_NODE] = NULL_INDEX;
        proxies_[proxyId].node = ROOT_NODE;

        return proxyId;
    }

    // 신규 삽입은 SAH로 sibling을 찾고 올라오면서 local rotation도 시도함.
    InsertLeaf( newLeaf, true );

    return proxyId;
}

void DynamicTree::DestroyProxy( std::int32_t proxyId )
{
    assert( 0 <= proxyId );
    assert( static_cast<std::size_t>( proxyId ) < proxies_.size() );
    assert( proxies_[proxyId].node != NULL_INDEX );

    const std::int32_t leafIndex = proxies_[proxyId].node;

    // 먼저 leaf를 tree에서 제거함.
    RemoveLeaf( leafIndex );

    // leaf 수명이 끝났으므로 proxy id도 free-list에 반환함.
    FreeProxy( proxyId );
}

void DynamicTree::MoveProxy( std::int32_t proxyId, const aabb2& aabb )
{
    assert( 0 <= proxyId );
    assert( static_cast<std::size_t>( proxyId ) < proxies_.size() );
    assert( proxies_[proxyId].node != NULL_INDEX );

    const std::int32_t leafIndex = proxies_[proxyId].node;

    // 재삽입해도 stable proxy id와 shape index는 그대로 유지함.
    const std::int32_t shapeIndex = nodes_[leafIndex].shapeIndex;

    // 기존 leaf만 tree에서 제거해서 proxy id의 수명은 유지함.
    RemoveLeaf( leafIndex );

    // 같은 proxy id로 새 AABB의 leaf를 만들어 다시 연결함.
    const TreeNode newLeaf = MakeLeafNode( aabb, proxyId, shapeIndex );

    if( proxyCount_ == 1 )
    {
        nodes_[ROOT_NODE] = newLeaf;
        parents_[ROOT_NODE] = NULL_INDEX;
        proxies_[proxyId].node = ROOT_NODE;
        return;
    }

    // MoveProxy 재삽입에서는 Box2D처럼 local rotation을 수행하지 않음.
    InsertLeaf( newLeaf, false );
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

    const float rootPerimeter = Perimeter( nodes_[ROOT_NODE].aabb );

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

bool DynamicTree::Validate() const
{
    // node 배열과 parent 배열의 기본 불변조건부터 확인함.
    if( nodes_.size() < 2 ||
        nodes_.size() != parents_.size() ||
        ( nodes_.size() & 1u ) != 0 )
    {
        return false;
    }

    if( parents_[ROOT_NODE] != NULL_INDEX ||
        !IsEmptyNode( nodes_[ROOT_NODE + 1] ) )
    {
        // root는 parent가 없어야 하고 1번 node는 항상 비워둠.
        return false;
    }

    if( proxyCount_ == 0 )
    {
        // proxy가 없으면 root도 비어있어야 함.
        return IsEmptyNode( nodes_[ROOT_NODE] );
    }

    if( IsEmptyNode( nodes_[ROOT_NODE] ) )
    {
        // proxy가 있는데 root가 비어있으면 잘못된 상태임.
        return false;
    }

    std::int32_t height = 0;
    std::size_t leafCount = 0;

    // root부터 내려가며 parent, AABB, height, proxy 연결을 검증함.
    if( !ValidateSubtree( ROOT_NODE, height, leafCount ) )
    {
        return false;
    }

    if( leafCount != proxyCount_ || height != GetHeight() )
    {
        return false;
    }

    std::size_t liveProxyCount = 0;

    // 반대로 각 proxy도 자기 leaf를 정확히 가리키는지 확인함.
    for( std::size_t proxyId = 0; proxyId < proxies_.size(); ++proxyId )
    {
        const std::int32_t nodeIndex = proxies_[proxyId].node;

        if( nodeIndex == NULL_INDEX )
        {
            continue;
        }

        if( nodeIndex < 0 || static_cast<std::size_t>( nodeIndex ) >= nodes_.size() )
        {
            return false;
        }

        const TreeNode& node = nodes_[nodeIndex];

        if( IsEmptyNode( node ) || !IsLeaf( node ) || GetProxyId( node ) != static_cast<std::int32_t>( proxyId ) )
        {
            return false;
        }

        ++liveProxyCount;
    }

    return liveProxyCount == proxyCount_;
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

TreeNode DynamicTree::MakeLeafNode( const aabb2& aabb, std::int32_t proxyId, std::int32_t shapeIndex )
{
    TreeNode node{};

    node.aabb = aabb;
    node.flagIndex = static_cast<std::uint32_t>( proxyId ) | TREE_LEAF_NODE | TREE_MOVED_NODE;
    node.shapeIndex = shapeIndex;

    return node;
}

TreeNode DynamicTree::MakeInternalNode( std::int32_t childPair ) const
{
    const TreeNode& child1 = nodes_[childPair];
    const TreeNode& child2 = nodes_[childPair + 1];

    TreeNode node{};

    node.aabb = Union( child1.aabb, child2.aabb );
    node.flagIndex =
        static_cast<std::uint32_t>( childPair ) |
        ( ( child1.flagIndex | child2.flagIndex ) & TREE_MOVED_NODE );
    node.height = 1 + std::max( GetNodeHeight( child1 ), GetNodeHeight( child2 ) );

    return node;
}

std::int32_t DynamicTree::AllocateProxy()
{
    // free-list가 비었으면 proxy pool을 50% 정도 늘림.
    if( proxyFreeList_ == NULL_INDEX )
    {
        const std::size_t oldCapacity = proxies_.size();
        const std::size_t growth = std::max<std::size_t>( oldCapacity / 2, 1 );
        const std::size_t newCapacity = oldCapacity + growth;

        proxies_.resize( newCapacity );

        for( std::size_t i = oldCapacity; i + 1 < newCapacity; ++i )
        {
            proxies_[i].node = NULL_INDEX;
            proxies_[i].next = static_cast<std::int32_t>( i + 1 );
        }

        proxies_[newCapacity - 1].node = NULL_INDEX;
        proxies_[newCapacity - 1].next = NULL_INDEX;

        proxyFreeList_ = static_cast<std::int32_t>( oldCapacity );
    }

    // free-list 맨 앞에서 사용할 proxy id를 꺼냄.
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

    // 반환된 proxy id를 free-list 맨 앞에 다시 붙임.
    proxy.node = NULL_INDEX;
    proxy.next = proxyFreeList_;

    proxyFreeList_ = proxyId;

    assert( proxyCount_ > 0 );
    --proxyCount_;
}

std::int32_t DynamicTree::AllocateSiblingPair()
{
    // 반납된 sibling pair가 있으면 먼저 재활용함.
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

    // 재활용할 pair가 없으면 node 배열 뒤에 두 칸을 추가함.
    const std::int32_t pair = static_cast<std::int32_t>( nodes_.size() );

    // 0번은 root, 1번은 비워두기 때문에 pair 시작 index는 항상 짝수임.
    assert( ( pair & 1 ) == 0 );

    nodes_.resize( nodes_.size() + 2 );
    parents_.resize( parents_.size() + 2, NULL_INDEX );

    nodes_[pair] = MakeEmptyNode();
    nodes_[pair + 1] = MakeEmptyNode();

    return pair;
}

void DynamicTree::FreeSiblingPair( std::int32_t pair )
{
    // sibling은 pair 단위로 관리하므로 짝수 index에서 시작해야 함.
    assert( pair >= 2 );
    assert( ( pair & 1 ) == 0 );

    nodes_[pair] = MakeEmptyNode();
    nodes_[pair + 1] = MakeEmptyNode();

    // 빈 pair의 첫 parent 값을 free-list의 next로 재활용함.
    parents_[pair] = pairFreeList_;
    parents_[pair + 1] = NULL_INDEX;

    pairFreeList_ = pair;
}

std::int32_t DynamicTree::FindBestSibling( const aabb2& boxD ) const
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

    // root부터 한쪽으로 내려가며 새 leaf와 묶을 비용이 가장 작은 sibling을 찾음.
    for( ;; )
    {
        const std::int32_t child1 = GetChildPair( nodes_[nodeIndex] );
        const std::int32_t child2 = child1 + 1;

        // 현재 node를 sibling으로 선택했을 때의 비용을 계산함.
        const float currentCost = directCost + inheritedCost;

        if( currentCost < bestCost )
        {
            bestSibling = nodeIndex;
            bestCost = currentCost;
        }

        // 현재 node에서 늘어난 AABB 비용은 어느 child로 내려가도 그대로 따라감.
        inheritedCost += directCost - areaBase;

        const bool leaf1 = IsLeaf( nodes_[child1] );
        const bool leaf2 = IsLeaf( nodes_[child2] );

        const aabb2 box1 = nodes_[child1].aabb;
        const aabb2 box2 = nodes_[child2].aabb;

        // 각 child로 내려갔을 때 늘어나는 AABB 비용을 계산함.
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
            const float cost1 = directCost1 + inheritedCost;

            if( cost1 < bestCost )
            {
                bestSibling = child1;
                bestCost = cost1;
            }
        }
        else
        {
            area1 = Perimeter( box1 );

            lowerCost1 = inheritedCost + directCost1 + std::min( areaD - area1, 0.0f );
        }

        if( leaf2 )
        {
            const float cost2 = directCost2 + inheritedCost;

            if( cost2 < bestCost )
            {
                bestSibling = child2;
                bestCost = cost2;
            }
        }
        else
        {
            area2 = Perimeter( box2 );

            lowerCost2 = inheritedCost + directCost2 + std::min( areaD - area2, 0.0f );
        }

        // 둘 다 leaf면 더 내려갈 곳이 없어서 끝냄.
        if( leaf1 && leaf2 )
        {
            break;
        }

        // 어느 child로 내려가도 현재 best보다 싸질 수 없으면 가지치기함.
        if( bestCost <= lowerCost1 && bestCost <= lowerCost2 )
        {
            break;
        }

        // 더 싸질 가능성이 있는 child 쪽으로만 내려감.
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

    // leaf면 stable proxy가 현재 node 위치를 가리키게 함.
    if( IsLeaf( node ) )
    {
        const std::int32_t proxyId = GetProxyId( node );
        proxies_[proxyId].node = nodeIndex;
        return;
    }

    // internal node면 두 child의 parent를 현재 node로 맞춤.
    const std::int32_t childPair = GetChildPair( node );

    parents_[childPair] = nodeIndex;
    parents_[childPair + 1] = nodeIndex;
}

void DynamicTree::SwapNodes( std::int32_t downIndex, std::int32_t upIndex )
{
    // rotation 후보인 child와 반대 subtree의 grandchild를 맞바꿈.
    std::swap( nodes_[downIndex], nodes_[upIndex] );

    // 위치가 바뀐 node의 proxy / parent 연결을 다시 맞춤.
    LinkChildren( downIndex );
    LinkChildren( upIndex );

    // downIndex의 sibling 구성이 바뀌었으므로 AABB와 height를 다시 계산함.
    const std::int32_t siblingIndex = downIndex ^ 1;

    assert( !IsLeaf( nodes_[siblingIndex] ) );

    nodes_[siblingIndex] = MakeInternalNode( GetChildPair( nodes_[siblingIndex] ) );
}

void DynamicTree::RotateNode( std::int32_t nodeIndex )
{
    const TreeNode& nodeA = nodes_[nodeIndex];

    assert( !IsLeaf( nodeA ) );

    const std::int32_t indexB = GetChildPair( nodeA );
    const std::int32_t indexC = indexB + 1;

    const TreeNode& nodeB = nodes_[indexB];
    const TreeNode& nodeC = nodes_[indexC];

    const bool leafB = IsLeaf( nodeB );
    const bool leafC = IsLeaf( nodeC );

    if( leafB && leafC )
    {
        return;
    }

    std::int32_t bestDown = NULL_INDEX;
    std::int32_t bestUp = NULL_INDEX;
    float bestDelta = 0.0f;

    // 가능한 swap 조합 중 perimeter를 가장 많이 줄이는 경우를 찾음.
    if( !leafC )
    {
        const std::int32_t indexF = GetChildPair( nodeC );
        const std::int32_t indexG = indexF + 1;

        const float areaC = Perimeter( nodeC.aabb );

        const float deltaBF =
            Perimeter( Union( nodeB.aabb, nodes_[indexG].aabb ) ) - areaC;

        if( deltaBF < bestDelta )
        {
            bestDown = indexB;
            bestUp = indexF;
            bestDelta = deltaBF;
        }

        const float deltaBG =
            Perimeter( Union( nodeB.aabb, nodes_[indexF].aabb ) ) - areaC;

        if( deltaBG < bestDelta )
        {
            bestDown = indexB;
            bestUp = indexG;
            bestDelta = deltaBG;
        }
    }

    if( !leafB )
    {
        const std::int32_t indexD =
            GetChildPair( nodeB );
        const std::int32_t indexE =
            indexD + 1;

        const float areaB =
            Perimeter( nodeB.aabb );

        const float deltaCD =
            Perimeter(
                Union(
                    nodeC.aabb,
                    nodes_[indexE].aabb
                )
            ) - areaB;

        if( deltaCD < bestDelta )
        {
            bestDown = indexC;
            bestUp = indexD;
            bestDelta = deltaCD;
        }

        const float deltaCE =
            Perimeter(
                Union(
                    nodeC.aabb,
                    nodes_[indexD].aabb
                )
            ) - areaB;

        if( deltaCE < bestDelta )
        {
            bestDown = indexC;
            bestUp = indexE;
        }
    }

    // perimeter가 실제로 줄어드는 경우에만 swap함.
    if( bestDown != NULL_INDEX )
    {
        SwapNodes( bestDown, bestUp );
    }
}

void DynamicTree::InsertLeaf(
    const TreeNode& leaf,
    bool shouldRotate )
{
    // 새 leaf와 묶였을 때 비용이 가장 작은 sibling을 찾음.
    const std::int32_t siblingIndex =
        FindBestSibling( leaf.aabb );

    const std::int32_t oldParent =
        parents_[siblingIndex];

    // sibling과 새 leaf를 담을 sibling pair를 확보함.
    const std::int32_t childPair =
        AllocateSiblingPair();

    // sibling 자리는 새 parent로 재사용하고 기존 sibling은 pair로 내려보냄.
    nodes_[childPair] = nodes_[siblingIndex];
    nodes_[childPair + 1] = leaf;

    parents_[childPair] = siblingIndex;
    parents_[childPair + 1] = siblingIndex;

    LinkChildren( childPair );
    LinkChildren( childPair + 1 );

    // 기존 sibling 자리를 두 child를 감싸는 internal node로 바꿈.
    nodes_[siblingIndex] = MakeInternalNode( childPair );
    parents_[siblingIndex] = oldParent;

    // 변경 지점부터 root까지 refit하고 신규 삽입이면 local rotation도 시도함.
    RefitAncestors( siblingIndex, shouldRotate );
}

void DynamicTree::RemoveLeaf( std::int32_t leafIndex )
{
    // root leaf 하나만 남은 경우에는 root만 비우고 끝냄.
    if( leafIndex == ROOT_NODE )
    {
        nodes_[ROOT_NODE] = MakeEmptyNode();
        parents_[ROOT_NODE] = NULL_INDEX;
        return;
    }

    const std::int32_t parentIndex = parents_[leafIndex];

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

    // 살아남은 sibling을 제거되는 parent 자리로 올림.
    nodes_[parentIndex] = nodes_[siblingIndex];
    parents_[parentIndex] = grandParent;

    // 올라온 node 종류에 맞춰 proxy 또는 child parent 연결을 다시 맞춤.
    LinkChildren( parentIndex );

    // 비게 된 sibling pair를 free-list에 반환함.
    FreeSiblingPair( childPair );

    // 구조가 바뀐 지점부터 root까지 AABB와 height를 다시 계산함.
    RefitAncestors(
        parentIndex,
        false
    );
}

void DynamicTree::RefitAncestors(
    std::int32_t nodeIndex,
    bool shouldRotate )
{
    // 변경 지점에서 root 방향으로 한 단계씩 올라감.
    while( nodeIndex != NULL_INDEX )
    {
        if( !IsLeaf( nodes_[nodeIndex] ) )
        {
            // 신규 삽입에서는 refit 전에 local rotation으로 비용을 줄여봄.
            if( shouldRotate )
            {
                RotateNode( nodeIndex );
            }

            // 현재 child 기준으로 AABB, moved flag, height를 다시 계산함.
            const std::int32_t childPair =
                GetChildPair( nodes_[nodeIndex] );

            nodes_[nodeIndex] =
                MakeInternalNode( childPair );
        }

        nodeIndex = parents_[nodeIndex];
    }
}

bool DynamicTree::ValidateSubtree(
    std::int32_t nodeIndex,
    std::int32_t& height,
    std::size_t& leafCount ) const
{
    // node index가 유효한 범위인지 확인함.
    if( nodeIndex < 0 ||
        static_cast<std::size_t>( nodeIndex ) >= nodes_.size() )
    {
        return false;
    }

    const TreeNode& node = nodes_[nodeIndex];

    if( IsEmptyNode( node ) )
    {
        return false;
    }

    // leaf면 proxy -> leaf 매핑을 확인하고 height와 개수를 누적함.
    if( IsLeaf( node ) )
    {
        const std::int32_t proxyId =
            GetProxyId( node );

        if( proxyId < 0 ||
            static_cast<std::size_t>( proxyId ) >= proxies_.size() ||
            proxies_[proxyId].node != nodeIndex )
        {
            return false;
        }

        height = 0;
        ++leafCount;
        return true;
    }

    // internal node의 child pair가 2번 이후 짝수 index에서 시작하는지 확인함.
    const std::int32_t childPair =
        GetChildPair( node );

    if( childPair < 2 ||
        ( childPair & 1 ) != 0 ||
        static_cast<std::size_t>( childPair + 1 ) >= nodes_.size() )
    {
        return false;
    }

    // 두 child가 현재 node를 parent로 가리키는지 확인함.
    if( parents_[childPair] != nodeIndex ||
        parents_[childPair + 1] != nodeIndex )
    {
        return false;
    }

    const TreeNode& child1 = nodes_[childPair];
    const TreeNode& child2 = nodes_[childPair + 1];

    if( IsEmptyNode( child1 ) ||
        IsEmptyNode( child2 ) )
    {
        return false;
    }

    // 저장된 AABB가 두 child의 union과 정확히 같은지 확인함.
    const aabb2 combined =
        Union( child1.aabb, child2.aabb );

    if( !ContainsAABB( node.aabb, combined ) ||
        !ContainsAABB( combined, node.aabb ) )
    {
        return false;
    }

    // internal node의 moved flag가 두 child의 moved 상태와 일치하는지 확인함.
    const bool moved =
        ( node.flagIndex & TREE_MOVED_NODE ) != 0;
    const bool childMoved =
        ( ( child1.flagIndex | child2.flagIndex ) &
          TREE_MOVED_NODE ) != 0;

    if( moved != childMoved )
    {
        return false;
    }

    std::int32_t height1 = 0;
    std::int32_t height2 = 0;

    // 두 child를 재귀적으로 내려가 실제 height와 leaf 개수를 다시 구함.
    if( !ValidateSubtree(
            childPair,
            height1,
            leafCount ) ||
        !ValidateSubtree(
            childPair + 1,
            height2,
            leafCount ) )
    {
        return false;
    }

    height =
        1 + std::max( height1, height2 );

    return node.height == height;
}

} // namespace zonai
