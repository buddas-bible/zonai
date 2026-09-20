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
    // 사용할 proxy id를 free-list에서 확보함.
    const std::int32_t proxyId = AllocateProxy();
    const TreeNode newLeaf = MakeLeafNode( aabb, proxyId, shapeIndex );

    // 첫 proxy면 별도 internal node 없이 root 자체를 leaf로 사용함.
    if( proxyCount_ == 1 )
    {
        // 첫 proxy는 root 자체가 leaf다.
        nodes_[ROOT_NODE] = newLeaf;
        parents_[ROOT_NODE] = NULL_INDEX;
        proxies_[proxyId].node = ROOT_NODE;

        return proxyId;
    }

    // 기존 tree가 있으면 SAH로 삽입 위치를 찾고 필요하면 회전함.
    InsertLeaf( newLeaf, true );

    return proxyId;
}

void DynamicTree::DestroyProxy( std::int32_t proxyId )
{
    assert( 0 <= proxyId );
    assert( static_cast<std::size_t>( proxyId ) < proxies_.size() );
    assert( proxies_[proxyId].node != NULL_INDEX );

    const std::int32_t leafIndex = proxies_[proxyId].node;

    // tree에서 leaf를 먼저 분리함.
    RemoveLeaf( leafIndex );

    // 더 이상 사용하지 않는 proxy id를 free-list에 반환함.
    FreeProxy( proxyId );
}

void DynamicTree::MoveProxy( std::int32_t proxyId, const aabb2& aabb )
{
    assert( 0 <= proxyId );
    assert( static_cast<std::size_t>( proxyId ) < proxies_.size() );
    assert( proxies_[proxyId].node != NULL_INDEX );

    const std::int32_t leafIndex = proxies_[proxyId].node;

    // leaf를 제거하기 전에 proxy가 가리키던 shape 정보를 보존함.
    const std::int32_t shapeIndex = nodes_[leafIndex].shapeIndex;

    // proxy id는 유지하고 tree의 기존 위치만 제거함.
    RemoveLeaf( leafIndex );

    // 같은 proxy id와 shape 정보로 새 AABB를 가진 leaf를 다시 만듦.
    const TreeNode newLeaf = MakeLeafNode( aabb, proxyId, shapeIndex );

    if( proxyCount_ == 1 )
    {
        nodes_[ROOT_NODE] = newLeaf;
        parents_[ROOT_NODE] = NULL_INDEX;
        proxies_[proxyId].node = ROOT_NODE;
        return;
    }

    // 이동 재삽입에서는 Box2D와 동일하게 rotation을 수행하지 않음.
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
    // node/parent 저장소의 기본 구조가 깨졌는지 먼저 확인함.
    if( nodes_.size() < 2 ||
        nodes_.size() != parents_.size() ||     // 
        ( nodes_.size() & 1u ) != 0 )
    {
        return false;
    }

    if( parents_[ROOT_NODE] != NULL_INDEX ||
        !IsEmptyNode( nodes_[ROOT_NODE + 1] ) )
    {
		// root의 parent는 NULL_INDEX여야 하고, root 다음 노드는 비어 있어야 한다.
        return false;
    }

    if( proxyCount_ == 0 )
    {
		// proxyCount_ == 0이면 root가 비어 있어야 한다.
        return IsEmptyNode( nodes_[ROOT_NODE] );
    }

    if( IsEmptyNode( nodes_[ROOT_NODE] ) )
    {
		// proxyCount_ > 0인데 root가 비어있으면 안 된다.
        return false;
    }

    std::int32_t height = 0;
    std::size_t leafCount = 0;

    // root부터 내려가며 parent, AABB, height, proxy 연결을 재귀적으로 검사함.
    if( !ValidateSubtree( ROOT_NODE, height, leafCount ) )
    {
        return false;
    }

    if( leafCount != proxyCount_ || height != GetHeight() )
    {
        return false;
    }

    std::size_t liveProxyCount = 0;

    // proxy 쪽에서도 반대로 올바른 leaf를 가리키는지 전부 확인함.
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
    // 남은 proxy 슬롯이 없으면 pool을 50% 정도 확장함.
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

    // free-list의 맨 앞 proxy를 꺼냄.
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

    // 반환되는 proxy를 free-list의 맨 앞에 다시 연결함.
    proxy.node = NULL_INDEX;
    proxy.next = proxyFreeList_;

    proxyFreeList_ = proxyId;

    assert( proxyCount_ > 0 );
    --proxyCount_;
}

std::int32_t DynamicTree::AllocateSiblingPair()
{
    // 이전에 반환된 sibling pair가 있으면 새로 늘리지 않고 재사용함.
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

    // 재사용할 pair가 없으면 node 저장소 끝에 두 칸을 추가함.
    const std::int32_t pair = static_cast<std::int32_t>( nodes_.size() );

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
    // pair 단위로만 반환하므로 항상 root 이후의 짝수 index여야 함.
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

    // root부터 한 경로만 greedy하게 내려가며 가장 싼 sibling 후보를 찾음.
    for( ;; )
    {
        const std::int32_t child1 = GetChildPair( nodes_[nodeIndex] );
        const std::int32_t child2 = child1 + 1;

        // 현재 node 자체를 sibling으로 선택했을 때의 비용임.
        const float currentCost = directCost + inheritedCost;

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

        // 각 child 아래로 내려갔을 때 직접 증가하는 AABB 비용을 계산함.
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

        // 둘 다 leaf면 더 내려갈 곳이 없으므로 탐색 종료함.
        if( leaf1 && leaf2 )
        {
            break;
        }

        // 어느 child로 내려가도 현재 best보다 싸질 수 없으면 가지치기함.
        if( bestCost <= lowerCost1 && bestCost <= lowerCost2 )
        {
            break;
        }

        // 더 낮은 lower bound를 가진 child 하나만 선택해서 계속 내려감.
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

    // leaf면 proxy가 새 node 위치를 가리키도록 갱신함.
    // leaf면 proxy -> node 역참조가 정확한지만 확인하고 종료함.
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
    // parent의 child와 반대쪽 subtree의 grandchild 위치를 서로 교환함.
    std::swap( nodes_[downIndex], nodes_[upIndex] );

    // node 위치가 바뀌었으므로 proxy/parent 연결을 새 index 기준으로 복구함.
    LinkChildren( downIndex );
    LinkChildren( upIndex );

    // downIndex의 sibling subtree가 달라졌으므로 그 internal 정보를 다시 계산함.
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

    // 가능한 네 가지 local swap 중 perimeter를 가장 많이 줄이는 경우를 찾음.
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

    // 비용이 실제로 감소하는 후보가 있을 때만 회전함.
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

    // 기존 sibling과 새 leaf가 들어갈 연속된 두 슬롯을 확보함.
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

    // 원래 sibling 위치를 두 child를 감싸는 새 internal node로 바꿈.
    nodes_[siblingIndex] = MakeInternalNode( childPair );
    parents_[siblingIndex] = oldParent;

    // 바뀐 AABB와 height를 root까지 다시 계산하고 필요하면 회전함.
    RefitAncestors( siblingIndex, shouldRotate );
}

void DynamicTree::RemoveLeaf( std::int32_t leafIndex )
{
    // root leaf 하나만 있는 경우에는 root를 비우는 것으로 끝남.
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

    // 제거되는 leaf의 sibling만 남으므로 parent 위치까지 한 단계 승격시킴.
    // 제거되는 parent 자리로 살아남은 sibling을 승격한다.
    nodes_[parentIndex] = nodes_[siblingIndex];
    parents_[parentIndex] = grandParent;

    // sibling이 leaf면 proxy->node를, internal이면
    // 그 자식들의 parent를 새 위치에 맞게 고친다.
    LinkChildren( parentIndex );

    // 더 이상 필요 없는 두 child 슬롯을 pair free-list에 반환함.
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

            // 현재 child 상태를 기준으로 AABB, moved flag, height를 다시 만듦.
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
    // 재귀 진입 전에 node index가 실제 저장소 범위인지 확인함.
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

    // internal node는 root 이후의 짝수 index에서 시작하는 child pair를 가져야 함.
    const std::int32_t childPair =
        GetChildPair( node );

    if( childPair < 2 ||
        ( childPair & 1 ) != 0 ||
        static_cast<std::size_t>( childPair + 1 ) >= nodes_.size() )
    {
        return false;
    }

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

    // 두 subtree를 재귀 검증하면서 실제 height와 leaf 수를 다시 계산함.
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
