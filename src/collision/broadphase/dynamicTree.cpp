#include "collision/broadphase/dynamicTree.h"

#include <algorithm>
#include <cassert>
#include <limits>

namespace zonai
{

/*
* DynamicTree 구현 목표
*
* [Box2D DynamicTree 구조]
* - DynamicTree는 BroadPhase에서 shape proxy의 AABB를 관리하는 BVH임.
* - leaf node는 하나의 proxy를 나타내고, internal node는 두 child의 AABB를 합친 범위를 저장함.
* - 같은 parent를 공유하는 두 child를 형제 노드라고 하며, 두 형제 노드는 항상 연속된 index의 pair로 배치함.
* - stable proxy id와 tree 안에서 바뀔 수 있는 node index를 분리해서 관리함.
* - proxy는 AABB 외에도 category bits와 user data를 보관하고, leaf는 proxy id를 통해 이를 참조함.
*
* [삽입]
* - 새 leaf를 넣을 때 SAH를 이용해 함께 묶을 형제 노드를 찾음.
* - 기존 형제 노드와 새 leaf를 child로 가지는 internal node를 만들고 tree에 연결함.
* - 삽입 지점부터 root 방향으로 올라가며 AABB, height, moved 상태를 다시 계산함.
* - 신규 삽입에서는 local rotation을 시도해 AABB perimeter 비용을 줄임.
*
* [local rotation]
* - internal node와 아래 subtree의 연결 관계를 바꿨을 때 비용이 줄어드는지 비교함.
* - 가능한 swap 중 perimeter를 가장 많이 줄이는 경우에만 rotation을 수행함.
* - 기존 형제 노드 pair 안에서 node를 재배치하므로 pair의 연속 index 구조는 유지됨.
*
* [삭제]
* - leaf를 제거하면 같은 parent를 공유하던 형제 노드를 기존 parent 자리로 올림.
* - 더 이상 필요하지 않은 형제 노드 pair는 free-list에 반환함.
* - 변경 지점부터 root 방향으로 AABB, height, moved 상태를 다시 계산함.
*
* [이동 / AABB 갱신]
* - 일반적인 MoveProxy는 기존 leaf를 제거하고 새 AABB로 다시 삽입함.
* - MoveProxy 재삽입에서는 신규 삽입과 달리 local rotation을 수행하지 않음.
* - Box2D는 필요에 따라 proxy AABB를 넓히는 EnlargeProxy와 별도의 moved 표시 경로도 사용함.
*
* [검색]
* - Query는 주어진 AABB와 겹치는 leaf를 순회하며 callback으로 proxy를 전달함.
* - callback이 false를 반환하면 탐색을 조기에 종료할 수 있음.
* - category/mask bits로 후보를 필터링하고, node/leaf 방문 횟수를 통계로 반환함.
* - AABB Query 외에도 RayCast와 BoxCast를 DynamicTree에서 처리함.
*
* [moved / rebuild]
* - moved flag는 변경된 leaf에서 ancestor 방향으로 전파해 변경된 subtree를 표시함.
* - BroadPhase는 이 정보를 이용해 충돌 pair를 다시 만들어야 할 영역을 찾음.
* - moved 상태를 clear/gather하는 기능과 stale subtree를 다시 구성하는 Rebuild/Refit 기능을 가짐.
*
* [Zonai 현재 구현 상태]
* - [완료] root 0번, 1번 비움, 형제 노드 pair를 연속 index로 배치하는 node 구조.
* - [완료] stable proxy id와 node index 분리, proxy/형제 노드 pair free-list.
* - [완료] SAH 기반 형제 노드 탐색과 leaf 삽입.
* - [완료] local rotation과 ancestor refit.
* - [완료] leaf 삭제와 남은 형제 노드 승격.
* - [완료] MoveProxy의 제거 후 재삽입, 재삽입 시 rotation 생략.
* - [완료] AABB Query와 callback 조기 종료.
* - [완료] GetProxyAABB, GetProxyCount, GetHeight, GetAreaRatio, Validate.
* - [부분] moved flag 저장과 ancestor 전파는 구현됐지만 mark/clear/gather 수명 관리는 아직 없음.
* - [부분] TreeProxy에 userData 공간은 있지만 생성/검색 경로에서 아직 사용하지 않음.
* - [부분] Query는 겹치는 proxy 순회까지만 구현했고 category/mask filtering과 TreeStats는 아직 없음.
* - [미구현] category bits 설정/조회, userData 조회.
* - [미구현] EnlargeProxy와 별도의 proxy moved 표시 경로.
* - [미구현] RayCast, BoxCast.
* - [미구현] Rebuild, Refit, moved proxy gather/clear.
* - [미구현] root bounds, byte count 등 보조 통계/조회 기능.
*
* [DynamicTree 이후 BroadPhase 목표]
* - static / kinematic / dynamic body를 별도 DynamicTree로 관리함.
* - moved 상태가 있는 형제 노드 pair를 모아 변경된 subtree만 pair 생성 대상으로 사용함.
* - dynamic tree 내부는 형제 subtree끼리 self collision하고, static/kinematic tree와는 cross collision함.
* - 생성된 candidate pair는 기존 pair와 중복되는지 확인하고 collision filtering을 적용함.
* - 살아남은 shape pair를 Contact 생성 단계로 넘김.
*/

DynamicTree::DynamicTree()
{
    // root는 0번을 고정으로 쓰고 1번은 비워둠.
    // 같은 parent를 공유하는 두 형제 노드는 2,3 / 4,5 / ...처럼 연속된 pair로 배치함.
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

    // SAH로 새 leaf와 묶을 형제 노드를 찾고, 삽입 후 root까지 올라가며 local rotation을 시도함.
    InsertLeaf( newLeaf, true );

    return proxyId;
}

void DynamicTree::DestroyProxy( std::int32_t proxyId )
{
    assert( 0 <= proxyId );
    assert( static_cast<std::size_t>( proxyId ) < proxies_.size() );
    assert( proxies_[proxyId].node != NULL_INDEX );

    const std::int32_t leafIndex = proxies_[proxyId].node;

    // tree에서 leaf를 제거하고 남은 형제 노드를 parent 자리로 올림.
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
    // node 배열과 parent 배열의 기본 불변조건부터 확인
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

    // internal node AABB = 두 child AABB를 모두 감싸는 AABB.
    node.aabb = Union( child1.aabb, child2.aabb );

    // child 중 하나라도 moved면 parent도 moved 상태로 올림.
    node.flagIndex =
        static_cast<std::uint32_t>( childPair ) |
        ( ( child1.flagIndex | child2.flagIndex ) & TREE_MOVED_NODE );

    // internal node height = 더 깊은 child height + 1.
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

    // free-list의 head를 다음 빈 proxy로 이동함.
    proxyFreeList_ = proxy.next;

    // 꺼낸 proxy는 사용 중 상태로 초기화함.
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
    // 반납된 형제 노드 pair가 있으면 먼저 재활용함.
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
    // 형제 노드는 pair 단위로 관리하므로 시작 index는 항상 짝수여야 함.
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
		// root가 leaf면 형제가 없음
        return nodeIndex;
    }

	const float areaD = Perimeter( boxD ); // 새 leaf AABB 둘레

	aabb2 nodeBox = nodes_[nodeIndex].aabb; // root AABB
	float areaBase = Perimeter( nodeBox );  // root AABB 둘레
	float directCost = Perimeter( Union( nodeBox, boxD ) ); // root AABB와 새 leaf AABB를 합친 둘레
	float inheritedCost = 0.0f; // root부터 내려가며 늘어난 AABB 비용을 누적함.

    std::int32_t bestSibling = nodeIndex;
    float bestCost = directCost;

    // root부터 한쪽으로 내려가며 새 leaf와 묶을 비용이 가장 작은 형제 노드를 찾음.
    for( ;; )
    {
        const std::int32_t child1 = GetChildPair( nodes_[nodeIndex] );
        const std::int32_t child2 = child1 + 1;

        // 현재 node를 형제 노드로 선택했을 때의 비용을 계산함.
        // 지금 비용 = union( 현재 노드, newAABB ) + 비용 누적 
        const float currentCost = directCost + inheritedCost;

        if( currentCost < bestCost )
        {
            bestSibling = nodeIndex;
            bestCost = currentCost;
        }

        // 새 AABB를 포함하면서 현재 노드의 perimeter가 증가한 만큼 누적 비용에 더함.
        // 누적 비용 += Perimeter( Union( 현재 노드 AABB, 새 AABB ) ) - Perimeter( 현재 노드 AABB )
        inheritedCost += ( directCost - areaBase );

        const bool leaf1 = IsLeaf( nodes_[child1] );
        const bool leaf2 = IsLeaf( nodes_[child2] );
        const aabb2 leftBox = nodes_[child1].aabb;
        const aabb2 rightBox = nodes_[child2].aabb;

        // 각 자식들과 newLeaf를 포함하는 AABB의 둘레를 계산
        const float leftUnionPerimeter = Perimeter( Union( leftBox, boxD ) );
        const float rightUnionPerimeter = Perimeter( Union( rightBox, boxD ) );

        float leftPerimeter = 0.0f;
        float rightPerimeter = 0.0f;

        float leftCost = std::numeric_limits<float>::max();
        float rightCost = std::numeric_limits<float>::max();

        if( leaf1 )
        {
            const float cost1 = leftUnionPerimeter + inheritedCost;

            if( cost1 < bestCost )
            {
                bestSibling = child1;
                bestCost = cost1;
            }
        }
        else
        {
            leftPerimeter = Perimeter( leftBox );

            // child1 subtree에서 나올 수 있는 최소 비용을 예상함.
            // areaD < leftPerimeter이면 더 작은 자식과 묶일 가능성을 lower bound에 반영함.
            leftCost = inheritedCost + leftUnionPerimeter + std::min( areaD - leftPerimeter, 0.0f );
        }

        if( leaf2 )
        {
            const float cost2 = rightUnionPerimeter + inheritedCost;

            if( cost2 < bestCost )
            {
                bestSibling = child2;
                bestCost = cost2;
            }
        }
        else
        {
            rightPerimeter = Perimeter( rightBox );

            rightCost = inheritedCost + rightUnionPerimeter + std::min( areaD - rightPerimeter, 0.0f );
        }

        // 둘 다 리프면 더 내려갈 곳이 없어서 끝냄.
        if( leaf1 && leaf2 )
        {
            break;
        }

        // 어느 자식으로 내려가도 현재 best보다 싸질 수 없으면 가지치기함.
        if( bestCost <= leftCost && bestCost <= rightCost )
        {
            break;
        }

        // 더 싸질 가능성이 있는 자식 쪽으로만 내려감.
        if( leftCost <= rightCost )
        {
            nodeIndex = child1;
            areaBase = leftPerimeter;
            directCost = leftUnionPerimeter;
        }
        else
        {
            nodeIndex = child2;
            areaBase = rightPerimeter;
            directCost = rightUnionPerimeter;
        }
    }

    return bestSibling;
}

void DynamicTree::LinkChildren( std::int32_t nodeIndex )
{
    const TreeNode& node = nodes_[nodeIndex];

    // leaf가 이동한 경우 stable proxy가 현재 node index를 다시 가리키게 함.
    if( IsLeaf( node ) )
    {
        const std::int32_t proxyId = GetProxyId( node );
        proxies_[proxyId].node = nodeIndex;
        return;
    }

    // internal node면 두 child가 현재 node를 parent로 가리키게 맞춤.
    const std::int32_t childPair = GetChildPair( node );

    parents_[childPair] = nodeIndex;
    parents_[childPair + 1] = nodeIndex;
}

void DynamicTree::SwapNodes( std::int32_t downIndex, std::int32_t upIndex )
{
    // rotation에서 아래로 내릴 child와 위로 올릴 grandchild의 node 내용을 교환함.
    std::swap( nodes_[downIndex], nodes_[upIndex] );

    // node index가 바뀌었으므로 leaf면 proxy, internal이면 child의 parent 연결을 다시 맞춤.
    LinkChildren( downIndex );
    LinkChildren( upIndex );

    // downIndex와 같은 pair의 반대 node는 rotation 후 child 구성이 바뀐 internal node임.
    // 새 child 기준으로 AABB, moved flag, height를 다시 계산함.
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

    // 현재 구조는 A-(B,C). B와 C가 모두 leaf면 바꿔 올릴 grandchild가 없어서 rotation 불가.
    if( leafB && leafC )
    {
        return;
    }

    // bestDown : 아래로 내려갈 child
    // bestUp   : 위로 올라올 grandchild
    // bestDelta: rotation 전후 perimeter 차이. 음수일수록 비용이 더 많이 줄어듦.
    std::int32_t bestDown = NULL_INDEX;
    std::int32_t bestUp = NULL_INDEX;
    float bestDelta = 0.0f;

    // 가능한 swap 조합 중 perimeter를 가장 많이 줄이는 경우를 찾음.
    // C가 internal이면 C=(F,G). B <-> F, B <-> G 두 경우를 비교함.
    if( !leafC )
    {
        const std::int32_t indexF = GetChildPair( nodeC );
        const std::int32_t indexG = indexF + 1;

        const float areaC = Perimeter( nodeC.aabb );

        // B <-> F 후 C=(B,G)
        // 비용 변화 = Perimeter( Union( B, G ) ) - 기존 C perimeter
        const float deltaBF =
            Perimeter( Union( nodeB.aabb, nodes_[indexG].aabb ) ) - areaC;

        if( deltaBF < bestDelta )
        {
            bestDown = indexB;
            bestUp = indexF;
            bestDelta = deltaBF;
        }

        // B <-> G 후 C=(F,B)
        // 비용 변화 = Perimeter( Union( F, B ) ) - 기존 C perimeter
        const float deltaBG =
            Perimeter( Union( nodeB.aabb, nodes_[indexF].aabb ) ) - areaC;

        if( deltaBG < bestDelta )
        {
            bestDown = indexB;
            bestUp = indexG;
            bestDelta = deltaBG;
        }
    }

    // B가 internal이면 B=(D,E). C <-> D, C <-> E 두 경우를 비교함.
    if( !leafB )
    {
        const std::int32_t indexD =
            GetChildPair( nodeB );
        const std::int32_t indexE =
            indexD + 1;

        const float areaB =
            Perimeter( nodeB.aabb );

        // C <-> D 후 B=(C,E)
        // 비용 변화 = Perimeter( Union( C, E ) ) - 기존 B perimeter
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

        // C <-> E 후 B=(D,C)
        // 비용 변화 = Perimeter( Union( D, C ) ) - 기존 B perimeter
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

    // bestDelta가 0보다 작은 경우만 후보가 선택되므로 실제 perimeter가 줄어드는 rotation만 수행함.
    if( bestDown != NULL_INDEX )
    {
        SwapNodes( bestDown, bestUp );
    }
}

void DynamicTree::InsertLeaf( const TreeNode& leaf, bool shouldRotate )
{
    // 새 leaf와 묶였을 때 비용이 가장 작은 형제 노드를 찾음.
    const std::int32_t siblingIndex = FindBestSibling( leaf.aabb );

    // 형제 노드 자리를 새 parent로 바꿀 예정이므로 기존 parent를 보관함.
    const std::int32_t oldParent = parents_[siblingIndex];

    // 형제 노드와 새 leaf를 담을 pair를 확보함.
    const std::int32_t childPair = AllocateSiblingPair();

    // 형제 노드 자리는 새 parent로 재사용하고 기존 형제 노드는 pair로 내려보냄.
    nodes_[childPair] = nodes_[siblingIndex];
    nodes_[childPair + 1] = leaf;

    parents_[childPair] = siblingIndex;
    parents_[childPair + 1] = siblingIndex;

    // pair로 이동한 형제 노드와 새 leaf의 proxy / child 연결을 새 index에 맞춤.
    LinkChildren( childPair );
    LinkChildren( childPair + 1 );

    // 기존 형제 노드 자리를 두 child를 감싸는 internal node로 바꿈.
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

    const std::int32_t childPair = GetChildPair( nodes_[parentIndex] );

    assert(
        leafIndex == childPair ||
        leafIndex == childPair + 1
    );

    // 같은 pair의 반대 node가 삭제 후 살아남을 형제 노드임.
    const std::int32_t siblingIndex =
        leafIndex == childPair ?
            childPair + 1 : childPair;

    const std::int32_t grandParent =
        parents_[parentIndex];

    // 살아남은 형제 노드를 제거되는 parent 자리로 올림.
    nodes_[parentIndex] = nodes_[siblingIndex];
    parents_[parentIndex] = grandParent;

    // 올라온 node 종류에 맞춰 proxy 또는 child parent 연결을 다시 맞춤.
    LinkChildren( parentIndex );

    // parent가 사라지면서 더 이상 필요하지 않은 child pair를 free-list에 반환함.
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

            // rotation으로 child 구성이 바뀔 수 있으므로 child pair는 rotation 뒤 다시 읽음.
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
