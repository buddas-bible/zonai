#include "collision/broadphase/dynamicTree.h"

#include <algorithm>
#include <cassert>
#include <limits>

namespace zonai
{

/*
* DynamicTree 구현 메모
*
* [구조]
* - BroadPhase에서 proxy AABB를 관리하는 BVH임.
* - leaf는 proxy 하나를 나타내고 internal node는 두 child AABB의 Union을 저장함.
* - root는 0번, 1번은 비워두고 같은 parent의 두 child를 연속된 pair로 배치함.
* - stable proxy id와 tree 내부에서 바뀔 수 있는 node index를 분리함.
*
* [주요 흐름]
* - 삽입: SAH로 새 leaf와 묶을 형제 노드를 찾고 parent를 만든 뒤 root까지 refit함.
* - 신규 삽입에서는 refit 중 local rotation을 시도해 perimeter 비용을 줄임.
* - 삭제: 남은 형제 노드를 parent 자리로 올리고 비어진 pair를 free-list에 반환함.
* - 이동: 기존 leaf를 제거하고 같은 proxy id로 재삽입함. MoveProxy에서는 rotation을 생략함.
* - Query: AABB가 겹치는 subtree만 내려가며 leaf proxy를 callback에 전달함.
*
* [현재 구현]
* - proxy / sibling pair free-list, SAH 삽입, 삭제, MoveProxy, Query, local rotation, Validate 구현함.
* - CreateProxy / MoveProxy는 필요할 때만 moved를 표시하고 ancestor로 전파함. ClearMoved로 소비 후 초기화함.
* - TreeProxy에 userData 공간은 있지만 생성/조회 경로에는 아직 연결하지 않음.
* - category / mask filtering과 TreeStats는 아직 없음.
*
* [Box2D에서 이어서 참고할 기능]
* - category bits / userData 조회, EnlargeProxy
* - CastRay / CastBox
* - moved mark / clear / gather, Rebuild / Refit
* - root bounds / byte count 등 보조 조회 기능
*
* [BroadPhase 다음 목표]
* - static / kinematic / dynamic body를 별도 tree로 관리함.
* - moved 형제 pair를 기준으로 dynamic self collision과 static / kinematic cross collision 후보를 생성함.
* - 중복 제거와 collision filtering 후 살아남은 pair를 Contact 생성 단계로 넘김.
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

std::int32_t DynamicTree::CreateProxy( const aabb2& aabb, std::int32_t shapeIndex, bool markMoved )
{
    assert( IsValidAABB( aabb ) );
    assert( shapeIndex >= 0 );

    // 사용할 proxy id를 free-list에서 확보함.
    const std::int32_t proxyId = AllocateProxy();

    // Box2D처럼 proxy와 leaf 양쪽에 shape index를 보관해 mapping 불변조건을 검증할 수 있게 함.
    proxies_[proxyId].userData = static_cast<std::uint64_t>( shapeIndex );

    const TreeNode newLeaf = MakeLeafNode( aabb, proxyId, shapeIndex, markMoved );
    
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

    assert( static_cast<std::size_t>( leafIndex ) < nodes_.size() );
    assert( IsLeaf( nodes_[leafIndex] ) );
    assert( GetProxyId( nodes_[leafIndex] ) == proxyId );

    // tree에서 leaf를 제거하고 남은 형제 노드를 parent 자리로 올림.
    RemoveLeaf( leafIndex );

    // leaf 수명이 끝났으므로 proxy id도 free-list에 반환함.
    FreeProxy( proxyId );
}

void DynamicTree::MoveProxy( std::int32_t proxyId, const aabb2& aabb, bool markMoved )
{
    assert( IsValidAABB( aabb ) );
    assert( aabb.max.x - aabb.min.x < MAX_TREE_AABB_EXTENT );
    assert( aabb.max.y - aabb.min.y < MAX_TREE_AABB_EXTENT );

    assert( 0 <= proxyId );
    assert( static_cast<std::size_t>( proxyId ) < proxies_.size() );
    assert( proxies_[proxyId].node != NULL_INDEX );

    const std::int32_t leafIndex = proxies_[proxyId].node;

    assert( static_cast<std::size_t>( leafIndex ) < nodes_.size() );
    assert( IsLeaf( nodes_[leafIndex] ) );
    assert( GetProxyId( nodes_[leafIndex] ) == proxyId );

    // 재삽입해도 stable proxy id와 shape index는 그대로 유지함.
    const std::int32_t shapeIndex = nodes_[leafIndex].shapeIndex;

    assert( static_cast<std::int32_t>( proxies_[proxyId].userData ) == shapeIndex );

    // 기존 leaf만 tree에서 제거해서 proxy id의 수명은 유지함.
    RemoveLeaf( leafIndex );

    // 같은 proxy id로 새 AABB의 leaf를 만들어 다시 연결함.
    const TreeNode newLeaf = MakeLeafNode( aabb, proxyId, shapeIndex, markMoved );

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

bool DynamicTree::HasMoved() const
{
    if( proxyCount_ == 0 )
    {
        return false;
    }

    return ( nodes_[ROOT_NODE].flagIndex & TREE_MOVED_NODE ) != 0;
}

bool DynamicTree::NeedsRebuild() const
{
    // 최신 Box2D처럼 moved branch뿐 아니라 DFS node 순서가 깨진 경우에도 rebuild가 필요함.
    return HasMoved() || !dfsOrdered_;
}

std::size_t DynamicTree::Rebuild( bool fullBuild )
{
    if( proxyCount_ == 0 )
    {
        return 0;
    }

    // DFS 순서도 정상이고 moved branch도 없으면 partial rebuild는 할 일이 없음.
    if( !fullBuild && !NeedsRebuild() )
    {
        return 0;
    }

    rebuildLeafIndices_.resize( proxyCount_ );
    rebuildLeafNodes_.resize( proxyCount_ );
    rebuildLeafCenters_.resize( proxyCount_ );

    const TreeNode& root = nodes_[ROOT_NODE];

    std::size_t leafCount = 0;
    std::array<std::int32_t, TREE_STACK_SIZE> stack{};
    std::size_t stackCount = 0;

    auto addBuildLeaf =
        [&]( TreeNode node )
        {
            // build leaf로 채택된 subtree는 이번 rebuild에서 소비 완료된 상태로 저장함.
            node.flagIndex &= ~TREE_MOVED_NODE;

            rebuildLeafIndices_[leafCount] = static_cast<std::int32_t>( leafCount );
            rebuildLeafNodes_[leafCount] = node;
            rebuildLeafCenters_[leafCount] = Center( node.aabb );
            ++leafCount;
        };

    // moved internal node만 펼치고 untouched subtree는 하나의 build leaf로 유지함.
    if( !IsLeaf( root ) &&
        ( fullBuild || ( root.flagIndex & TREE_MOVED_NODE ) != 0 ) )
    {
        stack[stackCount++] = GetChildPair( root );
    }
    else
    {
        addBuildLeaf( root );
    }

    while( stackCount > 0 )
    {
        const std::int32_t pair = stack[--stackCount];

        for( std::int32_t i = 0; i < 2; ++i )
        {
            TreeNode node = nodes_[pair + i];

            if( !IsLeaf( node ) &&
                ( fullBuild || ( node.flagIndex & TREE_MOVED_NODE ) != 0 ) )
            {
                assert( stackCount < stack.size() );
                stack[stackCount++] = GetChildPair( node );
                continue;
            }

            addBuildLeaf( node );
        }
    }

    assert( leafCount > 0 );
    assert( leafCount <= proxyCount_ );

    BuildRebuildTree( leafCount );

    // 새 배열은 DFS 순서로 조밀하게 구성되므로 free pair가 남지 않음.
    nodes_.swap( rebuildNodes_ );
    pairFreeList_ = NULL_INDEX;
    dfsOrdered_ = true;

    assert( Validate() );
    assert( !HasMoved() );

    return leafCount;
}

void DynamicTree::ClearMoved()
{
    if( !HasMoved() )
    {
        return;
    }

    TreeNode& root = nodes_[ROOT_NODE];
    root.flagIndex &= ~TREE_MOVED_NODE;

    if( IsLeaf( root ) )
    {
        return;
    }

    std::array<std::int32_t, TREE_STACK_SIZE> stack{};
    std::size_t stackCount = 0;
    stack[stackCount++] = GetChildPair( root );

    // moved가 전파된 branch만 내려가며 flag를 지움.
    while( stackCount > 0 )
    {
        const std::int32_t pair = stack[--stackCount];

        for( std::int32_t i = 0; i < 2; ++i )
        {
            TreeNode& node = nodes_[pair + i];

            if( ( node.flagIndex & TREE_MOVED_NODE ) == 0 )
            {
                continue;
            }

            node.flagIndex &= ~TREE_MOVED_NODE;

            if( IsLeaf( node ) )
            {
                continue;
            }

            assert( stackCount < TREE_STACK_SIZE );
            stack[stackCount++] = GetChildPair( node );
        }
    }
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

    // SAH가 줄일 수 있는 비용은 root와 leaf를 제외한 internal node 영역임.
    float internalPerimeter = 0.0f;

    for( std::size_t i = 2; i < nodes_.size(); ++i )
    {
        const TreeNode& node = nodes_[i];

        // empty/free node와 leaf는 둘 다 leaf tagged이므로 internal만 합산함.
        if( IsLeaf( node ) )
        {
            continue;
        }

        internalPerimeter += Perimeter( node.aabb );
    }

    return internalPerimeter / rootPerimeter;
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

void DynamicTree::SetChildPair( TreeNode& node, std::int32_t pair )
{
    node.flagIndex =
        ( node.flagIndex & ~TREE_NODE_INDEX_MASK ) |
        static_cast<std::uint32_t>( pair );
}

bool DynamicTree::IsNodeOrdered( std::int32_t nodeIndex ) const
{
    const TreeNode& node = nodes_[nodeIndex];

    // leaf는 child가 없어서 항상 ordered이고 internal node는 parent index가 child pair보다 앞에 있어야 함.
    return IsLeaf( node ) || nodeIndex < GetChildPair( node );
}

TreeNode DynamicTree::MakeEmptyNode()
{
    TreeNode node{};

    // Box2D와 같은 sentinel AABB.
    // min > max인 inverted box라 어떤 정상 AABB와도 overlap하지 않음.
    const float infinity = std::numeric_limits<float>::infinity();

    node.aabb = {
        { infinity, infinity },
        { -infinity, -infinity }
    };

    node.flagIndex = TREE_EMPTY_NODE;
    node.height = 0;

    return node;
}

TreeNode DynamicTree::MakeLeafNode(
    const aabb2& aabb,
    std::int32_t proxyId,
    std::int32_t shapeIndex,
    bool moved )
{
    TreeNode node{};

    node.aabb = aabb;
    node.flagIndex = static_cast<std::uint32_t>( proxyId ) | TREE_LEAF_NODE;

    if( moved )
    {
        node.flagIndex |= TREE_MOVED_NODE;
    }

    node.shapeIndex = shapeIndex;

    return node;
}

TreeNode DynamicTree::MakeInternalNodeFrom(
    const std::vector<TreeNode>& nodes,
    std::int32_t childPair )
{
    const TreeNode& child1 = nodes[childPair];
    const TreeNode& child2 = nodes[childPair + 1];

    TreeNode node{};

    // 두 child 기준으로 AABB, moved flag, height를 다시 구성함.
    node.aabb = Union( child1.aabb, child2.aabb );

    node.flagIndex =
        static_cast<std::uint32_t>( childPair ) |
        ( ( child1.flagIndex | child2.flagIndex ) & TREE_MOVED_NODE );

    node.height = 1 + std::max( GetNodeHeight( child1 ), GetNodeHeight( child2 ) );

    return node;
}

TreeNode DynamicTree::MakeInternalNode( std::int32_t childPair ) const
{
    return MakeInternalNodeFrom( nodes_, childPair );
}

std::size_t DynamicTree::PartitionRebuildLeaves(
    std::size_t startIndex,
    std::size_t count )
{
    if( count <= 2 )
    {
        return count / 2;
    }

    vec2 lower = rebuildLeafCenters_[startIndex];
    vec2 upper = lower;

    for( std::size_t i = 1; i < count; ++i )
    {
        const vec2 center = rebuildLeafCenters_[startIndex + i];

        lower.x = std::min( lower.x, center.x );
        lower.y = std::min( lower.y, center.y );
        upper.x = std::max( upper.x, center.x );
        upper.y = std::max( upper.y, center.y );
    }

    const vec2 extent = upper - lower;
    const vec2 midpoint = ( lower + upper ) * 0.5f;

    std::size_t first = 0;
    std::size_t last = count;

    // 최신 Box2D처럼 center가 가장 넓게 퍼진 축의 중간값을 기준으로 Hoare partition함.
    if( extent.x > extent.y )
    {
        const float pivot = midpoint.x;

        while( first < last )
        {
            while( first < last &&
                rebuildLeafCenters_[startIndex + first].x < pivot )
            {
                ++first;
            }

            while( first < last &&
                rebuildLeafCenters_[startIndex + last - 1].x >= pivot )
            {
                --last;
            }

            if( first < last )
            {
                std::swap(
                    rebuildLeafIndices_[startIndex + first],
                    rebuildLeafIndices_[startIndex + last - 1] );
                std::swap(
                    rebuildLeafCenters_[startIndex + first],
                    rebuildLeafCenters_[startIndex + last - 1] );

                ++first;
                --last;
            }
        }
    }
    else
    {
        const float pivot = midpoint.y;

        while( first < last )
        {
            while( first < last &&
                rebuildLeafCenters_[startIndex + first].y < pivot )
            {
                ++first;
            }

            while( first < last &&
                rebuildLeafCenters_[startIndex + last - 1].y >= pivot )
            {
                --last;
            }

            if( first < last )
            {
                std::swap(
                    rebuildLeafIndices_[startIndex + first],
                    rebuildLeafIndices_[startIndex + last - 1] );
                std::swap(
                    rebuildLeafCenters_[startIndex + first],
                    rebuildLeafCenters_[startIndex + last - 1] );

                ++first;
                --last;
            }
        }
    }

    assert( first == last );

    // 한쪽이 비는 퇴화 partition은 단순 중앙 분할로 보정함.
    if( first == 0 || first == count )
    {
        return count / 2;
    }

    return first;
}

std::int32_t DynamicTree::BumpRebuildPair(
    std::int32_t parent,
    std::int32_t& nodeEnd )
{
    const std::int32_t pair = nodeEnd;

    assert( pair >= 2 );
    assert( ( pair & 1 ) == 0 );
    assert( static_cast<std::size_t>( pair + 1 ) < rebuildNodes_.size() );

    nodeEnd += 2;
    parents_[pair] = parent;
    parents_[pair + 1] = parent;

    return pair;
}

void DynamicTree::CopySubtree(
    TreeNode node,
    std::int32_t newIndex,
    std::int32_t& nodeEnd )
{
    if( IsLeaf( node ) )
    {
        rebuildNodes_[newIndex] = node;
        proxies_[GetProxyId( node )].node = newIndex;
        return;
    }

    std::array<CopyItem, TREE_STACK_SIZE> stack{};
    std::size_t stackCount = 0;

    std::int32_t oldPair = GetChildPair( node );
    std::int32_t newPair = BumpRebuildPair( newIndex, nodeEnd );
    SetChildPair( node, newPair );

    // retained subtree root를 새 DFS 배열의 지정된 위치에 복사함.
    rebuildNodes_[newIndex] = node;

    for( ;; )
    {
        rebuildNodes_[newPair] = nodes_[oldPair];
        rebuildNodes_[newPair + 1] = nodes_[oldPair + 1];

        TreeNode& right = rebuildNodes_[newPair + 1];

        if( IsLeaf( right ) )
        {
            proxies_[GetProxyId( right )].node = newPair + 1;
        }
        else
        {
            assert( stackCount < stack.size() );
            stack[stackCount++] = { GetChildPair( right ), newPair + 1 };
        }

        TreeNode& left = rebuildNodes_[newPair];

        if( !IsLeaf( left ) )
        {
            const std::int32_t leftIndex = newPair;

            oldPair = GetChildPair( left );
            newPair = BumpRebuildPair( leftIndex, nodeEnd );
            SetChildPair( rebuildNodes_[leftIndex], newPair );
            continue;
        }

        proxies_[GetProxyId( left )].node = newPair;

        if( stackCount == 0 )
        {
            break;
        }

        const CopyItem item = stack[--stackCount];

        oldPair = item.oldPair;
        newPair = BumpRebuildPair( item.newIndex, nodeEnd );
        SetChildPair( rebuildNodes_[item.newIndex], newPair );
    }
}

void DynamicTree::PlaceRebuildLeaf(
    const TreeNode& node,
    std::int32_t newIndex,
    std::int32_t& nodeEnd )
{
    if( IsLeaf( node ) )
    {
        rebuildNodes_[newIndex] = node;
        proxies_[GetProxyId( node )].node = newIndex;
        return;
    }

    CopySubtree( node, newIndex, nodeEnd );
}

void DynamicTree::BuildRebuildTree( std::size_t leafCount )
{
    const std::size_t nodeCount = std::max<std::size_t>( 2, 2 * proxyCount_ );

    rebuildNodes_.resize( nodeCount );
    parents_.resize( nodeCount, NULL_INDEX );

    std::int32_t nodeEnd = 2;

    rebuildNodes_[ROOT_NODE + 1] = MakeEmptyNode();
    parents_[ROOT_NODE] = NULL_INDEX;
    parents_[ROOT_NODE + 1] = NULL_INDEX;

    if( leafCount == 1 )
    {
        PlaceRebuildLeaf(
            rebuildLeafNodes_[rebuildLeafIndices_[0]],
            ROOT_NODE,
            nodeEnd );

        assert( static_cast<std::size_t>( nodeEnd ) == nodeCount );
        return;
    }

    std::array<RebuildItem, TREE_STACK_SIZE> stack{};
    std::size_t top = 0;

    stack[0].nodeIndex = ROOT_NODE;
    stack[0].pair = BumpRebuildPair( ROOT_NODE, nodeEnd );
    stack[0].childCount = -1;
    stack[0].startIndex = 0;
    stack[0].endIndex = leafCount;
    stack[0].splitIndex = PartitionRebuildLeaves( 0, leafCount );

    for( ;; )
    {
        RebuildItem& item = stack[top];
        ++item.childCount;

        if( item.childCount == 2 )
        {
            rebuildNodes_[item.nodeIndex] =
                MakeInternalNodeFrom( rebuildNodes_, item.pair );

            if( top == 0 )
            {
                break;
            }

            --top;
            continue;
        }

        const std::int32_t slot = item.childCount;
        const std::size_t startIndex =
            slot == 0 ? item.startIndex : item.splitIndex;
        const std::size_t endIndex =
            slot == 0 ? item.splitIndex : item.endIndex;
        const std::size_t count = endIndex - startIndex;

        assert( count > 0 );

        const std::int32_t nodeIndex = item.pair + slot;

        if( count == 1 )
        {
            PlaceRebuildLeaf(
                rebuildLeafNodes_[rebuildLeafIndices_[startIndex]],
                nodeIndex,
                nodeEnd );
            continue;
        }

        assert( top + 1 < stack.size() );

        ++top;

        RebuildItem& child = stack[top];

        child.nodeIndex = nodeIndex;
        child.pair = BumpRebuildPair( nodeIndex, nodeEnd );
        child.childCount = -1;
        child.startIndex = startIndex;
        child.endIndex = endIndex;
        child.splitIndex =
            startIndex + PartitionRebuildLeaves( startIndex, count );
    }

    assert( static_cast<std::size_t>( nodeEnd ) == nodeCount );
}

std::int32_t DynamicTree::AllocateProxy()
{
    // free-list가 비었으면 proxy pool을 50% 정도 늘림.
    if( proxyFreeList_ == NULL_INDEX )
    {
        assert( proxyCount_ == proxies_.size() );

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

    // free-list head에서 proxy 하나를 꺼내고 다음 빈 proxy로 head를 옮김.
    const std::int32_t proxyId = proxyFreeList_;
    TreeProxy& proxy = proxies_[proxyId];

    proxyFreeList_ = proxy.next;

    // 꺼낸 proxy를 사용 상태로 초기화함.
    proxy.userData = 0;
    proxy.node = NULL_INDEX;
    proxy.next = NULL_INDEX;

    ++proxyCount_;

    return proxyId;
}

void DynamicTree::FreeProxy( std::int32_t proxyId )
{
    assert( proxyId >= 0 );
    assert( static_cast<std::size_t>( proxyId ) < proxies_.size() );
    assert( proxyCount_ > 0 );

    TreeProxy& proxy = proxies_[proxyId];

    // 반환된 proxy id를 free-list head 앞에 다시 붙임.
    proxy.userData = 0;
    proxy.node = NULL_INDEX;
    proxy.next = proxyFreeList_;

    proxyFreeList_ = proxyId;

    --proxyCount_;
}

std::int32_t DynamicTree::AllocateSiblingPair()
{
    // free-list head에서 반납된 pair 하나를 꺼내고 다음 pair로 head를 옮김.
    if( pairFreeList_ != NULL_INDEX )
    {
        const std::int32_t pair = pairFreeList_;

        pairFreeList_ = parents_[pair];

        // 재사용하기 전에 node와 parent 상태를 초기화함.
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
    assert( static_cast<std::size_t>( pair + 1 ) < nodes_.size() );

    nodes_[pair] = MakeEmptyNode();
    nodes_[pair + 1] = MakeEmptyNode();

    // free pair의 parents_[pair]를 next로 재활용하고 기존 head 앞에 붙임.
    parents_[pair] = pairFreeList_;
    parents_[pair + 1] = NULL_INDEX;

    pairFreeList_ = pair;
}

std::int32_t DynamicTree::FindBestSibling( const aabb2& boxD ) const
{
    std::int32_t nodeIndex = ROOT_NODE;

    if( IsLeaf( nodes_[nodeIndex] ) )
    {
		// 더 내려갈 child가 없으므로 root를 새 leaf의 형제 노드로 선택함.
        return nodeIndex;
    }

	const float areaD = Perimeter( boxD ); // 새 leaf perimeter

	aabb2 nodeBox = nodes_[nodeIndex].aabb; // 현재 node AABB
	float areaBase = Perimeter( nodeBox );  // 현재 node perimeter
	float directCost = Perimeter( Union( nodeBox, boxD ) ); // 새 leaf를 포함한 Union perimeter
	float inheritedCost = 0.0f; // ancestor에서 누적된 perimeter 증가 비용

    std::int32_t bestSibling = nodeIndex;
    float bestCost = directCost;
    const vec2 centerD = Center( boxD );

    // root부터 한쪽으로 내려가며 새 leaf와 묶을 비용이 가장 작은 형제 노드를 찾음.
    for( ;; )
    {
        const std::int32_t child1 = GetChildPair( nodes_[nodeIndex] );
        const std::int32_t child2 = child1 + 1;

        // 현재 node를 형제 노드로 선택하면 Union perimeter에 ancestor 증가 비용까지 부담함.
        // 선택 비용 = directCost + inheritedCost
        const float currentCost = directCost + inheritedCost;

        if( currentCost < bestCost )
        {
            bestSibling = nodeIndex;
            bestCost = currentCost;
        }

        // 아래로 더 내려갈 때도 현재 node가 새 leaf를 포함하며 늘어난 perimeter는 공통으로 부담함.
        // inheritedCost += Union perimeter - 기존 node perimeter
        inheritedCost += ( directCost - areaBase );

        const bool leaf1 = IsLeaf( nodes_[child1] );
        const bool leaf2 = IsLeaf( nodes_[child2] );
        const aabb2 leftBox = nodes_[child1].aabb;
        const aabb2 rightBox = nodes_[child2].aabb;

        // 각 child를 새 leaf와 바로 묶었을 때의 Union perimeter를 구함.
        const float leftUnionPerimeter = Perimeter( Union( leftBox, boxD ) );
        const float rightUnionPerimeter = Perimeter( Union( rightBox, boxD ) );

        float leftPerimeter = 0.0f;
        float rightPerimeter = 0.0f;

        // internal child는 실제 sibling이 아직 정해지지 않았으므로 subtree에서 가능한 최소 비용을 저장함.
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

            // child1 자체를 선택하거나 descendant까지 내려가는 경우 중 가능한 최소 비용을 구함.
            // areaD < leftPerimeter이면 작은 descendant와 묶일 가능성만큼 lower bound가 낮아짐.
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

        // lower bound가 같으면 Box2D처럼 새 leaf 중심과 더 가까운 subtree를 선택함.
        if( leftCost == rightCost && !leaf1 )
        {
            assert( leftCost < std::numeric_limits<float>::max() );
            assert( rightCost < std::numeric_limits<float>::max() );

            const vec2 delta1 = Center( leftBox ) - centerD;
            const vec2 delta2 = Center( rightBox ) - centerD;

            leftCost = LengthSquared( delta1 );
            rightCost = LengthSquared( delta2 );
        }

        // 더 싸질 가능성이 있는 internal child 쪽으로만 내려감.
        if( leftCost < rightCost && !leaf1 )
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

    // rotation으로 parent가 child보다 뒤 index로 이동할 수 있으므로 DFS 배열 순서를 추적함.
    if( !IsNodeOrdered( downIndex ) || !IsNodeOrdered( upIndex ) )
    {
        dfsOrdered_ = false;
    }

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
        const aabb2 BGUnion = Union( nodeB.aabb, nodes_[indexG].aabb );
        const float BGUnionPerimeter = Perimeter( BGUnion );
        const float deltaBF = BGUnionPerimeter - areaC;

        if( deltaBF < bestDelta )
        {
            bestDown = indexB;
            bestUp = indexF;
            bestDelta = deltaBF;
        }

        // B <-> G 후 C=(F,B)
        // 비용 변화 = Perimeter( Union( F, B ) ) - 기존 C perimeter
        const aabb2 FBUnion = Union( nodeB.aabb, nodes_[indexF].aabb );
        const float FBUnionPerimeter = Perimeter( FBUnion );
        const float deltaBG = FBUnionPerimeter - areaC;

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
        const std::int32_t indexD = GetChildPair( nodeB );
        const std::int32_t indexE = indexD + 1;

        const float areaB = Perimeter( nodeB.aabb );

        // C <-> D 후 B=(C,E)
        // 비용 변화 = Perimeter( Union( C, E ) ) - 기존 B perimeter
        const aabb2 CEUnion = Union( nodeC.aabb, nodes_[indexE].aabb );
        const float CEUnionPerimeter = Perimeter( CEUnion );
        const float deltaCD = CEUnionPerimeter - areaB;

        if( deltaCD < bestDelta )
        {
            bestDown = indexC;
            bestUp = indexD;
            bestDelta = deltaCD;
        }

        // C <-> E 후 B=(D,C)
        // 비용 변화 = Perimeter( Union( D, C ) ) - 기존 B perimeter
        const aabb2 CDUnion = Union( nodeC.aabb, nodes_[indexD].aabb );
        const float CDUnionPerimeter = Perimeter( CDUnion );
        const float deltaCE = CDUnionPerimeter - areaB;

        if( deltaCE < bestDelta )
        {
            bestDown = indexC;
            bestUp = indexE;
            bestDelta = deltaCE;
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

    // internal sibling을 새 pair로 내려보내면 기존 child가 더 앞 index에 남아 DFS 순서가 깨질 수 있음.
    if( !IsNodeOrdered( siblingIndex ) || !IsNodeOrdered( childPair ) )
    {
        dfsOrdered_ = false;
    }

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

void DynamicTree::RefitAncestors( std::int32_t nodeIndex, bool shouldRotate )
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

bool DynamicTree::ValidateSubtree( std::int32_t nodeIndex, std::int32_t& height, std::size_t& leafCount ) const
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
        const std::int32_t proxyId = GetProxyId( node );

        if( proxyId < 0 ||
            static_cast<std::size_t>( proxyId ) >= proxies_.size() ||
            proxies_[proxyId].node != nodeIndex ||
            static_cast<std::int32_t>( proxies_[proxyId].userData ) != node.shapeIndex )
        {
            return false;
        }

        height = 0;
        ++leafCount;
        return true;
    }

    // internal node의 child pair가 2번 이후 짝수 index에서 시작하는지 확인함.
    const std::int32_t childPair = GetChildPair( node );

    if( childPair < 2 ||
        ( childPair & 1 ) != 0 ||
        static_cast<std::size_t>( childPair + 1 ) >= nodes_.size() )
    {
        return false;
    }

    // Rebuild가 보장한 DFS 순서에서는 parent가 child pair보다 항상 앞에 있어야 함.
    if( dfsOrdered_ && nodeIndex >= childPair )
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
    const aabb2 combined = Union( child1.aabb, child2.aabb );

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

    height = 1 + std::max( height1, height2 );

    return node.height == height;
}

} // namespace zonai
