#pragma once

#include <array>
#include <cassert>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "collision/aabb2.h"
#include "memory/alignedAllocator.h"

namespace zonai
{

struct treeNode
{
    // 이 node 아래 모든 leaf를 감싸는 AABB.
    aabb2 aabb{};

    // Box2D와 같은 32-byte node layout을 유지함.
    // 향후 3D에서는 z bounds 공간으로 재사용할 수 있음.
    std::uint64_t padding = 0;

    // bit 31 : leaf flag
    // bit 30 : moved flag
    // bit 0~29 : internal node면 child pair index,
    //            leaf node면 proxy id
    std::uint32_t flagIndex = 0;

    union
    {
        // internal node의 높이. leaf는 0.
        std::int32_t height = 0;

        // leaf가 가리키는 shape index.
        std::int32_t shapeIndex;
    };
};

struct treeProxy
{
    std::uint64_t userData = 0;

    // stable proxy가 현재 가리키는 leaf node index.
    std::int32_t node = -1;

    // free-list에서 다음 빈 proxy index.
    std::int32_t next = -1;
};

static_assert( sizeof( treeNode ) == 32 );
static_assert( sizeof( treeProxy ) == 16 );

using treeNodeStorage =
    std::vector<treeNode, alignedAllocator<treeNode, 64>>;

// Debug Draw가 tree 내부 저장구조를 직접 노출받지 않고 node 상태를 읽기 위한 view.
struct treeNodeDebugInfo
{
    aabb2 aabb{};

    std::int32_t nodeIndex = -1;
    std::int32_t parentIndex = -1;

    // internal node면 첫 child index, leaf면 -1.
    std::int32_t childPair = -1;

    // leaf에서만 유효함.
    std::int32_t proxyId = -1;
    std::int32_t shapeIndex = -1;

    // leaf는 0, internal node는 subtree height.
    std::int32_t height = 0;

    bool isLeaf = false;
    bool isMoved = false;
    bool isRoot = false;
};

template <typename Callback>
concept treeQueryCallback =
    requires( Callback& callback, std::int32_t proxyId )
    {
        { callback( proxyId ) } -> std::convertible_to<bool>;
    };

class dynamicTree
{
    friend class broadPhase;

public:
    dynamicTree();

    // stable proxy id를 할당하고 새 leaf를 tree에 삽입함.
    std::int32_t CreateProxy( const aabb2& aabb, std::int32_t shapeIndex, bool markMoved = false );

    void DestroyProxy( std::int32_t proxyId );

    // leaf를 제거한 뒤 새 AABB 위치에 같은 stable proxy id로 다시 삽입함.
    void MoveProxy( std::int32_t proxyId, const aabb2& aabb, bool markMoved = false );

    // leaf bounds만 새 AABB로 바꾸고 topology는 유지함.
    // single-thread 경로라 ancestor bounds를 즉시 refit하고 다음 partial rebuild 대상으로 표시함.
    void UpdateProxy( std::int32_t proxyId, const aabb2& aabb );

    // 기존 proxy bounds를 포함하는 더 큰 AABB로 leaf를 넓히고
    // topology는 유지한 채 ancestor bounds와 moved flag만 갱신함.
    void EnlargeProxy( std::int32_t proxyId, const aabb2& aabb );

    // AABB / topology는 그대로 두고 이 proxy가 참여하는 pair만 다시 탐색하게 함.
    void MarkProxyMoved( std::int32_t proxyId );

    bool HasMoved() const;

    // moved branch가 있거나 DFS 배열 순서가 깨졌으면 rebuild가 필요함.
    bool NeedsRebuild() const;

    // stale branch만 다시 만들고 untouched subtree는 유지함.
    // 실제로 정렬한 build leaf 개수를 반환함.
    std::size_t Rebuild( bool fullBuild = false );

    void ClearMoved();

    std::size_t GetProxyCount() const;

    std::int32_t GetHeight() const;

    float GetAreaRatio() const;

    bool Validate() const;

    // proxy가 가리키는 leaf의 AABB를 반환함.
    const aabb2& GetProxyAABB( std::int32_t proxyId ) const;

    // stable proxy가 가리키는 shape index를 반환함.
    std::int32_t GetProxyShapeIndex( std::int32_t proxyId ) const;

    // Debug / tooling에서 live node를 read-only로 순회함.
    // 내부 vector와 flag bit layout은 외부에 노출하지 않음.
    template <typename Callback>
    void VisitNodes( Callback&& callback ) const
    {
        for( std::size_t i = 0; i < nodes_.size(); ++i )
        {
            const treeNode& node = nodes_[i];

            if( IsEmptyNode( node ) )
            {
                continue;
            }

            const bool isLeaf = IsLeaf( node );

            treeNodeDebugInfo info{};
            info.aabb = node.aabb;
            info.nodeIndex = static_cast<std::int32_t>( i );
            info.parentIndex = parents_[i];
            info.childPair = isLeaf ? NULL_INDEX : GetChildPair( node );
            info.proxyId = isLeaf ? GetProxyId( node ) : NULL_INDEX;
            info.shapeIndex = isLeaf ? node.shapeIndex : NULL_INDEX;
            info.height = GetNodeHeight( node );
            info.isLeaf = isLeaf;
            info.isMoved = ( node.flagIndex & TREE_MOVED_NODE ) != 0;
            info.isRoot = static_cast<std::int32_t>( i ) == ROOT_NODE;

            callback( info );
        }
    }

    // AABB가 겹치는 proxy를 찾아 callback으로 전달함.
    // callback이 false를 반환하면 즉시 순회를 끝냄.
    // Box2D처럼 callback 중 tree를 변경하지 않음. 조회 결과는 broad-phase AABB 후보임.
    template <treeQueryCallback Callback>
    void Query( const aabb2& aabb, Callback&& callback ) const
    {
        if( proxyCount_ == 0 )
        {
            return;
        }

        // Query 중 heap allocation을 피하려고 고정 크기 stack으로 순회함.
        std::array<std::int32_t, TREE_STACK_SIZE> stack{};
        std::size_t stackCount = 0;

        // root가 internal이면 첫 child pair부터, leaf면 root부터 시작함.
        stack[stackCount++] =
            IsLeaf( nodes_[ROOT_NODE] ) ?
                ROOT_NODE : GetChildPair( nodes_[ROOT_NODE] );

        while( stackCount > 0 )
        {
            const std::int32_t pair = stack[--stackCount];

            for( std::int32_t i = 0; i < 2; ++i )
            {
                const std::int32_t nodeIndex = pair + i;
                const treeNode& node = nodes_[nodeIndex];

                // AABB가 겹치지 않으면 subtree 전체를 가지치기함.
                if( IsEmptyNode( node ) || !Overlaps( aabb, node.aabb ) )
                {
                    continue;
                }

                // leaf를 찾으면 stable proxy id를 callback에 넘김.
                if( IsLeaf( node ) )
                {
                    const std::int32_t proxyId = GetProxyId( node );

                    const bool proceed = callback( proxyId );
                    if( proceed == false )
                    {
                        return;
                    }

                    continue;
                }

                // Box2D처럼 release에서도 고정 stack 바깥에 쓰지 않도록 먼저 범위를 확인함.
                if( stackCount < stack.size() )
                {
                    stack[stackCount++] = GetChildPair( node );
                }
                else
                {
                    assert( stackCount < stack.size() );
                }
            }
        }
    }

private:
    struct rebuildItem
    {
        std::int32_t nodeIndex = 0;
        std::int32_t pair = 0;
        std::int32_t childCount = 0;
        std::size_t startIndex = 0;
        std::size_t splitIndex = 0;
        std::size_t endIndex = 0;
    };

    struct copyItem
    {
        std::int32_t oldPair = 0;
        std::int32_t newIndex = 0;
    };

    static constexpr std::uint32_t TREE_MOVED_NODE = 1u << 30;
    static constexpr std::uint32_t TREE_LEAF_NODE = 1u << 31;
    static constexpr std::uint32_t TREE_NODE_INDEX_MASK = ~( TREE_MOVED_NODE | TREE_LEAF_NODE );
    static constexpr std::uint32_t TREE_EMPTY_NODE = TREE_NODE_INDEX_MASK | TREE_LEAF_NODE;

    static constexpr std::int32_t ROOT_NODE = 0;
    static constexpr std::int32_t NULL_INDEX = -1;

    static_assert( ROOT_NODE == 0 );
    static constexpr std::size_t INITIAL_PROXY_CAPACITY = 16;
    static constexpr std::size_t TREE_STACK_SIZE = 512;

    // Box2D의 single-precision broad-phase 범위 보호와 같은 상한.
    static constexpr float MAX_TREE_AABB_EXTENT = 1.0e5f;

    // flagIndex에서 node 상태와 저장된 index를 읽는 helper.
    static bool IsLeaf( const treeNode& node );
    static bool IsEmptyNode( const treeNode& node );
    static std::int32_t GetChildPair( const treeNode& node );
    static std::int32_t GetProxyId( const treeNode& node );
    static std::int32_t GetNodeHeight( const treeNode& node );
    static void SetChildPair( treeNode& node, std::int32_t pair );

    // sweep refit을 위해 internal node가 자기 child pair보다 앞 index에 있는지 확인함.
    bool IsNodeOrdered( std::int32_t nodeIndex ) const;

    static treeNode MakeEmptyNode();
    static treeNode MakeLeafNode( const aabb2& aabb, std::int32_t proxyId, std::int32_t shapeIndex, bool moved );

    static treeNode MakeInternalNodeFrom(
        const treeNodeStorage& nodes,
        std::int32_t childPair );

    treeNode MakeInternalNode( std::int32_t childPair ) const;

    std::size_t PartitionRebuildLeaves( std::size_t startIndex, std::size_t count );
    std::int32_t BumpRebuildPair( std::int32_t parent, std::int32_t& nodeEnd );
    void CopySubtree( treeNode node, std::int32_t newIndex, std::int32_t& nodeEnd );
    void PlaceRebuildLeaf( const treeNode& node, std::int32_t newIndex, std::int32_t& nodeEnd );
    void BuildRebuildTree( std::size_t leafCount );

    // proxy 할당 및 해제
    std::int32_t AllocateProxy();
    void FreeProxy( std::int32_t proxyId );

    // 형제 노드 pair 할당 및 해제
    std::int32_t AllocateSiblingPair();
    void FreeSiblingPair( std::int32_t pair );

    // 새 leaf와 묶였을 때 비용이 가장 작은 형제 노드 index를 반환함.
    std::int32_t FindBestSibling( const aabb2& aabb ) const;
    void LinkChildren( std::int32_t nodeIndex );
    void SwapNodes( std::int32_t downIndex, std::int32_t upIndex );
    void RotateNode( std::int32_t nodeIndex );

    void InsertLeaf( const treeNode& leaf, bool shouldRotate );
    void RemoveLeaf( std::int32_t leafIndex );
    void RefitAncestors( std::int32_t nodeIndex, bool shouldRotate );

    bool ValidateSubtree( std::int32_t nodeIndex, std::int32_t& height, std::size_t& leafCount ) const;

    treeNodeStorage nodes_{};
    std::vector<std::int32_t> parents_{};
    std::vector<treeProxy> proxies_{};

    // Rebuild에서 재사용하는 scratch buffer. 매 frame 작은 allocation이 생기지 않게 유지함.
    treeNodeStorage rebuildNodes_{};
    std::vector<std::int32_t> rebuildLeafIndices_{};
    treeNodeStorage rebuildLeafNodes_{};
    std::vector<vec2> rebuildLeafCenters_{};

    // free proxy 연결 리스트의 head index.
    std::int32_t proxyFreeList_ = NULL_INDEX;

    // free pair 연결 리스트의 head index. free pair의 parents_[pair]는 next index로 사용함.
    std::int32_t pairFreeList_ = NULL_INDEX;

    std::size_t proxyCount_ = 0;

    // Rebuild 후에는 parent가 child보다 앞에 놓이는 DFS 순서를 유지함.
    // 삽입이나 rotation으로 이 순서가 깨지면 다음 rebuild까지 false로 유지함.
    bool dfsOrdered_ = true;
};

} // namespace zonai
