#pragma once

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "collision/aabb2.h"

namespace zonai
{

struct TreeNode
{
    // 이 노드 아래의 모든 leaf를 감싸는 aabb.
    aabb2 aabb{};

    // bit 31 : leaf flag
    // bit 30 : moved flag
    // bit 0~29 : internal node면 child pair index,
    //            leaf node면 proxy id
    std::uint32_t flagIndex = 0;

    union
    {
        // internal node의 높이. leaf의 높이는 0이다.
        std::int32_t height = 0;

        // leaf node가 가리키는 shape index.
        std::int32_t shapeIndex;
    };
};

struct TreeProxy
{
    std::uint64_t userData = 0;

    // 이 proxy의 leaf가 현재 저장된 node index.
    std::int32_t node = -1;

    // free-list에서 다음 빈 proxy index.
    std::int32_t next = -1;
};

class DynamicTree
{
public:
    DynamicTree();

public:
	// aabb와 shapeIndex를 가지는 proxy를 생성하고, proxy id를 반환한다.
    std::int32_t CreateProxy( const aabb2& aabb, std::int32_t shapeIndex );
    
	// proxy id에 해당하는 proxy를 제거한다.
    void DestroyProxy( std::int32_t proxyId );

    // proxy id는 유지한 채 새 aabb 위치로 leaf를 다시 삽입한다.
    void MoveProxy( std::int32_t proxyId, const aabb2& aabb );

    std::size_t GetProxyCount() const;

    std::int32_t GetHeight() const;

    float GetAreaRatio() const;

    bool Validate() const;

	// proxy id에 해당하는 proxy의 aabb를 반환한다.
    const aabb2& GetProxyAABB( std::int32_t proxyId ) const;

    // 주어진 aabb와 겹치는 모든 proxy를 callback으로 전달한다.
    // callback이 false를 반환하면 query를 즉시 종료한다.
    template <typename Callback>
    void Query( const aabb2& aabb, Callback&& callback ) const
    {
        if( proxyCount_ == 0 )
        {
            return;
        }

        std::array<std::int32_t, TREE_STACK_SIZE> stack{};
        std::size_t stackCount = 0;

        stack[stackCount++] =
            IsLeaf( nodes_[ROOT_NODE] ) ?
                ROOT_NODE :
                GetChildPair( nodes_[ROOT_NODE] );

        while( stackCount > 0 )
        {
            const std::int32_t pair =
                stack[--stackCount];

            for( std::int32_t i = 0; i < 2; ++i )
            {
                const std::int32_t nodeIndex = pair + i;
                const TreeNode& node = nodes_[nodeIndex];

                if( IsEmptyNode( node ) ||
                    !Overlaps( aabb, node.aabb ) )
                {
                    continue;
                }

                if( IsLeaf( node ) )
                {
                    const std::int32_t proxyId =
                        GetProxyId( node );

                    if( !callback( proxyId ) )
                    {
                        return;
                    }

                    continue;
                }

                assert( stackCount < TREE_STACK_SIZE );
                stack[stackCount++] =
                    GetChildPair( node );
            }
        }
    }

private:
    static constexpr std::uint32_t TREE_MOVED_NODE = 1u << 30;
    static constexpr std::uint32_t TREE_LEAF_NODE = 1u << 31;
    static constexpr std::uint32_t TREE_NODE_INDEX_MASK = ~( TREE_MOVED_NODE | TREE_LEAF_NODE );
    static constexpr std::uint32_t TREE_EMPTY_NODE = TREE_NODE_INDEX_MASK | TREE_LEAF_NODE;

    static constexpr std::int32_t ROOT_NODE = 0;
    static constexpr std::int32_t NULL_INDEX = -1;
    static constexpr std::size_t INITIAL_PROXY_CAPACITY = 16;
    static constexpr std::size_t TREE_STACK_SIZE = 512;

private:
    /*
	* private로 함수 노출 방지
    * 객체 사용 없이 static으로 정의
    */
    static bool IsLeaf( const TreeNode& node );
    static bool IsEmptyNode( const TreeNode& node );
    static std::int32_t GetChildPair( const TreeNode& node );
    static std::int32_t GetProxyId( const TreeNode& node );
    static std::int32_t GetNodeHeight( const TreeNode& node );

    static TreeNode MakeEmptyNode();
    static TreeNode MakeLeafNode(
        const aabb2& aabb,
        std::int32_t proxyId,
        std::int32_t shapeIndex );

private:
    TreeNode MakeInternalNode( std::int32_t childPair ) const;

    std::int32_t AllocateProxy();
    void FreeProxy( std::int32_t proxyId );

    std::int32_t AllocateSiblingPair();
    void FreeSiblingPair( std::int32_t pair );

    std::int32_t FindBestSibling( const aabb2& aabb ) const;
    void LinkChildren( std::int32_t nodeIndex );
    void SwapNodes( std::int32_t downIndex, std::int32_t upIndex );
    void RotateNode( std::int32_t nodeIndex );

    void InsertLeaf( const TreeNode& leaf, bool shouldRotate );
    void RemoveLeaf( std::int32_t leafIndex );
    void RefitAncestors( std::int32_t nodeIndex, bool shouldRotate );

    bool ValidateSubtree(
        std::int32_t nodeIndex,
        std::int32_t& height,
        std::size_t& leafCount ) const;

    std::vector<TreeNode> nodes_{};
    std::vector<std::int32_t> parents_{};
    std::vector<TreeProxy> proxies_{};

    std::int32_t proxyFreeList_ = NULL_INDEX;
    std::int32_t pairFreeList_ = NULL_INDEX;
    std::size_t proxyCount_ = 0;
};

} // namespace zonai
