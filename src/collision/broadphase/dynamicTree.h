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
    // 아래에 있는 모든 leaf를 감싸는 AABB.
    aabb2 aabb{};

    // bit 31 : leaf flag
    // bit 30 : moved flag
    // bit 0~29 : internal node면 child pair index,
    //            leaf node면 proxy id
    std::uint32_t flagIndex = 0;

    union
    {
        // internal node 높이, leaf는 0.
        std::int32_t height = 0;

        // leaf가 가리키는 shape index.
        std::int32_t shapeIndex;
    };
};

struct TreeProxy
{
    std::uint64_t userData = 0;

    // 이 proxy의 leaf가 들어있는 node index.
    std::int32_t node = -1;

    // free-list의 다음 빈 proxy index.
    std::int32_t next = -1;
};

class DynamicTree
{
public:
    DynamicTree();

public:
    std::int32_t CreateProxy( const aabb2& aabb, std::int32_t shapeIndex );
    
    void DestroyProxy( std::int32_t proxyId );

    void MoveProxy( std::int32_t proxyId, const aabb2& aabb );

    std::size_t GetProxyCount() const;

    std::int32_t GetHeight() const;

    float GetAreaRatio() const;

    bool Validate() const;

    // proxy가 가지고 있는 AABB를 가져옴.
    const aabb2& GetProxyAABB( std::int32_t proxyId ) const;

    // AABB가 겹치는 proxy를 찾아서 callback으로 넘김.
    // callback이 false면 바로 끝냄.
    template <typename Callback>
    void Query( const aabb2& aabb, Callback&& callback ) const
    {
        if( proxyCount_ == 0 )
        {
            return;
        }

        // 재귀 대신 고정 크기 stack으로 tree를 돎.
        std::array<std::int32_t, TREE_STACK_SIZE> stack{};
        std::size_t stackCount = 0;

        // root가 internal이면 root의 child pair부터 시작함.
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

                // AABB가 안 겹치면 그 아래는 볼 필요 없으니 넘김.
                if( IsEmptyNode( node ) ||
                    !Overlaps( aabb, node.aabb ) )
                {
                    continue;
                }

                // leaf를 찾았으면 proxy id를 callback에 넘김.
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

                // internal node면 child pair를 stack에 넣고 계속 탐색함.
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
    // node 상태 체크
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

    bool ValidateSubtree( std::int32_t nodeIndex, std::int32_t& height, std::size_t& leafCount ) const;

    std::vector<TreeNode> nodes_{};
    std::vector<std::int32_t> parents_{};
    std::vector<TreeProxy> proxies_{};

    std::int32_t proxyFreeList_ = NULL_INDEX;
    std::int32_t pairFreeList_ = NULL_INDEX;
    std::size_t proxyCount_ = 0;
};

} // namespace zonai
