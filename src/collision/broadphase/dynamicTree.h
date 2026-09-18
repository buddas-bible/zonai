#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "collision/aabb2.h"

namespace zonai
{

struct TreeNode
{
    // 이 노드 아래의 모든 leaf를 감싸는 경계 상자.
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

class DynamicTree
{
public:
    DynamicTree();

    int CreateProxy( const aabb2& aabb, int shapeIndex );

    std::size_t GetProxyCount() const;
    int GetHeight() const;
    float GetAreaRatio() const;
    const aabb2& GetProxyAABB( int proxyId ) const;

private:
    struct TreeProxy
    {
        // 이 proxy의 leaf가 현재 저장된 node index.
        std::int32_t node = -1;

        // free-list에서 다음 빈 proxy index.
        std::int32_t next = -1;
    };

    static constexpr std::uint32_t TREE_MOVED_NODE = 1u << 30;
    static constexpr std::uint32_t TREE_LEAF_NODE = 1u << 31;
    static constexpr std::uint32_t TREE_NODE_INDEX_MASK =
        ~( TREE_MOVED_NODE | TREE_LEAF_NODE );
    static constexpr std::uint32_t TREE_EMPTY_NODE =
        TREE_NODE_INDEX_MASK | TREE_LEAF_NODE;

    static constexpr std::int32_t ROOT_NODE = 0;
    static constexpr std::int32_t NULL_INDEX = -1;
    static constexpr std::size_t INITIAL_PROXY_CAPACITY = 16;

    static bool IsLeaf( const TreeNode& node );
    static bool IsEmptyNode( const TreeNode& node );
    static std::int32_t GetChildPair( const TreeNode& node );
    static std::int32_t GetProxyId( const TreeNode& node );
    static int GetNodeHeight( const TreeNode& node );

    static TreeNode MakeEmptyNode();
    static TreeNode MakeLeafNode(
        const aabb2& aabb,
        int proxyId,
        int shapeIndex );

    TreeNode MakeInternalNode( std::int32_t childPair ) const;

    int AllocateProxy();
    std::int32_t AllocateSiblingPair();

    std::int32_t FindBestSibling( const aabb2& aabb ) const;
    void LinkChildren( std::int32_t nodeIndex );
    void InsertLeaf( const TreeNode& leaf );
    void RefitAncestors( std::int32_t nodeIndex );

    std::vector<TreeNode> nodes_{};
    std::vector<std::int32_t> parents_{};
    std::vector<TreeProxy> proxies_{};

    std::int32_t proxyFreeList_ = NULL_INDEX;
    std::size_t proxyCount_ = 0;
};

} // namespace zonai
