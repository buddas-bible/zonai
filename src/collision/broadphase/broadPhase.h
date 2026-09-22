#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "collision/broadphase/dynamicTree.h"
#include "dynamics/bodyType.h"

namespace zonai
{

using ProxyKey = std::int32_t;

constexpr ProxyKey MakeProxyKey( std::int32_t proxyId, BodyType type )
{
    constexpr std::uint32_t TYPE_BITS = 2;

    const std::uint32_t id = static_cast<std::uint32_t>( proxyId );
    const std::uint32_t bodyType = static_cast<std::uint32_t>( type );

    return static_cast<ProxyKey>( ( id << TYPE_BITS ) | bodyType );
}

constexpr std::int32_t GetProxyId( ProxyKey key )
{
    constexpr std::uint32_t TYPE_BITS = 2;

    return static_cast<std::int32_t>( static_cast<std::uint32_t>( key ) >> TYPE_BITS );
}

constexpr BodyType GetProxyType( ProxyKey key )
{
    constexpr std::uint32_t TYPE_MASK = 0x3u;

    return static_cast<BodyType>( static_cast<std::uint32_t>( key ) & TYPE_MASK );
}

class BroadPhase
{
public:
    ProxyKey CreateProxy( BodyType type, const aabb2& aabb, std::int32_t shapeIndex, bool forcePairCreation = false );
    void DestroyProxy( ProxyKey proxyKey );
    void MoveProxy( ProxyKey proxyKey, const aabb2& aabb );

    DynamicTree& GetTree( BodyType type );
    const DynamicTree& GetTree( BodyType type ) const;

private:
    static constexpr std::size_t BODY_TYPE_COUNT = static_cast<std::size_t>( BodyType::Count );

    static std::size_t GatherMovedSiblings( const DynamicTree& tree, std::span<std::int32_t> pairIndices );

    // body type마다 독립된 DynamicTree를 사용함.
    std::array<DynamicTree, BODY_TYPE_COUNT> trees_{};
};

} // namespace zonai
