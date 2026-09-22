#include "collision/broadphase/broadPhase.h"

#include <cassert>

namespace zonai
{

ProxyKey BroadPhase::CreateProxy(
    BodyType type,
    const aabb2& aabb,
    std::int32_t shapeIndex,
    bool forcePairCreation )
{
    // static은 강제 요청이 있을 때만 moved 처리하고 나머지 type은 새 pair 생성을 위해 moved 처리함.
    const bool markMoved = type != BodyType::Static || forcePairCreation;

    DynamicTree& tree = GetTree( type );
    const std::int32_t proxyId = tree.CreateProxy( aabb, shapeIndex, markMoved );

    return MakeProxyKey( proxyId, type );
}

void BroadPhase::DestroyProxy( ProxyKey proxyKey )
{
    const BodyType type = GetProxyType( proxyKey );
    const std::int32_t proxyId = GetProxyId( proxyKey );

    GetTree( type ).DestroyProxy( proxyId );
}

DynamicTree& BroadPhase::GetTree( BodyType type )
{
    const std::size_t index = static_cast<std::size_t>( type );

    assert( index < trees_.size() );

    return trees_[index];
}

const DynamicTree& BroadPhase::GetTree( BodyType type ) const
{
    const std::size_t index = static_cast<std::size_t>( type );

    assert( index < trees_.size() );

    return trees_[index];
}

} // namespace zonai
