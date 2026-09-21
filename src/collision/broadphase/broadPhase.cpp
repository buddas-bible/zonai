#include "collision/broadphase/broadPhase.h"

#include <cassert>

namespace zonai
{

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
