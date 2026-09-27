#include "dynamics/world.h"

#include <cassert>
#include <limits>

namespace zonai
{

std::int32_t World::CreateBody(
    BodyType type,
    transform2 transform )
{
    assert(
        bodies_.size() <
        static_cast<std::size_t>( std::numeric_limits<std::int32_t>::max() )
    );

    const std::int32_t bodyId =
        static_cast<std::int32_t>( bodies_.size() );

    Body body{};
    body.type = type;
    body.transform = transform;

    bodies_.push_back( body );

    return bodyId;
}

Body& World::GetBody( std::int32_t bodyId )
{
    assert( bodyId >= 0 );
    assert( static_cast<std::size_t>( bodyId ) < bodies_.size() );

    return bodies_[bodyId];
}

const Body& World::GetBody( std::int32_t bodyId ) const
{
    assert( bodyId >= 0 );
    assert( static_cast<std::size_t>( bodyId ) < bodies_.size() );

    return bodies_[bodyId];
}

} // namespace zonai
