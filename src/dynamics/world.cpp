#include "dynamics/world.h"

#include <cassert>
#include <limits>
#include <utility>

#include "dynamics/bodyShape.h"

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


std::int32_t World::CreateShape(
    std::int32_t bodyId,
    ShapeGeometry geometry,
    Filter filter )
{
    assert( bodyId >= 0 );
    assert( static_cast<std::size_t>( bodyId ) < bodies_.size() );
    assert( !std::holds_alternative<std::monostate>( geometry ) );
    assert(
        shapes_.size() <
        static_cast<std::size_t>( std::numeric_limits<std::int32_t>::max() )
    );

    const std::int32_t shapeId =
        static_cast<std::int32_t>( shapes_.size() );

    Shape shape{};
    shape.geometry = std::move( geometry );
    shape.filter = filter;

    shapes_.push_back( std::move( shape ) );

    Body& body = bodies_[bodyId];
    Shape& storedShape = shapes_[shapeId];

    const aabb2 worldAABB =
        ComputeShapeAABB( storedShape.geometry, body.transform );

    // Box2D의 기본 Shape 생성처럼 static Shape도 즉시 pair 탐색 대상이 되게 함.
    storedShape.proxyKey =
        broadPhase_.CreateProxy(
            body.type,
            worldAABB,
            shapeId,
            true
        );

    LinkShape( body, bodyId, shapes_, shapeId );

    return shapeId;
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


Shape& World::GetShape( std::int32_t shapeId )
{
    assert( shapeId >= 0 );
    assert( static_cast<std::size_t>( shapeId ) < shapes_.size() );

    return shapes_[shapeId];
}

const Shape& World::GetShape( std::int32_t shapeId ) const
{
    assert( shapeId >= 0 );
    assert( static_cast<std::size_t>( shapeId ) < shapes_.size() );

    return shapes_[shapeId];
}

} // namespace zonai
