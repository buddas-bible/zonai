#include "dynamics/world.h"

#include <cassert>
#include <cmath>
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


void World::SetBodyTransform(
    std::int32_t bodyId,
    transform2 transform )
{
    assert( bodyId >= 0 );
    assert( static_cast<std::size_t>( bodyId ) < bodies_.size() );

    // transform이 NaN / infinity를 포함하면 tree AABB까지 오염되므로 입구에서 차단함.
    assert( std::isfinite( transform.position.x ) );
    assert( std::isfinite( transform.position.y ) );
    assert( std::isfinite( transform.rotation.c ) );
    assert( std::isfinite( transform.rotation.s ) );

    Body& body = bodies_[bodyId];
    body.transform = transform;

    std::int32_t shapeId = body.headShapeId;
    std::int32_t visitedCount = 0;

    while( shapeId != Body::NULL_INDEX )
    {
        assert( shapeId >= 0 );
        assert( static_cast<std::size_t>( shapeId ) < shapes_.size() );
        assert( visitedCount < body.shapeCount );

        Shape& shape = shapes_[shapeId];

        assert( shape.bodyId == bodyId );

        const aabb2 worldAABB =
            ComputeShapeAABB( shape.geometry, body.transform );

        // disabled Body 개념은 아직 없지만 Box2D와 같은 수명 규칙을 위해
        // proxy가 실제로 존재하는 Shape만 BroadPhase에서 이동시킴.
        if( shape.proxyKey != Shape::NULL_INDEX )
        {
            broadPhase_.MoveProxy( shape.proxyKey, worldAABB );
        }

        shapeId = shape.nextShapeId;
        ++visitedCount;
    }

    // linked list와 Body의 shapeCount가 서로 일치해야 함.
    assert( visitedCount == body.shapeCount );
}

const Body& World::GetBody( std::int32_t bodyId ) const
{
    assert( bodyId >= 0 );
    assert( static_cast<std::size_t>( bodyId ) < bodies_.size() );

    return bodies_[bodyId];
}


const Shape& World::GetShape( std::int32_t shapeId ) const
{
    assert( shapeId >= 0 );
    assert( static_cast<std::size_t>( shapeId ) < shapes_.size() );

    return shapes_[shapeId];
}

} // namespace zonai
