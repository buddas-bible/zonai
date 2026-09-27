#include <cassert>
#include <cstdint>

#include "dynamics/world.h"

using namespace zonai;

int main()
{
    World world{};

    assert( world.GetBodyCount() == 0 );

    const std::int32_t staticBody =
        world.CreateBody();

    const std::int32_t dynamicBody =
        world.CreateBody(
            BodyType::Dynamic,
            {
                { 3.0f, -2.0f },
                rot2::FromRadians( 0.5f )
            }
        );

    const std::int32_t kinematicBody =
        world.CreateBody(
            BodyType::Kinematic,
            {
                { -4.0f, 1.5f },
                rot2::FromRadians( -0.25f )
            }
        );

    // 아직 Body id pool이 없으므로 생성 순서의 vector index가 bodyId가 됨.
    assert( staticBody == 0 );
    assert( dynamicBody == 1 );
    assert( kinematicBody == 2 );
    assert( world.GetBodyCount() == 3 );

    {
        const Body& body = world.GetBody( staticBody );

        assert( body.type == BodyType::Static );
        assert( body.headShapeId == Body::NULL_INDEX );
        assert( body.shapeCount == 0 );
    }

    {
        Body& body = world.GetBody( dynamicBody );

        assert( body.type == BodyType::Dynamic );
        assert( body.transform.position.x == 3.0f );
        assert( body.transform.position.y == -2.0f );
        assert( body.headShapeId == Body::NULL_INDEX );
        assert( body.shapeCount == 0 );

        // mutable GetBody가 실제 World storage를 반환하는지도 확인함.
        body.transform.position.x = 7.0f;
        assert( world.GetBody( dynamicBody ).transform.position.x == 7.0f );
    }

    {
        const World& constWorld = world;
        const Body& body = constWorld.GetBody( kinematicBody );

        assert( body.type == BodyType::Kinematic );
        assert( body.transform.position.x == -4.0f );
        assert( body.transform.position.y == 1.5f );
    }

    return 0;
}
