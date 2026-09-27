#include <cassert>
#include <cmath>
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

    // Shape geometry는 Body local space에 저장되고 proxy AABB는 world space로 계산됨.
    const std::int32_t shapeBody =
        world.CreateBody(
            BodyType::Dynamic,
            {
                { 10.0f, 5.0f },
                { 0.0f, 1.0f }
            }
        );

    const std::int32_t circleShape =
        world.CreateShape(
            shapeBody,
            circle2{ { 2.0f, 0.0f }, 1.0f }
        );

    assert( world.GetShapeCount() == 1 );

    const Shape& circle = world.GetShape( circleShape );
    const Body& owner = world.GetBody( shapeBody );

    assert( circle.bodyId == shapeBody );
    assert( circle.proxyKey != Shape::NULL_INDEX );
    assert( GetProxyType( circle.proxyKey ) == BodyType::Dynamic );
    assert( owner.headShapeId == circleShape );
    assert( owner.shapeCount == 1 );

    // local center (2, 0)을 90도 회전 후 (10, 5)만큼 이동하면 world center는 (10, 7).
    const aabb2& circleAABB =
        world.GetBroadPhase()
            .GetTree( BodyType::Dynamic )
            .GetProxyAABB( GetProxyId( circle.proxyKey ) );

    constexpr float epsilon = 1e-5f;

    assert( std::fabs( circleAABB.min.x - 9.0f ) < epsilon );
    assert( std::fabs( circleAABB.min.y - 6.0f ) < epsilon );
    assert( std::fabs( circleAABB.max.x - 11.0f ) < epsilon );
    assert( std::fabs( circleAABB.max.y - 8.0f ) < epsilon );

    const std::int32_t segmentShape =
        world.CreateShape(
            shapeBody,
            segment2{ { 0.0f, 0.0f }, { 1.0f, 0.0f } }
        );

    // 새 Shape는 Body shape list의 head에 삽입됨.
    assert( world.GetBody( shapeBody ).headShapeId == segmentShape );
    assert( world.GetBody( shapeBody ).shapeCount == 2 );
    assert( world.GetShape( segmentShape ).nextShapeId == circleShape );
    assert( world.GetShape( circleShape ).prevShapeId == segmentShape );

    // Body 이동은 연결된 모든 Shape의 BroadPhase proxy를 함께 갱신해야 함.
    world.SetBodyTransform(
        shapeBody,
        {
            { 20.0f, -3.0f },
            { 1.0f, 0.0f }
        }
    );

    const Body& movedBody = world.GetBody( shapeBody );
    assert( movedBody.transform.position.x == 20.0f );
    assert( movedBody.transform.position.y == -3.0f );

    const Shape& movedCircle = world.GetShape( circleShape );
    const aabb2& movedCircleAABB =
        world.GetBroadPhase()
            .GetTree( BodyType::Dynamic )
            .GetProxyAABB( GetProxyId( movedCircle.proxyKey ) );

    assert( std::fabs( movedCircleAABB.min.x - 21.0f ) < epsilon );
    assert( std::fabs( movedCircleAABB.min.y + 4.0f ) < epsilon );
    assert( std::fabs( movedCircleAABB.max.x - 23.0f ) < epsilon );
    assert( std::fabs( movedCircleAABB.max.y + 2.0f ) < epsilon );

    const Shape& movedSegment = world.GetShape( segmentShape );
    const aabb2& movedSegmentAABB =
        world.GetBroadPhase()
            .GetTree( BodyType::Dynamic )
            .GetProxyAABB( GetProxyId( movedSegment.proxyKey ) );

    assert( std::fabs( movedSegmentAABB.min.x - 20.0f ) < epsilon );
    assert( std::fabs( movedSegmentAABB.min.y + 3.0f ) < epsilon );
    assert( std::fabs( movedSegmentAABB.max.x - 21.0f ) < epsilon );
    assert( std::fabs( movedSegmentAABB.max.y + 3.0f ) < epsilon );

    assert( world.GetBroadPhase()
                .GetTree( BodyType::Dynamic )
                .Validate() );

    return 0;
}
