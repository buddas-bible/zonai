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
        const Body& body = world.GetBody( dynamicBody );

        assert( body.type == BodyType::Dynamic );
        assert( body.transform.position.x == 3.0f );
        assert( body.transform.position.y == -2.0f );
        assert( body.headShapeId == Body::NULL_INDEX );
        assert( body.shapeCount == 0 );
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

    // World collision pipeline: BroadPhase 후보를 실제 NarrowPhase 접촉까지 연결함.
    {
        World collisionWorld{};

        const std::int32_t groundBody =
            collisionWorld.CreateBody( BodyType::Static );

        const std::int32_t groundShape =
            collisionWorld.CreateShape(
                groundBody,
                MakeBox( { 1.0f, 1.0f } )
            );

        const std::int32_t circleBody =
            collisionWorld.CreateBody(
                BodyType::Dynamic,
                {
                    { 0.0f, 1.5f },
                    {}
                }
            );

        const std::int32_t circleShapeId =
            collisionWorld.CreateShape(
                circleBody,
                circle2{ {}, 0.5f }
            );

        int contactCount = 0;

        collisionWorld.UpdateCollisions(
            [&]( std::int32_t shapeIdA,
                 std::int32_t shapeIdB,
                 const localManifold2& manifold )
            {
                ++contactCount;

                assert( shapeIdA == groundShape );
                assert( shapeIdB == circleShapeId );
                assert( manifold.pointCount == 1 );
                assert( std::fabs( manifold.normal.x ) < epsilon );
                assert( std::fabs( manifold.normal.y - 1.0f ) < epsilon );
                assert( std::fabs( manifold.points[0].separation ) < epsilon );
            }
        );

        assert( contactCount == 1 );
        assert( collisionWorld.GetContactCount() == 1 );

        const contact2& contact =
            collisionWorld.GetContact( 0 );

        assert( contact.contactId == 0 );
        assert( contact.shapeIdA == groundShape );
        assert( contact.shapeIdB == circleShapeId );
        assert( contact.manifold.pointCount == 1 );

        const Body& ground = collisionWorld.GetBody( groundBody );
        const Body& circleOwner = collisionWorld.GetBody( circleBody );

        assert( ground.contactCount == 1 );
        assert( circleOwner.contactCount == 1 );

        // 같은 Contact를 Body A는 edge 0, Body B는 edge 1 key로 가리킴.
        assert( ground.headContactKey == MakeContactKey( 0, 0 ) );
        assert( circleOwner.headContactKey == MakeContactKey( 0, 1 ) );

        assert( contact.edges[0].bodyId == groundBody );
        assert( contact.edges[1].bodyId == circleBody );

        const ShapePairKey pairKey =
            MakeShapePairKey( groundShape, circleShapeId );

        assert( collisionWorld.GetBroadPhase().HasPair( pairKey ) );

        // BroadPhase에 새 moved proxy가 없어도 persistent Contact는 매 update 갱신됨.
        contactCount = 0;

        collisionWorld.UpdateCollisions(
            [&]( std::int32_t,
                 std::int32_t,
                 const localManifold2& manifold )
            {
                ++contactCount;
                assert( manifold.pointCount == 1 );
            }
        );

        assert( contactCount == 1 );
        assert( collisionWorld.GetContactCount() == 1 );

        // AABB가 분리되면 Contact와 pairSet entry를 함께 제거해야 함.
        collisionWorld.SetBodyTransform(
            circleBody,
            {
                { 0.0f, 5.0f },
                {}
            }
        );

        contactCount = 0;

        collisionWorld.UpdateCollisions(
            [&]( std::int32_t,
                 std::int32_t,
                 const localManifold2& )
            {
                ++contactCount;
            }
        );

        assert( contactCount == 0 );
        assert( collisionWorld.GetContactCount() == 0 );
        assert( !collisionWorld.GetBroadPhase().HasPair( pairKey ) );

        assert( collisionWorld.GetBody( groundBody ).headContactKey == Body::NULL_INDEX );
        assert( collisionWorld.GetBody( groundBody ).contactCount == 0 );
        assert( collisionWorld.GetBody( circleBody ).headContactKey == Body::NULL_INDEX );
        assert( collisionWorld.GetBody( circleBody ).contactCount == 0 );

        // 제거된 slot 0이 다음 Contact에서 stable id로 재사용되는지 확인함.
        collisionWorld.SetBodyTransform(
            circleBody,
            {
                { 0.0f, 1.5f },
                {}
            }
        );

        collisionWorld.UpdateCollisions(
            [&]( std::int32_t,
                 std::int32_t,
                 const localManifold2& )
            {
            }
        );

        assert( collisionWorld.GetContactCount() == 1 );
        assert( collisionWorld.GetContact( 0 ).contactId == 0 );
        assert( collisionWorld.GetBody( groundBody ).headContactKey == MakeContactKey( 0, 0 ) );
        assert( collisionWorld.GetBody( circleBody ).headContactKey == MakeContactKey( 0, 1 ) );
    }

    // AABB pair가 겹치면 실제 manifold가 비어 있어도 Contact 자체는 유지함.
    {
        World contactWorld{};

        const std::int32_t staticBodyId =
            contactWorld.CreateBody( BodyType::Static );

        const std::int32_t staticCircle =
            contactWorld.CreateShape(
                staticBodyId,
                circle2{ {}, 1.0f }
            );

        const std::int32_t dynamicBodyId =
            contactWorld.CreateBody(
                BodyType::Dynamic,
                {
                    { 1.5f, 1.5f },
                    {}
                }
            );

        const std::int32_t dynamicCircle =
            contactWorld.CreateShape(
                dynamicBodyId,
                circle2{ {}, 1.0f }
            );

        int touchingCount = 0;

        contactWorld.UpdateCollisions(
            [&]( std::int32_t,
                 std::int32_t,
                 const localManifold2& )
            {
                ++touchingCount;
            }
        );

        assert( touchingCount == 0 );
        assert( contactWorld.GetContactCount() == 1 );

        const contact2& contact = contactWorld.GetContact( 0 );
        assert( contact.manifold.pointCount == 0 );
        assert( contactWorld.GetBroadPhase().HasPair(
            MakeShapePairKey( staticCircle, dynamicCircle )
        ) );
    }

    // DestroyShape는 Contact / pairSet / proxy / Body shape list를 함께 정리함.
    {
        World destroyWorld{};

        const std::int32_t staticBodyId =
            destroyWorld.CreateBody( BodyType::Static );

        const std::int32_t groundShape =
            destroyWorld.CreateShape(
                staticBodyId,
                MakeBox( { 1.0f, 1.0f } )
            );

        const std::int32_t dynamicBodyId =
            destroyWorld.CreateBody(
                BodyType::Dynamic,
                {
                    { 0.0f, 1.5f },
                    {}
                }
            );

        const std::int32_t touchingShape =
            destroyWorld.CreateShape(
                dynamicBodyId,
                circle2{ {}, 0.5f }
            );

        const std::int32_t otherShape =
            destroyWorld.CreateShape(
                dynamicBodyId,
                circle2{ { 5.0f, 0.0f }, 0.5f }
            );

        assert( destroyWorld.GetShapeCount() == 3 );
        assert( destroyWorld.GetBody( dynamicBodyId ).shapeCount == 2 );
        assert( destroyWorld.GetBody( dynamicBodyId ).headShapeId == otherShape );

        destroyWorld.UpdateCollisions(
            []( std::int32_t,
                std::int32_t,
                const localManifold2& )
            {
            }
        );

        const ShapePairKey pairKey =
            MakeShapePairKey( groundShape, touchingShape );

        assert( destroyWorld.GetContactCount() == 1 );
        assert( destroyWorld.GetBroadPhase().HasPair( pairKey ) );

        // touchingShape는 head가 아닌 Shape이므로 중간/꼬리 unlink 경로도 함께 검증됨.
        destroyWorld.DestroyShape( touchingShape );

        assert( destroyWorld.GetShapeCount() == 2 );
        assert( destroyWorld.GetContactCount() == 0 );
        assert( !destroyWorld.GetBroadPhase().HasPair( pairKey ) );

        assert( destroyWorld.GetBody( staticBodyId ).contactCount == 0 );
        assert( destroyWorld.GetBody( dynamicBodyId ).contactCount == 0 );

        const Body& dynamicBody =
            destroyWorld.GetBody( dynamicBodyId );

        assert( dynamicBody.shapeCount == 1 );
        assert( dynamicBody.headShapeId == otherShape );
        assert( destroyWorld.GetShape( otherShape ).prevShapeId == Shape::NULL_INDEX );
        assert( destroyWorld.GetShape( otherShape ).nextShapeId == Shape::NULL_INDEX );

        assert( destroyWorld.GetBroadPhase()
                    .GetTree( BodyType::Dynamic )
                    .Validate() );

        // 삭제된 slot을 새 Shape가 재사용하되 Body list / proxy를 새 상태로 다시 구성해야 함.
        const std::int32_t reusedShape =
            destroyWorld.CreateShape(
                dynamicBodyId,
                circle2{ {}, 0.5f }
            );

        assert( reusedShape == touchingShape );
        assert( destroyWorld.GetShapeCount() == 3 );
        assert( destroyWorld.GetBody( dynamicBodyId ).shapeCount == 2 );
        assert( destroyWorld.GetBody( dynamicBodyId ).headShapeId == reusedShape );
        assert( destroyWorld.GetShape( reusedShape ).nextShapeId == otherShape );
        assert( destroyWorld.GetShape( otherShape ).prevShapeId == reusedShape );
        assert( destroyWorld.GetShape( reusedShape ).proxyKey != Shape::NULL_INDEX );

        destroyWorld.UpdateCollisions(
            []( std::int32_t,
                std::int32_t,
                const localManifold2& )
            {
            }
        );

        assert( destroyWorld.GetContactCount() == 1 );
        assert( destroyWorld.GetBroadPhase().HasPair(
            MakeShapePairKey( groundShape, reusedShape )
        ) );
    }

    // DestroyBody는 연결된 Contact / Shape / proxy를 전부 정리하고 Body slot을 재사용함.
    {
        World bodyWorld{};

        const std::int32_t groundBody =
            bodyWorld.CreateBody( BodyType::Static );

        const std::int32_t groundShape =
            bodyWorld.CreateShape(
                groundBody,
                MakeBox( { 1.0f, 1.0f } )
            );

        const std::int32_t dynamicBody =
            bodyWorld.CreateBody(
                BodyType::Dynamic,
                {
                    { 0.0f, 1.5f },
                    {}
                }
            );

        const std::int32_t touchingShape =
            bodyWorld.CreateShape(
                dynamicBody,
                circle2{ {}, 0.5f }
            );

        bodyWorld.CreateShape(
            dynamicBody,
            circle2{ { 5.0f, 0.0f }, 0.5f }
        );

        assert( bodyWorld.GetBodyCount() == 2 );
        assert( bodyWorld.GetShapeCount() == 3 );

        bodyWorld.UpdateCollisions(
            []( std::int32_t,
                std::int32_t,
                const localManifold2& )
            {
            }
        );

        const ShapePairKey pairKey =
            MakeShapePairKey( groundShape, touchingShape );

        assert( bodyWorld.GetContactCount() == 1 );
        assert( bodyWorld.GetBroadPhase().HasPair( pairKey ) );
        assert( bodyWorld.GetBody( groundBody ).contactCount == 1 );
        assert( bodyWorld.GetBody( dynamicBody ).contactCount == 1 );

        bodyWorld.DestroyBody( dynamicBody );

        assert( bodyWorld.GetBodyCount() == 1 );
        assert( bodyWorld.GetShapeCount() == 1 );
        assert( bodyWorld.GetContactCount() == 0 );
        assert( !bodyWorld.GetBroadPhase().HasPair( pairKey ) );

        const Body& ground =
            bodyWorld.GetBody( groundBody );

        assert( ground.shapeCount == 1 );
        assert( ground.headShapeId == groundShape );
        assert( ground.contactCount == 0 );
        assert( ground.headContactKey == Body::NULL_INDEX );

        assert( bodyWorld.GetBroadPhase()
                    .GetTree( BodyType::Dynamic )
                    .Validate() );

        // 삭제한 Body slot을 다음 Body가 그대로 재사용함.
        const std::int32_t reusedBody =
            bodyWorld.CreateBody(
                BodyType::Dynamic,
                {
                    { 0.0f, 1.5f },
                    {}
                }
            );

        assert( reusedBody == dynamicBody );
        assert( bodyWorld.GetBodyCount() == 2 );
        assert( bodyWorld.GetBody( reusedBody ).bodyId == reusedBody );
        assert( bodyWorld.GetBody( reusedBody ).shapeCount == 0 );
        assert( bodyWorld.GetBody( reusedBody ).contactCount == 0 );

        const std::int32_t reusedShape =
            bodyWorld.CreateShape(
                reusedBody,
                circle2{ {}, 0.5f }
            );

        bodyWorld.UpdateCollisions(
            []( std::int32_t,
                std::int32_t,
                const localManifold2& )
            {
            }
        );

        assert( bodyWorld.GetContactCount() == 1 );
        assert( bodyWorld.GetBroadPhase().HasPair(
            MakeShapePairKey( groundShape, reusedShape )
        ) );
        assert( bodyWorld.GetBody( groundBody ).contactCount == 1 );
        assert( bodyWorld.GetBody( reusedBody ).contactCount == 1 );
    }

    // Box2D처럼 segment-segment 조합은 BroadPhase 후보여도 Contact를 만들지 않음.
    {
        World segmentWorld{};

        const std::int32_t staticBodyId =
            segmentWorld.CreateBody( BodyType::Static );

        segmentWorld.CreateShape(
            staticBodyId,
            segment2{ { -1.0f, 0.0f }, { 1.0f, 0.0f } }
        );

        const std::int32_t dynamicBodyId =
            segmentWorld.CreateBody( BodyType::Dynamic );

        segmentWorld.CreateShape(
            dynamicBodyId,
            segment2{ { -1.0f, 0.0f }, { 1.0f, 0.0f } }
        );

        int contactCount = 0;

        segmentWorld.UpdateCollisions(
            [&]( std::int32_t,
                 std::int32_t,
                 const localManifold2& )
            {
                ++contactCount;
            }
        );

        assert( contactCount == 0 );
        assert( segmentWorld.GetContactCount() == 0 );
    }

    return 0;
}
