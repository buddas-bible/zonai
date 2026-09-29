#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>

#include "dynamics/world.h"

using namespace zonai;

namespace
{

constexpr std::int32_t Index( BodyId id )
{
    return id.index1 - 1;
}

constexpr std::int32_t Index( ShapeId id )
{
    return id.index1 - 1;
}

constexpr ShapePairKey PairKey( ShapeId a, ShapeId b )
{
    return MakeShapePairKey( Index( a ), Index( b ) );
}

} // namespace

int main()
{
    constexpr float epsilon = 1e-5f;

    {
        World world{};

        const BodyId staticBody =
            world.CreateBody();

        const BodyId dynamicBody =
            world.CreateBody(
                BodyType::Dynamic,
                {
                    { 3.0f, -2.0f },
                    rot2::FromRadians( 0.5f )
                }
            );

        const BodyId kinematicBody =
            world.CreateBody(
                BodyType::Kinematic,
                {
                    { -4.0f, 1.5f },
                    rot2::FromRadians( -0.25f )
                }
            );

        // public handle은 0을 null로 남기기 위해 내부 index + 1을 저장함.
        assert( staticBody.index1 == 1 );
        assert( dynamicBody.index1 == 2 );
        assert( kinematicBody.index1 == 3 );
        assert( staticBody.generation == 1 );
        assert( dynamicBody.generation == 1 );
        assert( kinematicBody.generation == 1 );

        assert( world.IsValid( staticBody ) );
        assert( world.IsValid( dynamicBody ) );
        assert( world.IsValid( kinematicBody ) );
        assert( !world.IsValid( BodyId{} ) );
        assert( !world.IsValid( ShapeId{} ) );
        assert( !world.IsValid( ContactId{} ) );
        assert( world.GetBodyCount() == 3 );

        const Body& dynamic = world.GetBody( dynamicBody );
        assert( dynamic.bodyId == Index( dynamicBody ) );
        assert( dynamic.type == BodyType::Dynamic );

        const transform2 dynamicTransform =
            world.GetBodyTransform( dynamicBody );

        assert( dynamicTransform.position.x == 3.0f );
        assert( dynamicTransform.position.y == -2.0f );

        const Body& kinematic = world.GetBody( kinematicBody );
        assert( kinematic.type == BodyType::Kinematic );

        const transform2 kinematicTransform =
            world.GetBodyTransform( kinematicBody );

        assert( kinematicTransform.position.x == -4.0f );
        assert( kinematicTransform.position.y == 1.5f );
    }

    // Shape geometry는 Body local space에 저장되고 proxy AABB는 world space로 계산됨.
    {
        World world{};

        const BodyId bodyId =
            world.CreateBody(
                BodyType::Dynamic,
                {
                    { 10.0f, 5.0f },
                    { 0.0f, 1.0f }
                }
            );

        const ShapeId circleId =
            world.CreateShape(
                bodyId,
                circle2{ { 2.0f, 0.0f }, 1.0f }
            );

        assert( world.IsValid( circleId ) );
        assert( world.GetShapeCount() == 1 );

        const Shape& circle = world.GetShape( circleId );
        const Body& body = world.GetBody( bodyId );

        assert( circle.bodyId == Index( bodyId ) );
        assert( circle.generation == circleId.generation );
        assert( circle.proxyKey != Shape::NULL_INDEX );
        assert( GetProxyType( circle.proxyKey ) == BodyType::Dynamic );
        assert( body.headShapeId == Index( circleId ) );
        assert( body.shapeCount == 1 );

        const aabb2& circleAABB =
            world.GetBroadPhase()
                .GetTree( BodyType::Dynamic )
                .GetProxyAABB( GetProxyId( circle.proxyKey ) );

        assert( std::fabs( circleAABB.min.x - 9.0f ) < epsilon );
        assert( std::fabs( circleAABB.min.y - 6.0f ) < epsilon );
        assert( std::fabs( circleAABB.max.x - 11.0f ) < epsilon );
        assert( std::fabs( circleAABB.max.y - 8.0f ) < epsilon );

        const ShapeId segmentId =
            world.CreateShape(
                bodyId,
                segment2{ { 0.0f, 0.0f }, { 1.0f, 0.0f } }
            );

        assert( world.GetBody( bodyId ).headShapeId == Index( segmentId ) );
        assert( world.GetBody( bodyId ).shapeCount == 2 );
        assert( world.GetShape( segmentId ).nextShapeId == Index( circleId ) );
        assert( world.GetShape( circleId ).prevShapeId == Index( segmentId ) );

        world.SetBodyTransform(
            bodyId,
            {
                { 20.0f, -3.0f },
                { 1.0f, 0.0f }
            }
        );

        const Shape& movedCircle = world.GetShape( circleId );
        const aabb2& movedCircleAABB =
            world.GetBroadPhase()
                .GetTree( BodyType::Dynamic )
                .GetProxyAABB( GetProxyId( movedCircle.proxyKey ) );

        assert( std::fabs( movedCircleAABB.min.x - 21.0f ) < epsilon );
        assert( std::fabs( movedCircleAABB.min.y + 4.0f ) < epsilon );
        assert( std::fabs( movedCircleAABB.max.x - 23.0f ) < epsilon );
        assert( std::fabs( movedCircleAABB.max.y + 2.0f ) < epsilon );

        assert( world.GetBroadPhase()
                    .GetTree( BodyType::Dynamic )
                    .Validate() );
    }

    // World collision pipeline과 persistent Contact.
    {
        World world{};

        const BodyId groundBody =
            world.CreateBody( BodyType::Static );

        const ShapeId groundShape =
            world.CreateShape(
                groundBody,
                MakeBox( { 1.0f, 1.0f } )
            );

        const BodyId circleBody =
            world.CreateBody(
                BodyType::Dynamic,
                {
                    { 0.0f, 1.5f },
                    {}
                }
            );

        const ShapeId circleShape =
            world.CreateShape(
                circleBody,
                circle2{ {}, 0.5f }
            );

        int touchingCount = 0;
        ContactId contactId{};

        world.UpdateCollisions(
            [&]( const ContactData& data )
            {
                ++touchingCount;
                contactId = data.contactId;

                assert( data.shapeIdA == groundShape );
                assert( data.shapeIdB == circleShape );
                assert( data.manifold.pointCount == 1 );
                assert( std::fabs( data.manifold.normal.x ) < epsilon );
                assert( std::fabs( data.manifold.normal.y - 1.0f ) < epsilon );
            }
        );

        assert( touchingCount == 1 );
        assert( world.GetContactCount() == 1 );
        assert( world.IsValid( contactId ) );
        assert( contactId.index1 == 1 );
        assert( contactId.generation == 1 );

        const ContactData contactData =
            world.GetContactData( contactId );

        assert( contactData.contactId == contactId );
        assert( contactData.shapeIdA == groundShape );
        assert( contactData.shapeIdB == circleShape );
        assert( contactData.manifold.pointCount == 1 );

        assert( world.GetBroadPhase().HasPair(
            PairKey( groundShape, circleShape )
        ) );

        // 새 BroadPhase 후보가 없어도 persistent Contact는 갱신됨.
        touchingCount = 0;

        world.UpdateCollisions(
            [&]( const ContactData& data )
            {
                ++touchingCount;
                assert( data.contactId == contactId );
                assert( data.manifold.pointCount == 1 );
            }
        );

        assert( touchingCount == 1 );

        // AABB가 분리되면 Contact / pairSet / Body contact list가 함께 정리됨.
        world.SetBodyTransform(
            circleBody,
            {
                { 0.0f, 5.0f },
                {}
            }
        );

        world.UpdateCollisions(
            []( const ContactData& )
            {
            }
        );

        assert( world.GetContactCount() == 0 );
        assert( !world.IsValid( contactId ) );
        assert( !world.GetBroadPhase().HasPair(
            PairKey( groundShape, circleShape )
        ) );
        assert( world.GetBody( groundBody ).contactCount == 0 );
        assert( world.GetBody( circleBody ).contactCount == 0 );

        // 같은 Contact slot을 재사용해도 generation이 달라져 예전 handle은 되살아나지 않음.
        world.SetBodyTransform(
            circleBody,
            {
                { 0.0f, 1.5f },
                {}
            }
        );

        ContactId reusedContactId{};

        world.UpdateCollisions(
            [&]( const ContactData& data )
            {
                reusedContactId = data.contactId;
            }
        );

        assert( world.IsValid( reusedContactId ) );
        assert( reusedContactId.index1 == contactId.index1 );
        assert( reusedContactId.generation != contactId.generation );
        assert( !world.IsValid( contactId ) );
        const ContactData reusedData =
            world.GetContactData( reusedContactId );

        assert( reusedData.contactId == reusedContactId );
        assert( reusedData.shapeIdA == groundShape );
        assert( reusedData.shapeIdB == circleShape );
    }

    // ContactData manifold는 Shape A local이 아니라 world space로 공개됨.
    {
        World world{};

        constexpr float halfPi = 1.57079632679f;

        const BodyId boxBody =
            world.CreateBody(
                BodyType::Static,
                {
                    { 10.0f, 5.0f },
                    rot2::FromRadians( halfPi )
                }
            );

        const ShapeId boxShape =
            world.CreateShape(
                boxBody,
                MakeBox( { 1.0f, 1.0f } )
            );

        const BodyId circleBody =
            world.CreateBody(
                BodyType::Dynamic,
                {
                    { 10.0f, 6.5f },
                    {}
                }
            );

        const ShapeId circleShape =
            world.CreateShape(
                circleBody,
                circle2{ {}, 0.5f }
            );

        ContactData data{};
        int contactCount = 0;

        world.UpdateCollisions(
            [&]( const ContactData& contactData )
            {
                data = contactData;
                ++contactCount;
            }
        );

        assert( contactCount == 1 );
        assert( data.shapeIdA == boxShape );
        assert( data.shapeIdB == circleShape );
        assert( data.manifold.pointCount == 1 );

        // A local +X normal / (1, 0) contact point가
        // 90도 회전 + (10, 5) 이동되어 world +Y / (10, 6)이 됨.
        assert( std::fabs( data.manifold.normal.x ) < epsilon );
        assert( std::fabs( data.manifold.normal.y - 1.0f ) < epsilon );
        assert( std::fabs( data.manifold.points[0].point.x - 10.0f ) < epsilon );
        assert( std::fabs( data.manifold.points[0].point.y - 6.0f ) < epsilon );

        const ContactData snapshot =
            world.GetContactData( data.contactId );

        assert( snapshot.contactId == data.contactId );
        assert( snapshot.shapeIdA == data.shapeIdA );
        assert( snapshot.shapeIdB == data.shapeIdB );
        assert( std::fabs(
            snapshot.manifold.points[0].point.y -
            data.manifold.points[0].point.y
        ) < epsilon );
    }

    // AABB는 겹치지만 실제 geometry가 떨어져 있어도 Contact 자체는 유지됨.
    {
        World world{};

        const BodyId staticBody =
            world.CreateBody( BodyType::Static );

        const ShapeId staticCircle =
            world.CreateShape(
                staticBody,
                circle2{ {}, 1.0f }
            );

        const BodyId dynamicBody =
            world.CreateBody(
                BodyType::Dynamic,
                {
                    { 1.5f, 1.5f },
                    {}
                }
            );

        const ShapeId dynamicCircle =
            world.CreateShape(
                dynamicBody,
                circle2{ {}, 1.0f }
            );

        int touchingCount = 0;

        world.UpdateCollisions(
            [&]( const ContactData& )
            {
                ++touchingCount;
            }
        );

        assert( touchingCount == 0 );
        assert( world.GetContactCount() == 1 );
        assert( world.GetBroadPhase().HasPair(
            PairKey( staticCircle, dynamicCircle )
        ) );
    }

    // Shape slot이 재사용되어도 오래된 ShapeId는 generation mismatch로 무효가 됨.
    {
        World world{};

        const BodyId staticBody =
            world.CreateBody( BodyType::Static );

        const ShapeId groundShape =
            world.CreateShape(
                staticBody,
                MakeBox( { 1.0f, 1.0f } )
            );

        const BodyId dynamicBody =
            world.CreateBody(
                BodyType::Dynamic,
                {
                    { 0.0f, 1.5f },
                    {}
                }
            );

        const ShapeId oldShape =
            world.CreateShape(
                dynamicBody,
                circle2{ {}, 0.5f }
            );

        world.UpdateCollisions(
            []( const ContactData& )
            {
            }
        );

        assert( world.GetContactCount() == 1 );

        world.DestroyShape( oldShape );

        assert( !world.IsValid( oldShape ) );
        assert( world.GetShapeCount() == 1 );
        assert( world.GetContactCount() == 0 );

        const ShapeId newShape =
            world.CreateShape(
                dynamicBody,
                circle2{ {}, 0.5f }
            );

        // 같은 slot을 재사용하지만 generation은 달라야 함.
        assert( newShape.index1 == oldShape.index1 );
        assert( newShape.generation != oldShape.generation );
        assert( world.IsValid( newShape ) );
        assert( !world.IsValid( oldShape ) );

        world.UpdateCollisions(
            []( const ContactData& )
            {
            }
        );

        assert( world.GetContactCount() == 1 );
        assert( world.GetBroadPhase().HasPair(
            PairKey( groundShape, newShape )
        ) );
    }

    // Body slot도 같은 index를 재사용하지만 generation으로 예전 handle을 차단함.
    {
        World world{};

        const BodyId groundBody =
            world.CreateBody( BodyType::Static );

        const ShapeId groundShape =
            world.CreateShape(
                groundBody,
                MakeBox( { 1.0f, 1.0f } )
            );

        const BodyId oldBody =
            world.CreateBody(
                BodyType::Dynamic,
                {
                    { 0.0f, 1.5f },
                    {}
                }
            );

        const ShapeId oldShape =
            world.CreateShape(
                oldBody,
                circle2{ {}, 0.5f }
            );

        world.UpdateCollisions(
            []( const ContactData& )
            {
            }
        );

        assert( world.GetContactCount() == 1 );

        world.DestroyBody( oldBody );

        assert( !world.IsValid( oldBody ) );
        assert( !world.IsValid( oldShape ) );
        assert( world.GetBodyCount() == 1 );
        assert( world.GetShapeCount() == 1 );
        assert( world.GetContactCount() == 0 );

        const BodyId newBody =
            world.CreateBody(
                BodyType::Dynamic,
                {
                    { 0.0f, 1.5f },
                    {}
                }
            );

        assert( newBody.index1 == oldBody.index1 );
        assert( newBody.generation != oldBody.generation );
        assert( world.IsValid( newBody ) );
        assert( !world.IsValid( oldBody ) );

        const ShapeId newShape =
            world.CreateShape(
                newBody,
                circle2{ {}, 0.5f }
            );

        assert( newShape.index1 == oldShape.index1 );
        assert( newShape.generation != oldShape.generation );
        assert( world.IsValid( newShape ) );
        assert( !world.IsValid( oldShape ) );

        world.UpdateCollisions(
            []( const ContactData& )
            {
            }
        );

        assert( world.GetContactCount() == 1 );
        assert( world.GetBroadPhase().HasPair(
            PairKey( groundShape, newShape )
        ) );
        assert( world.GetBody( groundBody ).contactCount == 1 );
        assert( world.GetBody( newBody ).contactCount == 1 );
    }

    // Body / Shape Contact query는 capacity와 실제 touching 수를 구분함.
    {
        World world{};

        const BodyId dynamicBody =
            world.CreateBody( BodyType::Dynamic );

        const ShapeId touchingShape =
            world.CreateShape(
                dynamicBody,
                circle2{ {}, 1.0f }
            );

        const ShapeId nonTouchingShape =
            world.CreateShape(
                dynamicBody,
                circle2{ { 10.0f, 0.0f }, 1.0f }
            );

        const BodyId touchingStaticBody =
            world.CreateBody(
                BodyType::Static,
                {
                    { 1.5f, 0.0f },
                    {}
                }
            );

        const ShapeId touchingStaticShape =
            world.CreateShape(
                touchingStaticBody,
                circle2{ {}, 1.0f }
            );

        const BodyId overlapStaticBody =
            world.CreateBody(
                BodyType::Static,
                {
                    { 11.5f, 1.5f },
                    {}
                }
            );

        const ShapeId overlapStaticShape =
            world.CreateShape(
                overlapStaticBody,
                circle2{ {}, 1.0f }
            );

        int touchingCallbackCount = 0;

        world.UpdateCollisions(
            [&]( const ContactData& )
            {
                ++touchingCallbackCount;
            }
        );

        // dynamicBody에는 AABB Contact가 2개지만 실제 geometry 접촉은 1개뿐임.
        assert( world.GetContactCount() == 2 );
        assert( world.GetBody( dynamicBody ).contactCount == 2 );
        assert( touchingCallbackCount == 1 );

        const std::size_t bodyCapacity =
            world.GetBodyContactCapacity( dynamicBody );

        assert( bodyCapacity == 2 );

        std::array<ContactData, 2> bodyContacts{};

        const std::size_t bodyContactCount =
            world.GetBodyContactData(
                dynamicBody,
                bodyContacts
            );

        assert( bodyContactCount == 1 );
        assert( bodyContacts[0].manifold.pointCount > 0 );

        const bool bodyHasTouchingPair =
            ( bodyContacts[0].shapeIdA == touchingShape &&
              bodyContacts[0].shapeIdB == touchingStaticShape ) ||
            ( bodyContacts[0].shapeIdA == touchingStaticShape &&
              bodyContacts[0].shapeIdB == touchingShape );

        assert( bodyHasTouchingPair );

        // Shape capacity도 Body contactCount를 그대로 사용하므로 보수적으로 2임.
        assert( world.GetShapeContactCapacity( touchingShape ) == 2 );
        assert( world.GetShapeContactCapacity( nonTouchingShape ) == 2 );

        std::array<ContactData, 2> touchingContacts{};
        const std::size_t touchingCount =
            world.GetShapeContactData(
                touchingShape,
                touchingContacts
            );

        assert( touchingCount == 1 );
        assert(
            touchingContacts[0].shapeIdA == touchingShape ||
            touchingContacts[0].shapeIdB == touchingShape
        );

        std::array<ContactData, 2> nonTouchingContacts{};
        const std::size_t nonTouchingCount =
            world.GetShapeContactData(
                nonTouchingShape,
                nonTouchingContacts
            );

        // AABB Contact는 존재하지만 manifold가 비어 있으므로 public query에서는 제외됨.
        assert( nonTouchingCount == 0 );

        // 정적 Body/Shape 쪽에서도 같은 touching Contact를 조회할 수 있음.
        assert( world.GetBodyContactCapacity( touchingStaticBody ) == 1 );
        assert( world.GetShapeContactCapacity( touchingStaticShape ) == 1 );

        std::array<ContactData, 1> staticContacts{};
        const std::size_t staticCount =
            world.GetBodyContactData(
                touchingStaticBody,
                staticContacts
            );

        assert( staticCount == 1 );
        assert( staticContacts[0].contactId == bodyContacts[0].contactId );

        // 두 번째 정적 Shape는 AABB overlap만 있으므로 capacity 1, 실제 반환 0.
        assert( world.GetBodyContactCapacity( overlapStaticBody ) == 1 );
        assert( world.GetShapeContactCapacity( overlapStaticShape ) == 1 );

        std::array<ContactData, 1> overlapContacts{};
        assert(
            world.GetShapeContactData(
                overlapStaticShape,
                overlapContacts
            ) == 0
        );

        // output span이 비어 있으면 아무 것도 쓰지 않고 0을 반환함.
        assert(
            world.GetBodyContactData(
                dynamicBody,
                std::span<ContactData>{}
            ) == 0
        );
    }

    // Box2D처럼 segment-segment 조합은 BroadPhase 후보여도 Contact를 만들지 않음.
    {
        World world{};

        const BodyId staticBody =
            world.CreateBody( BodyType::Static );

        (void)world.CreateShape(
            staticBody,
            segment2{ { -1.0f, 0.0f }, { 1.0f, 0.0f } }
        );

        const BodyId dynamicBody =
            world.CreateBody( BodyType::Dynamic );

        (void)world.CreateShape(
            dynamicBody,
            segment2{ { -1.0f, 0.0f }, { 1.0f, 0.0f } }
        );

        int contactCount = 0;

        world.UpdateCollisions(
            [&]( const ContactData& )
            {
                ++contactCount;
            }
        );

        assert( contactCount == 0 );
        assert( world.GetContactCount() == 0 );
    }

    return 0;
}
