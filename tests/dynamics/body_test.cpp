#include <array>
#include <cassert>
#include <cstddef>
#include <limits>
#include <memory>
#include <type_traits>

#include "dynamics/body.h"
#include "dynamics/bodyDef.h"
#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"
#include "dynamics/world.h"
#include "geometry/circle2.h"

using namespace zonai;

// Public handle이 world 객체 주소를 ownership token으로 사용하므로 생성된 world의 주소는 고정되어야 함.
static_assert( !std::is_copy_constructible_v<world> );
static_assert( !std::is_copy_assignable_v<world> );
static_assert( !std::is_move_constructible_v<world> );
static_assert( !std::is_move_assignable_v<world> );

int main()
{

    {
        bodyDef definition{};

        assert( definition.type == bodyType::Static );
        assert( definition.transform.position.x == 0.0f );
        assert( definition.transform.position.y == 0.0f );
        assert( definition.linearVelocity.x == 0.0f );
        assert( definition.linearVelocity.y == 0.0f );
        assert( definition.angularVelocity == 0.0f );
        assert( definition.linearDamping == 0.0f );
        assert( definition.angularDamping == 0.0f );
        assert( definition.gravityScale == 1.0f );
        assert( definition.enableSleep );
        assert( definition.isAwake );
        assert( definition.sleepThreshold == 0.05f );
        assert( definition.safetyFactor == 0.5f );
        assert( definition.enableContactRecycling );
        assert( !definition.isBullet );
        assert( !definition.allowFastRotation );
    }

    {
        body body{};

        // 기본 body는 정적이고 아직 shape가 연결되지 않은 상태로 시작함.
        assert( body.bodyId == body::NULL_INDEX );
        assert( body.generation == 0 );
        assert( body.nextFreeId == body::NULL_INDEX );
        assert( body.type == bodyType::Static );
        assert( body.mass == 0.0f );
        assert( body.inertia == 0.0f );
        assert( body.headContactKey == body::NULL_INDEX );
        assert( body.contactCount == 0 );
        assert( body.headShapeId == body::NULL_INDEX );
        assert( body.shapeCount == 0 );
        assert( body.safetyFactor == 0.5f );

    }

    {
        body body{};

        body.type = bodyType::Dynamic;
        body.headShapeId = 7;
        body.shapeCount = 3;

        assert( body.type == bodyType::Dynamic );
        assert( body.headShapeId == 7 );
        assert( body.shapeCount == 3 );
    }

    {
        bodySim bodySim{};

        // 기본 simulation slot은 비어 있고 transform은 identity임.
        assert( bodySim.bodyId == bodySim::NULL_INDEX );
        assert( bodySim.transform.position.x == 0.0f );
        assert( bodySim.transform.position.y == 0.0f );
        assert( bodySim.transform.rotation.c == 1.0f );
        assert( bodySim.transform.rotation.s == 0.0f );
        assert( bodySim.localCenter.x == 0.0f );
        assert( bodySim.localCenter.y == 0.0f );
        assert( bodySim.center.x == 0.0f );
        assert( bodySim.center.y == 0.0f );
        assert( bodySim.force.x == 0.0f );
        assert( bodySim.force.y == 0.0f );
        assert( bodySim.torque == 0.0f );
        assert( bodySim.invMass == 0.0f );
        assert( bodySim.invInertia == 0.0f );
        assert( bodySim.linearDamping == 0.0f );
        assert( bodySim.angularDamping == 0.0f );
        assert( bodySim.gravityScale == 1.0f );
        assert( bodySim.enableContactRecycling );
        assert( !bodySim.isBullet );
        assert( !bodySim.isFast );
        assert( !bodySim.hadTimeOfImpact );
        assert( !bodySim.allowFastRotation );
        assert( bodySim.minExtent == std::numeric_limits<float>::max() );
        assert( bodySim.maxExtent == 0.0f );

        bodySim.bodyId = 7;
        bodySim.transform.position = { 3.0f, -2.0f };
        bodySim.transform.rotation = rot2::FromRadians( 0.5f );

        assert( bodySim.bodyId == 7 );
        assert( bodySim.transform.position.x == 3.0f );
        assert( bodySim.transform.position.y == -2.0f );
    }

    {
        bodyState state{};

        assert( state.linearVelocity.x == 0.0f );
        assert( state.linearVelocity.y == 0.0f );
        assert( state.angularVelocity == 0.0f );

        state.linearVelocity = { 4.0f, -3.0f };
        state.angularVelocity = 2.5f;

        assert( state.linearVelocity.x == 4.0f );
        assert( state.linearVelocity.y == -3.0f );
        assert( state.angularVelocity == 2.5f );
    }

    {
        // Public handle은 생성된 world가 달라지면 같은 slot/generation이어도 유효하지 않아야 함.
        world worldA{};
        world worldB{};

        const bodyId bodyA =
            worldA.CreateBody( bodyType::Dynamic );

        const bodyId bodyB =
            worldB.CreateBody( bodyType::Dynamic );

        // 두 world의 첫 body라 내부 slot/generation은 의도적으로 같음.
        assert( bodyA.index1 == bodyB.index1 );
        assert( bodyA.generation == bodyB.generation );

        assert( worldA.IsValid( bodyA ) );
        assert( worldB.IsValid( bodyB ) );
        assert( !worldA.IsValid( bodyB ) );
        assert( !worldB.IsValid( bodyA ) );

        const shapeId shapeA =
            worldA.CreateShape(
                bodyA,
                circle2{ {}, 0.5f }
            );

        const shapeId shapeB =
            worldB.CreateShape(
                bodyB,
                circle2{ {}, 0.5f }
            );

        assert( shapeA.index1 == shapeB.index1 );
        assert( shapeA.generation == shapeB.generation );

        assert( worldA.IsValid( shapeA ) );
        assert( worldB.IsValid( shapeB ) );
        assert( !worldA.IsValid( shapeB ) );
        assert( !worldB.IsValid( shapeA ) );

        const bodyId staticBodyA =
            worldA.CreateBody( bodyType::Static );

        const bodyId staticBodyB =
            worldB.CreateBody( bodyType::Static );

        [[maybe_unused]] const shapeId staticShapeA =
            worldA.CreateShape(
                staticBodyA,
                circle2{ {}, 0.5f }
            );

        [[maybe_unused]] const shapeId staticShapeB =
            worldB.CreateShape(
                staticBodyB,
                circle2{ {}, 0.5f }
            );

        contactId contactA{};
        contactId contactB{};

        worldA.UpdateCollisions(
            [&]( const contactData& data )
            {
                contactA = data.id;
            }
        );

        worldB.UpdateCollisions(
            [&]( const contactData& data )
            {
                contactB = data.id;
            }
        );

        assert( !IsNull( contactA ) );
        assert( !IsNull( contactB ) );
        assert( contactA.index1 == contactB.index1 );
        assert( contactA.generation == contactB.generation );

        assert( worldA.IsValid( contactA ) );
        assert( worldB.IsValid( contactB ) );
        assert( !worldA.IsValid( contactB ) );
        assert( !worldB.IsValid( contactA ) );
    }

    {
        // 같은 메모리 주소에 새 world가 생성되어도 이전 world의 handle은 되살아나면 안 됨.
        alignas( world ) std::byte storage[sizeof( world )];

        world* firstWorld =
            std::construct_at(
                reinterpret_cast<world*>( storage )
            );

        const bodyId oldBody =
            firstWorld->CreateBody( bodyType::Dynamic );

        std::destroy_at( firstWorld );

        world* secondWorld =
            std::construct_at(
                reinterpret_cast<world*>( storage )
            );

        const bodyId newBody =
            secondWorld->CreateBody( bodyType::Dynamic );

        assert( oldBody.index1 == newBody.index1 );
        assert( oldBody.generation == newBody.generation );
        assert( secondWorld->IsValid( newBody ) );
        assert( !secondWorld->IsValid( oldBody ) );

        std::destroy_at( secondWorld );
    }

    {
        // Speculative manifold도 solver-active Contact이므로 public Contact query에 포함함.
        world world{};
        world.SetGravity( {} );

        const bodyId staticBody =
            world.CreateBody( bodyType::Static );

        (void)world.CreateShape(
            staticBody,
            circle2{ {}, 1.0f }
        );

        const bodyId dynamicBody =
            world.CreateBody(
                bodyType::Dynamic,
                {
                    { 2.01f, 0.0f },
                    {}
                }
            );

        const shapeId dynamicShape =
            world.CreateShape(
                dynamicBody,
                circle2{ {}, 1.0f }
            );

        world.UpdateCollisions(
            []( const contactData& )
            {
            }
        );

        std::array<contactData, 1> contacts{};

        assert(
            world.GetBodyContactData(
                dynamicBody,
                contacts
            ) == 1
        );

        assert( contacts[0].manifold.pointCount == 1 );
        assert( contacts[0].manifold.points[0].separation > 0.0f );

        assert(
            world.GetShapeContactData(
                dynamicShape,
                contacts
            ) == 1
        );
    }

    return 0;
}
