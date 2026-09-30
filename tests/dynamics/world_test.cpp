#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>

#include "dynamics/world.h"

using namespace zonai;

namespace
{

constexpr std::int32_t Index( bodyId id )
{
    return id.index1 - 1;
}

constexpr std::int32_t Index( shapeId id )
{
    return id.index1 - 1;
}

constexpr shapePairKey PairKey( shapeId a, shapeId b )
{
    return MakeShapePairKey( Index( a ), Index( b ) );
}

} // namespace

int main()
{
    constexpr float epsilon = 1e-5f;

    {
        world world{};

        const bodyId staticBody =
            world.CreateBody();

        const bodyId dynamicBody =
            world.CreateBody(
                bodyType::Dynamic,
                {
                    { 3.0f, -2.0f },
                    rot2::FromRadians( 0.5f )
                }
            );

        const bodyId kinematicBody =
            world.CreateBody(
                bodyType::Kinematic,
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
        assert( !world.IsValid( bodyId{} ) );
        assert( !world.IsValid( shapeId{} ) );
        assert( !world.IsValid( contactId{} ) );
        assert( world.GetBodyCount() == 3 );

        const body& dynamic = world.GetBody( dynamicBody );
        assert( dynamic.bodyId == Index( dynamicBody ) );
        assert( dynamic.type == bodyType::Dynamic );

        const transform2 dynamicTransform =
            world.GetBodyTransform( dynamicBody );

        assert( dynamicTransform.position.x == 3.0f );
        assert( dynamicTransform.position.y == -2.0f );

        const body& kinematic = world.GetBody( kinematicBody );
        assert( kinematic.type == bodyType::Kinematic );

        const transform2 kinematicTransform =
            world.GetBodyTransform( kinematicBody );

        assert( kinematicTransform.position.x == -4.0f );
        assert( kinematicTransform.position.y == 1.5f );
    }

    // BodyState는 non-static body의 선속도 / 각속도를 보관함.
    {
        world world{};

        const bodyId staticBody =
            world.CreateBody( bodyType::Static );

        const bodyId dynamicBody =
            world.CreateBody( bodyType::Dynamic );

        const bodyId kinematicBody =
            world.CreateBody( bodyType::Kinematic );

        assert( world.GetBodyLinearVelocity( staticBody ).x == 0.0f );
        assert( world.GetBodyLinearVelocity( staticBody ).y == 0.0f );
        assert( world.GetBodyAngularVelocity( staticBody ) == 0.0f );

        world.SetBodyLinearVelocity( staticBody, { 3.0f, 4.0f } );
        world.SetBodyAngularVelocity( staticBody, 5.0f );

        // Static body는 setter를 무시함.
        assert( world.GetBodyLinearVelocity( staticBody ).x == 0.0f );
        assert( world.GetBodyLinearVelocity( staticBody ).y == 0.0f );
        assert( world.GetBodyAngularVelocity( staticBody ) == 0.0f );

        world.SetBodyLinearVelocity( dynamicBody, { 3.0f, -4.0f } );
        world.SetBodyAngularVelocity( dynamicBody, 2.0f );

        const vec2 dynamicVelocity =
            world.GetBodyLinearVelocity( dynamicBody );

        assert( dynamicVelocity.x == 3.0f );
        assert( dynamicVelocity.y == -4.0f );
        assert( world.GetBodyAngularVelocity( dynamicBody ) == 2.0f );

        world.SetBodyLinearVelocity( kinematicBody, { -1.0f, 6.0f } );
        world.SetBodyAngularVelocity( kinematicBody, -0.5f );

        const vec2 kinematicVelocity =
            world.GetBodyLinearVelocity( kinematicBody );

        assert( kinematicVelocity.x == -1.0f );
        assert( kinematicVelocity.y == 6.0f );
        assert( world.GetBodyAngularVelocity( kinematicBody ) == -0.5f );

        // body slot 재사용 시 이전 body의 운동 상태는 남지 않아야 함.
        const bodyId oldBody = dynamicBody;
        world.DestroyBody( oldBody );

        const bodyId reusedBody =
            world.CreateBody( bodyType::Dynamic );

        assert( reusedBody.index1 == oldBody.index1 );
        assert( reusedBody.generation != oldBody.generation );
        assert( world.GetBodyLinearVelocity( reusedBody ).x == 0.0f );
        assert( world.GetBodyLinearVelocity( reusedBody ).y == 0.0f );
        assert( world.GetBodyAngularVelocity( reusedBody ) == 0.0f );
    }

    // Dynamic body의 mass data는 연결된 shape density / geometry를 합산해 계산됨.
    {
        world world{};

        const bodyId dynamicBody =
            world.CreateBody( bodyType::Dynamic );

        const shapeId rightCircle =
            world.CreateShape(
                dynamicBody,
                circle2{ { 2.0f, 0.0f }, 1.0f },
                {},
                2.0f
            );

        constexpr float pi = 3.14159265358979323846f;

        assert( std::fabs( world.GetShapeDensity( rightCircle ) - 2.0f ) < epsilon );
        assert( std::fabs( world.GetBodyMass( dynamicBody ) - 2.0f * pi ) < epsilon );
        assert( std::fabs( world.GetBodyLocalCenter( dynamicBody ).x - 2.0f ) < epsilon );
        assert( std::fabs( world.GetBodyLocalCenter( dynamicBody ).y ) < epsilon );
        assert(
            std::fabs(
                world.GetBodyRotationalInertia( dynamicBody ) -
                pi
            ) < epsilon
        );

        const shapeId leftCircle =
            world.CreateShape(
                dynamicBody,
                circle2{ { -2.0f, 0.0f }, 1.0f },
                {},
                2.0f
            );

        assert( std::fabs( world.GetBodyMass( dynamicBody ) - 4.0f * pi ) < epsilon );
        assert( std::fabs( world.GetBodyLocalCenter( dynamicBody ).x ) < epsilon );
        assert( std::fabs( world.GetBodyLocalCenter( dynamicBody ).y ) < epsilon );
        assert(
            std::fabs(
                world.GetBodyRotationalInertia( dynamicBody ) -
                18.0f * pi
            ) < epsilon
        );

        // density 0 shape는 collision geometry로는 남지만 body mass에는 기여하지 않음.
        world.SetShapeDensity( leftCircle, 0.0f );

        assert( world.GetShapeDensity( leftCircle ) == 0.0f );
        assert( std::fabs( world.GetBodyMass( dynamicBody ) - 2.0f * pi ) < epsilon );
        assert( std::fabs( world.GetBodyLocalCenter( dynamicBody ).x - 2.0f ) < epsilon );
        assert(
            std::fabs(
                world.GetBodyRotationalInertia( dynamicBody ) -
                pi
            ) < epsilon
        );

        world.DestroyShape( rightCircle );

        assert( world.GetBodyMass( dynamicBody ) == 0.0f );
        assert( world.GetBodyRotationalInertia( dynamicBody ) == 0.0f );
        assert( world.GetBodyLocalCenter( dynamicBody ).x == 0.0f );
        assert( world.GetBodyLocalCenter( dynamicBody ).y == 0.0f );

        // Static / Kinematic은 shape density와 무관하게 solver mass가 0임.
        const bodyId staticBody =
            world.CreateBody( bodyType::Static );

        (void)world.CreateShape(
            staticBody,
            circle2{ {}, 2.0f },
            {},
            5.0f
        );

        assert( world.GetBodyMass( staticBody ) == 0.0f );
        assert( world.GetBodyRotationalInertia( staticBody ) == 0.0f );
    }

    // shape surface 값은 mass와 독립적으로 저장되고 필요할 때 Contact에서 mix됨.
    {
        world world{};

        const bodyId bodyId =
            world.CreateBody( bodyType::Dynamic );

        const shapeId shapeId =
            world.CreateShape(
                bodyId,
                circle2{ {}, 1.0f }
            );

        assert( std::fabs( world.GetShapeFriction( shapeId ) - 0.6f ) < epsilon );
        assert( world.GetShapeRestitution( shapeId ) == 0.0f );

        world.SetShapeFriction( shapeId, 0.25f );
        world.SetShapeRestitution( shapeId, 0.75f );

        assert( std::fabs( world.GetShapeFriction( shapeId ) - 0.25f ) < epsilon );
        assert( std::fabs( world.GetShapeRestitution( shapeId ) - 0.75f ) < epsilon );
    }

    // Linear impulse는 timeStep 없이 즉시 COM velocity를 변경함.
    {
        world world{};
        world.SetGravity( {} );

        const bodyId bodyId =
            world.CreateBody( bodyType::Dynamic );

        (void)world.CreateShape(
            bodyId,
            circle2{ {}, 1.0f },
            {},
            2.0f
        );

        const float mass =
            world.GetBodyMass( bodyId );

        // J = M * 3 이므로
        //
        //     DeltaV = J / M = 3
        world.ApplyLinearImpulseToCenter(
            bodyId,
            { mass * 3.0f, 0.0f }
        );

        // Step을 호출하지 않아도 impulse는 즉시 velocity에 반영됨.
        assert(
            std::fabs(
                world.GetBodyLinearVelocity( bodyId ).x -
                3.0f
            ) < epsilon
        );
        assert( world.GetBodyAngularVelocity( bodyId ) == 0.0f );

        world.Step( 0.5f );

        // impulse는 force처럼 누적되어 다시 적용되지 않고
        // 이미 바뀐 velocity만 position 적분에 사용됨.
        assert(
            std::fabs(
                world.GetBodyLinearVelocity( bodyId ).x -
                3.0f
            ) < epsilon
        );
        assert(
            std::fabs(
                world.GetBodyTransform( bodyId ).position.x -
                1.5f
            ) < epsilon
        );
    }

    // COM에서 벗어난 point의 linear impulse는 선속도와 각속도를 동시에 변경함.
    {
        world world{};
        world.SetGravity( {} );

        const bodyId bodyId =
            world.CreateBody( bodyType::Dynamic );

        (void)world.CreateShape(
            bodyId,
            circle2{ {}, 1.0f }
        );

        const float mass =
            world.GetBodyMass( bodyId );

        const float inertia =
            world.GetBodyRotationalInertia( bodyId );

        /*
        * point = (1, 0), impulse = (0, I*2)
        *
        *     r = (1, 0)
        *
        *     angularImpulse
        *         = r x J
        *         = I * 2
        *
        *     DeltaW
        *         = angularImpulse / I
        *         = 2 rad/s
        */
        world.ApplyLinearImpulse(
            bodyId,
            { 0.0f, inertia * 2.0f },
            { 1.0f, 0.0f }
        );

        assert(
            std::fabs(
                world.GetBodyAngularVelocity( bodyId ) -
                2.0f
            ) < epsilon
        );

        // 같은 impulse 자체도 COM 선운동량을 바꾸므로 +Y velocity가 생김.
        assert(
            std::fabs(
                world.GetBodyLinearVelocity( bodyId ).y -
                ( inertia * 2.0f / mass )
            ) < epsilon
        );
    }

    // Angular impulse는 선속도에 영향을 주지 않고 각속도만 즉시 변경함.
    {
        world world{};
        world.SetGravity( {} );

        const bodyId bodyId =
            world.CreateBody( bodyType::Dynamic );

        (void)world.CreateShape(
            bodyId,
            circle2{ {}, 1.0f }
        );

        const float inertia =
            world.GetBodyRotationalInertia( bodyId );

        // L = I * 4 이므로
        //
        //     DeltaW = L / I = 4 rad/s
        world.ApplyAngularImpulse(
            bodyId,
            inertia * 4.0f
        );

        assert(
            std::fabs(
                world.GetBodyAngularVelocity( bodyId ) -
                4.0f
            ) < epsilon
        );
        assert( world.GetBodyLinearVelocity( bodyId ).x == 0.0f );
        assert( world.GetBodyLinearVelocity( bodyId ).y == 0.0f );
    }

    // Static / Kinematic body는 impulse에 의해 velocity가 바뀌지 않음.
    {
        world world{};
        world.SetGravity( {} );

        const bodyId staticBody =
            world.CreateBody( bodyType::Static );

        const bodyId kinematicBody =
            world.CreateBody( bodyType::Kinematic );

        world.SetBodyLinearVelocity(
            kinematicBody,
            { 1.0f, 2.0f }
        );
        world.SetBodyAngularVelocity(
            kinematicBody,
            3.0f
        );

        world.ApplyLinearImpulseToCenter(
            staticBody,
            { 100.0f, 100.0f }
        );
        world.ApplyAngularImpulse(
            staticBody,
            100.0f
        );

        world.ApplyLinearImpulse(
            kinematicBody,
            { 100.0f, 100.0f },
            { 10.0f, 0.0f }
        );
        world.ApplyAngularImpulse(
            kinematicBody,
            100.0f
        );

        assert( world.GetBodyLinearVelocity( staticBody ).x == 0.0f );
        assert( world.GetBodyLinearVelocity( staticBody ).y == 0.0f );
        assert( world.GetBodyAngularVelocity( staticBody ) == 0.0f );

        assert( world.GetBodyLinearVelocity( kinematicBody ).x == 1.0f );
        assert( world.GetBodyLinearVelocity( kinematicBody ).y == 2.0f );
        assert( world.GetBodyAngularVelocity( kinematicBody ) == 3.0f );
    }

    // Contact Solver는 접근 속도를 막는 동시에 기존 penetration도 조금씩 회복함.
    {
        world world{};
        world.SetGravity( {} );

        const bodyId staticBody =
            world.CreateBody(
                bodyType::Static,
                {
                    { 0.0f, 0.0f },
                    {}
                }
            );

        (void)world.CreateShape(
            staticBody,
            circle2{ {}, 1.0f }
        );

        const bodyId dynamicBody =
            world.CreateBody(
                bodyType::Dynamic,
                {
                    { 1.5f, 0.0f },
                    {}
                }
            );

        (void)world.CreateShape(
            dynamicBody,
            circle2{ {}, 1.0f }
        );

        world.SetBodyLinearVelocity(
            dynamicBody,
            { -2.0f, 0.0f }
        );

        world.Step( 0.25f );

        const vec2 velocity =
            world.GetBodyLinearVelocity(
                dynamicBody
            );

        const transform2 transform =
            world.GetBodyTransform(
                dynamicBody
            );

        // Push가 penetration을 줄이는 방향으로 position을 이동시키고,
        // Relax가 그 과정에서 만든 correction velocity는 다시 제거함.
        assert(
            std::fabs( velocity.x ) <
            epsilon
        );
        assert( transform.position.x > 1.5f );
        assert( transform.position.x < 2.0f );
    }

    // 접근 속도가 전혀 없어도 이미 겹친 Contact는 position만 점진적으로 회복함.
    {
        world world{};
        world.SetGravity( {} );

        const bodyId staticBody =
            world.CreateBody(
                bodyType::Static,
                {
                    { 0.0f, 0.0f },
                    {}
                }
            );

        (void)world.CreateShape(
            staticBody,
            circle2{ {}, 1.0f }
        );

        const bodyId dynamicBody =
            world.CreateBody(
                bodyType::Dynamic,
                {
                    { 1.5f, 0.0f },
                    {}
                }
            );

        (void)world.CreateShape(
            dynamicBody,
            circle2{ {}, 1.0f }
        );

        world.Step( 1.0f / 60.0f );

        const transform2 transform =
            world.GetBodyTransform(
                dynamicBody
            );

        // 처음에는 중심 거리가 1.5m라 0.5m 관통 상태임.
        // soft push가 한 step에서 일부를 회복하되 순간적으로 전부 밀어내지는 않음.
        assert( transform.position.x > 1.5f );
        assert( transform.position.x < 2.0f );

        // correction용 분리 속도는 position 적분 뒤 relax에서 제거됨.
        assert(
            std::fabs(
                world.GetBodyLinearVelocity(
                    dynamicBody
                ).x
            ) < epsilon
        );
    }

    // Contact가 있어도 서로 분리 중인 body에는 음수 normal impulse를 가하지 않음.
    {
        world world{};
        world.SetGravity( {} );

        const bodyId staticBody =
            world.CreateBody(
                bodyType::Static,
                {
                    { 0.0f, 0.0f },
                    {}
                }
            );

        (void)world.CreateShape(
            staticBody,
            circle2{ {}, 1.0f }
        );

        const bodyId dynamicBody =
            world.CreateBody(
                bodyType::Dynamic,
                {
                    { 1.5f, 0.0f },
                    {}
                }
            );

        (void)world.CreateShape(
            dynamicBody,
            circle2{ {}, 1.0f }
        );

        world.SetBodyLinearVelocity(
            dynamicBody,
            { 2.0f, 0.0f }
        );

        world.Step( 0.25f );

        assert(
            std::fabs(
                world.GetBodyLinearVelocity(
                    dynamicBody
                ).x -
                2.0f
            ) < epsilon
        );

        assert(
            std::fabs(
                world.GetBodyTransform(
                    dynamicBody
                ).position.x -
                2.0f
            ) < epsilon
        );
    }

    // 같은 질량의 Dynamic body 둘이 정면으로 접근하면 normal impulse가 상대속도를 제거함.
    {
        world world{};
        world.SetGravity( {} );

        const bodyId bodyA =
            world.CreateBody(
                bodyType::Dynamic,
                {
                    { -0.75f, 0.0f },
                    {}
                }
            );

        const bodyId bodyB =
            world.CreateBody(
                bodyType::Dynamic,
                {
                    { 0.75f, 0.0f },
                    {}
                }
            );

        (void)world.CreateShape(
            bodyA,
            circle2{ {}, 1.0f }
        );

        (void)world.CreateShape(
            bodyB,
            circle2{ {}, 1.0f }
        );

        world.SetBodyLinearVelocity(
            bodyA,
            { 1.0f, 0.0f }
        );

        world.SetBodyLinearVelocity(
            bodyB,
            { -1.0f, 0.0f }
        );

        world.Step( 0.25f );

        assert(
            std::fabs(
                world.GetBodyLinearVelocity(
                    bodyA
                ).x
            ) < epsilon
        );

        assert(
            std::fabs(
                world.GetBodyLinearVelocity(
                    bodyB
                ).x
            ) < epsilon
        );
    }

    // Contact friction은 normal impulse의 Coulomb 한계 안에서 tangential velocity를 줄임.
    {
        world world{};
        world.SetGravity( {} );

        const bodyId staticBody =
            world.CreateBody(
                bodyType::Static,
                {
                    { 0.0f, 0.0f },
                    {}
                }
            );

        (void)world.CreateShape(
            staticBody,
            circle2{ {}, 1.0f }
        );

        const bodyId dynamicBody =
            world.CreateBody(
                bodyType::Dynamic,
                {
                    { 0.0f, 1.5f },
                    {}
                }
            );

        (void)world.CreateShape(
            dynamicBody,
            circle2{ {}, 1.0f }
        );

        world.SetBodyLinearVelocity(
            dynamicBody,
            { 4.0f, -1.0f }
        );

        world.Step( 1.0f / 60.0f );

        const vec2 velocity =
            world.GetBodyLinearVelocity(
                dynamicBody
            );

        // normal constraint는 아래쪽 접근 속도를 제거하고,
        // 그 normal impulse를 한계로 friction이 +X 미끄러짐을 줄임.
        assert( std::fabs( velocity.y ) < epsilon );
        assert( velocity.x > 0.0f );
        assert( velocity.x < 4.0f );
    }

    // restitution은 충돌 전 normal 접근 속도를 반대 방향 속도로 되돌림.
    {
        world world{};
        world.SetGravity( {} );

        const bodyId staticBody =
            world.CreateBody(
                bodyType::Static,
                {
                    { 0.0f, 0.0f },
                    {}
                }
            );

        const shapeId staticShape =
            world.CreateShape(
                staticBody,
                circle2{ {}, 1.0f }
            );

        const bodyId dynamicBody =
            world.CreateBody(
                bodyType::Dynamic,
                {
                    { 1.9f, 0.0f },
                    {}
                }
            );

        const shapeId dynamicShape =
            world.CreateShape(
                dynamicBody,
                circle2{ {}, 1.0f }
            );

        // 기본 restitution 0인 static shape와 1인 dynamic shape는
        // max mixing으로 Contact restitution 1이 됨.
        world.SetShapeRestitution(
            dynamicShape,
            1.0f
        );

        assert( world.GetShapeRestitution( staticShape ) == 0.0f );
        assert( world.GetShapeRestitution( dynamicShape ) == 1.0f );

        world.SetBodyLinearVelocity(
            dynamicBody,
            { -4.0f, 0.0f }
        );

        world.Step( 1.0f / 60.0f );

        // 충돌 전 vn=-4, e=1이므로 relax 뒤 restitution 목표는 +4임.
        assert(
            std::fabs(
                world.GetBodyLinearVelocity(
                    dynamicBody
                ).x -
                4.0f
            ) < 1e-3f
        );
    }

    // 기본 gravity는 Dynamic body의 COM velocity에만 적용됨.
    {
        world world{};

        const vec2 defaultGravity = world.GetGravity();
        assert( defaultGravity.x == 0.0f );
        assert( defaultGravity.y == -10.0f );

        const bodyId dynamicBody =
            world.CreateBody( bodyType::Dynamic );

        (void)world.CreateShape(
            dynamicBody,
            circle2{ {}, 1.0f }
        );

        const bodyId kinematicBody =
            world.CreateBody( bodyType::Kinematic );

        world.SetBodyLinearVelocity(
            kinematicBody,
            { 0.0f, 2.0f }
        );

        world.Step( 0.5f );

        const vec2 dynamicVelocity =
            world.GetBodyLinearVelocity( dynamicBody );

        // v = 0 + g * dt = -10 * 0.5 = -5
        assert( std::fabs( dynamicVelocity.y + 5.0f ) < epsilon );

        const transform2 dynamicTransform =
            world.GetBodyTransform( dynamicBody );

        // semi-implicit Euler:
        // y = 0 + v_new * dt = -5 * 0.5 = -2.5
        assert( std::fabs( dynamicTransform.position.y + 2.5f ) < epsilon );

        // Kinematic은 gravity를 받지 않고 지정한 velocity만 적분함.
        assert(
            std::fabs(
                world.GetBodyLinearVelocity( kinematicBody ).y -
                2.0f
            ) < epsilon
        );
        assert(
            std::fabs(
                world.GetBodyTransform( kinematicBody ).position.y -
                1.0f
            ) < epsilon
        );
    }

    // Force는 F=Ma에 따라 질량으로 나뉘어 velocity를 바꾸고 한 Step 뒤 초기화됨.
    {
        world world{};
        world.SetGravity( {} );

        const bodyId bodyId =
            world.CreateBody( bodyType::Dynamic );

        (void)world.CreateShape(
            bodyId,
            circle2{ {}, 1.0f },
            {},
            2.0f
        );

        const float mass = world.GetBodyMass( bodyId );

        // F = M * 4 이므로 acceleration.x = 4.
        world.ApplyForceToCenter(
            bodyId,
            { mass * 4.0f, 0.0f }
        );

        world.Step( 0.5f );

        // v = a * dt = 4 * 0.5 = 2
        assert(
            std::fabs(
                world.GetBodyLinearVelocity( bodyId ).x -
                2.0f
            ) < epsilon
        );

        // x = v_new * dt = 2 * 0.5 = 1
        assert(
            std::fabs(
                world.GetBodyTransform( bodyId ).position.x -
                1.0f
            ) < epsilon
        );

        // force는 이전 Step에서 소비됐으므로 다시 Apply하지 않으면
        // 다음 Step에서는 같은 velocity로만 이동함.
        world.Step( 0.5f );

        assert(
            std::fabs(
                world.GetBodyLinearVelocity( bodyId ).x -
                2.0f
            ) < epsilon
        );
        assert(
            std::fabs(
                world.GetBodyTransform( bodyId ).position.x -
                2.0f
            ) < epsilon
        );
    }

    // center에서 벗어난 point에 Force를 가하면 r x F만큼 torque도 누적됨.
    {
        world world{};
        world.SetGravity( {} );

        const bodyId bodyId =
            world.CreateBody( bodyType::Dynamic );

        (void)world.CreateShape(
            bodyId,
            circle2{ {}, 1.0f }
        );

        const float inertia =
            world.GetBodyRotationalInertia( bodyId );

        // r=(1,0), F=(0,inertia*2)이므로
        // torque = r x F = inertia*2
        // angularAcceleration = torque / inertia = 2 rad/s^2
        world.ApplyForce(
            bodyId,
            { 0.0f, inertia * 2.0f },
            { 1.0f, 0.0f }
        );

        world.Step( 0.5f );

        // w = alpha * dt = 2 * 0.5 = 1 rad/s
        assert(
            std::fabs(
                world.GetBodyAngularVelocity( bodyId ) -
                1.0f
            ) < epsilon
        );

        // 같은 Force는 COM에도 선가속도를 만들므로 +Y velocity도 생김.
        assert( world.GetBodyLinearVelocity( bodyId ).y > 0.0f );
    }

    // 순수 torque는 COM 위치를 움직이지 않고 body origin만 COM 주위로 회전시킴.
    {
        world world{};
        world.SetGravity( {} );

        const bodyId bodyId =
            world.CreateBody( bodyType::Dynamic );

        (void)world.CreateShape(
            bodyId,
            circle2{ { 2.0f, 0.0f }, 1.0f }
        );

        const vec2 localCenter =
            world.GetBodyLocalCenter( bodyId );

        const transform2 before =
            world.GetBodyTransform( bodyId );

        const vec2 worldCenterBefore =
            TransformPoint( before, localCenter );

        const float inertia =
            world.GetBodyRotationalInertia( bodyId );

        // alpha = torque * invInertia = 2 rad/s^2
        world.ApplyTorque(
            bodyId,
            inertia * 2.0f
        );

        world.Step( 0.5f );

        const transform2 after =
            world.GetBodyTransform( bodyId );

        const vec2 worldCenterAfter =
            TransformPoint( after, localCenter );

        assert(
            std::fabs(
                worldCenterAfter.x -
                worldCenterBefore.x
            ) < epsilon
        );
        assert(
            std::fabs(
                worldCenterAfter.y -
                worldCenterBefore.y
            ) < epsilon
        );

        // COM이 local origin에서 떨어져 있으므로 회전 후 body origin은 이동함.
        assert(
            std::fabs(
                after.position.x -
                before.position.x
            ) > epsilon ||
            std::fabs(
                after.position.y -
                before.position.y
            ) > epsilon
        );
    }

    // ClearForces는 아직 소비하지 않은 force / torque를 제거함.
    {
        world world{};
        world.SetGravity( {} );

        const bodyId bodyId =
            world.CreateBody( bodyType::Dynamic );

        (void)world.CreateShape(
            bodyId,
            circle2{ {}, 1.0f }
        );

        world.ApplyForceToCenter(
            bodyId,
            { 100.0f, 0.0f }
        );
        world.ApplyTorque(
            bodyId,
            100.0f
        );

        world.ClearForces( bodyId );
        world.Step( 0.5f );

        assert( world.GetBodyLinearVelocity( bodyId ).x == 0.0f );
        assert( world.GetBodyAngularVelocity( bodyId ) == 0.0f );
    }

    // Step은 COM velocity로 Dynamic / Kinematic transform을 적분하고 proxy를 동기화함.
    {
        world world{};
        world.SetGravity( {} );

        const bodyId staticBody =
            world.CreateBody(
                bodyType::Static,
                {
                    { 5.0f, 2.0f },
                    {}
                }
            );

        const bodyId dynamicBody =
            world.CreateBody( bodyType::Dynamic );

        const shapeId dynamicShape =
            world.CreateShape(
                dynamicBody,
                circle2{ { 1.0f, 0.0f }, 0.25f }
            );

        const bodyId kinematicBody =
            world.CreateBody(
                bodyType::Kinematic,
                {
                    { -2.0f, 3.0f },
                    {}
                }
            );

        world.SetBodyLinearVelocity( staticBody, { 100.0f, 100.0f } );
        world.SetBodyAngularVelocity( staticBody, 10.0f );

        world.SetBodyLinearVelocity( dynamicBody, { 2.0f, -1.0f } );
        world.SetBodyAngularVelocity(
            dynamicBody,
            1.57079632679f
        );

        world.SetBodyLinearVelocity( kinematicBody, { 0.0f, 4.0f } );

        world.Step( 0.5f );

        const transform2 staticTransform =
            world.GetBodyTransform( staticBody );

        assert( std::fabs( staticTransform.position.x - 5.0f ) < epsilon );
        assert( std::fabs( staticTransform.position.y - 2.0f ) < epsilon );

        const transform2 dynamicTransform =
            world.GetBodyTransform( dynamicBody );

        const float sqrtHalf = std::sqrt( 0.5f );

        // circle 하나만 있으므로 localCenter=(1,0)이고,
        // COM은 (1,0) -> (2,-0.5)로 이동한 뒤 45도 회전함.
        // body origin은 center - R * localCenter로 다시 계산됨.
        assert(
            std::fabs(
                dynamicTransform.position.x -
                ( 2.0f - sqrtHalf )
            ) < epsilon
        );
        assert(
            std::fabs(
                dynamicTransform.position.y -
                ( -0.5f - sqrtHalf )
            ) < epsilon
        );

        assert( std::fabs( dynamicTransform.rotation.c - sqrtHalf ) < epsilon );
        assert( std::fabs( dynamicTransform.rotation.s - sqrtHalf ) < epsilon );

        const transform2 kinematicTransform =
            world.GetBodyTransform( kinematicBody );

        assert( std::fabs( kinematicTransform.position.x + 2.0f ) < epsilon );
        assert( std::fabs( kinematicTransform.position.y - 5.0f ) < epsilon );

        // local (1, 0)인 circle 중심도 body 회전에 따라 움직였는지 proxy AABB로 확인함.
        const shape& movedShape = world.GetShape( dynamicShape );
        const aabb2& movedAABB =
            world.GetBroadPhase()
                .GetTree( bodyType::Dynamic )
                .GetProxyAABB( GetProxyId( movedShape.proxyKey ) );

        const float expectedCenterX = 2.0f;
        const float expectedCenterY = -0.5f;

        assert(
            std::fabs(
                movedAABB.min.x -
                ( expectedCenterX - 0.25f )
            ) < epsilon
        );
        assert(
            std::fabs(
                movedAABB.max.y -
                ( expectedCenterY + 0.25f )
            ) < epsilon
        );
    }

    // Step 뒤에는 broadPhase pair와 Contact도 새 transform 기준으로 갱신됨.
    {
        world world{};
        world.SetGravity( {} );

        const bodyId staticBody =
            world.CreateBody(
                bodyType::Static,
                {
                    { 2.5f, 0.0f },
                    {}
                }
            );

        const shapeId staticShape =
            world.CreateShape(
                staticBody,
                circle2{ {}, 1.0f }
            );

        const bodyId dynamicBody =
            world.CreateBody( bodyType::Dynamic );

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

        assert( world.GetContactCount() == 0 );

        world.SetBodyLinearVelocity(
            dynamicBody,
            { 2.0f, 0.0f }
        );

        world.Step( 0.5f );

        assert( world.GetContactCount() == 1 );
        assert(
            world.GetBroadPhase().HasPair(
                PairKey( staticShape, dynamicShape )
            )
        );

        std::array<contactData, 1> contacts{};

        assert(
            world.GetBodyContactData(
                dynamicBody,
                contacts
            ) == 1
        );
        assert( contacts[0].manifold.pointCount > 0 );
    }

    // shape geometry는 body local space에 저장되고 proxy AABB는 world space로 계산됨.
    {
        world world{};

        const bodyId bodyId =
            world.CreateBody(
                bodyType::Dynamic,
                {
                    { 10.0f, 5.0f },
                    { 0.0f, 1.0f }
                }
            );

        const shapeId circleId =
            world.CreateShape(
                bodyId,
                circle2{ { 2.0f, 0.0f }, 1.0f }
            );

        assert( world.IsValid( circleId ) );
        assert( world.GetShapeCount() == 1 );

        const shape& circle = world.GetShape( circleId );
        const body& body = world.GetBody( bodyId );

        assert( circle.bodyId == Index( bodyId ) );
        assert( circle.generation == circleId.generation );
        assert( circle.proxyKey != shape::NULL_INDEX );
        assert( GetProxyType( circle.proxyKey ) == bodyType::Dynamic );
        assert( body.headShapeId == Index( circleId ) );
        assert( body.shapeCount == 1 );

        const aabb2& circleAABB =
            world.GetBroadPhase()
                .GetTree( bodyType::Dynamic )
                .GetProxyAABB( GetProxyId( circle.proxyKey ) );

        assert( std::fabs( circleAABB.min.x - 9.0f ) < epsilon );
        assert( std::fabs( circleAABB.min.y - 6.0f ) < epsilon );
        assert( std::fabs( circleAABB.max.x - 11.0f ) < epsilon );
        assert( std::fabs( circleAABB.max.y - 8.0f ) < epsilon );

        const shapeId segmentId =
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

        const shape& movedCircle = world.GetShape( circleId );
        const aabb2& movedCircleAABB =
            world.GetBroadPhase()
                .GetTree( bodyType::Dynamic )
                .GetProxyAABB( GetProxyId( movedCircle.proxyKey ) );

        assert( std::fabs( movedCircleAABB.min.x - 21.0f ) < epsilon );
        assert( std::fabs( movedCircleAABB.min.y + 4.0f ) < epsilon );
        assert( std::fabs( movedCircleAABB.max.x - 23.0f ) < epsilon );
        assert( std::fabs( movedCircleAABB.max.y + 2.0f ) < epsilon );

        assert( world.GetBroadPhase()
                    .GetTree( bodyType::Dynamic )
                    .Validate() );
    }

    // world collision pipeline과 persistent Contact.
    {
        world world{};

        const bodyId groundBody =
            world.CreateBody( bodyType::Static );

        const shapeId groundShape =
            world.CreateShape(
                groundBody,
                MakeBox( { 1.0f, 1.0f } )
            );

        const bodyId circleBody =
            world.CreateBody(
                bodyType::Dynamic,
                {
                    { 0.0f, 1.5f },
                    {}
                }
            );

        const shapeId circleShape =
            world.CreateShape(
                circleBody,
                circle2{ {}, 0.5f }
            );

        int touchingCount = 0;
        contactId contactHandle{};

        world.UpdateCollisions(
            [&]( const contactData& data )
            {
                ++touchingCount;
                contactHandle = data.id;

                assert( data.shapeA == groundShape );
                assert( data.shapeB == circleShape );
                assert( data.manifold.pointCount == 1 );
                assert( std::fabs( data.manifold.normal.x ) < epsilon );
                assert( std::fabs( data.manifold.normal.y - 1.0f ) < epsilon );
            }
        );

        assert( touchingCount == 1 );
        assert( world.GetContactCount() == 1 );
        assert( world.IsValid( contactHandle ) );
        assert( contactHandle.index1 == 1 );
        assert( contactHandle.generation == 1 );

        const contactData contactSnapshot =
            world.GetContactData( contactHandle );

        assert( contactSnapshot.id == contactHandle );
        assert( contactSnapshot.shapeA == groundShape );
        assert( contactSnapshot.shapeB == circleShape );
        assert( contactSnapshot.manifold.pointCount == 1 );

        assert( world.GetBroadPhase().HasPair(
            PairKey( groundShape, circleShape )
        ) );

        // 새 broadPhase 후보가 없어도 persistent Contact는 갱신됨.
        touchingCount = 0;

        world.UpdateCollisions(
            [&]( const contactData& data )
            {
                ++touchingCount;
                assert( data.id == contactHandle );
                assert( data.manifold.pointCount == 1 );
            }
        );

        assert( touchingCount == 1 );

        // AABB가 분리되면 Contact / pairSet / body contact list가 함께 정리됨.
        world.SetBodyTransform(
            circleBody,
            {
                { 0.0f, 5.0f },
                {}
            }
        );

        world.UpdateCollisions(
            []( const contactData& )
            {
            }
        );

        assert( world.GetContactCount() == 0 );
        assert( !world.IsValid( contactHandle ) );
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

        contactId reusedContactId{};

        world.UpdateCollisions(
            [&]( const contactData& data )
            {
                reusedContactId = data.id;
            }
        );

        assert( world.IsValid( reusedContactId ) );
        assert( reusedContactId.index1 == contactHandle.index1 );
        assert( reusedContactId.generation != contactHandle.generation );
        assert( !world.IsValid( contactHandle ) );
        const contactData reusedData =
            world.GetContactData( reusedContactId );

        assert( reusedData.id == reusedContactId );
        assert( reusedData.shapeA == groundShape );
        assert( reusedData.shapeB == circleShape );
    }

    // contactData manifold는 shape A local이 아니라 world space로 공개됨.
    {
        world world{};

        constexpr float halfPi = 1.57079632679f;

        const bodyId boxBody =
            world.CreateBody(
                bodyType::Static,
                {
                    { 10.0f, 5.0f },
                    rot2::FromRadians( halfPi )
                }
            );

        const shapeId boxShape =
            world.CreateShape(
                boxBody,
                MakeBox( { 1.0f, 1.0f } )
            );

        const bodyId circleBody =
            world.CreateBody(
                bodyType::Dynamic,
                {
                    { 10.0f, 6.5f },
                    {}
                }
            );

        const shapeId circleShape =
            world.CreateShape(
                circleBody,
                circle2{ {}, 0.5f }
            );

        contactData data{};
        int contactCount = 0;

        world.UpdateCollisions(
            [&]( const contactData& contactData )
            {
                data = contactData;
                ++contactCount;
            }
        );

        assert( contactCount == 1 );
        assert( data.shapeA == boxShape );
        assert( data.shapeB == circleShape );
        assert( data.manifold.pointCount == 1 );

        // A local +X normal / (1, 0) contact point가
        // 90도 회전 + (10, 5) 이동되어 world +Y / (10, 6)이 됨.
        assert( std::fabs( data.manifold.normal.x ) < epsilon );
        assert( std::fabs( data.manifold.normal.y - 1.0f ) < epsilon );
        assert( std::fabs( data.manifold.points[0].point.x - 10.0f ) < epsilon );
        assert( std::fabs( data.manifold.points[0].point.y - 6.0f ) < epsilon );

        const contactData snapshot =
            world.GetContactData( data.id );

        assert( snapshot.id == data.id );
        assert( snapshot.shapeA == data.shapeA );
        assert( snapshot.shapeB == data.shapeB );
        assert( std::fabs(
            snapshot.manifold.points[0].point.y -
            data.manifold.points[0].point.y
        ) < epsilon );
    }

    // AABB는 겹치지만 실제 geometry가 떨어져 있어도 Contact 자체는 유지됨.
    {
        world world{};

        const bodyId staticBody =
            world.CreateBody( bodyType::Static );

        const shapeId staticCircle =
            world.CreateShape(
                staticBody,
                circle2{ {}, 1.0f }
            );

        const bodyId dynamicBody =
            world.CreateBody(
                bodyType::Dynamic,
                {
                    { 1.5f, 1.5f },
                    {}
                }
            );

        const shapeId dynamicCircle =
            world.CreateShape(
                dynamicBody,
                circle2{ {}, 1.0f }
            );

        int touchingCount = 0;

        world.UpdateCollisions(
            [&]( const contactData& )
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

    // shape slot이 재사용되어도 오래된 shapeId는 generation mismatch로 무효가 됨.
    {
        world world{};

        const bodyId staticBody =
            world.CreateBody( bodyType::Static );

        const shapeId groundShape =
            world.CreateShape(
                staticBody,
                MakeBox( { 1.0f, 1.0f } )
            );

        const bodyId dynamicBody =
            world.CreateBody(
                bodyType::Dynamic,
                {
                    { 0.0f, 1.5f },
                    {}
                }
            );

        const shapeId oldShape =
            world.CreateShape(
                dynamicBody,
                circle2{ {}, 0.5f }
            );

        world.UpdateCollisions(
            []( const contactData& )
            {
            }
        );

        assert( world.GetContactCount() == 1 );

        world.DestroyShape( oldShape );

        assert( !world.IsValid( oldShape ) );
        assert( world.GetShapeCount() == 1 );
        assert( world.GetContactCount() == 0 );

        const shapeId newShape =
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
            []( const contactData& )
            {
            }
        );

        assert( world.GetContactCount() == 1 );
        assert( world.GetBroadPhase().HasPair(
            PairKey( groundShape, newShape )
        ) );
    }

    // body slot도 같은 index를 재사용하지만 generation으로 예전 handle을 차단함.
    {
        world world{};

        const bodyId groundBody =
            world.CreateBody( bodyType::Static );

        const shapeId groundShape =
            world.CreateShape(
                groundBody,
                MakeBox( { 1.0f, 1.0f } )
            );

        const bodyId oldBody =
            world.CreateBody(
                bodyType::Dynamic,
                {
                    { 0.0f, 1.5f },
                    {}
                }
            );

        const shapeId oldShape =
            world.CreateShape(
                oldBody,
                circle2{ {}, 0.5f }
            );

        world.UpdateCollisions(
            []( const contactData& )
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

        const bodyId newBody =
            world.CreateBody(
                bodyType::Dynamic,
                {
                    { 0.0f, 1.5f },
                    {}
                }
            );

        assert( newBody.index1 == oldBody.index1 );
        assert( newBody.generation != oldBody.generation );
        assert( world.IsValid( newBody ) );
        assert( !world.IsValid( oldBody ) );

        const shapeId newShape =
            world.CreateShape(
                newBody,
                circle2{ {}, 0.5f }
            );

        assert( newShape.index1 == oldShape.index1 );
        assert( newShape.generation != oldShape.generation );
        assert( world.IsValid( newShape ) );
        assert( !world.IsValid( oldShape ) );

        world.UpdateCollisions(
            []( const contactData& )
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

    // body / shape Contact query는 capacity와 실제 touching 수를 구분함.
    {
        world world{};

        const bodyId dynamicBody =
            world.CreateBody( bodyType::Dynamic );

        const shapeId touchingShape =
            world.CreateShape(
                dynamicBody,
                circle2{ {}, 1.0f }
            );

        const shapeId nonTouchingShape =
            world.CreateShape(
                dynamicBody,
                circle2{ { 10.0f, 0.0f }, 1.0f }
            );

        const bodyId touchingStaticBody =
            world.CreateBody(
                bodyType::Static,
                {
                    { 1.5f, 0.0f },
                    {}
                }
            );

        const shapeId touchingStaticShape =
            world.CreateShape(
                touchingStaticBody,
                circle2{ {}, 1.0f }
            );

        const bodyId overlapStaticBody =
            world.CreateBody(
                bodyType::Static,
                {
                    { 11.5f, 1.5f },
                    {}
                }
            );

        const shapeId overlapStaticShape =
            world.CreateShape(
                overlapStaticBody,
                circle2{ {}, 1.0f }
            );

        int touchingCallbackCount = 0;

        world.UpdateCollisions(
            [&]( const contactData& )
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

        std::array<contactData, 2> bodyContacts{};

        const std::size_t bodyContactCount =
            world.GetBodyContactData(
                dynamicBody,
                bodyContacts
            );

        assert( bodyContactCount == 1 );
        assert( bodyContacts[0].manifold.pointCount > 0 );

        const bool bodyHasTouchingPair =
            ( bodyContacts[0].shapeA == touchingShape &&
              bodyContacts[0].shapeB == touchingStaticShape ) ||
            ( bodyContacts[0].shapeA == touchingStaticShape &&
              bodyContacts[0].shapeB == touchingShape );

        assert( bodyHasTouchingPair );

        // shape capacity도 body contactCount를 그대로 사용하므로 보수적으로 2임.
        assert( world.GetShapeContactCapacity( touchingShape ) == 2 );
        assert( world.GetShapeContactCapacity( nonTouchingShape ) == 2 );

        std::array<contactData, 2> touchingContacts{};
        const std::size_t touchingCount =
            world.GetShapeContactData(
                touchingShape,
                touchingContacts
            );

        assert( touchingCount == 1 );
        assert(
            touchingContacts[0].shapeA == touchingShape ||
            touchingContacts[0].shapeB == touchingShape
        );

        std::array<contactData, 2> nonTouchingContacts{};
        const std::size_t nonTouchingCount =
            world.GetShapeContactData(
                nonTouchingShape,
                nonTouchingContacts
            );

        // AABB Contact는 존재하지만 manifold가 비어 있으므로 public query에서는 제외됨.
        assert( nonTouchingCount == 0 );

        // 정적 body/shape 쪽에서도 같은 touching Contact를 조회할 수 있음.
        assert( world.GetBodyContactCapacity( touchingStaticBody ) == 1 );
        assert( world.GetShapeContactCapacity( touchingStaticShape ) == 1 );

        std::array<contactData, 1> staticContacts{};
        const std::size_t staticCount =
            world.GetBodyContactData(
                touchingStaticBody,
                staticContacts
            );

        assert( staticCount == 1 );
        assert( staticContacts[0].id == bodyContacts[0].id );

        // 두 번째 정적 shape는 AABB overlap만 있으므로 capacity 1, 실제 반환 0.
        assert( world.GetBodyContactCapacity( overlapStaticBody ) == 1 );
        assert( world.GetShapeContactCapacity( overlapStaticShape ) == 1 );

        std::array<contactData, 1> overlapContacts{};
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
                std::span<contactData>{}
            ) == 0
        );
    }

    // Box2D처럼 segment-segment 조합은 broadPhase 후보여도 Contact를 만들지 않음.
    {
        world world{};

        const bodyId staticBody =
            world.CreateBody( bodyType::Static );

        (void)world.CreateShape(
            staticBody,
            segment2{ { -1.0f, 0.0f }, { 1.0f, 0.0f } }
        );

        const bodyId dynamicBody =
            world.CreateBody( bodyType::Dynamic );

        (void)world.CreateShape(
            dynamicBody,
            segment2{ { -1.0f, 0.0f }, { 1.0f, 0.0f } }
        );

        int contactCount = 0;

        world.UpdateCollisions(
            [&]( const contactData& )
            {
                ++contactCount;
            }
        );

        assert( contactCount == 0 );
        assert( world.GetContactCount() == 0 );
    }

    return 0;
}
