#include <array>
#include <cstdio>
#include <cstdlib>
#include <source_location>
#include <cmath>

#include "dynamics/world.h"

using namespace zonai;

namespace
{

// Release에서도 solver 검사를 실행하고 실패한 위치를 출력함.
void check( bool condition, const std::source_location& location = std::source_location::current() )
{
    if( !condition )
    {
        std::fprintf( stderr, "%s:%u: solver check failed\n", location.file_name(), location.line() );
        std::exit( EXIT_FAILURE );
    }
}

} // namespace

int main()
{
    constexpr float timeStep = 1.0f / 60.0f;

    // 두 point가 동시에 닿는 얇은 box의 단순 반발을 확인함.
    {
        constexpr float impactSpeed = 5.0f;

        world world{};
        world.SetGravity( {} );

        const bodyId groundBody = world.CreateBody( bodyType::Static, { { 0.0f, -1.0f }, {} } );

        const shapeId groundShape = world.CreateShape( groundBody, MakeBox( { 40.0f, 1.0f } ) );

        world.SetShapeFriction( groundShape, 0.0f );
        world.SetShapeRestitution( groundShape, 0.0f );

        bodyDef boxDefinition{};
        boxDefinition.type = bodyType::Dynamic;
        boxDefinition.transform.position = { 0.0f, 0.25f + 0.5f * impactSpeed * timeStep };
        boxDefinition.linearVelocity = { 0.0f, -impactSpeed };
        boxDefinition.enableSleep = false;

        const bodyId boxBody = world.CreateBody( boxDefinition );

        const shapeId boxShape = world.CreateShape( boxBody, MakeBox( { 1.0f, 0.25f } ) );

        world.SetShapeFriction( boxShape, 0.0f );
        world.SetShapeRestitution( boxShape, 1.0f );

        // 첫 Step 시작 시 아직 contact manifold가 없을 수 있으므로
        // 실제 impact 이후까지 충분히 진행함.
        for( int i = 0; i < 60; ++i )
        {
            world.Step( timeStep, 4 );
        }

        const float bounceSpeed = world.GetBodyLinearVelocity( boxBody ).y;

        const float spin = std::fabs( world.GetBodyAngularVelocity( boxBody ) );

        check( bounceSpeed > 4.0f );
        check( spin < 0.5f );
    }

    // 완전탄성 정사각 box를 수평 낙하시킬 때 두 contact point의 순차 restitution이
    // 에너지를 추가하거나 큰 잔여 회전을 만들면 안 됨.
    {
        constexpr float dropHeight = 10.0f;

        world world{};

        const bodyId groundBody = world.CreateBody( bodyType::Static );

        const shapeId groundShape = world.CreateShape( groundBody, segment2{ { -20.0f, 0.0f }, { 20.0f, 0.0f } } );

        world.SetShapeFriction( groundShape, 0.0f );
        world.SetShapeRestitution( groundShape, 0.0f );

        bodyDef boxDefinition{};
        boxDefinition.type = bodyType::Dynamic;
        boxDefinition.transform.position = { 0.0f, dropHeight };
        boxDefinition.safetyFactor = 0.01f;
        boxDefinition.enableSleep = false;

        const bodyId boxBody = world.CreateBody( boxDefinition );

        const shapeId boxShape = world.CreateShape( boxBody, MakeBox( { 0.5f, 0.5f } ) );

        world.SetShapeFriction( boxShape, 0.0f );
        world.SetShapeRestitution( boxShape, 1.0f );

        float firstBounceSpin = 0.0f;
        float firstApex = 0.0f;
        float previousSpeed = 0.0f;
        bool bounced = false;
        bool reachedApex = false;

        for( int i = 0; i < 600 && !reachedApex; ++i )
        {
            world.Step( timeStep, 4 );

            const float speed = world.GetBodyLinearVelocity( boxBody ).y;

            if( !bounced && previousSpeed <= 0.0f && speed > 0.0f )
            {
                firstBounceSpin = std::fabs( world.GetBodyAngularVelocity( boxBody ) );
                bounced = true;
            }

            if( bounced && previousSpeed > 0.0f && speed <= 0.0f )
            {
                firstApex = world.GetBodyTransform( boxBody ).position.y;
                reachedApex = true;
            }

            previousSpeed = speed;
        }

        check( bounced );
        check( reachedApex );

        // 완전탄성 충돌이므로 첫 apex가 시작 높이보다 높아지는 것은 solver가 만든 에너지임.
        check( firstApex <= 1.001f * dropHeight );

        // 대칭 수평 착지의 restitution 반복은 큰 잔여 회전을 남기면 안 됨.
        check( firstBounceSpin < 0.5f );
    }

    // 현재 solver의 4 sub-step에서 기존 contact sweep 수를 유지할 때
    // 여러 body가 연결된 stack이 무너지거나 크게 압축되면 안 됨.
    {
        constexpr int boxCount = 12;

        world world{};

        const bodyId groundBody = world.CreateBody( bodyType::Static );

        ( void )world.CreateShape( groundBody, segment2{ { -20.0f, 0.0f }, { 20.0f, 0.0f } } );

        std::array<bodyId, boxCount> boxes{};

        for( int i = 0; i < boxCount; ++i )
        {
            bodyDef definition{};
            definition.type = bodyType::Dynamic;
            definition.transform.position = { 0.0f, 0.5f + 1.01f * static_cast<float>( i ) };

            boxes[i] = world.CreateBody( definition );

            ( void )world.CreateShape( boxes[i], MakeBox( { 0.5f, 0.5f } ) );
        }

        for( int i = 0; i < 600; ++i )
        {
            world.Step( timeStep, 4 );
        }

        for( int i = 0; i < boxCount; ++i )
        {
            const transform2 transform = world.GetBodyTransform( boxes[i] );

            const float expectedHeight = 0.5f + static_cast<float>( i );

            check( std::fabs( transform.position.x ) < 0.2f );
            check( std::fabs( transform.position.y - expectedHeight ) < 0.25f );
        }
    }

    return 0;
}
