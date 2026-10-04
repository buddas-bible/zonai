#include <cassert>
#include <cmath>
#include <cstdio>

#include "dynamics/world.h"

using namespace zonai;

int main()
{
    constexpr float timeStep = 1.0f / 60.0f;
    constexpr float impactSpeed = 5.0f;

    world world{};
    world.SetGravity( {} );

    const bodyId groundBody =
        world.CreateBody(
            bodyType::Static,
            {
                { 0.0f, -1.0f },
                {}
            }
        );

    const shapeId groundShape =
        world.CreateShape(
            groundBody,
            MakeBox( { 40.0f, 1.0f } )
        );

    world.SetShapeFriction( groundShape, 0.0f );
    world.SetShapeRestitution( groundShape, 0.0f );

    bodyDef boxDefinition{};
    boxDefinition.type = bodyType::Dynamic;
    boxDefinition.transform.position =
        { 0.0f, 0.25f + 0.5f * impactSpeed * timeStep };
    boxDefinition.linearVelocity =
        { 0.0f, -impactSpeed };
    boxDefinition.enableSleep = false;

    const bodyId boxBody =
        world.CreateBody( boxDefinition );

    const shapeId boxShape =
        world.CreateShape(
            boxBody,
            MakeBox( { 1.0f, 0.25f } )
        );

    world.SetShapeFriction( boxShape, 0.0f );
    world.SetShapeRestitution( boxShape, 1.0f );

    // 첫 Step 시작 시 아직 contact manifold가 없을 수 있으므로
    // Box2D restitution 회귀 테스트처럼 실제 impact 이후까지 충분히 진행함.
    for( int i = 0; i < 60; ++i )
    {
        world.Step( timeStep, 4 );
    }

    const float bounceSpeed =
        world.GetBodyLinearVelocity( boxBody ).y;

    const float spin =
        std::fabs(
            world.GetBodyAngularVelocity( boxBody )
        );

    std::fprintf(
        stderr,
        "flat restitution: bounce=%.6f spin=%.6f\n",
        bounceSpeed,
        spin
    );

    // 두 contact point의 restitution은 서로 velocity를 바꾸므로 한 번의 sweep으로 끝내면
    // 첫 point가 만든 회전을 두 번째 point가 완전히 상쇄하지 못함.
    // 최신 Box2D처럼 restitution stage를 반복하면 대칭 충돌의 잔여 spin이 빠르게 줄어듦.
    assert( bounceSpeed > 4.0f );
    assert( spin < 0.5f );

    return 0;
}
