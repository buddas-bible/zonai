#pragma once

#include "dynamics/id.h"
#include "math/vec2.h"

namespace zonai
{
#pragma region Definition

struct distanceJointDef
{
    bodyId bodyA{};
    bodyId bodyB{};

    // Body origin 기준 local anchor. COM 이동과 분리해서 보관함.
    vec2 localAnchorA{};
    vec2 localAnchorB{};
    float length = 1.0f;
    bool collideConnected = false;

    // false는 rigid이며 limit을 무시함. true + Hertz 0은 spring 힘만 끔.
    bool enableSpring = false;
    float hertz = 5.0f;
    float dampingRatio = 0.7f;

    // 거리의 하한과 상한. 각 limit의 임펄스는 별도로 누적함.
    bool enableLimit = false;
    // 유한한 비음수, min <= max. World에서 LINEAR_SLOP 이상으로 제한함.
    // 같은 제한값은 Box2D처럼 length의 rigid 제약으로 돌아감.
    float minLength = 0.0f;
    float maxLength = 1.0e5f;
};

#pragma endregion

#pragma region Data

struct distanceJointData
{
    bodyId bodyA{};
    bodyId bodyB{};

    // World 좌표의 작용점.
    vec2 anchorA{};
    vec2 anchorB{};
    float length = 0.0f;
    float currentLength = 0.0f;
    bool collideConnected = false;

    // Spring 설정. Rigid 제약의 수치 안정화 계수와 구분함.
    bool enableSpring = false;
    float hertz = 0.0f;
    float dampingRatio = 0.0f;
    // 마지막 substep의 합산 impulse/h. 음수는 tension, 양수는 compression임.
    float axialForce = 0.0f;

    // 거리의 하한과 상한. 각 limit의 임펄스는 별도로 누적함.
    bool enableLimit = false;
    float minLength = 0.0f;
    float maxLength = 0.0f;
};

#pragma endregion
} // namespace zonai
