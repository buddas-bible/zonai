#pragma once

#include "dynamics/id.h"
#include "math/vec2.h"

namespace zonai
{
// Anchor는 COM이 아니라 Body origin 기준 local 좌표임.
struct distanceJointDef
{
    bodyId bodyA{};
    bodyId bodyB{};
    vec2 localAnchorA{};
    vec2 localAnchorB{};
    float length = 1.0f;
    bool collideConnected = false;
    // false는 rigid이며 limit을 무시함. true + Hertz 0은 spring 힘만 끔.
    bool enableSpring = false;
    float hertz = 5.0f;
    float dampingRatio = 0.7f;
    bool enableLimit = false;
    // 유한한 비음수, min <= max. World에서 LINEAR_SLOP 이상으로 제한함.
    // 같은 제한값은 Box2D처럼 length의 rigid 제약으로 돌아감.
    float minLength = 0.0f;
    float maxLength = 1.0e5f;
};

struct distanceJointData
{
    bodyId bodyA{};
    bodyId bodyB{};
    vec2 anchorA{};
    vec2 anchorB{};
    float length = 0.0f;
    float currentLength = 0.0f;
    bool collideConnected = false;
    bool enableSpring = false;
    float hertz = 0.0f;
    float dampingRatio = 0.0f;
    // 마지막 substep의 합산 impulse/h. 음수는 tension, 양수는 compression임.
    float axialForce = 0.0f;
    bool enableLimit = false;
    float minLength = 0.0f;
    float maxLength = 0.0f;
};
} // namespace zonai
