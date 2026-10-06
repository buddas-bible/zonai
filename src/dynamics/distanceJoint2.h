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
    // false는 기존 rigid 거리 제약. true + Hertz 0은 spring 축을 자유롭게 둠.
    bool enableSpring = false;
    float hertz = 5.0f;
    float dampingRatio = 0.7f;
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
    // 마지막 substep의 impulse/h. 음수는 tension, 양수는 compression임.
    float axialForce = 0.0f;
};
} // namespace zonai
