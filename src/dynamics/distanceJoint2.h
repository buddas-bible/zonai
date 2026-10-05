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
};
} // namespace zonai
