#pragma once

#include "dynamics/id.h"
#include "math/vec2.h"

namespace zonai
{
// A는 수명/graph 연결을 위한 Static Body, target은 world 좌표임.
// B의 클릭 위치를 origin 기준 local anchor로 저장하고 COM 기준으로 풀어냄.
struct mouseJointDef
{
    bodyId bodyA{};
    bodyId bodyB{};
    vec2 target{};
    float hertz = 5.0f;
    float dampingRatio = 0.7f;
    float maxForce = 1000.0f;
};

struct mouseJointData
{
    bodyId bodyA{};
    bodyId bodyB{};
    vec2 target{};
    vec2 anchorB{};
    vec2 force{};
    float hertz = 0.0f;
    float dampingRatio = 0.0f;
    float maxForce = 0.0f;
};
} // namespace zonai
