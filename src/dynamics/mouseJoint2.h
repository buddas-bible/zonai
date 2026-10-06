#pragma once

#include "dynamics/id.h"
#include "math/vec2.h"

namespace zonai
{
// A는 수명과 연결 관계를 유지하는 정적 물체이며, target은 월드 좌표임.
// B의 클릭 위치는 물체 원점 기준으로 저장하고 질량 중심 기준으로 계산함.
#pragma region Definition

struct mouseJointDef
{
    bodyId bodyA{};
    bodyId bodyB{};
    vec2 target{};
    float hertz = 5.0f; // 스프링 주파수 (Hz)
    float dampingRatio = 0.7f; // 감쇠비. 1이면 임계 감쇠
    float maxForce = 1000.0f; // 허용하는 힘의 최대 크기
};

#pragma endregion Definition

#pragma region Data

struct mouseJointData
{
    bodyId bodyA{};
    bodyId bodyB{};
    vec2 target{};
    vec2 anchorB{};
    vec2 force{};
    float hertz = 0.0f; // 스프링 주파수 (Hz)
    float dampingRatio = 0.0f; // 감쇠비. 1이면 임계 감쇠
    float maxForce = 0.0f; // 허용하는 힘의 최대 크기
};

#pragma endregion Data
} // namespace zonai
