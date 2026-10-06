#pragma once

#include "dynamics/id.h"
#include "math/vec2.h"

namespace zonai
{

// B를 A의 축 위에서 이동시키고 바퀴 회전은 허용하는 서스펜션 조인트.
struct wheelJointDef
{
    bodyId bodyA{};
    bodyId bodyB{};
    // 물체 원점 기준 작용점. 두 점이 같은 위치이면 스프링 변위가 0임.
    vec2 localAnchorA{};
    vec2 localAnchorB{};
    vec2 localAxisA{ 0.0f, 1.0f }; // A 기준 이동 방향. 유한한 단위 벡터이며 A와 함께 회전함.
    bool collideConnected = false;

    bool enableSpring = true;
    float hertz = 3.0f; // 축 방향 스프링 주파수 (Hz). 0은 스프링 힘만 끄며 제한은 유지함.
    float dampingRatio = 0.7f; // 무차원 감쇠 비율. 유한한 비음수.

    bool enableLimit = false;
    float lowerTranslation = 0.0f; // 부호 있는 축 방향 변위 (m). 유한하며 lower <= upper.
    float upperTranslation = 0.0f; // 두 값이 같으면 해당 변위를 유지함.
};

struct wheelJointData
{
    bodyId bodyA{};
    bodyId bodyB{};
    vec2 anchorA{}; // 월드 작용점. A는 스프링의 기준 위치임.
    vec2 anchorB{};
    vec2 axis{}; // 현재 월드 이동 축.
    float currentTranslation = 0.0f; // axis에 투영한 B-A 작용점 변위 (m).
    float lateralError = 0.0f; // axis의 왼쪽 수직 방향으로 벗어난 거리 (m).
    vec2 force{}; // 마지막 substep의 누적 임펄스 / h. B에 작용하는 전체 반력 (N).
    float springForce = 0.0f; // 축 방향 스프링 힘 (N). 양수는 axis 방향.
    float limitForce = 0.0f; // (lowerImpulse - upperImpulse) / h. 양수는 axis 방향.
    bool collideConnected = false;
    bool enableSpring = true;
    float hertz = 3.0f;
    float dampingRatio = 0.7f;
    bool enableLimit = false;
    float lowerTranslation = 0.0f;
    float upperTranslation = 0.0f;
};

} // namespace zonai
