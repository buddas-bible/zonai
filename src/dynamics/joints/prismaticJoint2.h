#pragma once

#include "dynamics/id.h"
#include "math/vec2.h"

namespace zonai
{

// A의 로컬 축을 따라 B의 이동만 허용하고 축 수직 이동과 상대 회전을 막는 슬라이더 조인트.
struct prismaticJointDef
{
    bodyId bodyA{};
    bodyId bodyB{};

    // Body 원점 기준 작용점. Solver에서 질량 중심 기준 팔 길이로 변환함.
    vec2 localAnchorA{};
    vec2 localAnchorB{};
    vec2 localAxisA{ 1.0f, 0.0f }; // A와 함께 회전하는 유한한 단위 이동 축.
    float referenceAngle = 0.0f; // B-A의 기준 상대 각도 (rad).

    bool enableLimit = false;
    float lowerTranslation = 0.0f;
    float upperTranslation = 0.0f;

    bool enableMotor = false;
    float motorSpeed = 0.0f;
    float maxMotorForce = 0.0f;
    bool collideConnected = false;
};

struct prismaticJointData
{
    bodyId bodyA{};
    bodyId bodyB{};

    vec2 anchorA{};
    vec2 anchorB{};
    vec2 axis{};
    float currentTranslation = 0.0f;
    float lateralError = 0.0f;
    float currentAngle = 0.0f;
    vec2 force{}; // 마지막 substep의 축 수직 + motor + translation limit 누적 선형 임펄스 / h.
    float torque = 0.0f; // 마지막 substep의 상대 회전 임펄스 / h.
    bool enableLimit = false;
    float lowerTranslation = 0.0f;
    float upperTranslation = 0.0f;
    bool enableMotor = false;
    float motorSpeed = 0.0f;
    float maxMotorForce = 0.0f;
    float motorForce = 0.0f;
    bool collideConnected = false;
};

} // namespace zonai
