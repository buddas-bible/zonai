#pragma once

#include <cstdint>
#include "math/vec2.h"

namespace zonai
{

// World의 stable Joint 슬롯에 저장하는 Prismatic의 지속 상태와 이전 substep 해.
struct prismaticJointSim2
{
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;
    vec2 localAnchorA{};
    vec2 localAnchorB{};
    vec2 localAxisA{ 1.0f, 0.0f };
    float referenceAngle = 0.0f;
    vec2 impulse{}; // x: 축 수직 임펄스, y: 상대 회전 임펄스.
    float subStepTime = 0.0f;

    bool enableLimit = false;
    float lowerTranslation = 0.0f;
    float upperTranslation = 0.0f;
    float lowerImpulse = 0.0f;
    float upperImpulse = 0.0f;

    bool enableMotor = false;
    float motorSpeed = 0.0f;
    float maxMotorForce = 0.0f;
    float motorImpulse = 0.0f;
};

} // namespace zonai
