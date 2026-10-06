#pragma once

#include <cstdint>
#include "math/vec2.h"

namespace zonai
{

// World 슬롯에 저장하는 원점 작용점·A의 로컬 축·이전 substep의 해.
struct wheelJointSim2
{
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;
    vec2 localAnchorA{};
    vec2 localAnchorB{};
    vec2 localAxisA{ 0.0f, 1.0f };
    float impulse = 0.0f; // 축 수직 방향의 누적 임펄스.
    float subStepTime = 0.0f;

    bool enableSpring = true;
    float hertz = 3.0f;
    float dampingRatio = 0.7f;
    float springImpulse = 0.0f;

    bool enableLimit = false;
    float lowerTranslation = 0.0f;
    float upperTranslation = 0.0f;
    float lowerImpulse = 0.0f;
    float upperImpulse = 0.0f;

    bool enableMotor = false;
    float motorSpeed = 0.0f;
    float maxMotorTorque = 0.0f;
    float motorImpulse = 0.0f;
};

} // namespace zonai
