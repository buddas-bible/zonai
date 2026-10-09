#pragma once

#include <cstdint>
#include "math/vec2.h"

namespace zonai
{

// World의 stable Joint slot에 계속 보관되는 Wheel Joint의 simulation 상태.
// 서스펜션의 로컬 기준과 spring/limit/motor 설정, 다음 substep warm start에 필요한 impulse를 유지함.
struct wheelJointSim2
{
    // 공용 Joint slot과 연결된 id, 그리고 두 Body의 simulation index.
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    // Body origin 기준 작용점과 A의 로컬 서스펜션 축. Prepare에서 COM/월드 기준으로 변환함.
    vec2 localAnchorA{};
    vec2 localAnchorB{};
    vec2 localAxisA{ 0.0f, 1.0f };

    // 서스펜션 축에서 옆으로 벗어나는 움직임을 막는 제약의 warm-start cache.
    float impulse = 0.0f;
    float subStepTime = 0.0f;

    // 축 방향 물리 스프링 설정과 warm-start cache.
    bool enableSpring = true;
    float hertz = 3.0f;
    float dampingRatio = 0.7f;
    float springImpulse = 0.0f;

    // 축 방향 이동 limit과 각 한쪽 제약의 warm-start cache.
    bool enableLimit = false;
    float lowerTranslation = 0.0f;
    float upperTranslation = 0.0f;
    float lowerImpulse = 0.0f;
    float upperImpulse = 0.0f;

    // 두 Body의 상대 회전 속도를 제어하는 모터 설정과 warm-start cache.
    bool enableMotor = false;
    float motorSpeed = 0.0f;
    float maxMotorTorque = 0.0f;
    float motorImpulse = 0.0f;
};

} // namespace zonai
