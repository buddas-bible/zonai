#pragma once

#include <cstdint>
#include "math/vec2.h"

namespace zonai
{

// World의 stable Joint slot에 계속 보관되는 Prismatic Joint의 simulation 상태.
// 로컬 기준 관계와 spring/limit/motor 설정, 다음 substep warm start에 필요한 impulse를 유지함.
struct prismaticJointSim2
{
    // 공용 Joint slot과 연결된 id, 그리고 두 Body의 simulation index.
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    // Body origin 기준 작용점과 A의 로컬 이동 축. Prepare에서 현재 COM/월드 기준으로 변환함.
    vec2 localAnchorA{};
    vec2 localAnchorB{};
    vec2 localAxisA{ 1.0f, 0.0f };
    float referenceAngle = 0.0f; // B가 A에 대해 유지할 기준 상대 각도.

    // 자유축에 수직인 이동과 상대 회전을 함께 푸는 2x2 제약의 warm-start cache.
    // x는 축 수직 선형 impulse, y는 상대 회전 impulse임.
    vec2 impulse{};
    float subStepTime = 0.0f;

    // 자유축 방향 이동 limit과 각 한쪽 제약의 warm-start cache.
    bool enableLimit = false;
    float lowerTranslation = 0.0f;
    float upperTranslation = 0.0f;
    float lowerImpulse = 0.0f;
    float upperImpulse = 0.0f;

    // 자유축 방향 목표속도를 만드는 모터 설정과 warm-start cache.
    bool enableMotor = false;
    float motorSpeed = 0.0f;
    float maxMotorForce = 0.0f;
    float motorImpulse = 0.0f;

    // 자유축 방향 목표 위치를 향한 실제 spring-damper 설정과 warm-start cache.
    bool enableSpring = false;
    float hertz = 0.0f;
    float dampingRatio = 0.7f;
    float targetTranslation = 0.0f;
    float springImpulse = 0.0f;
};

} // namespace zonai
