#pragma once

#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"
#include "dynamics/constraintSoftness2.h"
#include "dynamics/joints/prismaticJointSim2.h"

namespace zonai
{

struct prismaticJointConstraint2
{
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    // Step 시작 시 질량 중심 기준 작용점과 A의 월드 이동 축. 현재 누적 회전으로 갱신함.
    vec2 anchorA{};
    vec2 anchorB{};
    vec2 deltaCenter{};
    vec2 axisA{ 1.0f, 0.0f };
    rot2 relativeRotation{};

    float invMassA = 0.0f;
    float invMassB = 0.0f;
    float invInertiaA = 0.0f;
    float invInertiaB = 0.0f;
    vec2 impulse{}; // x: 축 수직, y: 상대 회전.
    constraintSoftness2 softness{};

    bool enableLimit = false;
    float lowerTranslation = 0.0f;
    float upperTranslation = 0.0f;
    float invSubStepTime = 0.0f;
    float lowerImpulse = 0.0f;
    float upperImpulse = 0.0f;

    bool enableMotor = false;
    float motorSpeed = 0.0f;
    float maxMotorImpulse = 0.0f;
    float motorImpulse = 0.0f;

    bool enableSpring = false;
    float hertz = 0.0f;
    float targetTranslation = 0.0f;
    float springImpulse = 0.0f;
    constraintSoftness2 springSoftness{};
};

// 원점 작용점을 COM 기준으로 바꾸고 A의 이동 축과 기준 상대 회전을 준비함.
[[nodiscard]] prismaticJointConstraint2 preparePrismaticJointConstraint( const prismaticJointSim2& joint, const bodySim& bodySimA, const bodySim& bodySimB, float subStepTime );
void warmStartPrismaticJointConstraint( const prismaticJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB );
// 축 수직 이동과 상대 회전을 2x2 block으로 함께 풀고 축 방향은 spring/motor/limit이 켜진 경우에만 제어함.
void solvePrismaticJointConstraint( prismaticJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB, bool useBias );

} // namespace zonai
