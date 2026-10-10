#pragma once

#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"
#include "dynamics/constraintSoftness2.h"
#include "dynamics/joints/prismaticJointSim2.h"

namespace zonai
{

// 한 substep 동안 Prismatic Joint를 반복해서 풀기 위한 임시 solver 데이터.
// 로컬 anchor/axis를 현재 COM/월드 기준으로 바꾸고 각 제약의 설정과 warm-start impulse를 모아 둠.
struct prismaticJointConstraint2
{
    // 결과 impulse를 원래 Joint slot에 저장하고 두 Body state를 찾기 위한 index.
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    // Step 시작 시 COM 기준 작용점, 두 COM 사이 거리, A의 월드 이동 축과 기준 상대 회전.
    // Body의 deltaPosition / deltaRotation과 결합해 현재 translation과 angle error를 계산함.
    vec2 anchorA{};
    vec2 anchorB{};
    vec2 deltaCenter{};
    vec2 axisA{ 1.0f, 0.0f };
    rot2 relativeRotation{};

    // Solver iteration에서 반복 사용하므로 Prepare에서 Body의 역질량 / 역관성을 복사함.
    float invMassA = 0.0f;
    float invMassB = 0.0f;
    float invInertiaA = 0.0f;
    float invInertiaB = 0.0f;

    // 자유축 수직 이동(x)과 상대 회전(y)을 함께 푸는 2x2 제약의 누적 impulse.
    vec2 impulse{};
    constraintSoftness2 softness{}; // hard constraint의 위치 안정화 계수.

    // 자유축 방향 이동 limit. lower/upper는 각각 한쪽 방향 impulse를 0 이상으로 누적함.
    bool enableLimit = false;
    float lowerTranslation = 0.0f;
    float upperTranslation = 0.0f;
    float invSubStepTime = 0.0f;
    float lowerImpulse = 0.0f;
    float upperImpulse = 0.0f;

    // 자유축 방향 속도 모터. maxMotorImpulse는 maxMotorForce * h임.
    bool enableMotor = false;
    float motorSpeed = 0.0f;
    float maxMotorImpulse = 0.0f;
    float motorImpulse = 0.0f;

    // 자유축 방향 목표 위치를 향한 실제 spring-damper 제약.
    // springSoftness는 hertz/dampingRatio를 이번 substep 계수로 변환한 값임.
    bool enableSpring = false;
    float hertz = 0.0f;
    float targetTranslation = 0.0f;
    float springImpulse = 0.0f;
    constraintSoftness2 springSoftness{};
};

// 원점 작용점을 COM 기준으로 바꾸고 A의 이동 축과 기준 상대 회전을 준비함.
[[nodiscard]] prismaticJointConstraint2 preparePrismaticJointConstraint( const prismaticJointSim2& joint, const bodySim& bodySimA, const bodySim& bodySimB, float subStepTime );

// 이전 substep의 각 누적 impulse를 두 Body에 먼저 적용해 반복 계산의 시작점으로 사용함.
void warmStartPrismaticJointConstraint( const prismaticJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB );

// 축 수직 이동과 상대 회전을 2x2 block으로 함께 풀고 축 방향은 spring/motor/limit이 켜진 경우에만 제어함.
void solvePrismaticJointConstraint( prismaticJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB, bool useBias );

} // namespace zonai
