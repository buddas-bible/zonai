#pragma once

#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"
#include "dynamics/constraintSoftness2.h"
#include "dynamics/joints/revoluteJointSim2.h"

namespace zonai
{

struct revoluteJointConstraint2
{
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    // Step 시작 시 질량 중심에서 작용점까지의 벡터.
    vec2 anchorA{};
    vec2 anchorB{};
    vec2 deltaCenter{};

    float invMassA = 0.0f;
    float invMassB = 0.0f;
    float invInertiaA = 0.0f;
    float invInertiaB = 0.0f;
    vec2 impulse{};
    constraintSoftness2 softness{};

    // 기준 각도를 뺀 Step 시작 시 상대 회전. 누적 회전을 곱한 뒤 atan2로 각도를 구함.
    rot2 relativeRotation{};
    bool enableLimit = false;
    float lowerAngle = 0.0f;
    float upperAngle = 0.0f;
    float angularMass = 0.0f; // 1 / (invInertiaA + invInertiaB)
    float invSubStepTime = 0.0f;
    float lowerImpulse = 0.0f;
    float upperImpulse = 0.0f;
    bool enableMotor = false;
    float motorSpeed = 0.0f;
    float maxMotorImpulse = 0.0f; // maxMotorTorque * subStepTime
    float motorImpulse = 0.0f;
};

// 원점 기준 작용점을 질량 중심 기준으로 바꾸고 수치 안정화 계수를 준비함.
[[nodiscard]] revoluteJointConstraint2 prepareRevoluteJointConstraint( const revoluteJointSim2& joint, const bodySim& bodySimA, const bodySim& bodySimB, float subStepTime );

// 이전 누적 임펄스를 두 작용점에 반대 방향으로 적용함.
void warmStartRevoluteJointConstraint( const revoluteJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB );

// 모터 → 각도 제한 → 연결점의 순서로 풂. 위반한 위치는 useBias pass에서만 보정함.
void solveRevoluteJointConstraint( revoluteJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB, bool useBias );

} // namespace zonai
