#pragma once

#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"
#include "dynamics/constraintSoftness2.h"
#include "dynamics/joints/revoluteJointSim2.h"

namespace zonai
{

// 한 substep 동안 Revolute Joint를 반복해서 풀기 위한 임시 solver 데이터.
// 영구 설정을 현재 COM/회전 기준으로 변환하고 effective mass와 warm-start impulse를 준비함.
struct revoluteJointConstraint2
{
    // 결과를 원래 Joint에 저장하고 두 Body state를 찾기 위한 index.
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    // Step 시작 시 COM에서 회전축 작용점까지의 월드 lever arm과 두 COM 사이의 거리.
    vec2 anchorA{};
    vec2 anchorB{};
    vec2 deltaCenter{};

    // Solver iteration에서 반복 사용하므로 Prepare에서 Body의 역질량 / 역관성을 복사함.
    float invMassA = 0.0f;
    float invMassB = 0.0f;
    float invInertiaA = 0.0f;
    float invInertiaB = 0.0f;

    // 두 작용점을 같은 위치에 두는 선형 제약의 누적 impulse와 hard correction 계수.
    vec2 impulse{};
    constraintSoftness2 softness{};

    // referenceAngle을 제외한 Step 시작 시 상대 회전.
    // Body의 deltaRotation과 결합해 현재 angular error를 계산함.
    rot2 relativeRotation{};

    // 각도 limit은 scalar 제약이며 lower/upper를 각각 한쪽 방향 impulse로 누적함.
    bool enableLimit = false;
    float lowerAngle = 0.0f;
    float upperAngle = 0.0f;
    float angularMass = 0.0f; // 1 / (invInertiaA + invInertiaB).
    float invSubStepTime = 0.0f;
    float lowerImpulse = 0.0f;
    float upperImpulse = 0.0f;

    // 모터는 상대 각속도를 motorSpeed로 만들며 maxMotorTorque * h 범위에서만 impulse를 누적함.
    bool enableMotor = false;
    float motorSpeed = 0.0f;
    float maxMotorImpulse = 0.0f;
    float motorImpulse = 0.0f;
};

// 원점 기준 작용점을 질량 중심 기준으로 바꾸고 수치 안정화 계수를 준비함.
[[nodiscard]] revoluteJointConstraint2 prepareRevoluteJointConstraint( const revoluteJointSim2& joint, const bodySim& bodySimA, const bodySim& bodySimB, float subStepTime );

// 이전 누적 임펄스를 두 작용점에 반대 방향으로 적용함.
void warmStartRevoluteJointConstraint( const revoluteJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB );

// 모터 → 각도 제한 → 연결점의 순서로 풂. 위반한 위치는 useBias pass에서만 보정함.
void solveRevoluteJointConstraint( revoluteJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB, bool useBias );

} // namespace zonai
