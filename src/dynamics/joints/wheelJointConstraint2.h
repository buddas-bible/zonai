#pragma once

#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"
#include "dynamics/constraintSoftness2.h"
#include "dynamics/joints/wheelJointSim2.h"

namespace zonai
{

// 한 substep 동안 Wheel Joint를 반복해서 풀기 위한 임시 solver 데이터.
// 로컬 anchor/axis를 현재 COM/월드 기준으로 바꾸고 각 제약의 effective mass와 warm-start impulse를 준비함.
struct wheelJointConstraint2
{
    // 결과 impulse를 원래 Joint slot에 저장하고 두 Body state를 찾기 위한 index.
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    // Step 시작 시 COM 기준 작용점, 두 COM 사이 거리와 A의 월드 서스펜션 축.
    vec2 anchorA{};
    vec2 anchorB{};
    vec2 deltaCenter{};
    vec2 axisA{};

    // Solver iteration에서 반복 사용하므로 Prepare에서 Body의 역질량 / 역관성을 복사함.
    float invMassA = 0.0f;
    float invMassB = 0.0f;
    float invInertiaA = 0.0f;
    float invInertiaB = 0.0f;

    // 축 수직 방향과 축 방향을 각각 풀기 위한 scalar effective mass.
    float perpendicularMass = 0.0f;
    float axialMass = 0.0f;

    // 축 수직 이동을 막는 기본 제약의 누적 impulse와 hard correction 계수.
    float impulse = 0.0f;
    constraintSoftness2 softness{};

    // 축 방향의 실제 물리 spring-damper 제약.
    bool enableSpring = false;
    float springImpulse = 0.0f;
    constraintSoftness2 springSoftness{};

    // 축 방향 이동 limit. lower/upper는 각각 한쪽 방향 impulse를 0 이상으로 누적함.
    bool enableLimit = false;
    float lowerTranslation = 0.0f;
    float upperTranslation = 0.0f;
    float lowerImpulse = 0.0f;
    float upperImpulse = 0.0f;
    float invSubStepTime = 0.0f;

    // 상대 회전을 제어하는 모터. motorMass는 invInertiaA + invInertiaB의 역수임.
    bool enableMotor = false;
    float motorSpeed = 0.0f;
    float motorMass = 0.0f;
    float maxMotorImpulse = 0.0f; // maxMotorTorque * h.
    float motorImpulse = 0.0f;
};

// 원점 작용점을 질량 중심 기준으로 바꾸고 회전하는 축의 유효 질량을 준비함.
[[nodiscard]] wheelJointConstraint2 prepareWheelJointConstraint( const wheelJointSim2& joint, const bodySim& bodySimA, const bodySim& bodySimB, float subStepTime );

// 이전 substep의 각 누적 impulse를 두 Body에 먼저 적용해 반복 계산의 시작점으로 사용함.
void warmStartWheelJointConstraint( const wheelJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB );

// 모터 → 스프링 → 하한 → 상한 → 축 수직 제약. 물리 스프링은 relaxation에서도 복원 bias를 유지함.
void solveWheelJointConstraint( wheelJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB, bool useBias );

} // namespace zonai
