#pragma once

#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"
#include "dynamics/constraintSoftness2.h"
#include "dynamics/joints/weldJointSim2.h"

namespace zonai
{

// 한 substep 동안 Weld Joint를 반복해서 풀기 위한 임시 solver 데이터.
// weldJointSim2의 영구 설정과 Body 상태를 현재 solver가 바로 사용할 형태로 변환하며, Solve 후 누적 impulse만 Joint에 다시 저장함.
struct weldJointConstraint2
{
    // 결과 impulse를 원래 Joint slot에 저장하고 두 Body state를 찾기 위한 index.
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    // Step 시작 시 COM에서 각 작용점까지의 월드 lever arm과 두 COM 사이의 거리.
    // Body의 deltaPosition / deltaRotation과 결합해 현재 선형 Weld 오차를 계산함.
    vec2 anchorA{};
    vec2 anchorB{};
    vec2 deltaCenter{};

    // Solver iteration에서 Body를 다시 조회하지 않도록 Prepare에서 복사한 역질량 / 역관성.
    float invMassA = 0.0f;
    float invMassB = 0.0f;
    float invInertiaA = 0.0f;
    float invInertiaB = 0.0f;

    // referenceAngle을 제외한 Step 시작 시 상대 회전과 회전 제약의 scalar effective mass.
    // deltaRotation과 relativeRotation을 결합해 현재 angular error를 계산함.
    rot2 relativeRotation{};
    float angularMass = 0.0f;

    // 이번 solver iteration 동안 갱신되는 선형 / 회전 누적 impulse.
    // Prepare에서 이전 substep cache를 받아오고 Solve가 끝나면 weldJointSim2에 다시 저장함.
    vec2 impulse{};
    float angularImpulse = 0.0f;

    // 0 Hz hard Weld와 양수 Hz soft Weld를 구분하기 위해 원래 주파수를 유지함.
    float linearHertz = 0.0f;
    float angularHertz = 0.0f;

    // hard Weld의 위치 안정화 계수와 soft Weld의 실제 spring-damper 계수.
    constraintSoftness2 softness{};
    constraintSoftness2 linearSpring{};
    constraintSoftness2 angularSpring{};
};

// Body origin 기준 anchor를 COM 기준 lever arm으로 바꾸고 hard/soft Weld 계수를 준비함.
[[nodiscard]] weldJointConstraint2 prepareWeldJointConstraint( const weldJointSim2& joint, const bodySim& bodySimA, const bodySim& bodySimB, float subStepTime );

// 이전 substep의 선형/각도 누적 임펄스를 두 Body에 반대로 적용함.
void warmStartWeldJointConstraint( const weldJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB );

// 상대 각도 scalar 제약을 푼 뒤 두 anchor를 일치시키는 2x2 선형 제약을 풂.
void solveWeldJointConstraint( weldJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB, bool useBias );

} // namespace zonai
