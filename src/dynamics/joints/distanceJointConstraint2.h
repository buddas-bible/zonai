#pragma once

#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"
#include "dynamics/constraintSoftness2.h"
#include "dynamics/joints/distanceJointSim2.h"

namespace zonai
{

// 한 substep 동안 Distance Joint를 반복해서 풀기 위한 임시 solver 데이터.
// distanceJointSim2와 Body 상태를 계산하기 좋은 형태로 미리 변환하며, Solve가 끝나면 누적 impulse만 Joint에 다시 저장함.
struct distanceJointConstraint2
{
    // 결과 impulse를 원래 Joint slot에 저장하고 두 Body state를 찾기 위한 index.
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    // Step 시작 시 COM에서 각 작용점까지의 월드 방향 lever arm과 두 COM 사이의 거리.
    // deltaPosition / deltaRotation을 더해 현재 두 작용점의 상대 위치를 계산함.
    vec2 anchorA{};
    vec2 anchorB{};
    vec2 deltaCenter{};

    // Solver iteration마다 Body를 다시 조회하지 않도록 Prepare에서 복사한 역질량 / 역관성.
    float invMassA = 0.0f;
    float invMassB = 0.0f;
    float invInertiaA = 0.0f;
    float invInertiaB = 0.0f;

    float length = 1.0f;    // 고정 거리 또는 스프링의 목표 거리.
    float axialMass = 0.0f; // 현재 두 작용점의 축 방향 effective mass.

    // 이번 solver iteration 동안 갱신되는 기본 거리 제약의 누적 impulse.
    float impulse = 0.0f;
    constraintSoftness2 softness{}; // hard correction 또는 물리 spring-damper 계수.

    // 물리 스프링 설정. hertz는 hard 제약과 실제 spring response를 구분하는 데 사용함.
    bool enableSpring = false;
    float hertz = 0.0f;

    // 거리 limit 설정과 각 한쪽 제약의 누적 impulse. lower/upper impulse는 0 이상만 허용함.
    bool enableLimit = false;
    float minLength = 0.0f;
    float maxLength = 0.0f;
    float lowerImpulse = 0.0f;
    float upperImpulse = 0.0f;
    float invSubStepTime = 0.0f; // speculative limit bias에서 거리 오차를 속도로 바꿀 때 사용함.
    constraintSoftness2 limitSoftness{};

    // 속도 모터는 목표 상대속도를 만들되 한 substep의 impulse를 maxMotorForce * h로 제한함.
    bool enableMotor = false;
    float motorSpeed = 0.0f;
    float motorImpulse = 0.0f;
    float maxMotorImpulse = 0.0f;
};

// 저장된 조인트를 이번 step의 작용점, 유효 질량, softness 계수로 변환함.
[[nodiscard]] distanceJointConstraint2 prepareDistanceJointConstraint( const distanceJointSim2& joint, const bodySim& bodySimA, const bodySim& bodySimB, float subStepTime );

// 이전 step의 누적 임펄스를 먼저 적용해서 반복 계산의 초기값으로 사용함.
void warmStartDistanceJointConstraint( const distanceJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB );

// 물리 스프링은 두 pass에서 bias를 유지하며, 고정 거리와 거리 제한은 useBias에 따라 보정함.
void solveDistanceJointConstraint( distanceJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB, bool useBias );

} // namespace zonai
