#pragma once

#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"
#include "dynamics/constraintSoftness2.h"
#include "dynamics/distanceJointSim2.h"

namespace zonai
{

struct distanceJointConstraint2
{
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    // Step 시작 시 질량 중심에서 작용점까지의 벡터. 누적 회전으로 현재 작용점을 구함.
    vec2 anchorA{};
    vec2 anchorB{};
    vec2 deltaCenter{};
    float invMassA = 0.0f;
    float invMassB = 0.0f;
    float invInertiaA = 0.0f;
    float invInertiaB = 0.0f;
    float length = 1.0f;    // 고정 거리 또는 스프링의 목표 거리
    float axialMass = 0.0f; // 축 방향의 유효 질량
    float impulse = 0.0f;   // 스프링 또는 고정 거리의 누적 임펄스
    constraintSoftness2 softness{};

    // 물리 스프링 설정. 고정 거리 제약의 수치 안정화 계수와 구분함.
    bool enableSpring = false;
    float hertz = 0.0f; // 스프링 주파수 (Hz)

    // 거리의 하한과 상한. 각 limit의 임펄스는 별도로 누적함.
    bool enableLimit = false;
    float minLength = 0.0f;
    float maxLength = 0.0f;
    float lowerImpulse = 0.0f; // 하한의 누적 임펄스. 0 이상
    float upperImpulse = 0.0f; // 상한의 누적 임펄스. 0 이상
    float invSubStepTime = 0.0f;
    constraintSoftness2 limitSoftness{};

    // 축 방향 속도 모터. 스프링 모드에서만 작동하며 0 Hz에서도 사용할 수 있음.
    bool enableMotor = false;
    float motorSpeed = 0.0f;   // 목표 상대속도 (m/s). 양수는 늘이고 음수는 줄임
    float motorImpulse = 0.0f; // 양방향 누적 임펄스. 절댓값을 F * h로 제한함
    float maxMotorImpulse = 0.0f;
};

// 저장된 조인트를 이번 step의 작용점, 유효 질량, softness 계수로 변환함.
[[nodiscard]] distanceJointConstraint2 prepareDistanceJointConstraint( const distanceJointSim2& joint, const bodySim& bodySimA, const bodySim& bodySimB, float subStepTime );

// 이전 step의 누적 임펄스를 먼저 적용해서 반복 계산의 초기값으로 사용함.
void warmStartDistanceJointConstraint( const distanceJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB );

// 물리 스프링은 두 pass에서 bias를 유지하며, 고정 거리와 거리 제한은 useBias에 따라 보정함.
void solveDistanceJointConstraint( distanceJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB, bool useBias );

} // namespace zonai
