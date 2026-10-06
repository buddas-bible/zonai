#pragma once

#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"
#include "dynamics/constraintSoftness2.h"
#include "dynamics/distanceJointSim2.h"

namespace zonai
{
#pragma region Constraint

struct distanceJointConstraint2
{
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    // Step 시작 시 COM 기준 lever arm. Delta rotation으로 현재 작용점을 구함.
    vec2 anchorA{};
    vec2 anchorB{};
    vec2 deltaCenter{};
    float invMassA = 0.0f;
    float invMassB = 0.0f;
    float invInertiaA = 0.0f;
    float invInertiaB = 0.0f;
    float length = 1.0f;
    float axialMass = 0.0f;
    float impulse = 0.0f;
    constraintSoftness2 softness{};

    // Spring 설정. Rigid 제약의 수치 안정화 계수와 구분함.
    bool enableSpring = false;
    float hertz = 0.0f;

    // 거리의 하한과 상한. 각 limit의 임펄스는 별도로 누적함.
    bool enableLimit = false;
    float minLength = 0.0f;
    float maxLength = 0.0f;
    float lowerImpulse = 0.0f;
    float upperImpulse = 0.0f;
    float invSubStepTime = 0.0f;
    constraintSoftness2 limitSoftness{};
};

#pragma endregion

#pragma region Solver

// Persistent joint를 이번 step의 anchor, 유효 질량, softness로 변환함.
[[nodiscard]] distanceJointConstraint2 prepareDistanceJointConstraint( const distanceJointSim2& joint, const bodySim& bodyA, const bodySim& bodyB, float subStepTime );

// 이전 step의 누적 임펄스를 먼저 적용해서 반복 계산의 초기값으로 사용함.
void warmStartDistanceJointConstraint( const distanceJointConstraint2& constraint, bodyState& stateA, bodyState& stateB );

// Spring은 두 pass에서 bias를 유지하며, rigid와 limit은 useBias에 따라 보정함.
void solveDistanceJointConstraint( distanceJointConstraint2& constraint, bodyState& stateA, bodyState& stateB, bool useBias );

#pragma endregion

} // namespace zonai
