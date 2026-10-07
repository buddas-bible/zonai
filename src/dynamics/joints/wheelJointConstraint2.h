#pragma once

#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"
#include "dynamics/constraintSoftness2.h"
#include "dynamics/joints/wheelJointSim2.h"

namespace zonai
{

struct wheelJointConstraint2
{
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;
    // Step 시작 시 질량 중심 기준의 작용점과 월드 축. 현재 누적 회전으로 갱신함.
    vec2 anchorA{};
    vec2 anchorB{};
    vec2 deltaCenter{};
    vec2 axisA{};
    float invMassA = 0.0f;
    float invMassB = 0.0f;
    float invInertiaA = 0.0f;
    float invInertiaB = 0.0f;
    float perpendicularMass = 0.0f;
    float axialMass = 0.0f;
    float impulse = 0.0f;
    constraintSoftness2 softness{}; // 축 수직 방향의 수치 안정화.

    bool enableSpring = false;
    float springImpulse = 0.0f;
    constraintSoftness2 springSoftness{}; // 축 방향의 물리 스프링.

    bool enableLimit = false;
    float lowerTranslation = 0.0f; // 부호 있는 축 방향 변위 (m). lower <= upper.
    float upperTranslation = 0.0f;
    float lowerImpulse = 0.0f;
    float upperImpulse = 0.0f;
    float invSubStepTime = 0.0f;

    bool enableMotor = false;
    float motorSpeed = 0.0f;
    float motorMass = 0.0f;
    float maxMotorImpulse = 0.0f;
    float motorImpulse = 0.0f;
};

// 원점 작용점을 질량 중심 기준으로 바꾸고 회전하는 축의 유효 질량을 준비함.
[[nodiscard]] wheelJointConstraint2 prepareWheelJointConstraint( const wheelJointSim2& joint, const bodySim& bodySimA, const bodySim& bodySimB, float subStepTime );
void warmStartWheelJointConstraint( const wheelJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB );
// 모터 → 스프링 → 하한 → 상한 → 축 수직 제약. 물리 스프링은 relaxation에서도 복원 bias를 유지함.
void solveWheelJointConstraint( wheelJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB, bool useBias );

} // namespace zonai
