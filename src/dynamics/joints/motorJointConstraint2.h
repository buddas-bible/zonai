#pragma once

#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"
#include "dynamics/constraintSoftness2.h"
#include "dynamics/joints/motorJointSim2.h"

namespace zonai
{

// 한 substep 동안 Motor Joint의 velocity / transform spring 제약을 반복해서 풀기 위한 임시 solver 데이터.
// motorJointSim2와 Body 상태를 현재 COM 기준 작용점, softness, impulse 한도로 바꿔서 보관함.
struct motorJointConstraint2
{
    // 결과 impulse를 원래 Joint slot에 저장하고 두 Body state를 찾기 위한 index.
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    // Step 시작 시 COM에서 각 작용점까지의 월드 방향 lever arm.
    // Solve에서는 deltaRotation을 반영해 현재 작용점 속도 v + w x r을 계산함.
    vec2 anchorA{};
    vec2 anchorB{};

    // Step 시작 시 두 COM 사이의 거리. deltaPosition과 현재 lever arm을 더해 anchor 위치 오차를 계산함.
    vec2 deltaCenter{};

    // referenceAngle을 제외한 Step 시작 시 상대 회전.
    // deltaRotation과 결합해 현재 angular spring의 상대각도 오차를 계산함.
    rot2 relativeRotation{};

    // Solver iteration마다 Body를 다시 조회하지 않도록 Prepare에서 복사한 역질량 / 역관성.
    float invMassA = 0.0f;
    float invMassB = 0.0f;
    float invInertiaA = 0.0f;
    float invInertiaB = 0.0f;
    float angularMass = 0.0f;

    // 목표 상대속도. 위치 bias가 아니라 실제 velocity constraint의 목표값임.
    vec2 linearVelocity{};
    float angularVelocity = 0.0f;

    // 양수 Hz일 때만 위치 / 각도 오차를 실제 spring-damper 응답으로 복원함.
    float linearHertz = 0.0f;
    float angularHertz = 0.0f;
    constraintSoftness2 linearSpring{};
    constraintSoftness2 angularSpring{};

    // 한 substep에서 사용할 수 있는 누적 impulse 한도. 각 force / torque에 h를 곱한 값임.
    float maxLinearImpulse = 0.0f;
    float maxLinearSpringImpulse = 0.0f;
    float maxAngularImpulse = 0.0f;
    float maxAngularSpringImpulse = 0.0f;

    // 이번 solver iteration 동안 갱신되는 누적 velocity / spring impulse.
    // Prepare에서 이전 substep cache를 받아오고 Solve 후 motorJointSim2에 다시 저장함.
    vec2 linearVelocityImpulse{};
    vec2 linearSpringImpulse{};
    float angularVelocityImpulse = 0.0f;
    float angularSpringImpulse = 0.0f;
};

// 로컬 작용점을 COM 기준 lever arm으로 바꾸고 force / torque를 substep impulse 한도로 변환함.
[[nodiscard]] motorJointConstraint2 prepareMotorJointConstraint( const motorJointSim2& joint, const bodySim& bodySimA, const bodySim& bodySimB, float subStepTime );

// 이전 substep의 velocity / spring 누적 impulse를 합쳐 두 Body에 반대 방향으로 적용함.
void warmStartMotorJointConstraint( const motorJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB );

// Velocity Motor와 실제 spring-damper 응답을 풀며 Contact bias처럼 제거할 임시 속도는 만들지 않음.
void solveMotorJointConstraint( motorJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB );

} // namespace zonai
