#pragma once

#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"
#include "dynamics/constraintSoftness2.h"
#include "dynamics/joints/mouseJointSim2.h"

namespace zonai
{

// 한 substep 동안 Mouse Joint를 반복해서 풀기 위한 임시 solver 데이터.
// 월드 target과 로컬 클릭 지점을 현재 COM 기준 오차로 바꾸고 2x2 effective mass와 힘 제한을 준비함.
struct mouseJointConstraint2
{
    // 결과 impulse를 원래 Joint slot에 저장하고 Body B state를 찾기 위한 index.
    std::int32_t jointId = -1;
    std::int32_t bodyIdB = -1;

    // Step 시작 시 COM에서 클릭 지점까지의 월드 lever arm과 target에 대한 초기 중심 오차.
    // Body B의 deltaPosition / deltaRotation을 더해 현재 spring 오차를 계산함.
    vec2 anchorB{};
    vec2 deltaCenter{};

    // Off-center impulse의 회전 coupling을 포함한 2x2 effective mass 역행렬의 두 열.
    vec2 massX{};
    vec2 massY{};

    // Solver iteration에서 반복 사용하므로 Prepare에서 Body B의 역질량 / 역관성을 복사함.
    float invMass = 0.0f;
    float invInertia = 0.0f;
    float maxImpulse = 0.0f; // maxForce * h. 누적 impulse의 최대 크기.

    // 이번 solver iteration 동안 갱신되는 누적 impulse와 spring-damper 계수.
    vec2 impulse{};
    constraintSoftness2 softness{};
};

// 클릭 위치의 질량 중심 기준 작용점, 2x2 유효 질량, 힘 제한을 준비함.
[[nodiscard]] mouseJointConstraint2 prepareMouseJointConstraint( const mouseJointSim2& joint, const bodySim& bodySimB, float subStepTime );

// 힘 제한 안에서 이전 step의 누적 임펄스를 먼저 적용함.
void warmStartMouseJointConstraint( const mouseJointConstraint2& constraint, bodyState& bodyStateB );

// 월드 목표점을 향한 점 스프링을 풀며, 누적 임펄스의 크기를 dt * maxForce로 제한함.
void solveMouseJointConstraint( mouseJointConstraint2& constraint, bodyState& bodyStateB );

} // namespace zonai
