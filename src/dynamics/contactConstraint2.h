#pragma once

#include <array>
#include <cstdint>

#include "collision/narrowphase/manifold2.h"
#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"
#include "dynamics/contactSim2.h"

namespace zonai
{

// Contact solver가 한 접촉점에 대해 반복해서 사용하는 계산 결과.
struct contactConstraintPoint2
{
    // center of mass에서 world contact point까지의 벡터.
    vec2 anchorA{};
    vec2 anchorB{};

    // narrow-phase가 계산한 signed separation.
    float separation = 0.0f;

    // 접촉 normal 방향 상대속도.
    // 음수면 서로 접근 중, 양수면 서로 멀어지는 중임.
    float relativeNormalVelocity = 0.0f;

    // normal impulse를 velocity 변화로 환산하는 effective mass의 역수.
    float normalMass = 0.0f;

    // iterative solver가 누적하는 normal impulse.
    // 접촉은 서로 밀어낼 수만 있으므로 항상 0 이상으로 유지함.
    float normalImpulse = 0.0f;
};

// persistent ContactSim을 한 solver step에서 바로 사용할 transient constraint로 변환한 값.
struct contactConstraint2
{
    std::int32_t bodyIdA = contactSim2::NULL_INDEX;
    std::int32_t bodyIdB = contactSim2::NULL_INDEX;

    vec2 normal{};

    float invMassA = 0.0f;
    float invInertiaA = 0.0f;

    float invMassB = 0.0f;
    float invInertiaB = 0.0f;

    std::array<contactConstraintPoint2, MAX_MANIFOLD_POINTS> points{};
    int pointCount = 0;
};

// ContactSim의 local manifold와 Body simulation 상태를 이용해
// normal solver가 바로 사용할 world-space constraint를 준비함.
[[nodiscard]] contactConstraint2 PrepareContactConstraint(
    const contactSim2& contactSim,
    const BodySim& bodySimA,
    const BodyState& bodyStateA,
    const BodySim& bodySimB,
    const BodyState& bodyStateB );

// 한 Contact의 normal constraint를 한 번 풀어 Body velocity에 impulse를 적용함.
// 여러 Contact를 여러 번 반복 호출하면 sequential impulse solver가 됨.
void SolveContactConstraint(
    contactConstraint2& constraint,
    BodyState& bodyStateA,
    BodyState& bodyStateB );

} // namespace zonai
