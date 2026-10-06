#pragma once

#include <array>
#include <cstdint>

#include "collision/narrowphase/manifold2.h"
#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"
#include "dynamics/constraintSoftness2.h"
#include "dynamics/contactSim2.h"

namespace zonai
{

// Contact penetration을 부드럽게 줄이기 위한 solver softness.
// massScale + impulseScale = 1 관계를 이용해 correction을 안정적으로 감쇠함.
using contactSoftness2 = constraintSoftness2;

// Hertz / damping ratio를 한 step에서 사용할 softness 계수로 변환함.
[[nodiscard]] contactSoftness2 MakeContactSoftness( float hertz, float dampingRatio, float timeStep );

// Contact solver가 한 접촉점에 대해 반복해서 사용하는 계산 결과.
struct contactConstraintPoint2
{
    // center of mass에서 world contact point까지의 벡터.
    vec2 anchorA{};
    vec2 anchorB{};

    // solver가 delta transform으로 현재 separation을 다시 계산하기 위한 기준값.
    float baseSeparation = 0.0f;

    // 접촉 normal 방향 상대속도.
    // 음수면 서로 접근 중, 양수면 서로 멀어지는 중임.
    float relativeNormalVelocity = 0.0f;

    // normal impulse를 velocity 변화로 환산하는 effective mass의 역수.
    float normalMass = 0.0f;

    // tangent impulse를 velocity 변화로 환산하는 effective mass의 역수.
    float tangentMass = 0.0f;

    // iterative solver가 누적하는 normal impulse.
    // 접촉은 서로 밀어낼 수만 있으므로 항상 0 이상으로 유지함.
    float normalImpulse = 0.0f;

    // 이번 step에서 normal 방향으로 실제 적용된 impulse의 누적값.
    // warm start / push / relax 결과를 합쳐 restitution의 압축량 판정에 사용함.
    float totalNormalImpulse = 0.0f;

    // totalNormalImpulse 중 restitution 단계에서 만든 반발 성분.
    // restitution iteration이 이미 사용한 반발량을 다시 만들지 않게 함.
    float restitutionImpulse = 0.0f;

    // 접촉면을 따라 미끄러지는 상대속도를 줄이는 누적 friction impulse.
    // Coulomb cone에 의해 |tangentImpulse| <= friction * normalImpulse로 제한됨.
    float tangentImpulse = 0.0f;
};

// persistent ContactSim을 한 solver step에서 바로 사용할 transient constraint로 변환한 값.
struct contactConstraint2
{
    // Solve가 끝난 뒤 impulse를 원래 ContactSim에 되돌려 저장하기 위한 stable id.
    std::int32_t contactId = contactSim2::NULL_INDEX;

    std::int32_t bodyIdA = contactSim2::NULL_INDEX;
    std::int32_t bodyIdB = contactSim2::NULL_INDEX;

    vec2 normal{};

    float invMassA = 0.0f;
    float invInertiaA = 0.0f;

    float invMassB = 0.0f;
    float invInertiaB = 0.0f;

    // penetration correction에서 사용할 soft constraint 계수.
    contactSoftness2 softness{};

    // correction 때문에 한 step에서 만들어질 수 있는 최대 분리 속도.
    float maxPushSpeed = 0.0f;

    // positive separation을 이번 step 안에서 닫지 않도록 speculative bias에 사용함.
    float invTimeStep = 0.0f;

    // 두 shape 사이의 Coulomb friction coefficient.
    float friction = 0.0f;

    // 두 shape의 restitution을 mix한 반발계수.
    float restitution = 0.0f;

    std::array<contactConstraintPoint2, MAX_MANIFOLD_POINTS> points{};
    int pointCount = 0;
};

// ContactSim의 local manifold와 body simulation 상태를 이용해
// normal solver가 바로 사용할 world-space constraint를 준비함.
[[nodiscard]] contactConstraint2 PrepareContactConstraint(
    const contactSim2& contactSim,
    const bodySim& bodySimA, const bodyState& bodyStateA,
    const bodySim& bodySimB, const bodyState& bodyStateB );

// 이전 step에서 캐싱한 누적 normal impulse를 solver 시작 전에 먼저 적용함.
void WarmStartContactConstraint(
    contactConstraint2& constraint,
    bodyState& bodyStateA, bodyState& bodyStateB );

// 한 Contact의 constraint를 한 번 풀어 body velocity에 impulse를 적용함.
// useBias=true면 penetration을 줄이는 normal soft push만 수행하고,
// false면 normal relax 뒤 tangent Coulomb friction까지 풂.
void SolveContactConstraint(
    contactConstraint2& constraint,
    bodyState& bodyStateA, bodyState& bodyStateB,
    bool useBias );

// 충돌 전 접근 속도가 threshold보다 충분히 클 때
// relax가 제거한 normal 속도에 restitution 목표 속도를 다시 적용함.
void ApplyRestitutionContactConstraint(
    contactConstraint2& constraint,
    bodyState& bodyStateA, bodyState& bodyStateB,
    float threshold );

// 이번 step에서 수렴한 누적 impulse를 persistent ContactSim에 저장함.
void StoreContactImpulses( const contactConstraint2& constraint, contactSim2& contactSim );

} // namespace zonai
