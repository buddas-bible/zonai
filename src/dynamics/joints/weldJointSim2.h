#pragma once

#include <cstdint>
#include "math/vec2.h"

namespace zonai
{

// World의 stable Joint slot에 계속 보관되는 Weld Joint의 simulation 상태.
// 기준 상대 transform과 softness 설정, 다음 substep warm start에 필요한 누적 impulse를 유지함.
struct weldJointSim2
{
    // 공용 Joint slot과 연결된 id, 그리고 두 Body의 simulation index.
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    // Body origin 기준의 로컬 작용점. Prepare에서 현재 COM 기준 lever arm으로 변환함.
    vec2 localAnchorA{};
    vec2 localAnchorB{};

    // B가 A에 대해 유지해야 하는 기준 상대 각도.
    float referenceAngle = 0.0f;

    // 선형 / 회전 Weld의 spring-damper 설정.
    // 0 Hz는 hard Weld, 양수 Hz는 해당 방향을 soft Weld로 만듦.
    float linearHertz = 0.0f;
    float linearDampingRatio = 0.0f;
    float angularHertz = 0.0f;
    float angularDampingRatio = 0.0f;

    // 이전 substep에서 구한 선형 / 회전 누적 impulse.
    // Prepare에서 Constraint로 복사하고 Solve가 끝나면 다시 여기 저장해 다음 warm start에 사용함.
    vec2 impulse{};
    float angularImpulse = 0.0f;
    float subStepTime = 0.0f; // cached impulse가 계산된 시간 간격.
};

} // namespace zonai
