#pragma once

#include "math/rot2.h"
#include "math/vec2.h"

namespace zonai
{

// Solver가 반복해서 읽고 쓰는 body의 운동 상태.
// sleep / awake solver set이 도입되기 전까지는 world가 body와 같은 stable slot index로 보관함.
struct bodyState
{
    // center of mass의 world-space 선속도.
    // force / impulse / constraint solver는 모두 이 속도를 기준으로 계산함.
    vec2 linearVelocity{};

    // 2D z축 기준 각속도(rad/s).
    float angularVelocity = 0.0f;

    // 이번 solver step에서 아직 bodySim transform에 반영하지 않은 COM 이동량.
    vec2 deltaPosition{};

    // 이번 solver step에서 아직 bodySim transform에 반영하지 않은 회전량.
    rot2 deltaRotation{};
};

} // namespace zonai
