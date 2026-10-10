#pragma once

#include <cstdint>
#include "math/vec2.h"

namespace zonai
{

// World의 stable Joint slot에 계속 보관되는 Mover Joint의 simulation 상태.
// 회전은 제어하지 않고 Body B가 A에 대해 가져야 하는 상대 선속도만 정의함.
struct moverJointSim2
{
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    // World 좌표계의 목표 상대 선속도와 x/y 방향별 최대 구동 힘.
    vec2 linearVelocity{};
    vec2 maxVelocityForce{};

    // 이전 substep의 누적 선형 impulse. 같은 h에서만 다음 warm start에 재사용함.
    vec2 linearVelocityImpulse{};
    float subStepTime = 0.0f;
};

} // namespace zonai
