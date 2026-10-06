#pragma once

#include <cstdint>
#include "math/vec2.h"

namespace zonai
{

struct mouseJointSim2
{
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;
    vec2 target{};
    vec2 localAnchorB{};
    float hertz = 5.0f;        // 스프링 주파수 (Hz)
    float dampingRatio = 0.7f; // 감쇠비. 1이면 임계 감쇠
    float maxForce = 1000.0f;  // 허용하는 힘의 최대 크기

    // 누적 임펄스. 다음 step의 warm start와 힘 조회에 사용함.
    vec2 impulse{};
    float subStepTime = 0.0f; // 누적 임펄스를 구한 시간 간격
};

} // namespace zonai
