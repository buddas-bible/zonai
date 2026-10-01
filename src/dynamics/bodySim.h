#pragma once

#include <cstdint>

#include "math/transform2.h"

namespace zonai
{

// Solver와 collision 준비에서 자주 사용하는 body의 simulation 상태.
// solver set이 도입되기 전까지는 World가 body와 같은 stable slot index로 보관함.
struct bodySim
{
    static constexpr std::int32_t NULL_INDEX = -1;

    // body origin의 world transform.
    transform2 transform{};

    // body local space의 center of mass.
    vec2 localCenter{};

    // center of mass의 world-space 위치.
    vec2 center{};

    // 한 simulation step 동안 누적되는 외력 / 토크.
    // Step에서 velocity에 반영한 뒤 0으로 초기화함.
    vec2 force{};
    float torque = 0.0f;

    // Solver에서 곱셈으로 사용하기 위한 역질량 / 역관성.
    float invMass = 0.0f;
    float invInertia = 0.0f;

    // local center of mass에서 가장 먼 shape bounds까지의 거리.
    // 회전 속도를 실제 body point의 선속도로 환산할 때 사용함.
    float maxExtent = 0.0f;

    // 이 simulation 데이터가 대응하는 World 내부 body index.
    // NULL_INDEX면 현재 free slot임.
    std::int32_t bodyId = NULL_INDEX;
};

} // namespace zonai
