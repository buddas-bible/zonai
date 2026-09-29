#pragma once

#include <cstdint>

#include "math/transform2.h"

namespace zonai
{

// Solver와 collision 준비에서 자주 사용하는 Body의 simulation 상태.
// solver set이 도입되기 전까지는 World가 Body와 같은 stable slot index로 보관함.
struct BodySim
{
    static constexpr std::int32_t NULL_INDEX = -1;

    // Body origin의 world transform.
    transform2 transform{};

    // Body local space의 center of mass.
    vec2 localCenter{};

    // center of mass의 world-space 위치.
    vec2 center{};

    // Solver에서 곱셈으로 사용하기 위한 역질량 / 역관성.
    float invMass = 0.0f;
    float invInertia = 0.0f;

    // 이 simulation 데이터가 대응하는 World 내부 Body index.
    // NULL_INDEX면 현재 free slot임.
    std::int32_t bodyId = NULL_INDEX;
};

} // namespace zonai
