#pragma once

#include "math/vec2.h"

namespace zonai
{

// Solver가 반복해서 읽고 쓰는 Body의 운동 상태.
// sleep / awake solver set이 도입되기 전까지는 World가 Body와 같은 stable slot index로 보관함.
struct BodyState
{
    // 현재 초기 Step에서 사용하는 Body origin의 world-space 선속도.
    // constraint solver 도입 시 center of mass 기준 상태로 전환할 예정임.
    vec2 linearVelocity{};

    // 2D z축 기준 각속도(rad/s).
    float angularVelocity = 0.0f;
};

} // namespace zonai
