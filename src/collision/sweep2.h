#pragma once

#include "math/rot2.h"
#include "math/transform2.h"
#include "math/vec2.h"

namespace zonai
{

// CCD에서 body의 시작 / 끝 center of mass와 회전을 시간 구간 [0, 1]로 표현함.
struct sweep2
{
    vec2 localCenter{};
    vec2 c1{};
    vec2 c2{};
    rot2 q1{};
    rot2 q2{};
};

// sweep의 time 지점에 해당하는 body origin transform을 계산함.
transform2 GetSweepTransform(
    const sweep2& sweep,
    float time );

} // namespace zonai
