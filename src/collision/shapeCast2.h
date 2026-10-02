#pragma once

#include "collision/shapeProxy2.h"
#include "math/transform2.h"

namespace zonai
{

// ShapeCast 결과. hit이면 fraction 지점에서 point / normal이 유효함.
struct castOutput2
{
    vec2 normal{};
    vec2 point{};

    float fraction = 0.0f;
    int iterations = 0;

    bool hit = false;
};

// A는 고정하고 B를 translationB 방향으로 이동시키는 선형 shape cast 입력.
struct shapeCastInput2
{
    shapeProxy2 proxyA{};
    shapeProxy2 proxyB{};

    // 시작 시점의 B relative transform. A local frame 기준임.
    transform2 transform{};

    vec2 translationB{};

    float maxFraction = 1.0f;
    bool canEncroach = false;
};

// GJK distance를 반복 사용해 B가 A에 처음 닿는 translation fraction을 계산함.
castOutput2 ShapeCast(
    const shapeCastInput2& input );

} // namespace zonai
