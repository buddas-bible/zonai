#pragma once

#include "collision/shapeProxy2.h"
#include "collision/sweep2.h"

namespace zonai
{

enum class toiState2
{
    Unknown,
    Failed,
    Overlapped,
    Hit,
    Separated
};

struct toiInput2
{
    shapeProxy2 proxyA{};
    shapeProxy2 proxyB{};

    sweep2 sweepA{};
    sweep2 sweepB{};

    float maxFraction = 1.0f;
};

struct toiOutput2
{
    toiState2 state = toiState2::Unknown;

    vec2 point{};
    vec2 normal{};

    float fraction = 1.0f;
};

// 두 moving convex shape가 처음 접촉하기 전까지의 최대 sweep fraction을 계산함.
toiOutput2 TimeOfImpact( const toiInput2& input );

} // namespace zonai
