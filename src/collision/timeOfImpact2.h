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

    // World 공간 회전 조회 전용: 끝 시점에 분리된 후에도 중간 접촉을 확인함.
    // CCD 기본 경로에서는 사용하지 않음.
    bool detectTransientContacts = false;
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
