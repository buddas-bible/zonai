#pragma once

#include <limits>
#include <span>

#include "collision/filter.h"
#include "math/vec2.h"

namespace zonai
{

// 여러 one-sided chain segment를 하나의 terrain macro object로 생성하는 입력값.
// Open chain은 실제 point들과 별도로 시작/끝 ghost point를 받아 이웃 normal을 이어줌.
struct chainDef
{
    std::span<const vec2> points{};

    // Open chain 전용 필수 입력. NaN 기본값은 ghost 누락을 유효한 좌표로 오인하지 않게 함.
    vec2 ghost1{ std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN() };
    vec2 ghost2{ std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN() };

    collisionFilter filter{};

    float friction = 0.6f;
    float restitution = 0.0f;

    bool isLoop = false;
};

} // namespace zonai
