#pragma once

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

    // Open chain 전용. Loop에서는 points가 스스로 닫히므로 사용하지 않음.
    vec2 ghost1{};
    vec2 ghost2{};

    collisionFilter filter{};

    float friction = 0.6f;
    float restitution = 0.0f;

    bool isLoop = false;
};

} // namespace zonai
