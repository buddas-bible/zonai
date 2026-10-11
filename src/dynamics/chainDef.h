#pragma once

#include <span>

#include "collision/filter.h"
#include "math/vec2.h"

namespace zonai
{

// 여러 one-sided chain segment를 하나의 terrain macro object로 생성하는 입력값.
// Open chain은 양 끝 point를 ghost vertex로 사용하므로 최소 4점이 필요함.
struct chainDef
{
    std::span<const vec2> points{};

    collisionFilter filter{};

    float friction = 0.6f;
    float restitution = 0.0f;

    bool isLoop = false;
};

} // namespace zonai
