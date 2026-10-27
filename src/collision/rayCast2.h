#pragma once

#include "collision/shapeGeometry.h"

namespace zonai
{

// RayCast의 정확한 geometry 교차 결과. fraction은 전체 translation의 [0, maxFraction].
struct rayCastHit2
{
    vec2 point{};
    vec2 normal{};
    float fraction = 0.0f;
    bool hit = false;
};

// World transform을 반영하여 실제 geometry에 광선을 검사함. 시작점이 solid 안이면 miss.
rayCastHit2 RayCastShapeGeometry( const shapeGeometry& geometry, const transform2& transform,
    vec2 origin, vec2 translation, float maxFraction = 1.0f );

} // namespace zonai
