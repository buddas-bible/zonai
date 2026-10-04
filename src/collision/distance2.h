#pragma once

#include <array>
#include <cstdint>

#include "collision/shapeProxy2.h"
#include "math/transform2.h"

namespace zonai
{

struct segmentDistanceResult2
{
    vec2 closest1{};
    vec2 closest2{};

    float fraction1 = 0.0f;
    float fraction2 = 0.0f;
    float distanceSquared = 0.0f;
};

// 두 선분 위의 최근접점을 계산하고 endpoint 범위로 fraction을 제한함.
segmentDistanceResult2 SegmentDistance(
    const vec2& p1,
    const vec2& q1,
    const vec2& p2,
    const vec2& q2 );

struct simplexCache2
{
    std::array<std::uint8_t, 3> indexA{};
    std::array<std::uint8_t, 3> indexB{};
    std::uint16_t count = 0;
};

struct distanceInput2
{
    shapeProxy2 proxyA{};
    shapeProxy2 proxyB{};

    // B를 A local frame으로 옮기는 relative transform.
    transform2 transform{};

    bool useRadii = false;
};

struct distanceOutput2
{
    vec2 pointA{};
    vec2 pointB{};
    vec2 normal{};

    float distance = 0.0f;
    int iterations = 0;
};

// GJK로 두 convex proxy 사이의 최소 거리를 계산함.
// cache는 이전 simplex를 다음 호출의 초기 simplex로 재사용하기 위한 저장소임.
distanceOutput2 ShapeDistance(
    const distanceInput2& input,
    simplexCache2& cache );

} // namespace zonai
