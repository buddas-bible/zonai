#pragma once

namespace zonai
{

constexpr float LINEAR_SLOP = 0.005f;
constexpr float SPECULATIVE_DISTANCE = 4.0f * LINEAR_SLOP;

// Dynamic Tree가 작은 이동마다 proxy를 다시 배치하지 않도록 사용하는 최대 fat margin.
constexpr float MAX_AABB_MARGIN = 0.05f;

// 작은 shape는 자기 크기에 비례해 더 작은 fat margin을 사용함.
constexpr float AABB_MARGIN_FRACTION = 0.125f;

} // namespace zonai
