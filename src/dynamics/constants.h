#pragma once

#include <numbers>

namespace zonai
{

// motion이 sleep threshold 아래에서 이 시간 이상 유지되어야 island가 잠듦.
constexpr float TIME_TO_SLEEP = 0.5f;

// Box2D 기본값과 같은 world 최대 선속도.
constexpr float DEFAULT_MAX_LINEAR_SPEED = 400.0f;

// 한 simulation step에서 허용하는 최대 회전량.
// 지나치게 큰 회전으로 collision 계산이 불안정해지는 것을 막음.
constexpr float MAX_ROTATION = 0.25f * std::numbers::pi_v<float>;

} // namespace zonai
