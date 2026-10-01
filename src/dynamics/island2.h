#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include "dynamics/body.h"
#include "dynamics/contactSim2.h"

namespace zonai
{

// 한 simulation step에서 solver-active Contact로 연결된 non-static body 묶음.
// Static body는 island body 목록에는 들어가지 않지만 그 body와 연결된 Contact는 포함됨.
struct island2
{
    std::vector<std::int32_t> bodyIds{};
    std::vector<std::int32_t> contactIds{};
};

// 현재 body / Contact 상태에서 solver island를 다시 구성함.
// manifold point가 하나 이상인 Contact만 island graph edge로 사용함.
[[nodiscard]] std::vector<island2> BuildIslands(
    std::span<const body> bodies,
    std::span<const contactSim2> contactSims );

} // namespace zonai
