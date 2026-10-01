#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "dynamics/body.h"
#include "dynamics/contactSim2.h"

namespace zonai
{

// islandGraph2의 flat body/contact 배열에서 이 island가 차지하는 연속 구간.
struct island2
{
    std::size_t bodyStart = 0;
    std::size_t bodyCount = 0;

    std::size_t contactStart = 0;
    std::size_t contactCount = 0;
};

// 한 simulation step에서 solver-active Contact로 구성한 transient connected components.
// island마다 vector를 따로 만들지 않고 body/contact id를 연속 배열에 모아둠.
struct islandGraph2
{
    std::vector<island2> islands{};
    std::vector<std::int32_t> bodyIds{};
    std::vector<std::int32_t> contactIds{};
};

// 현재 body / Contact 상태에서 solver island graph를 다시 구성함.
// manifold point가 하나 이상인 Contact만 graph edge로 사용함.
[[nodiscard]] islandGraph2 BuildIslands(
    std::span<const body> bodies,
    std::span<const contactSim2> contactSims );

} // namespace zonai
