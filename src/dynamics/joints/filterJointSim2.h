#pragma once

#include <cstdint>

namespace zonai
{

// World가 보관하는 Filter Joint의 persistent 데이터.
// Solver 상태는 없고 Body 연결 정보만 유지함.
struct filterJointSim2
{
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;
};

} // namespace zonai
