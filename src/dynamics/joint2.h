#pragma once

#include <array>
#include <cstdint>

namespace zonai
{
#pragma region Edge

struct jointEdge2
{
    std::int32_t bodyId = -1;
    std::int32_t prevKey = -1;
    std::int32_t nextKey = -1;
};

#pragma endregion

// Contact와 같이 (slot << 1) | edgeIndex로 Body의 양방향 연결을 유지함.
#pragma region Storage

struct joint2
{
    std::int32_t jointId = -1;
    std::uint16_t generation = 0;
    std::int32_t nextFree = -1;
    std::array<jointEdge2, 2> edges{};
    bool collideConnected = false;
};

#pragma endregion
} // namespace zonai
