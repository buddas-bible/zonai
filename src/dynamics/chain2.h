#pragma once

#include <cstdint>
#include <vector>

namespace zonai
{

// World가 소유하는 parent chain macro object.
// 실제 collision은 shapeIds가 가리키는 chainSegment2 runtime shape들이 담당함.
struct chain2
{
    static constexpr std::int32_t NULL_INDEX = -1;

    std::int32_t chainId = NULL_INDEX;
    std::int32_t bodyId = NULL_INDEX;

    std::uint16_t generation = 0;

    // Loop 여부만 parent가 기억하고 point 좌표는 child geometry를 source of truth로 사용함.
    bool isLoop = false;

    // free slot 재사용용.
    std::int32_t nextFreeId = NULL_INDEX;

    // 같은 body에 연결된 parent chain singly linked list.
    std::int32_t nextChainId = NULL_INDEX;

    // 내부 chainSegment2 shape의 stable slot index.
    std::vector<std::int32_t> shapeIds{};
};

} // namespace zonai
