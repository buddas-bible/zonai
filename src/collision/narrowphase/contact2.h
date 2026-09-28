#pragma once

#include <array>
#include <cstdint>

#include "collision/narrowphase/manifold2.h"

namespace zonai
{

struct contactEdge2
{
    std::int32_t bodyId = -1;
    std::int32_t prevKey = -1;
    std::int32_t nextKey = -1;
};

// contactId와 어느 Body 쪽 edge인지를 하나의 정수 key로 묶음.
// 하위 1bit가 edgeIndex, 나머지 상위 bit가 contactId임.
constexpr std::int32_t MakeContactKey(
    std::int32_t contactId,
    std::int32_t edgeIndex )
{
    return ( contactId << 1 ) | edgeIndex;
}

constexpr std::int32_t GetContactId(
    std::int32_t contactKey )
{
    return contactKey >> 1;
}

constexpr std::int32_t GetContactEdgeIndex(
    std::int32_t contactKey )
{
    return contactKey & 1;
}

// BroadPhase AABB pair가 유지되는 동안 World가 보관하는 최소 persistent Contact.
// 실제 geometry가 닿지 않는 순간에는 manifold.pointCount가 0일 수 있음.
struct contact2
{
    static constexpr std::int32_t NULL_INDEX = -1;

    // 각 Shape가 속한 Body에 하나씩 연결되는 intrusive contact edge.
    std::array<contactEdge2, 2> edges{};

    std::int32_t shapeIdA = NULL_INDEX;
    std::int32_t shapeIdB = NULL_INDEX;

    // stable slot id. NULL_INDEX면 현재 free slot임.
    std::int32_t contactId = NULL_INDEX;

    // free slot일 때만 다음 free contact id를 저장함.
    std::int32_t nextFreeId = NULL_INDEX;

    localManifold2 manifold{};
};

} // namespace zonai
