#pragma once

#include <array>
#include <cstdint>

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

// BroadPhase AABB pair가 유지되는 동안 World가 보관하는 persistent Contact.
// 수명 / Shape 연결 / Body intrusive list 같은 cold data만 보관하고
// manifold과 solver hot data는 contactSim2에 분리함.
struct contact2
{
    static constexpr std::int32_t NULL_INDEX = -1;

    // 각 Shape가 속한 Body에 하나씩 연결되는 intrusive contact edge.
    std::array<contactEdge2, 2> edges{};

    std::int32_t shapeIdA = NULL_INDEX;
    std::int32_t shapeIdB = NULL_INDEX;

    // stable slot id. NULL_INDEX면 현재 free slot임.
    std::int32_t contactId = NULL_INDEX;

    // 생성 시점에 두 body가 모두 허용했으면 작은 상대 이동에서 manifold를 재활용함.
    // 이후 body 설정이 바뀌어도 기존 Contact의 값은 유지함.
    bool enableRecycling = false;

    // slot이 재사용될 때 증가해 오래된 ContactId를 검출함.
    std::uint32_t generation = 0;

    // free slot일 때만 다음 free contact id를 저장함.
    std::int32_t nextFreeId = NULL_INDEX;
};

} // namespace zonai
