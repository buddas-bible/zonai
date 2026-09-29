#pragma once

#include <cstdint>

#include "dynamics/bodyType.h"

namespace zonai
{

struct Body
{
    static constexpr std::int32_t NULL_INDEX = -1;

    // World 내부 stable slot id. NULL_INDEX면 현재 free slot임.
    std::int32_t bodyId = NULL_INDEX;

    // slot이 재사용될 때 증가해 오래된 BodyId를 검출함.
    std::uint16_t generation = 0;

    // free slot일 때 다음 재사용 가능한 Body index.
    std::int32_t nextFreeId = NULL_INDEX;

    // Body의 물리 동작 종류.
    BodyType type = BodyType::Static;

    // Dynamic Body에 연결된 Shape 질량의 합.
    float mass = 0.0f;

    // Dynamic Body의 center of mass 기준 회전 관성.
    float inertia = 0.0f;

    // [contactId : edgeIndex] key로 연결된 첫 Contact.
    // 하위 1bit는 Contact의 어느 edge가 이 Body에 연결됐는지 나타냄.
    std::int32_t headContactKey = NULL_INDEX;

    // 이 Body에 연결된 Contact 개수.
    std::int32_t contactCount = 0;

    // 이 Body에 연결된 첫 Shape index.
    std::int32_t headShapeId = NULL_INDEX;

    // 이 Body에 연결된 Shape 개수.
    std::int32_t shapeCount = 0;
};

} // namespace zonai
