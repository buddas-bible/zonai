#pragma once

#include <cstdint>

#include "dynamics/bodyType.h"
#include "math/transform2.h"

namespace zonai
{

struct Body
{
    static constexpr std::int32_t NULL_INDEX = -1;

    // World 내부 stable slot id. NULL_INDEX면 현재 free slot임.
    std::int32_t bodyId = NULL_INDEX;

    // free slot일 때 다음 재사용 가능한 Body index.
    std::int32_t nextFreeId = NULL_INDEX;

    // Body의 물리 동작 종류.
    BodyType type = BodyType::Static;

    // 아직 solver용 BodySim이 없으므로 현재는 Body가 world transform을 직접 보관함.
    transform2 transform{};

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
