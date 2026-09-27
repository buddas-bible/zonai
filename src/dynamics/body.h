#pragma once

#include <cstdint>

#include "dynamics/bodyType.h"
#include "math/transform2.h"

namespace zonai
{

struct Body
{
    static constexpr std::int32_t NULL_INDEX = -1;

    // Body의 물리 동작 종류.
    BodyType type = BodyType::Static;

    // 아직 solver용 BodySim이 없으므로 현재는 Body가 world transform을 직접 보관함.
    transform2 transform{};

    // 이 Body에 연결된 첫 Shape index.
    std::int32_t headShapeId = NULL_INDEX;

    // 이 Body에 연결된 Shape 개수.
    std::int32_t shapeCount = 0;
};

} // namespace zonai
