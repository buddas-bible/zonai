#pragma once

#include <cstdint>

#include "collision/filter.h"

namespace zonai
{

struct Shape
{
    static constexpr std::int32_t NULL_INDEX = -1;

    // 이 shape를 소유하는 body index. 아직 연결되지 않았으면 NULL_INDEX임.
    std::int32_t bodyId = NULL_INDEX;

    // sensor overlap 저장소 index. NULL_INDEX면 일반 collision shape임.
    std::int32_t sensorIndex = NULL_INDEX;

    // category / mask / group 기반 collision filter.
    Filter filter{};
};

} // namespace zonai
