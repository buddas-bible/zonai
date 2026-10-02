#pragma once

#include <cstdint>
#include <vector>

#include "dynamics/id.h"

namespace zonai
{

struct sensorVisitor2
{
    std::int32_t shapeIndex = -1;
    std::uint16_t generation = 0;

    constexpr bool operator==( const sensorVisitor2& ) const = default;
};

struct sensor2
{
    std::int32_t shapeIndex = -1;

    // 이전 Step에서 유지된 overlap.
    std::vector<sensorVisitor2> overlaps{};

    // CCD가 Step 도중 검출한 transient crossing.
    // UpdateSensors가 현재 geometry overlap과 합친 뒤 매 Step 비움.
    std::vector<sensorVisitor2> hits{};
};

struct sensorBeginEvent2
{
    shapeId sensorShapeId{};
    shapeId visitorShapeId{};
};

struct sensorEndEvent2
{
    shapeId sensorShapeId{};
    shapeId visitorShapeId{};
};

} // namespace zonai
