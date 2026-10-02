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
    std::vector<sensorVisitor2> overlaps{};
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
