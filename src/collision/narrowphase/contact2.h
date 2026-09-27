#pragma once

#include <cstdint>

#include "collision/narrowphase/manifold2.h"

namespace zonai
{

// BroadPhase AABB pair가 유지되는 동안 World가 보관하는 최소 persistent Contact.
// 실제 geometry가 닿지 않는 순간에는 manifold.pointCount가 0일 수 있음.
struct contact2
{
    static constexpr std::int32_t NULL_INDEX = -1;

    std::int32_t shapeIdA = NULL_INDEX;
    std::int32_t shapeIdB = NULL_INDEX;
    localManifold2 manifold{};
};

} // namespace zonai
