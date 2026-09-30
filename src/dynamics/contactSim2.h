#pragma once

#include <cstdint>

#include "collision/narrowphase/manifold2.h"

namespace zonai
{

// Solver / collision 갱신에서 자주 사용하는 Contact의 simulation 데이터.
// solver set / constraint graph가 도입되기 전까지는 contact2와 같은 stable slot index로 보관함.
struct contactSim2
{
    static constexpr std::int32_t NULL_INDEX = -1;

    // 이 simulation 데이터가 대응하는 World 내부 Contact index.
    // NULL_INDEX면 현재 free slot임.
    std::int32_t contactId = NULL_INDEX;

    std::int32_t bodyIdA = NULL_INDEX;
    std::int32_t bodyIdB = NULL_INDEX;

    std::int32_t shapeIdA = NULL_INDEX;
    std::int32_t shapeIdB = NULL_INDEX;

    // Contact solver가 반복해서 사용하는 inverse mass / inverse inertia.
    float invMassA = 0.0f;
    float invInertiaA = 0.0f;

    float invMassB = 0.0f;
    float invInertiaB = 0.0f;

    // Shape A local space에 저장되는 현재 narrow-phase manifold.
    // AABB pair만 유지되고 geometry는 떨어져 있으면 pointCount가 0일 수 있음.
    localManifold2 manifold{};
};

} // namespace zonai
