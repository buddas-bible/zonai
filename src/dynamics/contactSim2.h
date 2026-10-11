#pragma once

#include <array>
#include <cstdint>

#include "collision/narrowphase/manifold2.h"
#include "math/transform2.h"

namespace zonai
{

// 이전 step의 solver 결과를 manifold point 순서에 맞춰 보관함.
// point id가 다음 narrow-phase 결과와 같으면 warm start에 재사용함.
struct contactImpulse2
{
    float normalImpulse = 0.0f;
    float tangentImpulse = 0.0f;
};

// Solver / collision 갱신에서 자주 사용하는 Contact의 simulation 데이터.
// solver set / constraint graph가 도입되기 전까지는 contact2와 같은 stable slot index로 보관함.
struct contactSim2
{
    static constexpr std::int32_t NULL_INDEX = -1;

    // 이 simulation 데이터가 대응하는 world 내부 Contact index.
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

    // shape A local space에 저장되는 현재 narrow-phase manifold.
    // AABB pair만 유지되고 geometry는 떨어져 있으면 pointCount가 0일 수 있음.
    localManifold2 manifold{};

    // manifold.points[i]와 같은 순서의 cached solver impulse.
    // narrow-phase가 새 manifold를 만들면 point id로 이전 값을 다시 연결함.
    std::array<contactImpulse2, MAX_MANIFOLD_POINTS> impulses{};

    // Contact recycling용 cache.
    //
    // fresh narrowphase 시점의 동일 world contact point를 A/B local anchor로 각각 저장하고,
    // 작은 상대 이동에서는 이 두 anchor를 새 transform으로 옮겨 separation만 갱신함.
    // 재활용 중 기준 anchor / pose는 유지하고, fresh narrowphase에서만 다시 저장함.
    std::array<vec2, MAX_MANIFOLD_POINTS> recyclePointA{};
    std::array<vec2, MAX_MANIFOLD_POINTS> recyclePointB{};
    std::array<float, MAX_MANIFOLD_POINTS> recycleSeparation{};

    transform2 cachedRelativeTransform{};
    rot2 cachedRotationA{};
    rot2 cachedRotationB{};

    bool recycleCacheValid = false;
};

} // namespace zonai
