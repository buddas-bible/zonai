#pragma once

#include <cstdint>
#include <variant>

#include "geometry/capsule2.h"
#include "geometry/circle2.h"
#include "geometry/polygon2.h"
#include "geometry/segment2.h"

#include "collision/filter.h"
#include "math/transform2.h"

namespace zonai
{

// Box2D의 shape type + union을 C++20에서 안전하게 표현함.
// monostate는 아직 geometry가 연결되지 않은 Shape 상태임.
using ShapeGeometry =
    std::variant<
        std::monostate,
        circle2,
        capsule2,
        polygon2,
        segment2
    >;

struct Shape
{
    static constexpr std::int32_t NULL_INDEX = -1;

    // 이 shape를 소유하는 body index. 아직 연결되지 않았으면 NULL_INDEX임.
    std::int32_t bodyId = NULL_INDEX;

    // 같은 Body에 연결된 이전 / 다음 Shape index.
    // Body의 head부터 index 기반 doubly linked list로 순회함.
    std::int32_t prevShapeId = NULL_INDEX;
    std::int32_t nextShapeId = NULL_INDEX;

    // slot이 재사용될 때 증가해 오래된 ShapeId를 검출함.
    std::uint16_t generation = 0;

    // sensor overlap 저장소 index. NULL_INDEX면 일반 collision shape임.
    std::int32_t sensorIndex = NULL_INDEX;

    // free slot일 때 다음 재사용 가능한 Shape index.
    std::int32_t nextFreeId = NULL_INDEX;

    // BroadPhase에 등록된 proxy key. 아직 proxy가 없으면 NULL_INDEX임.
    std::int32_t proxyKey = NULL_INDEX;

    // Body local space에 저장되는 실제 collision geometry.
    // variant가 Box2D의 type + union 역할을 대신함.
    ShapeGeometry geometry{};

    // category / mask / group 기반 collision filter.
    Filter filter{};
};

// Body local geometry에 transform을 적용해 world-space AABB를 계산함.
aabb2 ComputeShapeAABB(
    const ShapeGeometry& geometry,
    const transform2& transform );

} // namespace zonai
