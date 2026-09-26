#pragma once

#include <cstdint>
#include <variant>

#include "geometry/capsule2.h"
#include "geometry/circle2.h"
#include "geometry/polygon2.h"
#include "geometry/segment2.h"

#include "collision/filter.h"

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

    // sensor overlap 저장소 index. NULL_INDEX면 일반 collision shape임.
    std::int32_t sensorIndex = NULL_INDEX;

    // 실제 collision geometry. variant가 Box2D의 type + union 역할을 대신함.
    ShapeGeometry geometry{};

    // category / mask / group 기반 collision filter.
    Filter filter{};
};

} // namespace zonai
