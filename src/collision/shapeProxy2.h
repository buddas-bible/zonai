#pragma once

#include <array>

#include "collision/shapeGeometry.h"

namespace zonai
{

// GJK / ShapeCast / TOI가 구체적인 geometry 종류와 무관하게 사용할 convex point cloud.
struct shapeProxy2
{
    std::array<vec2, MAX_POLYGON_VERTICES> points{};
    int count = 0;
    float radius = 0.0f;
};

shapeProxy2 MakeShapeProxy( const shapeGeometry& geometry );

} // namespace zonai
