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

    // Chain edge는 distance proxy 자체는 segment와 같지만 TOI에서 앞면만 충돌해야 함.
    // 일반 segment는 false라 기존 양면 의미를 그대로 유지함.
    bool oneSided = false;
};

shapeProxy2 MakeShapeProxy( const shapeGeometry& geometry );

} // namespace zonai
