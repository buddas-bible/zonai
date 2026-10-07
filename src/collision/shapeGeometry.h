#pragma once

#include <variant>

#include "geometry/capsule2.h"
#include "geometry/circle2.h"
#include "geometry/polygon2.h"
#include "geometry/segment2.h"

#include "collision/aabb2.h"
#include "math/transform2.h"

namespace zonai
{

// Box2D의 shape type + union을 C++20 variant로 표현함.
// monostate는 아직 geometry가 연결되지 않은 상태임.
using shapeGeometry = std::variant<std::monostate, circle2, capsule2, polygon2, segment2>;

struct shapeExtent2
{
    float minExtent = 0.0f;
    float maxExtent = 0.0f;
};

// body local geometry에 transform을 적용해 world-space AABB를 계산함.
aabb2 ComputeShapeAABB( const shapeGeometry& geometry, const transform2& transform );

// Dynamic Tree fat AABB에 사용할 shape 크기 기반 margin을 계산함.
float ComputeShapeAABBMargin( const shapeGeometry& geometry );

// localCenter 기준으로 CCD에 사용할 최소 / 최대 shape extent를 계산함.
shapeExtent2 ComputeShapeExtent( const shapeGeometry& geometry, const vec2& localCenter );

} // namespace zonai
