#pragma once

#include "geometry/capsule2.h"
#include "geometry/circle2.h"
#include "geometry/polygon2.h"
#include "geometry/segment2.h"
#include "math/vec2.h"

namespace zonai
{

// Shape의 질량, local center of mass, center 기준 회전 관성을 묶음.
struct massData2
{
    float mass = 0.0f;
    vec2 center{};
    float rotationalInertia = 0.0f;
};

massData2 ComputeMass( const circle2& circle, float density );
massData2 ComputeMass( const capsule2& capsule, float density );
massData2 ComputeMass( const polygon2& polygon, float density );

// 선분은 면적이 없으므로 질량을 만들지 않음.
massData2 ComputeMass( const segment2& segment, float density );

} // namespace zonai
