#pragma once

#include "geometry/capsule2.h"
#include "geometry/circle2.h"
#include "geometry/polygon2.h"
#include "geometry/segment2.h"
#include "math/vec2.h"

namespace zonai
{

// Shape 하나의 local-space 질량 특성.
// 2D에서는 z축 회전만 존재하므로 rotationalInertia는 관성 텐서의 z축 성분에 해당하는 스칼라 I임.
struct massData2
{
    // M = density * area
    float mass = 0.0f;

    // Shape local space에서의 center of mass.
    vec2 center{};

    // center를 지나는 z축 기준 회전 관성.
    // I = integral( r^2 dm )
    float rotationalInertia = 0.0f;
};

massData2 ComputeMass( const circle2& circle, float density );
massData2 ComputeMass( const capsule2& capsule, float density );
massData2 ComputeMass( const polygon2& polygon, float density );

// 선분은 면적이 없으므로 질량을 만들지 않음.
massData2 ComputeMass( const segment2& segment, float density );

} // namespace zonai
