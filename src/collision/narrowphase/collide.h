#pragma once

#include "geometry/circle2.h"
#include "geometry/box2.h"
#include "geometry/capsule2.h"
#include "geometry/polygon2.h"
#include "math/transform2.h"
#include "collision/narrowphase/manifold2.h"

namespace zonai
{
    bool Collide(
        const circle2& a,
        const transform2& transformA,
        const circle2& b,
        const transform2& transformB,
        manifold2& manifold);

    bool Collide(
        const circle2& circle,
        const transform2& circleTransform,
        const box2& box,
        const transform2& boxTransform,
        manifold2& manifold);

    bool Collide(
        const box2& a,
        const transform2& transformA,
        const box2& b,
        const transform2& transformB,
        manifold2& manifold);
}