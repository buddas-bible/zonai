#pragma once

#include "vec2.h"
#include "rot2.h"

namespace zonai
{
    struct transform2
    {
        vec2 position{};
        rot2 rotation{};
    };

    inline vec2 TransformPoint(
        const transform2& transform,
        const vec2& point)
    {
        return transform.position + Rotate(transform.rotation, point);
    }

    inline vec2 InverseTransformPoint(
        const transform2& transform,
        const vec2& point)
    {
        return InverseRotate(
            transform.rotation,
            point - transform.position
        );
    }

    inline vec2 TransformVector(
        const transform2& transform,
        const vec2& vector)
    {
        return Rotate(transform.rotation, vector);
    }

    inline vec2 InverseTransformVector(
        const transform2& transform,
        const vec2& vector)
    {
        return InverseRotate(transform.rotation, vector);
    }
}