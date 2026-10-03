#pragma once

#include "math/rot2.h"
#include "math/vec2.h"

namespace zonai
{

struct transform2
{
    vec2 position{};
    rot2 rotation{};
};

inline vec2 TransformPoint(
    const transform2& transform,
    const vec2& point )
{
    return transform.position + Rotate( transform.rotation, point );
}

inline vec2 InverseTransformPoint(
    const transform2& transform,
    const vec2& point )
{
    return InverseRotate( transform.rotation, point - transform.position );
}

inline vec2 TransformVector(
    const transform2& transform,
    const vec2& vector )
{
    return Rotate( transform.rotation, vector );
}

inline vec2 InverseTransformVector(
    const transform2& transform,
    const vec2& vector )
{
    return InverseRotate( transform.rotation, vector );
}

inline transform2 Inverse( const transform2& transform )
{
    const rot2 inverseRotation = Inverse( transform.rotation );

    return
    {
        Rotate( inverseRotation, -transform.position ),
        inverseRotation
    };
}

// a * b 합성 transform.
inline transform2 Mul( const transform2& a, const transform2& b )
{
    return
    {
        TransformPoint( a, b.position ),
        a.rotation * b.rotation
    };
}

// inverse(a) * b. B local 좌표를 A local 좌표로 옮기는 상대 transform.
inline transform2 InverseMul( const transform2& a, const transform2& b )
{
    return
    {
        InverseTransformPoint( a, b.position ),
        Inverse( a.rotation ) * b.rotation
    };
}

} // namespace zonai
