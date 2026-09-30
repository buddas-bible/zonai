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

// local-space point를 world space로 변환함.
inline vec2 TransformPoint(
    const transform2& transform,
    const vec2& point )
{
    return transform.position + Rotate( transform.rotation, point );
}

// world-space point를 local space로 변환함.
inline vec2 InverseTransformPoint(
    const transform2& transform,
    const vec2& point )
{
    return InverseRotate( transform.rotation, point - transform.position );
}

// local-space vector를 world space로 변환함.
inline vec2 TransformVector(
    const transform2& transform,
    const vec2& vector )
{
    return Rotate( transform.rotation, vector );
}

// world-space vector를 local space로 변환함.
inline vec2 InverseTransformVector(
    const transform2& transform,
    const vec2& vector )
{
    return InverseRotate( transform.rotation, vector );
}

// transform2의 역변환을 계산함.
inline transform2 Inverse( const transform2& transform )
{
    const rot2 inverseRotation = Inverse( transform.rotation );

    return
    {
        Rotate( inverseRotation, -transform.position ),
        inverseRotation
    };
}

// a * b 합성 transform을 계산함.
inline transform2 Mul( const transform2& a, const transform2& b )
{
    return
    {
        TransformPoint( a, b.position ),
        a.rotation * b.rotation
    };
}

// inverse(a) * b를 계산해 B local에서 A local로의 상대 transform을 반환함.
inline transform2 InverseMul( const transform2& a, const transform2& b )
{
    return
    {
        InverseTransformPoint( a, b.position ),
        Inverse( a.rotation ) * b.rotation
    };
}

} // namespace zonai
