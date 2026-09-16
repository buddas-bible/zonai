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

// local-space point를 transform의 world space로 변환합니다.
inline vec2 TransformPoint(
    const transform2& transform,
    const vec2& point )
{
    return transform.position + zonai::Rotate( transform.rotation, point );
}

// world-space point를 transform의 local space로 변환합니다.
inline vec2 InverseTransformPoint(
    const transform2& transform,
    const vec2& point )
{
    return zonai::InverseRotate( transform.rotation, point - transform.position );
}

// local-space vector를 transform의 world space로 변환합니다.
inline vec2 TransformVector(
    const transform2& transform,
    const vec2& vector )
{
    return zonai::Rotate( transform.rotation, vector );
}

// world-space vector를 transform의 local space로 변환합니다.
inline vec2 InverseTransformVector(
    const transform2& transform,
    const vec2& vector )
{
    return zonai::InverseRotate( transform.rotation, vector );
}

// transform2의 역변환을 계산합니다.
inline transform2 Inverse( const transform2& transform )
{
    const rot2 inverseRotation = zonai::Inverse( transform.rotation );

    return
    {
        zonai::Rotate( inverseRotation, -transform.position ),
        inverseRotation
    };
}

// // a * b 합성 transform을 계산합니다.
inline transform2 Mul( const transform2& a, const transform2& b )
{
    return
    {
        zonai::TransformPoint( a, b.position ),
        a.rotation * b.rotation
    };
}

// inverse(a) * b를 계산하여 B local space에서 A local space로의 상대 transform을 반환합니다.
inline transform2 InverseMul( const transform2& a, const transform2& b )
{
    return
    {
        zonai::InverseTransformPoint( a, b.position ),
        zonai::Inverse( a.rotation ) * b.rotation
    };
}

}