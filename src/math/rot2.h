#pragma once

#include <cmath>

#include "vec2.h"

namespace zonai
{

struct rot2
{
    float c = 1.0f;
    float s = 0.0f;

    static rot2 FromRadians( float radians )
    {
        return
        {
            std::cos( radians ),
            std::sin( radians )
        };
    }
};

inline rot2 operator*( const rot2& a, const rot2& b )
{
    return
    {
        a.c * b.c - a.s * b.s,
        a.s * b.c + a.c * b.s
    };
}

// 주어진 회전(rot2)의 역방향 회전을 반환합니다.
inline rot2 Inverse( const rot2& rotation )
{
    return { rotation.c, -rotation.s };
}

// 주어진 회전(rot2)과 벡터(vec2)를 받아서, 벡터를 회전시킨 결과를 반환합니다.
inline vec2 Rotate( const rot2& rotation, const vec2& vector )
{
    return
    {
        rotation.c * vector.x - rotation.s * vector.y,
        rotation.s * vector.x + rotation.c * vector.y
    };
}

// 주어진 회전(rot2)과 벡터(vec2)를 받아서, 벡터를 회전의 역방향으로 회전시킨 결과를 반환합니다.
// x' = c * x + s * y;
// y' = -s * x + c * y;
inline vec2 InverseRotate( const rot2& rotation, const vec2& vector )
{
    return
    {
        rotation.c * vector.x + rotation.s * vector.y,
        -rotation.s * vector.x + rotation.c * vector.y
    };
}

}