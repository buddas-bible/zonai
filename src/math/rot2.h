#pragma once

#include <cmath>

#include "math/deterministicMath.h"
#include "math/vec2.h"

namespace zonai
{

struct rot2
{
    float c = 1.0f;
    float s = 0.0f;

    static rot2 FromRadians( float radians )
    {
        // Use the validated binary32 rotation contract in its supported range.
        // Preserve arbitrary-angle API behavior outside it until deterministic
        // full-range argument reduction has been independently validated.
        constexpr float limit = 0x1.921fb6p+2f;
        if( radians >= -limit && radians <= limit )
        {
            const auto rotation = deterministicSinCosBounded( radians );
            return { rotation.cosine, rotation.sine };
        }
        return { std::cos( radians ), std::sin( radians ) };
    }
};

inline rot2 operator*( const rot2& a, const rot2& b )
{
    return { a.c * b.c - a.s * b.s, a.s * b.c + a.c * b.s };
}

inline rot2 Inverse( const rot2& rotation )
{
    return { rotation.c, -rotation.s };
}

inline vec2 Rotate( const rot2& rotation, const vec2& vector )
{
    return { rotation.c * vector.x - rotation.s * vector.y, rotation.s * vector.x + rotation.c * vector.y };
}

inline vec2 InverseRotate( const rot2& rotation, const vec2& vector )
{
    return { rotation.c * vector.x + rotation.s * vector.y, -rotation.s * vector.x + rotation.c * vector.y };
}

} // namespace zonai
