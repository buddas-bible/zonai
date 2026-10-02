#include "collision/sweep2.h"

#include <cassert>
#include <cmath>

namespace zonai
{

transform2 GetSweepTransform(
    const sweep2& sweep,
    float time )
{
    assert( std::isfinite( time ) );
    assert( time >= 0.0f );
    assert( time <= 1.0f );

    const float inverseTime = 1.0f - time;

    const vec2 center =
        sweep.c1 * inverseTime +
        sweep.c2 * time;

    // Box2D와 같이 회전의 cos / sin을 선형 보간한 뒤 정규화함.
    rot2 rotation
    {
        sweep.q1.c * inverseTime +
        sweep.q2.c * time,

        sweep.q1.s * inverseTime +
        sweep.q2.s * time
    };

    const float lengthSquared =
        rotation.c * rotation.c +
        rotation.s * rotation.s;

    assert( lengthSquared > 0.0f );

    if( lengthSquared > 0.0f )
    {
        const float inverseLength =
            1.0f / std::sqrt( lengthSquared );

        rotation.c *= inverseLength;
        rotation.s *= inverseLength;
    }

    return
    {
        center - Rotate( rotation, sweep.localCenter ),
        rotation
    };
}

} // namespace zonai
