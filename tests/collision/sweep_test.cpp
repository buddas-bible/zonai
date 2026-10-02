#include <cassert>
#include <cmath>

#include "collision/sweep2.h"

using namespace zonai;

int main()
{
    constexpr float epsilon = 1e-5f;

    const sweep2 sweep
    {
        { 1.0f, 0.0f },
        { 0.0f, 0.0f },
        { 10.0f, 0.0f },
        {},
        rot2::FromRadians( 0.5f * 3.14159265358979323846f )
    };

    const transform2 start =
        GetSweepTransform(
            sweep,
            0.0f
        );

    assert( std::fabs( start.position.x + 1.0f ) < epsilon );
    assert( std::fabs( start.position.y ) < epsilon );

    const transform2 middle =
        GetSweepTransform(
            sweep,
            0.5f
        );

    const vec2 middleCenter =
        TransformPoint(
            middle,
            sweep.localCenter
        );

    assert( std::fabs( middleCenter.x - 5.0f ) < epsilon );
    assert( std::fabs( middleCenter.y ) < epsilon );
    assert( std::fabs( middle.rotation.c - std::sqrt( 0.5f ) ) < epsilon );
    assert( std::fabs( middle.rotation.s - std::sqrt( 0.5f ) ) < epsilon );

    const transform2 end =
        GetSweepTransform(
            sweep,
            1.0f
        );

    const vec2 endCenter =
        TransformPoint(
            end,
            sweep.localCenter
        );

    assert( std::fabs( endCenter.x - 10.0f ) < epsilon );
    assert( std::fabs( endCenter.y ) < epsilon );
    assert( std::fabs( end.position.x - 10.0f ) < epsilon );
    assert( std::fabs( end.position.y + 1.0f ) < epsilon );

    return 0;
}
