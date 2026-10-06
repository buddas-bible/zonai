#include "collision/distance2.h"

#include <algorithm>
#include <cfloat>

namespace zonai
{

segmentDistanceResult2 SegmentDistance( const vec2& p1, const vec2& q1, const vec2& p2, const vec2& q2 )
{
    const vec2 direction1 = q1 - p1;
    const vec2 direction2 = q2 - p2;
    const vec2 offset = p1 - p2;

    const float lengthSquared1 = Dot( direction1, direction1 );
    const float lengthSquared2 = Dot( direction2, direction2 );
    const float offset1 = Dot( offset, direction1 );
    const float offset2 = Dot( offset, direction2 );

    constexpr float epsilonSquared = FLT_EPSILON * FLT_EPSILON;

    float fraction1 = 0.0f;
    float fraction2 = 0.0f;

    if( lengthSquared1 < epsilonSquared || lengthSquared2 < epsilonSquared )
    {
        // 한쪽 또는 양쪽 선분이 점으로 퇴화한 경우를 따로 처리함.
        if( lengthSquared1 >= epsilonSquared )
        {
            fraction1 = std::clamp( -offset1 / lengthSquared1, 0.0f, 1.0f );
        }
        else if( lengthSquared2 >= epsilonSquared )
        {
            fraction2 = std::clamp( offset2 / lengthSquared2, 0.0f, 1.0f );
        }
    }
    else
    {
        const float directionsDot = Dot( direction1, direction2 );

        const float denominator = lengthSquared1 * lengthSquared2 - directionsDot * directionsDot;

        if( denominator != 0.0f )
        {
            fraction1 = std::clamp( ( directionsDot * offset2 - offset1 * lengthSquared2 ) / denominator, 0.0f, 1.0f );
        }

        fraction2 = ( directionsDot * fraction1 + offset2 ) / lengthSquared2;

        if( fraction2 < 0.0f )
        {
            fraction2 = 0.0f;
            fraction1 = std::clamp( -offset1 / lengthSquared1, 0.0f, 1.0f );
        }
        else if( fraction2 > 1.0f )
        {
            fraction2 = 1.0f;
            fraction1 = std::clamp( ( directionsDot - offset1 ) / lengthSquared1, 0.0f, 1.0f );
        }
    }

    const vec2 closest1 = p1 + direction1 * fraction1;

    const vec2 closest2 = p2 + direction2 * fraction2;

    return { closest1, closest2, fraction1, fraction2, LengthSquared( closest2 - closest1 ) };
}

} // namespace zonai
