#include "collision/shapeCast2.h"

#include <algorithm>
#include <cassert>
#include <cmath>

#include "collision/constants.h"
#include "collision/distance2.h"

namespace zonai
{

castOutput2 ShapeCast( const shapeCastInput2& input )
{
    assert( input.proxyA.count > 0 );
    assert( input.proxyB.count > 0 );
    assert( input.proxyA.radius >= 0.0f );
    assert( input.proxyB.radius >= 0.0f );
    assert( std::isfinite( input.translationB.x ) );
    assert( std::isfinite( input.translationB.y ) );
    assert( std::isfinite( input.maxFraction ) );
    assert( input.maxFraction >= 0.0f );
    assert( input.maxFraction <= 1.0f );

    const float totalRadius = input.proxyA.radius + input.proxyB.radius;

    // proxy core 사이 목표 거리를 radius 합보다 LINEAR_SLOP만큼 작게 둠.
    // 실제 표면 접촉 직전까지 보수적으로 전진하기 위한 Box2D 방식임.
    float target = std::max( LINEAR_SLOP, totalRadius - LINEAR_SLOP );

    const float tolerance = 0.25f * LINEAR_SLOP;

    assert( target > tolerance );

    simplexCache2 cache{};
    float fraction = 0.0f;

    distanceInput2 distanceInput{};
    distanceInput.proxyA = input.proxyA;
    distanceInput.proxyB = input.proxyB;
    distanceInput.transform = input.transform;
    distanceInput.useRadii = false;

    castOutput2 output{};

    constexpr int MAX_ITERATIONS = 20;

    for( int iteration = 0; iteration < MAX_ITERATIONS; ++iteration )
    {
        ++output.iterations;

        const distanceOutput2 distanceOutput = ShapeDistance( distanceInput, cache );

        if( distanceOutput.distance < target + tolerance )
        {
            if( iteration == 0 )
            {
                if( input.canEncroach && distanceOutput.distance > 2.0f * LINEAR_SLOP )
                {
                    // 이미 가까운 상태에서도 LINEAR_SLOP만큼 더 접근할 수 있게
                    // 현재 거리 바로 안쪽으로 target을 다시 잡음.
                    target = distanceOutput.distance - LINEAR_SLOP;
                }
                else
                {
                    // 시작 시점부터 target 안쪽이면 fraction 0의 hit로 처리함.
                    output.hit = true;

                    const vec2 pointA = distanceOutput.pointA + distanceOutput.normal * input.proxyA.radius;

                    const vec2 pointB = distanceOutput.pointB - distanceOutput.normal * input.proxyB.radius;

                    output.point = ( pointA + pointB ) * 0.5f;

                    return output;
                }
            }
            else
            {
                assert( distanceOutput.distance > 0.0f );
                assert( LengthSquared( distanceOutput.normal ) > 0.0f );

                output.fraction = fraction;
                output.point = distanceOutput.pointA + distanceOutput.normal * input.proxyA.radius;

                output.normal = distanceOutput.normal;

                output.hit = true;

                return output;
            }
        }

        assert( distanceOutput.distance > 0.0f );

        // normal 방향 상대 이동이 음수여야 두 shape가 서로 접근 중임.
        const float denominator = Dot( input.translationB, distanceOutput.normal );

        if( denominator >= 0.0f ) return output;

        // 현재 거리에서 target 거리까지 줄이는 데 필요한 이동 fraction.
        fraction += ( target - distanceOutput.distance ) / denominator;

        if( fraction >= input.maxFraction ) return output;

        distanceInput.transform.position = input.transform.position + input.translationB * fraction;
    }

    return output;
}

} // namespace zonai
