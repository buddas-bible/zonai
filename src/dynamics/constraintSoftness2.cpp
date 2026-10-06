#include "dynamics/constraintSoftness2.h"

#include <cassert>
#include <cmath>
#include <numbers>

namespace zonai
{

constraintSoftness2 makeConstraintSoftness( float hertz, float dampingRatio, float timeStep )
{
    assert( std::isfinite( hertz ) && hertz >= 0.0f );
    assert( std::isfinite( dampingRatio ) && dampingRatio >= 0.0f );
    assert( std::isfinite( timeStep ) && timeStep >= 0.0f );

    if( hertz == 0.0f || timeStep == 0.0f ) return {};

    const float omega = 2.0f * std::numbers::pi_v<float> * hertz;
    const float a1 = 2.0f * dampingRatio + timeStep * omega;
    const float a2 = timeStep * omega * a1;
    const float a3 = 1.0f / ( 1.0f + a2 );

    return { omega / a1, a2 * a3, a3 };
}

} // namespace zonai
