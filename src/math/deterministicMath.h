#pragma once

#include <cmath>
#include <limits>

namespace zonai
{

// Deterministic two-component Euclidean magnitude for binary64 physics math.
//
// Divide by the larger magnitude before squaring so finite input pairs with
// finite representable results do not overflow/underflow as x*x + y*y would.
// This has stable evaluation order with the engine's non-contracting FP policy.
// The primitive sqrt and target FP environment still require platform CI checks.
[[nodiscard]] inline double deterministicHypot( double x, double y ) noexcept
{
    if( std::isinf( x ) || std::isinf( y ) )
        return std::numeric_limits<double>::infinity();

    if( std::isnan( x ) || std::isnan( y ) )
        return std::numeric_limits<double>::quiet_NaN();

    double major = std::fabs( x );
    double minor = std::fabs( y );

    if( major < minor )
    {
        const double temporary = major;
        major = minor;
        minor = temporary;
    }

    if( minor == 0.0 ) return major;

    const double ratio = minor / major;
    const double magnitude = std::sqrt( 1.0 + ratio * ratio );
    return major * magnitude;
}

} // namespace zonai
