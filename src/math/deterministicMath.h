#pragma once

#include <bit>
#include <cmath>
#include <cstdint>
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


    
// For broad-phase and CCD *upper bounds*, unlike ordinary Euclidean lengths.
// The scaled hypot result may round below the exact distance; advance 16
// representable binary64 values toward +infinity to protect conservative
// candidate/rate bounds. This guard does not replace geometric float-error
// padding or a proof for unbounded inputs; regression oracles remain required.
[[nodiscard]] inline double deterministicHypotUpperBound( double x, double y ) noexcept
{
    static_assert( sizeof( double ) == sizeof( std::uint64_t ) );
    static_assert( std::numeric_limits<double>::is_iec559 );

    const double magnitude = deterministicHypot( x, y );
    if( !std::isfinite( magnitude ) || magnitude == 0.0 )
        return magnitude;

    constexpr std::uint64_t POSITIVE_INFINITY_BITS = 0x7ff0000000000000ull;
    constexpr std::uint64_t OUTWARD_ULPS = 16;
    const std::uint64_t bits = std::bit_cast<std::uint64_t>( magnitude );

    // Positive IEEE-754 binary64 finite encodings are monotonically ordered.
    // Saturating avoids encoding a NaN when expanding a value near DBL_MAX.
    return std::bit_cast<double>( bits >= POSITIVE_INFINITY_BITS - OUTWARD_ULPS
        ? POSITIVE_INFINITY_BITS : bits + OUTWARD_ULPS );
}

} // namespace zonai
