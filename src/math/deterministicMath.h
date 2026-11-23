#pragma once

#include <array>
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

    constexpr std::uint64_t positiveInfinityBits = 0x7ff0000000000000ull;
    constexpr std::uint64_t outwardUlps = 16;
    const std::uint64_t bits = std::bit_cast<std::uint64_t>( magnitude );

    // Positive IEEE-754 binary64 finite encodings are monotonically ordered.
    // Saturating avoids encoding a NaN when expanding a value near DBL_MAX.
    return std::bit_cast<double>( bits >= positiveInfinityBits - outwardUlps
        ? positiveInfinityBits : bits + outwardUlps );
}


// Deterministic atan2 for IEEE-754 binary32 input and output.
//
// Keep the transcendental path independent of the platform C library.
// Use binary64 intermediates and an odd Taylor polynomial on
// [-tan(pi/8), tan(pi/8)]. The pi/4 identity reduces the full [0,1]
// ratio, then symmetry restores the full [-pi,pi] quadrant. The
// 31st-degree remainder is below 1e-14 radians on this reduced interval.
// The compiler must not use fast-math or fused multiply-add here.
[[nodiscard]] inline float deterministicAtan2( float y, float x ) noexcept
{
    static_assert( std::numeric_limits<float>::is_iec559 );
    static_assert( std::numeric_limits<double>::is_iec559 );

    constexpr double pi = 3.14159265358979323846264338327950288;
    constexpr double halfPi = pi * 0.5;
    constexpr double quarterPi = pi * 0.25;
    constexpr double tanPiOverEight = 0.41421356237309504880168872420969808;
    constexpr float piFloat = static_cast<float>( pi );
    constexpr float halfPiFloat = static_cast<float>( halfPi );
    constexpr float quarterPiFloat = static_cast<float>( quarterPi );

    if( std::isnan( x ) || std::isnan( y ) )
        return std::numeric_limits<float>::quiet_NaN();

    const bool negativeX = std::signbit( x );
    const bool negativeY = std::signbit( y );

    // Preserve the signed zero and signed pi conventions of IEEE atan2.
    if( y == 0.0f )
        return negativeX ? ( negativeY ? -piFloat : piFloat ) : y;

    if( x == 0.0f )
        return negativeY ? -halfPiFloat : halfPiFloat;

    if( std::isinf( y ) )
    {
        if( std::isinf( x ) )
        {
            const float angle = negativeX ? 3.0f * quarterPiFloat : quarterPiFloat;
            return negativeY ? -angle : angle;
        }

        return negativeY ? -halfPiFloat : halfPiFloat;
    }

    if( std::isinf( x ) )
    {
        if( negativeX ) return negativeY ? -piFloat : piFloat;
        return negativeY ? -0.0f : 0.0f;
    }

    const double ax = std::fabs( static_cast<double>( x ) );
    const double ay = std::fabs( static_cast<double>( y ) );
    const double minimum = ax < ay ? ax : ay;
    const double maximum = ax < ay ? ay : ax;
    const double ratio = minimum / maximum;
    const bool reduced = ratio > tanPiOverEight;
    const double t = reduced ? ( ratio - 1.0 ) / ( ratio + 1.0 ) : ratio;
    const double square = t * t;

    // atan(t) = t * (1 - t^2/3 + t^4/5 - ... - t^30/31).
    // Coefficients are fixed binary64 constants; Horner evaluation order
    // is intentionally stable and used on both compiler toolchains.
    constexpr std::array<double, 16> coefficients{
        1.0, -1.0 / 3.0, 1.0 / 5.0, -1.0 / 7.0,
        1.0 / 9.0, -1.0 / 11.0, 1.0 / 13.0, -1.0 / 15.0,
        1.0 / 17.0, -1.0 / 19.0, 1.0 / 21.0, -1.0 / 23.0,
        1.0 / 25.0, -1.0 / 27.0, 1.0 / 29.0, -1.0 / 31.0
    };

    double polynomial = coefficients.back();
    for( int i = static_cast<int>( coefficients.size() ) - 2; i >= 0; --i )
        polynomial = coefficients[static_cast<std::size_t>( i )] + square * polynomial;

    double angle = t * polynomial;
    if( reduced ) angle += quarterPi;
    if( ay > ax ) angle = halfPi - angle;
    if( negativeX ) angle = pi - angle;
    if( negativeY ) angle = -angle;
    return static_cast<float>( angle );
}

} // namespace zonai
