#pragma once

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>

namespace zonai
{

// Linear interpolation of finite binary32 endpoints for t in [0, 1].
// Explicit endpoint handling preserves their exact bits (including signed zero).
// A binary64 convex combination avoids binary32 overflow in b-a and fixes
// the operation order across standard libraries. Requires round-to-nearest,
// gradual underflow, and the engine's non-contracting FP policy.
// Invalid inputs (non-finite endpoints/weight or weight outside [0,1])
// return a canonical quiet NaN; extrapolation is deliberately unsupported.
[[nodiscard]] inline float deterministicLerp( float a, float b, float t ) noexcept
{
    static_assert( sizeof( float ) == sizeof( std::uint32_t ) );
    static_assert( std::numeric_limits<float>::is_iec559 );
    static_assert( std::numeric_limits<double>::is_iec559 );

    if( !std::isfinite( a ) || !std::isfinite( b ) ||
        !std::isfinite( t ) || t < 0.0f || t > 1.0f )
        return std::bit_cast<float>( 0x7fc00000u );

    if( t == 0.0f ) return a;
    if( t == 1.0f ) return b;
    if( a == b ) return a;

    const double weight = static_cast<double>( t );
    const double left = ( 1.0 - weight ) * static_cast<double>( a );
    const double right = weight * static_cast<double>( b );
    return static_cast<float>( left + right );
}

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


struct deterministicSinCosResult
{
    float cosine;
    float sine;
};

namespace detail
{

[[nodiscard]] inline deterministicSinCosResult evaluateReducedSinCos( double reduced, int quadrant, bool negative ) noexcept
{
    const double square = reduced * reduced;

    // Taylor polynomials on approximately [-pi/4, pi/4].
    // Fixed binary64 Horner order, no FMA or fast-math.
    constexpr std::array<double, 8> sineCoefficients{
        -1.0 / 6.0, 1.0 / 120.0, -1.0 / 5040.0, 1.0 / 362880.0,
        -1.0 / 39916800.0, 1.0 / 6227020800.0,
        -1.0 / 1307674368000.0, 1.0 / 355687428096000.0
    };
    constexpr std::array<double, 8> cosineCoefficients{
        -1.0 / 2.0, 1.0 / 24.0, -1.0 / 720.0, 1.0 / 40320.0,
        -1.0 / 3628800.0, 1.0 / 479001600.0,
        -1.0 / 87178291200.0, 1.0 / 20922789888000.0
    };
    double sinePolynomial = sineCoefficients.back();
    double cosinePolynomial = cosineCoefficients.back();
    for( int i = static_cast<int>( sineCoefficients.size() ) - 2; i >= 0; --i )
    {
        sinePolynomial = sineCoefficients[static_cast<std::size_t>( i )] + square * sinePolynomial;
        cosinePolynomial = cosineCoefficients[static_cast<std::size_t>( i )] + square * cosinePolynomial;
    }
    const double s = reduced * ( 1.0 + square * sinePolynomial );
    const double c = 1.0 + square * cosinePolynomial;
    double cosine = c, sine = s;
    switch( quadrant & 3 )
    {
    case 1: cosine = -s; sine = c; break;
    case 2: cosine = -c; sine = -s; break;
    case 3: cosine = s; sine = -c; break;
    default: break;
    }
    if( negative ) sine = -sine;
    return { static_cast<float>( cosine ), static_cast<float>( sine ) };
}

} // namespace detail

// Deterministic bounded sine/cosine for finite binary32 radians in
// [-float(2*pi), +float(2*pi)]. Invalid inputs return canonical qNaNs.
[[nodiscard]] inline deterministicSinCosResult deterministicSinCosBounded( float radians ) noexcept
{
    static_assert( sizeof( float ) == sizeof( std::uint32_t ) );
    static_assert( std::numeric_limits<float>::is_iec559 );
    static_assert( std::numeric_limits<double>::is_iec559 );

    constexpr float limit = 0x1.921fb6p+2f;
    constexpr double twoOverPi = 0.636619772367581343075535053490057448;
    // Split pi/2 so cancellation near quadrant boundaries retains precision.
    constexpr double halfPiHigh = 0x1.921fb54400000p+0;
    constexpr double halfPiLow = 6.07710050650619224932e-11;

    if( !std::isfinite( radians ) || radians < -limit || radians > limit )
    {
        const float nan = std::bit_cast<float>( 0x7fc00000u );
        return { nan, nan };
    }
    if( radians == 0.0f ) return { 1.0f, radians };

    const double angle = std::fabs( static_cast<double>( radians ) );
    const int quadrant = static_cast<int>( angle * twoOverPi + 0.5 );
    const double reduced = ( angle - quadrant * halfPiHigh ) - quadrant * halfPiLow;
    return detail::evaluateReducedSinCos( reduced, quadrant, radians < 0.0f );
}

// Full binary32 candidate. Retain the exact bounded path for small angles;
// larger finite values use integer Payne-Hanek style argument reduction.
// The table is floor((2/pi)*2^256), least-significant 32-bit limb first.
// No floating-point quotient of a huge angle and no platform libm reduction.
[[nodiscard]] inline deterministicSinCosResult deterministicSinCos( float radians ) noexcept
{
    constexpr float limit = 0x1.921fb6p+2f;
    if( !std::isfinite( radians ) )
    {
        const float nan = std::bit_cast<float>( 0x7fc00000u );
        return { nan, nan };
    }
    if( radians >= -limit && radians <= limit )
        return deterministicSinCosBounded( radians );

    const auto raw = std::bit_cast<std::uint32_t>( radians );
    const bool negative = ( raw & 0x80000000u ) != 0;
    const int exponent = static_cast<int>( ( raw >> 23 ) & 0xffu );
    const std::uint32_t significand = ( raw & 0x007fffffu ) | 0x00800000u;
    // abs(x) = significand * 2^(exponent-150). Since |x| > 2*pi,
    // exponent is in [129,254] and the binary point lies at bit [152,277].
    const int shift = 406 - exponent;
    constexpr std::array<std::uint32_t, 8> twoOverPi{
        0xdebbc561u, 0xfe5163abu, 0x3c439041u, 0xdb629599u,
        0xf534ddc0u, 0xfc2757d1u, 0x4e441529u, 0xa2f9836eu
    };
    std::array<std::uint32_t, 9> product{};
    std::uint64_t carry = 0;
    for( std::size_t i = 0; i < twoOverPi.size(); ++i )
    {
        const std::uint64_t value = static_cast<std::uint64_t>( significand ) * twoOverPi[i] + carry;
        product[i] = static_cast<std::uint32_t>( value );
        carry = value >> 32;
    }
    product.back() = static_cast<std::uint32_t>( carry );

    const auto bit = [&]( int index )
    {
        return ( product[static_cast<std::size_t>( index / 32 )] >> ( index % 32 ) ) & 1u;
    };
    const bool roundUp = bit( shift - 1 ) != 0;
    const int quadrant = static_cast<int>( ( bit( shift ) | ( bit( shift + 1 ) << 1 ) ) +
        static_cast<unsigned>( roundUp ) ) & 3;

    // Recover the magnitude of the signed fractional distance to the
    // nearest integer. Two's complement avoids subtracting nearly equal
    // doubles when the fractional part is close to one.
    if( roundUp )
    {
        carry = 1;
        for( auto& limb : product )
        {
            const std::uint64_t value = static_cast<std::uint64_t>( ~limb ) + carry;
            limb = static_cast<std::uint32_t>( value );
            carry = value >> 32;
        }
    }

    double fraction = 0.0;
    const int wholeLimbs = shift / 32;
    for( int i = 0; i < wholeLimbs; ++i )
        fraction = ( fraction + product[static_cast<std::size_t>( i )] ) * 0x1p-32;

    const int remainingBits = shift % 32;
    if( remainingBits != 0 )
    {
        const std::uint32_t divisor = 1u << remainingBits;
        const std::uint32_t high = product[static_cast<std::size_t>( wholeLimbs )] & ( divisor - 1u );
        fraction = ( fraction + high ) / static_cast<double>( divisor );
    }

    constexpr double halfPi = 0x1.921fb54442d18p+0;
    const double reduced = ( roundUp ? -fraction : fraction ) * halfPi;
    return detail::evaluateReducedSinCos( reduced, quadrant, negative );
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
    constexpr float threeQuarterPiFloat = static_cast<float>( 3.0 * quarterPi );

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
            const float angle = negativeX ? threeQuarterPiFloat : static_cast<float>( quarterPi );
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
