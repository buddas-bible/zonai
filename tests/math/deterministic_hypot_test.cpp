#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <utility>

#include "math/deterministicMath.h"

using namespace zonai;

namespace
{

void check( bool condition, const char* description )
{
    if( condition ) return;
    std::fprintf( stderr, "deterministic hypot failed: %s\n", description );
    std::exit( EXIT_FAILURE );
}

[[nodiscard]] std::uint64_t nextRandom( std::uint64_t& state )
{
    state ^= state >> 12;
    state ^= state << 25;
    state ^= state >> 27;
    return state * 2685821657736338717ull;
}

void checkPair( double x, double y )
{
    const double actual = deterministicHypot( x, y );
    const double oracle = std::hypot( x, y );

    check( !std::isnan( actual ), "finite arguments produced NaN" );
    check( actual >= std::max( std::fabs( x ), std::fabs( y ) ) ||
        std::isinf( actual ), "hypotenuse smaller than input magnitude" );

    const double transposed = deterministicHypot( y, x );
    const double reflected = deterministicHypot( -x, -y );

    check( std::bit_cast<std::uint64_t>( actual ) ==
        std::bit_cast<std::uint64_t>( transposed ), "operand symmetry" );
    check( std::bit_cast<std::uint64_t>( actual ) ==
        std::bit_cast<std::uint64_t>( reflected ), "sign symmetry" );

    if( std::isinf( oracle ) )
    {
        check( std::isinf( actual ), "overflow semantics differ from std::hypot" );
        return;
    }

    check( std::isfinite( actual ), "unexpected overflow for representable result" );

    // Reference libm can vary by one ULP across supported platforms.
    // ULP distance is only an accuracy check; it is NOT the determinism gate.
    const std::uint64_t expectedBits = std::bit_cast<std::uint64_t>( oracle );
    const std::uint64_t actualBits = std::bit_cast<std::uint64_t>( actual );
    const std::uint64_t distance = expectedBits > actualBits
        ? expectedBits - actualBits : actualBits - expectedBits;

    check( distance <= 4, "scaled result exceeds 4 ULP from std::hypot" );
}

void testSpecialValues()
{
    constexpr double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();

    check( deterministicHypot( -0.0, -0.0 ) == 0.0 &&
        !std::signbit( deterministicHypot( -0.0, -0.0 ) ),
        "zero result must be positive" );
    check( std::isinf( deterministicHypot( inf, nan ) ),
        "infinity must dominate NaN" );
    check( std::isinf( deterministicHypot( nan, -inf ) ),
        "infinity must dominate NaN regardless of order" );
    check( std::isnan( deterministicHypot( nan, 1.0 ) ),
        "NaN with finite input must produce NaN" );
}

void testScaleAndAccuracy()
{
    constexpr double minimum = std::numeric_limits<double>::min();
    constexpr double subnormal = std::numeric_limits<double>::denorm_min();
    constexpr double maximum = std::numeric_limits<double>::max();

    const std::array<std::pair<double, double>, 20> cases{ {
        { 0.0, 0.0 }, { -0.0, -0.0 },
        { 3.0, 4.0 }, { -3.0, 4.0 },
        { 0.75, static_cast<double>( 0.1f ) },
        { subnormal, 0.0 }, { subnormal, subnormal },
        { minimum, subnormal }, { minimum, minimum },
        { maximum, 0.0 }, { maximum, maximum },
        { maximum * 0.5, maximum * 0.5 },
        { maximum * 0.5, subnormal },
        { 1e308, 1e-308 }, { 1e-308, 1e-308 },
        { 1e200, 1e200 }, { 1e-200, 1e-200 },
        { 1e-300, 1e300 }, { 1e-300, 1e-200 },
        { -subnormal, subnormal }
    } };

    for( const auto& [x, y] : cases )
        checkPair( x, y );

    std::uint64_t seed = 0x6e6f6e6465746572ull;

    for( int i = 0; i < 20000; ++i )
    {
        // Exponent mask excludes NaN/Inf; wide range includes subnormals,
        // large finite values and pairs with extreme scale differences.
        const double x = std::bit_cast<double>( nextRandom( seed ) & 0xffefffffffffffffull );
        const double y = std::bit_cast<double>( nextRandom( seed ) & 0xffefffffffffffffull );
        checkPair( x, y );
    }
}

} // namespace

int main()
{
    static_assert( sizeof( double ) == sizeof( std::uint64_t ) );
    static_assert( std::numeric_limits<double>::is_iec559 );

    testSpecialValues();
    testScaleAndAccuracy();
    std::puts( "deterministic hypot accuracy and range tests passed" );
    return EXIT_SUCCESS;
}
