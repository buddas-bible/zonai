#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include "math/deterministicMath.h"

using namespace zonai;

namespace
{

void check( bool valid, const char* message )
{
    if( valid ) return;
    std::fprintf( stderr, "deterministic atan2 failed: %s\n", message );
    std::exit( EXIT_FAILURE );
}

[[nodiscard]] std::uint32_t orderedFloatBits( float value )
{
    const auto bits = std::bit_cast<std::uint32_t>( value );
    return bits & 0x80000000u ? ~bits : bits | 0x80000000u;
}

void checkPair( float y, float x )
{
    const float actual = deterministicAtan2( y, x );
    const float oracle = std::atan2( y, x );

    if( std::isnan( y ) || std::isnan( x ) )
    {
        check( std::isnan( actual ), "NaN input should return NaN" );
        return;
    }

    check( std::isfinite( actual ), "finite angle required" );
    check( actual >= -3.141593f && actual <= 3.141593f,
        "result outside [-pi,pi]" );
    check( std::signbit( actual ) == std::signbit( y ),
        "atan2 output sign must match y, including signed zero" );

    if( actual == 0.0f )
        check( std::bit_cast<std::uint32_t>( actual ) ==
            std::bit_cast<std::uint32_t>( oracle ),
            "signed zero must agree with the IEEE atan2 convention" );

    const std::uint32_t actualBits = orderedFloatBits( actual );
    const std::uint32_t oracleBits = orderedFloatBits( oracle );
    const auto distance = actualBits > oracleBits
        ? actualBits - oracleBits : oracleBits - actualBits;

    // The reference std::atan2f itself differs by 1 ULP across MSVC/GCC.
    // This is only a precision check. Bitwise determinism is tested in CI
    // against the new function's own bytes across both platforms.
    if( distance > 2 )
    {
        std::fprintf( stderr, "accuracy failed y=0x%08x x=0x%08x"
            " got=0x%08x std=0x%08x distance=%u\n",
            std::bit_cast<std::uint32_t>( y ),
            std::bit_cast<std::uint32_t>( x ),
            std::bit_cast<std::uint32_t>( actual ),
            std::bit_cast<std::uint32_t>( oracle ), distance );
        check( false, "atan2 exceeds 2 ULP accuracy bound" );
    }

    if( !std::isnan( y ) )
        check( std::bit_cast<std::uint32_t>( deterministicAtan2( -y, x ) ) ==
            std::bit_cast<std::uint32_t>( -actual ),
            "reflection across x axis must be bitwise symmetric" );
}

[[nodiscard]] std::uint32_t nextBits( std::uint64_t& state )
{
    state ^= state >> 12;
    state ^= state << 25;
    state ^= state >> 27;
    return static_cast<std::uint32_t>( state * 2685821657736338717ull );
}

void checkSpecialValues()
{
    constexpr std::array<float, 6> values{
        0.0f, -0.0f, 1.0f, -1.0f,
        std::numeric_limits<float>::infinity(),
        -std::numeric_limits<float>::infinity()
    };

    for( float y : values )
        for( float x : values )
            checkPair( y, x );

    constexpr float subnormal = std::numeric_limits<float>::denorm_min();
    constexpr float maximum = std::numeric_limits<float>::max();
    constexpr float minimum = std::numeric_limits<float>::min();

    for( float y : { subnormal, -subnormal, minimum, -minimum, maximum, -maximum } )
        for( float x : { subnormal, -subnormal, minimum, -minimum, maximum, -maximum } )
            checkPair( y, x );

    const float nan = std::numeric_limits<float>::quiet_NaN();
    checkPair( nan, 1.0f );
    checkPair( 1.0f, nan );
    checkPair( nan, nan );

    check( std::bit_cast<std::uint32_t>( deterministicAtan2( 0.0f, 1.0f ) ) ==
        0x00000000u, "positive zero" );
    check( std::bit_cast<std::uint32_t>( deterministicAtan2( -0.0f, 1.0f ) ) ==
        0x80000000u, "negative zero" );
    check( deterministicAtan2( 0.0f, -0.0f ) > 3.14f,
        "negative x zero must use positive pi" );
    check( deterministicAtan2( -0.0f, -0.0f ) < -3.14f,
        "negative signed zero pair must use negative pi" );
}

void checkRecordedInputs()
{
    // Proven cross-platform std::atan2f divergences from Spline Rail PR #335.
    constexpr std::array<std::array<std::uint32_t, 2>, 3> inputBits{ {
        { 0x3c216d9au, 0x3f7ffcd3u },
        { 0xbd0ffd6bu, 0x3f7fd77eu },
        { 0x3f7d908fu, 0xbe0ce895u }
    } };

    for( const auto& pair : inputBits )
        checkPair( std::bit_cast<float>( pair[0] ),
            std::bit_cast<float>( pair[1] ) );
}

void checkCoverage()
{
    for( int i = -160; i <= 160; ++i )
    {
        const float angle = static_cast<float>( i ) / 160.0f;
        checkPair( angle, 1.0f );
        checkPair( 1.0f, angle );
        checkPair( angle, -1.0f );
        checkPair( -1.0f, angle );
        checkPair( angle, angle );
    }

    std::uint64_t state = 0x6d6174682d617432ull;
    for( int i = 0; i < 50000; ++i )
    {
        const float y = std::bit_cast<float>( nextBits( state ) );
        const float x = std::bit_cast<float>( nextBits( state ) );
        checkPair( y, x );
    }
}

} // namespace

int main()
{
    static_assert( sizeof( float ) == 4 );
    static_assert( std::numeric_limits<float>::is_iec559 );
    static_assert( std::numeric_limits<double>::is_iec559 );

    checkSpecialValues();
    checkRecordedInputs();
    checkCoverage();

    std::puts( "deterministic atan2 sign, accuracy, and range tests passed" );
    return EXIT_SUCCESS;
}
