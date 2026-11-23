#include <array>
#include <bit>
#include <cfenv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>

#include "math/deterministicMath.h"
#include "math/rot2.h"
#include "math/vec2.h"

using namespace zonai;

namespace
{

static_assert( sizeof( float ) == sizeof( std::uint32_t ) );
static_assert( sizeof( double ) == sizeof( std::uint64_t ) );
static_assert( std::numeric_limits<float>::is_iec559 );
static_assert( std::numeric_limits<double>::is_iec559 );

void check( bool condition, const char* description )
{
    if( condition ) return;

    std::fprintf( stderr, "FP math audit failed: %s\n", description );
    std::exit( EXIT_FAILURE );
}

[[nodiscard]] float nonContractedMultiplyAdd()
{
    // a*b rounds to 1 + 2^-22, then adding c yields exactly +0.
    // Fusing would leave the residual 2^-46. Volatile inputs prevent
    // compile-time constant folding; compiler FP policy controls contraction.
    volatile float inputA = std::bit_cast<float>( 0x3f800001u );
    volatile float inputB = std::bit_cast<float>( 0x3f800001u );
    volatile float inputC = std::bit_cast<float>( 0xbf800002u );
    const float a = inputA;
    const float b = inputB;
    const float c = inputC;
    return a * b + c;
}

void validateEnvironment()
{
#if defined(__FAST_MATH__)
    check( false, "FAST_MATH must not be enabled in physics builds" );
#endif
    check( std::fegetround() == FE_TONEAREST, "non-default FP rounding mode" );
    check( std::bit_cast<std::uint32_t>( nonContractedMultiplyAdd() ) == 0u,
        "multiply-add was fused or evaluated with unexpected precision" );
}

void outputFloat( std::FILE* stream, const char* operation, std::size_t sample, float value )
{
    check( std::isfinite( value ), "non-finite float result in audit sample" );

    std::fprintf( stream, "%s.%02zu\t%08x\n",
        operation, sample, std::bit_cast<std::uint32_t>( value ) );
}

void outputDouble( std::FILE* stream, const char* operation, std::size_t sample, double value )
{
    check( std::isfinite( value ), "non-finite double result in audit sample" );

    const std::uint64_t bits = std::bit_cast<std::uint64_t>( value );
    std::fprintf( stream, "%s.%02zu\t%016llx\n",
        operation, sample, static_cast<unsigned long long>( bits ) );
}

void emitFingerprint( std::FILE* stream )
{
    // Use exact source IEEE-754 bits rather than decimal angle parsing.
    constexpr std::array<std::uint32_t, 16> angleBits{
        0x00000000u, 0x80000000u,
        0x3dcccccdU, 0xbdcccccdU,
        0x3f490fdbU, 0x3fc90fdbU,
        0x40490fdbU, 0x40c90fdbU,
        0x41200000U, 0xc1200000U,
        0x3eaaaaabU, 0xbeaaaaabU,
        0x3f000000U, 0xbf000000U,
        0x40a00000U, 0x40c00000U
    };

    outputFloat( stream, "fp.mulAddSeparate", 0, nonContractedMultiplyAdd() );

    for( std::size_t i = 0; i < angleBits.size(); ++i )
    {
        const float angle = std::bit_cast<float>( angleBits[i] );
        const rot2 rotation = rot2::FromRadians( angle );
        const vec2 input{ 0.75f, -0.375f };
        const vec2 rotated = Rotate( rotation, input );

        // The rot2 output is authoritative physics state. The library
        // functions below are potential nondeterminism sources, not trusted
        // replacement implementations.
        outputFloat( stream, "rot2.cos", i, rotation.c );
        outputFloat( stream, "rot2.sin", i, rotation.s );
        outputFloat( stream, "rotated.x", i, rotated.x );
        outputFloat( stream, "rotated.y", i, rotated.y );
        outputFloat( stream, "zonai.atan2f", i,
            deterministicAtan2( angle, 0.75f ) );
        outputFloat( stream, "libm.atan2f", i, std::atan2( angle, 0.75f ) );
        outputDouble( stream, "libm.atan2", i,
            std::atan2( static_cast<double>( angle ), 0.75 ) );
        outputDouble( stream, "zonai.hypot", i,
            deterministicHypot( static_cast<double>( angle ), 0.75 ) );
        outputDouble( stream, "zonai.hypotUpper", i,
            deterministicHypotUpperBound( static_cast<double>( angle ), 0.75 ) );
        // Keep the known platform-dependent libm sample as a research
        // diagnostic; R2a's strict gate must not hide existing differences.
        outputDouble( stream, "libm.hypot", i,
            std::hypot( static_cast<double>( angle ), 0.75 ) );
        outputFloat( stream, "sqrtf", i, std::sqrt( angle * angle + 0.5625f ) );
    }

    // Exact float pairs recovered from the first Spline Rail cross-platform
    // trace: closed-loop tick 1 relative angle, open tick 4 tangent angle,
    // and closed-loop tick 2 tangent angle. Capture known libm divergence
    // without treating it as an acceptable physics difference.
    constexpr std::array<std::array<std::uint32_t, 2>, 3> railAtan2InputBits{ {
        { 0x3c216d9au, 0x3f7ffcd3u },
        { 0xbd0ffd6bu, 0x3f7fd77eu },
        { 0x3f7d908fu, 0xbe0ce895u }
    } };

    for( std::size_t i = 0; i < railAtan2InputBits.size(); ++i )
    {
        const float y = std::bit_cast<float>( railAtan2InputBits[i][0] );
        const float x = std::bit_cast<float>( railAtan2InputBits[i][1] );
        outputFloat( stream, "zonai.railAtan2", i,
            deterministicAtan2( y, x ) );
        outputFloat( stream, "libm.railAtan2", i, std::atan2( y, x ) );
    }

    // Broad raw-bit sample, not only the three known libm failures.
    std::uint64_t atan2Seed = 0x6d6174682d617432ull;
    const auto nextAtanBits = [&]() -> std::uint32_t
    {
        atan2Seed ^= atan2Seed >> 12;
        atan2Seed ^= atan2Seed << 25;
        atan2Seed ^= atan2Seed >> 27;
        return static_cast<std::uint32_t>( atan2Seed * 2685821657736338717ull );
    };
    for( std::size_t i = 0; i < 256; ++i )
    {
        std::uint32_t yBits = nextAtanBits(), xBits = nextAtanBits();
        // Keep the large corpus finite; NaN/Inf and signed-zero semantics
        // are checked separately by deterministicAtan2Tests.
        if( ( yBits & 0x7f800000u ) == 0x7f800000u ) yBits &= 0xff7fffffu;
        if( ( xBits & 0x7f800000u ) == 0x7f800000u ) xBits &= 0xff7fffffu;
        const float y = std::bit_cast<float>( yBits );
        const float x = std::bit_cast<float>( xBits );
        outputFloat( stream, "zonai.atan2Wide", i, deterministicAtan2( y, x ) );
    }

    // Read-only angle wrap audit. Do not replace the solver's remainder
    // until we know whether its finite-domain results differ across OSes.
    constexpr std::array<float, 16> angleWrapInputs{
        0.0f, -0.0f, 0.1f, -0.1f,
        std::numbers::pi_v<float>, -std::numbers::pi_v<float>,
        2.0f * std::numbers::pi_v<float>, -2.0f * std::numbers::pi_v<float>,
        3.0f * std::numbers::pi_v<float>, -3.0f * std::numbers::pi_v<float>,
        0.5f * std::numbers::pi_v<float>, -0.5f * std::numbers::pi_v<float>,
        10.0f, -10.0f, 1000.0f, -1000.0f
    };
    for( std::size_t i = 0; i < angleWrapInputs.size(); ++i )
    {
        outputFloat( stream, "libm.angleRemainder", i,
            std::remainder( angleWrapInputs[i],
                2.0f * std::numbers::pi_v<float> ) );
    }

    const std::array<float, 5> rangeInputs{ 0.0f, 0.125f, 0.25f, 0.5f, 0.875f };
    for( std::size_t i = 0; i < rangeInputs.size(); ++i )
    {
        const double x = rangeInputs[i];
        outputDouble( stream, "libm.asin", i, std::asin( x ) );
        outputDouble( stream, "libm.acos", i, std::acos( x ) );
    }

    check( std::ferror( stream ) == 0, "cannot write deterministic audit output" );
}

} // namespace

int main( int argc, char** argv )
{
    try
    {
        validateEnvironment();

        if( argc == 1 )
        {
            std::puts( "FP math contract audit passed" );
            return EXIT_SUCCESS;
        }

        if( argc != 3 || std::string( argv[1] ) != "--emit" )
        {
            std::fprintf( stderr, "Usage: fpMathAuditTests [--emit OUTPUT.tsv]\n" );
            return EXIT_FAILURE;
        }

        std::FILE* stream = std::fopen( argv[2], "wb" );
        if( stream == nullptr ) throw std::runtime_error( "cannot create FP audit output" );

        emitFingerprint( stream );
        if( std::fclose( stream ) != 0 )
            throw std::runtime_error( "cannot close FP audit output" );

        std::puts( "FP math fingerprint emitted" );
        return EXIT_SUCCESS;
    }
    catch( const std::exception& error )
    {
        std::fprintf( stderr, "FP math audit error: %s\n", error.what() );
        return EXIT_FAILURE;
    }
}
