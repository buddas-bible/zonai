#include <array>
#include <bit>
#include <cfenv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
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
        outputFloat( stream, "libm.atan2f", i, std::atan2( angle, 0.75f ) );
        outputDouble( stream, "libm.atan2", i,
            std::atan2( static_cast<double>( angle ), 0.75 ) );
        outputDouble( stream, "zonai.hypot", i,
            deterministicHypot( static_cast<double>( angle ), 0.75 ) );
        // Keep the known platform-dependent libm sample as a research
        // diagnostic; R2a's strict gate must not hide existing differences.
        outputDouble( stream, "libm.hypot", i,
            std::hypot( static_cast<double>( angle ), 0.75 ) );
        outputFloat( stream, "sqrtf", i, std::sqrt( angle * angle + 0.5625f ) );
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
