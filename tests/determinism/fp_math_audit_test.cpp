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

// The bounded interpolation contract is deliberately tested independently
// of Fluid. Do not replace the production sampleComponent path in R3f.
void auditLerp( std::FILE* stream )
{
    const auto bits = []( float value ) { return std::bit_cast<std::uint32_t>( value ); };
    const auto fromBits = []( std::uint32_t value ) { return std::bit_cast<float>( value ); };
    const float maximum = std::numeric_limits<float>::max();
    const float tiny = std::numeric_limits<float>::denorm_min();

    struct lerpCase
    {
        float a, b, t;
        std::uint32_t expected;
    };
    const std::array<lerpCase, 15> exactCases{ {
        { -0.0f, 1.0f, 0.0f, 0x80000000u },
        { 1.0f, -0.0f, 1.0f, 0x80000000u },
        { -0.0f, 0.0f, 0.5f, 0x80000000u },
        { 0.0f, -0.0f, 0.5f, 0x00000000u },
        { 1.0f, 2.0f, 0.25f, 0x3fa00000u },
        { 2.0f, 1.0f, 0.25f, 0x3fe00000u },
        { -1.0f, 1.0f, 0.5f, 0x00000000u },
        { -maximum, maximum, 0.5f, 0x00000000u },
        { maximum, -maximum, 0.25f, 0x7effffffu },
        { maximum, maximum, 0.5f, 0x7f7fffffu },
        { tiny, tiny, 0.5f, 0x00000001u },
        { tiny, 3.0f * tiny, 0.5f, 0x00000002u },
        { 0.0f, tiny, 0.5f, 0x00000000u },
        { 0.0f, 3.0f * tiny, 0.5f, 0x00000002u },
        // R3e observed identical operands, but std::lerp produced
        // 3f0db109 on Ubuntu and 3f0db10a on Windows.
        { fromBits( 0x3d094d24u ), fromBits( 0x3f12fd58u ),
            fromBits( 0x3f763360u ), 0x3f0db10au }
    } };

    for( std::size_t i = 0; i < exactCases.size(); ++i )
    {
        const auto& item = exactCases[i];
        const float result = deterministicLerp( item.a, item.b, item.t );
        check( bits( result ) == item.expected, "bounded lerp exact-bit fixture" );
        if( stream ) outputFloat( stream, "zonai.lerpExact", i, result );
    }

    const float infinity = std::numeric_limits<float>::infinity();
    const float nan = fromBits( 0x7fc12345u );
    const std::array<std::array<float, 3>, 7> invalidCases{ {
        { infinity, 1.0f, 0.5f }, { 1.0f, -infinity, 0.0f },
        { nan, 1.0f, 0.5f }, { 1.0f, nan, 0.5f },
        { 0.0f, 1.0f, nan }, { 0.0f, 1.0f, -0.01f },
        { 0.0f, 1.0f, 1.01f }
    } };
    for( std::size_t i = 0; i < invalidCases.size(); ++i )
    {
        const auto& item = invalidCases[i];
        const auto result = bits( deterministicLerp( item[0], item[1], item[2] ) );
        check( result == 0x7fc00000u, "bounded lerp canonical invalid-input NaN" );
        if( stream ) std::fprintf( stream, "zonai.lerpInvalid.%02zu\t%08x\n", i, result );
    }

    std::uint64_t seed = 0x6c6572702d723366ull;
    const auto nextBits = [&]() -> std::uint32_t
    {
        seed ^= seed >> 12;
        seed ^= seed << 25;
        seed ^= seed >> 27;
        return static_cast<std::uint32_t>( seed * 2685821657736338717ull );
    };
    const auto nextFinite = [&]()
    {
        auto value = nextBits();
        if( ( value & 0x7f800000u ) == 0x7f800000u ) value &= 0xff7fffffu;
        return fromBits( value );
    };
    constexpr std::array<float, 5> weights{ 0.0f, 0.25f, 0.5f, 0.75f, 1.0f };

    for( std::size_t i = 0; i < 1024; ++i )
    {
        const float a = nextFinite(), b = nextFinite();
        const float t = static_cast<float>( nextBits() >> 8 ) * 0x1p-24f;
        const float low = a < b ? a : b;
        const float high = a < b ? b : a;
        const float result = deterministicLerp( a, b, t );

        check( std::isfinite( result ) && result >= low && result <= high,
            "bounded lerp escaped finite endpoint interval" );
        check( bits( deterministicLerp( a, b, 0.0f ) ) == bits( a ) &&
            bits( deterministicLerp( a, b, 1.0f ) ) == bits( b ),
            "bounded lerp endpoint bits" );

        float previous = a;
        for( const float weight : weights )
        {
            const float sample = deterministicLerp( a, b, weight );
            check( std::isfinite( sample ) && sample >= low && sample <= high,
                "bounded lerp fixed-weight interval" );
            check( a <= b ? sample >= previous : sample <= previous,
                "bounded lerp fixed-weight monotonicity" );
            previous = sample;
        }

        // Independent difference-form double reference. A scale-relative
        // error budget also covers cancellation near zero and subnormals.
        // The cross-OS fingerprint remains strict (no tolerance).
        const double reference = static_cast<double>( a ) +
            ( static_cast<double>( b ) - static_cast<double>( a ) ) * static_cast<double>( t );
        const double scaleA = std::fabs( static_cast<double>( a ) );
        const double scaleB = std::fabs( static_cast<double>( b ) );
        const double scale = scaleA < scaleB ? scaleB : scaleA;
        const double budget = scale * std::numeric_limits<float>::epsilon() +
            static_cast<double>( tiny );
        check( std::fabs( static_cast<double>( result ) - reference ) <= budget,
            "bounded lerp reference error" );

        if( stream ) outputFloat( stream, "zonai.lerpWide", i, result );
    }
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

    // R3i swept-rotation witnesses: identical float operands, divergent
    // std::atan2 outputs on Ubuntu/Windows, identical engine candidate.
    struct sweptAngleCase
    {
        std::uint32_t y, x, expected;
    };
    constexpr std::array<sweptAngleCase, 4> sweptCases{ {
        { 0x3dfd4d3cu, 0x3f7e08ceu, 0x3dfdf3b6u },
        { 0x3e50f43au, 0x3f7a9d10u, 0x3e526e97u },
        { 0x3ec9d0e7u, 0x3f6b45feu, 0x3ecf7200u },
        { 0x3e9a4d99u, 0x3f7418fcu, 0x3e9cbd68u }
    } };
    for( std::size_t i = 0; i < sweptCases.size(); ++i )
    {
        const auto& item = sweptCases[i];
        const float y = std::bit_cast<float>( item.y );
        const float x = std::bit_cast<float>( item.x );
        const float angle = deterministicAtan2( y, x );
        check( std::bit_cast<std::uint32_t>( angle ) == item.expected,
            "R3i swept rotation exact-bit witness" );
        outputFloat( stream, "zonai.sweptAtan2", i, angle );
        outputFloat( stream, "libm.sweptAtan2", i, std::atan2( y, x ) );
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
            auditLerp( nullptr );
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
        auditLerp( stream );
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
