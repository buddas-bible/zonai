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

// R3k candidate contract, independent of rot2 and Fluid production paths.
void auditSinCos( std::FILE* stream )
{
    const auto bits = []( float value ) { return std::bit_cast<std::uint32_t>( value ); };
    const auto fromBits = []( std::uint32_t value ) { return std::bit_cast<float>( value ); };
    struct angleCase
    {
        std::uint32_t angle, cosine, sine;
    };
    // Exact binary32 inputs and rounded high-precision reference values,
    // including the two R3j witnesses and adjacent quadrant boundaries.
    constexpr std::array<angleCase, 19> exactCases{ {
        { 0x00000000u, 0x3f800000u, 0x00000000u },
        { 0x80000000u, 0x3f800000u, 0x80000000u },
        { 0x00000001u, 0x3f800000u, 0x00000001u },
        { 0x80000001u, 0x3f800000u, 0x80000001u },
        { 0x3e0d4fdfu, 0x3f7d90f5u, 0x3e0cdd28u },
        { 0x3e67983bu, 0x3f797b01u, 0x3e65a013u },
        { 0x3f490fdau, 0x3f3504f4u, 0x3f3504f3u },
        { 0x3f490fdbu, 0x3f3504f3u, 0x3f3504f3u },
        { 0x3f490fdcu, 0x3f3504f2u, 0x3f3504f4u },
        { 0x3fc90fdau, 0x33a22169u, 0x3f800000u },
        { 0x3fc90fdbu, 0xb33bbd2eu, 0x3f800000u },
        { 0x3fc90fdcu, 0xb42eef4cu, 0x3f800000u },
        { 0x40490fdau, 0xbf800000u, 0x34222169u },
        { 0x40490fdbu, 0xbf800000u, 0xb3bbbd2eu },
        { 0x40490fdcu, 0xbf800000u, 0xb4aeef4cu },
        { 0x4096cbe4u, 0x324cde2eu, 0xbf800000u },
        { 0x40c90fdau, 0x3f800000u, 0xb4a22169u },
        { 0x40c90fdbu, 0x3f800000u, 0x343bbd2eu },
        { 0xc0c90fdbu, 0x3f800000u, 0xb43bbd2eu }
    } };
    for( std::size_t i = 0; i < exactCases.size(); ++i )
    {
        const auto& item = exactCases[i];
        const auto result = deterministicSinCosBounded( fromBits( item.angle ) );
        const auto rotation = rot2::FromRadians( fromBits( item.angle ) );
        check( bits( rotation.c ) == bits( result.cosine ) &&
            bits( rotation.s ) == bits( result.sine ), "rot2 bounded exact-bit dispatch" );
        check( bits( result.cosine ) == item.cosine && bits( result.sine ) == item.sine,
            "bounded sine/cosine exact-bit fixture" );
        if( stream )
        {
            outputFloat( stream, "zonai.sinCosExact.cos", i, result.cosine );
            outputFloat( stream, "zonai.sinCosExact.sin", i, result.sine );
        }
    }
    // Keep the observed libm witnesses outside the authoritative fingerprint.
    if( stream )
    {
        for( std::size_t i = 4; i < 6; ++i )
        {
            volatile float input = fromBits( exactCases[i].angle );
            const float angle = input;
            outputFloat( stream, "libm.rotationWitness.sin", i - 4, std::sin( angle ) );
            outputFloat( stream, "libm.rotationWitness.cos", i - 4, std::cos( angle ) );
        }
    }

    constexpr std::array<std::uint32_t, 5> invalidCases{
        0x40c90fdcu, 0xc0c90fdcu, 0x7f800000u, 0xff800000u, 0x7fc12345u
    };
    for( std::size_t i = 0; i < invalidCases.size(); ++i )
    {
        const auto result = deterministicSinCosBounded( fromBits( invalidCases[i] ) );
        check( bits( result.cosine ) == 0x7fc00000u && bits( result.sine ) == 0x7fc00000u,
            "bounded sine/cosine canonical invalid-input NaN" );
        if( stream )
        {
            std::fprintf( stream, "zonai.sinCosInvalid.cos.%02zu\t%08x\n", i, bits( result.cosine ) );
            std::fprintf( stream, "zonai.sinCosInvalid.sin.%02zu\t%08x\n", i, bits( result.sine ) );
        }
    }

    const auto verify = [&]( const char* cosKey, const char* sinKey, std::size_t i, float angle )
    {
        const auto result = deterministicSinCosBounded( angle );
        const auto rotation = rot2::FromRadians( angle );
        check( bits( rotation.c ) == bits( result.cosine ) &&
            bits( rotation.s ) == bits( result.sine ), "rot2 bounded dispatch" );
        const auto opposite = deterministicSinCosBounded( -angle );
        check( std::isfinite( result.cosine ) && std::isfinite( result.sine ) &&
            std::abs( result.cosine ) <= 1.0f && std::abs( result.sine ) <= 1.0f,
            "bounded sine/cosine finite range" );
        check( bits( opposite.cosine ) == bits( result.cosine ) &&
            bits( opposite.sine ) == ( bits( result.sine ) ^ 0x80000000u ),
            "bounded sine/cosine even/odd symmetry" );
        const double c = result.cosine, s = result.sine;
        check( std::abs( c * c + s * s - 1.0 ) <= 2.0e-7,
            "bounded sine/cosine rotation norm" );
        // Accuracy check only: the reference libm is not an exact-bit oracle.
        const double referenceCos = std::cos( static_cast<double>( angle ) );
        const double referenceSin = std::sin( static_cast<double>( angle ) );
        const auto close = []( double value, double reference )
        {
            const double tolerance = 2.0 * std::numeric_limits<float>::epsilon() *
                std::abs( reference ) + std::numeric_limits<float>::denorm_min();
            return std::abs( value - reference ) <= tolerance;
        };
        check( close( c, referenceCos ) && close( s, referenceSin ),
            "bounded sine/cosine numerical reference" );
        if( stream )
        {
            outputFloat( stream, cosKey, i, result.cosine );
            outputFloat( stream, sinKey, i, result.sine );
        }
    };

    // The unbounded legacy path is still available. It is deliberately
    // NOT an authoritative cross-platform fingerprint until range reduction
    // is implemented and validated. Test same-platform compatibility.
    constexpr std::array<std::uint32_t, 8> legacyAngles{
        0x40c90fdcu, 0xc0c90fdcu, 0x42c80000u, 0xc2c80000u,
        0x4b800000u, 0xcb800000u, 0x7f7fffffu, 0xff7fffffu
    };
    for( const auto raw : legacyAngles )
    {
        volatile float input = fromBits( raw );
        const float angle = input;
        const auto rotation = rot2::FromRadians( angle );
        check( bits( rotation.c ) == bits( std::cos( angle ) ) &&
            bits( rotation.s ) == bits( std::sin( angle ) ),
            "rot2 legacy large-angle compatibility" );
    }
    for( const auto raw : { 0x7f800000u, 0xff800000u, 0x7fc12345u } )
    {
        volatile float input = fromBits( raw );
        const auto rotation = rot2::FromRadians( input );
        check( std::isnan( rotation.c ) && std::isnan( rotation.s ),
            "rot2 legacy non-finite behavior" );
    }

    constexpr float limit = 0x1.921fb6p+2f;
    for( std::size_t i = 0; i <= 4096; ++i )
    {
        const float angle = static_cast<float>( static_cast<int>( i ) - 2048 ) * ( limit / 2048.0f );
        verify( "zonai.sinCosGrid.cos", "zonai.sinCosGrid.sin", i, angle );
    }

    std::uint64_t seed = 0x73696e636f732d6bull;
    for( std::size_t i = 0; i < 1024; ++i )
    {
        seed ^= seed >> 12;
        seed ^= seed << 25;
        seed ^= seed >> 27;
        const auto raw = static_cast<std::uint32_t>( seed * 2685821657736338717ull );
        const auto magnitude = ( raw & 0x7fffffffu ) % 0x40c90fdcu;
        const float angle = fromBits( magnitude | ( raw & 0x80000000u ) );
        verify( "zonai.sinCosWide.cos", "zonai.sinCosWide.sin", i, angle );
    }
}

// R3o: reconstruct the wind source's actual float time recurrence for
// the 180-tick, eight-World/four-Fluid-substep Sandbox fixture. This is a
// separate diagnostic, not a capture of the source's production locals.
void auditWindSource( std::FILE* stream )
{
    constexpr float timeStep = 1.0f / 60.0f;
    constexpr int worldSubsteps = 8;
    constexpr int fluidSubsteps = 4;
    constexpr int tickCount = 180;
    constexpr float strength = 8.0f;
    constexpr float worldSubstepTime = timeStep / worldSubsteps;
    constexpr float windTimeStep = worldSubstepTime * ( worldSubsteps / fluidSubsteps );
    float windTime = 0.0f;
    bool sawPositive = false, sawNegative = false;
    for( std::size_t i = 0; i < tickCount * fluidSubsteps; ++i )
    {
        windTime += windTimeStep;
        const float angle = 1.7f * windTime;
        // Prevent compile-time folding of the stdlib witness. Keep the
        // engine's operation order identical to rigidBodyDemo::step().
        volatile float input = angle;
        const float legacy = std::sin( input );
        const float candidate = rot2::FromRadians( angle ).s;
        check( std::bit_cast<std::uint32_t>( candidate ) ==
            std::bit_cast<std::uint32_t>( deterministicSinCosBounded( angle ).sine ),
            "R3p wind source must use the bounded rotation path throughout this recurrence" );
        const float legacyDelta = 0.5f * strength * windTimeStep * legacy;
        const float candidateDelta = 0.5f * strength * windTimeStep * candidate;
        // Exact engine candidate results from both R3o platform traces.
        // Keep old libm outputs diagnostic; never assert they must differ.
        struct windWitness { std::size_t index; std::uint32_t angle, sine, delta; };
        constexpr std::array<windWitness, 3> witnesses{ {
            { 49, 0x3eb55554u, 0x3eb190edu, 0x3bbd6764u },
            { 372, 0x40291814u, 0x3ef53d85u, 0x3c02cb7bu },
            { 394, 0x40331149u, 0x3eac831du, 0x3bb80353u }
        } };
        for( const auto& witness : witnesses )
            if( i == witness.index )
                check( std::bit_cast<std::uint32_t>( angle ) == witness.angle &&
                    std::bit_cast<std::uint32_t>( candidate ) == witness.sine &&
                    std::bit_cast<std::uint32_t>( candidateDelta ) == witness.delta,
                    "R3p exact wind source witness" );
        check( std::isfinite( legacyDelta ) && std::isfinite( candidateDelta ),
            "wind source bounded recurrence produced a nonfinite result" );
        sawPositive |= candidate > 0.0f;
        sawNegative |= candidate < 0.0f;
        if( stream )
        {
            outputFloat( stream, "zonai.windSource.time", i, windTime );
            outputFloat( stream, "zonai.windSource.angle", i, angle );
            outputFloat( stream, "zonai.windSource.candidateSin", i, candidate );
            outputFloat( stream, "zonai.windSource.candidateDeltaY", i, candidateDelta );
            outputFloat( stream, "libm.windSource.sin", i, legacy );
            outputFloat( stream, "libm.windSource.deltaY", i, legacyDelta );
        }
    }
    check( sawPositive && sawNegative && windTime > 2.99f && windTime < 3.01f,
        "wind source recurrence did not exercise both oscillation signs" );
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
            auditSinCos( nullptr );
            auditWindSource( nullptr );
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
        auditSinCos( stream );
        auditWindSource( stream );
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
