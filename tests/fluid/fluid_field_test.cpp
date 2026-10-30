#include "fluid/fluidField2.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <stdexcept>

using namespace zonai;

namespace
{

void check( bool condition, const char* message )
{
    if( condition ) return;
    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}

bool close( float a, float b, float tolerance = 1.0e-4f )
{
    return std::abs( a - b ) <= tolerance;
}

void checkBoundsAndSampling()
{
    bool rejected = false;
    try { fluidField2 invalid{ 0, 4, 1.0f }; }
    catch( const std::invalid_argument& ) { rejected = true; }
    check( rejected, "zero-width fluid grid must be rejected" );

    rejected = false;
    try { fluidField2 invalid{ 4, 4, 0.0f }; }
    catch( const std::invalid_argument& ) { rejected = true; }
    check( rejected, "zero cell size must be rejected" );

    fluidField2 field{ 12, 8, 0.5f, { -3.0f, -2.0f } };
    check( field.getWidth() == 12 && field.getHeight() == 8 && close( field.getCellSize(), 0.5f ),
        "grid dimensions or cell size were not retained" );
    check( close( field.getCellCenter( 0, 0 ).x, -2.75f ) &&
        close( field.getCellCenter( 0, 0 ).y, -1.75f ), "cell center does not respect origin and spacing" );
    check( close( field.getDivergenceRms(), 0.0f ) && close( field.sampleVelocity( { 0.0f, 0.0f } ).x, 0.0f ),
        "new fluid field must be at rest" );

    field.addVelocity( { -2.0f, 0.0f }, { 4.0f, 0.0f }, 1.0f );
    check( field.sampleVelocity( { -2.0f, 0.0f } ).x > 2.0f,
        "localized velocity source must change nearby face values" );
    check( close( field.sampleVelocity( { 20.0f, 0.0f } ).x, 0.0f ),
        "fluid sampling must return zero outside the domain" );
    check( close( field.sampleVelocity( { 1.5f, 0.0f } ).x, 0.0f ),
        "localized source must not instantly affect distant cells" );
    field.addVelocity( { 0.0f, 0.0f }, { 0.0f, 2.0f }, -1.0f );
    field.addVelocity( { 0.0f, 0.0f }, { std::numeric_limits<float>::quiet_NaN(), 0.0f }, 1.0f );
    const vec2 before = field.sampleVelocity( { -2.0f, 0.0f } );
    field.step( 0.0f );
    field.step( std::numeric_limits<float>::quiet_NaN() );
    check( field.sampleVelocity( { -2.0f, 0.0f } ).x == before.x,
        "zero or non-finite time steps must not alter fluid state" );
    field.clear();
    check( close( field.getDivergenceRms(), 0.0f ) &&
        close( field.sampleVelocity( { -2.0f, 0.0f } ).x, 0.0f ), "clear must reset the velocity field" );
}

void checkPressureProjection()
{
    constexpr float dt = 1.0f / 60.0f;
    fluidField2 projected{ 24, 16, 0.5f, { -6.0f, -4.0f } };
    fluidField2 unprojected{ 24, 16, 0.5f, { -6.0f, -4.0f } };

    // 순수 x방향 국소 속도 주입은 발산을 만들어 압력 투영 효과를 검사하기 좋음.
    for( fluidField2* field : { &projected, &unprojected } )
        field->addVelocity( { -2.0f, 0.4f }, { 3.0f, 0.0f }, 1.5f );

    const float before = projected.getDivergenceRms();
    check( before > 0.01f, "pressure test requires a compressible input" );
    projected.step( dt, 120 );
    unprojected.step( dt, 1 );

    const float after = projected.getDivergenceRms();
    check( std::isfinite( after ) && after < before * 0.25f,
        "Jacobi pressure projection must significantly reduce cell divergence" );
    check( after < unprojected.getDivergenceRms(),
        "more projection iterations should reduce divergence on this velocity source" );

    for( int i = 0; i < 120; ++i )
    {
        // 유체 소스는 delta-v. 외력을 받는 동안에도 divergence가 발산하지 않아야 함.
        projected.addVelocity( { -2.5f, 0.2f }, { 2.0f * dt, 0.4f * dt }, 1.6f );
        projected.step( dt, 100 );
    }

    check( std::isfinite( projected.getDivergenceRms() ) && projected.getDivergenceRms() < 0.1f,
        "steady local wind source produced non-finite or excessive residual divergence" );
    const vec2 current = projected.sampleVelocity( { -1.5f, 0.0f } );
    check( IsFinite( current ) && Length( current ) > 0.001f,
        "advected flow must remain nonzero and finite near the wind source" );
    projected.clear();
    check( projected.getDivergenceRms() == 0.0f, "clear after time integration must reset divergence" );
}

} // namespace

int main()
{
    checkBoundsAndSampling();
    checkPressureProjection();
}
