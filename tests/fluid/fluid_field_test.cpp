#include "fluid/fluidField2.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <numbers>
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

void checkCircleSolidBoundary()
{
    fluidField2 field{ 24, 16, 0.5f, { -6.0f, -4.0f } };
    const fluidCircleObstacle2 obstacle{ { 0.0f, 0.0f }, 0.9f, {}, 0.0f };
    std::array<fluidObstacleForce2, 1> feedback{};

    field.step( 1.0f / 60.0f, std::span<const fluidCircleObstacle2>( &obstacle, 1 ), feedback );
    check( field.getSolidCellCount() >= 8 && field.isSolidCell( 12, 8 ),
        "circle obstacle must occupy cells in the MAC grid" );
    check( std::abs( feedback[0].force.x ) < 0.001f && std::abs( feedback[0].force.y ) < 0.001f,
        "stationary circle in stationary fluid must not generate pressure force" );

    const fluidCircleObstacle2 moving{ { 0.0f, 0.0f }, 0.9f, { 1.0f, 0.0f }, 0.0f };
    field.step( 1.0f / 60.0f, std::span<const fluidCircleObstacle2>( &moving, 1 ), feedback, 160 );
    check( std::isfinite( field.getDivergenceRms() ) && field.getDivergenceRms() < 0.5f,
        "moving solid projection must keep fluid divergence bounded" );
    check( std::isfinite( feedback[0].force.x ) && std::isfinite( feedback[0].torque ) &&
        feedback[0].force.x < -0.01f, "moving solid must receive opposite pressure reaction" );
    check( field.sampleVelocity( { 1.25f, 0.0f } ).x > 0.001f,
        "moving solid must perturb neighboring fluid velocity" );

    bool mismatched = false;
    try { field.step( 1.0f / 60.0f, std::span<const fluidCircleObstacle2>( &obstacle, 1 ), {} ); }
    catch( const std::invalid_argument& ) { mismatched = true; }
    check( mismatched, "obstacle feedback buffer mismatch must be rejected" );

    field.step( 1.0f / 60.0f );
    check( field.getSolidCellCount() == 0 && !field.isSolidCell( 12, 8 ),
        "solid occupancy must be refreshed when no obstacle is supplied" );
    field.clear();
    check( field.getSolidCellCount() == 0 && field.getDivergenceRms() == 0.0f,
        "fluid clear must reset pressure, velocity and solid masks" );
}

void checkGeneralShapeObstacles()
{
    constexpr float dt = 1.0f / 60.0f;
    fluidField2 field{ 32, 32, 0.25f, { -4.0f, -4.0f } };
    std::array<fluidObstacleForce2, 1> forces{};

    // 로컬 X 캡슐을 90도 회전: Y 축을 따라 점유하고 X 바깥은 비워야 한다.
    fluidShapeObstacle2 capsule{
        capsule2{ { -1.0f, 0.0f }, { 1.0f, 0.0f }, 0.3f },
        { {}, rot2::FromRadians( 0.5f * std::numbers::pi_v<float> ) },
        {}, {}, 0.0f
    };
    field.step( dt, std::span<const fluidShapeObstacle2>( &capsule, 1 ), forces, 120 );
    check( field.isSolidCell( 16, 19 ) && !field.isSolidCell( 19, 16 ),
        "rotated capsule must use transformed geometry rather than an unrotated bounding box" );
    check( close( field.getDivergenceRms(), 0.0f ) &&
        close( forces[0].force.x, 0.0f ) && close( forces[0].force.y, 0.0f ),
        "stationary rotated capsule in resting fluid must not create pressure force" );

    capsule.linearVelocity = { 1.0f, 0.0f };
    field.step( dt, std::span<const fluidShapeObstacle2>( &capsule, 1 ), forces, 160 );
    check( std::isfinite( field.getDivergenceRms() ) &&
        std::isfinite( forces[0].torque ) && forces[0].force.x < -0.01f,
        "moving rotated capsule must push fluid and feel opposing pressure" );

    // Thin rotated box is a polygon; its exact orientation must affect occupied cells.
    field.clear();
    fluidShapeObstacle2 box{
        MakeBox( { 0.8f, 0.2f } ),
        { {}, rot2::FromRadians( 0.5f * std::numbers::pi_v<float> ) },
        {}, {}, 0.0f
    };
    field.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), forces, 120 );
    check( field.isSolidCell( 16, 18 ) && !field.isSolidCell( 18, 16 ),
        "rotated polygon occupancy must respect transform and narrow sides" );

    box.linearVelocity = { 1.0f, 0.0f };
    field.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), forces, 160 );
    check( forces[0].force.x < -0.01f && std::isfinite( forces[0].torque ),
        "moving polygon must produce a finite opposing pressure force" );

    // 서로 다른 shape들이 같은 frame에 공존하더라도 owner/feedback index를 보존해야 한다.
    field.clear();
    const std::array<fluidShapeObstacle2, 3> mixed{
        fluidShapeObstacle2{ circle2{ {}, 0.45f }, { { -2.0f, 0.0f }, {} }, { -2.0f, 0.0f }, {}, 0.0f },
        fluidShapeObstacle2{ capsule2{ { -0.5f, 0.0f }, { 0.5f, 0.0f }, 0.3f },
            { { 0.0f, 0.0f }, rot2::FromRadians( 0.5f * std::numbers::pi_v<float> ) }, {}, {}, 0.0f },
        fluidShapeObstacle2{ MakeBox( { 0.5f, 0.5f } ), { { 2.0f, 0.0f }, {} }, { 2.0f, 0.0f }, {}, 0.0f }
    };
    std::array<fluidObstacleForce2, 3> mixedForces{};
    field.step( dt, mixed, mixedForces );
    check( field.getSolidCellCount() > 16 &&
        field.isSolidCell( 8, 16 ) && field.isSolidCell( 16, 16 ) && field.isSolidCell( 24, 16 ),
        "circle, capsule and convex polygon must occupy a shared grid" );
    check( close( field.getDivergenceRms(), 0.0f ), "stationary mixed obstacles must leave fluid at rest" );

    fluidShapeObstacle2 unsupported{ segment2{ { -1.0f, 0.0f }, { 1.0f, 0.0f } }, {}, {}, {}, 0.0f };
    field.step( dt, std::span<const fluidShapeObstacle2>( &unsupported, 1 ), forces );
    check( field.getSolidCellCount() == 0, "unsupported line/chain geometry must not create fake solid cells" );
    field.clear();
    check( field.getSolidCellCount() == 0, "mixed obstacle masks must be cleared" );
}

} // namespace

int main()
{
    checkBoundsAndSampling();
    checkPressureProjection();
    checkCircleSolidBoundary();
    checkGeneralShapeObstacles();
}
