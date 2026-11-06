#include "fluid/fluidField2.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>

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

void checkStepTimingProfiler()
{
    constexpr float dt = 1.0f / 60.0f;
    fluidField2 field{ 24, 16, 0.5f, { -6.0f, -4.0f } };
    check( !field.isStepTimingEnabled() && field.getStepTiming().total == 0.0,
        "field stage timing must be off by default" );
    field.setCutCellProjectionEnabled( true );
    field.setAnalyticCircleCutCellEnabled( true );
    field.setStepTimingEnabled( true );
    field.setTracerConcentration( 8, 8, 3.0f );
    fluidShapeObstacle2 circle{ circle2{ {}, 0.8f }, { { -0.4f, 0.0f }, {} },
        { -0.4f, 0.0f }, { 0.6f, 0.0f }, 0.0f };
    std::array<fluidObstacleForce2, 1> reaction{};
    field.step( dt, std::span<const fluidShapeObstacle2>( &circle, 1 ), reaction );
    const fluidStepTiming2 first = field.getStepTiming();
    check( first.total > 0.0 && first.occupancy >= 0.0 && first.geometry > 0.0 &&
        first.advection > 0.0 && first.pressure > 0.0 && first.tracer >= 0.0 &&
        first.swept >= 0.0 && std::isfinite( first.accounted() ) &&
        first.accounted() <= first.total + 0.1,
        "opt-in timings must cover the fluid step without double-counting phases" );

    circle.transform.position.x += dt * circle.linearVelocity.x;
    circle.center = circle.transform.position;
    field.step( dt, std::span<const fluidShapeObstacle2>( &circle, 1 ), reaction );
    const fluidStepTiming2 second = field.getStepTiming();
    check( field.getSweptFaceFlux().usedSweptFlux && second.swept > 0.0 &&
        second.total >= second.accounted() - 0.1,
        "moving obstacle must record swept cost in the latest-step profile" );

    fluidStepTiming2 frame = first;
    frame.accumulate( second );
    check( std::abs( frame.total - first.total - second.total ) < 1.0e-6 &&
        std::abs( frame.geometry - first.geometry - second.geometry ) < 1.0e-6 &&
        frame.accounted() <= frame.total + 0.2,
        "accumulated phase stats must sum multiple fluid substeps" );

    field.setStepTimingEnabled( false );
    check( field.getStepTiming().total == 0.0,
        "turning off profiling must discard stale duration values" );
    field.step( dt, std::span<const fluidShapeObstacle2>( &circle, 1 ), reaction );
    check( field.getStepTiming().total == 0.0,
        "normal field steps must not publish profiling values" );
    field.setStepTimingEnabled( true );
    field.step( dt, std::span<const fluidShapeObstacle2>( &circle, 1 ), reaction );
    field.clear();
    check( field.isStepTimingEnabled() && field.getStepTiming().total == 0.0 &&
        field.getStepTiming().accounted() == 0.0,
        "clear must retain the profiler choice and reset its latest observation" );
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

void checkDiscreteMomentumExchange()
{
    fluidField2 field{ 12, 10, 0.5f, { -3.0f, -2.5f } };
    const vec2 requested{ 1.25f, -0.75f };
    const vec2 before = field.getFluidMomentum();
    const vec2 applied = field.applyMomentumImpulse( { 0.0f, 0.0f }, requested, 1.3f );
    const vec2 after = field.getFluidMomentum();

    check( Length( applied - requested ) < 2.0e-5f,
        "normalized MAC deposition must deliver the requested impulse when faces are available" );
    check( Length( ( after - before ) - applied ) < 2.0e-5f,
        "actual reported impulse must equal the measured fluid MAC momentum change" );

    const vec2 unchanged = field.getFluidMomentum();
    check( LengthSquared( field.applyMomentumImpulse( { 500.0f, 500.0f }, requested, 0.3f ) ) == 0.0f &&
        LengthSquared( field.applyMomentumImpulse( {}, requested, -1.0f ) ) == 0.0f &&
        LengthSquared( field.applyMomentumImpulse( {},
            { std::numeric_limits<float>::quiet_NaN(), 1.0f }, 1.0f ) ) == 0.0f &&
        Length( field.getFluidMomentum() - unchanged ) < 1.0e-6f,
        "invalid or unsupported deposition must leave momentum untouched" );

    // U-face에만 닿는 kernel이면 U에만 충격량을 전달하고 V는 전달하지 않아야 한다.
    fluidField2 singleFace{ 8, 8, 1.0f, { -4.0f, -4.0f } };
    const vec2 oneComponent = singleFace.applyMomentumImpulse( { 0.0f, -0.5f }, { 2.0f, 3.0f }, 0.2f );
    check( close( oneComponent.x, 2.0f ) && oneComponent.y == 0.0f &&
        Length( singleFace.getFluidMomentum() - vec2{ 2.0f, 0.0f } ) < 1.0e-6f,
        "obstructed or absent component faces must not manufacture momentum" );

    // Solid에 인접한 face는 운동학 경계이므로 유효 fluid momentum에도 impulse에도 포함하지 않음.
    fluidField2 occupied{ 24, 24, 0.5f, { -6.0f, -6.0f } };
    const fluidCircleObstacle2 solid{ {}, 1.1f, {}, 0.0f };
    std::array<fluidObstacleForce2, 1> forces{};
    occupied.step( 1.0f / 60.0f, std::span<const fluidCircleObstacle2>( &solid, 1 ), forces );
    const vec2 solidBefore = occupied.getFluidMomentum();
    const vec2 deposited = occupied.applyMomentumImpulse( { 1.6f, 0.0f }, { 0.9f, -0.4f }, 1.0f );
    check( occupied.getSolidCellCount() > 0 &&
        Length( deposited - vec2{ 0.9f, -0.4f } ) < 5.0e-5f &&
        Length( ( occupied.getFluidMomentum() - solidBefore ) - deposited ) < 5.0e-5f,
        "solid masks must be respected while delivering equal-and-opposite discrete momentum" );

    occupied.clear();
    check( LengthSquared( occupied.getFluidMomentum() ) == 0.0f,
        "clear must erase previously deposited momentum" );
}

void checkPressureMomentumLedger()
{
    constexpr float dt = 1.0f / 60.0f;
    const auto checkBudget = []( const fluidField2& field, const char* message )
    {
        const fluidPressureMomentum2& budget = field.getPressureMomentum();
        const fluidStepMomentum2& stages = field.getStepMomentum();
        const float magnitude = Length( budget.fluid ) + Length( budget.solid ) + Length( budget.wall );
        check( IsFinite( budget.fluid ) && IsFinite( budget.solid ) && IsFinite( budget.wall ) &&
            Length( budget.residual() ) < 5.0e-4f * ( 1.0f + magnitude ), message );
        check( Length( budget.fluid - stages.pressureDelta() ) < 1.0e-5f * ( 1.0f + magnitude ),
            "pressure stage momentum must match the independently stored pressure impulse" );
        check( Length( stages.afterPressure - field.getFluidMomentum() ) < 1.0e-5f * ( 1.0f + magnitude ),
            "end-of-step face momentum must match the final diagnostic snapshot" );
        check( Length( stages.occupancyDelta() + stages.advectionDelta() + stages.pressureDelta() -
            ( stages.afterPressure - stages.before ) ) < 1.0e-5f * ( 1.0f + magnitude ),
            "stage deltas must telescope to the measured total momentum change" );
    };

    // 외벽에 가까운 velocity source는 pressure를 통해 벽에 운동량을 전달한다.
    fluidField2 walls{ 24, 16, 0.5f, { -6.0f, -4.0f } };
    walls.addVelocity( { -4.9f, -0.2f }, { 2.8f, 0.4f }, 1.7f );
    walls.step( dt, 160 );
    checkBudget( walls, "wall-only projection pressure impulses must close momentum budget" );
    check( LengthSquared( walls.getPressureMomentum().solid ) == 0.0f &&
        Length( walls.getPressureMomentum().wall ) > 1.0e-5f,
        "pressure must report wall impulse but no solid impulse in an empty domain" );

    // 정지 obstacle에 대한 flow pressure traction은 fluid / solid / wall에 분배된다.
    fluidField2 obstacleField{ 32, 24, 0.25f, { -4.0f, -3.0f } };
    const fluidCircleObstacle2 obstacle{ { 0.0f, 0.0f }, 0.9f, {}, 0.0f };
    std::array<fluidObstacleForce2, 1> pressureForces{};
    obstacleField.addVelocity( { -1.2f, 0.1f }, { 2.0f, -0.2f }, 1.4f );
    obstacleField.step( dt, std::span<const fluidCircleObstacle2>( &obstacle, 1 ), pressureForces, 160 );
    checkBudget( obstacleField, "stationary circle pressure impulses must close momentum budget" );
    check( Length( obstacleField.getPressureMomentum().solid ) > 1.0e-5f,
        "stationary solid in nonuniform flow must receive pressure impulse" );
    check( Length( obstacleField.getPressureMomentum().solid - dt * pressureForces[0].force ) < 1.0e-4f,
        "pressure budget solid impulse must match the force returned to World" );
    check( obstacleField.getStepMomentum().occupancyDelta().x != 0.0f,
        "new obstacle occupancy must be reported separately from advection and pressure" );

    // Moving circle imposes no-through boundary even in initially resting fluid.
    fluidField2 movingField{ 32, 24, 0.25f, { -4.0f, -3.0f } };
    const fluidCircleObstacle2 moving{ { 0.0f, 0.0f }, 0.9f, { 1.0f, 0.2f }, 0.0f };
    movingField.step( dt, std::span<const fluidCircleObstacle2>( &moving, 1 ), pressureForces, 160 );
    checkBudget( movingField, "moving circle pressure reaction must close its discrete pressure budget" );
    check( Length( movingField.getPressureMomentum().solid ) > 1.0e-5f &&
        Length( movingField.getPressureMomentum().fluid ) > 1.0e-5f,
        "moving solid pressure should affect both rigid and fluid momentum" );

    // Reset invalidates the last completed step diagnostics.
    movingField.clear();
    check( LengthSquared( movingField.getPressureMomentum().residual() ) == 0.0f &&
        LengthSquared( movingField.getStepMomentum().afterPressure ) == 0.0f,
        "clear must reset pressure and stage momentum ledgers" );
}

void checkMovingSolidRemap()
{
    constexpr float dt = 1.0f / 60.0f;
    fluidField2 field{ 32, 24, 0.25f, { -4.0f, -3.0f } };
    std::array<fluidObstacleForce2, 1> feedback{};

    fluidCircleObstacle2 circle{ { 0.0f, 0.0f }, 0.9f, { 5.0f, 0.0f }, 0.0f };
    field.step( dt, std::span<const fluidCircleObstacle2>( &circle, 1 ), feedback );
    const int occupied = field.getSolidCellCount();
    const fluidSolidTransition2 first = field.getSolidTransition();
    check( occupied > 0 && first.coveredCells == occupied && first.exposedCells == 0 &&
        first.openedUFaces == 0 && first.openedVFaces == 0 && first.fluidVolumeDelta < 0.0f,
        "first solid rasterization must only cover cells and close fluid faces" );

    // 크기와 각도가 같은 원을 격자 크기만큼 이동하면 유효 체적이 변하지 않는다.
    circle.center.x += field.getCellSize();
    circle.linearVelocity = {};
    field.step( dt, std::span<const fluidCircleObstacle2>( &circle, 1 ), feedback, 160 );
    const fluidSolidTransition2 moved = field.getSolidTransition();
    check( field.getSolidCellCount() == occupied &&
        moved.exposedCells > 0 && moved.coveredCells == moved.exposedCells &&
        moved.fluidVolumeDelta == 0.0f &&
        moved.openedUFaces + moved.openedVFaces > 0 &&
        moved.extrapolatedFaces + moved.fallbackFaces == moved.openedUFaces + moved.openedVFaces,
        "translating solid must account for each re-exposed face and conserve discrete cell count" );
    check( std::isfinite( field.getDivergenceRms() ) && IsFinite( field.getFluidMomentum() ) &&
        IsFinite( field.getPressureMomentum().residual() ),
        "moving solid remap must preserve finite projected fluid state" );

    // 동일 장애물을 다음 step에 그대로 제공하면 전이 통계가 전부 0.
    field.step( dt, std::span<const fluidCircleObstacle2>( &circle, 1 ), feedback );
    const fluidSolidTransition2 still = field.getSolidTransition();
    check( still.exposedCells == 0 && still.coveredCells == 0 &&
        still.openedUFaces == 0 && still.openedVFaces == 0 && still.fluidVolumeDelta == 0.0f,
        "unchanged occupancy must not remap or report phantom fluid volume changes" );

    // Solid 삭제 시 경계 속도가 남아 있던 face를 유체 속도로 복원한다.
    field.step( dt );
    const fluidSolidTransition2 removed = field.getSolidTransition();
    check( removed.exposedCells == occupied && removed.coveredCells == 0 &&
        close( removed.fluidVolumeDelta, occupied * 0.25f * 0.25f ) &&
        removed.extrapolatedFaces + removed.fallbackFaces == removed.openedUFaces + removed.openedVFaces,
        "removing solid must reopen all fluid cells and reconstruct exposed MAC faces" );
    field.step( dt );
    check( field.getSolidTransition().exposedCells == 0 && field.getSolidTransition().coveredCells == 0 &&
        std::isfinite( field.getDivergenceRms() ),
        "fluid-only follow-up must not keep stale solid transition state" );

    // 회전한 얇은 다각형에서는 같은 부피의 cell들이 가리고 다시 열릴 수 있다.
    field.clear();
    fluidShapeObstacle2 rectangle{ MakeBox( { 0.85f, 0.2f } ), {}, {}, {}, 0.0f };
    field.step( dt, std::span<const fluidShapeObstacle2>( &rectangle, 1 ), feedback );
    rectangle.transform.rotation = rot2::FromRadians( 0.5f * std::numbers::pi_v<float> );
    field.step( dt, std::span<const fluidShapeObstacle2>( &rectangle, 1 ), feedback );
    const fluidSolidTransition2 rotated = field.getSolidTransition();
    check( rotated.exposedCells > 0 && rotated.coveredCells > 0 &&
        rotated.openedUFaces + rotated.openedVFaces > 0 &&
        rotated.extrapolatedFaces + rotated.fallbackFaces == rotated.openedUFaces + rotated.openedVFaces,
        "rotating polygon must reconstruct newly opened fluid faces" );

    field.clear();
    check( field.getSolidTransition().exposedCells == 0 &&
        field.getSolidTransition().coveredCells == 0 && field.getSolidCellCount() == 0,
        "clear must reset solid transition diagnostics" );
}

void checkBoundedObstacleRasterAndReusablePressure()
{
    constexpr float dt = 1.0f / 60.0f;
    const auto verify = [&]( bool cutCell )
    {
        fluidField2 reference{ 40, 30, 0.5f, { -10.0f, -7.5f } };
        fluidField2 withFarShapes{ 40, 30, 0.5f, { -10.0f, -7.5f } };
        reference.setCutCellProjectionEnabled( cutCell );
        withFarShapes.setCutCellProjectionEnabled( cutCell );
        fluidShapeObstacle2 box{ MakeBox( { 0.85f, 0.35f } ),
            { { -0.25f, 0.3f }, rot2::FromRadians( 0.3f ) }, { -0.25f, 0.3f }, {}, 0.0f };
        fluidShapeObstacle2 capsule{ capsule2{ { -0.65f, 0.0f }, { 0.65f, 0.0f }, 0.22f },
            { { 3.5f, -1.0f }, rot2::FromRadians( -0.45f ) }, { 3.5f, -1.0f }, {}, 0.0f };
        const fluidShapeObstacle2 farAway{ circle2{ {}, 0.8f },
            { { 10000.0f, 10000.0f }, {} }, { 10000.0f, 10000.0f }, {}, 0.0f };
        const std::array nearby{ box, capsule };
        const std::array expanded{ box, capsule, farAway };
        std::array<fluidObstacleForce2, 2> referenceForce{};
        std::array<fluidObstacleForce2, 3> expandedForce{};
        reference.addVelocity( { -2.0f, 0.2f }, { 2.0f, -0.1f }, 1.5f );
        withFarShapes.addVelocity( { -2.0f, 0.2f }, { 2.0f, -0.1f }, 1.5f );
        reference.step( dt, nearby, referenceForce, 120 );
        withFarShapes.step( dt, expanded, expandedForce, 120 );
        check( reference.getSolidCellCount() == withFarShapes.getSolidCellCount() &&
            close( reference.getCutCellTransition().fluidVolume,
                withFarShapes.getCutCellTransition().fluidVolume ) &&
            close( Length( referenceForce[0].force - expandedForce[0].force ), 0.0f ) &&
            close( Length( referenceForce[1].force - expandedForce[1].force ), 0.0f ) &&
            LengthSquared( expandedForce[2].force ) == 0.0f,
            "distant obstacles must not change local fluid occupancy or pressure reaction" );
        for( int y = 0; y < reference.getHeight(); ++y )
            for( int x = 0; x < reference.getWidth(); ++x )
                check( reference.isSolidCell( x, y ) == withFarShapes.isSolidCell( x, y ) &&
                    close( reference.getCellFluidFraction( x, y ), withFarShapes.getCellFluidFraction( x, y ) ),
                    "bounded obstacle sampling must preserve every cell mask and fractional coverage" );
    };
    verify( false );
    verify( true );

    // Reuse of scratch memory must not retain an old pressure solution after clear().
    fluidField2 reused{ 24, 16, 0.5f, { -6.0f, -4.0f } };
    fluidShapeObstacle2 dynamic{ circle2{ {}, 0.75f },
        { { 0.0f, 0.0f }, {} }, {}, { 1.25f, 0.0f }, 0.0f, 2.0f, 16.0f };
    std::array<fluidObstacleForce2, 1> reuseForce{};
    std::array<fluidObstacleForce2, 1> freshForce{};
    for( int test = 0; test < 5; ++test )
    {
        reused.clear();
        fluidField2 fresh{ 24, 16, 0.5f, { -6.0f, -4.0f } };
        dynamic.linearVelocity = { test % 2 == 0 ? 1.25f : -0.75f, 0.0f };
        reused.step( dt, std::span<const fluidShapeObstacle2>( &dynamic, 1 ), reuseForce, 160 );
        fresh.step( dt, std::span<const fluidShapeObstacle2>( &dynamic, 1 ), freshForce, 160 );
        check( reused.getPressureMomentum().usedCoupledProjection &&
            fresh.getPressureMomentum().usedCoupledProjection &&
            !reused.getPressureMomentum().coupledSolveFailed &&
            close( Length( reuseForce[0].force - freshForce[0].force ), 0.0f, 1.0e-5f ) &&
            close( reused.getDivergenceRms(), fresh.getDivergenceRms(), 1.0e-5f ),
            "reused PCG buffers must reproduce cold-field coupled projection");
    }
}

void checkCoupledPressureConvergenceDiagnostics()
{
    constexpr float dt = 1.0f / 60.0f;
    fluidShapeObstacle2 moving{ circle2{ {}, 0.75f },
        { { 0.0f, 0.0f }, {} }, {}, { 1.5f, 0.0f }, 0.0f, 2.0f, 16.0f };
    std::array<fluidObstacleForce2, 1> feedback{};

    // PCG 1회는 이 영역의 다수 압력 자유도를 해결하기 부족하다.
    fluidField2 limited{ 32, 24, 0.25f, { -4.0f, -3.0f } };
    limited.addVelocity( { -1.2f, 0.25f }, { 3.0f, -0.6f }, 1.0f );
    limited.step( dt, std::span<const fluidShapeObstacle2>( &moving, 1 ), feedback, 1 );
    const fluidPressureMomentum2 limitedStats = limited.getPressureMomentum();
    check( limitedStats.coupledStatus == fluidCoupledPressureStatus2::iterationLimit &&
        limitedStats.coupledSolveFailed && !limitedStats.usedCoupledProjection &&
        limitedStats.coupledIterations == 1 && limitedStats.coupledIterationLimit == 1 &&
        limitedStats.coupledInitialResidualRms > 0.0f &&
        limitedStats.coupledFinalResidualRms > 0.0f &&
        limitedStats.coupledRelativeResidual > 1.0e-5f,
        "one CG iteration must report unmet true residual, not a successful pressure solve" );
    check( LengthSquared( feedback[0].force ) == 0.0f &&
        LengthSquared( limitedStats.solid ) == 0.0f &&
        LengthSquared( limitedStats.wall ) == 0.0f,
        "failed coupled PCG must not inject an uncoupled Jacobi reaction into a dynamic body" );

    fluidField2 converged{ 32, 24, 0.25f, { -4.0f, -3.0f } };
    converged.addVelocity( { -1.2f, 0.25f }, { 3.0f, -0.6f }, 1.0f );
    converged.step( dt, std::span<const fluidShapeObstacle2>( &moving, 1 ), feedback, 200 );
    const fluidPressureMomentum2 good = converged.getPressureMomentum();
    check( good.coupledStatus == fluidCoupledPressureStatus2::converged &&
        good.usedCoupledProjection && !good.coupledSolveFailed &&
        good.coupledIterations > 1 && good.coupledIterations <= good.coupledIterationLimit &&
        good.coupledInitialResidualRms > 0.0f &&
        good.coupledRelativeResidual <= 1.0001e-5f,
        "adequate PCG iterations must yield small measured true residual and a valid pressure solve" );
    check( std::isfinite( feedback[0].force.x ) && std::isfinite( feedback[0].force.y ) &&
        Length( feedback[0].force ) > 0.001f,
        "converged coupled pressure should return finite nonzero reaction" );

    fluidField2 noContact{ 24, 16, 0.5f, { -6.0f, -4.0f } };
    fluidShapeObstacle2 outside = moving;
    outside.transform.position = { 100.0f, 100.0f };
    outside.center = outside.transform.position;
    noContact.addVelocity( { -2.0f, 0.0f }, { 3.0f, 0.0f }, 1.0f );
    noContact.step( dt, std::span<const fluidShapeObstacle2>( &outside, 1 ), feedback );
    check( noContact.getPressureMomentum().coupledStatus == fluidCoupledPressureStatus2::noBoundary &&
        !noContact.getPressureMomentum().coupledSolveFailed &&
        !noContact.getPressureMomentum().usedCoupledProjection,
        "dynamic body outside the Fluid Field should use legacy pressure without reporting a PCG failure" );

    converged.clear();
    check( converged.getPressureMomentum().coupledStatus == fluidCoupledPressureStatus2::notRequested &&
        converged.getPressureMomentum().coupledIterations == 0,
        "clearing Fluid Field must clear previous PCG diagnostics" );
}

void checkCoupledPressureBoundaryFaceSynchronization()
{
    constexpr float dt = 1.0f / 60.0f;
    const fluidShapeObstacle2 moving{
        MakeBox( { 0.8f, 0.3f } ),
        { { -0.5f, 0.25f }, rot2::FromRadians( 0.37f ) },
        { -0.5f, 0.25f }, { 1.25f, -0.45f }, 1.7f, 0.6f, 2.3f
    };
    constexpr int width = 32, height = 24;
    constexpr float spacing = 0.25f;
    constexpr vec2 origin{ -4.0f, -3.0f };
    std::array<fluidObstacleForce2, 1> feedback{};

    fluidField2 field{ width, height, spacing, origin };
    field.addVelocity( { -1.75f, 0.65f }, { 1.8f, -0.7f }, 1.0f );
    field.step( dt, std::span<const fluidShapeObstacle2>( &moving, 1 ), feedback, 200 );
    const fluidPressureMomentum2& pressure = field.getPressureMomentum();
    check( pressure.usedCoupledProjection &&
        pressure.coupledStatus == fluidCoupledPressureStatus2::converged &&
        !pressure.coupledSolveFailed && pressure.synchronizedBoundaryFaces > 0 &&
        pressure.boundaryNormalChangeRms > 1.0e-5f,
        "a moving and rotating coupled box must update its solid MAC faces from pressure impulses" );

    const vec2 predictedLinear = moving.linearVelocity +
        ( dt * moving.inverseMass ) * feedback[0].force;
    const float predictedAngular = moving.angularVelocity +
        dt * moving.inverseInertia * feedback[0].torque;
    int checkedFaces = 0;
    const auto checkNormal = [&]( vec2 point, bool horizontal )
    {
        const vec2 predicted = predictedLinear +
            Cross( predictedAngular, point - moving.center );
        const vec2 stored = field.sampleVelocity( point );
        const float mismatch = horizontal ? stored.x - predicted.x : stored.y - predicted.y;
        check( std::abs( mismatch ) < 1.0e-4f,
            "every coupled fluid-solid MAC face must use pressure-predicted rigid velocity" );
        ++checkedFaces;
    };

    for( int y = 0; y < height; ++y )
        for( int x = 1; x < width; ++x )
            if( field.isSolidCell( x - 1, y ) != field.isSolidCell( x, y ) )
                checkNormal( origin + spacing *
                    vec2{ static_cast<float>( x ), static_cast<float>( y ) + 0.5f }, true );
    for( int y = 1; y < height; ++y )
        for( int x = 0; x < width; ++x )
            if( field.isSolidCell( x, y - 1 ) != field.isSolidCell( x, y ) )
                checkNormal( origin + spacing *
                    vec2{ static_cast<float>( x ) + 0.5f, static_cast<float>( y ) }, false );
    check( checkedFaces == pressure.synchronizedBoundaryFaces && checkedFaces > 10,
        "coupled boundary synchronization must report the actual tested fluid-solid MAC faces" );

    // Non-converged pressure must not apply a speculative correction.
    fluidField2 limited{ width, height, spacing, origin };
    limited.addVelocity( { -1.75f, 0.65f }, { 1.8f, -0.7f }, 1.0f );
    limited.step( dt, std::span<const fluidShapeObstacle2>( &moving, 1 ), feedback, 1 );
    check( limited.getPressureMomentum().coupledSolveFailed &&
        limited.getPressureMomentum().synchronizedBoundaryFaces == 0 &&
        limited.getPressureMomentum().boundaryNormalChangeRms == 0.0f &&
        LengthSquared( feedback[0].force ) == 0.0f,
        "failed coupled PCG must not synchronize boundaries with an invalid pressure impulse" );
    for( int y = 0; y < height; ++y )
        for( int x = 1; x < width; ++x )
            if( limited.isSolidCell( x - 1, y ) != limited.isSolidCell( x, y ) )
            {
                const vec2 point = origin + spacing *
                    vec2{ static_cast<float>( x ), static_cast<float>( y ) + 0.5f };
                const float expected = ( moving.linearVelocity +
                    Cross( moving.angularVelocity, point - moving.center ) ).x;
                check( std::abs( limited.sampleVelocity( point ).x - expected ) < 1.0e-4f,
                    "failed PCG must preserve the original kinematic no-through boundary velocity" );
            }
}

void checkCoupledPressureEnergyAudit()
{
    constexpr float dt = 1.0f / 60.0f;
    constexpr std::array<float, 3> MASSES{ 0.25f, 2.0f, 12.0f };
    constexpr std::array<float, 3> INERTIAS{ 0.08f, 1.0f, 8.0f };
    constexpr std::array<float, 2> CELL_SIZES{ 0.5f, 0.25f };

    // 같은 물리 영역을 두 해상도로 계산한다. 별도의 source, Drag, 외력 없음.
    // 여기서는 형상을 고정한 채 전달 속도만 적분해 pressure 연산을 단독 계측한다.
    // 이는 움직이는 경계의 공간-시간 체적 보존을 검증하는 테스트가 아니다.
    for( float h : CELL_SIZES )
        for( std::size_t massIndex = 0; massIndex < MASSES.size(); ++massIndex )
        {
            const int width = static_cast<int>( 12.0f / h );
            const int height = static_cast<int>( 8.0f / h );
            fluidField2 field{ width, height, h, { -6.0f, -4.0f } };
            fluidShapeObstacle2 obstacle{
                MakeBox( { 0.65f, 0.35f } ),
                { { 0.0f, 0.0f }, rot2::FromRadians( 0.35f ) },
                {}, { 1.1f, -0.45f }, 0.85f,
                1.0f / MASSES[massIndex], 1.0f / INERTIAS[massIndex]
            };
            std::array<fluidObstacleForce2, 1> feedback{};
            const auto kinetic = [&]
            {
                const double linear = static_cast<double>( Dot( obstacle.linearVelocity, obstacle.linearVelocity ) );
                const double spin = static_cast<double>( obstacle.angularVelocity ) * obstacle.angularVelocity;
                return 0.5 * ( MASSES[massIndex] * linear + INERTIAS[massIndex] * spin );
            };
            const double initialEnergy = kinetic();
            double peakEnergy = initialEnergy;
            double positivePressureWork = 0.0;
            float maxMomentumError = 0.0f;
            for( int step = 0; step < 36; ++step )
            {
                const double previousRigidEnergy = kinetic();
                field.step( dt, std::span<const fluidShapeObstacle2>( &obstacle, 1 ), feedback, 200 );
                const fluidPressureMomentum2& pressure = field.getPressureMomentum();
                check( pressure.usedCoupledProjection &&
                    pressure.coupledStatus == fluidCoupledPressureStatus2::converged &&
                    !pressure.coupledSolveFailed,
                    "isolated pressure energy trial requires a converged body-mass-aware solver" );
                const auto& energy = pressure.energy;
                check( std::isfinite( energy.fluidBefore ) && std::isfinite( energy.fluidAfter ) &&
                    std::isfinite( energy.rigidBefore ) && std::isfinite( energy.rigidAfter ) &&
                    std::abs( energy.fluidAfter - field.getFluidKineticEnergy() ) < 1.0e-8 * ( 1.0 + energy.fluidAfter ) &&
                    std::abs( energy.rigidBefore - previousRigidEnergy ) < 1.0e-5 * ( 1.0 + previousRigidEnergy ),
                    "pressure energy ledger must agree with the independent face sum and initial body mass/inertia" );
                obstacle.linearVelocity += dt * obstacle.inverseMass * feedback[0].force;
                obstacle.angularVelocity += dt * obstacle.inverseInertia * feedback[0].torque;
                const double currentRigidEnergy = kinetic();
                check( std::abs( currentRigidEnergy - energy.rigidAfter ) < 1.0e-5 * ( 1.0 + currentRigidEnergy ),
                    "pressure-predicted rigid kinetic energy must equal integrated force/torque with given mass and inertia" );
                const float impulseMagnitude = Length( pressure.fluid ) + Length( pressure.solid ) + Length( pressure.wall );
                const float normalizedError = Length( pressure.residual() ) / ( 1.0f + impulseMagnitude );
                maxMomentumError = std::max( maxMomentumError, normalizedError );
                check( normalizedError < 5.0e-4f,
                    "fluid, solid and wall pressure impulses must close at both resolutions and across masses" );
                peakEnergy = std::max( peakEnergy, energy.fluidAfter + currentRigidEnergy );
                positivePressureWork += std::max( 0.0, energy.totalDelta() );
                check( std::isfinite( peakEnergy ) && peakEnergy < 4.0 * ( 1.0 + initialEnergy ),
                    "source-free isolated pressure exchange must not exhibit explosive kinetic-energy growth" );
            }
            std::printf( "FSI energy audit: h=%.2f mass=%.2f inertia=%.2f initial=%.6f peak=%.6f final=%.6f positivePressureDelta=%.6f maxMomentumError=%.3g\n",
                h, MASSES[massIndex], INERTIAS[massIndex], initialEnergy, peakEnergy,
                field.getFluidKineticEnergy() + kinetic(), positivePressureWork, maxMomentumError );
        }

    fluidField2 empty{ 8, 8, 0.5f, { -2.0f, -2.0f } };
    check( empty.getFluidKineticEnergy() == 0.0, "fresh Fluid Field has no MAC kinetic energy" );
    empty.addVelocity( {}, { 2.0f, 1.0f }, 1.0f );
    check( empty.getFluidKineticEnergy() > 0.0, "velocity injection must increase MAC kinetic energy" );
    empty.clear();
    check( empty.getFluidKineticEnergy() == 0.0 &&
        empty.getPressureMomentum().energy.totalDelta() == 0.0,
        "clearing Fluid Field must clear kinetic energy and pressure-stage energy diagnostics" );
}

void checkFractionalCutCells()
{
    constexpr float dt = 1.0f / 60.0f;
    fluidField2 field{ 8, 8, 1.0f, { -4.0f, -4.0f } };
    std::array<fluidObstacleForce2, 1> feedback{};

    // 정확히 1.5m x 1.5m인 축 정렬 Box: 일부 cell/face는 절반만 막힌다.
    fluidShapeObstacle2 box{
        MakeBox( { 0.75f, 0.75f } ),
        { { 0.25f, 0.25f }, {} }, { 0.25f, 0.25f }, {}, 0.0f
    };
    field.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), feedback );
    const fluidCutCellTransition2 original = field.getCutCellTransition();
    check( close( field.getCellFluidFraction( 4, 4 ), 0.0f ) &&
        close( field.getCellFluidFraction( 3, 4 ), 0.5f ) &&
        close( field.getCellFluidFraction( 2, 4 ), 1.0f ),
        "fractional cell geometry must distinguish fully covered, half-covered and open cells" );
    check( close( field.getUFaceOpenFraction( 4, 3 ), 0.5f ) &&
        close( field.getVFaceOpenFraction( 3, 4 ), 0.5f ) &&
        close( field.getUFaceOpenFraction( 4, 4 ), 0.0f ) &&
        close( field.getVFaceOpenFraction( 4, 4 ), 0.0f ),
        "MAC face fractions must count sub-face openings rather than only cell-center occupancy" );
    check( field.getUFaceOpenFraction( 0, 4 ) == 0.0f &&
        field.getUFaceOpenFraction( 8, 4 ) == 0.0f &&
        field.getVFaceOpenFraction( 4, 0 ) == 0.0f &&
        field.getVFaceOpenFraction( 4, 8 ) == 0.0f &&
        field.getCellFluidFraction( -1, 4 ) == 0.0f,
        "closed domain wall and out-of-range fraction queries must be zero" );
    check( close( original.previousFluidVolume, 64.0f ) &&
        close( original.fluidVolume, 61.75f ) &&
        close( original.coveredVolume, 2.25f ) &&
        original.releasedVolume == 0.0f &&
        original.partialCells > 0 && original.partialUFaces > 0 && original.partialVFaces > 0,
        "sampled cut-cell volumes must match an axis-aligned box exactly at quarter-cell offsets" );

    // 1/4 cell 이동은 covered/released 영역을 만들지만 전체 점유 체적은 그대로다.
    box.transform.position.x += 0.25f;
    box.center.x += 0.25f;
    field.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), feedback );
    const fluidCutCellTransition2 translated = field.getCutCellTransition();
    check( translated.coveredVolume > 0.0f && translated.releasedVolume > 0.0f &&
        close( translated.fluidVolume, original.fluidVolume ) &&
        std::abs( translated.volumeDelta() ) < 1.0e-5f &&
        close( translated.coveredVolume, translated.releasedVolume ),
        "translating box must report covered and released partial volumes independently" );
    field.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), feedback );
    check( field.getCutCellTransition().coveredVolume == 0.0f &&
        field.getCutCellTransition().releasedVolume == 0.0f &&
        close( field.getCutCellTransition().volumeDelta(), 0.0f ),
        "stationary shape must have zero fractional volume transitions" );

    // 이 원은 어떤 cell center도 덮지 않지만 sub-cell은 차지한다.
    field.clear();
    const fluidCircleObstacle2 smallCircle{ {}, 0.65f, {}, 0.0f };
    field.step( dt, std::span<const fluidCircleObstacle2>( &smallCircle, 1 ), feedback );
    const float circleArea = std::numbers::pi_v<float> * 0.65f * 0.65f;
    const float sampledArea = 64.0f - field.getCutCellTransition().fluidVolume;
    check( field.getSolidCellCount() == 0 && field.getCutCellTransition().partialCells >= 4 &&
        std::abs( sampledArea - circleArea ) < 0.2f,
        "sub-cell coverage must detect geometry missed entirely by the center-only solid mask" );

    // 격자 안쪽에서 회전·이동하는 thin box의 전체 점유 면적은 거의 일정.
    field.clear();
    fluidShapeObstacle2 thinBox{
        MakeBox( { 1.0f, 0.2f } ), { { 0.5f, 0.0f }, {} }, { 0.5f, 0.0f }, {}, 0.0f
    };
    field.step( dt, std::span<const fluidShapeObstacle2>( &thinBox, 1 ), feedback );
    const float initialArea = 64.0f - field.getCutCellTransition().fluidVolume;
    thinBox.transform.rotation = rot2::FromRadians( 0.5f * std::numbers::pi_v<float> );
    field.step( dt, std::span<const fluidShapeObstacle2>( &thinBox, 1 ), feedback );
    const fluidCutCellTransition2 rotated = field.getCutCellTransition();
    check( rotated.coveredVolume > 0.0f && rotated.releasedVolume > 0.0f &&
        std::abs( 64.0f - rotated.fluidVolume - initialArea ) < 0.25f &&
        rotated.partialCells > 0,
        "rotated polygon must update partially blocked cells and volume transitions" );

    field.step( dt );
    check( close( field.getCutCellTransition().fluidVolume, 64.0f ) &&
        field.getCutCellTransition().releasedVolume > 0.0f,
        "removing solid must release its fractional occupancy" );
    field.clear();
    check( field.getCutCellTransition().fluidVolume == 0.0f &&
        close( field.getCellFluidFraction( 4, 4 ), 1.0f ) &&
        close( field.getUFaceOpenFraction( 4, 4 ), 1.0f ),
        "clear must reset the diagnostics and restore fully open interior geometry" );
}

void checkWeightedCutCellPressure()
{
    constexpr float dt = 1.0f / 60.0f;

    // 빈 domain에서 a=1이므로 기존 MAC과 동일한 압력 투영을 수행한다.
    fluidField2 empty{ 24, 16, 0.5f, { -6.0f, -4.0f } };
    empty.setCutCellProjectionEnabled( true );
    empty.addVelocity( { -2.0f, 0.4f }, { 3.0f, 0.0f }, 1.5f );
    empty.step( dt, 160 );
    const fluidCutCellPressure2 noObstacle = empty.getCutCellPressure();
    check( noObstacle.activeCells == 24 * 16 &&
        noObstacle.divergenceBefore > 0.01f &&
        noObstacle.divergenceAfter < noObstacle.divergenceBefore * 0.3f &&
        std::abs( noObstacle.sourceImbalance ) < 1.0e-4f,
        "aperture weighted solver must project empty-domain velocity with zero net flux" );

    // 작은 원은 cell-center mask에 없더라도 fractional face를 막는다.
    fluidField2 cut{ 24, 24, 0.5f, { -6.0f, -6.0f } };
    cut.setCutCellProjectionEnabled( true );
    const fluidCircleObstacle2 circle{ {}, 0.65f, {}, 0.0f };
    std::array<fluidObstacleForce2, 1> pressure{};
    cut.addVelocity( { -1.0f, 0.0f }, { 2.0f, 0.0f }, 1.4f );
    cut.step( dt, std::span<const fluidCircleObstacle2>( &circle, 1 ), pressure, 200 );
    const fluidCutCellPressure2 cutStats = cut.getCutCellPressure();
    check( cutStats.activeCells > 0 && cut.getCutCellTransition().partialCells > 0 &&
        cutStats.divergenceBefore > 0.01f &&
        cutStats.divergenceAfter < cutStats.divergenceBefore * 0.55f &&
        std::isfinite( cutStats.pressureResidual ) && IsFinite( pressure[0].force ),
        "partial circle apertures must be used by the pressure projection and return finite feedback" );
    check( LengthSquared( cut.getPressureMomentum().residual() ) == 0.0f,
        "binary pressure momentum ledger must not be reported for weighted cut-cell mode" );

    // 정지 유체와 정지 장애물은 압력이나 운동량을 생성하지 않는다.
    fluidField2 resting{ 24, 24, 0.5f, { -6.0f, -6.0f } };
    resting.setCutCellProjectionEnabled( true );
    resting.step( dt, std::span<const fluidCircleObstacle2>( &circle, 1 ), pressure );
    check( resting.getCutCellPressure().divergenceAfter == 0.0f &&
        resting.getCutCellPressure().pressureResidual == 0.0f &&
        LengthSquared( pressure[0].force ) == 0.0f,
        "stationary cut-cell boundary must not generate spurious pressure" );

    // 이동하는 경계의 blocked fraction은 별도 solid velocity flux로 계산한다.
    // 실제 이동한 snapshot만 ΔV/dt 소스를 생성한다.
    const fluidCircleObstacle2 moving{ { 0.25f, 0.0f }, 0.65f, { 1.0f, 0.0f }, 0.0f };
    resting.step( dt, std::span<const fluidCircleObstacle2>( &moving, 1 ), pressure, 200 );
    check( resting.getCutCellPressure().divergenceBefore > 0.01f &&
        std::isfinite( resting.getCutCellPressure().divergenceAfter ) &&
        std::isfinite( resting.getCutCellPressure().sourceImbalance ) &&
        IsFinite( pressure[0].force ),
        "moving cut-cell boundary must enter fluid divergence through solid normal velocity" );

    resting.setCutCellProjectionEnabled( false );
    resting.step( dt, std::span<const fluidCircleObstacle2>( &moving, 1 ), pressure );
    check( resting.getCutCellPressure().activeCells == 0 &&
        IsFinite( resting.getPressureMomentum().residual() ),
        "disabling experimental mode must return to original center-mask pressure solver" );
    resting.clear();
    check( resting.getCutCellPressure().activeCells == 0 &&
        !resting.isCutCellProjectionEnabled(),
        "clear should reset diagnostics but retain the selected pressure mode" );
}

void checkCutCellCoupledPressure()
{
    constexpr float dt = 1.0f / 60.0f;
    const fluidShapeObstacle2 moving{
        MakeBox( { 0.85f, 0.35f } ),
        { { 0.1f, -0.15f }, rot2::FromRadians( 0.25f ) },
        { 0.1f, -0.15f }, { 1.2f, -0.35f }, 0.8f, 0.5f, 2.0f
    };
    std::array<fluidObstacleForce2, 1> feedback{};
    const auto seed = [&]( fluidField2& field )
    {
        field.setCutCellProjectionEnabled( true );
        field.addVelocity( { -1.2f, 0.0f }, { 1.5f, -0.3f }, 1.25f );
    };

    fluidField2 coupled{ 24, 16, 0.5f, { -6.0f, -4.0f } };
    seed( coupled );
    check( !coupled.isCutCellCoupledProjectionEnabled(),
        "cut-cell mass-aware projection must be opt-in" );
    coupled.setCutCellCoupledProjectionEnabled( true );
    coupled.step( dt, std::span<const fluidShapeObstacle2>( &moving, 1 ), feedback, 200 );
    const fluidCutCellPressure2 stats = coupled.getCutCellPressure();
    check( stats.usedCoupledProjection && !stats.coupledSolveFailed &&
        !stats.usedIncompleteCholesky &&
        stats.coupledStatus == fluidCoupledPressureStatus2::converged &&
        stats.coupledRelativeResidual < 1.1e-5f && stats.activeCells > 0 &&
        std::isfinite( stats.pressureResidual ) &&
        LengthSquared( feedback[0].force ) > 1.0e-6f &&
        IsFinite( feedback[0].force ) && std::isfinite( feedback[0].torque ),
        "weighted mass-aware pressure must converge and return finite rigid feedback" );
    check( LengthSquared( coupled.getPressureMomentum().residual() ) == 0.0f &&
        !coupled.getPressureMomentum().usedCoupledProjection,
        "cut-cell coupled pressure must not populate the binary MAC momentum ledger" );

    fluidField2 limited{ 24, 16, 0.5f, { -6.0f, -4.0f } };
    seed( limited );
    limited.setCutCellCoupledProjectionEnabled( true );
    limited.step( dt, std::span<const fluidShapeObstacle2>( &moving, 1 ), feedback, 1 );
    const fluidCutCellPressure2 failure = limited.getCutCellPressure();
    check( !failure.usedCoupledProjection && failure.coupledSolveFailed &&
        failure.coupledStatus == fluidCoupledPressureStatus2::iterationLimit &&
        LengthSquared( feedback[0].force ) == 0.0f && feedback[0].torque == 0.0f,
        "failed weighted PCG must not inject uncoupled pressure into a movable body" );

    // 역질량을 주지 않은 기존 형상과 비결합 경로는 변경하지 않는다.
    fluidField2 legacy{ 24, 16, 0.5f, { -6.0f, -4.0f } };
    seed( legacy );
    legacy.setCutCellCoupledProjectionEnabled( true );
    fluidShapeObstacle2 fixed = moving;
    fixed.inverseMass = 0.0f;
    fixed.inverseInertia = 0.0f;
    legacy.step( dt, std::span<const fluidShapeObstacle2>( &fixed, 1 ), feedback, 200 );
    check( legacy.getCutCellPressure().coupledStatus == fluidCoupledPressureStatus2::notRequested &&
        !legacy.getCutCellPressure().usedCoupledProjection &&
        !legacy.getCutCellPressure().coupledSolveFailed &&
        IsFinite( feedback[0].force ),
        "massless cut-cell obstacles must retain the legacy weighted pressure path" );

    coupled.clear();
    check( coupled.isCutCellCoupledProjectionEnabled() &&
        coupled.getCutCellPressure().coupledStatus == fluidCoupledPressureStatus2::notRequested,
        "clear must retain the opted-in solver choice but reset coupled diagnostics" );
}

void checkCutCellPredictedBoundarySynchronization()
{
    constexpr float dt = 1.0f / 60.0f;
    constexpr float radius = 0.87f;
    constexpr float h = 0.5f;
    constexpr vec2 origin{ -6.0f, -4.0f };
    const fluidShapeObstacle2 moving{ circle2{ {}, radius },
        { { 0.18f, -0.12f }, {} }, { 0.18f, -0.12f },
        { 0.9f, -0.4f }, 1.7f, 0.7f, 2.0f };
    std::array<fluidObstacleForce2, 1> feedback{};
    fluidField2 field{ 24, 16, h, origin };
    field.setCutCellProjectionEnabled( true );
    field.setCutCellCoupledProjectionEnabled( true );
    field.addVelocity( { -1.2f, 0.05f }, { 1.2f, -0.1f }, 1.5f );
    field.step( dt, std::span<const fluidShapeObstacle2>( &moving, 1 ), feedback, 200 );
    const fluidCutCellPressure2& stats = field.getCutCellPressure();
    check( stats.usedCoupledProjection && !stats.coupledSolveFailed &&
        LengthSquared( feedback[0].force ) > 1.0e-8f &&
        stats.synchronizedBoundaryFaces > stats.synchronizedPartialFaces &&
        stats.synchronizedPartialFaces > 0 && stats.boundaryNormalChangeRms > 1.0e-6f,
        "coupled cut-cell pressure must synchronize fully blocked and partially blocked faces" );

    const vec2 linear = moving.linearVelocity + ( dt * moving.inverseMass ) * feedback[0].force;
    const float angular = moving.angularVelocity + dt * moving.inverseInertia * feedback[0].torque;
    int checkedFaces = 0;
    int partialFaces = 0;
    const auto checkFace = [&]( int x, int y, bool uFace )
    {
        const float open = uFace ? field.getUFaceOpenFraction( x, y ) :
            field.getVFaceOpenFraction( x, y );
        if( open >= 1.0f ) return;
        float sum = 0.0f;
        int blocked = 0;
        const vec2 facePoint = origin + h * ( uFace
            ? vec2{ static_cast<float>( x ), static_cast<float>( y ) + 0.5f }
            : vec2{ static_cast<float>( x ) + 0.5f, static_cast<float>( y ) } );
        for( int i = 0; i < 4; ++i )
        {
            const float shift = ( i + 0.5f ) * 0.25f - 0.5f;
            const vec2 point = facePoint + h * ( uFace ? vec2{ 0.0f, shift } : vec2{ shift, 0.0f } );
            if( LengthSquared( point - moving.center ) > radius * radius ) continue;
            const vec2 expected = linear + Cross( angular, point - moving.center );
            sum += uFace ? expected.x : expected.y;
            ++blocked;
        }
        if( blocked == 0 ) return;
        const float expected = sum / blocked;
        const float recorded = uFace ? field.getUFaceSolidVelocity( x, y ) :
            field.getVFaceSolidVelocity( x, y );
        check( std::abs( recorded - expected ) < 1.0e-4f,
            "cut-cell solid normal velocity must match the pressure-predicted rigid surface speed" );
        if( open == 0.0f )
        {
            const vec2 storedFace = field.sampleVelocity( facePoint );
            const float actual = uFace ? storedFace.x : storedFace.y;
            check( std::abs( actual - expected ) < 1.0e-4f,
                "fully blocked MAC faces must use the synchronized solid normal velocity" );
        }
        else ++partialFaces;
        ++checkedFaces;
    };
    for( int y = 0; y < field.getHeight(); ++y )
        for( int x = 1; x < field.getWidth(); ++x ) checkFace( x, y, true );
    for( int y = 1; y < field.getHeight(); ++y )
        for( int x = 0; x < field.getWidth(); ++x ) checkFace( x, y, false );
    check( checkedFaces == stats.synchronizedBoundaryFaces &&
        partialFaces == stats.synchronizedPartialFaces &&
        field.getUFaceSolidVelocity( -1, 0 ) == 0.0f &&
        field.getVFaceSolidVelocity( 0, field.getHeight() ) == 0.0f,
        "synchronization diagnostics must report exactly the sampled blocked faces" );

    field.clear();
    check( field.getCutCellPressure().synchronizedBoundaryFaces == 0 &&
        field.getCutCellPressure().boundaryNormalChangeRms == 0.0f &&
        field.getUFaceSolidVelocity( field.getWidth() / 2, 3 ) == 0.0f,
        "clearing fluid must reset cut-cell boundary synchronization state" );

    // A failed coupled solve must keep the original supplied kinematic boundary.
    fluidField2 failed{ 24, 16, h, origin };
    failed.setCutCellProjectionEnabled( true );
    failed.setCutCellCoupledProjectionEnabled( true );
    failed.addVelocity( { -1.2f, 0.05f }, { 1.2f, -0.1f }, 1.5f );
    failed.step( dt, std::span<const fluidShapeObstacle2>( &moving, 1 ), feedback, 1 );
    check( failed.getCutCellPressure().coupledSolveFailed &&
        failed.getCutCellPressure().synchronizedBoundaryFaces == 0 &&
        failed.getCutCellPressure().boundaryNormalChangeRms == 0.0f &&
        LengthSquared( feedback[0].force ) == 0.0f,
        "failed cut-cell PCG must not apply any predicted boundary velocity" );
}

void checkCoupledCutCellAdjointContinuity()
{
    constexpr std::array<float, 2> CELL_SIZES{ 0.5f, 0.25f };
    constexpr std::array<float, 3> MASSES{ 0.5f, 2.0f, 12.0f };
    constexpr std::array<float, 3> INERTIAS{ 0.12f, 0.8f, 4.0f };
    constexpr std::array<float, 2> TIME_STEPS{ 1.0f / 60.0f, 1.0f / 120.0f };

    // 동일한 12x8m 영역에서 질량·관성·격자·dt를 바꿔가며,
    // PCG Schur 잔차와 실제 pressure-predicted blocked flux의 연속성 잔차를 대조.
    for( float h : CELL_SIZES )
        for( std::size_t i = 0; i < MASSES.size(); ++i )
            for( float dt : TIME_STEPS )
            {
                fluidField2 field{ static_cast<int>( 12.0f / h ),
                    static_cast<int>( 8.0f / h ), h, { -6.0f, -4.0f } };
                field.setCutCellProjectionEnabled( true );
                field.setCutCellCoupledProjectionEnabled( true );
                fluidShapeObstacle2 obstacle{
                    MakeBox( { 0.73f, 0.31f } ),
                    { { 0.13f, -0.17f }, rot2::FromRadians( 0.31f ) },
                    { 0.13f, -0.17f }, { 1.0f, -0.5f }, 1.2f,
                    1.0f / MASSES[i], 1.0f / INERTIAS[i]
                };
                std::array<fluidObstacleForce2, 1> forces{};
                field.addVelocity( { -1.1f, 0.25f }, { 1.3f, -0.4f }, 1.4f );
                field.step( dt, std::span<const fluidShapeObstacle2>( &obstacle, 1 ), forces, 200 );

                const fluidCutCellPressure2& stats = field.getCutCellPressure();
                check( stats.usedCoupledProjection && !stats.coupledSolveFailed &&
                    stats.coupledStatus == fluidCoupledPressureStatus2::converged &&
                    std::isfinite( stats.predictedContinuityRms ) &&
                    std::isfinite( stats.predictedSolidVolumeL1 ) &&
                    stats.synchronizedPartialFaces > 0 && stats.predictedSolidVolumeL1 > 0.0f,
                    "weighted sub-face coupling must converge and predict nonzero boundary swept volume" );
                // Adjoint pair의 실제 고체 velocity flux와 PCG A*p-b가 같은
                // 보정된 연속방정식을 측정해야 한다. 반올림 오차는 허용한다.
                const float tolerance = 2.0e-3f + 0.1f * stats.pressureResidual;
                if( std::abs( stats.predictedContinuityRms - stats.pressureResidual ) >= tolerance ||
                    stats.predictedContinuityRms >= 0.05f * stats.divergenceBefore + 2.0e-3f )
                {
                    std::fprintf( stderr,
                        "cut-cell adjoint mismatch: h=%.3f mass=%.3f inertia=%.3f dt=%.6f "
                        "PCG=%.6g predicted=%.6g before=%.6g volumeL1=%.6g\n",
                        h, MASSES[i], INERTIAS[i], dt, stats.pressureResidual,
                        stats.predictedContinuityRms, stats.divergenceBefore,
                        stats.predictedSolidVolumeL1 );
                    check( false,
                        "pressure reaction and blocked-face fluid volume flux must satisfy the same continuity equation" );
                }
            }

    // 기존 고정 경계/실패 모드에는 예측 swept volume을 덧씌우지 않는다.
    fluidField2 legacy{ 24, 16, 0.5f, { -6.0f, -4.0f } };
    legacy.setCutCellProjectionEnabled( true );
    fluidShapeObstacle2 fixed{ circle2{ {}, 0.8f },
        { { 0.0f, 0.0f }, {} }, {}, { 0.9f, 0.0f }, 0.0f };
    std::array<fluidObstacleForce2, 1> force{};
    legacy.step( 1.0f / 60.0f, std::span<const fluidShapeObstacle2>( &fixed, 1 ), force, 100 );
    check( legacy.getCutCellPressure().predictedContinuityRms == 0.0f &&
        legacy.getCutCellPressure().predictedSolidVolumeL1 == 0.0f,
        "legacy cut-cell pressure must not pretend to predict coupled volume response" );
}

void checkDisjointAnalyticMultiShapeCutCell()
{
    constexpr float dt = 1.0f / 60.0f;
    constexpr float h = 0.5f;
    constexpr float domainArea = 24.0f * 16.0f * h * h;
    fluidField2 field{ 24, 16, h, { -6.0f, -4.0f } };
    field.setCutCellProjectionEnabled( true );
    field.setDisjointAnalyticCutCellEnabled( true );
    polygon2 rounded = MakeBox( { 0.34f, 0.29f } );
    rounded.radius = 0.17f;
    std::array<fluidShapeObstacle2, 4> objects{{
        { circle2{ {}, 0.42f }, { { -3.4f, -1.15f }, {} },
            { -3.4f, -1.15f }, {}, 0.0f },
        { capsule2{ { -0.4f, 0.0f }, { 0.4f, 0.0f }, 0.23f },
            { { -1.5f, 1.1f }, {} }, { -1.5f, 1.1f }, {}, 0.0f },
        { MakeBox( { 0.46f, 0.31f } ),
            { { 1.4f, -1.1f }, rot2::FromRadians( 0.32f ) },
            { 1.4f, -1.1f }, {}, 0.0f },
        { rounded, { { 3.65f, 1.0f }, rot2::FromRadians( 0.21f ) },
            { 3.65f, 1.0f }, {}, 0.0f }
    }};
    std::array<fluidObstacleForce2, 4> forces{};
    const float circleArea = std::numbers::pi_v<float> * 0.42f * 0.42f;
    const float capsuleArea = 2.0f * 0.23f * 0.8f +
        std::numbers::pi_v<float> * 0.23f * 0.23f;
    const float sharpArea = 4.0f * 0.46f * 0.31f;
    const float roundedArea = 4.0f * 0.34f * 0.29f +
        4.0f * ( 0.34f + 0.29f ) * 0.17f +
        std::numbers::pi_v<float> * 0.17f * 0.17f;
    const float expectedFluidArea = domainArea -
        ( circleArea + capsuleArea + sharpArea + roundedArea );

    field.step( dt, objects, forces, 160 );
    const auto& first = field.getCutCellTransition();
    check( first.usedDisjointAnalyticGeometry && first.disjointAnalyticShapes == 4 &&
        std::abs( first.fluidVolume - expectedFluidArea ) < 1.0e-4f &&
        !field.getCutCellPressure().hasVolumeHistory,
        "separated mixed circle/capsule/sharp/rounded shapes must have exact additive union area" );

    field.setCutCellSampleCount( 8 );
    field.step( dt, objects, forces, 160 );
    check( field.getCutCellTransition().usedDisjointAnalyticGeometry &&
        std::abs( field.getCutCellTransition().fluidVolume - expectedFluidArea ) < 1.0e-4f &&
        !field.getCutCellPressure().hasVolumeHistory,
        "disjoint analytic union must ignore 4x4/8x8 spatial resolution with clean history" );

    // Across a moving frame the obstacle AABBs remain separated.
    objects[0].transform.position.x += 0.07f;
    objects[0].center = objects[0].transform.position;
    objects[2].transform.rotation = rot2::FromRadians( 0.41f );
    field.step( dt, objects, forces, 160 );
    check( field.getCutCellTransition().usedDisjointAnalyticGeometry &&
        std::abs( field.getCutCellTransition().fluidVolume - expectedFluidArea ) < 1.0e-4f &&
        field.getCutCellPressure().hasVolumeHistory &&
        field.getSweptFaceFlux().usedSweptFlux &&
        field.getSweptFaceFlux().totalAbsoluteFlux > 0.0f &&
        std::isfinite( field.getSweptFaceFlux().geometryMismatch ),
        "moving and rotating separated shapes must use owner-specific swept face integration" );

    // Sharp Polygon may lie exactly on a grid line. Inclusive boundary
    // clipping must agree with the previous single-shape analytic convention.
    fluidField2 aligned{ 24, 16, h, { -6.0f, -4.0f } };
    aligned.setCutCellProjectionEnabled( true );
    aligned.setDisjointAnalyticCutCellEnabled( true );
    std::array<fluidShapeObstacle2, 2> alignedShapes{{
        { MakeBox( { 0.5f, 0.5f } ), { { 0.0f, 0.0f }, {} },
            {}, {}, 0.0f },
        { circle2{ {}, 0.25f }, { { 3.0f, 1.0f }, {} },
            { 3.0f, 1.0f }, {}, 0.0f }
    }};
    std::array<fluidObstacleForce2, 2> alignedFeedback{};
    aligned.step( dt, alignedShapes, alignedFeedback, 160 );
    check( aligned.getCutCellTransition().usedDisjointAnalyticGeometry &&
        std::abs( aligned.getCutCellTransition().fluidVolume -
            ( domainArea - 1.0f - std::numbers::pi_v<float> * 0.25f * 0.25f ) ) < 1.0e-4f &&
        aligned.getUFaceOpenFraction( 13, 8 ) < 2.0e-5f &&
        aligned.getVFaceOpenFraction( 12, 9 ) < 2.0e-5f,
        "multi analytic clipping must include grid-aligned sharp polygon boundary faces" );

    // Two disjoint circles can block different segments of *the same* U face.
    // Their face averages and Schur pressure owners must not be merged into body 0.
    fluidField2 shared{ 24, 16, h, { -6.0f, -4.0f } };
    shared.setCutCellProjectionEnabled( true );
    shared.setDisjointAnalyticCutCellEnabled( true );
    constexpr float radius = 0.13f;
    std::array<fluidShapeObstacle2, 2> circles{{
        { circle2{ {}, radius }, { { 0.0f, 0.08f }, {} },
            { 0.0f, 0.08f }, { 1.0f, 0.0f }, 0.0f },
        { circle2{ {}, radius }, { { 0.0f, 0.42f }, {} },
            { 0.0f, 0.42f }, { -0.2f, 0.0f }, 0.0f }
    }};
    std::array<fluidObstacleForce2, 2> reactions{};
    shared.step( dt, circles, reactions, 160 );
    const float blocked = ( radius + 0.08f ) + ( 0.50f - ( 0.42f - radius ) );
    check( shared.getCutCellTransition().usedDisjointAnalyticGeometry &&
        std::abs( shared.getCutCellTransition().fluidVolume -
            ( domainArea - 2.0f * std::numbers::pi_v<float> * radius * radius ) ) < 1.0e-4f &&
        std::abs( shared.getUFaceOpenFraction( 12, 8 ) - ( 1.0f - blocked / h ) ) < 2.0e-5f &&
        std::abs( shared.getUFaceSolidVelocity( 12, 8 ) - 0.4f ) < 2.0e-5f,
        "two distinct body owners on the same MAC face must share exact blocked length and weighted velocity" );

    // Transposed two-owner case: different solid segments of a shared V face.
    // Validate both the clipped aperture and owner-specific Schur feedback.
    fluidField2 vertical{ 24, 16, h, { -6.0f, -4.0f } };
    vertical.setCutCellProjectionEnabled( true );
    vertical.setDisjointAnalyticCutCellEnabled( true );
    std::array<fluidShapeObstacle2, 2> verticalCircles{{
        { circle2{ {}, radius }, { { 0.08f, 0.0f }, {} },
            { 0.08f, 0.0f }, { 0.0f, 1.0f }, 0.0f },
        { circle2{ {}, radius }, { { 0.42f, 0.0f }, {} },
            { 0.42f, 0.0f }, { 0.0f, -0.2f }, 0.0f }
    }};
    std::array<fluidObstacleForce2, 2> verticalReactions{};
    vertical.step( dt, verticalCircles, verticalReactions, 160 );
    check( vertical.getCutCellTransition().usedDisjointAnalyticGeometry &&
        std::abs( vertical.getVFaceOpenFraction( 12, 8 ) - ( 1.0f - blocked / h ) ) < 2.0e-5f &&
        std::abs( vertical.getVFaceSolidVelocity( 12, 8 ) - 0.4f ) < 2.0e-5f,
        "two distinct owners of a V face must preserve exact blocked length and weighted velocity" );

    vertical.setCutCellCoupledProjectionEnabled( true );
    verticalCircles[0].inverseMass = 0.5f;
    verticalCircles[1].inverseMass = 0.8f;
    verticalCircles[0].inverseInertia = 1.3f;
    verticalCircles[1].inverseInertia = 1.1f;
    vertical.addVelocity( { 0.25f, -1.1f }, { -0.4f, 1.3f }, 1.4f );
    vertical.step( dt, verticalCircles, verticalReactions, 200 );
    const auto& verticalPressure = vertical.getCutCellPressure();
    check( verticalPressure.usedCoupledProjection && !verticalPressure.coupledSolveFailed &&
        verticalPressure.coupledStatus == fluidCoupledPressureStatus2::converged &&
        std::isfinite( verticalPressure.predictedContinuityRms ) &&
        std::abs( verticalPressure.predictedContinuityRms - verticalPressure.pressureResidual ) <
            2.0e-3f + 0.1f * verticalPressure.pressureResidual,
        "same-V-face disjoint owners must retain Schur B/B-transpose blocked flux consistency" );

    shared.setCutCellCoupledProjectionEnabled( true );
    circles[0].inverseMass = 0.5f;
    circles[1].inverseMass = 0.8f;
    circles[0].inverseInertia = 1.3f;
    circles[1].inverseInertia = 1.1f;
    shared.addVelocity( { -1.1f, 0.25f }, { 1.3f, -0.4f }, 1.4f );
    shared.step( dt, circles, reactions, 200 );
    const auto& pressure = shared.getCutCellPressure();
    check( pressure.usedCoupledProjection && !pressure.coupledSolveFailed &&
        pressure.coupledStatus == fluidCoupledPressureStatus2::converged &&
        std::isfinite( pressure.predictedContinuityRms ) &&
        std::abs( pressure.predictedContinuityRms - pressure.pressureResidual ) <
            2.0e-3f + 0.1f * pressure.pressureResidual,
        "same-face disjoint owners must retain Schur B/B-transpose blocked flux consistency" );

    // Slightly intersecting circles inside the tolerance must use sampled union.
    // Do not misinterpret an analytic->sampled transition as swept displacement.
    circles[1].transform.position.y = 0.339999f;
    circles[1].center.y = 0.339999f;
    shared.step( dt, circles, reactions, 200 );
    fluidField2 sampled{ 24, 16, h, { -6.0f, -4.0f } };
    sampled.setCutCellProjectionEnabled( true );
    sampled.step( dt, circles, reactions, 200 );
    check( !shared.getCutCellTransition().usedDisjointAnalyticGeometry &&
        shared.getCutCellTransition().disjointAnalyticShapes == 0 &&
        !shared.getCutCellPressure().hasVolumeHistory &&
        !shared.getSweptFaceFlux().usedSweptFlux &&
        std::abs( shared.getCutCellTransition().fluidVolume -
            sampled.getCutCellTransition().fluidVolume ) < 1.0e-5f,
        "near-tangent circles must fall back to sampled union and invalidate geometry history" );

    circles[1].transform.position.y = 0.42f;
    circles[1].center.y = 0.42f;
    shared.step( dt, circles, reactions, 200 );
    check( shared.getCutCellTransition().usedDisjointAnalyticGeometry &&
        !shared.getCutCellPressure().hasVolumeHistory &&
        !shared.getSweptFaceFlux().usedSweptFlux,
        "returning to separated AABBs must not create artificial swept volume" );

    shared.setDisjointAnalyticCutCellEnabled( false );
    shared.step( dt, circles, reactions, 200 );
    check( !shared.getCutCellTransition().usedDisjointAnalyticGeometry &&
        !shared.getCutCellPressure().hasVolumeHistory,
        "disabling multi analytic option must restore original sampled geometry without stale history" );
    shared.clear();
    check( !shared.isDisjointAnalyticCutCellEnabled(),
        "clear must preserve an explicitly disabled disjoint multi analytic mode" );
}

void checkAnalyticCutCellPreconditionerComparison()
{
    constexpr float dt = 1.0f / 60.0f;
    constexpr vec2 origin{ -6.0f, -4.0f };
    const std::array<fluidShapeObstacle2, 2> bodies{{
        { circle2{ {}, 0.58f }, { { -0.21f, -0.18f }, {} },
            { -0.21f, -0.18f }, { 0.95f, -0.30f }, 0.70f, 0.35f, 1.10f },
        { circle2{ {}, 0.58f }, { { 0.26f, 0.19f }, {} },
            { 0.26f, 0.19f }, { -0.65f, 0.75f }, -0.55f, 0.80f, 0.70f }
    }};
    const auto seed = [&]( fluidField2& field, bool incomplete )
    {
        field.setCutCellProjectionEnabled( true );
        field.setCutCellCoupledProjectionEnabled( true );
        field.setDisjointAnalyticCutCellEnabled( true );
        field.setAnalyticCutCellIncompleteCholeskyEnabled( incomplete );
        field.setCutCellPressureTimingEnabled( true );
        field.addVelocity( { -1.2f, 0.14f }, { 1.4f, -0.27f }, 1.6f );
    };
    fluidField2 jacobi{ 24, 16, 0.5f, origin };
    fluidField2 ic0{ 24, 16, 0.5f, origin };
    check( jacobi.isAnalyticCutCellIncompleteCholeskyEnabled(),
        "IC0 must remain the default analytic Cut-cell preconditioner" );
    seed( jacobi, false );
    seed( ic0, true );
    std::array<fluidObstacleForce2, 2> jacobiForces{}, ic0Forces{};
    jacobi.step( dt, bodies, jacobiForces, 200 );
    ic0.step( dt, bodies, ic0Forces, 200 );
    const auto& a = jacobi.getCutCellPressure();
    const auto& b = ic0.getCutCellPressure();
    check( a.usedCoupledProjection && b.usedCoupledProjection &&
        !a.usedIncompleteCholesky && b.usedIncompleteCholesky &&
        a.coupledStatus == fluidCoupledPressureStatus2::converged &&
        b.coupledStatus == fluidCoupledPressureStatus2::converged &&
        a.coupledIterations > 0 && b.coupledIterations > 0 &&
        std::isfinite( a.coupledSolveMilliseconds ) && a.coupledSolveMilliseconds >= 0.0 &&
        std::isfinite( b.coupledSolveMilliseconds ) && b.coupledSolveMilliseconds >= 0.0,
        "Jacobi and IC0 must solve the identical analytic pressure system with measured elapsed time" );
    for( const auto* solved : { &a, &b } )
    {
        const fluidCoupledPressureTiming2& timing = solved->coupledTiming;
        check( timing.total > 0.0 && std::isfinite( timing.accounted() ) &&
            timing.accounted() <= timing.total + 0.1 &&
            timing.assembly > 0.0 && timing.validation > 0.0 &&
            timing.multiply > 0.0 && timing.preconditioner > 0.0 &&
            timing.matrixProducts == solved->coupledIterations &&
            timing.preconditionerApplications >= 1 &&
            timing.preconditionerApplications <= solved->coupledIterations + 1,
            "Schur PCG timing must separate non-overlapping phases and count actual operations" );
    }
    check( a.coupledTiming.factorization >= 0.0 &&
        b.coupledTiming.factorization > a.coupledTiming.factorization &&
        b.coupledTiming.preconditionerApplications > 0,
        "IC0 must report factorization separately from preconditioner application" );
    fluidCoupledPressureTiming2 aggregate = a.coupledTiming;
    aggregate.accumulate( b.coupledTiming );
    check( std::abs( aggregate.total - a.coupledTiming.total - b.coupledTiming.total ) < 1.0e-6 &&
        aggregate.matrixProducts == a.coupledTiming.matrixProducts + b.coupledTiming.matrixProducts,
        "solver phase diagnostics must aggregate independent solves" );

    check( std::abs( a.pressureResidual - b.pressureResidual ) < 2.0e-3f &&
        std::abs( a.predictedContinuityRms - b.predictedContinuityRms ) < 2.0e-3f,
        "the pressure residual and predicted continuity must agree across preconditioners" );
    for( int i = 0; i < 2; ++i )
    {
        const float scale = 1.0f + Length( ic0Forces[i].force );
        check( Length( jacobiForces[i].force - ic0Forces[i].force ) < 2.0e-2f * scale &&
            std::abs( jacobiForces[i].torque - ic0Forces[i].torque ) < 2.0e-2f * scale,
            "Jacobi and IC0 must return consistent owner-specific pressure feedback" );
    }
    // Higher resolution previously reached the Jacobi 200-iteration limit.
    fluidField2 fineJacobi{ 48, 32, 0.25f, origin };
    fluidField2 fineIc0{ 48, 32, 0.25f, origin };
    seed( fineJacobi, false );
    seed( fineIc0, true );
    fineJacobi.step( dt, bodies, jacobiForces, 200 );
    fineIc0.step( dt, bodies, ic0Forces, 200 );
    const auto& coarse = fineJacobi.getCutCellPressure();
    const auto& fine = fineIc0.getCutCellPressure();
    check( !coarse.usedIncompleteCholesky && fine.usedIncompleteCholesky &&
        coarse.coupledSolveFailed &&
        coarse.coupledStatus == fluidCoupledPressureStatus2::iterationLimit &&
        !coarse.hasAnalyticImpulseBudget &&
        LengthSquared( jacobiForces[0].force ) == 0.0f &&
        LengthSquared( jacobiForces[1].force ) == 0.0f &&
        fine.usedCoupledProjection &&
        fine.coupledStatus == fluidCoupledPressureStatus2::converged &&
        fine.hasAnalyticImpulseBudget && fine.coupledIterations < 200,
        "IC0 must converge where fine-grid Jacobi safely reaches its established iteration limit" );
    check( coarse.coupledTiming.total > 0.0 && fine.coupledTiming.total > 0.0 &&
        coarse.coupledTiming.matrixProducts == coarse.coupledIterations &&
        fine.coupledTiming.matrixProducts == fine.coupledIterations &&
        coarse.coupledTiming.validation > 0.0 &&
        fine.coupledTiming.validation > 0.0 &&
        !coarse.hasAnalyticImpulseBudget,
        "failed Jacobi and converged IC0 must both retain their actual PCG workload diagnostics" );
    fineJacobi.clear();
    check( !fineJacobi.isAnalyticCutCellIncompleteCholeskyEnabled() &&
        fineJacobi.isCutCellPressureTimingEnabled() &&
        fineJacobi.getCutCellPressure().coupledSolveMilliseconds == 0.0 &&
        fineJacobi.getCutCellPressure().coupledTiming.total == 0.0,
        "clear must preserve the benchmark mode and reset elapsed-time diagnostics" );
    fineIc0.setCutCellPressureTimingEnabled( false );
    fineIc0.clear();
    fineIc0.addVelocity( { -1.2f, 0.14f }, { 1.4f, -0.27f }, 1.6f );
    fineIc0.step( dt, bodies, ic0Forces, 200 );
    check( fineIc0.getCutCellPressure().coupledTiming.total == 0.0 &&
        fineIc0.getCutCellPressure().coupledTiming.matrixProducts == 0 &&
        fineIc0.getCutCellPressure().coupledSolveMilliseconds == 0.0,
        "disabled cut-cell profiling must not retain stale phase counts or durations" );
}

void checkCachedIc0Differential()
{
    constexpr float dt = 1.0f / 60.0f;
    constexpr vec2 origin{ -6.0f, -4.0f };
    for( float h : { 0.5f, 0.25f } )
        for( int mode = 0; mode < 3; ++mode )
        {
            std::array<fluidShapeObstacle2, 2> bodies{{
                { circle2{ {}, mode == 2 ? 0.78f : 0.58f },
                    { { -0.21f, -0.18f }, {} }, { -0.21f, -0.18f },
                    { 0.95f, -0.30f }, 0.70f, 0.35f, 1.10f },
                { circle2{ {}, mode == 2 ? 0.18f : 0.58f },
                    { { mode == 1 ? 1.50f : 0.26f, 0.19f }, {} },
                    { mode == 1 ? 1.50f : 0.26f, 0.19f },
                    { -0.65f, 0.75f }, -0.55f, 0.80f, 0.70f }
            }};
            if( mode == 2 )
            {
                bodies[1].transform.position = { -0.12f, -0.12f };
                bodies[1].center = bodies[1].transform.position;
            }
            fluidField2 reference{ static_cast<int>( 12.0f / h ),
                static_cast<int>( 8.0f / h ), h, origin };
            fluidField2 cached{ static_cast<int>( 12.0f / h ),
                static_cast<int>( 8.0f / h ), h, origin };
            check( cached.isAnalyticCutCellCachedIc0Enabled(),
                "per-solve cached IC0 sweep must default to on" );
            for( fluidField2* field : { &reference, &cached } )
            {
                field->setCutCellProjectionEnabled( true );
                field->setCutCellCoupledProjectionEnabled( true );
                field->setDisjointAnalyticCutCellEnabled( true );
                field->setCutCellPressureTimingEnabled( true );
                field->addVelocity( { -1.2f, 0.14f }, { 1.4f, -0.27f }, 1.6f );
            }
            reference.setAnalyticCutCellCachedIc0Enabled( false );
            std::array<fluidObstacleForce2, 2> originalForces{}, cachedForces{};
            // Multiple moving snapshots must rebuild coefficients; no stale
            // face opening or pivot from a previous pressure solve may survive.
            for( int frame = 0; frame < 4; ++frame )
            {
                reference.step( dt, bodies, originalForces, 200 );
                cached.step( dt, bodies, cachedForces, 200 );
                const auto& a = reference.getCutCellPressure();
                const auto& b = cached.getCutCellPressure();
                check( a.usedCoupledProjection && b.usedCoupledProjection &&
                    a.coupledStatus == fluidCoupledPressureStatus2::converged &&
                    b.coupledStatus == fluidCoupledPressureStatus2::converged &&
                    a.usedIncompleteCholesky && b.usedIncompleteCholesky &&
                    a.coupledRelativeResidual <= 1.1e-5f &&
                    b.coupledRelativeResidual <= 1.1e-5f &&
                    a.hasAnalyticImpulseBudget && b.hasAnalyticImpulseBudget &&
                    a.coupledTiming.preconditionerApplications > 0 &&
                    b.coupledTiming.preconditionerApplications > 0,
                    "reference and cached IC0 must both pass true residual and impulse-budget gates" );
                check( std::abs( a.predictedContinuityRms - b.predictedContinuityRms ) < 5.0e-3f &&
                    std::abs( a.pressureResidual - b.pressureResidual ) < 5.0e-3f &&
                    std::abs( a.coupledIterations - b.coupledIterations ) <= 3,
                    "cached IC0 must preserve coupled pressure continuity and comparable convergence" );
                for( std::size_t i = 0; i < bodies.size(); ++i )
                {
                    const float scale = 1.0f + Length( originalForces[i].force );
                    check( Length( originalForces[i].force - cachedForces[i].force ) < 2.0e-2f * scale &&
                        std::abs( originalForces[i].torque - cachedForces[i].torque ) < 2.0e-2f * scale,
                        "cached IC0 must preserve each body's linear/angular pressure feedback" );
                }
                const vec2 probe{ -1.05f, 0.14f };
                const float momentumScale = 1.0f + Length( reference.getFluidMomentum() );
                check( Length( reference.sampleVelocity( probe ) - cached.sampleVelocity( probe ) ) < 2.0e-2f &&
                    Length( reference.getFluidMomentum() - cached.getFluidMomentum() ) < 2.0e-2f * momentumScale,
                    "cached IC0 must not significantly change the fluid velocity or momentum" );
                for( fluidShapeObstacle2& body : bodies )
                {
                    body.transform.position += dt * body.linearVelocity;
                    body.center = body.transform.position;
                }
            }
            cached.clear();
            check( cached.isAnalyticCutCellCachedIc0Enabled() &&
                cached.getCutCellPressure().coupledTiming.total == 0.0,
                "clear must retain cached IC0 choice without stale pressure timing" );
            reference.clear();
            check( !reference.isAnalyticCutCellCachedIc0Enabled(),
                "clear must retain reference IC0 comparison option" );
        }
}

void checkActiveCellIc0SweepDifferential()
{
    constexpr float dt = 1.0f / 60.0f;
    constexpr vec2 origin{ -6.0f, -4.0f };
    for( float h : { 0.5f, 0.25f } )
        for( int mode = 0; mode < 3; ++mode )
        {
            std::array<fluidShapeObstacle2, 2> bodies{{
                { circle2{ {}, mode == 2 ? 0.78f : 0.58f },
                    { { -0.21f, -0.18f }, {} }, { -0.21f, -0.18f },
                    { 0.95f, -0.30f }, 0.70f, 0.35f, 1.10f },
                { circle2{ {}, mode == 2 ? 0.18f : 0.58f },
                    { { mode == 1 ? 1.50f : 0.26f, 0.19f }, {} },
                    { mode == 1 ? 1.50f : 0.26f, 0.19f },
                    { -0.65f, 0.75f }, -0.55f, 0.80f, 0.70f }
            }};
            if( mode == 2 )
            {
                bodies[1].transform.position = { -0.12f, -0.12f };
                bodies[1].center = bodies[1].transform.position;
            }
            fluidField2 dense{ static_cast<int>( 12.0f / h ),
                static_cast<int>( 8.0f / h ), h, origin };
            fluidField2 active{ static_cast<int>( 12.0f / h ),
                static_cast<int>( 8.0f / h ), h, origin };
            check( active.isAnalyticCutCellActiveIc0SweepEnabled(),
                "active-cell IC0 sweep should be the default" );
            for( fluidField2* field : { &dense, &active } )
            {
                field->setCutCellProjectionEnabled( true );
                field->setCutCellCoupledProjectionEnabled( true );
                field->setDisjointAnalyticCutCellEnabled( true );
                field->setCutCellPressureTimingEnabled( true );
                field->addVelocity( { -1.2f, 0.14f }, { 1.4f, -0.27f }, 1.6f );
            }
            dense.setAnalyticCutCellActiveIc0SweepEnabled( false );
            std::array<fluidObstacleForce2, 2> denseForces{}, activeForces{};
            // Recompute the active-cell schedule from each changed cut-cell
            // geometry. Previously solid cells cannot be carried over.
            for( int frame = 0; frame < 4; ++frame )
            {
                dense.step( dt, bodies, denseForces, 200 );
                active.step( dt, bodies, activeForces, 200 );
                const auto& a = dense.getCutCellPressure();
                const auto& b = active.getCutCellPressure();
                check( a.usedCoupledProjection && b.usedCoupledProjection &&
                    a.coupledStatus == fluidCoupledPressureStatus2::converged &&
                    b.coupledStatus == fluidCoupledPressureStatus2::converged &&
                    a.usedIncompleteCholesky && b.usedIncompleteCholesky &&
                    a.hasAnalyticImpulseBudget && b.hasAnalyticImpulseBudget &&
                    a.coupledIterations == b.coupledIterations &&
                    a.coupledTiming.matrixProducts == b.coupledTiming.matrixProducts &&
                    a.coupledRelativeResidual <= 1.1e-5f &&
                    b.coupledRelativeResidual <= 1.1e-5f,
                    "dense and active-cell IC0 must converge with the same PCG workload" );
                check( std::abs( a.coupledRelativeResidual - b.coupledRelativeResidual ) < 1.0e-6f &&
                    std::abs( a.predictedContinuityRms - b.predictedContinuityRms ) < 2.0e-4f &&
                    std::abs( a.pressureResidual - b.pressureResidual ) < 2.0e-4f,
                    "active-cell IC0 must retain true pressure residual and continuity" );
                for( std::size_t i = 0; i < bodies.size(); ++i )
                {
                    const float scale = 1.0f + Length( denseForces[i].force );
                    check( Length( denseForces[i].force - activeForces[i].force ) < 1.0e-3f * scale &&
                        std::abs( denseForces[i].torque - activeForces[i].torque ) < 1.0e-3f * scale,
                        "active-cell IC0 must preserve individual body pressure feedback" );
                }
                const float massScale = 1.0f + Length( dense.getFluidMomentum() );
                check( Length( dense.getFluidMomentum() - active.getFluidMomentum() ) < 1.0e-3f * massScale &&
                    Length( dense.sampleVelocity( { -1.05f, 0.14f } ) -
                        active.sampleVelocity( { -1.05f, 0.14f } ) ) < 1.0e-3f,
                    "active-cell IC0 must preserve fluid velocities and momentum" );
                for( fluidShapeObstacle2& body : bodies )
                {
                    body.transform.position += dt * body.linearVelocity;
                    body.center = body.transform.position;
                }
            }
            dense.clear();
            active.clear();
            check( !dense.isAnalyticCutCellActiveIc0SweepEnabled() &&
                active.isAnalyticCutCellActiveIc0SweepEnabled() &&
                dense.getCutCellPressure().coupledTiming.total == 0.0 &&
                active.getCutCellPressure().coupledTiming.total == 0.0,
                "clear must preserve the selected active sweep mode and discard old data" );
        }
}

void checkCachedSchurLaplacianDifferential()
{
    constexpr float dt = 1.0f / 60.0f;
    constexpr vec2 origin{ -6.0f, -4.0f };
    for( float h : { 0.5f, 0.25f } )
        for( int mode = 0; mode < 3; ++mode )
        {
            std::array<fluidShapeObstacle2, 2> bodies{{
                { circle2{ {}, mode == 2 ? 0.78f : 0.58f },
                    { { -0.21f, -0.18f }, {} }, { -0.21f, -0.18f },
                    { 0.95f, -0.30f }, 0.70f, 0.35f, 1.10f },
                { circle2{ {}, mode == 2 ? 0.18f : 0.58f },
                    { { mode == 1 ? 1.50f : 0.26f, 0.19f }, {} },
                    { mode == 1 ? 1.50f : 0.26f, 0.19f },
                    { -0.65f, 0.75f }, -0.55f, 0.80f, 0.70f }
            }};
            if( mode == 1 )
            {
                // Mixed static/dynamic bodies must retain coupled pressure
                // convergence and immovable obstacle feedback.
                bodies[1].inverseMass = 0.0f;
                bodies[1].inverseInertia = 0.0f;
            }
            if( mode == 2 )
            {
                bodies[1].transform.position = { -0.12f, -0.12f };
                bodies[1].center = bodies[1].transform.position;
            }
            fluidField2 reference{ static_cast<int>( 12.0f / h ),
                static_cast<int>( 8.0f / h ), h, origin };
            fluidField2 cached{ static_cast<int>( 12.0f / h ),
                static_cast<int>( 8.0f / h ), h, origin };
            check( cached.isAnalyticCutCellCachedSchurEdgesEnabled() &&
                cached.isAnalyticCutCellIndexedBoundaryFacesEnabled(),
                "analytic Schur edge and boundary index caches must be on by default" );
            for( fluidField2* field : { &reference, &cached } )
            {
                field->setCutCellProjectionEnabled( true );
                field->setCutCellCoupledProjectionEnabled( true );
                field->setDisjointAnalyticCutCellEnabled( true );
                field->setCutCellPressureTimingEnabled( true );
                field->addVelocity( { -1.2f, 0.14f }, { 1.4f, -0.27f }, 1.6f );
            }
            reference.setAnalyticCutCellCachedSchurEdgesEnabled( false );
            reference.setAnalyticCutCellIndexedBoundaryFacesEnabled( false );
            std::array<fluidObstacleForce2, 2> refForces{}, cachedForces{};
            // The edge list must be rebuilt for every changing occupancy.
            for( int frame = 0; frame < 4; ++frame )
            {
                reference.step( dt, bodies, refForces, 200 );
                cached.step( dt, bodies, cachedForces, 200 );
                const auto& a = reference.getCutCellPressure();
                const auto& b = cached.getCutCellPressure();
                check( a.usedCoupledProjection && b.usedCoupledProjection &&
                    a.coupledStatus == fluidCoupledPressureStatus2::converged &&
                    b.coupledStatus == fluidCoupledPressureStatus2::converged &&
                    a.usedIncompleteCholesky && b.usedIncompleteCholesky &&
                    a.hasAnalyticImpulseBudget && b.hasAnalyticImpulseBudget &&
                    a.coupledIterations == b.coupledIterations &&
                    a.coupledTiming.matrixProducts == b.coupledTiming.matrixProducts &&
                    a.coupledTiming.cachedLaplacianEdges == 0 &&
                    b.coupledTiming.cachedLaplacianEdges > 0,
                    "Schur edge cache must preserve solver convergence while recording reused faces" );
                check( std::abs( a.coupledRelativeResidual - b.coupledRelativeResidual ) < 1.0e-6f &&
                    std::abs( a.predictedContinuityRms - b.predictedContinuityRms ) < 1.0e-4f &&
                    std::abs( a.pressureResidual - b.pressureResidual ) < 1.0e-4f,
                    "cached Laplacian and direct matrix-free paths must retain true residual and continuity" );
                for( std::size_t i = 0; i < bodies.size(); ++i )
                {
                    const float scale = 1.0f + Length( refForces[i].force );
                    check( Length( refForces[i].force - cachedForces[i].force ) < 1.0e-4f * scale &&
                        std::abs( refForces[i].torque - cachedForces[i].torque ) < 1.0e-4f * scale,
                        "cached Schur edges must preserve per-body pressure force and torque" );
                }
                const float scale = 1.0f + Length( reference.getFluidMomentum() );
                check( Length( reference.getFluidMomentum() - cached.getFluidMomentum() ) < 1.0e-4f * scale &&
                    Length( reference.sampleVelocity( { -1.05f, 0.14f } ) -
                        cached.sampleVelocity( { -1.05f, 0.14f } ) ) < 1.0e-4f,
                    "cached Schur edges must not alter fluid momentum and velocity" );
                if( mode == 1 )
                {
                    // A static obstacle may become dynamic and return to
                    // static without stale Schur response entering the solve.
                    if( frame == 1 )
                    {
                        bodies[1].inverseMass = 0.8f;
                        bodies[1].inverseInertia = 0.7f;
                    }
                    else if( frame == 2 )
                    {
                        bodies[1].inverseMass = 0.0f;
                        bodies[1].inverseInertia = 0.0f;
                    }
                }
                for( fluidShapeObstacle2& body : bodies )
                {
                    body.transform.position += dt * body.linearVelocity;
                    body.center = body.transform.position;
                }
            }
            cached.clear();
            reference.clear();
            check( cached.isAnalyticCutCellCachedSchurEdgesEnabled() &&
                cached.isAnalyticCutCellIndexedBoundaryFacesEnabled() &&
                !reference.isAnalyticCutCellCachedSchurEdgesEnabled() &&
                !reference.isAnalyticCutCellIndexedBoundaryFacesEnabled() &&
                cached.getCutCellPressure().coupledTiming.cachedLaplacianEdges == 0,
                "clear must retain selected edge-cache option but discard old data" );
        }
}

void checkIndexedAnalyticBoundarySyncCircleUnion()
{
    constexpr float dt = 1.0f / 60.0f;
    for( float h : { 0.5f, 0.25f } )
        for( int mode = 0; mode < 2; ++mode )
        {
            const bool contained = mode == 1;
            const vec2 first{ -0.21f, -0.18f };
            const vec2 second = contained ? vec2{ -0.12f, -0.12f } :
                vec2{ 0.26f, 0.19f };
            std::array<fluidShapeObstacle2, 2> bodies{{
                { circle2{ {}, contained ? 0.78f : 0.58f }, { first, {} }, first,
                    { 0.95f, -0.30f }, 0.70f, 0.35f, 1.10f },
                { circle2{ {}, contained ? 0.18f : 0.58f }, { second, {} }, second,
                    { -0.65f, 0.75f }, -0.55f, 0.80f, 0.70f }
            }};
            fluidField2 reference{ static_cast<int>( 12.0f / h ),
                static_cast<int>( 8.0f / h ), h, { -6.0f, -4.0f } };
            fluidField2 indexed{ static_cast<int>( 12.0f / h ),
                static_cast<int>( 8.0f / h ), h, { -6.0f, -4.0f } };
            for( fluidField2* field : { &reference, &indexed } )
            {
                field->setCutCellProjectionEnabled( true );
                field->setCutCellCoupledProjectionEnabled( true );
                field->setDisjointAnalyticCutCellEnabled( true );
                field->addVelocity( { -1.2f, 0.14f }, { 1.4f, -0.27f }, 1.6f );
            }
            reference.setAnalyticCutCellIndexedBoundarySyncEnabled( false );
            std::array<fluidObstacleForce2, 2> refFeedback{}, indexedFeedback{};
            for( int frame = 0; frame < 3; ++frame )
            {
                reference.step( dt, bodies, refFeedback, 200 );
                indexed.step( dt, bodies, indexedFeedback, 200 );
                const auto& a = reference.getCutCellPressure();
                const auto& b = indexed.getCutCellPressure();
                check( a.usedCoupledProjection && b.usedCoupledProjection &&
                    a.coupledIterations == b.coupledIterations &&
                    std::abs( a.coupledRelativeResidual - b.coupledRelativeResidual ) < 1.0e-6f &&
                    std::abs( a.predictedContinuityRms - b.predictedContinuityRms ) < 1.0e-5f,
                    "indexed boundary sync must preserve overlapping-circle continuity" );
                for( int y = 0; y < reference.getHeight(); ++y )
                    for( int x = 1; x < reference.getWidth(); ++x )
                        check( reference.getUFaceSolidVelocity( x, y ) ==
                            indexed.getUFaceSolidVelocity( x, y ),
                            "indexed U boundary sync must preserve overlapping-circle solid speed" );
                for( int y = 1; y < reference.getHeight(); ++y )
                    for( int x = 0; x < reference.getWidth(); ++x )
                        check( reference.getVFaceSolidVelocity( x, y ) ==
                            indexed.getVFaceSolidVelocity( x, y ),
                            "indexed V boundary sync must preserve overlapping-circle solid speed" );
                for( std::size_t i = 0; i < bodies.size(); ++i )
                {
                    const float scale = 1.0f + Length( refFeedback[i].force );
                    check( Length( refFeedback[i].force - indexedFeedback[i].force ) <
                        1.0e-4f * scale, "indexed boundary sync must preserve circle traction" );
                    bodies[i].transform.position += dt * bodies[i].linearVelocity;
                    bodies[i].center = bodies[i].transform.position;
                }
            }
        }
}

void checkIndexedAnalyticBoundaryDifferential( bool compareMovableOnly = false,
    bool compareBoundarySync = false )
{
    constexpr float dt = 1.0f / 60.0f;
    for( float h : { 0.5f, 0.25f } )
        for( int staticCount : { 16, 48 } )
        {
            std::vector<fluidShapeObstacle2> bodies{
                { circle2{ {}, 0.58f }, { { -0.21f, -0.18f }, {} }, { -0.21f, -0.18f },
                    { 0.95f, -0.30f }, 0.70f, 0.35f, 1.10f },
                { circle2{ {}, 0.58f }, { { 1.50f, 0.19f }, {} }, { 1.50f, 0.19f },
                    { -0.65f, 0.75f }, -0.55f, 0.80f, 0.70f }
            };
            int added = 0;
            for( int row = 0; row < 7 && added < staticCount; ++row )
                for( int col = 0; col < 10 && added < staticCount; ++col )
                {
                    const vec2 position{ -5.25f + 1.05f * col, -3.15f + 0.95f * row };
                    bool separated = true;
                    for( const auto& body : bodies )
                    {
                        const float clearance = std::get<circle2>( body.geometry ).radius + 0.17f;
                        if( LengthSquared( position - body.center ) < clearance * clearance )
                        {
                            separated = false;
                            break;
                        }
                    }
                    if( !separated ) continue;
                    bodies.push_back( { circle2{ {}, 0.12f }, { position, {} },
                        position, {}, 0.0f, 0.0f, 0.0f } );
                    ++added;
                }
            check( added == staticCount,
                "static Schur comparison must contain the requested disjoint obstacles" );

            fluidField2 reference{ static_cast<int>( 12.0f / h ),
                static_cast<int>( 8.0f / h ), h, { -6.0f, -4.0f } };
            fluidField2 indexed{ static_cast<int>( 12.0f / h ),
                static_cast<int>( 8.0f / h ), h, { -6.0f, -4.0f } };
            for( fluidField2* field : { &reference, &indexed } )
            {
                field->setCutCellProjectionEnabled( true );
                field->setCutCellCoupledProjectionEnabled( true );
                field->setDisjointAnalyticCutCellEnabled( true );
                field->addVelocity( { -1.2f, 0.14f }, { 1.4f, -0.27f }, 1.6f );
            }
            if( compareBoundarySync )
                reference.setAnalyticCutCellIndexedBoundarySyncEnabled( false );
            else if( compareMovableOnly )
                reference.setAnalyticCutCellMovableSchurFacesEnabled( false );
            else
                reference.setAnalyticCutCellIndexedBoundaryFacesEnabled( false );
            std::vector<fluidObstacleForce2> refForces( bodies.size() ), indexedForces( bodies.size() );
            for( int frame = 0; frame < 3; ++frame )
            {
                reference.step( dt, bodies, refForces, 200 );
                indexed.step( dt, bodies, indexedForces, 200 );
                const auto& a = reference.getCutCellPressure();
                const auto& b = indexed.getCutCellPressure();
                check( a.usedCoupledProjection && b.usedCoupledProjection &&
                    a.coupledStatus == fluidCoupledPressureStatus2::converged &&
                    b.coupledStatus == fluidCoupledPressureStatus2::converged &&
                    a.coupledIterations == b.coupledIterations &&
                    std::abs( a.coupledRelativeResidual - b.coupledRelativeResidual ) < 1.0e-6f &&
                    std::abs( a.predictedContinuityRms - b.predictedContinuityRms ) < 1.0e-4f,
                    "indexed analytic faces must preserve Schur convergence with many static bodies" );
                check( indexed.getCutCellTransition().usedDisjointAnalyticGeometry &&
                    reference.getCutCellTransition().usedDisjointAnalyticGeometry,
                    "indexed analytic differential must remain on the analytic geometry path" );
                if( compareBoundarySync )
                {
                    for( int y = 0; y < reference.getHeight(); ++y )
                        for( int x = 1; x < reference.getWidth(); ++x )
                            check( reference.getUFaceOpenFraction( x, y ) ==
                                    indexed.getUFaceOpenFraction( x, y ) &&
                                reference.getUFaceSolidVelocity( x, y ) ==
                                    indexed.getUFaceSolidVelocity( x, y ),
                                "indexed boundary sync must preserve every U-face aperture and solid speed" );
                    for( int y = 1; y < reference.getHeight(); ++y )
                        for( int x = 0; x < reference.getWidth(); ++x )
                            check( reference.getVFaceOpenFraction( x, y ) ==
                                    indexed.getVFaceOpenFraction( x, y ) &&
                                reference.getVFaceSolidVelocity( x, y ) ==
                                    indexed.getVFaceSolidVelocity( x, y ),
                                "indexed boundary sync must preserve every V-face aperture and solid speed" );
                }
                for( std::size_t i = 0; i < bodies.size(); ++i )
                {
                    const float scale = 1.0f + Length( refForces[i].force );
                    check( Length( refForces[i].force - indexedForces[i].force ) < 1.0e-4f * scale &&
                        std::abs( refForces[i].torque - indexedForces[i].torque ) < 1.0e-4f * scale,
                        "indexed analytic faces must retain dynamic and static traction" );
                }
                const float scale = 1.0f + Length( reference.getFluidMomentum() );
                check( Length( reference.getFluidMomentum() - indexed.getFluidMomentum() ) <
                    1.0e-4f * scale, "indexed analytic faces must preserve fluid momentum" );
                for( std::size_t i = 0; i < 2; ++i )
                {
                    bodies[i].transform.position += dt * bodies[i].linearVelocity;
                    bodies[i].center = bodies[i].transform.position;
                }
            }
            reference.clear();
            indexed.clear();
            check( compareBoundarySync
                    ? !reference.isAnalyticCutCellIndexedBoundarySyncEnabled() &&
                        indexed.isAnalyticCutCellIndexedBoundarySyncEnabled()
                    : compareMovableOnly
                        ? !reference.isAnalyticCutCellMovableSchurFacesEnabled() &&
                            indexed.isAnalyticCutCellMovableSchurFacesEnabled()
                        : !reference.isAnalyticCutCellIndexedBoundaryFacesEnabled() &&
                            indexed.isAnalyticCutCellIndexedBoundaryFacesEnabled(),
                "clear must preserve the selected analytic boundary processing mode" );
        }
}

void checkBoundedAnalyticFaceClippingDifferential()
{
    constexpr float dt = 1.0f / 60.0f;
    for( float h : { 0.5f, 0.25f } )
        for( int staticCount : { 16, 48 } )
        {
            std::vector<fluidShapeObstacle2> bodies{
                { circle2{ {}, 0.58f }, { { -0.21f, -0.18f }, {} }, { -0.21f, -0.18f },
                    { 0.95f, -0.30f }, 0.70f, 0.35f, 1.10f },
                { circle2{ {}, 0.58f }, { { 1.50f, 0.19f }, {} }, { 1.50f, 0.19f },
                    { -0.65f, 0.75f }, -0.55f, 0.80f, 0.70f }
            };
            int added = 0;
            for( int row = 0; row < 7 && added < staticCount; ++row )
                for( int col = 0; col < 10 && added < staticCount; ++col )
                {
                    const vec2 position{ -5.25f + 1.05f * col, -3.15f + 0.95f * row };
                    bool separated = true;
                    for( const auto& body : bodies )
                    {
                        const float clearance = std::get<circle2>( body.geometry ).radius + 0.17f;
                        if( LengthSquared( position - body.center ) < clearance * clearance )
                        {
                            separated = false;
                            break;
                        }
                    }
                    if( !separated ) continue;
                    bodies.push_back( { circle2{ {}, 0.12f }, { position, {} },
                        position, {}, 0.0f, 0.0f, 0.0f } );
                    ++added;
                }
            check( added == staticCount,
                "static Schur comparison must contain the requested disjoint obstacles" );

            fluidField2 reference{ static_cast<int>( 12.0f / h ),
                static_cast<int>( 8.0f / h ), h, { -6.0f, -4.0f } };
            fluidField2 bounded{ static_cast<int>( 12.0f / h ),
                static_cast<int>( 8.0f / h ), h, { -6.0f, -4.0f } };
            for( fluidField2* field : { &reference, &bounded } )
            {
                field->setCutCellProjectionEnabled( true );
                field->setCutCellCoupledProjectionEnabled( true );
                field->setDisjointAnalyticCutCellEnabled( true );
                field->addVelocity( { -1.2f, 0.14f }, { 1.4f, -0.27f }, 1.6f );
            }
            reference.setAnalyticCutCellBoundedFaceClippingEnabled( false );
            std::vector<fluidObstacleForce2> refForces( bodies.size() ), boundedForces( bodies.size() );
            for( int frame = 0; frame < 3; ++frame )
            {
                reference.step( dt, bodies, refForces, 200 );
                bounded.step( dt, bodies, boundedForces, 200 );
                const auto& a = reference.getCutCellPressure();
                const auto& b = bounded.getCutCellPressure();
                check( a.usedCoupledProjection && b.usedCoupledProjection &&
                    a.coupledStatus == fluidCoupledPressureStatus2::converged &&
                    b.coupledStatus == fluidCoupledPressureStatus2::converged &&
                    a.coupledIterations == b.coupledIterations &&
                    std::abs( a.coupledRelativeResidual - b.coupledRelativeResidual ) < 1.0e-6f &&
                    std::abs( a.predictedContinuityRms - b.predictedContinuityRms ) < 1.0e-4f,
                    "bounded analytic faces must preserve Schur convergence with many static bodies" );
                check( bounded.getCutCellTransition().usedDisjointAnalyticGeometry &&
                    reference.getCutCellTransition().usedDisjointAnalyticGeometry,
                    "bounded analytic differential must remain on the analytic geometry path" );
                for( int y = 0; y < bounded.getHeight(); ++y )
                    for( int x = 1; x < bounded.getWidth(); ++x )
                        check( reference.getUFaceOpenFraction( x, y ) ==
                            bounded.getUFaceOpenFraction( x, y ) &&
                            reference.getUFaceSolidVelocity( x, y ) ==
                            bounded.getUFaceSolidVelocity( x, y ),
                            "bounded analytic U faces must preserve apertures and velocities" );
                for( int y = 1; y < bounded.getHeight(); ++y )
                    for( int x = 0; x < bounded.getWidth(); ++x )
                        check( reference.getVFaceOpenFraction( x, y ) ==
                            bounded.getVFaceOpenFraction( x, y ) &&
                            reference.getVFaceSolidVelocity( x, y ) ==
                            bounded.getVFaceSolidVelocity( x, y ),
                            "bounded analytic V faces must preserve apertures and velocities" );
                for( std::size_t i = 0; i < bodies.size(); ++i )
                {
                    const float scale = 1.0f + Length( refForces[i].force );
                    check( Length( refForces[i].force - boundedForces[i].force ) < 1.0e-4f * scale &&
                        std::abs( refForces[i].torque - boundedForces[i].torque ) < 1.0e-4f * scale,
                        "bounded analytic faces must retain dynamic and static traction" );
                }
                const float scale = 1.0f + Length( reference.getFluidMomentum() );
                check( Length( reference.getFluidMomentum() - bounded.getFluidMomentum() ) <
                    1.0e-4f * scale, "bounded analytic faces must preserve fluid momentum" );
                for( std::size_t i = 0; i < 2; ++i )
                {
                    bodies[i].transform.position += dt * bodies[i].linearVelocity;
                    bodies[i].center = bodies[i].transform.position;
                }
            }
            reference.clear();
            bounded.clear();
            check( !reference.isAnalyticCutCellBoundedFaceClippingEnabled() &&
                bounded.isAnalyticCutCellBoundedFaceClippingEnabled(),
                "clear must preserve selected analytic boundary indexing mode" );
        }
}

void checkBoundedAnalyticCellAreaDifferential( bool compareGeometryReuse = false )
{
    constexpr float dt = 1.0f / 60.0f;
    for( float h : { 0.5f, 0.25f } )
        for( int mode = 0; mode < 3; ++mode )
        {
            // Include overlapping circles, strictly contained circles and
            // many separated static obstacles in the analytic geometry path.
            const float radiusA = mode == 2 ? 0.78f : 0.58f;
            const float radiusB = mode == 2 ? 0.18f : 0.58f;
            const vec2 second = mode == 1 ? vec2{ 1.5f, 0.19f } :
                mode == 2 ? vec2{ -0.12f, -0.12f } : vec2{ 0.26f, 0.19f };
            std::vector<fluidShapeObstacle2> bodies{
                { circle2{ {}, radiusA }, { { -0.21f, -0.18f }, {} },
                    { -0.21f, -0.18f }, { 0.95f, -0.30f }, 0.70f, 0.35f, 1.10f },
                { circle2{ {}, radiusB }, { second, {} }, second,
                    { -0.65f, 0.75f }, -0.55f, 0.80f, 0.70f }
            };
            if( mode == 1 )
            {
                int added = 0;
                for( int row = 0; row < 7 && added < 48; ++row )
                    for( int col = 0; col < 10 && added < 48; ++col )
                    {
                        const vec2 position{ -5.25f + 1.05f * col, -3.15f + 0.95f * row };
                        bool separated = true;
                        for( const auto& body : bodies )
                        {
                            const float clearance = std::get<circle2>( body.geometry ).radius + 0.17f;
                            if( LengthSquared( position - body.center ) < clearance * clearance )
                            {
                                separated = false;
                                break;
                            }
                        }
                        if( !separated ) continue;
                        bodies.push_back( { circle2{ {}, 0.12f }, { position, {} },
                            position, {}, 0.0f, 0.0f, 0.0f } );
                        ++added;
                    }
                check( added == 48, "bounded cell area test needs 48 separated static obstacles" );
            }

            fluidField2 reference{ static_cast<int>( 12.0f / h ),
                static_cast<int>( 8.0f / h ), h, { -6.0f, -4.0f } };
            fluidField2 bounded{ static_cast<int>( 12.0f / h ),
                static_cast<int>( 8.0f / h ), h, { -6.0f, -4.0f } };
            check( bounded.isAnalyticCutCellBoundedCellAreasEnabled(),
                "bounded analytic cell areas must be enabled by default" );
            for( fluidField2* field : { &reference, &bounded } )
            {
                field->setCutCellProjectionEnabled( true );
                field->setCutCellCoupledProjectionEnabled( true );
                field->setDisjointAnalyticCutCellEnabled( true );
                field->addVelocity( { -1.2f, 0.14f }, { 1.4f, -0.27f }, 1.6f );
            }
            if( compareGeometryReuse )
                reference.setAnalyticCutCellCurrentGeometryReuseEnabled( false );
            else
                reference.setAnalyticCutCellBoundedCellAreasEnabled( false );
            std::vector<fluidObstacleForce2> oldForces( bodies.size() ), newForces( bodies.size() );
            for( int frame = 0; frame < 3; ++frame )
            {
                reference.step( dt, bodies, oldForces, 200 );
                bounded.step( dt, bodies, newForces, 200 );
                const auto& a = reference.getCutCellPressure();
                const auto& b = bounded.getCutCellPressure();
                check( reference.getCutCellTransition().usedDisjointAnalyticGeometry &&
                    bounded.getCutCellTransition().usedDisjointAnalyticGeometry,
                    "bounded cell area differential must exercise analytic geometry" );
                check( a.usedCoupledProjection && b.usedCoupledProjection &&
                    a.coupledStatus == fluidCoupledPressureStatus2::converged &&
                    b.coupledStatus == fluidCoupledPressureStatus2::converged &&
                    a.coupledIterations == b.coupledIterations &&
                    std::abs( a.coupledRelativeResidual - b.coupledRelativeResidual ) < 1.0e-6f &&
                    std::abs( a.predictedContinuityRms - b.predictedContinuityRms ) < 1.0e-4f,
                    "bounded analytic cell areas must preserve coupled pressure convergence" );
                check( std::abs( reference.getCutCellTransition().fluidVolume -
                    bounded.getCutCellTransition().fluidVolume ) < 1.0e-5f,
                    "bounded analytic cell areas must preserve fluid volume" );
                for( int y = 0; y < bounded.getHeight(); ++y )
                    for( int x = 0; x < bounded.getWidth(); ++x )
                        check( reference.getCellFluidFraction( x, y ) ==
                            bounded.getCellFluidFraction( x, y ),
                            "bounded analytic cell areas must preserve every cell fraction" );
                if( compareGeometryReuse )
                {
                    for( int y = 0; y < reference.getHeight(); ++y )
                        for( int x = 1; x < reference.getWidth(); ++x )
                            check( reference.getUFaceOpenFraction( x, y ) ==
                                    bounded.getUFaceOpenFraction( x, y ) &&
                                reference.getUFaceSolidVelocity( x, y ) ==
                                    bounded.getUFaceSolidVelocity( x, y ),
                                "same-step analytic reuse must preserve each U-face aperture and solid speed" );
                    for( int y = 1; y < reference.getHeight(); ++y )
                        for( int x = 0; x < reference.getWidth(); ++x )
                            check( reference.getVFaceOpenFraction( x, y ) ==
                                    bounded.getVFaceOpenFraction( x, y ) &&
                                reference.getVFaceSolidVelocity( x, y ) ==
                                    bounded.getVFaceSolidVelocity( x, y ),
                                "same-step analytic reuse must preserve each V-face aperture and solid speed" );
                }
                for( std::size_t i = 0; i < bodies.size(); ++i )
                {
                    const float scale = 1.0f + Length( oldForces[i].force );
                    check( Length( oldForces[i].force - newForces[i].force ) < 1.0e-4f * scale &&
                        std::abs( oldForces[i].torque - newForces[i].torque ) < 1.0e-4f * scale,
                        "bounded analytic cell areas must preserve pressure force and torque" );
                }
                const float scale = 1.0f + Length( reference.getFluidMomentum() );
                check( Length( reference.getFluidMomentum() - bounded.getFluidMomentum() ) <
                    1.0e-4f * scale, "bounded analytic cell areas must preserve fluid momentum" );
                for( std::size_t i = 0; i < 2; ++i )
                {
                    bodies[i].transform.position += dt * bodies[i].linearVelocity;
                    bodies[i].center = bodies[i].transform.position;
                }
            }
            bounded.clear();
            reference.clear();
            check( compareGeometryReuse
                    ? bounded.isAnalyticCutCellCurrentGeometryReuseEnabled() &&
                        !reference.isAnalyticCutCellCurrentGeometryReuseEnabled()
                    : bounded.isAnalyticCutCellBoundedCellAreasEnabled() &&
                        !reference.isAnalyticCutCellBoundedCellAreasEnabled(),
                "clear must preserve the selected analytic geometry option" );
        }
}

void checkAnalyticCutCellPressureImpulseBudget()
{
    constexpr vec2 origin{ -6.0f, -4.0f };
    constexpr std::array<float, 2> GRID_SIZES{ 0.5f, 0.25f };
    constexpr std::array<float, 2> TIME_STEPS{ 1.0f / 60.0f, 1.0f / 120.0f };
    int unavailableCases = 0;
    int iterationLimitCases = 0;
    std::array<int, 3> convergedModes{};

    for( float h : GRID_SIZES )
        for( float dt : TIME_STEPS )
            for( int geometryMode = 0; geometryMode < 3; ++geometryMode )
            {
                // 0: partial two-circle overlap, 1: separate, 2: strict containment.
                std::array<fluidShapeObstacle2, 2> bodies{{
                    { circle2{ {}, geometryMode == 2 ? 0.78f : 0.58f },
                        { { -0.21f, -0.18f }, {} }, { -0.21f, -0.18f },
                        { 0.95f, -0.30f }, 0.70f, 0.35f, 1.10f },
                    { circle2{ {}, geometryMode == 2 ? 0.18f : 0.58f },
                        { { geometryMode == 1 ? 1.50f : 0.26f, 0.19f }, {} },
                        { geometryMode == 1 ? 1.50f : 0.26f, 0.19f },
                        { -0.65f, 0.75f }, -0.55f, 0.80f, 0.70f }
                }};
                if( geometryMode == 2 )
                {
                    bodies[1].transform.position = { -0.12f, -0.12f };
                    bodies[1].center = bodies[1].transform.position;
                }
                fluidField2 field{ static_cast<int>( 12.0f / h ),
                    static_cast<int>( 8.0f / h ), h, origin };
                field.setCutCellProjectionEnabled( true );
                field.setCutCellCoupledProjectionEnabled( true );
                field.setDisjointAnalyticCutCellEnabled( true );
                field.addVelocity( { -1.2f, 0.14f }, { 1.4f, -0.27f }, 1.6f );
                std::array<fluidObstacleForce2, 2> reactions{};
                field.step( dt, bodies, reactions, 200 );

                const auto& projection = field.getCutCellPressure();
                const auto& budget = projection.impulse;
                if( projection.coupledSolveFailed &&
                    projection.coupledStatus == fluidCoupledPressureStatus2::iterationLimit &&
                    field.getCutCellTransition().usedDisjointAnalyticGeometry &&
                    !projection.hasAnalyticImpulseBudget &&
                    LengthSquared( reactions[0].force ) == 0.0f &&
                    LengthSquared( reactions[1].force ) == 0.0f &&
                    reactions[0].torque == 0.0f && reactions[1].torque == 0.0f )
                {
                    // Keep a diagnostic if the new preconditioner still hits the cap.
                    std::fprintf( stderr,
                        "IC0 iteration limit: mode=%d h=%.3f dt=%.6f rel=%.6g iterations=%d\n",
                        geometryMode, h, dt, projection.coupledRelativeResidual,
                        projection.coupledIterations );
                    ++iterationLimitCases;
                    continue;
                }
                if( !projection.usedCoupledProjection || projection.coupledSolveFailed ||
                    projection.coupledStatus != fluidCoupledPressureStatus2::converged ||
                    !projection.hasAnalyticImpulseBudget ||
                    !field.getCutCellTransition().usedDisjointAnalyticGeometry )
                {
                    std::fprintf( stderr,
                        "analytic impulse unavailable: mode=%d h=%.3f dt=%.6f "
                        "coupled=%d failed=%d status=%d relative=%.6g analytic=%d impulse=%d\n",
                        geometryMode, h, dt, projection.usedCoupledProjection,
                        projection.coupledSolveFailed, static_cast<int>( projection.coupledStatus ),
                        projection.coupledRelativeResidual,
                        field.getCutCellTransition().usedDisjointAnalyticGeometry,
                        projection.hasAnalyticImpulseBudget );
                    ++unavailableCases;
                    continue;
                }
                ++convergedModes[geometryMode];
                check( projection.usedIncompleteCholesky &&
                    projection.coupledIterations > 0 && projection.coupledIterations < 200 &&
                    projection.coupledRelativeResidual < 1.01e-5f,
                    "analytic Cut-cell PCG must converge within 200 iterations using IC(0)" );

                const vec2 expected = dt * ( reactions[0].force + reactions[1].force );
                const float scale = 1.0f + Length( budget.fluid ) +
                    Length( budget.solid ) + Length( budget.wall );
                if( Length( budget.solid - expected ) > 2.0e-5f * scale ||
                    Length( budget.residual() ) > 2.0e-4f * scale )
                {
                    std::fprintf( stderr,
                        "analytic impulse closure mismatch: mode=%d h=%.2f dt=%.6f "
                        "fluid=(%.6g,%.6g) solid=(%.6g,%.6g) wall=(%.6g,%.6g) "
                        "residual=(%.6g,%.6g)\n",
                        geometryMode, h, dt, budget.fluid.x, budget.fluid.y,
                        budget.solid.x, budget.solid.y, budget.wall.x, budget.wall.y,
                        budget.residual().x, budget.residual().y );
                    check( false,
                        "analytic cut-cell pressure fluid/rigid/wall impulse must close at MAC precision" );
                }
                check( IsFinite( budget.fluid ) && IsFinite( budget.solid ) &&
                    IsFinite( budget.wall ) && IsFinite( budget.residual() ),
                    "coupled cut-cell impulse bookkeeping must remain finite" );
                check( LengthSquared( field.getPressureMomentum().residual() ) == 0.0f,
                    "analytic cut-cell ledger must not write to binary MAC pressure momentum" );

                if( geometryMode == 2 )
                    check( LengthSquared( reactions[1].force ) < 1.0e-10f &&
                        std::abs( reactions[1].torque ) < 1.0e-5f,
                        "fully hidden inner circle must not receive coupled pressure impulse" );

                field.clear();
                check( !field.getCutCellPressure().hasAnalyticImpulseBudget &&
                    LengthSquared( field.getCutCellPressure().impulse.residual() ) == 0.0f,
                    "clear must reset analytic cut-cell pressure impulse diagnostics" );
            }

    check( unavailableCases == 0 && iterationLimitCases == 0 &&
        convergedModes[0] == 4 && convergedModes[1] == 4 && convergedModes[2] == 4,
        "IC(0) must converge for all 2-circle modes at both grid sizes and time steps" );

    // Failed PCG and sampled geometry must not claim a valid analytic pressure audit.
    const std::array<fluidShapeObstacle2, 2> touching{{
        { circle2{ {}, 0.58f }, { { -0.21f, -0.18f }, {} },
            { -0.21f, -0.18f }, { 0.9f, -0.3f }, 0.7f, 0.35f, 1.10f },
        { circle2{ {}, 0.58f }, { { 0.26f, 0.19f }, {} },
            { 0.26f, 0.19f }, { -0.6f, 0.7f }, -0.5f, 0.8f, 0.7f }
    }};
    for( int scenario = 0; scenario < 3; ++scenario )
    {
        fluidField2 field{ 24, 16, 0.5f, origin };
        field.setCutCellProjectionEnabled( true );
        field.setCutCellCoupledProjectionEnabled( true );
        field.setDisjointAnalyticCutCellEnabled( scenario != 1 );
        field.addVelocity( { -1.2f, 0.14f }, { 1.4f, -0.27f }, 1.6f );
        std::array<fluidObstacleForce2, 2> forces{};
        std::array<fluidShapeObstacle2, 2> inputs = touching;
        if( scenario == 2 )
        {
            inputs[0].inverseMass = 0.0f;
            inputs[0].inverseInertia = 0.0f;
            inputs[1].inverseMass = 0.0f;
            inputs[1].inverseInertia = 0.0f;
        }
        field.step( 1.0f / 60.0f, inputs, forces, scenario == 0 ? 1 : 200 );
        const auto& pressure = field.getCutCellPressure();
        check( scenario == 1 ? !pressure.usedIncompleteCholesky :
            ( scenario == 2 ? !pressure.usedIncompleteCholesky : pressure.usedIncompleteCholesky ),
            "IC(0) must be limited to an attempted analytic coupled pressure solve" );
        check( !pressure.hasAnalyticImpulseBudget &&
            LengthSquared( pressure.impulse.fluid ) == 0.0f &&
            LengthSquared( pressure.impulse.solid ) == 0.0f &&
            LengthSquared( pressure.impulse.wall ) == 0.0f,
            "failure, sampled, and uncoupled pressure paths must not populate analytic impulse ledger" );
        if( scenario == 0 )
            check( pressure.coupledSolveFailed &&
                LengthSquared( forces[0].force ) + LengthSquared( forces[1].force ) == 0.0f,
                "failed Schur pressure must produce no analytic impulse or rigid feedback" );
    }
}

void checkAnalyticPartialCircleUnion()
{
    constexpr float dt = 1.0f / 60.0f;
    constexpr float h = 0.5f;
    constexpr float radius = 0.60f;
    constexpr float area = 24.0f * 16.0f * h * h;

    std::array<fluidShapeObstacle2, 2> circles{{
        { circle2{ {}, radius }, { { -0.16f, -0.20f }, {} },
            { -0.16f, -0.20f }, { 1.0f, -0.3f }, 0.0f },
        { circle2{ {}, radius }, { { 0.23f, 0.25f }, {} },
            { 0.23f, 0.25f }, { -0.5f, 0.7f }, 0.0f }
    }};
    const float distance = Length( circles[1].center - circles[0].center );
    const float lensArea = 2.0f * radius * radius *
        std::acos( distance / ( 2.0f * radius ) ) -
        0.5f * distance * std::sqrt( 4.0f * radius * radius - distance * distance );
    const float expectedFluidArea = area -
        ( 2.0f * std::numbers::pi_v<float> * radius * radius - lensArea );

    fluidField2 field{ 24, 16, h, { -6.0f, -4.0f } };
    field.setCutCellProjectionEnabled( true );
    field.setDisjointAnalyticCutCellEnabled( true );
    std::array<fluidObstacleForce2, 2> reactions{};
    field.step( dt, circles, reactions, 160 );
    const auto& first = field.getCutCellTransition();
    check( first.usedDisjointAnalyticGeometry && first.usedPartialCircleAnalyticUnion &&
        first.disjointAnalyticShapes == 2 && first.containedAnalyticShapes == 0 &&
        std::abs( first.fluidVolume - expectedFluidArea ) < 1.0e-4f &&
        !field.getCutCellPressure().hasVolumeHistory,
        "partially overlapping circles must use analytic lens area, not double-count solid volume" );

    // x=0, 0<=y<=0.5: circle 0 covers [0,aTop] and circle 1 covers [0,0.5].
    // The overlap segment belongs only to the first owner.
    const float aTop = circles[0].center.y +
        std::sqrt( radius * radius - circles[0].center.x * circles[0].center.x );
    const float expectedU = ( aTop * circles[0].linearVelocity.x +
        ( h - aTop ) * circles[1].linearVelocity.x ) / h;
    const float aRight = circles[0].center.x +
        std::sqrt( radius * radius - circles[0].center.y * circles[0].center.y );
    const float expectedV = ( aRight * circles[0].linearVelocity.y +
        ( h - aRight ) * circles[1].linearVelocity.y ) / h;
    check( field.getUFaceOpenFraction( 12, 8 ) < 2.0e-5f &&
        field.getVFaceOpenFraction( 12, 8 ) < 2.0e-5f &&
        std::abs( field.getUFaceSolidVelocity( 12, 8 ) - expectedU ) < 2.0e-5f &&
        std::abs( field.getVFaceSolidVelocity( 12, 8 ) - expectedV ) < 2.0e-5f,
        "shared overlapping U/V MAC face must assign each blocked interval to only one body" );

    field.setCutCellSampleCount( 8 );
    field.step( dt, circles, reactions, 160 );
    check( field.getCutCellTransition().usedPartialCircleAnalyticUnion &&
        std::abs( field.getCutCellTransition().fluidVolume - expectedFluidArea ) < 1.0e-4f &&
        !field.getCutCellPressure().hasVolumeHistory,
        "partial two-circle union must be independent of 4x4/8x8 spatial samples" );

    // Changing the obstacle array order changes owner identity, so invalidate
    // temporal history as required for any obstacle composition change.
    field.invalidateObstacleHistory();
    std::swap( circles[0], circles[1] );
    field.step( dt, circles, reactions, 160 );
    check( field.getCutCellTransition().usedPartialCircleAnalyticUnion &&
        std::abs( field.getCutCellTransition().fluidVolume - expectedFluidArea ) < 1.0e-4f,
        "reordered overlapping circles must preserve exact union area" );

    field.setCutCellCoupledProjectionEnabled( true );
    circles[0].inverseMass = 0.6f;
    circles[0].inverseInertia = 1.1f;
    circles[1].inverseMass = 0.4f;
    circles[1].inverseInertia = 0.9f;
    field.addVelocity( { -1.0f, 0.1f }, { 1.2f, -0.3f }, 1.4f );
    field.step( dt, circles, reactions, 200 );
    const auto& pressure = field.getCutCellPressure();
    check( pressure.usedCoupledProjection && !pressure.coupledSolveFailed &&
        pressure.coupledStatus == fluidCoupledPressureStatus2::converged &&
        std::isfinite( pressure.predictedContinuityRms ) &&
        std::abs( pressure.predictedContinuityRms - pressure.pressureResidual ) <
            2.0e-3f + 0.1f * pressure.pressureResidual,
        "two partially overlapping circles must preserve multi-owner Schur B/B-transpose consistency" );
    check( IsFinite( reactions[0].force ) && IsFinite( reactions[1].force ) &&
        std::isfinite( reactions[0].torque ) && std::isfinite( reactions[1].torque ),
        "partially overlapping circle pressure feedback must remain finite for both owners" );

    circles[0].transform.position += vec2{ 0.02f, 0.01f };
    circles[0].center = circles[0].transform.position;
    field.step( dt, circles, reactions, 200 );
    check( field.getCutCellTransition().usedPartialCircleAnalyticUnion &&
        field.getCutCellPressure().hasVolumeHistory &&
        field.getSweptFaceFlux().usedSweptFlux &&
        std::isfinite( field.getSweptFaceFlux().geometryMismatch ),
        "moving partially overlapping circles must use union-aware swept face flux" );

    circles[0].transform.position = { 2.0f, 1.4f };
    circles[0].center = circles[0].transform.position;
    field.step( dt, circles, reactions, 200 );
    check( field.getCutCellTransition().usedDisjointAnalyticGeometry &&
        !field.getCutCellTransition().usedPartialCircleAnalyticUnion &&
        !field.getCutCellPressure().hasVolumeHistory &&
        !field.getSweptFaceFlux().usedSweptFlux,
        "partial overlap to disjoint transition must invalidate swept volume history" );

    circles[0].transform.position = { 1.039999f, -0.20f };
    circles[0].center = circles[0].transform.position;
    field.step( dt, circles, reactions, 200 );
    check( !field.getCutCellTransition().usedDisjointAnalyticGeometry &&
        !field.getCutCellTransition().usedPartialCircleAnalyticUnion &&
        !field.getCutCellPressure().hasVolumeHistory,
        "near-tangent circle intersection must use conservative sampled union" );
    field.clear();
    check( field.isDisjointAnalyticCutCellEnabled() &&
        !field.getCutCellTransition().usedPartialCircleAnalyticUnion,
        "clear must preserve union mode setting while resetting overlap diagnostics" );
}

void checkAnalyticContainedCircleUnion()
{
    constexpr float dt = 1.0f / 60.0f;
    constexpr float h = 0.5f;
    constexpr float outerRadius = 0.78f;
    constexpr float innerRadius = 0.18f;
    constexpr float domainArea = 24.0f * 16.0f * h * h;
    constexpr float expectedArea = domainArea - std::numbers::pi_v<float> * outerRadius * outerRadius;

    fluidShapeObstacle2 outer{ circle2{ {}, outerRadius }, { {}, {} }, {},
        { 1.2f, -0.3f }, 0.0f };
    fluidShapeObstacle2 inner{ circle2{ {}, innerRadius }, { { 0.12f, 0.08f }, {} },
        { 0.12f, 0.08f }, { -5.0f, 4.0f }, 1.0f };
    std::array<fluidShapeObstacle2, 2> bodies{ outer, inner };
    std::array<fluidObstacleForce2, 2> feedback{};

    fluidField2 field{ 24, 16, h, { -6.0f, -4.0f } };
    field.setCutCellProjectionEnabled( true );
    field.setDisjointAnalyticCutCellEnabled( true );
    field.step( dt, bodies, feedback, 180 );
    check( field.getCutCellTransition().usedDisjointAnalyticGeometry &&
        field.getCutCellTransition().disjointAnalyticShapes == 2 &&
        field.getCutCellTransition().containedAnalyticShapes == 1 &&
        std::abs( field.getCutCellTransition().fluidVolume - expectedArea ) < 1.0e-4f &&
        !field.getCutCellPressure().hasVolumeHistory,
        "fully enclosed circle must not add solid area to analytic union" );
    // Both circles cover part of these faces. Only the outer body's velocity counts.
    check( field.getUFaceOpenFraction( 12, 8 ) < 2.0e-5f &&
        field.getVFaceOpenFraction( 12, 8 ) < 2.0e-5f &&
        std::abs( field.getUFaceSolidVelocity( 12, 8 ) - outer.linearVelocity.x ) < 2.0e-5f &&
        std::abs( field.getVFaceSolidVelocity( 12, 8 ) - outer.linearVelocity.y ) < 2.0e-5f,
        "hidden circle must not overwrite either shared U/V face solid velocity" );

    field.setCutCellSampleCount( 8 );
    field.step( dt, bodies, feedback, 180 );
    check( field.getCutCellTransition().containedAnalyticShapes == 1 &&
        std::abs( field.getCutCellTransition().fluidVolume - expectedArea ) < 1.0e-4f &&
        !field.getCutCellPressure().hasVolumeHistory,
        "contained analytic union must be independent of 4x4/8x8 spatial samples" );

    // The first input shape may be enclosed; pressure ownership is the outer shape's.
    std::swap( bodies[0], bodies[1] );
    field.step( dt, bodies, feedback, 180 );
    check( field.getCutCellTransition().usedDisjointAnalyticGeometry &&
        field.getCutCellTransition().containedAnalyticShapes == 1 &&
        std::abs( field.getCutCellTransition().fluidVolume - expectedArea ) < 1.0e-4f &&
        !field.getCutCellPressure().hasVolumeHistory &&
        std::abs( field.getUFaceSolidVelocity( 12, 8 ) - outer.linearVelocity.x ) < 2.0e-5f,
        "input order inversion must keep exposed circle owner and invalidate containment history" );

    // Compare coupled pressure with the exact single-circle oracle.
    bodies[0].inverseMass = 2.0f;
    bodies[0].inverseInertia = 3.0f;
    bodies[1].inverseMass = 0.5f;
    bodies[1].inverseInertia = 1.2f;
    fluidField2 nested{ 24, 16, h, { -6.0f, -4.0f } };
    fluidField2 reference{ 24, 16, h, { -6.0f, -4.0f } };
    nested.setCutCellProjectionEnabled( true );
    nested.setDisjointAnalyticCutCellEnabled( true );
    nested.setCutCellCoupledProjectionEnabled( true );
    reference.setCutCellProjectionEnabled( true );
    reference.setAnalyticCircleCutCellEnabled( true );
    reference.setCutCellCoupledProjectionEnabled( true );
    std::array<fluidObstacleForce2, 2> nestedReactions{};
    std::array<fluidObstacleForce2, 1> referenceReaction{};
    nested.addVelocity( { -1.2f, 0.2f }, { 1.3f, -0.4f }, 1.4f );
    reference.addVelocity( { -1.2f, 0.2f }, { 1.3f, -0.4f }, 1.4f );
    nested.step( dt, bodies, nestedReactions, 200 );
    reference.step( dt, std::span<const fluidShapeObstacle2>( &bodies[1], 1 ),
        referenceReaction, 200 );
    const auto& coupled = nested.getCutCellPressure();
    check( coupled.usedCoupledProjection && !coupled.coupledSolveFailed &&
        coupled.coupledStatus == fluidCoupledPressureStatus2::converged &&
        std::isfinite( coupled.predictedContinuityRms ) &&
        std::abs( coupled.predictedContinuityRms - coupled.pressureResidual ) <
            2.0e-3f + 0.1f * coupled.pressureResidual,
        "contained circle pressure must preserve coupled Schur adjoint consistency" );
    check( Length( nestedReactions[0].force ) < 1.0e-5f &&
        std::abs( nestedReactions[0].torque ) < 1.0e-5f &&
        Length( nestedReactions[1].force - referenceReaction[0].force ) < 5.0e-3f &&
        std::abs( nestedReactions[1].torque - referenceReaction[0].torque ) < 5.0e-3f,
        "fully occluded body must receive zero pressure traction and outer body must match single-circle coupling" );

    // A moving hidden circle must not displace any extra fluid or contribute swept flux.
    fluidField2 transitions{ 24, 16, h, { -6.0f, -4.0f } };
    transitions.setCutCellProjectionEnabled( true );
    transitions.setDisjointAnalyticCutCellEnabled( true );
    bodies[0] = inner;
    bodies[1] = outer;
    bodies[0].linearVelocity = {};
    bodies[1].linearVelocity = {};
    transitions.step( dt, bodies, feedback, 160 );
    bodies[0].transform.position += vec2{ 0.03f, -0.01f };
    bodies[0].center = bodies[0].transform.position;
    transitions.step( dt, bodies, feedback, 160 );
    check( transitions.getCutCellPressure().hasVolumeHistory &&
        transitions.getCutCellTransition().containedAnalyticShapes == 1 &&
        std::abs( transitions.getCutCellTransition().volumeDelta() ) < 1.0e-5f &&
        transitions.getSweptFaceFlux().usedSweptFlux &&
        transitions.getSweptFaceFlux().totalAbsoluteFlux < 1.0e-6f,
        "moving a fully hidden circle must generate no fictitious displaced fluid" );

    // Partial intersection switches to the exact lens path and clears the
    // previous full-containment history.
    bodies[0].transform.position = { 0.65f, 0.0f };
    bodies[0].center = bodies[0].transform.position;
    transitions.step( dt, bodies, feedback, 160 );
    check( transitions.getCutCellTransition().usedDisjointAnalyticGeometry &&
        transitions.getCutCellTransition().usedPartialCircleAnalyticUnion &&
        transitions.getCutCellTransition().containedAnalyticShapes == 0 &&
        !transitions.getCutCellPressure().hasVolumeHistory &&
        !transitions.getSweptFaceFlux().usedSweptFlux,
        "contained-to-partial circle union transition must invalidate history" );

    bodies[0].transform.position = { 0.12f, 0.08f };
    bodies[0].center = bodies[0].transform.position;
    transitions.step( dt, bodies, feedback, 160 );
    check( transitions.getCutCellTransition().usedDisjointAnalyticGeometry &&
        transitions.getCutCellTransition().containedAnalyticShapes == 1 &&
        !transitions.getCutCellPressure().hasVolumeHistory &&
        !transitions.getSweptFaceFlux().usedSweptFlux,
        "returning to full containment must restart volume and swept history" );

    // Tangency is deliberately not counted as strict containment.
    bodies[0].transform.position = { outerRadius - innerRadius, 0.0f };
    bodies[0].center = bodies[0].transform.position;
    transitions.step( dt, bodies, feedback, 160 );
    check( !transitions.getCutCellTransition().usedDisjointAnalyticGeometry,
        "tangent containment must conservatively fall back to sampled union" );

    transitions.clear();
    check( transitions.isDisjointAnalyticCutCellEnabled() &&
        transitions.getCutCellTransition().containedAnalyticShapes == 0,
        "clear must retain enabled option while dropping containment diagnostics" );
}

void checkDisjointAnalyticOverlappingAabbs()
{
    constexpr float dt = 1.0f / 60.0f;
    constexpr float h = 0.5f;
    constexpr float domainArea = 24.0f * 16.0f * h * h;
    constexpr float circleRadius = 0.20f;
    fluidField2 field{ 24, 16, h, { -6.0f, -4.0f } };
    field.setCutCellProjectionEnabled( true );
    field.setDisjointAnalyticCutCellEnabled( true );
    std::array<fluidShapeObstacle2, 2> circles{{
        { circle2{ {}, circleRadius }, { { 0.08f, 0.08f }, {} },
            { 0.08f, 0.08f }, {}, 0.0f },
        { circle2{ {}, circleRadius }, { { 0.42f, 0.42f }, {} },
            { 0.42f, 0.42f }, {}, 0.0f }
    }};
    std::array<fluidObstacleForce2, 2> reactions{};
    const float expectedArea = domainArea -
        2.0f * std::numbers::pi_v<float> * circleRadius * circleRadius;
    check( Overlaps( ComputeShapeAABB( circles[0].geometry, circles[0].transform ),
        ComputeShapeAABB( circles[1].geometry, circles[1].transform ) ),
        "diagonally separated circles must have overlapping AABBs in this fixture" );

    field.step( dt, circles, reactions, 160 );
    check( field.getCutCellTransition().usedDisjointAnalyticGeometry &&
        field.getCutCellTransition().disjointAnalyticShapes == 2 &&
        std::abs( field.getCutCellTransition().fluidVolume - expectedArea ) < 1.0e-4f,
        "GJK-separated circles with overlapping AABBs must retain exact analytic union" );

    field.setCutCellSampleCount( 8 );
    field.step( dt, circles, reactions, 160 );
    check( field.getCutCellTransition().usedDisjointAnalyticGeometry &&
        !field.getCutCellPressure().hasVolumeHistory &&
        std::abs( field.getCutCellTransition().fluidVolume - expectedArea ) < 1.0e-4f,
        "overlapping-AABB disjoint analytic area must be independent of spatial samples" );

    circles[1].transform.position += vec2{ 0.03f, 0.03f };
    circles[1].center = circles[1].transform.position;
    field.step( dt, circles, reactions, 160 );
    check( field.getCutCellTransition().usedDisjointAnalyticGeometry &&
        field.getCutCellPressure().hasVolumeHistory &&
        field.getSweptFaceFlux().usedSweptFlux &&
        std::isfinite( field.getSweptFaceFlux().geometryMismatch ),
        "moving GJK-separated shapes must preserve swept geometry history" );

    // Both AABBs and the actual circles now intersect. The exact clipped lens
    // union is supported, but the disjoint->partial mode switch resets history.
    circles[1].transform.position = { 0.20f, 0.20f };
    circles[1].center = circles[1].transform.position;
    field.step( dt, circles, reactions, 160 );
    const float intersectDistance = Length( circles[1].center - circles[0].center );
    const float lens = 2.0f * circleRadius * circleRadius *
        std::acos( intersectDistance / ( 2.0f * circleRadius ) ) -
        0.5f * intersectDistance *
            std::sqrt( 4.0f * circleRadius * circleRadius - intersectDistance * intersectDistance );
    check( field.getCutCellTransition().usedDisjointAnalyticGeometry &&
        field.getCutCellTransition().usedPartialCircleAnalyticUnion &&
        !field.getCutCellPressure().hasVolumeHistory &&
        !field.getSweptFaceFlux().usedSweptFlux &&
        std::abs( field.getCutCellTransition().fluidVolume -
            ( expectedArea + lens ) ) < 1.0e-4f,
        "actual partial circle overlap must use exact lens union and invalidate geometry history" );

    circles[1].transform.position = { 0.42f, 0.42f };
    circles[1].center = circles[1].transform.position;
    field.step( dt, circles, reactions, 160 );
    check( field.getCutCellTransition().usedDisjointAnalyticGeometry &&
        !field.getCutCellPressure().hasVolumeHistory &&
        !field.getSweptFaceFlux().usedSweptFlux,
        "returning to GJK separation must not invent swept volume" );

    // A tangent pair is numerically ambiguous, so keep the sampled union.
    circles[1].transform.position = { 0.479999f, 0.08f };
    circles[1].center = circles[1].transform.position;
    field.step( dt, circles, reactions, 160 );
    check( !field.getCutCellTransition().usedDisjointAnalyticGeometry,
        "touching shapes must conservatively fall back to sampled union" );

    // Mixed convex proxies share the same exact separation test.
    // The rotated capsule AABB overlaps the small circle's AABB, not its shape.
    polygon2 rounded = MakeBox( { 0.30f, 0.20f } );
    rounded.radius = 0.12f;
    std::array<fluidShapeObstacle2, 4> mixed{{
        { capsule2{ { -0.45f, 0.0f }, { 0.45f, 0.0f }, 0.08f },
            { {}, rot2::FromRadians( std::numbers::pi_v<float> * 0.25f ) },
            {}, {}, 0.0f },
        { circle2{ {}, 0.18f }, { { 0.45f, 0.0f }, {} },
            { 0.45f, 0.0f }, {}, 0.0f },
        { MakeBox( { 0.33f, 0.23f } ), { { 3.0f, 1.0f }, {} },
            { 3.0f, 1.0f }, {}, 0.0f },
        { rounded, { { -3.0f, -1.0f }, rot2::FromRadians( 0.29f ) },
            { -3.0f, -1.0f }, {}, 0.0f }
    }};
    std::array<fluidObstacleForce2, 4> mixedReactions{};
    const float capsuleArea = 2.0f * 0.08f * 0.90f +
        std::numbers::pi_v<float> * 0.08f * 0.08f;
    const float mixedArea = domainArea - ( capsuleArea +
        std::numbers::pi_v<float> * 0.18f * 0.18f +
        4.0f * 0.33f * 0.23f +
        4.0f * 0.30f * 0.20f + 4.0f * ( 0.30f + 0.20f ) * 0.12f +
        std::numbers::pi_v<float> * 0.12f * 0.12f );
    check( Overlaps( ComputeShapeAABB( mixed[0].geometry, mixed[0].transform ),
        ComputeShapeAABB( mixed[1].geometry, mixed[1].transform ) ),
        "rotated capsule and circle fixture must have overlapping AABBs" );

    fluidField2 mixedField{ 24, 16, h, { -6.0f, -4.0f } };
    mixedField.setCutCellProjectionEnabled( true );
    mixedField.setDisjointAnalyticCutCellEnabled( true );
    mixedField.step( dt, mixed, mixedReactions, 160 );
    check( mixedField.getCutCellTransition().usedDisjointAnalyticGeometry &&
        mixedField.getCutCellTransition().disjointAnalyticShapes == 4 &&
        std::abs( mixedField.getCutCellTransition().fluidVolume - mixedArea ) < 1.0e-4f,
        "GJK-separated mixed capsule/circle/polygon/rounded geometry must keep exact union" );
}

void checkAnalyticRoundedPolygonCutCell()
{
    constexpr float dt = 1.0f / 60.0f;
    constexpr float h = 0.5f;
    constexpr float radius = 0.35f;
    constexpr float halfExtent = 0.35f;
    constexpr vec2 center{ 0.0f, -0.15f };
    constexpr float coreArea = 4.0f * halfExtent * halfExtent;
    constexpr float perimeter = 8.0f * halfExtent;
    constexpr float expectedVolume = 24.0f * 16.0f * h * h -
        ( coreArea + perimeter * radius + std::numbers::pi_v<float> * radius * radius );

    fluidField2 field{ 24, 16, h, { -6.0f, -4.0f } };
    field.setCutCellProjectionEnabled( true );
    field.setAnalyticRoundedPolygonCutCellEnabled( true );
    polygon2 geometry = MakeBox( { halfExtent, halfExtent } );
    geometry.radius = radius;
    fluidShapeObstacle2 box{ geometry, { center, {} }, center, {}, 0.0f };
    std::array<fluidObstacleForce2, 1> forces{};
    field.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), forces, 160 );
    check( std::abs( field.getCutCellTransition().fluidVolume - expectedVolume ) < 1.0e-4f &&
        !field.getCutCellPressure().hasVolumeHistory &&
        !field.getSweptFaceFlux().usedSweptFlux,
        "analytic rounded Polygon must have area A + perimeter*r + pi*r^2 on first snapshot" );

    // 오른쪽 위 원호(vertex=(0.35,0.20))와 x=0.5, y=0.5 face의 정확한 교차.
    const float topRightX = center.x + halfExtent;
    const float topRightY = center.y + halfExtent;
    const float yUpper = topRightY +
        std::sqrt( radius * radius - ( 0.5f - topRightX ) * ( 0.5f - topRightX ) );
    const float xUpper = topRightX +
        std::sqrt( radius * radius - ( 0.5f - topRightY ) * ( 0.5f - topRightY ) );
    check( std::abs( field.getUFaceOpenFraction( 13, 9 ) -
        ( 1.0f - ( yUpper - 0.5f ) / h ) ) < 2.0e-5f,
        "rounded Polygon U face must clip the curved corner rather than its sharp core" );
    check( std::abs( field.getVFaceOpenFraction( 13, 9 ) -
        ( 1.0f - ( xUpper - 0.5f ) / h ) ) < 2.0e-5f,
        "rounded Polygon V face must clip the circular vertex sector" );

    field.setCutCellSampleCount( 8 );
    field.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), forces, 160 );
    check( std::abs( field.getCutCellTransition().fluidVolume - expectedVolume ) < 1.0e-4f &&
        !field.getCutCellPressure().hasVolumeHistory,
        "analytic rounded Polygon volume must be independent of 4x4/8x8 samples" );

    for( float angle : { 0.26f, 0.64f, 1.13f, -0.79f } )
    {
        box.transform.rotation = rot2::FromRadians( angle );
        field.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), forces, 180 );
        check( std::abs( field.getCutCellTransition().fluidVolume - expectedVolume ) < 1.0e-4f &&
            field.getSweptFaceFlux().usedSweptFlux &&
            std::isfinite( field.getSweptFaceFlux().geometryMismatch ),
            "arbitrarily rotated rounded Polygon must keep exact area and finite swept flux" );
    }

    field.setCutCellCoupledProjectionEnabled( true );
    box.linearVelocity = { 0.8f, -0.3f };
    box.angularVelocity = 0.8f;
    box.inverseMass = 0.5f;
    box.inverseInertia = 1.2f;
    field.addVelocity( { -1.1f, 0.25f }, { 1.3f, -0.4f }, 1.4f );
    field.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), forces, 200 );
    const auto& pressure = field.getCutCellPressure();
    check( pressure.usedCoupledProjection && !pressure.coupledSolveFailed &&
        pressure.coupledStatus == fluidCoupledPressureStatus2::converged &&
        pressure.synchronizedPartialFaces > 0 &&
        std::isfinite( pressure.predictedContinuityRms ) &&
        std::abs( pressure.predictedContinuityRms - pressure.pressureResidual ) <
            2.0e-3f + 0.1f * pressure.pressureResidual,
        "rounded Polygon Schur traction and predicted blocked face flux must stay adjoint" );

    box.transform.position += vec2{ 0.16f, 0.08f };
    box.center = box.transform.position;
    field.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), forces, 200 );
    check( field.getSweptFaceFlux().usedSweptFlux &&
        field.getSweptFaceFlux().totalAbsoluteFlux > 0.0f &&
        std::isfinite( field.getSweptFaceFlux().geometryMismatch ) &&
        std::abs( field.getCutCellTransition().fluidVolume - expectedVolume ) < 1.0e-4f,
        "moving rounded Polygon must integrate swept face flux with invariant area" );

    field.setAnalyticRoundedPolygonCutCellEnabled( false );
    field.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), forces, 200 );
    check( !field.getCutCellPressure().hasVolumeHistory &&
        !field.getSweptFaceFlux().usedSweptFlux,
        "disabling rounded Polygon analysis must reset swept geometry history" );

    fluidField2 fallback{ 24, 16, h, { -6.0f, -4.0f } };
    fluidField2 sampled{ 24, 16, h, { -6.0f, -4.0f } };
    fallback.setCutCellProjectionEnabled( true );
    sampled.setCutCellProjectionEnabled( true );
    fallback.setAnalyticRoundedPolygonCutCellEnabled( true );
    std::array<fluidShapeObstacle2, 2> obstacles{ box, box };
    obstacles[1].transform.position += vec2{ 1.2f, 0.7f };
    obstacles[1].center = obstacles[1].transform.position;
    std::array<fluidObstacleForce2, 2> reactions{};
    fallback.step( dt, obstacles, reactions, 120 );
    sampled.step( dt, obstacles, reactions, 120 );
    check( std::abs( fallback.getCutCellTransition().fluidVolume -
        sampled.getCutCellTransition().fluidVolume ) < 1.0e-5f,
        "multiple rounded polygons must use existing sampled union geometry" );

    // 비축정렬/비직각의 CCW 삼각형에서도 core + perimeter*r + pi*r²를 만족.
    constexpr std::array<vec2, 3> TRIANGLE{
        vec2{ -0.50f, -0.35f }, vec2{ 0.55f, -0.20f }, vec2{ 0.08f, 0.67f }
    };
    constexpr float triangleRadius = 0.17f;
    const float triangleArea = 0.5f * Cross( TRIANGLE[1] - TRIANGLE[0],
        TRIANGLE[2] - TRIANGLE[0] );
    const float trianglePerimeter =
        Length( TRIANGLE[1] - TRIANGLE[0] ) +
        Length( TRIANGLE[2] - TRIANGLE[1] ) +
        Length( TRIANGLE[0] - TRIANGLE[2] );
    const float triangleFluidVolume = 24.0f * 16.0f * h * h -
        ( triangleArea + trianglePerimeter * triangleRadius +
            std::numbers::pi_v<float> * triangleRadius * triangleRadius );

    polygon2 triangle = MakePolygon( TRIANGLE );
    triangle.radius = triangleRadius;
    fluidShapeObstacle2 triangularBody{
        triangle, { { 0.16f, -0.08f }, {} }, { 0.16f, -0.08f }, {}, 0.0f
    };
    fluidField2 triangleField{ 24, 16, h, { -6.0f, -4.0f } };
    triangleField.setCutCellProjectionEnabled( true );
    triangleField.setAnalyticRoundedPolygonCutCellEnabled( true );
    for( float angle : { 0.0f, 0.37f, 1.02f, -0.61f } )
    {
        triangularBody.transform.rotation = rot2::FromRadians( angle );
        triangleField.step( dt,
            std::span<const fluidShapeObstacle2>( &triangularBody, 1 ), forces, 180 );
        check( std::abs( triangleField.getCutCellTransition().fluidVolume -
            triangleFluidVolume ) < 1.0e-4f,
            "nonrectangular rounded convex triangle must preserve analytic offset area under rotation" );
    }

    field.clear();
    check( !field.getSweptFaceFlux().usedSweptFlux &&
        !field.isAnalyticRoundedPolygonCutCellEnabled(),
        "clear must retain explicit rounded geometry off setting" );
}

void checkAnalyticSingleCapsuleCutCell()
{
    constexpr float dt = 1.0f / 60.0f;
    constexpr float h = 0.5f;
    constexpr float radius = 0.30f;
    constexpr float axisLength = 1.40f;
    constexpr float expectedVolume = 24.0f * 16.0f * h * h -
        ( 2.0f * radius * axisLength + std::numbers::pi_v<float> * radius * radius );

    fluidField2 field{ 24, 16, h, { -6.0f, -4.0f } };
    field.setCutCellProjectionEnabled( true );
    field.setAnalyticCapsuleCutCellEnabled( true );
    fluidShapeObstacle2 capsule{
        capsule2{ { -0.70f, 0.0f }, { 0.70f, 0.0f }, radius },
        { { 0.0f, -0.17f }, {} }, { 0.0f, -0.17f }, {}, 0.0f
    };
    std::array<fluidObstacleForce2, 1> feedback{};
    field.step( dt, std::span<const fluidShapeObstacle2>( &capsule, 1 ), feedback, 180 );
    check( std::abs( field.getCutCellTransition().fluidVolume - expectedVolume ) < 1.0e-4f &&
        !field.getCutCellPressure().hasVolumeHistory &&
        !field.getSweptFaceFlux().usedSweptFlux,
        "analytic Capsule must recover central strip and semicircle areas on first snapshot" );

    // x=0의 U-face (y=0..0.5): 가운데 strip의 blocked 길이는 0.13 m.
    check( std::abs( field.getUFaceOpenFraction( 12, 8 ) - 0.74f ) < 2.0e-5f,
        "Capsule U aperture must clip its straight core without spatial samples" );
    // y=0의 V-face (x=0.5..1): 중앙 strip + 오른쪽 둥근 cap 한 구간.
    const float capExtent = std::sqrt( radius * radius - 0.17f * 0.17f );
    const float expectedVOpen = 1.0f - ( 0.70f + capExtent - 0.50f ) / h;
    check( std::abs( field.getVFaceOpenFraction( 13, 8 ) - expectedVOpen ) < 2.0e-5f,
        "Capsule V aperture must combine core and semicircle face intervals exactly" );

    field.setCutCellSampleCount( 8 );
    field.step( dt, std::span<const fluidShapeObstacle2>( &capsule, 1 ), feedback, 180 );
    check( std::abs( field.getCutCellTransition().fluidVolume - expectedVolume ) < 1.0e-4f &&
        !field.getCutCellPressure().hasVolumeHistory,
        "Capsule analytic area must remain unchanged at 4x4 or 8x8 and reset history on switching" );

    // 임의 회전한 Capsule의 넓이는 항상 2*r*L + pi*r^2.
    for( const float angle : { 0.19f, 0.58f, 1.23f, -0.77f } )
    {
        capsule.transform.rotation = rot2::FromRadians( angle );
        field.step( dt, std::span<const fluidShapeObstacle2>( &capsule, 1 ), feedback, 180 );
        check( std::abs( field.getCutCellTransition().fluidVolume - expectedVolume ) < 1.0e-4f &&
            field.getSweptFaceFlux().usedSweptFlux &&
            std::isfinite( field.getSweptFaceFlux().geometryMismatch ),
            "rotated Capsule must retain exact area and finite swept-face geometry diagnostics" );
    }

    field.setCutCellCoupledProjectionEnabled( true );
    capsule.linearVelocity = { 0.85f, -0.25f };
    capsule.angularVelocity = 0.9f;
    capsule.inverseMass = 0.5f;
    capsule.inverseInertia = 1.25f;
    field.addVelocity( { -1.1f, 0.25f }, { 1.3f, -0.4f }, 1.4f );
    field.step( dt, std::span<const fluidShapeObstacle2>( &capsule, 1 ), feedback, 200 );
    const fluidCutCellPressure2& pressure = field.getCutCellPressure();
    check( pressure.usedCoupledProjection && !pressure.coupledSolveFailed &&
        pressure.coupledStatus == fluidCoupledPressureStatus2::converged &&
        pressure.synchronizedPartialFaces > 0 &&
        std::isfinite( pressure.predictedContinuityRms ) &&
        std::abs( pressure.predictedContinuityRms - pressure.pressureResidual ) <
            2.0e-3f + 0.1f * pressure.pressureResidual,
        "rotated Capsule Schur traction must agree with clipped blocked-face velocity response" );

    capsule.transform.position += vec2{ 0.15f, 0.08f };
    capsule.center = capsule.transform.position;
    field.step( dt, std::span<const fluidShapeObstacle2>( &capsule, 1 ), feedback, 200 );
    check( field.getSweptFaceFlux().usedSweptFlux &&
        field.getSweptFaceFlux().totalAbsoluteFlux > 0.0f &&
        std::isfinite( field.getSweptFaceFlux().geometryMismatch ) &&
        std::abs( field.getCutCellTransition().fluidVolume - expectedVolume ) < 1.0e-4f,
        "translated Capsule must produce finite swept pressure geometry and conserve analytic shape area" );

    field.setAnalyticCapsuleCutCellEnabled( false );
    field.step( dt, std::span<const fluidShapeObstacle2>( &capsule, 1 ), feedback, 200 );
    check( !field.getCutCellPressure().hasVolumeHistory &&
        !field.getSweptFaceFlux().usedSweptFlux,
        "disabling analytic Capsule must invalidate prior geometric source history" );

    // Capsule 둘을 하나의 해석 경로로 합치지 않는다: 혼합/중첩은 기존 표본 방식.
    fluidField2 fallback{ 24, 16, h, { -6.0f, -4.0f } };
    fluidField2 sampled{ 24, 16, h, { -6.0f, -4.0f } };
    fallback.setCutCellProjectionEnabled( true );
    sampled.setCutCellProjectionEnabled( true );
    fallback.setAnalyticCapsuleCutCellEnabled( true );
    std::array<fluidShapeObstacle2, 2> obstacles{ capsule, capsule };
    obstacles[1].transform.position += vec2{ 1.6f, 0.5f };
    obstacles[1].center = obstacles[1].transform.position;
    std::array<fluidObstacleForce2, 2> forces{};
    fallback.step( dt, obstacles, forces, 120 );
    sampled.step( dt, obstacles, forces, 120 );
    check( std::abs( fallback.getCutCellTransition().fluidVolume -
        sampled.getCutCellTransition().fluidVolume ) < 1.0e-5f,
        "multiple Capsule obstacles must preserve sampled union geometry fallback" );

    field.clear();
    check( field.isAnalyticCapsuleCutCellEnabled() == false &&
        !field.getSweptFaceFlux().usedSweptFlux,
        "clear must not enable analytic Capsule after the explicit disable" );
}

void checkAnalyticSharpPolygonCutCell()
{
    constexpr float dt = 1.0f / 60.0f;
    constexpr float h = 0.5f;
    constexpr vec2 center{ 0.13f, -0.20f };
    constexpr float halfWidth = 0.63f;
    constexpr float halfHeight = 0.33f;
    constexpr float expectedArea = 24.0f * 16.0f * h * h - 4.0f * halfWidth * halfHeight;

    fluidField2 field{ 24, 16, h, { -6.0f, -4.0f } };
    field.setCutCellProjectionEnabled( true );
    field.setAnalyticPolygonCutCellEnabled( true );
    fluidShapeObstacle2 box{
        MakeBox( { halfWidth, halfHeight } ),
        { center, {} }, center, {}, 0.0f
    };
    std::array<fluidObstacleForce2, 1> forces{};
    field.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), forces, 180 );
    check( std::abs( field.getCutCellTransition().fluidVolume - expectedArea ) < 1.0e-4f &&
        !field.getSweptFaceFlux().usedSweptFlux &&
        !field.getCutCellPressure().hasVolumeHistory,
        "analytic sharp polygon clipping must reproduce box area on first frame" );

    // U x=0.5 y=[0,0.5], box upper y=0.13 -> blocked length=0.13.
    check( std::abs( field.getUFaceOpenFraction( 13, 8 ) - 0.74f ) < 1.0e-5f,
        "analytic polygon U face must use exact vertical line clipping" );
    // V y=0 x=[0.5,1.0], box right x=0.76 -> blocked length=0.26.
    check( std::abs( field.getVFaceOpenFraction( 13, 8 ) - 0.48f ) < 1.0e-5f,
        "analytic polygon V face must use exact horizontal line clipping" );

    field.setCutCellSampleCount( 8 );
    field.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), forces, 180 );
    check( std::abs( field.getCutCellTransition().fluidVolume - expectedArea ) < 1.0e-4f &&
        !field.getCutCellPressure().hasVolumeHistory,
        "analytical polygon area must not depend on 4x4 or 8x8 sampling" );

    box.transform.rotation = rot2::FromRadians( 0.41f );
    field.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), forces, 180 );
    check( field.getSweptFaceFlux().usedSweptFlux &&
        field.getCutCellPressure().hasVolumeHistory &&
        std::isfinite( field.getSweptFaceFlux().geometryMismatch ) &&
        std::abs( field.getCutCellTransition().fluidVolume - expectedArea ) < 1.0e-4f,
        "rotated sharp polygon must keep analytic covered area and produce swept faces" );

    field.setAnalyticPolygonCutCellEnabled( false );
    field.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), forces, 180 );
    check( !field.getCutCellPressure().hasVolumeHistory &&
        !field.getSweptFaceFlux().usedSweptFlux,
        "changing analytic polygon geometry mode must invalidate old volume history" );

    // Pressure Schur and predicted blocked-face flux must use the same clipped lengths.
    field.setAnalyticPolygonCutCellEnabled( true );
    field.setCutCellCoupledProjectionEnabled( true );
    box.linearVelocity = { 0.8f, -0.4f };
    box.angularVelocity = 1.1f;
    box.inverseMass = 0.5f;
    box.inverseInertia = 1.25f;
    field.addVelocity( { -1.1f, 0.25f }, { 1.3f, -0.4f }, 1.4f );
    field.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), forces, 200 );
    const auto& pressure = field.getCutCellPressure();
    check( pressure.usedCoupledProjection && !pressure.coupledSolveFailed &&
        pressure.coupledStatus == fluidCoupledPressureStatus2::converged &&
        pressure.synchronizedPartialFaces > 0 &&
        std::isfinite( pressure.predictedContinuityRms ) &&
        std::abs( pressure.predictedContinuityRms - pressure.pressureResidual ) <
            2.0e-3f + 0.1f * pressure.pressureResidual,
        "sharp polygon clipping must keep Schur traction and blocked-flux prediction adjoint" );

    box.transform.position += vec2{ 0.2f, -0.1f };
    box.center = box.transform.position;
    field.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), forces, 200 );
    check( field.getSweptFaceFlux().usedSweptFlux &&
        field.getSweptFaceFlux().totalAbsoluteFlux > 0.0f &&
        std::isfinite( field.getSweptFaceFlux().geometryMismatch ) &&
        std::abs( field.getCutCellTransition().fluidVolume - expectedArea ) < 1.0e-4f,
        "moving and rotating sharp polygon must retain area and swept face volume diagnostics" );

    // Rounded polygon remains handled by original sampled geometry, not silently as a sharp core.
    fluidField2 fallback{ 24, 16, h, { -6.0f, -4.0f } };
    fluidField2 sampled{ 24, 16, h, { -6.0f, -4.0f } };
    fallback.setCutCellProjectionEnabled( true );
    sampled.setCutCellProjectionEnabled( true );
    fallback.setAnalyticPolygonCutCellEnabled( true );
    auto rounded = box;
    std::get<polygon2>( rounded.geometry ).radius = 0.12f;
    fallback.step( dt, std::span<const fluidShapeObstacle2>( &rounded, 1 ), forces, 120 );
    sampled.step( dt, std::span<const fluidShapeObstacle2>( &rounded, 1 ), forces, 120 );
    check( std::abs( fallback.getCutCellTransition().fluidVolume -
        sampled.getCutCellTransition().fluidVolume ) < 1.0e-5f,
        "rounded polygon must fall back to original sampled cut-cell geometry" );
    field.clear();
    check( field.isAnalyticPolygonCutCellEnabled() &&
        !field.getSweptFaceFlux().usedSweptFlux,
        "clear must preserve analytic polygon selection while resetting geometry diagnostics" );
}

void checkAnalyticSingleCircleCutCell()
{
    constexpr float dt = 1.0f / 60.0f;
    constexpr float h = 0.5f;
    constexpr float radius = 0.83f;
    constexpr vec2 center{ 0.17f, -0.21f };
    constexpr float expected = 24.0f * 16.0f * h * h -
        std::numbers::pi_v<float> * radius * radius;
    fluidField2 analytic{ 24, 16, h, { -6.0f, -4.0f } };
    analytic.setCutCellProjectionEnabled( true );
    analytic.setAnalyticCircleCutCellEnabled( true );
    fluidShapeObstacle2 circle{
        circle2{ {}, radius }, { center, {} }, center, {}, 0.0f
    };
    std::array<fluidObstacleForce2, 1> forces{};
    analytic.step( dt, std::span<const fluidShapeObstacle2>( &circle, 1 ), forces, 160 );
    const float area = analytic.getCutCellTransition().fluidVolume;
    check( std::abs( area - expected ) < 1.0e-4f &&
        !analytic.getCutCellPressure().hasVolumeHistory,
        "analytic circle-rectangle clipping must match exact disc area on initial frame" );

    // x=0.5, y=[0.5,1.0] 수직 MAC 면은 원의 윗부분에서 일부만 막힘.
    const float verticalHalfChord = std::sqrt( radius * radius -
        ( 0.5f - center.x ) * ( 0.5f - center.x ) );
    const float clippedU = std::clamp( ( center.y + verticalHalfChord - 0.5f ) / h, 0.0f, 1.0f );
    check( std::abs( analytic.getUFaceOpenFraction( 13, 9 ) - ( 1.0f - clippedU ) ) < 2.0e-5f,
        "analytic circle U face must use the exact blocked interval length" );

    // y=0.5, x=[0.5,1.0] 수평 MAC 면 역시 실제 원-직선 교차 길이.
    const float horizontalHalfChord = std::sqrt( radius * radius -
        ( 0.5f - center.y ) * ( 0.5f - center.y ) );
    const float clippedV = std::clamp( ( center.x + horizontalHalfChord - 0.5f ) / h, 0.0f, 1.0f );
    check( std::abs( analytic.getVFaceOpenFraction( 13, 9 ) - ( 1.0f - clippedV ) ) < 2.0e-5f,
        "analytic circle V face must use the exact blocked interval length" );

    analytic.setCutCellSampleCount( 8 );
    analytic.step( dt, std::span<const fluidShapeObstacle2>( &circle, 1 ), forces, 160 );
    check( std::abs( analytic.getCutCellTransition().fluidVolume - expected ) < 1.0e-4f &&
        !analytic.getCutCellPressure().hasVolumeHistory,
        "analytic Circle coverage must be independent of 4x4/8x8 and reset history on sample changes" );
    analytic.setCutCellSampleCount( 4 );
    analytic.setCutCellCoupledProjectionEnabled( true );
    circle.linearVelocity = { 0.9f, -0.4f };
    circle.inverseMass = 0.5f;
    circle.inverseInertia = 1.2f;
    analytic.addVelocity( { -1.1f, 0.25f }, { 1.3f, -0.4f }, 1.4f );
    analytic.step( dt, std::span<const fluidShapeObstacle2>( &circle, 1 ), forces, 200 );
    const auto& pressure = analytic.getCutCellPressure();
    check( pressure.usedCoupledProjection && !pressure.coupledSolveFailed &&
        std::isfinite( pressure.predictedContinuityRms ) &&
        std::abs( pressure.predictedContinuityRms - pressure.pressureResidual ) <
            2.0e-3f + 0.1f * pressure.pressureResidual,
        "analytic Circle Schur pressure and predicted blocked face flux must remain adjoint" );

    circle.transform.position.x += 0.2f;
    circle.center.x += 0.2f;
    analytic.step( dt, std::span<const fluidShapeObstacle2>( &circle, 1 ), forces, 200 );
    check( analytic.getSweptFaceFlux().usedSweptFlux &&
        analytic.getCutCellPressure().hasVolumeHistory &&
        std::isfinite( analytic.getSweptFaceFlux().geometryMismatch ) &&
        std::abs( analytic.getCutCellTransition().fluidVolume - expected ) < 1.0e-4f,
        "moving analytic Circle must use clipped swept faces without losing its exact covered area" );

    // 실험 경로를 꺼도 이전의 exact area를 표본 점유율 차이로 해석하면 안 된다.
    analytic.setAnalyticCircleCutCellEnabled( false );
    analytic.step( dt, std::span<const fluidShapeObstacle2>( &circle, 1 ), forces, 200 );
    check( !analytic.getCutCellPressure().hasVolumeHistory &&
        !analytic.getSweptFaceFlux().usedSweptFlux,
        "switching exact Circle geometry off must invalidate previous swept occupancy history" );

    fluidField2 fallback{ 24, 16, h, { -6.0f, -4.0f } };
    fallback.setCutCellProjectionEnabled( true );
    fallback.setAnalyticCircleCutCellEnabled( true );
    circle.geometry = MakeBox( { 0.7f, 0.3f } );
    fallback.step( dt, std::span<const fluidShapeObstacle2>( &circle, 1 ), forces, 160 );
    check( fallback.getCutCellTransition().partialCells > 0 &&
        !fallback.getSweptFaceFlux().usedSweptFlux,
        "unsupported polygon must safely fall back to existing spatial cut-cell sampling" );

    analytic.clear();
    check( !analytic.getSweptFaceFlux().usedSweptFlux &&
        !analytic.isAnalyticCircleCutCellEnabled(),
        "the optional analytic Circle toggle must remain off across clear" );
}

void checkCutCellSpatialQuadrature()
{
    constexpr float dt = 1.0f / 60.0f;
    constexpr float radius = 0.83f;
    constexpr float h = 0.5f;
    constexpr int width = 24;
    constexpr int height = 16;
    const float expectedFluidArea = width * height * h * h -
        std::numbers::pi_v<float> * radius * radius;
    fluidField2 base{ width, height, h, { -6.0f, -4.0f } };
    fluidField2 refined{ width, height, h, { -6.0f, -4.0f } };
    for( fluidField2* field : { &base, &refined } )
        field->setCutCellProjectionEnabled( true );
    refined.setCutCellSampleCount( 8 );
    check( base.getCutCellSampleCount() == 4 && refined.getCutCellSampleCount() == 8,
        "spatial quadrature must default to 4x4 and allow 8x8" );
    fluidShapeObstacle2 disk{
        circle2{ {}, radius }, { { 0.18f, -0.23f }, {} },
        { 0.18f, -0.23f }, {}, 0.0f
    };
    std::array<fluidObstacleForce2, 1> feedback{};
    for( fluidField2* field : { &base, &refined } )
        field->step( dt, std::span<const fluidShapeObstacle2>( &disk, 1 ), feedback, 120 );
    const float error4 = std::abs( base.getCutCellTransition().fluidVolume - expectedFluidArea );
    const float error8 = std::abs( refined.getCutCellTransition().fluidVolume - expectedFluidArea );
    check( error8 < error4 * 0.6f && error8 < 0.004f,
        "8x8 geometry sampling should improve analytic disk area for the reference fixture" );

    // 이산화 규칙이 바뀌면 이전 4x4 snapshot을 8x8 기하에 대입해서는 안 된다.
    base.setCutCellSampleCount( 8 );
    check( !base.getSweptFaceFlux().usedSweptFlux && base.getCutCellSampleCount() == 8,
        "changing cut-cell spatial resolution must invalidate the old swept history" );
    base.step( dt, std::span<const fluidShapeObstacle2>( &disk, 1 ), feedback, 120 );
    check( !base.getCutCellPressure().hasVolumeHistory &&
        !base.getSweptFaceFlux().usedSweptFlux &&
        std::abs( base.getCutCellTransition().fluidVolume -
            refined.getCutCellTransition().fluidVolume ) < 1.0e-5f,
        "first 8x8 frame must rebuild same geometry without synthetic swept-volume source" );

    disk.transform.position.x += 0.2f;
    disk.center.x += 0.2f;
    for( fluidField2* field : { &base, &refined } )
        field->step( dt, std::span<const fluidShapeObstacle2>( &disk, 1 ), feedback, 120 );
    check( base.getSweptFaceFlux().usedSweptFlux &&
        refined.getSweptFaceFlux().usedSweptFlux &&
        base.getCutCellPressure().hasVolumeHistory &&
        std::isfinite( base.getSweptFaceFlux().geometryMismatch ) &&
        std::abs( base.getSweptFaceFlux().geometryMismatch -
            refined.getSweptFaceFlux().geometryMismatch ) < 1.0e-4f,
        "spatial resolution changes must preserve consistent swept face quadrature on the next moving frame" );

    // 정적 Circle뿐 아니라 이동/회전하는 Body와 PCG Schur 역시 8점 표본을 사용해야 한다.
    fluidField2 coupled{ 24, 16, 0.5f, { -6.0f, -4.0f } };
    coupled.setCutCellProjectionEnabled( true );
    coupled.setCutCellCoupledProjectionEnabled( true );
    coupled.setCutCellSampleCount( 8 );
    fluidShapeObstacle2 box{
        MakeBox( { 0.73f, 0.31f } ),
        { { 0.13f, -0.17f }, rot2::FromRadians( 0.31f ) },
        { 0.13f, -0.17f }, { 1.0f, -0.5f }, 1.2f, 0.5f, 1.25f
    };
    coupled.addVelocity( { -1.1f, 0.25f }, { 1.3f, -0.4f }, 1.4f );
    coupled.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), feedback, 200 );
    const fluidCutCellPressure2& stats = coupled.getCutCellPressure();
    check( stats.usedCoupledProjection && !stats.coupledSolveFailed &&
        stats.coupledStatus == fluidCoupledPressureStatus2::converged &&
        stats.synchronizedPartialFaces > 0 &&
        std::isfinite( stats.predictedContinuityRms ) &&
        std::abs( stats.predictedContinuityRms - stats.pressureResidual ) <
            2.0e-3f + 0.1f * stats.pressureResidual,
        "8x8 cut-cell pressure B/B^T and predicted blocked-face volume flux must remain adjoint" );
    coupled.clear();
    check( coupled.getCutCellSampleCount() == 8 &&
        !coupled.getSweptFaceFlux().usedSweptFlux,
        "clear must keep spatial quadrature selection without retaining old shape history" );
    coupled.setCutCellSampleCount( 3 );
    check( coupled.getCutCellSampleCount() == 4,
        "unsupported small sample counts must resolve to safe 4x4 default" );
}

void checkCutCellTemporalContinuity()
{
    constexpr float dt = 1.0f / 60.0f;
    fluidField2 field{ 12, 12, 1.0f, { -6.0f, -6.0f } };
    field.setCutCellProjectionEnabled( true );
    std::array<fluidObstacleForce2, 1> feedback{};
    fluidShapeObstacle2 box{
        MakeBox( { 0.75f, 0.75f } ),
        { { 0.25f, 0.25f }, {} }, { 0.25f, 0.25f }, {}, 0.0f
    };

    // 최초 snapshot은 움직이기 전 형상을 알 수 없으므로 시간적 소스를 넣지 않는다.
    field.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), feedback, 200 );
    const fluidCutCellPressure2 first = field.getCutCellPressure();
    check( !first.hasVolumeHistory && first.volumeRate == 0.0f &&
        first.divergenceBefore == 0.0f && first.divergenceAfter == 0.0f &&
        field.getCutCellTransition().coveredVolume > 0.0f,
        "initial obstacle placement must not be mistaken for a physical swept-volume step" );

    // 고정 면적 강체의 1/4 셀 평행 이동: 총 ΔV는 0이지만 국소 ΔV_i는 다르다.
    box.transform.position.x += 0.25f;
    box.center.x += 0.25f;
    field.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), feedback, 200 );
    const fluidCutCellPressure2 moved = field.getCutCellPressure();
    check( moved.hasVolumeHistory && std::abs( moved.volumeRate ) < 1.0e-4f &&
        field.getCutCellTransition().coveredVolume > 0.0f &&
        field.getCutCellTransition().releasedVolume > 0.0f &&
        moved.divergenceBefore > 0.01f &&
        moved.divergenceAfter < moved.divergenceBefore * 0.6f &&
        moved.connectedRegions == 1 &&
        std::abs( moved.sourceImbalance ) < 1.0e-4f &&
        std::isfinite( moved.pressureResidual ) && IsFinite( feedback[0].force ),
        "translation must drive pressure through local dV/dt with closed-domain global balance" );

    // 정지한 동일한 도형은 추가 체적 소스를 만들지 않는다.
    field.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), feedback, 200 );
    check( field.getCutCellPressure().hasVolumeHistory &&
        field.getCutCellPressure().volumeRate == 0.0f &&
        field.getCutCellPressure().redistributedVolumeRate == 0.0f &&
        std::isfinite( field.getCutCellPressure().divergenceAfter ),
        "static occupancy must not create fictitious moving-boundary volume source" );

    // 강체를 영역 밖으로 이동시키면 폐쇄 영역의 전체 유체 체적이 증가한다.
    // 이를 압력으로 해결할 수 없으므로 compatibility correction이 드러나야 한다.
    box.transform.position = { 50.0f, 50.0f };
    box.center = box.transform.position;
    field.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), feedback, 200 );
    const fluidCutCellPressure2 departed = field.getCutCellPressure();
    check( departed.volumeRate > 0.0f &&
        departed.sourceImbalance > 0.0f &&
        departed.compatibilityCorrection >= 0.99f * departed.sourceImbalance,
        "closed domain must report unresolvable positive net fluid volume growth" );

    // 완전히 새로 덮인 셀에는 압력 미지수가 없기 때문에 그 체적 소스를
    // 가까운 활성 셀로 옮겨야 한다. 그대로 버리면 이동 강체의 ΔV가 누락된다.
    fluidField2 covered{ 12, 12, 1.0f, { -6.0f, -6.0f } };
    covered.setCutCellProjectionEnabled( true );
    fluidShapeObstacle2 largeBox{
        MakeBox( { 1.5f, 1.5f } ),
        { { -1.5f, 0.0f }, {} }, { -1.5f, 0.0f }, {}, 0.0f
    };
    covered.step( dt, std::span<const fluidShapeObstacle2>( &largeBox, 1 ), feedback );
    largeBox.transform.position.x += 1.0f;
    largeBox.center.x += 1.0f;
    covered.step( dt, std::span<const fluidShapeObstacle2>( &largeBox, 1 ), feedback, 200 );
    const fluidCutCellPressure2 coveredStats = covered.getCutCellPressure();
    check( coveredStats.redistributedVolumeRate < -1.0f &&
        coveredStats.unresolvedVolumeRate == 0.0f &&
        std::abs( coveredStats.volumeRate ) < 1.0e-4f &&
        std::isfinite( coveredStats.divergenceAfter ),
        "fully covered new solid cells must deliver missing volume source to nearby active cells" );

    // 도메인을 가로지르는 장벽으로 분리된 유체 영역은 압력 Neumann
    // 호환성을 각 영역에서 독립적으로 검사해야 한다.
    fluidField2 pockets{ 12, 12, 1.0f, { -6.0f, -6.0f } };
    pockets.setCutCellProjectionEnabled( true );
    std::array<fluidShapeObstacle2, 2> split{
        fluidShapeObstacle2{ MakeBox( { 0.5f, 6.1f } ),
            { { 0.5f, 0.0f }, {} }, { 0.5f, 0.0f }, {}, 0.0f },
        fluidShapeObstacle2{ circle2{ {}, 0.65f },
            { { -2.5f, 0.0f }, {} }, { -2.5f, 0.0f }, {}, 0.0f }
    };
    std::array<fluidObstacleForce2, 2> splitFeedback{};
    pockets.step( dt, split, splitFeedback );
    split[1].transform.position.x += 0.25f;
    split[1].center.x += 0.25f;
    pockets.step( dt, split, splitFeedback, 200 );
    const fluidCutCellPressure2 splitStats = pockets.getCutCellPressure();
    check( splitStats.connectedRegions == 2 &&
        std::isfinite( splitStats.pressureResidual ) &&
        splitStats.compatibilityCorrection + 1.0e-4f >=
            std::abs( splitStats.sourceImbalance ),
        "disconnected fluid regions must each receive a separate Neumann compatibility correction" );

    field.clear();
    field.step( dt );
    check( !field.getCutCellPressure().hasVolumeHistory &&
        field.getCutCellPressure().volumeRate == 0.0f,
        "clear must invalidate temporal geometry history" );
}

void checkSweptSolidFaceFlux()
{
    constexpr float dt = 1.0f / 60.0f;
    fluidField2 field{ 16, 16, 0.5f, { -4.0f, -4.0f } };
    field.setCutCellProjectionEnabled( true );
    std::array<fluidObstacleForce2, 1> feedback{};

    fluidShapeObstacle2 box{
        MakeBox( { 0.8f, 0.3f } ),
        { { -1.0f, -0.25f }, {} }, { -1.0f, -0.25f }, {}, 0.0f
    };

    field.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), feedback, 200 );
    check( !field.getSweptFaceFlux().usedSweptFlux &&
        !field.getCutCellPressure().hasVolumeHistory,
        "first shape snapshot must not synthesize a past swept path" );

    // 한 step 동안 여러 셀을 지나가면 각 공유 face에 동일한 적분 flux가 저장된다.
    box.transform.position.x += 1.25f;
    box.center.x += 1.25f;
    field.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), feedback, 200 );
    const fluidSweptFaceFlux2 moving = field.getSweptFaceFlux();
    const fluidCutCellPressure2 projection = field.getCutCellPressure();
    check( moving.usedSweptFlux && moving.timeSamples >= 4 &&
        moving.totalAbsoluteFlux > 0.5f &&
        std::isfinite( moving.geometryMismatch ) && moving.geometryMismatch >= 0.0f &&
        projection.hasVolumeHistory && projection.divergenceBefore > 0.01f &&
        std::abs( projection.sourceImbalance ) < 1.0e-3f &&
        std::isfinite( projection.divergenceAfter ) && IsFinite( feedback[0].force ),
        "translating polygon must integrate flux through crossed faces and preserve shared-face balance" );

    field.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), feedback, 200 );
    check( field.getSweptFaceFlux().usedSweptFlux &&
        field.getSweptFaceFlux().totalAbsoluteFlux == 0.0f &&
        field.getSweptFaceFlux().timeSamples == 1,
        "stationary shape should produce zero swept flux without extra temporal samples" );

    // 회전만 한 얇은 다각형은 위치 변위가 없어도 실제 면 경계가 쓸고 지나간다.
    box.transform.rotation = rot2::FromRadians( 0.5f * std::numbers::pi_v<float> );
    field.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), feedback, 200 );
    check( field.getSweptFaceFlux().usedSweptFlux &&
        field.getSweptFaceFlux().timeSamples >= 4 &&
        field.getSweptFaceFlux().totalAbsoluteFlux > 0.01f &&
        std::isfinite( field.getCutCellPressure().pressureResidual ),
        "rotating polygon must generate sampled swept flux without translation" );

    // 기하가 변하면 강체 운동으로 착각하지 않고 snapshot ΔV 방식으로 돌아간다.
    box.geometry = MakeBox( { 1.0f, 0.3f } );
    field.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), feedback );
    check( !field.getSweptFaceFlux().usedSweptFlux &&
        field.getCutCellPressure().hasVolumeHistory,
        "shape geometry edits must invalidate swept path and use snapshot continuity" );

    // 도메인 밖을 가로지르는 shape은 wall-crossing을 내부 swept flux로 숨기면 안 된다.
    box.transform.position = { 5.0f, -0.25f };
    box.center = box.transform.position;
    field.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), feedback );
    check( !field.getSweptFaceFlux().usedSweptFlux &&
        field.getCutCellPressure().volumeRate > 0.0f,
        "crossing closed outer wall must revert to observable geometric volume imbalance" );

    field.clear();
    check( !field.getSweptFaceFlux().usedSweptFlux &&
        field.getSweptFaceFlux().totalAbsoluteFlux == 0.0f,
        "clear must reset swept volume diagnostics and previous shape history" );
}

void checkMovingOnlyAnalyticSweptDifferential( bool compareBounded = false,
    bool compareSparse = false )
{
    constexpr float dt = 1.0f / 60.0f;
    for( float h : { 0.5f, 0.25f } )
        for( int staticCount : { 16, 48 } )
        {
            // Body 1 is kinematic but moving; use transform displacement, not
            // inverse mass, to select swept surface contributors.
            const vec2 first{ -0.30f, -0.20f };
            const vec2 second{ 1.45f, 0.20f };
            std::vector<fluidShapeObstacle2> bodies{
                { circle2{ {}, 0.45f }, { first, {} }, first,
                    {}, 0.0f, 0.5f, 0.6f },
                { circle2{ {}, 0.45f }, { second, {} }, second,
                    {}, 0.0f, 0.0f, 0.0f }
            };
            int added = 0;
            for( int row = 0; row < 7 && added < staticCount; ++row )
                for( int col = 0; col < 10 && added < staticCount; ++col )
                {
                    const vec2 position{ -5.25f + 1.05f * col, -3.15f + 0.95f * row };
                    bool separated = true;
                    for( std::size_t i = 0; i < 2; ++i )
                        if( LengthSquared( position - bodies[i].center ) < 1.25f * 1.25f )
                        {
                            separated = false;
                            break;
                        }
                    if( !separated ) continue;
                    bodies.push_back( { circle2{ {}, 0.12f }, { position, {} },
                        position, {}, 0.0f, 0.0f, 0.0f } );
                    ++added;
                }
            check( added == staticCount,
                "moving swept differential requires enough separated stationary shapes" );

            fluidField2 reference{ static_cast<int>( 12.0f / h ),
                static_cast<int>( 8.0f / h ), h, { -6.0f, -4.0f } };
            fluidField2 movingOnly{ static_cast<int>( 12.0f / h ),
                static_cast<int>( 8.0f / h ), h, { -6.0f, -4.0f } };
            check( movingOnly.isAnalyticCutCellMovingSweptFacesEnabled(),
                "moving-only analytic swept traversal must be enabled by default" );
            for( fluidField2* field : { &reference, &movingOnly } )
            {
                field->setCutCellProjectionEnabled( true );
                field->setCutCellCoupledProjectionEnabled( true );
                field->setDisjointAnalyticCutCellEnabled( true );
                if( staticCount == 16 && h == 0.5f )
                    field->setAdaptiveSweptSamplingEnabled( true );
                field->addVelocity( { -1.2f, 0.14f }, { 1.4f, -0.27f }, 1.6f );
            }
            if( compareSparse )
                reference.setAnalyticCutCellSparseSweptFacesEnabled( false );
            else if( compareBounded )
                reference.setAnalyticCutCellBoundedSweptFacesEnabled( false );
            else
                reference.setAnalyticCutCellMovingSweptFacesEnabled( false );
            std::vector<fluidObstacleForce2> refForces( bodies.size() ), newForces( bodies.size() );
            for( int frame = 0; frame < 3; ++frame )
            {
                reference.step( dt, bodies, refForces, 200 );
                movingOnly.step( dt, bodies, newForces, 200 );
                const auto& a = reference.getSweptFaceFlux();
                const auto& b = movingOnly.getSweptFaceFlux();
                check( a.usedSweptFlux == b.usedSweptFlux &&
                    a.timeSamples == b.timeSamples &&
                    a.refinements == b.refinements &&
                    a.totalAbsoluteFlux == b.totalAbsoluteFlux &&
                    a.rawGeometryMismatch == b.rawGeometryMismatch &&
                    a.geometryMismatch == b.geometryMismatch,
                    "moving-only swept faces must preserve each swept volume diagnostic exactly" );
                if( frame > 0 )
                    check( b.usedSweptFlux && b.timeSamples >= 2 && b.totalAbsoluteFlux > 0.0f,
                        "moving-only swept comparison must integrate time-varying geometry" );
                const auto& pa = reference.getCutCellPressure();
                const auto& pb = movingOnly.getCutCellPressure();
                check( pa.usedCoupledProjection && pb.usedCoupledProjection &&
                    pa.coupledStatus == pb.coupledStatus &&
                    pa.coupledIterations == pb.coupledIterations &&
                    pa.coupledRelativeResidual == pb.coupledRelativeResidual &&
                    pa.predictedContinuityRms == pb.predictedContinuityRms,
                    "moving-only swept faces must preserve coupled pressure and continuity" );
                for( std::size_t i = 0; i < bodies.size(); ++i )
                    check( refForces[i].force.x == newForces[i].force.x &&
                        refForces[i].force.y == newForces[i].force.y &&
                        refForces[i].torque == newForces[i].torque,
                        "moving-only swept faces must preserve per-body traction" );
                const vec2 originalMomentum = reference.getFluidMomentum();
                const vec2 optimizedMomentum = movingOnly.getFluidMomentum();
                check( originalMomentum.x == optimizedMomentum.x &&
                    originalMomentum.y == optimizedMomentum.y,
                    "moving-only swept faces must preserve fluid momentum" );
                if( frame == 0 )
                {
                    bodies[0].transform.position += vec2{ 0.24f, 0.08f };
                    bodies[1].transform.position += vec2{ -0.12f, -0.09f };
                }
                else if( frame == 1 )
                {
                    bodies[0].transform.position += vec2{ -0.14f, 0.03f };
                    bodies[1].transform.position += vec2{ 0.10f, 0.04f };
                }
                for( std::size_t i = 0; i < 2; ++i )
                    bodies[i].center = bodies[i].transform.position;
            }
            reference.clear();
            movingOnly.clear();
            check( compareSparse
                    ? !reference.isAnalyticCutCellSparseSweptFacesEnabled() &&
                        movingOnly.isAnalyticCutCellSparseSweptFacesEnabled()
                    : compareBounded
                        ? !reference.isAnalyticCutCellBoundedSweptFacesEnabled() &&
                            movingOnly.isAnalyticCutCellBoundedSweptFacesEnabled()
                        : !reference.isAnalyticCutCellMovingSweptFacesEnabled() &&
                            movingOnly.isAnalyticCutCellMovingSweptFacesEnabled(),
                "clear must preserve selected analytic swept traversal mode" );
        }
}

void checkBoundedSweptMixedAnalyticShapes( bool compareSparse = false )
{
    constexpr float dt = 1.0f / 60.0f;
    for( float h : { 0.5f, 0.25f } )
    {
        const vec2 boxCenter{ -2.0f, -1.0f };
        const vec2 capsuleCenter{ 1.0f, -0.2f };
        const vec2 staticCenter{ 3.5f, 2.0f };
        std::array<fluidShapeObstacle2, 3> bodies{{
            { MakeBox( { 0.65f, 0.25f } ),
                { boxCenter, rot2::FromRadians( 0.20f ) }, boxCenter,
                {}, 0.0f, 0.8f, 0.6f },
            { capsule2{ { -0.4f, 0.0f }, { 0.4f, 0.0f }, 0.20f },
                { capsuleCenter, {} }, capsuleCenter,
                {}, 0.0f, 0.0f, 0.0f },
            { circle2{ {}, 0.35f }, { staticCenter, {} }, staticCenter,
                {}, 0.0f, 0.0f, 0.0f }
        }};
        fluidField2 reference{ static_cast<int>( 12.0f / h ),
            static_cast<int>( 10.0f / h ), h, { -6.0f, -5.0f } };
        fluidField2 bounded{ static_cast<int>( 12.0f / h ),
            static_cast<int>( 10.0f / h ), h, { -6.0f, -5.0f } };
        for( fluidField2* field : { &reference, &bounded } )
        {
            field->setCutCellProjectionEnabled( true );
            field->setCutCellCoupledProjectionEnabled( true );
            field->setDisjointAnalyticCutCellEnabled( true );
            field->addVelocity( { -1.5f, -0.7f }, { 1.4f, 0.6f }, 1.3f );
        }
        if( compareSparse )
            reference.setAnalyticCutCellSparseSweptFacesEnabled( false );
        else
            reference.setAnalyticCutCellBoundedSweptFacesEnabled( false );
        std::array<fluidObstacleForce2, 3> refForces{}, newForces{};
        for( int frame = 0; frame < 3; ++frame )
        {
            reference.step( dt, bodies, refForces, 200 );
            bounded.step( dt, bodies, newForces, 200 );
            check( reference.getCutCellTransition().usedDisjointAnalyticGeometry &&
                bounded.getCutCellTransition().usedDisjointAnalyticGeometry,
                "mixed moving swept comparison must remain on disjoint analytic geometry" );
            const auto& a = reference.getSweptFaceFlux();
            const auto& b = bounded.getSweptFaceFlux();
            check( a.usedSweptFlux == b.usedSweptFlux &&
                a.timeSamples == b.timeSamples &&
                a.totalAbsoluteFlux == b.totalAbsoluteFlux &&
                a.geometryMismatch == b.geometryMismatch &&
                a.rawGeometryMismatch == b.rawGeometryMismatch,
                "AABB-bounded swept clipping must preserve polygon and capsule flux exactly" );
            if( frame > 0 )
                check( b.usedSweptFlux && b.totalAbsoluteFlux > 0.0f &&
                    b.timeSamples >= 2,
                    "polygon and capsule transform changes must generate swept face flux" );
            const auto& pa = reference.getCutCellPressure();
            const auto& pb = bounded.getCutCellPressure();
            check( pa.usedCoupledProjection == pb.usedCoupledProjection &&
                pa.coupledStatus == pb.coupledStatus &&
                pa.coupledIterations == pb.coupledIterations &&
                pa.coupledRelativeResidual == pb.coupledRelativeResidual &&
                pa.predictedContinuityRms == pb.predictedContinuityRms,
                "mixed analytic swept clipping must preserve coupled pressure and continuity" );
            for( std::size_t i = 0; i < bodies.size(); ++i )
                check( refForces[i].force.x == newForces[i].force.x &&
                    refForces[i].force.y == newForces[i].force.y &&
                    refForces[i].torque == newForces[i].torque,
                    "mixed analytic swept clipping must preserve per-body traction" );
            const vec2 first = reference.getFluidMomentum();
            const vec2 second = bounded.getFluidMomentum();
            check( first.x == second.x && first.y == second.y,
                "mixed analytic swept clipping must preserve fluid momentum" );

            if( frame == 0 )
            {
                bodies[0].transform.position += { 0.32f, 0.14f };
                bodies[0].transform.rotation = rot2::FromRadians( 0.52f );
                bodies[1].transform.position += { -0.18f, 0.12f };
                bodies[1].transform.rotation = rot2::FromRadians( -0.26f );
            }
            else if( frame == 1 )
            {
                bodies[0].transform.position += { -0.16f, -0.09f };
                bodies[0].transform.rotation = rot2::FromRadians( -0.18f );
                bodies[1].transform.position += { 0.11f, -0.08f };
                bodies[1].transform.rotation = rot2::FromRadians( 0.34f );
            }
            for( std::size_t i = 0; i < 2; ++i )
                bodies[i].center = bodies[i].transform.position;
        }
    }
}

void checkAdaptiveSweptTemporalQuadrature()
{
    constexpr float dt = 1.0f / 60.0f;
    const std::array<vec2, 3> translations{
        vec2{ 0.32f, 0.12f }, vec2{ 0.85f, -0.14f }, vec2{ -0.64f, 0.27f }
    };
    const std::array<float, 3> rotations{ 0.27f, 0.67f, -0.82f };
    for( std::size_t trial = 0; trial < translations.size(); ++trial )
    {
        fluidField2 standard{ 20, 18, 0.5f, { -5.0f, -4.5f } };
        fluidField2 adaptive{ 20, 18, 0.5f, { -5.0f, -4.5f } };
        standard.setCutCellProjectionEnabled( true );
        adaptive.setCutCellProjectionEnabled( true );
        adaptive.setAdaptiveSweptSamplingEnabled( true );
        fluidShapeObstacle2 polygon{
            MakeBox( { 0.81f, 0.28f } ),
            { { -0.93f, -0.42f }, rot2::FromRadians( 0.13f ) },
            { -0.93f, -0.42f }, {}, 0.0f
        };
        std::array<fluidObstacleForce2, 1> forces{};
        for( fluidField2* field : { &standard, &adaptive } )
            field->step( dt, std::span<const fluidShapeObstacle2>( &polygon, 1 ), forces, 40 );
        polygon.transform.position += translations[trial];
        polygon.center = polygon.transform.position;
        polygon.transform.rotation = rot2::FromRadians( 0.13f + rotations[trial] );
        for( fluidField2* field : { &standard, &adaptive } )
            field->step( dt, std::span<const fluidShapeObstacle2>( &polygon, 1 ), forces, 40 );

        const fluidSweptFaceFlux2& basic = standard.getSweptFaceFlux();
        const fluidSweptFaceFlux2& refined = adaptive.getSweptFaceFlux();
        check( basic.usedSweptFlux && refined.usedSweptFlux &&
            basic.timeSamples == refined.initialTimeSamples &&
            refined.timeSamples >= basic.timeSamples &&
            refined.timeSamples <= 64 && refined.refinements >= 0 &&
            std::isfinite( refined.geometryMismatch ) &&
            std::abs( basic.geometryMismatch - refined.initialGeometryMismatch ) < 1.0e-4f &&
            refined.geometryMismatch <= basic.geometryMismatch + 1.0e-5f,
            "adaptive swept temporal quadrature must never worsen measured cell geometry mismatch" );
        if( refined.refinements > 0 )
        {
            check( refined.geometryMismatch < refined.initialGeometryMismatch &&
                refined.timeSamples > refined.initialTimeSamples,
                "adaptive swept quadrature must accept only strictly beneficial refinements" );
        }
    }

    fluidField2 staticField{ 20, 18, 0.5f, { -5.0f, -4.5f } };
    staticField.setCutCellProjectionEnabled( true );
    staticField.setAdaptiveSweptSamplingEnabled( true );
    fluidShapeObstacle2 stationary{
        circle2{ {}, 0.8f }, { { 0.0f, 0.0f }, {} }, {}, {}, 0.0f
    };
    std::array<fluidObstacleForce2, 1> forces{};
    staticField.step( dt, std::span<const fluidShapeObstacle2>( &stationary, 1 ), forces, 40 );
    staticField.step( dt, std::span<const fluidShapeObstacle2>( &stationary, 1 ), forces, 40 );
    check( staticField.getSweptFaceFlux().usedSweptFlux &&
        staticField.getSweptFaceFlux().timeSamples == 1 &&
        staticField.getSweptFaceFlux().initialTimeSamples == 0 &&
        staticField.getSweptFaceFlux().refinements == 0 &&
        staticField.getSweptFaceFlux().totalAbsoluteFlux == 0.0f,
        "static solid should skip all temporal face integration even in adaptive mode" );
    staticField.clear();
    check( staticField.isAdaptiveSweptSamplingEnabled() &&
        !staticField.getSweptFaceFlux().usedSweptFlux &&
        staticField.getSweptFaceFlux().refinements == 0,
        "clearing field must keep optional adaptive setting and reset quadrature statistics" );
}

void checkConservativeMacAdvection()
{
    constexpr float dt = 1.0f / 30.0f;
    fluidField2 conservative{ 24, 20, 0.5f, { -6.0f, -5.0f } };
    fluidField2 legacy{ 24, 20, 0.5f, { -6.0f, -5.0f } };
    conservative.setConservativeAdvectionEnabled( true );

    for( fluidField2* field : { &conservative, &legacy } )
    {
        field->addVelocity( { -1.0f, 0.8f }, { 4.0f, 1.5f }, 1.5f );
        field->addVelocity( { 1.2f, -0.8f }, { -1.2f, 2.5f }, 0.8f );
    }

    conservative.step( dt, 1 );
    legacy.step( dt, 1 );
    const fluidAdvectionStats2& stats = conservative.getAdvectionStats();
    const fluidAdvectionStats2& old = legacy.getAdvectionStats();
    const float scale = 1.0f + Length( stats.before );
    check( stats.conservative && !old.conservative &&
        stats.substeps >= 1 && stats.maxCfl > 0.0f && !stats.cflLimited &&
        Length( stats.before - old.before ) < 1.0e-4f * scale,
        "conservative and semi-Lagrangian advection must start from the same momentum" );
    check( Length( stats.delta() ) < 2.0e-4f * scale &&
        IsFinite( stats.after ) && IsFinite( conservative.sampleVelocity( { 1.0f, 0.0f } ) ),
        "shared MAC dual-edge fluxes must conserve weighted face momentum on an empty closed grid" );

    // 큰 속도/시간 간격은 여러 CFL substep으로 분리하되, 각 단계가 같은 flux를 공유.
    conservative.clear();
    conservative.addVelocity( { -0.5f, 0.0f }, { 8.0f, 0.0f }, 1.0f );
    conservative.step( 0.2f, 1 );
    const fluidAdvectionStats2 substepped = conservative.getAdvectionStats();
    check( substepped.conservative && substepped.substeps >= 3 &&
        !substepped.cflLimited &&
        Length( substepped.delta() ) < 2.0e-4f * ( 1.0f + Length( substepped.before ) ),
        "CFL substeps must not break closed-grid advection momentum conservation" );

    // 부분 개방 면을 적용하더라도 유량은 양쪽 dual cell에서 똑같이 계상되어야 한다.
    fluidField2 partial{ 32, 32, 0.5f, { -8.0f, -8.0f } };
    partial.setCutCellProjectionEnabled( true );
    partial.setConservativeAdvectionEnabled( true );
    const fluidCircleObstacle2 obstacle{ { 0.0f, 0.0f }, 0.8f, {}, 0.0f };
    std::array<fluidObstacleForce2, 1> feedback{};
    partial.step( dt, std::span<const fluidCircleObstacle2>( &obstacle, 1 ), feedback );
    partial.addVelocity( { -1.7f, 0.3f }, { 2.0f, 0.4f }, 1.2f );
    partial.step( dt, std::span<const fluidCircleObstacle2>( &obstacle, 1 ), feedback, 120 );
    const fluidAdvectionStats2 cut = partial.getAdvectionStats();
    check( cut.conservative && partial.getCutCellTransition().partialCells > 0 &&
        Length( cut.delta() ) < 3.0e-4f * ( 1.0f + Length( cut.before ) ) &&
        std::isfinite( partial.getCutCellPressure().divergenceAfter ) &&
        IsFinite( feedback[0].force ),
        "fractionally open faces must conserve dual-face advection momentum independently of pressure" );

    partial.setConservativeAdvectionEnabled( false );
    partial.step( dt, std::span<const fluidCircleObstacle2>( &obstacle, 1 ), feedback );
    check( !partial.getAdvectionStats().conservative &&
        partial.getAdvectionStats().substeps == 1,
        "disabling conservative advection must restore legacy semi-Lagrangian path" );
    partial.setConservativeAdvectionEnabled( true );
    partial.clear();
    check( partial.getAdvectionStats().substeps == 0 &&
        partial.isConservativeAdvectionEnabled(),
        "clear must reset advection measurements but preserve the chosen algorithm" );
}


void checkConservativeTracerTransport()
{
    constexpr float dt = 1.0f / 60.0f;
    fluidField2 field{ 24, 20, 0.5f, { -6.0f, -5.0f } };
    check( close( field.getTracerMass(), 0.0f ) && field.getTracerStats().substeps == 0,
        "tracer must be disabled and empty by default" );
    field.setTracerConcentration( 12, 10, 4.0f );
    check( close( field.getTracerMass(), 1.0f ) && close( field.getTracerConcentration( 12, 10 ), 4.0f ),
        "setting concentration must initialize mass from actual cell volume" );
    field.setTracerConcentration( -1, 10, 9.0f );
    field.setTracerConcentration( 12, 10, -1.0f );
    field.setTracerConcentration( 12, 10, std::numeric_limits<float>::infinity() );
    check( close( field.getTracerMass(), 1.0f ), "invalid tracer writes must not alter mass" );

    field.addVelocity( { 0.25f, 0.25f }, { 8.0f, 5.0f }, 2.0f );
    field.step( 0.12f, 100 );
    check( field.getTracerStats().substeps >= 2 && !field.getTracerStats().cflLimited,
        "tracer transport must substep for high face CFL" );
    for( int i = 0; i < 40; ++i ) field.step( dt, 100 );
    check( close( field.getTracerMass(), 1.0f, 1.0e-4f ) &&
        std::abs( field.getTracerStats().delta() ) < 1.0e-4f,
        "paired internal tracer fluxes must conserve total mass on a closed grid" );
    float minConcentration = 0.0f;
    int occupied = 0;
    for( int y = 0; y < field.getHeight(); ++y )
        for( int x = 0; x < field.getWidth(); ++x )
        {
            const float concentration = field.getTracerConcentration( x, y );
            check( std::isfinite( concentration ) && concentration >= -1.0e-5f,
                "mass-limited transport must keep finite nonnegative tracer density" );
            minConcentration = std::min( minConcentration, concentration );
            if( concentration > 1.0e-5f ) ++occupied;
        }
    check( occupied > 1 && minConcentration >= -1.0e-5f,
        "velocity transport must move tracer from its initial cell" );

    fluidField2 cut{ 32, 32, 0.5f, { -8.0f, -8.0f } };
    cut.setCutCellProjectionEnabled( true );
    fluidCircleObstacle2 obstacle{ { 0.0f, 0.0f }, 0.8f, {}, 0.0f };
    std::array<fluidObstacleForce2, 1> feedback{};
    cut.step( dt, std::span<const fluidCircleObstacle2>( &obstacle, 1 ), feedback );
    const float fraction = cut.getCellFluidFraction( 14, 15 );
    check( fraction > 0.0f && fraction < 1.0f, "test requires a partial cut cell" );
    cut.setTracerConcentration( 14, 15, 3.0f );
    check( close( cut.getTracerMass(), 3.0f * 0.25f * fraction ) &&
        close( cut.getTracerConcentration( 14, 15 ), 3.0f ),
        "tracer mass must respect fractional fluid volume" );
    cut.addVelocity( { -1.0f, -0.4f }, { 3.0f, 0.6f }, 1.5f );
    for( int i = 0; i < 15; ++i )
        cut.step( dt, std::span<const fluidCircleObstacle2>( &obstacle, 1 ), feedback );
    check( close( cut.getTracerMass(), 3.0f * 0.25f * fraction, 1.0e-4f ) &&
        std::abs( cut.getTracerStats().delta() ) < 1.0e-4f,
        "static cut-cell open faces must exchange equal tracer mass" );

    // Moving cut-cell의 swept interface transport는 미구현. 질량을 조용히 버리지 않고
    // 일시적으로 막힌 셀 안에 보관하며 trappedMass로 이 한계를 드러낸다.
    fluidField2 moving{ 20, 20, 1.0f };
    moving.setCutCellProjectionEnabled( true );
    moving.setTracerConcentration( 7, 7, 2.0f );
    moving.step( dt );
    fluidCircleObstacle2 cover{ { 7.5f, 7.5f }, 1.0f, {}, 0.0f };
    moving.step( dt, std::span<const fluidCircleObstacle2>( &cover, 1 ), feedback );
    check( close( moving.getTracerMass(), 2.0f ) &&
        close( moving.getTracerConcentration( 7, 7 ), 0.0f ) &&
        close( moving.getTracerStats().trappedMass, 2.0f ),
        "covered cells must report trapped tracer mass rather than silently delete it" );
    cover.center = { 11.5f, 7.5f };
    moving.step( dt, std::span<const fluidCircleObstacle2>( &cover, 1 ), feedback );
    check( close( moving.getTracerMass(), 2.0f ) &&
        moving.getTracerConcentration( 7, 7 ) >= 0.0f &&
        close( moving.getTracerStats().trappedMass, 0.0f ),
        "exposed cells must release the trapped category without losing total mass" );
    moving.clear();
    check( close( moving.getTracerMass(), 0.0f ) && moving.getTracerStats().substeps == 0,
        "clear must discard tracer state and diagnostics" );
}


void checkSweptTracerRemap()
{
    constexpr float dt = 1.0f / 60.0f;
    std::array<fluidObstacleForce2, 1> feedback{};
    const auto makeBox = []()
    {
        return fluidShapeObstacle2{ MakeBox( { 0.8f, 0.3f } ),
            { { -1.0f, -0.25f }, {} }, { -1.0f, -0.25f }, {}, 0.0f };
    };

    // 같은 이동 경로: 12a 기본값은 가려진 cell에 질량이 갇힌다.
    fluidField2 legacy{ 16, 16, 0.5f, { -4.0f, -4.0f } };
    legacy.setCutCellProjectionEnabled( true );
    auto oldBox = makeBox();
    legacy.step( dt, std::span<const fluidShapeObstacle2>( &oldBox, 1 ), feedback );
    check( close( legacy.getCellFluidFraction( 8, 7 ), 1.0f ),
        "baseline tracer source must be fluid before moving obstacle" );
    legacy.setTracerConcentration( 8, 7, 4.0f );
    oldBox.transform.position.x += 1.25f;
    oldBox.center.x += 1.25f;
    legacy.step( dt, std::span<const fluidShapeObstacle2>( &oldBox, 1 ), feedback, 160 );
    check( legacy.getSweptFaceFlux().usedSweptFlux &&
        close( legacy.getTracerStats().trappedMass, 1.0f, 1.0e-4f ) &&
        !legacy.getTracerStats().usedSweptRemap,
        "default mode must keep Part 12a trapped tracer behavior" );

    fluidField2 moving{ 16, 16, 0.5f, { -4.0f, -4.0f } };
    moving.setCutCellProjectionEnabled( true );
    moving.setSweptTracerRemapEnabled( true );
    auto box = makeBox();
    moving.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), feedback );
    moving.setTracerConcentration( 8, 7, 4.0f );
    box.transform.position.x += 1.25f;
    box.center.x += 1.25f;
    moving.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), feedback, 160 );
    const fluidTracerStats2 moved = moving.getTracerStats();
    check( moved.usedSweptRemap && moved.sweptRemappedMass + moved.fallbackRemappedMass > 0.9f &&
        moved.sweptRequestedMass >= moved.sweptRemappedMass - 1.0e-5f &&
        moved.unresolvedDisplacementMass < 1.0e-5f &&
        moved.trappedMass < 1.0e-5f && close( moving.getTracerMass(), 1.0f, 1.0e-4f ) &&
        std::abs( moved.delta() ) < 1.0e-4f,
        "swept-guided remap must evacuate covered tracer and preserve stored mass" );
    for( int y = 0; y < moving.getHeight(); ++y )
        for( int x = 0; x < moving.getWidth(); ++x )
            check( std::isfinite( moving.getTracerConcentration( x, y ) ) &&
                moving.getTracerConcentration( x, y ) >= -1.0e-5f,
                "swept remap and advection must keep concentrations nonnegative" );

    // 선두 edge를 일부만 지난 경우 이웃의 개방 공간으로 직접 swept 방향 이송.
    fluidField2 direct{ 16, 16, 0.5f, { -4.0f, -4.0f } };
    direct.setCutCellProjectionEnabled( true );
    direct.setSweptTracerRemapEnabled( true );
    auto leadingBox = makeBox();
    direct.step( dt, std::span<const fluidShapeObstacle2>( &leadingBox, 1 ), feedback );
    direct.setTracerConcentration( 9, 7, 4.0f );
    // 이동 끝의 열린 partial face에서 마지막 시간 표본까지 swept 유량이 생기도록 설정.
    leadingBox.transform.position.x += 1.5f;
    leadingBox.center.x += 1.5f;
    direct.step( dt, std::span<const fluidShapeObstacle2>( &leadingBox, 1 ), feedback, 160 );
    const fluidTracerStats2& directStats = direct.getTracerStats();
    check( directStats.sweptRemappedMass > 0.0f &&
        close( direct.getTracerMass(), 1.0f, 1.0e-4f ) &&
        directStats.trappedMass < 1.0e-5f,
        "swept face outward flux must guide direct conservative mass transfer to the open neighbor" );
    const float displaced = 1.0f - direct.getCellFluidFraction( 9, 7 );
    check( directStats.sweptRequestedMass >= directStats.sweptRemappedMass - 1.0e-5f &&
        close( directStats.sweptLimitedMass,
            std::max( 0.0f, directStats.sweptRequestedMass - displaced ), 1.0e-4f ) &&
        close( directStats.sweptRemappedMass + directStats.fallbackRemappedMass +
            directStats.unresolvedDisplacementMass, displaced, 1.0e-4f ),
        "integrated swept volume must bound physical tracer transfer and expose uncovered displacement" );

    moving.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), feedback );
    check( moving.getTracerStats().usedSweptRemap &&
        close( moving.getTracerStats().sweptRemappedMass, 0.0f ) &&
        close( moving.getTracerStats().fallbackRemappedMass, 0.0f ),
        "stationary shape with valid history must not move tracer through fake swept transport" );

    // 이동 대신 순수 회전으로 새로 가려진 partial fluid cell의 질량을 보정.
    fluidField2 rotating{ 32, 32, 0.25f, { -4.0f, -4.0f } };
    rotating.setCutCellProjectionEnabled( true );
    rotating.setSweptTracerRemapEnabled( true );
    fluidShapeObstacle2 rotor{ MakeBox( { 1.0f, 0.2f } ), { {}, {} }, {}, {}, 0.0f };
    rotating.step( dt, std::span<const fluidShapeObstacle2>( &rotor, 1 ), feedback );
    rotating.setTracerConcentration( 15, 18, 1.0f );
    const float initialMass = rotating.getTracerMass();
    rotor.transform.rotation = rot2::FromRadians( 0.5f * std::numbers::pi_v<float> );
    rotating.step( dt, std::span<const fluidShapeObstacle2>( &rotor, 1 ), feedback, 160 );
    check( rotating.getSweptFaceFlux().usedSweptFlux &&
        rotating.getTracerStats().usedSweptRemap &&
        rotating.getTracerStats().sweptRemappedMass + rotating.getTracerStats().fallbackRemappedMass > 0.0f &&
        close( rotating.getTracerMass(), initialMass, 1.0e-4f ),
        "rotating obstacle must remap shrinking cut-cell tracer without changing total mass" );

    // 최초/변경된 geometry는 실제 이동 경로가 없으므로 opt-in이라도 실행하지 않는다.
    fluidField2 unknown{ 16, 16, 0.5f, { -4.0f, -4.0f } };
    unknown.setCutCellProjectionEnabled( true );
    unknown.setSweptTracerRemapEnabled( true );
    unknown.setTracerConcentration( 8, 7, 4.0f );
    auto createdBox = makeBox();
    createdBox.transform.position.x += 1.25f;
    createdBox.center.x += 1.25f;
    unknown.step( dt, std::span<const fluidShapeObstacle2>( &createdBox, 1 ), feedback );
    check( !unknown.getSweptFaceFlux().usedSweptFlux && !unknown.getTracerStats().usedSweptRemap &&
        close( unknown.getTracerMass(), 1.0f ),
        "first snapshot must not invent swept tracer remap or lose mass" );
    createdBox.geometry = MakeBox( { 1.0f, 0.3f } );
    unknown.step( dt, std::span<const fluidShapeObstacle2>( &createdBox, 1 ), feedback );
    check( !unknown.getTracerStats().usedSweptRemap && close( unknown.getTracerMass(), 1.0f ),
        "geometry edit must reject swept remap while preserving tracer mass" );

    moving.clear();
    check( moving.isSweptTracerRemapEnabled() && close( moving.getTracerMass(), 0.0f ) &&
        !moving.getTracerStats().usedSweptRemap,
        "clear must reset swept remap diagnostics while retaining the selected mode" );
}

void checkSweptGeometryCorrection()
{
    constexpr float dt = 1.0f / 60.0f;
    std::array<fluidObstacleForce2, 1> feedback{};
    fluidField2 baseline{ 16, 16, 0.5f, { -4.0f, -4.0f } };
    fluidField2 corrected{ 16, 16, 0.5f, { -4.0f, -4.0f } };
    baseline.setCutCellProjectionEnabled( true );
    corrected.setCutCellProjectionEnabled( true );
    corrected.setSweptGeometryCorrectionEnabled( true );
    corrected.setSweptTracerRemapEnabled( true );

    fluidShapeObstacle2 box{ MakeBox( { 0.8f, 0.3f } ),
        { { -1.0f, -0.25f }, {} }, { -1.0f, -0.25f }, {}, 0.0f };
    for( fluidField2* field : { &baseline, &corrected } )
    {
        field->step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), feedback, 160 );
        check( !field->getSweptFaceFlux().usedSweptFlux,
            "initial cut-cell snapshot must not use synthetic swept correction" );
    }

    corrected.setTracerConcentration( 8, 7, 4.0f );
    const float mass = corrected.getTracerMass();
    box.transform.position.x += 1.25f;
    box.center.x += 1.25f;
    baseline.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), feedback, 160 );
    corrected.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), feedback, 160 );
    const fluidSweptFaceFlux2& raw = baseline.getSweptFaceFlux();
    const fluidSweptFaceFlux2& balanced = corrected.getSweptFaceFlux();
    check( raw.usedSweptFlux && balanced.usedSweptFlux &&
        raw.geometryMismatch > 0.0f &&
        close( raw.geometryMismatch, balanced.rawGeometryMismatch, 1.0e-4f ) &&
        balanced.geometryMismatch <= balanced.rawGeometryMismatch + 1.0e-4f,
        "swept face correction must not increase local geometry mismatch relative to same raw path" );
    check( balanced.correctedFaces > 0 &&
        balanced.correctionAbsoluteFlux > 0.0f &&
        balanced.geometryMismatch < balanced.rawGeometryMismatch,
        "translation must cancel opposite cut-cell residuals by paired internal face corrections" );
    check( close( corrected.getTracerMass(), mass, 1.0e-4f ) &&
        std::abs( corrected.getTracerStats().delta() ) < 1.0e-4f &&
        std::isfinite( corrected.getCutCellPressure().divergenceAfter ),
        "corrected swept flux must remain usable by pressure and tracer transport without mass loss" );

    // 같은 장애물의 연속된 변환에서 정지 경로 및 기하 변경은 임의 유량을 만들지 않음.
    corrected.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), feedback, 160 );
    check( corrected.getSweptFaceFlux().usedSweptFlux &&
        corrected.getSweptFaceFlux().correctionAbsoluteFlux == 0.0f &&
        corrected.getSweptFaceFlux().rawGeometryMismatch == 0.0f,
        "static obstacle must not accumulate fictitious swept correction" );
    box.geometry = MakeBox( { 0.7f, 0.3f } );
    corrected.step( dt, std::span<const fluidShapeObstacle2>( &box, 1 ), feedback, 160 );
    check( !corrected.getSweptFaceFlux().usedSweptFlux &&
        corrected.getSweptFaceFlux().correctedFaces == 0,
        "geometry edits must invalidate swept correction" );
    corrected.clear();
    check( corrected.isSweptGeometryCorrectionEnabled() &&
        corrected.getSweptFaceFlux().correctedFaces == 0 &&
        close( corrected.getTracerMass(), 0.0f ),
        "clear must remove correction diagnostics while preserving experimental setting" );
}

} // namespace

int main()
{
    checkBoundsAndSampling();
    checkStepTimingProfiler();
    checkPressureProjection();
    checkCircleSolidBoundary();
    checkGeneralShapeObstacles();
    checkDiscreteMomentumExchange();
    checkPressureMomentumLedger();
    checkMovingSolidRemap();
    checkBoundedObstacleRasterAndReusablePressure();
    checkCoupledPressureConvergenceDiagnostics();
    checkCoupledPressureBoundaryFaceSynchronization();
    checkCoupledPressureEnergyAudit();
    checkFractionalCutCells();
    checkWeightedCutCellPressure();
    checkCutCellCoupledPressure();
    checkCutCellPredictedBoundarySynchronization();
    checkCoupledCutCellAdjointContinuity();
    checkCutCellSpatialQuadrature();
    checkAnalyticSingleCircleCutCell();
    checkAnalyticSharpPolygonCutCell();
    checkAnalyticSingleCapsuleCutCell();
    checkAnalyticRoundedPolygonCutCell();
    checkDisjointAnalyticMultiShapeCutCell();
    checkDisjointAnalyticOverlappingAabbs();
    checkAnalyticContainedCircleUnion();
    checkAnalyticPartialCircleUnion();
    checkAnalyticCutCellPressureImpulseBudget();
    checkAnalyticCutCellPreconditionerComparison();
    checkCachedIc0Differential();
    checkActiveCellIc0SweepDifferential();
    checkCachedSchurLaplacianDifferential();
    checkIndexedAnalyticBoundaryDifferential();
    checkIndexedAnalyticBoundaryDifferential( true );
    checkIndexedAnalyticBoundaryDifferential( false, true );
    checkIndexedAnalyticBoundarySyncCircleUnion();
    checkBoundedAnalyticFaceClippingDifferential();
    checkBoundedAnalyticCellAreaDifferential();
    checkBoundedAnalyticCellAreaDifferential( true );
    checkCutCellTemporalContinuity();
    checkSweptSolidFaceFlux();
    checkMovingOnlyAnalyticSweptDifferential();
    checkMovingOnlyAnalyticSweptDifferential( true );
    checkMovingOnlyAnalyticSweptDifferential( false, true );
    checkBoundedSweptMixedAnalyticShapes();
    checkBoundedSweptMixedAnalyticShapes( true );
    checkAdaptiveSweptTemporalQuadrature();
    checkConservativeMacAdvection();
    checkConservativeTracerTransport();
    checkSweptTracerRemap();
    checkSweptGeometryCorrection();
}
