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

} // namespace

int main()
{
    checkBoundsAndSampling();
    checkPressureProjection();
    checkCircleSolidBoundary();
    checkGeneralShapeObstacles();
    checkDiscreteMomentumExchange();
    checkPressureMomentumLedger();
    checkMovingSolidRemap();
    checkFractionalCutCells();
    checkWeightedCutCellPressure();
    checkCutCellTemporalContinuity();
    checkSweptSolidFaceFlux();
}
