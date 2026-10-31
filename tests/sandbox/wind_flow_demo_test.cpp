#include "rigidBodyDemo.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace zonai;
using namespace zonai::sandbox;

namespace
{

void check( bool condition, const char* message )
{
    if( condition ) return;
    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}

bool near( vec2 a, vec2 b, float tolerance = 1.0e-3f )
{
    return Length( a - b ) < tolerance;
}

bool close( float a, float b, float tolerance = 1.0e-4f )
{
    return std::abs( a - b ) <= tolerance;
}

void checkFluidCoupling()
{
    rigidBodyDemo scene{ demoKind::windFlow };
    const fluidField2* field = scene.getWindField();
    check( field != nullptr && field->getWidth() == 24 && field->getHeight() == 14,
        "wind demo must own its separate MAC grid" );
    check( scene.getShapes().size() == 3, "wind demo must show three dynamic rigid bodies" );
    check( std::holds_alternative<circle2>( scene.getWorld().GetShape( scene.getShapes()[0].shapeHandle ).geometry ) &&
        std::holds_alternative<capsule2>( scene.getWorld().GetShape( scene.getShapes()[1].shapeHandle ).geometry ) &&
        std::holds_alternative<polygon2>( scene.getWorld().GetShape( scene.getShapes()[2].shapeHandle ).geometry ),
        "wind demo must exercise circle, capsule and convex polygon together" );
    check( near( scene.getWorld().GetGravity(), {} ), "wind trial should initially be zero gravity" );

    const bodyId first = scene.getShapes()[0].bodyHandle;
    const vec2 initial = scene.getWorld().GetBodyTransform( first ).position;

    // 단방향 coupling이 없으면 유체만 움직이고 Body는 정지해야 함.
    scene.setWindObstacleCouplingEnabled( false );
    scene.setWindDragCoefficient( 0.0f );
    for( int i = 0; i < 120; ++i ) scene.step( 1.0f / 60.0f, 4 );
    check( near( scene.getWorld().GetBodyTransform( first ).position, initial ),
        "zero drag rate must decouple the wind field from body motion" );

    const vec2 flow = field->sampleVelocity( { -3.0f, 0.0f } );
    check( IsFinite( flow ) && Length( flow ) > 1.0e-4f,
        "wind source must create an observable finite velocity field" );
    check( std::isfinite( field->getDivergenceRms() ) && field->getDivergenceRms() < 0.25f,
        "running wind demo must keep pressure projection residual bounded" );

    scene.resetWindFlow();
    check( field->getDivergenceRms() == 0.0f && near( scene.getWorld().GetBodyTransform( first ).position, initial ),
        "wind reset must clear velocity and restore body transforms" );

    // 소스가 꺼진 빈 유체는 강체를 가속하지 않아야 함.
    scene.setWindSourceEnabled( false );
    scene.setWindDragCoefficient( 3.0f );
    for( int i = 0; i < 60; ++i ) scene.step( 1.0f / 60.0f, 4 );
    check( near( scene.getWorld().GetBodyTransform( first ).position, initial ),
        "empty field with source disabled must leave bodies at rest" );

    scene.setWindSourceEnabled( true );
    scene.setWindSourceStrength( 25.0f );
    for( int i = 0; i < 300; ++i ) scene.step( 1.0f / 60.0f, 4 );
    check( Length( scene.getWorld().GetBodyTransform( first ).position - initial ) > 0.01f,
        "fluid drag must move a body in the flow" );

    scene.resetWindFlow();
    check( near( scene.getWorld().GetBodyTransform( first ).position, initial ) &&
        near( scene.getWorld().GetBodyLinearVelocity( first ), {} ) &&
        scene.getWindSourceStrength() == 25.0f && scene.getWindDragCoefficient() == 3.0f,
        "wind reset must restore motion while preserving edited parameters" );
}

void checkRotatedSandboxObstacles()
{
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.setWindSourceEnabled( false );
    scene.setWindDragCoefficient( 0.0f );
    const auto visuals = scene.getShapes();

    // 회전한 Capsule과 Polygon이 World에서 유체 solver까지 전달되는지 확인.
    for( std::size_t i : { 1u, 2u } )
    {
        const bodyId body = visuals[i].bodyHandle;
        const transform2 pose = scene.getWorld().GetBodyTransform( body );
        scene.getWorld().SetBodyTransform( body, { pose.position, rot2::FromRadians( 0.5f * 3.14159265358979323846f ) } );
    }
    scene.step( 1.0f / 60.0f, 4 );
    check( scene.getWindField()->getSolidCellCount() >= 5,
        "rotated capsule/polygon body geometry must enter the fluid solid occupancy" );
    for( const fluidObstacleForce2& pressure : scene.getWindPressureForces() )
        check( IsFinite( pressure.force ) && std::isfinite( pressure.torque ),
            "mixed rotated shapes in resting fluid must have finite pressure feedback" );
}

void checkMovingSolidReaction()
{
    rigidBodyDemo disabled{ demoKind::windFlow };
    rigidBodyDemo enabled{ demoKind::windFlow };
    const bodyId bodyDisabled = disabled.getShapes()[0].bodyHandle;
    const bodyId bodyEnabled = enabled.getShapes()[0].bodyHandle;

    for( rigidBodyDemo* scene : { &disabled, &enabled } )
    {
        scene->setWindSourceEnabled( false );
        scene->setWindDragCoefficient( 0.0f );
        scene->getWorld().SetBodyLinearVelocity( scene->getShapes()[0].bodyHandle, { 2.0f, 0.0f } );
    }
    disabled.setWindObstacleCouplingEnabled( false );
    enabled.setWindObstacleCouplingEnabled( true );
    // 이 검사는 1회 유체 압력 반력의 방향에 대한 기존 oracle이다.
    // Substep 평균 힘은 별도의 동기화 회귀에서 검증한다.
    enabled.setWindSynchronizedCouplingEnabled( false );

    disabled.step( 1.0f / 60.0f, 4 );
    enabled.step( 1.0f / 60.0f, 4 );

    check( disabled.getWindField()->getSolidCellCount() == 0 &&
        enabled.getWindField()->getSolidCellCount() > 0,
        "bidirectional toggle must control solid occupancy" );
    check( near( disabled.getWorld().GetBodyLinearVelocity( bodyDisabled ), { 2.0f, 0.0f } ),
        "disabled feedback must preserve momentum in an empty fluid" );
    check( enabled.getWindPressureForces()[0].force.x < -0.001f &&
        enabled.getWorld().GetBodyLinearVelocity( bodyEnabled ).x <
            disabled.getWorld().GetBodyLinearVelocity( bodyDisabled ).x,
        "a moving solid must stir fluid and receive opposite pressure force" );

    const vec2 wake = enabled.getWindField()->sampleVelocity( { -1.0f, 0.0f } );
    check( IsFinite( wake ) && Length( wake ) > 0.0001f,
        "moving solid must change the velocity field without a wind source" );

    enabled.resetWindFlow();
    check( enabled.getWindField()->getSolidCellCount() == 0 &&
        near( enabled.getWindPressureForces()[0].force, {} ) &&
        near( enabled.getWorld().GetBodyLinearVelocity( bodyEnabled ), {} ),
        "wind reset must clear pressure feedback and solid mask" );
    check( enabled.isWindObstacleCouplingEnabled(), "wind reset must preserve coupling setting" );
}

void checkDragMomentumBudget()
{
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.setWindSourceEnabled( false );
    scene.setWindDragCoefficient( 10.0f );
    const bodyId body = scene.getShapes()[0].bodyHandle;
    scene.getWorld().SetBodyLinearVelocity( body, { 2.0f, 0.0f } );

    // 압력 feedback은 별도 물리 과정. 여기서는 drag와 그 국소 유체 impulse 쌍만 검증함.
    scene.step( 1.0f / 60.0f, 4 );
    const vec2 bodyImpulse = scene.getWindDragBodyImpulse();
    const vec2 fluidImpulse = scene.getWindDragFluidImpulse();
    check( Length( bodyImpulse ) > 0.0001f && Length( fluidImpulse ) > 0.0001f,
        "moving body must exchange nonzero drag impulse with the fluid" );
    check( Length( scene.getWindDragExchangeResidual() ) < 1.0e-5f &&
        Length( bodyImpulse + fluidImpulse ) < 1.0e-5f,
        "bidirectional drag must deliver equal-and-opposite impulses" );

    scene.resetWindFlow();
    check( LengthSquared( scene.getWindDragBodyImpulse() ) == 0.0f &&
        LengthSquared( scene.getWindDragFluidImpulse() ) == 0.0f,
        "reset must clear per-step drag exchange diagnostics" );

    scene.setWindObstacleCouplingEnabled( false );
    scene.getWorld().SetBodyLinearVelocity( body, { 2.0f, 0.0f } );
    scene.step( 1.0f / 60.0f, 4 );
    check( Length( scene.getWindDragBodyImpulse() ) > 0.0001f &&
        LengthSquared( scene.getWindDragFluidImpulse() ) == 0.0f,
        "one-way mode must not deposit the reaction impulse into fluid" );

    scene.setWindObstacleCouplingEnabled( true );
    scene.resetWindFlow();
    scene.setWindDragCoefficient( 0.0f );
    scene.getWorld().SetBodyLinearVelocity( body, { 2.0f, 0.0f } );
    scene.step( 1.0f / 60.0f, 4 );
    check( LengthSquared( scene.getWindDragBodyImpulse() ) == 0.0f &&
        LengthSquared( scene.getWindDragFluidImpulse() ) == 0.0f &&
        Length( scene.getWindPressureForces()[0].force ) > 0.01f,
        "pressure reaction must remain independent when drag coefficient is zero" );
}

void checkSynchronizedWindCoupling()
{
    constexpr float dt = 1.0f / 60.0f;
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.setWindSourceEnabled( false );
    scene.setWindDragCoefficient( 0.0f );
    scene.setWindObstacleCouplingEnabled( true );
    check( scene.isWindSynchronizedCouplingEnabled(),
        "wind sandbox must enable synchronized pressure coupling by default" );

    const bodyId moving = scene.getShapes()[0].bodyHandle;
    scene.getWorld().SetBodyLinearVelocity( moving, { 2.0f, 0.0f } );
    scene.step( dt, 4 );
    check( scene.getWindCouplingSteps() == 4 &&
        scene.getWindField()->getSolidCellCount() > 0 &&
        IsFinite( scene.getWorld().GetBodyLinearVelocity( moving ) ) &&
        IsFinite( scene.getWindPressureForces()[0].force ),
        "bidirectional wind must advance fluid and rigid geometry on matching substep clocks" );

    scene.resetWindFlow();
    scene.setWindSynchronizedCouplingEnabled( false );
    scene.getWorld().SetBodyLinearVelocity( moving, { 2.0f, 0.0f } );
    scene.step( dt, 4 );
    check( scene.getWindCouplingSteps() == 1 &&
        IsFinite( scene.getWindPressureForces()[0].force ),
        "legacy one-exchange wind mode must remain reproducible" );

    scene.resetWindFlow();
    scene.setWindSynchronizedCouplingEnabled( true );
    scene.setWindObstacleCouplingEnabled( false );
    scene.step( dt, 4 );
    check( scene.getWindCouplingSteps() == 1 &&
        scene.getWindField()->getSolidCellCount() == 0,
        "one-way wind should keep legacy World substeps without fluid obstacle feedback" );

    scene.resetWindFlow();
    scene.setWindObstacleCouplingEnabled( true );
    scene.getWorld().SetBodyLinearVelocity( moving, { 2.0f, 0.0f } );
    float maxSpeed = 0.0f;
    for( int i = 0; i < 120; ++i )
    {
        scene.step( dt, 4 );
        const vec2 velocity = scene.getWorld().GetBodyLinearVelocity( moving );
        check( IsFinite( velocity ) && IsFinite( scene.getWindPressureForces()[0].force ) &&
            std::isfinite( scene.getWindPressureForces()[0].torque ),
            "synchronized moving obstacle must keep finite velocity and pressure feedback over 120 frames" );
        maxSpeed = std::max( maxSpeed, Length( velocity ) );
        if( maxSpeed >= 20.0f )
        {
            const vec2 pressure = scene.getWindPressureForces()[0].force;
            std::fprintf( stderr, "wind coupled instability: frame=%d speed=%.5f body=(%.5f,%.5f) pressure=(%.5f,%.5f)\n",
                i, maxSpeed, velocity.x, velocity.y, pressure.x, pressure.y );
            check( false, "synchronized pressure feedback must not accelerate a released body without wind source or drag without bound" );
        }
    }
}

void checkPressureLedgerInSandbox()
{
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.setWindSourceEnabled( false );
    scene.setWindSynchronizedCouplingEnabled( false );
    scene.setWindDragCoefficient( 0.0f );
    const bodyId body = scene.getShapes()[0].bodyHandle;
    scene.getWorld().SetBodyLinearVelocity( body, { 2.0f, 0.0f } );
    scene.step( 1.0f / 60.0f, 4 );

    const fluidPressureMomentum2& budget = scene.getWindField()->getPressureMomentum();
    const fluidStepMomentum2& stages = scene.getWindField()->getStepMomentum();
    const float scale = 1.0f + Length( budget.fluid ) + Length( budget.solid ) + Length( budget.wall );
    check( Length( budget.solid ) > 1.0e-5f &&
        Length( budget.residual() ) < 5.0e-4f * scale,
        "wind demo moving rigid body must close the fluid/solid/wall pressure momentum budget" );

    vec2 forceSum{};
    for( const fluidObstacleForce2& pressure : scene.getWindPressureForces() )
        forceSum += pressure.force;
    check( Length( budget.solid - forceSum * ( 1.0f / 60.0f ) ) < 1.0e-4f * scale &&
        Length( stages.pressureDelta() - budget.fluid ) < 1.0e-4f * scale,
        "Sandbox applied pressure reaction must match core pressure ledger" );

    scene.resetWindFlow();
    check( LengthSquared( scene.getWindField()->getPressureMomentum().residual() ) == 0.0f &&
        LengthSquared( scene.getWindField()->getStepMomentum().afterPressure ) == 0.0f,
        "wind reset must clear core pressure diagnostics" );

    scene.setWindObstacleCouplingEnabled( false );
    scene.step( 1.0f / 60.0f, 4 );
    check( LengthSquared( scene.getWindField()->getPressureMomentum().solid ) == 0.0f,
        "one-way mode must not report rigid pressure reaction" );
}

void checkExperimentalCutCellProjection()
{
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.setWindSourceEnabled( false );
    scene.setWindDragCoefficient( 0.0f );
    scene.setWindCutCellProjectionEnabled( true );
    check( scene.isWindCutCellProjectionEnabled(), "wind sandbox must enable weighted cut-cell pressure" );

    const bodyId body = scene.getShapes()[0].bodyHandle;
    scene.getWorld().SetBodyLinearVelocity( body, { 1.5f, 0.0f } );
    scene.step( 1.0f / 60.0f, 4 );
    const fluidCutCellPressure2& stats = scene.getWindField()->getCutCellPressure();
    check( stats.activeCells > 0 && std::isfinite( stats.divergenceBefore ) &&
        std::isfinite( stats.divergenceAfter ) && std::isfinite( stats.sourceImbalance ) &&
        LengthSquared( scene.getWindField()->getPressureMomentum().residual() ) == 0.0f,
        "cut-cell sandbox must execute weighted projection without using legacy pressure ledger" );

    scene.resetWindFlow();
    check( scene.isWindCutCellProjectionEnabled() &&
        scene.getWindField()->getCutCellPressure().activeCells == 0,
        "reset must clear weighted pressure diagnostics without disabling experiment" );
    scene.setWindCutCellProjectionEnabled( false );
    scene.step( 1.0f / 60.0f, 4 );
    check( !scene.isWindCutCellProjectionEnabled() &&
        scene.getWindField()->getCutCellPressure().activeCells == 0,
        "sandbox must restore binary pressure mode when experiment is disabled" );
}

void checkSandboxConservativeAdvectionTrial()
{
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.setWindConservativeAdvectionEnabled( true );
    scene.setWindCutCellProjectionEnabled( false );
    scene.beginWindAdvectionTrial();
    check( scene.isWindConservativeAdvectionEnabled() &&
        !scene.isWindSourceEnabled() && !scene.isWindObstacleCouplingEnabled() &&
        scene.getWindDragCoefficient() == 0.0f,
        "advection trial must isolate a repeatable vortex from source, drag and rigid coupling" );

    const fluidField2* field = scene.getWindField();
    scene.seedWindTracer();
    const float tracerMass = field->getTracerMass();
    check( close( tracerMass, 2.0f ), "sandbox must seed a reproducible tracer patch" );
    const vec2 seeded = field->sampleVelocity( { 1.5f, 0.0f } );
    check( IsFinite( seeded ) && Length( seeded ) > 0.05f,
        "advection trial must seed observable fluid circulation before the first step" );

    for( int i = 0; i < 50; ++i )
    {
        scene.step( 1.0f / 60.0f, 4 );
        const fluidAdvectionStats2& adv = field->getAdvectionStats();
        check( adv.conservative && adv.substeps >= 1 && !adv.cflLimited &&
            IsFinite( adv.before ) && IsFinite( adv.after ) &&
            Length( adv.delta() ) < 3.0e-4f * ( 1.0f + Length( adv.before ) ) &&
            std::isfinite( field->getDivergenceRms() ),
            "sandbox isolated vortex must preserve transport face momentum without pressure contamination" );
        check( close( field->getTracerMass(), tracerMass, 1.0e-4f ) &&
            std::abs( field->getTracerStats().delta() ) < 1.0e-4f,
            "sandbox vortex must conserve the total tracer mass on the closed grid" );
    }

    scene.setWindConservativeAdvectionEnabled( false );
    scene.beginWindAdvectionTrial();
    check( close( field->getTracerMass(), 0.0f ) && field->getTracerStats().substeps == 0,
        "sandbox advection trial reset must clear tracer state" );
    scene.step( 1.0f / 60.0f, 4 );
    check( !field->getAdvectionStats().conservative && field->getAdvectionStats().substeps == 1 &&
        IsFinite( field->getAdvectionStats().after ),
        "same sandbox preset must run with the legacy semi-Lagrangian method" );

    scene.setWindConservativeAdvectionEnabled( true );
    scene.setWindCutCellProjectionEnabled( true );
    scene.setWindSweptTracerRemapEnabled( true );
    scene.setWindSweptGeometryCorrectionEnabled( true );
    scene.setWindObstacleCouplingEnabled( true );
    scene.resetWindFlow();
    check( scene.isWindSweptTracerRemapEnabled() &&
        scene.isWindSweptGeometryCorrectionEnabled() && !field->getTracerStats().usedSweptRemap,
        "sandbox reset must preserve swept tracer remap selection and clear last-step diagnostics" );
    scene.step( 1.0f / 60.0f, 4 );
    check( field->getAdvectionStats().conservative &&
        field->getCutCellPressure().activeCells > 0 &&
        !field->getTracerStats().usedSweptRemap &&
        std::isfinite( field->getCutCellPressure().pressureResidual ),
        "conservative advection must also integrate with moving-solid cut-cell wind sandbox" );

    // 장면의 실제 Circle transform을 바꾸어 연속된 강체 snapshot의 swept remap 경로를 검사.
    scene.seedWindTracer();
    const float tracerBefore = field->getTracerMass();
    const bodyId circleBody = scene.getShapes()[0].bodyHandle;
    scene.getWorld().SetBodyTransform( circleBody, { { -2.75f, -0.25f }, {} } );
    scene.step( 1.0f / 60.0f, 4 );
    check( field->getSweptFaceFlux().usedSweptFlux &&
        field->getSweptFaceFlux().geometryMismatch <=
            field->getSweptFaceFlux().rawGeometryMismatch + 1.0e-4f &&
        field->getTracerStats().usedSweptRemap &&
        close( field->getTracerMass(), tracerBefore, 1.0e-4f ) &&
        std::isfinite( field->getTracerStats().unresolvedDisplacementMass ),
        "sandbox moving Circle must run swept-guided tracer remap without mass loss" );

    scene.resetWindFlow();
    check( scene.isWindConservativeAdvectionEnabled() &&
        field->getAdvectionStats().substeps == 0,
        "wind reset must preserve algorithm choice and clear its diagnostics" );
}

} // namespace

int main()
{
    checkFluidCoupling();
    checkMovingSolidReaction();
    checkSynchronizedWindCoupling();
    checkRotatedSandboxObstacles();
    checkDragMomentumBudget();
    checkPressureLedgerInSandbox();
    checkExperimentalCutCellProjection();
    checkSandboxConservativeAdvectionTrial();
}
