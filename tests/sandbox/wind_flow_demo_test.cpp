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
    check( scene.isWindSynchronizedCouplingEnabled() && scene.getWindFluidSubstepCount() == 1,
        "mass-aware wind coupling must default to one expensive Fluid step independent of World substeps" );
    scene.setWindFluidSubstepCount( 4 );

    const bodyId moving = scene.getShapes()[0].bodyHandle;
    scene.getWorld().SetBodyLinearVelocity( moving, { 2.0f, 0.0f } );
    scene.step( dt, 4 );
    check( scene.getWindCouplingSteps() == 4 &&
        scene.getWindField()->getPressureMomentum().usedCoupledProjection &&
        !scene.getWindField()->getPressureMomentum().coupledSolveFailed &&
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
    scene.setWindCutCellProjectionEnabled( true );
    scene.step( dt, 4 );
    check( scene.getWindCouplingSteps() == 1 &&
        !scene.getWindField()->getPressureMomentum().usedCoupledProjection,
        "Cut-cell pressure must not use unsupported explicit synchronized subcycling" );

    scene.resetWindFlow();
    scene.setWindCutCellProjectionEnabled( false );
    scene.getWorld().SetBodyLinearVelocity( moving, { 2.0f, 0.0f } );
    scene.step( dt, 1 );
    check( scene.getWindCouplingSteps() == 1 &&
        scene.getWindField()->getPressureMomentum().usedCoupledProjection,
        "mass-aware pressure must remain active even with one rigid substep" );
    scene.resetWindFlow();
    scene.getWorld().SetBodyLinearVelocity( moving, { 2.0f, 0.0f } );
    float maxSpeed = 0.0f;
    for( int i = 0; i < 120; ++i )
    {
        scene.step( dt, 4 );
        const vec2 velocity = scene.getWorld().GetBodyLinearVelocity( moving );
        const fluidPressureMomentum2& pressure = scene.getWindField()->getPressureMomentum();
        check( pressure.usedCoupledProjection &&
            pressure.coupledStatus == fluidCoupledPressureStatus2::converged &&
            !pressure.coupledSolveFailed &&
            pressure.coupledRelativeResidual <= 1.0001e-5f &&
            IsFinite( velocity ) && IsFinite( scene.getWindPressureForces()[0].force ) &&
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

void checkPressurePredictedWorldVelocity()
{
    constexpr float dt = 1.0f / 60.0f;
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.setWindSourceEnabled( false );
    scene.setWindDragCoefficient( 0.0f );

    // 중력/Drag/Contact가 없는 단일 World Step에서 실제 속도 증분은
    // Fluid가 반환한 압력 반력 dt*F/m, dt*tau/I와 같아야 한다.
    const bodyId body = scene.getShapes()[2].bodyHandle;
    auto& world = scene.getWorld();
    transform2 pose = world.GetBodyTransform( body );
    pose.rotation = rot2::FromRadians( 0.4f );
    world.SetBodyTransform( body, pose );
    world.SetBodyLinearVelocity( body, { 1.1f, -0.7f } );
    world.SetBodyAngularVelocity( body, 0.8f );

    const vec2 initialLinear = world.GetBodyLinearVelocity( body );
    const float initialAngular = world.GetBodyAngularVelocity( body );
    const float inverseMass = 1.0f / world.GetBodyMass( body );
    const float inverseInertia = 1.0f / world.GetBodyRotationalInertia( body );
    scene.step( dt, 1 );

    const fluidPressureMomentum2& pressure = scene.getWindField()->getPressureMomentum();
    check( pressure.usedCoupledProjection &&
        pressure.coupledStatus == fluidCoupledPressureStatus2::converged &&
        pressure.synchronizedBoundaryFaces > 0,
        "World and Fluid test requires converged pressure and synchronized MAC boundaries" );
    const fluidObstacleForce2& force = scene.getWindPressureForces()[2];
    const vec2 expectedLinear = initialLinear + ( dt * inverseMass ) * force.force;
    const float expectedAngular = initialAngular + dt * inverseInertia * force.torque;
    check( near( world.GetBodyLinearVelocity( body ), expectedLinear, 1.0e-3f ) &&
        close( world.GetBodyAngularVelocity( body ), expectedAngular, 1.0e-3f ),
        "without drag, gravity or contacts, World velocity must match Fluid pressure-predicted body velocity" );
    double actualRigidEnergy = 0.0;
    for( const visualShape& visual : scene.getShapes() )
    {
        const double mass = world.GetBodyMass( visual.bodyHandle );
        const double inertia = world.GetBodyRotationalInertia( visual.bodyHandle );
        const vec2 linearVelocity = world.GetBodyLinearVelocity( visual.bodyHandle );
        const double angularVelocity = world.GetBodyAngularVelocity( visual.bodyHandle );
        actualRigidEnergy += 0.5 * mass * Dot( linearVelocity, linearVelocity ) +
            0.5 * inertia * angularVelocity * angularVelocity;
    }
    check( std::abs( actualRigidEnergy - pressure.energy.rigidAfter ) <
        1.0e-3 * ( 1.0 + actualRigidEnergy ) &&
        std::abs( scene.getWindField()->getFluidKineticEnergy() - pressure.energy.fluidAfter ) <
            1.0e-6 * ( 1.0 + pressure.energy.fluidAfter ),
        "isolated World pressure response must match predicted body energy and measured free-fluid face energy" );
}

void checkNoSourceWindEnergyEnvelope()
{
    constexpr float dt = 1.0f / 60.0f;
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.setWindSourceEnabled( false );
    scene.setWindDragCoefficient( 0.0f );
    scene.setWindFluidSubstepCount( 1 );
    const bodyId moving = scene.getShapes()[0].bodyHandle;
    auto& world = scene.getWorld();
    world.SetBodyLinearVelocity( moving, { 1.8f, 0.4f } );
    world.SetBodyAngularVelocity( moving, 0.7f );

    const auto energy = [&]
    {
        double result = scene.getWindField()->getFluidKineticEnergy();
        for( const visualShape& shape : scene.getShapes() )
        {
            const vec2 v = world.GetBodyLinearVelocity( shape.bodyHandle );
            const double w = world.GetBodyAngularVelocity( shape.bodyHandle );
            result += 0.5 * world.GetBodyMass( shape.bodyHandle ) * Dot( v, v ) +
                0.5 * world.GetBodyRotationalInertia( shape.bodyHandle ) * w * w;
        }
        return result;
    };

    const double initial = energy();
    double peak = initial;
    double cumulativePositivePressure = 0.0;
    float peakMomentumResidual = 0.0f;
    for( int frame = 0; frame < 60; ++frame )
    {
        scene.step( dt, 8 );
        const fluidPressureMomentum2& pressure = scene.getWindField()->getPressureMomentum();
        check( pressure.usedCoupledProjection && !pressure.coupledSolveFailed,
            "no-source World8/Fluid1 energy trial requires converged coupled pressure" );
        const float magnitude = Length( pressure.fluid ) + Length( pressure.solid ) + Length( pressure.wall );
        peakMomentumResidual = std::max( peakMomentumResidual,
            Length( pressure.residual() ) / ( 1.0f + magnitude ) );
        peak = std::max( peak, energy() );
        cumulativePositivePressure += std::max( 0.0, pressure.energy.totalDelta() );
        check( std::isfinite( peak ) && std::isfinite( cumulativePositivePressure ) &&
            peak < 10.0 * ( 1.0 + initial ) &&
            peakMomentumResidual < 5.0e-4f,
            "source-free moving World must avoid explosive total kinetic energy and bound discrete pressure momentum error" );
    }
    std::printf( "Wind moving geometry energy: initial=%.6f peak=%.6f final=%.6f positivePressureDelta=%.6f maxMomentumError=%.3g\n",
        initial, peak, energy(), cumulativePositivePressure, peakMomentumResidual );
}

void checkFluidRateAndStaticObstacles()
{
    constexpr float dt = 1.0f / 60.0f;
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.setWindSourceEnabled( false );
    scene.setWindDragCoefficient( 0.0f );
    auto& world = scene.getWorld();
    const bodyId dynamicBody = scene.getShapes()[0].bodyHandle;
    world.SetBodyLinearVelocity( dynamicBody, { 2.0f, 0.0f } );
    scene.step( dt, 8 );
    check( scene.getWindCouplingSteps() == 1 &&
        scene.getWindField()->getPressureMomentum().usedCoupledProjection,
        "World substeps must not force additional Fluid projection when fluid rate is one" );
    for( int frame = 0; frame < 120; ++frame )
    {
        scene.step( dt, 8 );
        const vec2 velocity = world.GetBodyLinearVelocity( dynamicBody );
        const fluidPressureMomentum2& pressure = scene.getWindField()->getPressureMomentum();
        check( scene.getWindCouplingSteps() == 1 &&
            pressure.usedCoupledProjection &&
            pressure.coupledStatus == fluidCoupledPressureStatus2::converged &&
            !pressure.coupledSolveFailed &&
            pressure.coupledRelativeResidual <= 1.0001e-5f &&
            IsFinite( velocity ) && Length( velocity ) < 20.0f,
            "one Fluid step with eight World substeps must avoid added-mass speed runaway" );
    }
    scene.resetWindFlow();
    scene.setWindFluidSubstepCount( 2 );
    world.SetBodyLinearVelocity( dynamicBody, { 2.0f, 0.0f } );
    scene.step( dt, 8 );
    check( scene.getWindCouplingSteps() == 2 &&
        scene.getWindField()->getPressureMomentum().usedCoupledProjection,
        "two Fluid exchanges must group eight World substeps without disabling coupled pressure" );
    scene.setWindFluidSubstepCount( 32 );
    scene.step( dt, 3 );
    check( scene.getWindCouplingSteps() == 3,
        "Fluid exchange count must never exceed the World substeps in one external step" );
    scene.setWindFluidSubstepCount( 1 );
    scene.resetWindFlow();

    const shapeId fixedBox = scene.addWindStaticBox( { -3.0f, 0.0f }, { 0.45f, 0.4f } );
    const shapeId fixedCircle = scene.addWindStaticCircle( { 1.5f, -1.0f }, 0.65f );
    check( world.IsValid( fixedBox ) && world.IsValid( fixedCircle ) &&
        scene.getWindStaticObstacleCount() == 2 && scene.getShapes().size() == 5,
        "Static wind objects must be created as real World shapes" );
    const bodyId fixedBody = scene.getShapes()[3].bodyHandle;
    check( world.GetBody( fixedBody ).type == bodyType::Static,
        "Static wind obstacle must use World::Static for collision and mass behaviour" );
    const vec2 fixedPosition = world.GetBodyTransform( fixedBody ).position;
    scene.step( dt, 8 );
    check( scene.getWindField()->isSolidCell( 6, 7 ) &&
        scene.getWindField()->isSolidCell( 15, 5 ) &&
        near( world.GetBodyTransform( fixedBody ).position, fixedPosition ),
        "both static box and circle must block fluid and remain unmoved" );
    scene.setWindObstacleCouplingEnabled( false );
    scene.step( dt, 8 );
    check( scene.getWindField()->isSolidCell( 6, 7 ) &&
        scene.getWindField()->isSolidCell( 15, 5 ) &&
        scene.getWindField()->getSolidCellCount() > 0,
        "static walls must remain fluid barriers when dynamic bidirectional coupling is disabled" );
    scene.setWindObstacleCouplingEnabled( true );

    scene.setWindConservativeAdvectionEnabled( true );
    scene.setWindFluidSubstepCount( 2 );
    scene.configureWindField( 32, 20, 0.5f );
    check( scene.getWindField()->getWidth() == 32 &&
        scene.getWindField()->getHeight() == 20 &&
        close( scene.getWindField()->getCellSize(), 0.5f ) &&
        scene.getWindField()->isConservativeAdvectionEnabled() &&
        scene.getWindStaticObstacleCount() == 2 &&
        near( world.GetBodyTransform( fixedBody ).position, fixedPosition ),
        "resizing Fluid Field must retain static World geometry and mode selection" );
    scene.step( dt, 4 );
    check( scene.getWindCouplingSteps() == 2 &&
        scene.getWindField()->isSolidCell( 10, 10 ),
        "resized Fluid grid must sample existing Static obstacles in new coordinates" );

    check( scene.removeWindStaticObstacle( fixedBox ) &&
        !world.IsValid( fixedBox ) && scene.getWindStaticObstacleCount() == 1,
        "removing Static obstacle must remove its collision shape from World" );
    scene.step( dt, 8 );
    check( !scene.getWindField()->isSolidCell( 10, 10 ) &&
        world.IsValid( fixedCircle ) &&
        !scene.removeWindStaticObstacle( scene.getShapes()[0].shapeHandle ),
        "removed static geometry must disappear from Fluid while dynamic objects remain protected" );

    scene.configureWindField( 16, 12, 0.25f );
    scene.setWindSourceEnabled( true );
    scene.step( dt, 8 );
    check( scene.getWindField()->getWidth() == 16 &&
        scene.getWindField()->getHeight() == 12 &&
        IsFinite( scene.getWindField()->sampleVelocity( { -1.0f, 0.0f } ) ),
        "small resized Fluid domain must accept wind sources without nonfinite results" );
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

void checkExperimentalCutCellWorldCoupling()
{
    constexpr float dt = 1.0f / 60.0f;
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.setWindSourceEnabled( false );
    scene.setWindDragCoefficient( 0.0f );
    scene.setWindCutCellProjectionEnabled( true );
    scene.setWindCutCellCoupledProjectionEnabled( true );
    scene.setWindFluidSubstepCount( 2 );
    check( scene.isWindCutCellCoupledProjectionEnabled() &&
        scene.isWindSynchronizedCouplingEnabled() &&
        scene.isWindObstacleCouplingEnabled(),
        "experimental cut-cell coupling should require an explicit opt-in and existing bidirectional clock" );

    const bodyId moving = scene.getShapes()[0].bodyHandle;
    scene.getWorld().SetBodyLinearVelocity( moving, { 1.1f, 0.4f } );
    scene.getWorld().SetBodyAngularVelocity( moving, 0.8f );
    scene.addWindVortex();
    scene.step( dt, 4 );
    const fluidCutCellPressure2 first = scene.getWindField()->getCutCellPressure();
    check( scene.getWindCouplingSteps() == 2 &&
        first.usedCoupledProjection && !first.coupledSolveFailed &&
        first.coupledStatus == fluidCoupledPressureStatus2::converged &&
        first.synchronizedBoundaryFaces > 0 &&
        std::isfinite( first.predictedContinuityRms ) &&
        IsFinite( scene.getWorld().GetBodyLinearVelocity( moving ) ),
        "opt-in weighted Schur pressure must receive World mass/inertia and advance substep-aligned rigid bodies" );

    scene.step( dt, 4 );
    const fluidCutCellPressure2 moved = scene.getWindField()->getCutCellPressure();
    const fluidSweptFaceFlux2 swept = scene.getWindField()->getSweptFaceFlux();
    check( scene.getWindCouplingSteps() == 2 && swept.usedSweptFlux &&
        moved.hasVolumeHistory && std::isfinite( moved.volumeRate ) &&
        std::isfinite( moved.predictedContinuityRms ) &&
        std::isfinite( moved.predictedSolidVolumeL1 ) &&
        IsFinite( scene.getWorld().GetBodyLinearVelocity( moving ) ),
        "next cut-cell fluid interval must use the actual World geometry swept history without reapplying predicted displacement" );

    scene.configureWindField( 32, 18, 0.5f );
    check( scene.isWindCutCellProjectionEnabled() &&
        scene.isWindCutCellCoupledProjectionEnabled() &&
        scene.getWindField()->getCutCellPressure().activeCells == 0,
        "field resize must keep explicit experimental coupling while resetting old geometry history" );

    scene.setWindSynchronizedCouplingEnabled( false );
    scene.addWindVortex();
    scene.step( dt, 4 );
    check( scene.getWindCouplingSteps() == 1 &&
        !scene.getWindField()->getCutCellPressure().usedCoupledProjection,
        "disabling synchronized coupling must keep the weighted legacy solver despite the opt-in" );

    scene.resetWindFlow();
    scene.setWindSynchronizedCouplingEnabled( true );
    scene.setWindCutCellCoupledProjectionEnabled( false );
    scene.addWindVortex();
    scene.step( dt, 4 );
    check( scene.getWindCouplingSteps() == 1 &&
        !scene.getWindField()->getCutCellPressure().usedCoupledProjection &&
        !scene.getWindField()->getCutCellPressure().coupledSolveFailed,
        "cut-cell mass coupling disabled must preserve the old single-exchange weighted projection" );

    scene.resetWindFlow();
    scene.setWindCutCellCoupledProjectionEnabled( true );
    scene.setWindObstacleCouplingEnabled( false );
    scene.step( dt, 4 );
    check( scene.getWindCouplingSteps() == 1 &&
        !scene.getWindField()->getCutCellPressure().usedCoupledProjection,
        "one-way field with optional static obstacles must not silently insert dynamic-body inverse masses" );
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
    checkPressurePredictedWorldVelocity();
    checkNoSourceWindEnergyEnvelope();
    checkFluidRateAndStaticObstacles();
    checkRotatedSandboxObstacles();
    checkDragMomentumBudget();
    checkPressureLedgerInSandbox();
    checkExperimentalCutCellProjection();
    checkExperimentalCutCellWorldCoupling();
    checkSandboxConservativeAdvectionTrial();
}
