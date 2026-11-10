#include "rigidBodyDemo.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string_view>
#include <vector>

using namespace zonai;
using namespace zonai::sandbox;

namespace
{

constexpr float dt = 1.0f / 60.0f;
constexpr int worldSubsteps = 8;
constexpr int warmupFrames = 4;
constexpr int measuredFrames = 12;

struct frameSample
{
    double fluid = 0.0;
    double world = 0.0;
    fluidStepTiming2 phases{};
    fluidCoupledPressureTiming2 pcg{};
    int coupled = 0;
    int failed = 0;
    int analytic = 0;
};

double median( std::vector<double> samples )
{
    std::sort( samples.begin(), samples.end() );
    return samples[samples.size() / 2];
}

struct summary
{
    frameSample medians{};
    int coupledFrames = 0;
    int failedFrames = 0;
    int analyticFrames = 0;
    int validFrames = 0;
    double warmStartGuessRatio = 0.0;
};

summary measure( float h, int fluidSubsteps, bool cached, bool active, bool schurCached,
    bool warmStartProbe = false, bool warmStart = false )
{
    // Run the real sandbox World/Fluid interleave (including rigid force feedback,
    // drag sampling, and World::Step). No synthetic obstacle trajectory or snapshots.
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.configureWindField( static_cast<int>( 12.0f / h ),
        static_cast<int>( 7.0f / h ), h );
    scene.setWindCutCellProjectionEnabled( true );
    scene.setWindCutCellCoupledProjectionEnabled( true );
    scene.setWindDisjointAnalyticEnabled( true );
    scene.setWindCachedIc0Enabled( cached );
    scene.setWindActiveIc0SweepEnabled( active );
    scene.setWindCachedSchurEdgesEnabled( schurCached );
    scene.setWindPressureWarmStartProbeEnabled( warmStartProbe );
    scene.setWindPressureWarmStartEnabled( warmStart );
    scene.setWindFluidSubstepCount( fluidSubsteps );
    scene.setWindStepProfilingEnabled( true );
    scene.setWindSourceStrength( 8.0f );
    scene.setWindDragCoefficient( 3.0f );
    scene.seedWindTracer();
    const auto shapes = scene.getShapes();
    scene.getWorld().SetBodyLinearVelocity( shapes[0].bodyHandle, { 0.7f, -0.10f } );
    scene.getWorld().SetBodyAngularVelocity( shapes[1].bodyHandle, 0.5f );

    for( int i = 0; i < warmupFrames; ++i )
        scene.step( dt, worldSubsteps );

    std::vector<frameSample> samples;
    samples.reserve( measuredFrames );
    summary stats{};
    for( int frame = 0; frame < measuredFrames; ++frame )
    {
        scene.step( dt, worldSubsteps );
        frameSample sample{};
        sample.fluid = scene.getWindFluidMilliseconds();
        sample.world = scene.getWindWorldMilliseconds();
        sample.phases = scene.getWindFieldTiming();
        sample.pcg = scene.getWindCoupledTiming();
        const fluidCutCellPressure2& p = scene.getWindField()->getCutCellPressure();
        sample.coupled = p.usedCoupledProjection ? 1 : 0;
        sample.failed = p.coupledSolveFailed ? 1 : 0;
        sample.analytic = scene.getWindField()->getCutCellTransition().usedDisjointAnalyticGeometry ? 1 : 0;
        ++stats.validFrames;
        stats.coupledFrames += sample.coupled;
        stats.failedFrames += sample.failed;
        stats.analyticFrames += sample.analytic;
        samples.push_back( sample );
    }
    const auto get = [&]( auto accessor )
    {
        std::vector<double> values;
        values.reserve( samples.size() );
        for( const auto& sample : samples ) values.push_back( accessor( sample ) );
        return median( std::move( values ) );
    };
    stats.medians.fluid = get( []( const auto& s ) { return s.fluid; } );
    stats.medians.world = get( []( const auto& s ) { return s.world; } );
    auto& m = stats.medians.phases;
    m.total = get( []( const auto& s ) { return s.phases.total; } );
    m.occupancy = get( []( const auto& s ) { return s.phases.occupancy; } );
    m.geometry = get( []( const auto& s ) { return s.phases.geometry; } );
    m.swept = get( []( const auto& s ) { return s.phases.swept; } );
    m.remap = get( []( const auto& s ) { return s.phases.remap; } );
    m.boundaries = get( []( const auto& s ) { return s.phases.boundaries; } );
    m.advection = get( []( const auto& s ) { return s.phases.advection; } );
    m.pressure = get( []( const auto& s ) { return s.phases.pressure; } );
    m.tracer = get( []( const auto& s ) { return s.phases.tracer; } );
    m.other = get( []( const auto& s ) { return s.phases.other; } );
    auto& pcg = stats.medians.pcg;
    pcg.total = get( []( const auto& s ) { return s.pcg.total; } );
    pcg.assembly = get( []( const auto& s ) { return s.pcg.assembly; } );
    pcg.factorization = get( []( const auto& s ) { return s.pcg.factorization; } );
    pcg.multiply = get( []( const auto& s ) { return s.pcg.multiply; } );
    pcg.preconditioner = get( []( const auto& s ) { return s.pcg.preconditioner; } );
    pcg.ic0Forward = get( []( const auto& s ) { return s.pcg.ic0Forward; } );
    pcg.ic0Backward = get( []( const auto& s ) { return s.pcg.ic0Backward; } );
    pcg.warmStartRhsSquared = get( []( const auto& s ) { return s.pcg.warmStartRhsSquared; } );
    pcg.warmStartGuessSquared = get( []( const auto& s ) { return s.pcg.warmStartGuessSquared; } );
    pcg.warmStartProbes = static_cast<int>(
        get( []( const auto& s ) { return s.pcg.warmStartProbes; } ) );
    pcg.warmStartAttempts = static_cast<int>(
        get( []( const auto& s ) { return s.pcg.warmStartAttempts; } ) );
    pcg.warmStartAccepted = static_cast<int>(
        get( []( const auto& s ) { return s.pcg.warmStartAccepted; } ) );
    stats.warmStartGuessRatio = get( []( const auto& s )
    {
        return s.pcg.warmStartRhsSquared > 0.0
            ? std::sqrt( s.pcg.warmStartGuessSquared / s.pcg.warmStartRhsSquared ) : 0.0;
    } );
    pcg.vector = get( []( const auto& s ) { return s.pcg.vector; } );
    pcg.validation = get( []( const auto& s ) { return s.pcg.validation; } );
    pcg.output = get( []( const auto& s ) { return s.pcg.output; } );
    pcg.matrixProducts = static_cast<int>(
        get( []( const auto& s ) { return s.pcg.matrixProducts; } ) );
    pcg.preconditionerApplications = static_cast<int>(
        get( []( const auto& s ) { return s.pcg.preconditionerApplications; } ) );
    pcg.cachedLaplacianEdges = static_cast<int>(
        get( []( const auto& s ) { return s.pcg.cachedLaplacianEdges; } ) );
    return stats;
}

// Per-frame trace, not a timing comparison: identical Fluid cadence for ON
// and OFF, with the actual World/Fluid/Drag path used by the Sandbox.
void auditCoupling()
{
    std::puts( "cell_m,world_substeps,fluid_substeps,obstacle_coupling,drag,frame,"
        "actual_fluid_steps,pre_drag_continuity_rms,post_drag_continuity_rms,"
        "max_drag_continuity_increase,pre_drag_max_face_speed,post_drag_max_face_speed,"
        "max_drag_face_speed_increase,nonfinite_faces,pcg_status,pcg_iterations,"
        "pcg_relative_residual,pcg_failed,body0_speed,body0_omega,body0_x,body0_y,"
        "drag_body_impulse,drag_exchange_residual,peak_face_substep,peak_face_is_u,"
        "peak_face_x,peak_face_y,peak_face_open_fraction,peak_face_signed_velocity,"
        "peak_open_weighted_max_speed,peak_sampled_normal_velocity,"
        "peak_adjacent_cell_normal_speed,peak_adjacent_cell_fluid_fraction,"
        "peak_face_flux_normal_velocity,fluid_ms,world_ms" );
    for( float h : { 0.5f, 0.25f } )
        for( int steps : { 1, 4 } )
            for( bool coupled : { false, true } )
                for( float drag : { 0.0f, 3.0f } )
                {
                    rigidBodyDemo scene{ demoKind::windFlow };
                    scene.configureWindField( static_cast<int>( 12.0f / h ),
                        static_cast<int>( 7.0f / h ), h );
                    scene.setWindCutCellProjectionEnabled( true );
                    scene.setWindCutCellCoupledProjectionEnabled( true );
                    scene.setWindDisjointAnalyticEnabled( true );
                    scene.setWindFluidSubstepCount( steps );
                    scene.setWindCouplingAuditEnabled( true );
                    scene.setWindObstacleCouplingEnabled( coupled );
                    scene.setWindSourceStrength( 8.0f );
                    scene.setWindDragCoefficient( drag );
                    const bodyId body = scene.getShapes()[0].bodyHandle;
                    scene.getWorld().SetBodyLinearVelocity( body, { 0.7f, -0.10f } );
                    scene.getWorld().SetBodyAngularVelocity( scene.getShapes()[1].bodyHandle, 0.5f );
                    for( int frame = 0; frame < warmupFrames + measuredFrames; ++frame )
                    {
                        scene.step( dt, worldSubsteps );
                        if( frame < warmupFrames ) continue;
                        const auto& audit = scene.getWindCouplingAudit();
                        const fluidFlowAudit2& peak = audit.peakBeforeDrag;
                        const auto& pressure = scene.getWindField()->getCutCellPressure();
                        const vec2 position = scene.getWorld().GetBodyTransform( body ).position;
                        const vec2 speed = scene.getWorld().GetBodyLinearVelocity( body );
                        std::printf( "%.2f,%d,%d,%d,%.1f,%d,%d,%.7g,%.7g,%.7g,"
                            "%.7g,%.7g,%.7g,%d,%d,%d,%.7g,%d,%.7g,%.7g,%.7g,%.7g,"
                            "%.7g,%.7g,%d,%d,%d,%d,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.4f,%.4f\n",
                            h, worldSubsteps, steps, coupled ? 1 : 0, drag,
                            frame - warmupFrames, scene.getWindCouplingSteps(),
                            audit.maxContinuityBeforeDrag, audit.maxContinuityAfterDrag,
                            audit.maxContinuityIncreaseFromDrag,
                            audit.maxFaceSpeedBeforeDrag, audit.maxFaceSpeedAfterDrag,
                            audit.maxFaceSpeedIncreaseFromDrag, audit.nonFiniteFluidFaces,
                            static_cast<int>( pressure.coupledStatus ), pressure.coupledIterations,
                            pressure.coupledRelativeResidual, pressure.coupledSolveFailed ? 1 : 0,
                            Length( speed ), scene.getWorld().GetBodyAngularVelocity( body ),
                            position.x, position.y,
                            Length( scene.getWindDragBodyImpulse() ),
                            Length( scene.getWindDragExchangeResidual() ),
                            audit.peakFluidSubstep, peak.maxFaceIsU ? 1 : 0,
                            peak.maxFaceX, peak.maxFaceY, peak.maxFaceOpenFraction,
                            peak.maxFaceVelocity, peak.maxOpenWeightedFaceSpeed,
                            audit.peakSampledNormalVelocity, audit.peakAdjacentCellNormalSpeed,
                            audit.peakAdjacentCellFluidFraction,
                            peak.maxFaceVelocity * peak.maxFaceOpenFraction,
                            scene.getWindFluidMilliseconds(), scene.getWindWorldMilliseconds() );
                    }
                }
}

// Isolate when the verified h=0.25 moving-circle V(17,13) face spikes.
// The target is sampled in the same Fluid interval as each frame's raw peak.
// Its fixed index is deliberately specific to the regression scene.
void auditFaceStages()
{
    constexpr float h = 0.25f;
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.configureWindField( 48, 28, h );
    scene.setWindCutCellProjectionEnabled( true );
    scene.setWindCutCellCoupledProjectionEnabled( true );
    scene.setWindDisjointAnalyticEnabled( true );
    scene.setWindFluidSubstepCount( 4 );
    scene.setWindCouplingAuditEnabled( true );
    scene.setWindFaceStageAuditTarget( 17, 13, false );
    scene.setWindObstacleCouplingEnabled( true );
    scene.setWindSourceStrength( 8.0f );
    scene.setWindDragCoefficient( 0.0f );
    scene.getWorld().SetBodyLinearVelocity( scene.getShapes()[0].bodyHandle, { 0.7f, -0.10f } );
    scene.getWorld().SetBodyAngularVelocity( scene.getShapes()[1].bodyHandle, 0.5f );
    std::puts( "frame,peak_substep,peak_face_is_u,peak_face_x,peak_face_y,"
        "peak_face_speed,peak_face_open,target_valid,target_open,"
        "target_before_geometry,target_after_remap,target_after_boundary,"
        "target_after_advection,target_after_pressure,target_pressure_delta,"
        "lower_cell_fraction,upper_cell_fraction,lower_volume_rate,upper_volume_rate,"
        "lower_pressure,upper_pressure,predicted_pressure_delta,"
        "upper_west_open,upper_east_open,upper_south_open,upper_north_open,"
        "upper_rhs,upper_laplacian,upper_schur,upper_row_residual,"
        "upper_final_open_flux,upper_raw_continuity,"
        "lower_rhs,lower_row_residual" );
    for( int frame = 0; frame < warmupFrames + measuredFrames; ++frame )
    {
        scene.step( dt, worldSubsteps );
        if( frame < warmupFrames ) continue;
        const auto& audit = scene.getWindCouplingAudit();
        const auto& peak = audit.peakBeforeDrag;
        const auto& stages = audit.peakFaceStages;
        std::printf( "%d,%d,%d,%d,%d,%.9g,%.9g,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
            "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
            "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
            frame - warmupFrames, audit.peakFluidSubstep,
            peak.maxFaceIsU ? 1 : 0, peak.maxFaceX, peak.maxFaceY,
            peak.maxFaceVelocity, peak.maxFaceOpenFraction,
            stages.valid ? 1 : 0, stages.openFraction, stages.beforeGeometry,
            stages.afterRemap, stages.afterBoundary, stages.afterAdvection,
            stages.afterPressure, stages.afterPressure - stages.afterAdvection,
            stages.lowerCellFraction, stages.upperCellFraction,
            stages.lowerVolumeRate, stages.upperVolumeRate,
            stages.lowerPressure, stages.upperPressure, stages.predictedPressureDelta,
            stages.upperRow.westOpen, stages.upperRow.eastOpen,
            stages.upperRow.southOpen, stages.upperRow.northOpen,
            stages.upperRow.rhs, stages.upperRow.laplacian, stages.upperRow.schur,
            stages.upperRow.rowResidual, stages.upperRow.finalOpenFlux,
            stages.upperRow.rawContinuity,
            stages.lowerRow.rhs, stages.lowerRow.rowResidual );
    }
}

// Pressure-only differential: both runs start at the identical live World
// snapshot AFTER geometry, swept flux and velocity advection, BEFORE PCG.
// Unlike separate World trajectories, this probes only Schur L traversal.
void auditFrozenPressure( bool smallCellTrial = false, bool thresholdSweep = false )
{
    constexpr int targetMeasuredFrame = 6;
    constexpr int fluidSubsteps = 4;
    constexpr int targetSubstep = 3;
    rigidBodyDemo scene{ demoKind::windFlow };
    constexpr float h = 0.25f;
    scene.configureWindField( 48, 28, h );
    scene.setWindCutCellProjectionEnabled( true );
    scene.setWindCutCellCoupledProjectionEnabled( true );
    scene.setWindDisjointAnalyticEnabled( true );
    scene.setWindFluidSubstepCount( fluidSubsteps );
    scene.setWindFaceStageAuditTarget( 17, 13, false );
    scene.setWindObstacleCouplingEnabled( true );
    scene.setWindSourceStrength( 8.0f );
    scene.setWindDragCoefficient( 0.0f );
    // Passive tracer must be nonzero for the frozen-state mass audit.
    scene.seedWindTracer();
    scene.getWorld().SetBodyLinearVelocity( scene.getShapes()[0].bodyHandle, { 0.7f, -0.10f } );
    scene.getWorld().SetBodyAngularVelocity( scene.getShapes()[1].bodyHandle, 0.5f );
    std::optional<fluidField2> snapshot;
    int capturedIndex = -1;
    int fluidIndex = 0;
    const int targetIndex = ( warmupFrames + targetMeasuredFrame ) * fluidSubsteps + targetSubstep;
    scene.setWindPrePressureSnapshotObserver( [&]( const fluidField2& before )
    {
        if( fluidIndex == targetIndex )
        {
            snapshot.emplace( before );
            capturedIndex = fluidIndex;
        }
        ++fluidIndex;
    } );
    for( int frame = 0; frame <= warmupFrames + targetMeasuredFrame; ++frame )
        scene.step( dt, worldSubsteps );
    scene.setWindPrePressureSnapshotObserver( {} );

    if( !snapshot || capturedIndex != targetIndex )
    {
        std::fprintf( stderr, "frozen Cut-cell pressure substep was not captured\n" );
        std::exit( 1 );
    }
    if( thresholdSweep )
    {
        // Exactly the same World/Fluid state, pressure inputs, and obstacle
        // transforms for every candidate. Only P^T A P grouping changes.
        std::puts( "threshold,merged_cells,orphan_slivers,pcg_status,pcg_iterations,"
            "target_v,max_face_speed,group_continuity,individual_continuity,"
            "impulse_residual,virtual_exchange,snapshot_decrease,tracer_mass" );
        float baselineTargetSpeed = 0.0f;
        float baselineTracerMass = 0.0f;
        for( float threshold : { 0.0f, 1.0e-9f, 1.0e-8f, 1.0e-6f, 1.0e-4f, 1.0e-2f } )
        {
            fluidField2 trial = *snapshot;
            trial.setPrePressureSnapshotObserver( {} );
            trial.setSmallCellPressureMergeFraction( threshold );
            if( !trial.replayFrozenCutCellPressure() )
            {
                std::fprintf( stderr, "pressure threshold replay failed\n" );
                std::exit( 1 );
            }
            const auto& p = trial.getCutCellPressure();
            const vec2 point = snapshot->getOrigin() + h * vec2{ 17.5f, 13.0f };
            std::printf( "%.9g,%d,%d,%d,%d,%.9g,%.9g,%.9g,%.9g,"
                "%.9g,%.9g,%.9g,%.9g\n",
                threshold, p.mergedPressureCells, p.unmergedSmallPressureCells,
                static_cast<int>( p.coupledStatus ), p.coupledIterations,
                trial.sampleVelocity( point ).y,
                trial.getFlowAudit( dt / fluidSubsteps ).maxFluidFaceSpeed,
                p.groupedPredictedContinuityRms, p.predictedContinuityRms,
                Length( p.impulse.residual() ), p.virtualInternalExchangeVolume,
                p.mergedSnapshotDecreaseVolume, trial.getTracerMass() );
            if( threshold == 0.0f )
            {
                baselineTargetSpeed = std::abs( trial.sampleVelocity( point ).y );
                baselineTracerMass = trial.getTracerMass();
                continue;
            }
            if( threshold == 1.0e-4f )
            {
                // Golden frozen-frame regression, not a general proof of
                // moving-boundary volume or tracer transport conservation.
                const float sliverSpeed = std::abs( trial.sampleVelocity( point ).y );
                if( !p.usedCoupledProjection || p.mergedPressureCells == 0 ||
                    !std::isfinite( sliverSpeed ) ||
                    sliverSpeed >= 0.5f * baselineTargetSpeed ||
                    !std::isfinite( p.groupedPredictedContinuityRms ) ||
                    p.groupedPredictedContinuityRms > 1.0e-5f ||
                    !std::isfinite( p.virtualInternalExchangeVolume ) ||
                    p.virtualInternalExchangeVolume <= 0.0f ||
                    p.mergedSnapshotDecreaseVolume <= 0.0f ||
                    p.virtualInternalExchangeVolume >
                        2.0f * p.mergedSnapshotDecreaseVolume ||
                    Length( p.impulse.residual() ) > 1.0e-4f ||
                    std::abs( trial.getTracerMass() - baselineTracerMass ) > 1.0e-6f )
                {
                    std::fprintf( stderr, "small-cell frozen pressure conservation regression\n" );
                    std::exit( 1 );
                }
            }
        }
        return;
    }
    fluidField2 baseline = *snapshot;
    fluidField2 reference = *snapshot;
    baseline.setPrePressureSnapshotObserver( {} );
    reference.setPrePressureSnapshotObserver( {} );
    if( smallCellTrial )
        reference.setSmallCellPressureMergeFraction( 1.0e-4f );
    else reference.setAnalyticCutCellCachedSchurEdgesEnabled( false );
    const auto forceA = baseline.replayFrozenCutCellPressure();
    const auto forceB = reference.replayFrozenCutCellPressure();
    if( !forceA || !forceB || forceA->size() != forceB->size() )
    {
        std::fprintf( stderr, "frozen pressure replay failed\n" );
        std::exit( 1 );
    }
    const vec2 point = snapshot->getOrigin() + h * vec2{ 17.5f, 13.0f };
    const float actual = scene.getWindField()->sampleVelocity( point ).y;
    const float a = baseline.sampleVelocity( point ).y;
    const float b = reference.sampleVelocity( point ).y;
    float forceDifference = 0.0f;
    float torqueDifference = 0.0f;
    for( std::size_t i = 0; i < forceA->size(); ++i )
    {
        forceDifference = std::max( forceDifference,
            Length( ( *forceA )[i].force - ( *forceB )[i].force ) );
        torqueDifference = std::max( torqueDifference,
            std::abs( ( *forceA )[i].torque - ( *forceB )[i].torque ) );
    }
    const auto& pA = baseline.getCutCellPressure();
    const auto& pB = reference.getCutCellPressure();
    const auto& live = scene.getWindField()->getCutCellPressure();
    const auto& traced = scene.getWindField()->getFaceStageAudit();
    if( smallCellTrial )
    {
        // Paired repeated frozen solves avoid attributing CI runner noise
        // or separate World trajectories to the pressure model. The two
        // pressure solvers alternate execution order on the same snapshot.
        std::array<std::vector<double>, 2> timings{};
        for( int iteration = 0; iteration < 7; ++iteration )
            for( int order = 0; order < 2; ++order )
            {
                const int variant = ( iteration + order ) & 1;
                fluidField2 probe = *snapshot;
                probe.setPrePressureSnapshotObserver( {} );
                if( variant == 1 ) probe.setSmallCellPressureMergeFraction( 1.0e-4f );
                probe.setCutCellPressureTimingEnabled( true );
                const auto feedback = probe.replayFrozenCutCellPressure();
                if( !feedback || !probe.getCutCellPressure().usedCoupledProjection )
                {
                    std::fprintf( stderr, "frozen pressure timing variant failed\n" );
                    std::exit( 1 );
                }
                timings[variant].push_back( probe.getCutCellPressure().coupledSolveMilliseconds );
            }
        const double baseMedianMs = median( timings[0] );
        const double mergedMedianMs = median( timings[1] );
        std::puts( "substep_index,face_open,upper_fraction,base_v,merged_v,"
            "base_status,merged_status,merged_cells,orphan_slivers,merged_fraction,"
            "base_continuity_rms,merged_continuity_rms,"
            "base_predicted_continuity_rms,merged_predicted_continuity_rms,"
            "merged_group_continuity_rms,virtual_exchange_m3,snapshot_decrease_m3,"
            "base_impulse_residual,merged_impulse_residual,"
            "base_source_imbalance,merged_source_imbalance,"
            "base_tracer_mass,merged_tracer_mass,base_energy,merged_energy,"
            "base_schur_ms,merged_schur_ms,feedback_force_delta,feedback_torque_delta" );
        std::printf( "%d,%.9g,%.9g,%.9g,%.9g,%d,%d,%d,%d,%.9g,"
            "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
            "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
            capturedIndex, traced.openFraction, traced.upperCellFraction,
            a, b, static_cast<int>( pA.coupledStatus ), static_cast<int>( pB.coupledStatus ),
            pB.mergedPressureCells, pB.unmergedSmallPressureCells,
            pB.mergedFluidFraction, pA.divergenceAfter, pB.divergenceAfter,
            pA.predictedContinuityRms, pB.predictedContinuityRms,
            pB.groupedPredictedContinuityRms, pB.virtualInternalExchangeVolume,
            pB.mergedSnapshotDecreaseVolume,
            Length( pA.impulse.residual() ), Length( pB.impulse.residual() ),
            pA.sourceImbalance, pB.sourceImbalance,
            baseline.getTracerMass(), reference.getTracerMass(),
            baseline.getFluidKineticEnergy(), reference.getFluidKineticEnergy(),
            baseMedianMs, mergedMedianMs,
            forceDifference, torqueDifference );
        return;
    }
    std::puts( "substep_index,field_substeps,face_open,upper_cell_fraction,"
        "live_v,default_v,reference_v,default_pcg_status,reference_pcg_status,"
        "default_pressure_residual,reference_pressure_residual,"
        "default_continuity,reference_continuity,live_continuity,"
        "default_pressure_impulse_residual,reference_pressure_impulse_residual,"
        "default_tracer_mass,reference_tracer_mass,force_difference,torque_difference,"
        "live_upper_pressure_row_residual" );
    std::printf( "%d,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%d,"
        "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
        capturedIndex, fluidIndex, traced.openFraction, traced.upperCellFraction,
        actual, a, b, static_cast<int>( pA.coupledStatus ),
        static_cast<int>( pB.coupledStatus ), pA.pressureResidual, pB.pressureResidual,
        pA.divergenceAfter, pB.divergenceAfter, live.divergenceAfter,
        Length( pA.impulse.residual() ), Length( pB.impulse.residual() ),
        baseline.getTracerMass(), reference.getTracerMass(),
        forceDifference, torqueDifference, traced.upperRow.rowResidual );
}

// Cross-cadence *sensitivity* audit: each row has its own live World history,
// but baseline and pooling in that row share one exact pre-pressure snapshot.
// These are not matched full-World trajectory A/B runs.
void auditSmallCellPressureCadences()
{
    std::puts( "cell_m,world_substeps,fluid_substeps,snapshot_index,pcg_base,pcg_pooled,"
        "pcg_iters_base,pcg_iters_pooled,merged_cells,orphan_slivers,"
        "max_velocity_base,max_velocity_pooled,max_weighted_base,max_weighted_pooled,"
        "individual_continuity_base,individual_continuity_pooled,group_continuity_pooled,"
        "virtual_exchange_m3,snapshot_decrease_m3,impulse_residual_base,"
        "impulse_residual_pooled,tracer_mass_base,tracer_mass_pooled,"
        "feedback_force_delta,feedback_torque_delta" );
    for( float h : { 0.5f, 0.25f } )
        for( int fluidSubsteps : { 2, 4, 8 } )
        {
            rigidBodyDemo scene{ demoKind::windFlow };
            scene.configureWindField( static_cast<int>( 12.0f / h ),
                static_cast<int>( 7.0f / h ), h );
            scene.setWindCutCellProjectionEnabled( true );
            scene.setWindCutCellCoupledProjectionEnabled( true );
            scene.setWindDisjointAnalyticEnabled( true );
            scene.setWindFluidSubstepCount( fluidSubsteps );
            scene.setWindObstacleCouplingEnabled( true );
            scene.setWindSourceStrength( 8.0f );
            scene.setWindDragCoefficient( 0.0f );
            scene.seedWindTracer();
            scene.getWorld().SetBodyLinearVelocity(
                scene.getShapes()[0].bodyHandle, { 0.7f, -0.10f } );
            scene.getWorld().SetBodyAngularVelocity(
                scene.getShapes()[1].bodyHandle, 0.5f );
            const int target = ( warmupFrames + 6 ) * fluidSubsteps + fluidSubsteps - 1;
            int fluidIndex = 0;
            std::optional<fluidField2> frozen;
            scene.setWindPrePressureSnapshotObserver( [&]( const fluidField2& ready )
            {
                if( fluidIndex == target ) frozen.emplace( ready );
                ++fluidIndex;
            } );
            for( int frame = 0; frame <= warmupFrames + 6; ++frame )
                scene.step( dt, worldSubsteps );
            scene.setWindPrePressureSnapshotObserver( {} );
            if( !frozen )
            {
                std::fprintf( stderr, "small-cell cadence snapshot missing\n" );
                std::exit( 1 );
            }
            fluidField2 base = *frozen, pooled = *frozen;
            base.setPrePressureSnapshotObserver( {} );
            pooled.setPrePressureSnapshotObserver( {} );
            pooled.setSmallCellPressureMergeFraction( 1.0e-4f );
            const auto forcesA = base.replayFrozenCutCellPressure();
            const auto forcesB = pooled.replayFrozenCutCellPressure();
            if( !forcesA || !forcesB || forcesA->size() != forcesB->size() )
            {
                std::fprintf( stderr, "small-cell cadence pressure replay failed\n" );
                std::exit( 1 );
            }
            float forceDelta = 0.0f, torqueDelta = 0.0f;
            for( std::size_t i = 0; i < forcesA->size(); ++i )
            {
                forceDelta = std::max( forceDelta,
                    Length( ( *forcesA )[i].force - ( *forcesB )[i].force ) );
                torqueDelta = std::max( torqueDelta,
                    std::abs( ( *forcesA )[i].torque - ( *forcesB )[i].torque ) );
            }
            const auto& a = base.getCutCellPressure();
            const auto& p = pooled.getCutCellPressure();
            const auto peakA = base.getFlowAudit( dt / fluidSubsteps );
            const auto peakB = pooled.getFlowAudit( dt / fluidSubsteps );
            std::printf( "%.9g,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
                "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
                "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
                h, worldSubsteps, fluidSubsteps, target,
                static_cast<int>( a.coupledStatus ), static_cast<int>( p.coupledStatus ),
                a.coupledIterations, p.coupledIterations,
                p.mergedPressureCells, p.unmergedSmallPressureCells,
                peakA.maxFluidFaceSpeed, peakB.maxFluidFaceSpeed,
                peakA.maxOpenWeightedFaceSpeed, peakB.maxOpenWeightedFaceSpeed,
                a.predictedContinuityRms, p.predictedContinuityRms,
                p.groupedPredictedContinuityRms,
                p.virtualInternalExchangeVolume, p.mergedSnapshotDecreaseVolume,
                Length( a.impulse.residual() ), Length( p.impulse.residual() ),
                base.getTracerMass(), pooled.getTracerMass(), forceDelta, torqueDelta );
        }
}

// Unlike a frozen one-substep replay this runs TWO independent, initially
// identical, real World/Fluid timelines with force/torque feedback, collisions,
// advection and moving solid geometry. Once the first pressure merge occurs,
// later geometry/forces necessarily diverge; the numbers are no longer
// same-state A/B after that point. This probes feedback stability, not a
// controlled pressure-only difference or pointwise volume conservation.
void auditSmallCellWorldFeedback()
{
    constexpr int frames = 24;
    constexpr float h = 0.25f;
    constexpr int fluidSubsteps = 4;
    std::puts( "drag,frame,model,fluid_steps,merge_fraction,merged_cells,"
        "orphan_slivers,pcg_status,pcg_iterations,pcg_failed,"
        "peak_face_speed,peak_open_weighted_speed,nonfinite_faces,"
        "individual_continuity,group_continuity,"
        "virtual_exchange_m3,snapshot_decrease_m3,pressure_impulse_residual,"
        "tracer_mass,body0_x,body0_y,body0_speed,body0_omega,"
        "body1_omega,pressure_force0,pressure_torque0" );

    for( float drag : { 0.0f, 3.0f } )
    {
        rigidBodyDemo baseline{ demoKind::windFlow };
        rigidBodyDemo pooled{ demoKind::windFlow };
        const auto configure = [&]( rigidBodyDemo& scene, float threshold )
        {
            scene.configureWindField( 48, 28, h );
            scene.setWindCutCellProjectionEnabled( true );
            scene.setWindCutCellCoupledProjectionEnabled( true );
            scene.setWindDisjointAnalyticEnabled( true );
            scene.setWindFluidSubstepCount( fluidSubsteps );
            scene.setWindCouplingAuditEnabled( true );
            scene.setWindObstacleCouplingEnabled( true );
            scene.setWindSourceStrength( 8.0f );
            scene.setWindDragCoefficient( drag );
            scene.seedWindTracer();
            scene.setWindSmallCellPressureMergeFraction( threshold );
            const auto shapes = scene.getShapes();
            scene.getWorld().SetBodyLinearVelocity(
                shapes[0].bodyHandle, { 0.7f, -0.10f } );
            scene.getWorld().SetBodyAngularVelocity( shapes[1].bodyHandle, 0.5f );
        };
        configure( baseline, 0.0f );
        configure( pooled, 1.0e-4f );
        if( baseline.getWindField()->getSmallCellPressureMergeFraction() != 0.0f ||
            pooled.getWindField()->getSmallCellPressureMergeFraction() != 1.0e-4f )
        {
            std::fprintf( stderr, "World feedback trial did not select pooling modes\n" );
            std::exit( 1 );
        }

        int lastFrameMerged = -1;
        int totalLastSubstepMerges = 0;
        for( int frame = 0; frame < frames; ++frame )
        {
            baseline.step( dt, worldSubsteps );
            pooled.step( dt, worldSubsteps );
            for( int variant = 0; variant < 2; ++variant )
            {
                rigidBodyDemo& scene = variant == 0 ? baseline : pooled;
                const auto& audit = scene.getWindCouplingAudit();
                const auto& pressure = scene.getWindField()->getCutCellPressure();
                const bodyId body0 = scene.getShapes()[0].bodyHandle;
                const bodyId body1 = scene.getShapes()[1].bodyHandle;
                const vec2 pos = scene.getWorld().GetBodyTransform( body0 ).position;
                const vec2 velocity = scene.getWorld().GetBodyLinearVelocity( body0 );
                const auto pressureForces = scene.getWindPressureForces();
                const fluidObstacleForce2 force = pressureForces.empty()
                    ? fluidObstacleForce2{} : pressureForces[0];
                // This field is the LAST Fluid substep only, whereas the
                // face peak is the maximum over all Fluid substeps in frame.
                if( variant == 1 && pressure.mergedPressureCells > 0 )
                {
                    lastFrameMerged = frame;
                    totalLastSubstepMerges += pressure.mergedPressureCells;
                }
                std::printf( "%.1f,%d,%d,%d,%.9g,%d,%d,%d,%d,%d,"
                    "%.9g,%.9g,%d,%.9g,%.9g,%.9g,%.9g,%.9g,"
                    "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
                    drag, frame, variant, scene.getWindCouplingSteps(),
                    scene.getWindField()->getSmallCellPressureMergeFraction(),
                    pressure.mergedPressureCells, pressure.unmergedSmallPressureCells,
                    static_cast<int>( pressure.coupledStatus ), pressure.coupledIterations,
                    pressure.coupledSolveFailed ? 1 : 0,
                    audit.maxFaceSpeedBeforeDrag, audit.peakBeforeDrag.maxOpenWeightedFaceSpeed,
                    audit.nonFiniteFluidFaces, pressure.predictedContinuityRms,
                    pressure.groupedPredictedContinuityRms,
                    pressure.virtualInternalExchangeVolume,
                    pressure.mergedSnapshotDecreaseVolume,
                    Length( pressure.impulse.residual() ),
                    scene.getWindField()->getTracerMass(),
                    pos.x, pos.y, Length( velocity ),
                    scene.getWorld().GetBodyAngularVelocity( body0 ),
                    scene.getWorld().GetBodyAngularVelocity( body1 ),
                    Length( force.force ), force.torque );
            }
        }
        // Informational signal only; no assertion that merging must improve
        // an entire diverged World trajectory or that the target sliver
        // exists on every fluid cadence.
        std::fprintf( stderr, "small-cell feedback drag=%.1f last-merged-frame=%d "
            "sum-last-substep-merged=%d\n",
            drag, lastFrameMerged, totalLastSubstepMerges );
    }
}

// Seed tracer just BEFORE the reproducible collapsing V(17,13) face,
// then track actual per-cell stored mass through live geometry and flow.
// The existing swept tracer remap and pressure pooling are independent
// toggles. Summing raw cell masses detects material hidden behind solid
// masks, unlike concentration (which is zero for a fully covered cell).
void auditSmallCellLocalTracer()
{
    constexpr float h = 0.25f;
    constexpr int fluidSubsteps = 4;
    constexpr int frames = 20;
    constexpr int seedAfterFrame = 4;
    constexpr int targetX = 17;
    constexpr int targetY = 13;
    std::puts( "swept_remap,frame,model,seed_x,seed_y,seed_fraction,seed_mass,"
        "merged_cells,pcg_status,pcg_failed,target_fluid_fraction,"
        "target_cell_mass,target_concentration,neighbor_mass,region_mass,"
        "region_trapped_mass,global_trapped_mass,global_mass,centroid_x,centroid_y,"
        "swept_remapped_kg,fallback_remapped_kg,unresolved_kg,"
        "virtual_exchange_m3,snapshot_decrease_m3,peak_face_speed" );

    for( bool sweptRemap : { false, true } )
    {
        rigidBodyDemo baseline{ demoKind::windFlow };
        rigidBodyDemo pooled{ demoKind::windFlow };
        const auto configure = [&]( rigidBodyDemo& scene, float threshold )
        {
            scene.configureWindField( 48, 28, h );
            scene.setWindCutCellProjectionEnabled( true );
            scene.setWindCutCellCoupledProjectionEnabled( true );
            scene.setWindDisjointAnalyticEnabled( true );
            scene.setWindFluidSubstepCount( fluidSubsteps );
            scene.setWindObstacleCouplingEnabled( true );
            scene.setWindSourceStrength( 8.0f );
            scene.setWindDragCoefficient( 0.0f );
            scene.setWindSmallCellPressureMergeFraction( threshold );
            scene.setWindSweptTracerRemapEnabled( sweptRemap );
            const auto shapes = scene.getShapes();
            scene.getWorld().SetBodyLinearVelocity( shapes[0].bodyHandle, { 0.7f, -0.10f } );
            scene.getWorld().SetBodyAngularVelocity( shapes[1].bodyHandle, 0.5f );
        };
        configure( baseline, 0.0f );
        configure( pooled, 1.0e-4f );

        int seedX = targetX, seedY = targetY;
        float seedFraction = 0.0f;
        double seedMass = 0.0;
        for( int frame = 0; frame < frames; ++frame )
        {
            baseline.step( dt, worldSubsteps );
            pooled.step( dt, worldSubsteps );
            if( frame == seedAfterFrame )
            {
                // Both World histories have identical pressure treatment
                // before the targeted disappearance; compare fractions to
                // confirm the seed represents the same source fluid volume.
                const fluidField2& seedField = *baseline.getWindField();
                if( seedField.getCellFluidFraction( targetX, targetY ) <= 0.0f )
                {
                    // Deterministic closest useful neighbor if the exact
                    // vanishing cell was already fully covered at seed time.
                    const std::array<std::array<int, 2>, 4> candidates{{
                        {{ targetX + 1, targetY }}, {{ targetX, targetY - 1 }},
                        {{ targetX - 1, targetY }}, {{ targetX, targetY + 1 }}
                    }};
                    float best = 0.0f;
                    for( const auto& candidate : candidates )
                    {
                        const float fraction =
                            seedField.getCellFluidFraction( candidate[0], candidate[1] );
                        if( fraction > best )
                        {
                            best = fraction;
                            seedX = candidate[0];
                            seedY = candidate[1];
                        }
                    }
                }
                seedFraction = seedField.getCellFluidFraction( seedX, seedY );
                if( seedFraction <= 0.0f ||
                    std::abs( seedFraction -
                        pooled.getWindField()->getCellFluidFraction( seedX, seedY ) ) > 1.0e-6f )
                {
                    std::fprintf( stderr, "local tracer seed is not shared fluid geometry\n" );
                    std::exit( 1 );
                }
                baseline.setWindTracerCellConcentration( seedX, seedY, 4.0f );
                pooled.setWindTracerCellConcentration( seedX, seedY, 4.0f );
                seedMass = baseline.getWindField()->getTracerCellMass( seedX, seedY );
                if( seedMass <= 0.0 ||
                    std::abs( seedMass - pooled.getWindField()->getTracerCellMass( seedX, seedY ) ) > 1.0e-12 )
                {
                    std::fprintf( stderr, "local tracer patch was not seeded consistently\n" );
                    std::exit( 1 );
                }
            }
            if( frame <= seedAfterFrame ) continue;
            for( int variant = 0; variant < 2; ++variant )
            {
                const rigidBodyDemo& scene = variant == 0 ? baseline : pooled;
                const fluidField2& field = *scene.getWindField();
                const auto& pressure = field.getCutCellPressure();
                const auto& tracer = field.getTracerStats();
                double regionMass = 0.0;
                double regionTrapped = 0.0;
                double weightedX = 0.0;
                double weightedY = 0.0;
                double neighborMass = 0.0;
                // Unit-depth mass stays in the source cell even when hidden
                // behind a fully solid mask; this query does not discard it.
                for( int y = targetY - 2; y <= targetY + 2; ++y )
                    for( int x = targetX - 2; x <= targetX + 2; ++x )
                    {
                        const double mass = field.getTracerCellMass( x, y );
                        regionMass += mass;
                        weightedX += mass * x;
                        weightedY += mass * y;
                        if( field.getCellFluidFraction( x, y ) <= 0.0f )
                            regionTrapped += mass;
                        if( x != targetX || y != targetY )
                            neighborMass += mass;
                    }
                const double centroidX = regionMass > 0.0 ? weightedX / regionMass : 0.0;
                const double centroidY = regionMass > 0.0 ? weightedY / regionMass : 0.0;
                const float totalMass = field.getTracerMass();
                if( !std::isfinite( totalMass ) ||
                    std::abs( static_cast<double>( totalMass ) - seedMass ) >
                        1.0e-5 * std::max( 1.0, seedMass ) ||
                    !std::isfinite( centroidX ) || !std::isfinite( centroidY ) )
                {
                    std::fprintf( stderr, "local tracer stored mass or centroid invalid\n" );
                    std::exit( 1 );
                }
                std::printf( "%d,%d,%d,%d,%d,%.9g,%.9g,%d,%d,%d,"
                    "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
                    "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
                    sweptRemap ? 1 : 0, frame, variant, seedX, seedY,
                    seedFraction, seedMass, pressure.mergedPressureCells,
                    static_cast<int>( pressure.coupledStatus ),
                    pressure.coupledSolveFailed ? 1 : 0,
                    field.getCellFluidFraction( targetX, targetY ),
                    field.getTracerCellMass( targetX, targetY ),
                    field.getTracerConcentration( targetX, targetY ),
                    neighborMass, regionMass, regionTrapped,
                    tracer.trappedMass, totalMass, centroidX, centroidY,
                    tracer.sweptRemappedMass, tracer.fallbackRemappedMass,
                    tracer.unresolvedDisplacementMass,
                    pressure.virtualInternalExchangeVolume,
                    pressure.mergedSnapshotDecreaseVolume,
                    field.getFlowAudit( dt / fluidSubsteps ).maxFluidFaceSpeed );
            }
        }
    }
}

} // namespace

int main( int argc, char** argv )
{
    bool csv = false;
    bool reference = false;
    bool dense = false;
    bool schurReference = false;
    bool warmStartProbe = false;
    bool warmStartSweep = false;
    bool couplingAudit = false;
    bool faceStageAudit = false;
    bool frozenPressureAudit = false;
    bool smallCellMergeAudit = false;
    bool smallCellThresholdSweep = false;
    bool smallCellCadenceAudit = false;
    bool smallCellWorldFeedbackAudit = false;
    bool smallCellLocalTracerAudit = false;
    for( int i = 1; i < argc; ++i )
    {
        csv |= std::string_view( argv[i] ) == "--csv";
        reference |= std::string_view( argv[i] ) == "--ic0-reference";
        dense |= std::string_view( argv[i] ) == "--ic0-dense";
        schurReference |= std::string_view( argv[i] ) == "--schur-reference";
        warmStartProbe |= std::string_view( argv[i] ) == "--pressure-reuse-probe";
        warmStartSweep |= std::string_view( argv[i] ) == "--pcg-warmstart-sweep";
        couplingAudit |= std::string_view( argv[i] ) == "--coupling-spike-audit";
        faceStageAudit |= std::string_view( argv[i] ) == "--face-stage-audit";
        frozenPressureAudit |= std::string_view( argv[i] ) == "--frozen-pressure-audit";
        smallCellMergeAudit |= std::string_view( argv[i] ) == "--small-cell-merge-audit";
        smallCellThresholdSweep |= std::string_view( argv[i] ) == "--small-cell-merge-threshold-audit";
        smallCellCadenceAudit |= std::string_view( argv[i] ) == "--small-cell-merge-cadence-audit";
        smallCellWorldFeedbackAudit |= std::string_view( argv[i] ) == "--small-cell-world-feedback-audit";
        smallCellLocalTracerAudit |= std::string_view( argv[i] ) == "--small-cell-local-tracer-audit";
    }
    if( couplingAudit ) { auditCoupling(); return 0; }
    if( faceStageAudit ) { auditFaceStages(); return 0; }
    if( frozenPressureAudit ) { auditFrozenPressure(); return 0; }
    if( smallCellMergeAudit ) { auditFrozenPressure( true ); return 0; }
    if( smallCellThresholdSweep ) { auditFrozenPressure( true, true ); return 0; }
    if( smallCellCadenceAudit ) { auditSmallCellPressureCadences(); return 0; }
    if( smallCellWorldFeedbackAudit ) { auditSmallCellWorldFeedback(); return 0; }
    if( smallCellLocalTracerAudit ) { auditSmallCellLocalTracer(); return 0; }
    if( warmStartSweep )
    {
        std::puts( "cell_m,world_substeps,fluid_substeps,warm_start,frames,"
            "converged_frames,failed_frames,pcg_iterations,preconditioner_calls,"
            "warm_attempts,warm_accepted,schur_ms,pressure_ms,field_ms,fluid_ms" );
        for( float h : { 0.5f, 0.25f } )
            for( int steps : { 1, 2, 4, 8 } )
                for( bool enabled : { false, true } )
                {
                    const summary stats = measure( h, steps, true, true, true, false, enabled );
                    const auto& p = stats.medians.phases;
                    const auto& pcg = stats.medians.pcg;
                    std::printf( "%.2f,%d,%d,%s,%d,%d,%d,%d,%d,%d,%d,"
                        "%.5f,%.5f,%.5f,%.5f\n",
                        h, worldSubsteps, steps, enabled ? "warm" : "cold",
                        stats.validFrames, stats.coupledFrames, stats.failedFrames,
                        pcg.matrixProducts, pcg.preconditionerApplications,
                        pcg.warmStartAttempts, pcg.warmStartAccepted,
                        pcg.total, p.pressure, p.total, stats.medians.fluid );
                }
        return 0;
    }
    csv |= warmStartProbe;
    const char* ic0Mode = reference ? "reference" : dense ? "dense" : "active";
    const bool schurCached = !reference && !dense && !schurReference;
    if( csv )
        std::printf( "%s%s\n",
            "cell_m,world_substeps,fluid_substeps,frames,pcg_converged_frames,pcg_failed_frames,analytic_frames,fluid_coupling_ms,world_ms,field_ms,mask_ms,geometry_ms,swept_ms,remap_ms,boundary_ms,advection_ms,pressure_ms,tracer_ms,other_ms,"
            "schur_total_ms,schur_assembly_ms,schur_ic0_ms,schur_operator_ms,"
            "schur_preconditioner_ms,schur_vector_ms,schur_validation_ms,schur_output_ms,"
            "operator_calls,preconditioner_calls,ic0_mode,schur_mode,cached_laplacian_edges,"
            "schur_ic0_forward_ms,schur_ic0_backward_ms",
            warmStartProbe ? ",pressure_guess_ratio,pressure_probe_count" : "" );
    else
        std::puts( "Wind sandbox: actual World/Fluid interleaving, 8 World substeps, 12 measured frames after 4 warmup frames; medians per frame; informational only." );

    for( float h : { 0.5f, 0.25f } )
        for( int steps : { 1, 2, 4, 8 } )
        {
            const summary stats = measure( h, steps, !reference, !dense, schurCached, warmStartProbe );
            const auto& p = stats.medians.phases;
            const auto& pcg = stats.medians.pcg;
            if( csv )
            {
                std::printf( "%.2f,%d,%d,%d,%d,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
                    h, worldSubsteps, steps, stats.validFrames, stats.coupledFrames,
                    stats.failedFrames, stats.analyticFrames, stats.medians.fluid, stats.medians.world,
                    p.total, p.occupancy, p.geometry, p.swept, p.remap, p.boundaries,
                    p.advection, p.pressure, p.tracer, p.other );
                std::printf( ",%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d,%s,%s,%d,%.4f,%.4f",
                    pcg.total, pcg.assembly, pcg.factorization, pcg.multiply, pcg.preconditioner,
                    pcg.vector, pcg.validation, pcg.output,
                    pcg.matrixProducts, pcg.preconditionerApplications,
                    ic0Mode, schurCached ? "cached" : "reference", pcg.cachedLaplacianEdges,
                    pcg.ic0Forward, pcg.ic0Backward );
                if( warmStartProbe )
                    std::printf( ",%.6f,%d", stats.warmStartGuessRatio, pcg.warmStartProbes );
                std::putchar( '\n' );
            }
            else
                std::printf( "h=%.2f World=%d Fluid=%d: field %.3f ms, pressure %.3f ms, "
                    "Schur %.3f (setup %.3f IC0 %.3f A*v %.3f precond %.3f [fwd %.3f / bwd %.3f] vec %.3f), "
                    "World %.3f ms, converged %d/%d, failed %d, IC0=%s\n",
                    h, worldSubsteps, steps, p.total, p.pressure,
                    pcg.total, pcg.assembly, pcg.factorization,
                    pcg.multiply, pcg.preconditioner, pcg.ic0Forward, pcg.ic0Backward,
                    pcg.vector, stats.medians.world,
                    stats.coupledFrames, stats.validFrames, stats.failedFrames,
                    ic0Mode );
        }
}
