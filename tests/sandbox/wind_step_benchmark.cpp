#include "rigidBodyDemo.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <tuple>
#include <utility>
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
        "virtual_exchange_m3,snapshot_decrease_m3,peak_face_speed,"
        "unresolved_sources,unresolved_partial_kg,unresolved_solid_kg,"
        "unresolved_connected_kg,unresolved_disconnected_kg,"
        "unresolved_zero_outward_kg,unresolved_insufficient_outward_kg,"
        "worst_source_x,worst_source_y,worst_unmatched_kg,"
        "worst_previous_fraction,worst_current_fraction,"
        "worst_west_open,worst_east_open,worst_south_open,worst_north_open,"
        "worst_west_swept_m3,worst_east_swept_m3,"
        "worst_south_swept_m3,worst_north_swept_m3,"
        "worst_west_gain,worst_east_gain,worst_south_gain,worst_north_gain,"
        "worst_mass_pre_mac_kg,worst_mass_post_mac_kg,"
        "worst_mac_west_mass_kg,worst_mac_east_mass_kg,"
        "worst_mac_south_mass_kg,worst_mac_north_mass_kg,"
        "worst_mac_west_volume_m3,worst_mac_east_volume_m3,"
        "worst_mac_south_volume_m3,worst_mac_north_volume_m3,"
        "worst_geom_loss_m3,worst_displaced_kg,worst_swept_transfer_kg,"
        "worst_mac_net_out_kg,worst_mac_gross_out_kg,worst_mac_gross_in_kg,"
        "worst_mac_net_out_volume_m3,worst_swept_positive_volume_m3,"
        "worst_swept_negative_volume_m3,"
        "worst_swept_west_mass_kg,worst_swept_east_mass_kg,"
        "worst_swept_south_mass_kg,worst_swept_north_mass_kg,"
        "worst_mass_before_swept_kg,worst_mass_after_swept_kg,"
        "worst_other_sources_inward_swept_kg" );

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
                // GitHub Actions prefixes each partial printf flush with
                // a new log line, so emit each complete CSV record atomically.
                std::string csvLine;
                const auto appendCsv = [&]( const char* format, auto... values )
                {
                    std::array<char, 1024> segment{};
                    const int length = std::snprintf( segment.data(), segment.size(),
                        format, values... );
                    if( length < 0 || static_cast<std::size_t>( length ) >= segment.size() )
                    {
                        std::fprintf( stderr, "small-cell tracer CSV segment overflow\n" );
                        std::exit( 1 );
                    }
                    csvLine.append( segment.data(), static_cast<std::size_t>( length ) );
                };
                appendCsv( "%d,%d,%d,%d,%d,%.9g,%.9g,%d,%d,%d,"
                    "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
                    "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",
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
                const auto& unresolved = tracer.unresolvedAudit;
                if( std::abs( unresolved.totalMass - tracer.unresolvedDisplacementMass ) >
                        1.0e-6 * std::max( 1.0, unresolved.totalMass ) ||
                    std::abs( unresolved.totalMass -
                        unresolved.partialCellMass - unresolved.fullyCoveredCellMass ) > 1.0e-12 ||
                    std::abs( unresolved.totalMass -
                        unresolved.connectedFluidFaceMass - unresolved.noConnectedFluidFaceMass ) > 1.0e-12 ||
                    std::abs( unresolved.totalMass -
                        unresolved.noUsableOutwardSweepMass - unresolved.insufficientOutwardSweepMass ) > 1.0e-12 ||
                    ( unresolved.sources > 0 &&
                        ( unresolved.worstX < 0 || unresolved.worstY < 0 ||
                            unresolved.worstUnmatchedMass <= 0.0 ) ) )
                {
                    std::fprintf( stderr, "invalid unmatched swept tracer mass accounting\n" );
                    std::exit( 1 );
                }
                appendCsv( ",%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%d,%.9g,"
                    "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
                    "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",
                    unresolved.sources, unresolved.partialCellMass,
                    unresolved.fullyCoveredCellMass, unresolved.connectedFluidFaceMass,
                    unresolved.noConnectedFluidFaceMass,
                    unresolved.noUsableOutwardSweepMass,
                    unresolved.insufficientOutwardSweepMass,
                    unresolved.worstX, unresolved.worstY, unresolved.worstUnmatchedMass,
                    unresolved.worstPreviousFraction, unresolved.worstCurrentFraction,
                    unresolved.worstFaceOpen[0], unresolved.worstFaceOpen[1],
                    unresolved.worstFaceOpen[2], unresolved.worstFaceOpen[3],
                    unresolved.worstRawOutwardSweptVolume[0],
                    unresolved.worstRawOutwardSweptVolume[1],
                    unresolved.worstRawOutwardSweptVolume[2],
                    unresolved.worstRawOutwardSweptVolume[3],
                    unresolved.worstNeighborGain[0], unresolved.worstNeighborGain[1],
                    unresolved.worstNeighborGain[2], unresolved.worstNeighborGain[3] );
                if( unresolved.sources > 0 )
                {
                    const double netMacOut = unresolved.worstMacOutwardTracerMass[0] +
                        unresolved.worstMacOutwardTracerMass[1] +
                        unresolved.worstMacOutwardTracerMass[2] +
                        unresolved.worstMacOutwardTracerMass[3];
                    if( !std::isfinite( netMacOut ) ||
                        std::abs( unresolved.worstMassAfterMacAdvection -
                            unresolved.worstMassBeforeMacAdvection + netMacOut ) > 1.0e-9 ||
                        !std::isfinite( unresolved.worstMacOutwardFluidVolume[0] ) ||
                        !std::isfinite( unresolved.worstMacOutwardFluidVolume[1] ) ||
                        !std::isfinite( unresolved.worstMacOutwardFluidVolume[2] ) ||
                        !std::isfinite( unresolved.worstMacOutwardFluidVolume[3] ) )
                    {
                        std::fprintf( stderr, "worst swept source MAC transport mismatch\n" );
                        std::exit( 1 );
                    }
                }
                appendCsv( ",%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
                    "%.9g,%.9g,%.9g,%.9g",
                    unresolved.worstMassBeforeMacAdvection,
                    unresolved.worstMassAfterMacAdvection,
                    unresolved.worstMacOutwardTracerMass[0],
                    unresolved.worstMacOutwardTracerMass[1],
                    unresolved.worstMacOutwardTracerMass[2],
                    unresolved.worstMacOutwardTracerMass[3],
                    unresolved.worstMacOutwardFluidVolume[0],
                    unresolved.worstMacOutwardFluidVolume[1],
                    unresolved.worstMacOutwardFluidVolume[2],
                    unresolved.worstMacOutwardFluidVolume[3] );

                // These are independent accounting quantities: do not
                // subtract a gross MAC export from unresolved swept demand
                // and call the difference a true geometric conservation
                // residual. MAC and Swept are distinct operators and
                // transport different donor snapshots within a Fluid step.
                const double geometryDecrease =
                    std::max( 0.0, static_cast<double>(
                        unresolved.worstPreviousFraction -
                        unresolved.worstCurrentFraction ) ) * h * h;
                double netMacMass = 0.0;
                double grossMacOut = 0.0;
                double grossMacIn = 0.0;
                double netMacVolume = 0.0;
                double sweptPositive = 0.0;
                double sweptNegative = 0.0;
                for( int direction = 0; direction < 4; ++direction )
                {
                    const double macMass =
                        unresolved.worstMacOutwardTracerMass[direction];
                    netMacMass += macMass;
                    grossMacOut += std::max( 0.0, macMass );
                    grossMacIn += std::max( 0.0, -macMass );
                    netMacVolume +=
                        unresolved.worstMacOutwardFluidVolume[direction];
                    const double swept =
                        unresolved.worstRawOutwardSweptVolume[direction];
                    sweptPositive += std::max( 0.0, swept );
                    sweptNegative += std::min( 0.0, swept );
                }
                if( !std::isfinite( netMacVolume ) ||
                    !std::isfinite( geometryDecrease ) ||
                    !std::isfinite( grossMacOut ) ||
                    !std::isfinite( grossMacIn ) ||
                    !std::isfinite( sweptPositive ) ||
                    !std::isfinite( sweptNegative ) ||
                    std::abs( netMacMass - grossMacOut + grossMacIn ) > 1.0e-12 ||
                    ( unresolved.sources > 0 &&
                        ( unresolved.worstDisplacedMass + 1.0e-12 <
                            unresolved.worstSweptTransferMass ||
                          unresolved.worstUnmatchedMass >
                            unresolved.worstDisplacedMass + 1.0e-12 ) ) )
                {
                    std::fprintf( stderr, "worst swept donor geometric/MAC budget invalid\n" );
                    std::exit( 1 );
                }
                appendCsv( ",%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
                    "%.9g,%.9g,%.9g",
                    geometryDecrease, unresolved.worstDisplacedMass,
                    unresolved.worstSweptTransferMass, netMacMass,
                    grossMacOut, grossMacIn, netMacVolume,
                    sweptPositive, sweptNegative );

                double actualSweptOut = 0.0;
                for( const double mass : unresolved.worstSweptOutwardTracerMass )
                    actualSweptOut += mass;
                const double receivedFromOtherSweptSources =
                    unresolved.worstMassAfterSweptRemap -
                    unresolved.worstMassBeforeSweptRemap + actualSweptOut;
                if( unresolved.sources > 0 &&
                    ( std::abs( actualSweptOut -
                        unresolved.worstSweptTransferMass ) > 1.0e-10 ||
                      std::abs( unresolved.worstMassAfterSweptRemap -
                        unresolved.worstMassBeforeMacAdvection ) > 1.0e-12 ||
                      receivedFromOtherSweptSources < -1.0e-10 ||
                      !std::isfinite( receivedFromOtherSweptSources ) ) )
                {
                    std::fprintf( stderr, "worst donor applied Swept transport mismatch\n" );
                    std::exit( 1 );
                }
                appendCsv( ",%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",
                    unresolved.worstSweptOutwardTracerMass[0],
                    unresolved.worstSweptOutwardTracerMass[1],
                    unresolved.worstSweptOutwardTracerMass[2],
                    unresolved.worstSweptOutwardTracerMass[3],
                    unresolved.worstMassBeforeSweptRemap,
                    unresolved.worstMassAfterSweptRemap,
                    receivedFromOtherSweptSources );
                std::puts( csvLine.c_str() );
            }
        }
    }
}

// Actual World8 / Fluid4 interleave around disappearing Cut-cell (18,13).
// Read-only, opt-in: independently reconstruct every cell from directed
// transfers, then measure transport across the center+four-neighbor stencil.
// Swept/fallback kilograms and endpoint MAC volume are distinct ledgers.
void auditSmallCellAppliedTransfers( bool connectedExperiment = false,
    bool faceCapacityAudit = false, bool groupedFaceAudit = false,
    bool linearVelocityAudit = false )
{
    constexpr float h = 0.25f;
    constexpr int fluidSteps = 4;
    constexpr int frames = 20;
    constexpr int seedAfterFrame = 4;
    constexpr int width = 48, height = 28;
    constexpr int targetX = 18, targetY = 13;
    constexpr int target = targetY * width + targetX;
    constexpr int cells = width * height;
    constexpr float fluidDt = dt / fluidSteps;
    if( ( faceCapacityAudit && !connectedExperiment ) ||
        ( groupedFaceAudit && !faceCapacityAudit ) ||
        ( linearVelocityAudit && !groupedFaceAudit ) )
    {
        std::fputs( "face capacity audit mode arguments are invalid\n", stderr );
        std::exit( 1 );
    }
    std::puts( "mode,variant,frame,fluid_substep,merged_cells,pcg_status,"
        "source_fraction,source_mass_before_kg,source_mass_swept_kg,"
        "source_mass_mac_kg,stencil_mass_before_kg,stencil_mass_swept_kg,"
        "stencil_mass_mac_kg,stencil_swept_boundary_net_kg,"
        "stencil_mac_boundary_net_kg,applied_swept_kg,applied_fallback_kg,"
        "applied_mac_kg,applied_connected_kg,connected_sources,"
        "unresolved_kg,unresolved_sources,worst_x,worst_y,"
        "source_snapshot_delta_m3,source_pressure_delta_m3,"
        "source_swept_divergence_m3,source_endpoint_mac_out_m3" );
    if( faceCapacityAudit )
        std::puts( "face_capacity,variant,frame,fluid_substep,events,opposite_mac_events,"
            "opposite_mac_mass_kg,opposite_swept_mass_kg,"
            "transfer_kg,endpoint_mac_equiv_kg,positive_swept_equiv_kg,"
            "aperture_lt_1e3_count,aperture_lt_1e2_mass_kg,"
            "min_aperture,max_transfer_over_positive_mac,"
            "max_event_source_x,max_event_source_y,max_event_dir,"
            "max_event_aperture,max_event_transfer_kg,"
            "max_event_mac_out_m3,max_event_swept_out_m3" );
    if( groupedFaceAudit )
    {
        std::puts( "face_group_summary,variant,frame,fluid_substep,unique_faces,"
            "multi_event_faces,opposite_mac_faces,tiny_endpoint_faces,"
            "tiny_time_mean_faces,gross_connected_kg,unique_endpoint_mac_kg,"
            "mean_aperture_mac_proxy_kg,unique_swept_kg,max_group_over_mac,"
            "max_group_over_mean_mac" );
        std::puts( "face_group_detail,variant,frame,fluid_substep,face_type,face_x,"
            "face_y,events,positive_kg,negative_kg,endpoint_open,"
            "time_mean_open,swept_forward_m3,mac_forward_m3,"
            "unique_endpoint_mac_kg,mean_aperture_mac_proxy_kg,"
            "unique_swept_kg,group_over_mean_mac" );
    }
    if( linearVelocityAudit )
    {
        std::puts( "linear_face_summary,variant,frame,fluid_substep,unique_faces,"
            "total_connected_kg,linear_proxy_kg,endpoint_proxy_kg,"
            "opposed_linear_faces,opposed_linear_kg,max_group_over_linear" );
        std::puts( "linear_face_detail,variant,frame,fluid_substep,face_type,face_x,"
            "face_y,events,positive_kg,negative_kg,endpoint_open,time_mean_open,"
            "linear_proxy_m3,endpoint_mac_m3,linear_proxy_kg,"
            "group_over_linear" );
    }

    int commonSeedX = -1, commonSeedY = -1;
    double commonSeedMass = 0.0;
    bool sawActualSwept = false, sawActualMac = false, sawUnresolved = false;
    bool sawPooledPressureCell = false, sawConnectedResidual = false;
    bool sawTinyAperture = false, sawOpposingMacFlux = false;
    bool sawGroupedFace = false, sawGroupedTinyFace = false;
    bool sawLinearProxy = false, sawLinearProxyDifference = false;
    for( int variant = 0; variant < 2; ++variant )
    {
        rigidBodyDemo scene{ demoKind::windFlow };
        scene.configureWindField( width, height, h );
        scene.setWindCutCellProjectionEnabled( true );
        scene.setWindCutCellCoupledProjectionEnabled( true );
        scene.setWindDisjointAnalyticEnabled( true );
        scene.setWindFluidSubstepCount( fluidSteps );
        scene.setWindObstacleCouplingEnabled( true );
        scene.setWindSourceStrength( 8.0f );
        scene.setWindDragCoefficient( 0.0f );
        scene.setWindSweptTracerRemapEnabled( true );
        scene.setWindConnectedResidualTracerRemapEnabled( connectedExperiment && variant == 1 );
        scene.setWindSweptApertureAuditEnabled( groupedFaceAudit && variant == 1 );
        scene.setWindSmallCellPressureMergeFraction(
            !connectedExperiment && variant == 1 ? 1.0e-4f : 0.0f );
        const auto shapes = scene.getShapes();
        scene.getWorld().SetBodyLinearVelocity( shapes[0].bodyHandle, { 0.7f, -0.10f } );
        scene.getWorld().SetBodyAngularVelocity( shapes[1].bodyHandle, 0.5f );

        std::vector<double> before( cells ), afterSwept( cells );
        std::vector<double> sweptDelta( cells ), macDelta( cells );
        std::array<double, 4> applied{};
        std::array<double, 2> stencilBoundaryNet{};
        struct connectedFaceCapacity
        {
            int events = 0;
            int oppositeMacEvents = 0;
            int tinyApertureEvents = 0;
            int worstSourceX = -1, worstSourceY = -1, worstDirection = -1;
            double oppositeMacMass = 0.0;
            double oppositeSweptMass = 0.0;
            double transferredMass = 0.0;
            double macEquivalentMass = 0.0;
            double sweptEquivalentMass = 0.0;
            double smallApertureMass = 0.0;
            double minAperture = 1.0;
            double maxPositiveMacRatio = 0.0;
            double worstRatio = 0.0;
            double worstAperture = 0.0;
            double worstTransfer = 0.0;
            double worstMacOutwardVolume = 0.0;
            double worstSweptOutwardVolume = 0.0;
        } faceCapacity{};

        // One canonical physical U/V face per substep, independent of how
        // many directed tracer callbacks target it. MAC and Swept data are
        // sampled once per face; concentrations are donor-direction-specific.
        struct groupedFace
        {
            int key = -1;
            int x = -1, y = -1;
            bool isU = true;
            int events = 0;
            double positiveMass = 0.0, negativeMass = 0.0;
            double positiveConcentration = 0.0, negativeConcentration = 0.0;
            double endpointOpen = 0.0, timeMeanOpen = 0.0;
            double macForward = 0.0, sweptForward = 0.0;
            double linearProxyForward = 0.0;
        };
        std::vector<groupedFace> groupedFaces;

        int stepCount = 0, frameIndex = 0, substepIndex = 0;
        bool sweptObserved = false;

        const auto inStencil = [&]( int cell )
        {
            const int x = cell % width, y = cell / width;
            return std::abs( x - targetX ) + std::abs( y - targetY ) <= 1;
        };
        const auto stencilMass = [&]( const std::vector<double>& masses )
        {
            double sum = 0.0;
            for( int cell = 0; cell < cells; ++cell )
                if( inStencil( cell ) ) sum += masses[cell];
            return sum;
        };
        const auto fail = []( const char* message )
        {
            std::fprintf( stderr, "%s\n", message );
            std::exit( 1 );
        };
        scene.setWindTracerTransferObserver( [&]( const fluidTracerTransfer2& transfer )
        {
            const int source = transfer.sourceCell;
            const int destination = transfer.destinationCell;
            if( source < 0 || source >= cells || destination < 0 ||
                destination >= cells || source == destination ||
                !std::isfinite( transfer.mass ) || transfer.mass <= 0.0 )
                fail( "invalid applied World tracer transfer" );
            const int sx = source % width, sy = source / width;
            const int dx = destination % width, dy = destination / width;
            if( transfer.kind != fluidTracerTransferKind2::fallback )
            {
                if( std::abs( sx - dx ) + std::abs( sy - dy ) != 1 )
                    fail( "Swept/MAC/connected transfer bypassed a shared face" );
            }
            if( transfer.kind == fluidTracerTransferKind2::connectedResidual )
            {
                const fluidField2& field = *scene.getWindField();
                const float open = sx != dx
                    ? field.getUFaceOpenFraction( std::max( sx, dx ), sy )
                    : field.getVFaceOpenFraction( sx, std::max( sy, dy ) );
                if( !connectedExperiment || variant == 0 ||
                    !( field.getCellFluidFraction( sx, sy ) > 0.0f ) ||
                    !( field.getCellFluidFraction( dx, dy ) > 0.0f ) ||
                    !( open > 0.0f ) )
                    fail( "connected residual tracer crossed a closed face or inactive fluid cell" );
                if( faceCapacityAudit )
                {
                    const int side = dx < sx ? 0 : dx > sx ? 1 : dy < sy ? 2 : 3;
                    const auto budget = field.getCutCellVolumeBudget( sx, sy, fluidDt );
                    if( !budget.valid || !budget.hasVolumeHistory ||
                        !budget.hasSweptHistory ||
                        !( budget.previousFraction > budget.currentFraction ) ||
                        !( budget.currentFraction > 0.0f ) )
                        fail( "connected residual face has no valid shrinking donor history" );
                    // Read the *pre-Swept* donor concentration. These
                    // kg-equivalent volumes are independent comparisons,
                    // NOT caps or a combined physical face flux.
                    const double concentration = before[source] /
                        ( static_cast<double>( budget.previousFraction ) * h * h );
                    const double macOut = budget.macOutwardVolume[side];
                    const double sweptOut = budget.solidSweptOutwardVolume[side];
                    const double macEquivalent = std::max( 0.0, macOut ) * concentration;
                    const double sweptEquivalent = std::max( 0.0, sweptOut ) * concentration;
                    if( !std::isfinite( concentration ) || concentration < 0.0 ||
                        !std::isfinite( macEquivalent ) || !std::isfinite( sweptEquivalent ) )
                        fail( "nonfinite local tracer face capacity comparison" );
                    ++faceCapacity.events;
                    faceCapacity.transferredMass += transfer.mass;
                    faceCapacity.macEquivalentMass += macEquivalent;
                    faceCapacity.sweptEquivalentMass += sweptEquivalent;
                    faceCapacity.minAperture = std::min( faceCapacity.minAperture,
                        static_cast<double>( open ) );
                    if( open < 1.0e-3f ) ++faceCapacity.tinyApertureEvents;
                    if( open < 1.0e-2f ) faceCapacity.smallApertureMass += transfer.mass;
                    if( macOut <= 0.0 )
                    {
                        ++faceCapacity.oppositeMacEvents;
                        faceCapacity.oppositeMacMass += transfer.mass;
                    }
                    if( sweptOut <= 0.0 ) faceCapacity.oppositeSweptMass += transfer.mass;
                    const double ratio = macEquivalent > 1.0e-18
                        ? transfer.mass / macEquivalent : -1.0;
                    faceCapacity.maxPositiveMacRatio =
                        std::max( faceCapacity.maxPositiveMacRatio, ratio );
                    const double risk = macEquivalent > 1.0e-18
                        ? ratio : std::numeric_limits<double>::infinity();
                    if( risk > faceCapacity.worstRatio )
                    {
                        faceCapacity.worstRatio = risk;
                        faceCapacity.worstSourceX = sx;
                        faceCapacity.worstSourceY = sy;
                        faceCapacity.worstDirection = side;
                        faceCapacity.worstAperture = open;
                        faceCapacity.worstTransfer = transfer.mass;
                        faceCapacity.worstMacOutwardVolume = macOut;
                        faceCapacity.worstSweptOutwardVolume = sweptOut;
                    }
                    if( groupedFaceAudit )
                    {
                        if( !field.hasSweptApertureAudit() )
                            fail( "missing time-sampled aperture for moving Cut-cell" );
                        const bool isU = sx != dx;
                        const int faceX = isU ? std::max( sx, dx ) : sx;
                        const int faceY = isU ? sy : std::max( sy, dy );
                        const int key = isU ? faceY * ( width + 1 ) + faceX :
                            ( width + 1 ) * height + faceY * width + faceX;
                        const double orientation = destination > source ? 1.0 : -1.0;
                        auto it = std::find_if( groupedFaces.begin(), groupedFaces.end(),
                            [&]( const groupedFace& group ) { return group.key == key; } );
                        if( it == groupedFaces.end() )
                        {
                            groupedFace group{};
                            group.key = key;
                            group.x = faceX;
                            group.y = faceY;
                            group.isU = isU;
                            group.endpointOpen = open;
                            group.timeMeanOpen = isU ?
                                field.getSweptUTimeMeanOpenFraction( faceX, faceY ) :
                                field.getSweptVTimeMeanOpenFraction( faceX, faceY );
                            group.macForward = orientation * macOut;
                            group.sweptForward = orientation * sweptOut;
                            if( linearVelocityAudit )
                                group.linearProxyForward = isU ?
                                    field.getSweptULinearVelocityProxyVolume( faceX, faceY, fluidDt ) :
                                    field.getSweptVLinearVelocityProxyVolume( faceX, faceY, fluidDt );
                            if( !std::isfinite( group.timeMeanOpen ) ||
                                group.timeMeanOpen < -1.0e-6 ||
                                group.timeMeanOpen > 1.0 + 1.0e-6 )
                                fail( "integrated open aperture must lie within [0,1]" );
                            groupedFaces.push_back( group );
                            it = std::prev( groupedFaces.end() );
                        }
                        else if( std::abs( it->endpointOpen - open ) > 1.0e-6 ||
                            std::abs( it->macForward - orientation * macOut ) > 1.0e-9 ||
                            std::abs( it->sweptForward - orientation * sweptOut ) > 1.0e-9 ||
                            ( linearVelocityAudit && !std::isfinite( it->linearProxyForward ) ) )
                            fail( "duplicate physical face disagrees on signed shared flux" );

                        ++it->events;
                        if( orientation > 0.0 )
                        {
                            it->positiveMass += transfer.mass;
                            it->positiveConcentration = concentration;
                        }
                        else
                        {
                            it->negativeMass += transfer.mass;
                            it->negativeConcentration = concentration;
                        }
                    }
                }
            }
            const int kind = static_cast<int>( transfer.kind );
            if( kind < 0 || kind > 3 ) fail( "unrecognized tracer transfer type" );
            const bool mac = transfer.kind == fluidTracerTransferKind2::macFace;
            std::vector<double>& delta = mac ? macDelta : sweptDelta;
            delta[source] -= transfer.mass;
            delta[destination] += transfer.mass;
            applied[kind] += transfer.mass;
            const int stage = mac ? 1 : 0;
            if( inStencil( source ) != inStencil( destination ) )
                stencilBoundaryNet[stage] +=
                    inStencil( destination ) ? transfer.mass : -transfer.mass;
        } );
        scene.setWindTracerStageObserver( [&]( const fluidField2& field,
            fluidTracerStage2 stage )
        {
            double signedSum = 0.0;
            if( stage == fluidTracerStage2::afterSweptRemap )
            {
                if( sweptObserved ) fail( "duplicate World Swept stage observation" );
                for( int y = 0; y < height; ++y )
                    for( int x = 0; x < width; ++x )
                    {
                        const int cell = y * width + x;
                        const double actual = field.getTracerCellMass( x, y );
                        if( !std::isfinite( actual ) ||
                            std::abs( actual - ( before[cell] + sweptDelta[cell] ) ) > 1.0e-9 )
                            fail( "World Swept/fallback applied transfers do not reconstruct local mass" );
                        afterSwept[cell] = actual;
                        signedSum += sweptDelta[cell];
                    }
                if( std::abs( signedSum ) > 1.0e-9 )
                    fail( "World Swept transfer ledger is not zero-sum" );
                sweptObserved = true;
                return;
            }
            if( stage != fluidTracerStage2::afterMacAdvection || !sweptObserved )
                fail( "World tracer stage order mismatch" );

            double totalBefore = 0.0, totalAfter = 0.0;
            std::vector<double> afterMac( cells );
            for( int y = 0; y < height; ++y )
                for( int x = 0; x < width; ++x )
                {
                    const int cell = y * width + x;
                    const double actual = field.getTracerCellMass( x, y );
                    if( !std::isfinite( actual ) ||
                        std::abs( actual - ( afterSwept[cell] + macDelta[cell] ) ) > 1.0e-9 )
                        fail( "World MAC applied transfers do not reconstruct local mass" );
                    afterMac[cell] = actual;
                    totalBefore += before[cell];
                    totalAfter += actual;
                    signedSum += macDelta[cell];
                }
            const auto& tracer = field.getTracerStats();
            if( groupedFaceAudit && variant == 1 )
            {
                int multiEventFaces = 0, oppositeMacFaces = 0;
                int tinyEndpointFaces = 0, tinyTimeMeanFaces = 0;
                int opposedLinearFaces = 0;
                double opposedLinearKg = 0.0;
                double linearProxyKg = 0.0;
                double largestLinearRatio = 0.0;
                double gross = 0.0, endpointMacKg = 0.0;
                double meanMacKg = 0.0, sweptKg = 0.0;
                double largestEndpointRatio = 0.0, largestMeanRatio = 0.0;
                for( const groupedFace& group : groupedFaces )
                {
                    if( group.events > 1 ) ++multiEventFaces;
                    if( group.endpointOpen < 1.0e-3 ) ++tinyEndpointFaces;
                    if( group.timeMeanOpen < 1.0e-3 ) ++tinyTimeMeanFaces;
                    const double mass = group.positiveMass + group.negativeMass;
                    const double macKg = std::max( 0.0, group.macForward ) *
                        group.positiveConcentration +
                        std::max( 0.0, -group.macForward ) * group.negativeConcentration;
                    const double sweptMass = std::max( 0.0, group.sweptForward ) *
                        group.positiveConcentration +
                        std::max( 0.0, -group.sweptForward ) * group.negativeConcentration;
                    const double meanMac = group.endpointOpen > 0.0
                        ? macKg * group.timeMeanOpen / group.endpointOpen : 0.0;
                    if( ( group.macForward >= 0.0 && group.negativeMass > 0.0 ) ||
                        ( group.macForward <= 0.0 && group.positiveMass > 0.0 ) )
                        ++oppositeMacFaces;
                    gross += mass;
                    endpointMacKg += macKg;
                    meanMacKg += meanMac;
                    sweptKg += sweptMass;
                    if( linearVelocityAudit )
                    {
                        const double linearKg =
                            std::max( 0.0, group.linearProxyForward ) *
                                group.positiveConcentration +
                            std::max( 0.0, -group.linearProxyForward ) *
                                group.negativeConcentration;
                        if( !std::isfinite( linearKg ) || linearKg < 0.0 )
                            fail( "invalid time-linear MAC flux proxy" );
                        linearProxyKg += linearKg;
                        const bool opposite =
                            ( group.linearProxyForward <= 0.0 &&
                                group.positiveMass > 0.0 ) ||
                            ( group.linearProxyForward >= 0.0 &&
                                group.negativeMass > 0.0 );
                        if( opposite )
                        {
                            ++opposedLinearFaces;
                            opposedLinearKg += group.linearProxyForward <= 0.0 ?
                                group.positiveMass : group.negativeMass;
                        }
                        const double linearRatio =
                            linearKg > 1.0e-18 ? mass / linearKg : 0.0;
                        largestLinearRatio = std::max( largestLinearRatio, linearRatio );
                        sawLinearProxy |= linearKg > 0.0;
                        sawLinearProxyDifference |= std::abs(
                            group.linearProxyForward - group.macForward ) > 1.0e-12;
                        if( group.endpointOpen < 1.0e-3 ||
                            group.timeMeanOpen < 1.0e-3 || linearRatio > 20.0 )
                            std::printf( "linear_face_detail,%d,%d,%d,%d,%d,%d,%d,"
                                "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
                                "%.12g,%.9g\n",
                                variant, frameIndex, substepIndex, group.isU ? 0 : 1,
                                group.x, group.y, group.events,
                                group.positiveMass, group.negativeMass,
                                group.endpointOpen, group.timeMeanOpen,
                                group.linearProxyForward, group.macForward,
                                linearKg, linearRatio );
                    }
                    const double ratio = macKg > 1.0e-18 ? mass / macKg : 0.0;
                    const double meanRatio = meanMac > 1.0e-18 ? mass / meanMac : 0.0;
                    largestEndpointRatio = std::max( largestEndpointRatio, ratio );
                    largestMeanRatio = std::max( largestMeanRatio, meanRatio );
                    if( group.endpointOpen < 1.0e-3 ||
                        group.timeMeanOpen < 1.0e-3 || ratio > 20.0 )
                        std::printf( "face_group_detail,%d,%d,%d,%d,%d,%d,%d,"
                            "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
                            "%.12g,%.12g,%.12g,%.9g\n",
                            variant, frameIndex, substepIndex, group.isU ? 0 : 1,
                            group.x, group.y, group.events,
                            group.positiveMass, group.negativeMass,
                            group.endpointOpen, group.timeMeanOpen,
                            group.sweptForward, group.macForward, macKg,
                            meanMac, sweptMass, meanRatio );
                }
                if( std::abs( gross - applied[3] ) > 1.0e-10 )
                    fail( "unique physical faces do not reconcile all connected transfers" );
                if( linearVelocityAudit && !groupedFaces.empty() )
                    std::printf( "linear_face_summary,%d,%d,%d,%d,%.12g,%.12g,"
                        "%.12g,%d,%.12g,%.9g\n",
                        variant, frameIndex, substepIndex,
                        static_cast<int>( groupedFaces.size() ), gross,
                        linearProxyKg, endpointMacKg, opposedLinearFaces,
                        opposedLinearKg, largestLinearRatio );
                if( !groupedFaces.empty() )
                {
                    sawGroupedFace = true;
                    sawGroupedTinyFace |= tinyEndpointFaces > 0;
                    std::printf( "face_group_summary,%d,%d,%d,%d,%d,%d,%d,%d,"
                        "%.12g,%.12g,%.12g,%.12g,%.9g,%.9g\n",
                        variant, frameIndex, substepIndex,
                        static_cast<int>( groupedFaces.size() ), multiEventFaces,
                        oppositeMacFaces, tinyEndpointFaces, tinyTimeMeanFaces,
                        gross, endpointMacKg, meanMacKg, sweptKg,
                        largestEndpointRatio, largestMeanRatio );
                }
            }
            if( faceCapacityAudit && variant == 1 )
            {
                if( std::abs( faceCapacity.transferredMass -
                    applied[3] ) > 1.0e-10 ||
                    !std::isfinite( faceCapacity.transferredMass ) ||
                    faceCapacity.events < tracer.connectedResidualSources ||
                    faceCapacity.minAperture <= 0.0 )
                    fail( "connected residual face capacity ledger is inconsistent" );
                sawTinyAperture |= faceCapacity.tinyApertureEvents > 0;
                sawOpposingMacFlux |= faceCapacity.oppositeMacEvents > 0;
                if( faceCapacity.events > 0 )
                    std::printf( "face_capacity,%d,%d,%d,%d,%d,"
                        "%.12g,%.12g,%.12g,%.12g,%.12g,%d,%.12g,"
                        "%.9g,%.9g,%d,%d,%d,%.9g,%.12g,%.12g,%.12g\n",
                        variant, frameIndex, substepIndex,
                        faceCapacity.events, faceCapacity.oppositeMacEvents,
                        faceCapacity.oppositeMacMass, faceCapacity.oppositeSweptMass,
                        faceCapacity.transferredMass, faceCapacity.macEquivalentMass,
                        faceCapacity.sweptEquivalentMass, faceCapacity.tinyApertureEvents,
                        faceCapacity.smallApertureMass, faceCapacity.minAperture,
                        faceCapacity.maxPositiveMacRatio, faceCapacity.worstSourceX,
                        faceCapacity.worstSourceY, faceCapacity.worstDirection,
                        faceCapacity.worstAperture, faceCapacity.worstTransfer,
                        faceCapacity.worstMacOutwardVolume,
                        faceCapacity.worstSweptOutwardVolume );
            }
            if( std::abs( signedSum ) > 1.0e-9 ||
                std::abs( totalAfter - totalBefore ) > 1.0e-8 ||
                std::abs( applied[0] - tracer.sweptRemappedMass ) > 1.0e-5 ||
                std::abs( applied[1] - tracer.fallbackRemappedMass ) > 1.0e-5 ||
                std::abs( applied[3] - tracer.connectedResidualRemappedMass ) > 1.0e-5 ||
                std::abs( tracer.unresolvedAudit.totalMass -
                    tracer.unresolvedDisplacementMass ) > 1.0e-6 )
                fail( "World applied transfers or unresolved tracer accounting mismatch" );

            const double stencilBefore = stencilMass( before );
            const double stencilSwept = stencilMass( afterSwept );
            const double stencilMac = stencilMass( afterMac );
            if( std::abs( stencilSwept - stencilBefore - stencilBoundaryNet[0] ) > 1.0e-9 ||
                std::abs( stencilMac - stencilSwept - stencilBoundaryNet[1] ) > 1.0e-9 ||
                std::abs( afterSwept[target] - before[target] - sweptDelta[target] ) > 1.0e-9 ||
                std::abs( afterMac[target] - afterSwept[target] - macDelta[target] ) > 1.0e-9 )
                fail( "World source+neighbor stencil does not close its boundary tracer flux" );

            sawActualSwept |= applied[0] > 0.0;
            sawActualMac |= applied[2] > 0.0;
            sawUnresolved |= tracer.unresolvedAudit.totalMass > 0.0;
            sawConnectedResidual |= applied[3] > 0.0;
            const auto& pressure = field.getCutCellPressure();
            if( ( connectedExperiment || variant == 0 ) &&
                pressure.mergedPressureCells != 0 )
                fail( "unpooled tracer audit unexpectedly enabled pressure pooling" );
            if( !connectedExperiment && variant == 1 &&
                pressure.mergedPressureCells > 0 )
                sawPooledPressureCell = true;
            const auto volume = field.getCutCellVolumeBudget( targetX, targetY, fluidDt );
            if( !volume.valid || !volume.hasVolumeHistory || !volume.hasSweptHistory )
                fail( "World local geometry ledger missing matching moving-cut-cell history" );
            if( frameIndex == 5 || frameIndex == 6 || frameIndex == 19 ||
                tracer.unresolvedAudit.totalMass > 0.0 ||
                pressure.mergedPressureCells > 0 )
                std::printf( "%d,%d,%d,%d,%d,%d,%.9g,%.12g,%.12g,%.12g,"
                    "%.12g,%.12g,%.12g,%.12g,%.12g,"
                    "%.12g,%.12g,%.12g,%.12g,%d,%.12g,%d,%d,%d,"
                    "%.12g,%.12g,%.12g,%.12g\n",
                    connectedExperiment ? 1 : 0, variant, frameIndex, substepIndex,
                    pressure.mergedPressureCells,
                    static_cast<int>( pressure.coupledStatus ), volume.currentFraction,
                    before[target], afterSwept[target], afterMac[target],
                    stencilBefore, stencilSwept, stencilMac,
                    stencilBoundaryNet[0], stencilBoundaryNet[1],
                    applied[0], applied[1], applied[2], applied[3],
                    tracer.connectedResidualSources,
                    tracer.unresolvedAudit.totalMass, tracer.unresolvedAudit.sources,
                    tracer.unresolvedAudit.worstX, tracer.unresolvedAudit.worstY,
                    volume.snapshotDeltaVolume, volume.pressureSourceDeltaVolume,
                    volume.solidSweptDivergenceVolume, volume.macNetOutwardVolume );

            before.swap( afterMac );
            std::fill( sweptDelta.begin(), sweptDelta.end(), 0.0 );
            std::fill( macDelta.begin(), macDelta.end(), 0.0 );
            applied = {};
            stencilBoundaryNet = {};
            faceCapacity = {};
            groupedFaces.clear();
            sweptObserved = false;
            ++stepCount;
            ++substepIndex;
        } );

        for( int frame = 0; frame < frames; ++frame )
        {
            frameIndex = frame;
            substepIndex = 0;
            scene.step( dt, worldSubsteps );
            if( frame == seedAfterFrame )
            {
                int seedX = 17, seedY = 13;
                const fluidField2& field = *scene.getWindField();
                if( field.getCellFluidFraction( seedX, seedY ) <= 0.0f )
                {
                    float best = 0.0f;
                    for( const std::array<int, 2> point : {
                        std::array<int, 2>{ 18, 13 }, { 17, 12 }, { 16, 13 }, { 17, 14 } } )
                    {
                        const float fraction = field.getCellFluidFraction( point[0], point[1] );
                        if( fraction > best )
                        {
                            best = fraction;
                            seedX = point[0];
                            seedY = point[1];
                        }
                    }
                }
                if( field.getCellFluidFraction( seedX, seedY ) <= 0.0f )
                    fail( "World tracer audit found no open seed cell" );
                if( variant == 0 )
                {
                    commonSeedX = seedX;
                    commonSeedY = seedY;
                }
                if( commonSeedX != seedX || commonSeedY != seedY )
                    fail( "World pressure variants diverged before shared tracer seed" );

                scene.setWindTracerCellConcentration( seedX, seedY, 4.0f );
                const double seedMass = field.getTracerCellMass( seedX, seedY );
                if( variant == 0 ) commonSeedMass = seedMass;
                if( seedMass <= 0.0 || std::abs( seedMass - commonSeedMass ) > 1.0e-10 )
                    fail( "World pressure variants must share initial tracer mass" );
                for( int y = 0; y < height; ++y )
                    for( int x = 0; x < width; ++x )
                        before[y * width + x] = field.getTracerCellMass( x, y );
            }
            else if( frame > seedAfterFrame && substepIndex != fluidSteps )
                fail( "expected exactly four real Fluid steps per World frame" );
        }
        if( stepCount != fluidSteps * ( frames - seedAfterFrame - 1 ) ||
            sweptObserved || std::abs( scene.getWindField()->getTracerMass() -
                commonSeedMass ) > 1.0e-5 )
            fail( "World applied tracer audit skipped a step or lost mass" );
        scene.setWindTracerStageObserver( {} );
        scene.setWindTracerTransferObserver( {} );
    }
    if( faceCapacityAudit )
        std::fprintf( stderr, "face capacity coverage: tiny_aperture=%d opposite_mac=%d\n",
            sawTinyAperture ? 1 : 0, sawOpposingMacFlux ? 1 : 0 );
    if( !sawActualSwept || !sawActualMac || !sawUnresolved ||
        ( connectedExperiment ? !sawConnectedResidual : !sawPooledPressureCell ) ||
        ( faceCapacityAudit && !sawOpposingMacFlux ) ||
        ( groupedFaceAudit && ( !sawGroupedFace || !sawGroupedTinyFace ) ) ||
        ( linearVelocityAudit && ( !sawLinearProxy || !sawLinearProxyDifference ) ) )
    {
        std::fprintf( stderr, "real Cut-cell tracer audit did not exercise all required paths\n" );
        std::exit( 1 );
    }
}

// S2c independent reference: move one obstacle along an EXACTLY
// prescribed geometric trajectory, and solve the fluid at progressively
// smaller real dt. The reference is the sum of solved substep endpoint
// velocities coupled to sampled substep open geometry, not a guessed
// interpolation of the coarse solution. Even the finest run is not an
// analytic/continuous-fluid ground truth.
struct prescribedCutCellFaceResult
{
    int substeps = 0;
    int faceType = 0;
    int x = 0, y = 0; // Physical face segment indexed on the h=0.25 reference grid
    double timeOpen = 0.0; // Mean open time of that physical 0.25m face (s)
    double macEndpoint = 0.0;
    double macLinear = 0.0;
    double solidSwept = 0.0;
};

std::vector<prescribedCutCellFaceResult> auditPrescribedMovingCutCellSubsteps(
    bool convergenceAudit = false, bool rotating = false,
    float h = 0.25f, bool spatialAudit = false )
{
    constexpr float baseH = 0.25f;
    constexpr float totalDt = 1.0f / 30.0f;
    const int faceSegments = static_cast<int>( std::lround( baseH / h ) );
    const int width = static_cast<int>( std::lround( 8.0f / h ) );
    const int height = width;
    // A circle gives no changing geometry under rotation. Use an elongated
    // capsule for the pure-rotation trial so the Cut-cell interface genuinely
    // sweeps across U/V faces without a translating COM.
    const vec2 start = rotating ? vec2{ -0.20f, -0.12f } :
        vec2{ -0.38f, -0.10f };
    const vec2 finish = rotating ? start : vec2{ -0.14f, 0.06f };
    const vec2 motion = ( finish - start ) / totalDt;
    const float startAngle = rotating ? 0.18f : 0.0f;
    const float finishAngle = rotating ? 1.28f : 0.0f;
    const float angularSpeed = ( finishAngle - startAngle ) / totalDt;
    const std::vector<int> cadences = spatialAudit
        ? std::vector<int>{ 16, 32, 64, 128 }
        : convergenceAudit ? std::vector<int>{ 1, 2, 4, 8, 16, 32, 64 }
            : std::vector<int>{ 1, 2, 4, 8, 16 };
    struct monitoredFace
    {
        bool isU;
        int x, y;
    };
    // Faces near the moving-circle boundary, with one far-field control.
    constexpr std::array<monitoredFace, 10> faces{{
        { true, 15, 15 }, { true, 16, 16 }, { true, 17, 16 },
        { true, 18, 16 }, { true, 17, 17 },
        { false, 15, 16 }, { false, 16, 16 }, { false, 17, 16 },
        { false, 17, 17 }, { false, 3, 3 }
    }};
    struct integratedFace
    {
        double timeOpen = 0.0; // sec, integral of OPEN fraction dt
        double macEndpoint = 0.0; // unit-depth m^3: sum dt*h*openEnd*uEnd
        double macLinear = 0.0; // unit-depth m^3: sum dt*h*open(t)*linear u(t)
        double solidSwept = 0.0; // unit-depth m^3: geometric solid flux
        int movingSamples = 0;
    };
    const auto fail = []( const char* message )
    {
        std::fprintf( stderr, "prescribed Cut-cell reference: %s\n", message );
        std::exit( 1 );
    };
    if( rotating && !convergenceAudit )
        fail( "rotation is only supported by the independent extended convergence mode" );
    if( spatialAudit && ( !convergenceAudit ||
        ( faceSegments != 1 && faceSegments != 2 ) ||
        std::abs( faceSegments * h - baseH ) > 1.0e-6f ) )
        fail( "spatial audit needs physical h=0.25m face segments of h=0.25/0.125m" );
    if( spatialAudit )
    {
        std::puts( "prescribed_spatial,mode,cell_m,fluid_substeps,"
            "face_type,face_x,face_y,time_open_seconds,mac_endpoint_m3,"
            "mac_linear_m3,solid_swept_m3,reference_128_linear_m3,"
            "linear_delta_to_128_m3,open_time_delta_to_128_s" );
        std::puts( "prescribed_spatial_temporal_summary,mode,cell_m,fluid_substeps,"
            "faces,rms_linear_to_128_m3,rms_to_previous_m3,"
            "rms_open_time_to_128_s,sign_mismatch_to_128_faces" );
    }
    else if( convergenceAudit )
    {
        std::puts( "prescribed_convergence,mode,fluid_substeps,face_type,"
            "face_x,face_y,start_x,start_y,finish_x,finish_y,"
            "end_fluid_fraction,time_open_seconds,mac_endpoint_m3,"
            "mac_linear_m3,solid_swept_m3,reference_64_mac_linear_m3,"
            "coarse_to_ref_mac_linear_delta_m3,"
            "coarse_to_ref_endpoint_delta_m3,geometry_time_open_delta_seconds,"
            "start_angle_rad,finish_angle_rad" );
        std::puts( "prescribed_convergence_summary,mode,fluid_substeps,"
            "faces,rms_to_64_linear_m3,rms_to_previous_linear_m3,"
            "rms_to_64_open_time_s,max_abs_to_64_linear_m3,"
            "sign_mismatch_to_64_faces" );
    }
    else
        std::puts( "prescribed_face,fluid_substeps,face_type,face_x,face_y,"
            "start_x,start_y,finish_x,finish_y,end_fluid_fraction,"
            "time_open_seconds,mac_endpoint_m3,mac_linear_m3,solid_swept_m3,"
            "reference_16_mac_linear_m3,coarse_to_ref_mac_linear_delta_m3,"
            "coarse_to_ref_endpoint_delta_m3,geometry_time_open_delta_seconds" );

    std::vector<prescribedCutCellFaceResult> result;
    result.reserve( cadences.size() * faces.size() );
    std::vector<std::array<integratedFace, faces.size()>> history( cadences.size() );
    std::vector<float> endFraction( cadences.size() );
    std::vector<std::array<float, 4>> endGeometry( cadences.size() );
    bool observedGeometryMotion = false;
    bool observedVelocityHistoryDifference = false;
    for( std::size_t cadenceIndex = 0; cadenceIndex < cadences.size(); ++cadenceIndex )
    {
        const int steps = cadences[cadenceIndex];
        const float subDt = totalDt / static_cast<float>( steps );
        fluidField2 field{ width, height, h, { -4.0f, -4.0f } };
        field.setCutCellProjectionEnabled( true );
        field.setDisjointAnalyticCutCellEnabled( true );
        field.setSweptApertureAuditEnabled( true );
        if( field.isConnectedResidualTracerRemapEnabled() ||
            field.getSmallCellPressureMergeFraction() != 0.0f )
            fail( "reference unexpectedly enabled a solver or tracer experiment" );

        // Priming must use a motionless obstacle for *every* cadence.
        // Otherwise the cadence-dependent priming dt would apply different
        // boundary velocities before the reference interval even starts.
        fluidShapeObstacle2 circle{
            rotating ? shapeGeometry{ capsule2{
                { -0.63f, 0.0f }, { 0.63f, 0.0f }, 0.19f } } :
                shapeGeometry{ circle2{ {}, 0.58f } },
            { start, rot2::FromRadians( startAngle ) }, start,
            {}, 0.0f, 0.0f, 0.0f };
        std::array<fluidObstacleForce2, 1> feedback{};
        field.step( subDt, std::span<const fluidShapeObstacle2>( &circle, 1 ),
            feedback, 120 );
        if( field.hasSweptApertureAudit() )
            fail( "unchanged initial obstacle must not create a swept time history" );

        // Identical initial velocity for every cadence. No extra forcing is
        // applied in the interval: only different solved temporal resolutions.
        field.addVelocity( { -0.24f, -0.30f }, { 1.3f, 0.65f }, 0.65f );
        field.addVelocity( { 0.35f, 0.25f }, { -0.55f, 0.8f }, 0.60f );

        auto& integrated = history[cadenceIndex];
        for( int step = 0; step < steps; ++step )
        {
            const float alpha = static_cast<float>( step + 1 ) / steps;
            circle.transform.position = start + alpha * ( finish - start );
            circle.transform.rotation = rot2::FromRadians(
                startAngle + alpha * ( finishAngle - startAngle ) );
            circle.center = circle.transform.position;
            // Both COM linear and angular velocity come from the SAME
            // prescribed path for ALL cadences, not World feedback.
            circle.linearVelocity = motion;
            circle.angularVelocity = angularSpeed;
            field.step( subDt, std::span<const fluidShapeObstacle2>( &circle, 1 ),
                feedback, 120 );
            if( !field.hasSweptApertureAudit() ||
                !field.getSweptFaceFlux().usedSweptFlux )
                fail( "moving-step history was not observed" );

            // Match the SAME physical h=0.25m face segment at both spatial
            // resolutions. At h=0.125, this is two adjacent U/V subfaces.
            // Sum transport volumes, but average OPEN time by physical length.
            for( std::size_t i = 0; i < faces.size(); ++i )
            {
                const monitoredFace face = faces[i];
                integratedFace& value = integrated[i];
                for( int segment = 0; segment < faceSegments; ++segment )
                {
                    const int faceX = face.x * faceSegments +
                        ( face.isU ? 0 : segment );
                    const int faceY = face.y * faceSegments +
                        ( face.isU ? segment : 0 );
                    const float open = face.isU
                        ? field.getSweptUTimeMeanOpenFraction( faceX, faceY )
                        : field.getSweptVTimeMeanOpenFraction( faceX, faceY );
                    const double linear = face.isU
                        ? field.getSweptULinearVelocityProxyVolume( faceX, faceY, subDt )
                        : field.getSweptVLinearVelocityProxyVolume( faceX, faceY, subDt );
                    const int cellX = face.isU ? faceX - 1 : faceX;
                    const int cellY = face.isU ? faceY : faceY - 1;
                    const auto budget = field.getCutCellVolumeBudget( cellX, cellY, subDt );
                    if( !budget.valid || !budget.hasVolumeHistory ||
                        !budget.hasSweptHistory || !std::isfinite( open ) ||
                        open < 0.0f || open > 1.0f + 1.0e-5f ||
                        !std::isfinite( linear ) )
                        fail( "nonfinite or missing moving-face volume history" );
                    // Only the aperture time integral is length averaged:
                    // every m^3 flux has already integrated its own face h.
                    value.timeOpen += static_cast<double>( open ) * subDt / faceSegments;
                    value.macLinear += linear;
                    value.macEndpoint += budget.macOutwardVolume[face.isU ? 1 : 3];
                    value.solidSwept += budget.solidSweptOutwardVolume[face.isU ? 1 : 3];
                    if( open > 0.0f && open < 1.0f )
                        ++value.movingSamples;
                    if( !std::isfinite( value.timeOpen ) ||
                        !std::isfinite( value.macEndpoint ) ||
                        !std::isfinite( value.macLinear ) ||
                        !std::isfinite( value.solidSwept ) )
                        fail( "nonfinite accumulated moving-face volume" );
                    observedVelocityHistoryDifference |= steps > 1 &&
                        std::abs( linear - budget.macOutwardVolume[face.isU ? 1 : 3] ) >
                            1.0e-11;
                    observedGeometryMotion |= open > 0.0f && open < 1.0f;
                }
            }
        }

        if( std::abs( circle.center.x - finish.x ) > 1.0e-7f ||
            std::abs( circle.center.y - finish.y ) > 1.0e-7f ||
            std::abs( circle.transform.rotation.c - std::cos( finishAngle ) ) > 1.0e-6f ||
            std::abs( circle.transform.rotation.s - std::sin( finishAngle ) ) > 1.0e-6f )
            fail( "prescribed rigid obstacle did not end at the same pose" );
        endFraction[cadenceIndex] = field.getCellFluidFraction(
            17 * faceSegments, 16 * faceSegments );
        endGeometry[cadenceIndex] = {
            field.getCellFluidFraction( 15 * faceSegments, 15 * faceSegments ),
            field.getCellFluidFraction( 16 * faceSegments, 16 * faceSegments ),
            field.getCellFluidFraction( 17 * faceSegments, 16 * faceSegments ),
            field.getCellFluidFraction( 18 * faceSegments, 17 * faceSegments ) };
        for( std::size_t j = 0; j < endGeometry[cadenceIndex].size(); ++j )
            if( std::abs( endGeometry[cadenceIndex][j] - endGeometry[0][j] ) >
                2.0e-5f )
                fail( "identical final pose produced different Cut-cell geometry" );
        if( !std::isfinite( endFraction[cadenceIndex] ) ||
            std::abs( endFraction[cadenceIndex] - endFraction[0] ) > 2.0e-5f )
            fail( "equal prescribed final geometry changed across cadences" );
    }

    const auto& finest = history.back();
    bool referenceDiffers = false;
    for( std::size_t i = 0; i < faces.size(); ++i )
    {
        const auto face = faces[i];
        for( std::size_t k = 0; k < cadences.size(); ++k )
        {
            const auto& row = history[k][i];
            const auto& reference = finest[i];
            if( row.timeOpen < -1.0e-9 ||
                row.timeOpen > totalDt + 1.0e-6 ||
                std::abs( row.timeOpen - reference.timeOpen ) > 3.0e-3 )
                fail( "open geometry time integral diverged on matched trajectory" );

            referenceDiffers |= k == 0 &&
                std::abs( row.macLinear - reference.macLinear ) > 1.0e-9;
            result.push_back( { cadences[k], face.isU ? 0 : 1,
                face.x, face.y, row.timeOpen, row.macEndpoint,
                row.macLinear, row.solidSwept } );
            if( spatialAudit )
                std::printf( "prescribed_spatial,%s,%.9g,%d,%d,%d,%d,"
                    "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n",
                    rotating ? "rotation" : "translation", h,
                    cadences[k], face.isU ? 0 : 1, face.x, face.y,
                    row.timeOpen, row.macEndpoint, row.macLinear,
                    row.solidSwept, reference.macLinear,
                    row.macLinear - reference.macLinear,
                    row.timeOpen - reference.timeOpen );
            else if( convergenceAudit )
                std::printf( "prescribed_convergence,%s,%d,%d,%d,%d,"
                    "%.9g,%.9g,%.9g,%.9g,%.9g,"
                    "%.12g,%.12g,%.12g,%.12g,%.12g,"
                    "%.12g,%.12g,%.12g,%.9g,%.9g\n",
                    rotating ? "rotation" : "translation",
                    cadences[k], face.isU ? 0 : 1, face.x, face.y,
                    start.x, start.y, finish.x, finish.y,
                    endFraction[k],
                    row.timeOpen, row.macEndpoint, row.macLinear, row.solidSwept,
                    reference.macLinear,
                    row.macLinear - reference.macLinear,
                    row.macEndpoint - reference.macEndpoint,
                    row.timeOpen - reference.timeOpen, startAngle, finishAngle );
            else
                std::printf( "prescribed_face,%d,%d,%d,%d,"
                    "%.9g,%.9g,%.9g,%.9g,%.9g,"
                    "%.12g,%.12g,%.12g,%.12g,%.12g,"
                    "%.12g,%.12g,%.12g\n",
                    cadences[k], face.isU ? 0 : 1, face.x, face.y,
                    start.x, start.y, finish.x, finish.y,
                    endFraction[k],
                    row.timeOpen, row.macEndpoint, row.macLinear, row.solidSwept,
                    reference.macLinear,
                    row.macLinear - reference.macLinear,
                    row.macEndpoint - reference.macEndpoint,
                    row.timeOpen - reference.timeOpen );
        }
    }
    if( convergenceAudit )
        for( std::size_t k = 0; k < cadences.size(); ++k )
        {
            double squaredReference = 0.0, squaredPrevious = 0.0;
            double squaredOpen = 0.0, peakReference = 0.0;
            int differentSigns = 0;
            for( std::size_t i = 0; i < faces.size(); ++i )
            {
                const double difference = history[k][i].macLinear - finest[i].macLinear;
                squaredReference += difference * difference;
                peakReference = std::max( peakReference, std::abs( difference ) );
                const double geometry = history[k][i].timeOpen - finest[i].timeOpen;
                squaredOpen += geometry * geometry;
                if( k > 0 )
                {
                    const double previous = history[k - 1][i].macLinear -
                        history[k][i].macLinear;
                    squaredPrevious += previous * previous;
                }
                if( history[k][i].macLinear * finest[i].macLinear < 0.0 )
                    ++differentSigns;
            }
            if( spatialAudit )
                std::printf( "prescribed_spatial_temporal_summary,%s,%.9g,%d,%zu,"
                    "%.12g,%.12g,%.12g,%d\n",
                    rotating ? "rotation" : "translation", h, cadences[k], faces.size(),
                    std::sqrt( squaredReference / faces.size() ),
                    k > 0 ? std::sqrt( squaredPrevious / faces.size() ) : 0.0,
                    std::sqrt( squaredOpen / faces.size() ), differentSigns );
            else
                std::printf( "prescribed_convergence_summary,%s,%d,%zu,"
                    "%.12g,%.12g,%.12g,%.12g,%d\n",
                    rotating ? "rotation" : "translation", cadences[k], faces.size(),
                    std::sqrt( squaredReference / faces.size() ),
                    k > 0 ? std::sqrt( squaredPrevious / faces.size() ) : 0.0,
                    std::sqrt( squaredOpen / faces.size() ),
                    peakReference, differentSigns );
        }
    if( !observedGeometryMotion || !observedVelocityHistoryDifference ||
        !referenceDiffers )
        fail( "reference did not exercise moving aperture and solved intermediate velocities" );
    return result;
}

// S2c physical-face forensic, no solver modifications. On an identical
// prescribed rotating capsule, observe the SAME physical V(16,16) segment
// at both h=0.25 and h=0.125, with 64 and 128 real Fluid steps. The
// pre-pressure observer samples the current field after advection/boundary
// application. Its velocity is NOT the beginning-of-frame velocity.
// The post-pressure result is sampled after the normal projection.
void auditPrescribedRotatingFacePressureSplit()
{
    constexpr float totalDt = 1.0f / 30.0f;
    constexpr vec2 center{ -0.20f, -0.12f };
    constexpr float startAngle = 0.18f, endAngle = 1.28f;
    constexpr float angularSpeed = ( endAngle - startAngle ) / totalDt;
    constexpr float physicalFaceLength = 0.25f;
    const auto fail = []( const char* message )
    {
        std::fprintf( stderr, "rotation pressure split: %s\n", message );
        std::exit( 1 );
    };
    struct faceLedger
    {
        float h = 0.0f;
        int steps = 0;
        double preLinear = 0.0;
        double pressureLinear = 0.0;
        double postLinear = 0.0;
        double preEndpoint = 0.0;
        double pressureEndpoint = 0.0;
        double postEndpoint = 0.0;
        double sweptSolid = 0.0;
        double openTime = 0.0;
        double endpointOpenTime = 0.0;
        double peakAbsPressureLinear = 0.0;
        int nonzeroPressureSteps = 0;
    };
    std::array<faceLedger, 4> ledgers{};
    std::puts( "rotation_pressure_substep,cell_m,fluid_substeps,substep,"
        "angle_rad,physical_face_length_m,pre_projection_mac_m3,"
        "post_projection_mac_m3,pressure_endpoint_delta_m3,"
        "pre_projection_linear_proxy_m3,post_projection_linear_proxy_m3,"
        "pressure_linear_proxy_delta_m3,time_mean_open,endpoint_open,"
        "solid_swept_m3" );
    std::puts( "rotation_pressure_cross,fluid_substeps,coarse_h,fine_h,"
        "coarse_pre_linear_m3,fine_pre_linear_m3,"
        "coarse_pressure_linear_m3,fine_pressure_linear_m3,"
        "coarse_post_linear_m3,fine_post_linear_m3,"
        "coarse_pre_mac_m3,fine_pre_mac_m3,"
        "coarse_pressure_mac_m3,fine_pressure_mac_m3,"
        "coarse_post_mac_m3,fine_post_mac_m3,"
        "coarse_solid_swept_m3,fine_solid_swept_m3,"
        "coarse_open_time_s,fine_open_time_s,"
        "geometry_pre_linear_delta_m3,velocity_pre_linear_delta_m3,"
        "pressure_linear_delta_m3,total_linear_delta_m3,"
        "pre_mac_delta_m3,pressure_mac_delta_m3,total_mac_delta_m3" );

    for( int grid = 0; grid < 2; ++grid )
        for( int resolution = 0; resolution < 2; ++resolution )
        {
            const float h = grid == 0 ? 0.25f : 0.125f;
            const int segments = grid == 0 ? 1 : 2;
            const int steps = resolution == 0 ? 64 : 128;
            const float subDt = totalDt / steps;
            faceLedger& ledger = ledgers[grid * 2 + resolution];
            ledger.h = h;
            ledger.steps = steps;
            fluidField2 field{ 32 * segments, 32 * segments, h, { -4.0f, -4.0f } };
            field.setCutCellProjectionEnabled( true );
            field.setDisjointAnalyticCutCellEnabled( true );
            field.setSweptApertureAuditEnabled( true );
            if( field.getSmallCellPressureMergeFraction() != 0.0f ||
                field.isConnectedResidualTracerRemapEnabled() )
                fail( "experimental physics must remain disabled" );

            fluidShapeObstacle2 capsule{
                capsule2{ { -0.63f, 0.0f }, { 0.63f, 0.0f }, 0.19f },
                { center, rot2::FromRadians( startAngle ) },
                center, {}, 0.0f, 0.0f, 0.0f
            };
            std::array<fluidObstacleForce2, 1> feedback{};
            field.step( subDt, std::span<const fluidShapeObstacle2>( &capsule, 1 ),
                feedback, 120 );
            if( field.hasSweptApertureAudit() )
                fail( "unchanged initial shape must not create swept history" );
            field.addVelocity( { -0.24f, -0.30f }, { 1.3f, 0.65f }, 0.65f );
            field.addVelocity( { 0.35f, 0.25f }, { -0.55f, 0.8f }, 0.60f );

            std::array<double, 2> preLinear{}, preEndpoint{};
            bool observedPrePressure = false;
            field.setPrePressureSnapshotObserver( [&]( const fluidField2& snapshot )
            {
                if( !snapshot.hasSweptApertureAudit() || observedPrePressure )
                    fail( "missing or duplicated pre-pressure moving-face observer" );
                observedPrePressure = true;
                for( int segment = 0; segment < segments; ++segment )
                {
                    const int faceX = 16 * segments + segment;
                    const int faceY = 16 * segments;
                    const auto budget = snapshot.getCutCellVolumeBudget(
                        faceX, faceY - 1, subDt );
                    if( !budget.valid || !budget.hasSweptHistory )
                        fail( "pre-pressure face has no current swept geometry" );
                    preLinear[segment] = snapshot.getSweptVLinearVelocityProxyVolume(
                        faceX, faceY, subDt );
                    preEndpoint[segment] = budget.macOutwardVolume[3];
                }
            } );
            for( int step = 0; step < steps; ++step )
            {
                observedPrePressure = false;
                const float alpha = static_cast<float>( step + 1 ) / steps;
                capsule.transform.rotation = rot2::FromRadians(
                    startAngle + alpha * ( endAngle - startAngle ) );
                capsule.angularVelocity = angularSpeed;
                field.step( subDt, std::span<const fluidShapeObstacle2>( &capsule, 1 ),
                    feedback, 120 );
                if( !observedPrePressure || !field.hasSweptApertureAudit() )
                    fail( "missing pressure stage or moving-face time samples" );

                double preLinearStep = 0.0, postLinearStep = 0.0;
                double preEndpointStep = 0.0, postEndpointStep = 0.0;
                double sweptStep = 0.0, meanOpen = 0.0, endpointOpen = 0.0;
                for( int segment = 0; segment < segments; ++segment )
                {
                    const int faceX = 16 * segments + segment;
                    const int faceY = 16 * segments;
                    const auto budget = field.getCutCellVolumeBudget(
                        faceX, faceY - 1, subDt );
                    if( !budget.valid || !budget.hasVolumeHistory ||
                        !budget.hasSweptHistory )
                        fail( "post-pressure face has invalid swept/volume history" );
                    preLinearStep += preLinear[segment];
                    preEndpointStep += preEndpoint[segment];
                    postLinearStep += field.getSweptVLinearVelocityProxyVolume(
                        faceX, faceY, subDt );
                    postEndpointStep += budget.macOutwardVolume[3];
                    sweptStep += budget.solidSweptOutwardVolume[3];
                    meanOpen += field.getSweptVTimeMeanOpenFraction(
                        faceX, faceY ) / segments;
                    endpointOpen += field.getVFaceOpenFraction(
                        faceX, faceY ) / segments;
                }
                const double pressureLinearStep = postLinearStep - preLinearStep;
                const double pressureEndpointStep = postEndpointStep - preEndpointStep;
                if( !std::isfinite( preLinearStep ) ||
                    !std::isfinite( postLinearStep ) ||
                    !std::isfinite( pressureLinearStep ) ||
                    !std::isfinite( preEndpointStep ) ||
                    !std::isfinite( postEndpointStep ) ||
                    !std::isfinite( pressureEndpointStep ) ||
                    !std::isfinite( sweptStep ) ||
                    !( meanOpen >= 0.0 && meanOpen <= 1.0 + 1.0e-5 ) ||
                    !( endpointOpen >= 0.0 && endpointOpen <= 1.0 + 1.0e-5 ) )
                    fail( "nonfinite or invalid staged face flux" );

                ledger.preLinear += preLinearStep;
                ledger.postLinear += postLinearStep;
                ledger.pressureLinear += pressureLinearStep;
                ledger.preEndpoint += preEndpointStep;
                ledger.postEndpoint += postEndpointStep;
                ledger.pressureEndpoint += pressureEndpointStep;
                ledger.sweptSolid += sweptStep;
                ledger.openTime += meanOpen * subDt;
                ledger.endpointOpenTime += endpointOpen * subDt;
                ledger.peakAbsPressureLinear = std::max(
                    ledger.peakAbsPressureLinear, std::abs( pressureLinearStep ) );
                ledger.nonzeroPressureSteps += std::abs( pressureLinearStep ) > 1.0e-12;
                std::printf( "rotation_pressure_substep,%.9g,%d,%d,%.9g,"
                    "%.9g,%.12g,%.12g,%.12g,%.12g,%.12g,"
                    "%.12g,%.12g,%.12g,%.12g\n",
                    h, steps, step, startAngle + alpha * ( endAngle - startAngle ),
                    physicalFaceLength, preEndpointStep, postEndpointStep,
                    pressureEndpointStep, preLinearStep, postLinearStep,
                    pressureLinearStep, meanOpen, endpointOpen, sweptStep );
            }
            if( !ledger.nonzeroPressureSteps ||
                std::abs( ledger.postLinear -
                    ( ledger.preLinear + ledger.pressureLinear ) ) > 1.0e-9 ||
                std::abs( ledger.postEndpoint -
                    ( ledger.preEndpoint + ledger.pressureEndpoint ) ) > 1.0e-9 )
                fail( "invalid or unexercised pressure stage decomposition" );
        }

    for( int resolution = 0; resolution < 2; ++resolution )
    {
        const faceLedger& coarse = ledgers[resolution];
        const faceLedger& fine = ledgers[2 + resolution];
        if( coarse.steps != fine.steps ||
            std::abs( coarse.h - 0.25f ) > 1.0e-6f ||
            std::abs( fine.h - 0.125f ) > 1.0e-6f )
            fail( "comparison does not refer to same physical face/time interval" );

        const double coarseExposure = physicalFaceLength * coarse.openTime;
        const double fineExposure = physicalFaceLength * fine.openTime;
        if( coarseExposure <= 0.0 || fineExposure <= 0.0 )
            fail( "selected physical face never opened in one reference run" );
        const double effectiveCoarseVelocity = coarse.preLinear / coarseExposure;
        const double effectiveFineVelocity = fine.preLinear / fineExposure;
        // Exact algebra, not a causal inference about the pressure solver:
        // Δpost = Δpre + Δpressure, while
        // Δpre = (Ec - Ef) * uEffF + Ec * (uEffC - uEffF).
        const double geometryDelta = ( coarseExposure - fineExposure ) *
            effectiveFineVelocity;
        const double preVelocityDelta = coarseExposure *
            ( effectiveCoarseVelocity - effectiveFineVelocity );
        const double pressureDelta = coarse.pressureLinear - fine.pressureLinear;
        const double totalLinearDelta = coarse.postLinear - fine.postLinear;
        const double preMacDelta = coarse.preEndpoint - fine.preEndpoint;
        const double pressureMacDelta =
            coarse.pressureEndpoint - fine.pressureEndpoint;
        const double totalMacDelta = coarse.postEndpoint - fine.postEndpoint;
        const double tolerance = 1.0e-9;
        if( std::abs( geometryDelta + preVelocityDelta + pressureDelta -
            totalLinearDelta ) > tolerance ||
            std::abs( preMacDelta + pressureMacDelta - totalMacDelta ) >
            tolerance || !std::isfinite( effectiveCoarseVelocity ) ||
            !std::isfinite( effectiveFineVelocity ) )
            fail( "algebraic pressure/geometry/velocity closure violated" );
        std::printf( "rotation_pressure_cross,%d,0.25,0.125,"
            "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
            "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
            "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
            "%.12g,%.12g,%.12g,%.12g,%.12g\n",
            coarse.steps,
            coarse.preLinear, fine.preLinear,
            coarse.pressureLinear, fine.pressureLinear,
            coarse.postLinear, fine.postLinear,
            coarse.preEndpoint, fine.preEndpoint,
            coarse.pressureEndpoint, fine.pressureEndpoint,
            coarse.postEndpoint, fine.postEndpoint,
            coarse.sweptSolid, fine.sweptSolid,
            coarse.openTime, fine.openTime,
            geometryDelta, preVelocityDelta, pressureDelta, totalLinearDelta,
            preMacDelta, pressureMacDelta, totalMacDelta );
    }
}

// S2c same-state pressure ONLY experiment. The frozen pre-pressure field
// includes geometry, all MAC velocities after advection/boundaries, swept
// history, and the captured obstacle. Three independent deep COPIES get
// different Jacobi iteration budgets. Never advance those copies or write
// their projected velocities back to the real World/Fluid history.
// The two grid sizes are NOT an identical discretized state; comparisons
// across h retain the previously observed spatial/history difference.
void auditRotatingFrozenPressureIterationSweep()
{
    constexpr float totalDt = 1.0f / 30.0f;
    constexpr vec2 center{ -0.20f, -0.12f };
    constexpr float startAngle = 0.18f, endAngle = 1.28f;
    constexpr float angularSpeed = ( endAngle - startAngle ) / totalDt;
    constexpr std::array<int, 3> iterations{{ 40, 120, 200 }};
    constexpr std::array<int, 3> checkpointNumerators{{ 11, 12, 16 }};
    const auto fail = []( const char* message )
    {
        std::fprintf( stderr, "frozen rotating pressure replay: %s\n", message );
        std::exit( 1 );
    };
    std::puts( "rotation_frozen_pressure,cell_m,fluid_substeps,"
        "checkpoint_numerator_16,angle_rad,iterations,"
        "pre_endpoint_m3,post_endpoint_m3,pressure_endpoint_m3,"
        "pre_linear_proxy_m3,post_linear_proxy_m3,pressure_linear_proxy_m3,"
        "pressure_divergence_after,pressure_residual,"
        "time_mean_open,solid_swept_m3" );
    std::puts( "rotation_frozen_pressure_cross,fluid_substeps,"
        "checkpoint_numerator_16,iterations,"
        "coarse_pre_linear_m3,fine_pre_linear_m3,"
        "coarse_pressure_linear_m3,fine_pressure_linear_m3,"
        "coarse_post_linear_m3,fine_post_linear_m3,"
        "coarse_pre_endpoint_m3,fine_pre_endpoint_m3,"
        "coarse_pressure_endpoint_m3,fine_pressure_endpoint_m3,"
        "coarse_post_endpoint_m3,fine_post_endpoint_m3,"
        "post_linear_delta_m3,pressure_linear_delta_m3,"
        "pre_linear_delta_m3" );
    struct caseResult
    {
        int steps = 0, checkpoint = 0, iterations = 0;
        float h = 0.0f;
        double preEndpoint = 0.0, postEndpoint = 0.0;
        double preLinear = 0.0, postLinear = 0.0;
        double divergenceAfter = 0.0, pressureResidual = 0.0;
        double openTime = 0.0, swept = 0.0;
    };
    std::vector<caseResult> cases;
    cases.reserve( 2 * 2 * 3 * iterations.size() );
    bool observedPressureCorrection = false;
    bool observedIterationSensitivity = false;

    for( float h : { 0.25f, 0.125f } )
        for( int steps : { 64, 128 } )
        {
            const int segments = static_cast<int>( std::lround( 0.25f / h ) );
            const float subDt = totalDt / steps;
            fluidField2 field{ 32 * segments, 32 * segments, h, { -4.0f, -4.0f } };
            field.setCutCellProjectionEnabled( true );
            field.setDisjointAnalyticCutCellEnabled( true );
            field.setSweptApertureAuditEnabled( true );
            if( field.getSmallCellPressureMergeFraction() > 0.0f ||
                field.isConnectedResidualTracerRemapEnabled() )
                fail( "experimental solver/tracer flags must stay off" );
            fluidShapeObstacle2 capsule{
                capsule2{ { -0.63f, 0.0f }, { 0.63f, 0.0f }, 0.19f },
                { center, rot2::FromRadians( startAngle ) }, center,
                {}, 0.0f, 0.0f, 0.0f
            };
            std::array<fluidObstacleForce2, 1> feedback{};
            field.step( subDt, std::span<const fluidShapeObstacle2>( &capsule, 1 ),
                feedback, 120 );
            field.addVelocity( { -0.24f, -0.30f }, { 1.3f, 0.65f }, 0.65f );
            field.addVelocity( { 0.35f, 0.25f }, { -0.55f, 0.8f }, 0.60f );
            bool captured = false;
            int currentStep = -1;

            // At 11/16, 12/16 and end of the exact same trajectory, freeze
            // the live PRE-projection state and replay a pressure-only solve
            // on three independent copies. The live field keeps 120 iters.
            field.setPrePressureSnapshotObserver( [&]( const fluidField2& frozen )
            {
                const int stepOneBased = currentStep + 1;
                const int checkpoint = stepOneBased * 16 / steps;
                if( stepOneBased * 16 != checkpoint * steps ||
                    std::find( checkpointNumerators.begin(),
                        checkpointNumerators.end(), checkpoint ) ==
                        checkpointNumerators.end() )
                    return;
                if( captured || !frozen.hasSweptApertureAudit() )
                    fail( "same-state snapshot missing or observed twice" );
                captured = true;
                double preEndpoint = 0.0, preLinear = 0.0;
                double openTime = 0.0, swept = 0.0;
                for( int segment = 0; segment < segments; ++segment )
                {
                    const int faceX = 16 * segments + segment;
                    const int faceY = 16 * segments;
                    const auto budget = frozen.getCutCellVolumeBudget(
                        faceX, faceY - 1, subDt );
                    if( !budget.valid || !budget.hasSweptHistory )
                        fail( "frozen pressure face geometry invalid" );
                    preEndpoint += budget.macOutwardVolume[3];
                    preLinear += frozen.getSweptVLinearVelocityProxyVolume(
                        faceX, faceY, subDt );
                    openTime += frozen.getSweptVTimeMeanOpenFraction(
                        faceX, faceY ) * subDt / segments;
                    swept += budget.solidSweptOutwardVolume[3];
                }
                double precedingLinear = 0.0;
                for( int solverIterations : iterations )
                {
                    fluidField2 copy = frozen;
                    copy.setPrePressureSnapshotObserver( {} );
                    const auto applied = copy.replayFrozenCutCellPressure(
                        solverIterations );
                    if( !applied.has_value() ||
                        copy.replayFrozenCutCellPressure().has_value() )
                        fail( "frozen pressure replay not one-use or rejected budget" );
                    double postEndpoint = 0.0, postLinear = 0.0;
                    for( int segment = 0; segment < segments; ++segment )
                    {
                        const int faceX = 16 * segments + segment;
                        const int faceY = 16 * segments;
                        const auto budget = copy.getCutCellVolumeBudget(
                            faceX, faceY - 1, subDt );
                        postEndpoint += budget.macOutwardVolume[3];
                        postLinear += copy.getSweptVLinearVelocityProxyVolume(
                            faceX, faceY, subDt );
                    }
                    const auto& pressure = copy.getCutCellPressure();
                    if( !std::isfinite( postEndpoint ) ||
                        !std::isfinite( postLinear ) ||
                        !std::isfinite( pressure.divergenceAfter ) ||
                        !std::isfinite( pressure.pressureResidual ) )
                        fail( "nonfinite pressure-only replay" );
                    observedPressureCorrection |=
                        std::abs( postEndpoint - preEndpoint ) > 1.0e-12;
                    observedIterationSensitivity |= solverIterations > iterations.front() &&
                        std::abs( postLinear - precedingLinear ) > 1.0e-12;
                    precedingLinear = postLinear;
                    cases.push_back( { steps, checkpoint, solverIterations, h,
                        preEndpoint, postEndpoint, preLinear, postLinear,
                        pressure.divergenceAfter, pressure.pressureResidual,
                        openTime, swept } );
                    std::printf( "rotation_frozen_pressure,%.9g,%d,%d,%.9g,%d,"
                        "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
                        "%.12g,%.12g,%.12g,%.12g\n",
                        h, steps, checkpoint,
                        startAngle + static_cast<float>( stepOneBased ) / steps *
                            ( endAngle - startAngle ),
                        solverIterations, preEndpoint, postEndpoint,
                        postEndpoint - preEndpoint,
                        preLinear, postLinear, postLinear - preLinear,
                        pressure.divergenceAfter, pressure.pressureResidual,
                        openTime, swept );
                }
            } );
            for( int step = 0; step < steps; ++step )
            {
                currentStep = step;
                captured = false;
                const float alpha = static_cast<float>( step + 1 ) / steps;
                capsule.transform.rotation = rot2::FromRadians(
                    startAngle + alpha * ( endAngle - startAngle ) );
                capsule.angularVelocity = angularSpeed;
                field.step( subDt, std::span<const fluidShapeObstacle2>( &capsule, 1 ),
                    feedback, 120 );
                const int one = step + 1;
                const int numerator = one * 16 / steps;
                const bool checkpoint = one * 16 == numerator * steps &&
                    std::find( checkpointNumerators.begin(),
                        checkpointNumerators.end(), numerator ) !=
                        checkpointNumerators.end();
                if( captured != checkpoint )
                    fail( "frozen snapshot mismatch at a prescribed timestep" );
                if( checkpoint )
                {
                    const caseResult* baseline = nullptr;
                    for( const auto& r : cases )
                        if( r.steps == steps && r.h == h &&
                            r.checkpoint == numerator && r.iterations == 120 )
                            baseline = &r;
                    if( baseline == nullptr )
                        fail( "120-iteration baseline was not captured" );
                    double liveEndpoint = 0.0, liveLinear = 0.0;
                    for( int segment = 0; segment < segments; ++segment )
                    {
                        const int x = 16 * segments + segment;
                        const int y = 16 * segments;
                        liveEndpoint += field.getCutCellVolumeBudget(
                            x, y - 1, subDt ).macOutwardVolume[3];
                        liveLinear += field.getSweptVLinearVelocityProxyVolume(
                            x, y, subDt );
                    }
                    if( std::abs( liveEndpoint - baseline->postEndpoint ) > 1.0e-8 ||
                        std::abs( liveLinear - baseline->postLinear ) > 1.0e-8 )
                        fail( "copy-replayed pressure differs from actual live 120-iteration Step" );
                }
            }
        }

    if( cases.size() != 36 ||
        !observedPressureCorrection || !observedIterationSensitivity )
        fail( "insufficient controlled pressure replay coverage" );
    for( int steps : { 64, 128 } )
        for( int checkpoint : checkpointNumerators )
            for( int budget : iterations )
            {
                const caseResult *coarse = nullptr, *fine = nullptr;
                for( const auto& row : cases )
                    if( row.steps == steps && row.checkpoint == checkpoint &&
                        row.iterations == budget )
                    {
                        if( row.h == 0.25f ) coarse = &row;
                        else if( row.h == 0.125f ) fine = &row;
                    }
                if( !coarse || !fine )
                    fail( "missing matched physical face and time checkpoint" );
                const double preDifference = coarse->preLinear - fine->preLinear;
                const double pressureDifference =
                    ( coarse->postLinear - coarse->preLinear ) -
                    ( fine->postLinear - fine->preLinear );
                const double postDifference = coarse->postLinear - fine->postLinear;
                if( std::abs( preDifference + pressureDifference -
                    postDifference ) > 1.0e-10 )
                    fail( "matched frozen pressure ledger does not close" );
                std::printf( "rotation_frozen_pressure_cross,%d,%d,%d,"
                    "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
                    "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
                    "%.12g,%.12g,%.12g\n",
                    steps, checkpoint, budget,
                    coarse->preLinear, fine->preLinear,
                    coarse->postLinear - coarse->preLinear,
                    fine->postLinear - fine->preLinear,
                    coarse->postLinear, fine->postLinear,
                    coarse->preEndpoint, fine->preEndpoint,
                    coarse->postEndpoint - coarse->preEndpoint,
                    fine->postEndpoint - fine->preEndpoint,
                    coarse->postEndpoint, fine->postEndpoint,
                    postDifference, pressureDifference, preDifference );
            }
}

// S2c controlled short restart. "Matched" means the SAME continuous
// analytic velocity function sampled at the two staggered MAC grids, NOT
// identical arrays or a mathematically exact cross-grid restriction.
// Geometry, swept displacement and solid velocity remain independently
// discretized. The injected state is copied from frozen pre-pressure fields,
// then replayed through their actual normal pressure solver. In contrast,
// history resumes from the untouched frozen state. Both advance 1/2/4
// actual following Fluid steps with the SAME prescribed capsule motion.
void auditMatchedWorldMacPressureRestart( bool probeZeroMac = false,
    bool sweepZeroMacJacobi = false )
{
    constexpr float totalDt = 1.0f / 30.0f;
    constexpr float startAngle = 0.18f, endAngle = 1.28f;
    constexpr vec2 center{ -0.20f, -0.12f };
    constexpr float angularSpeed = ( endAngle - startAngle ) / totalDt;
    constexpr std::array<int, 2> checkpoints{{ 11, 12 }};
    constexpr std::array<int, 4> horizons{{ 0, 1, 2, 4 }};
    const auto fail = []( const char* message )
    {
        std::fprintf( stderr, "matched-world MAC pressure restart: %s\n", message );
        std::exit( 1 );
    };
    // Smooth, exactly divergence-free affine velocity on R². On every
    // OPEN MAC face both grids sample this SAME world-coordinate function;
    // closed faces retain their independently sampled solid boundary speed.
    const auto worldVelocity = []( vec2 p ) -> vec2
    {
        return { 0.30f - 0.08f * ( p.y + 0.12f ),
            0.42f + 0.12f * ( p.x + 0.20f ) };
    };
    struct result
    {
        float h = 0.0f;
        int fluidSteps = 0, checkpoint = 0, horizon = 0;
        double naturalPre = 0.0, matchedPre = 0.0;
        double naturalPost = 0.0, matchedPost = 0.0;
        double openTime = 0.0, sweptVolume = 0.0;
    };
    std::vector<result> results;
    results.reserve( 2 * 2 * checkpoints.size() * horizons.size() );
    struct zeroResult
    {
        float h = 0.0f;
        int fluidSteps = 0, checkpoint = 0;
        double pre = 0.0, post = 0.0;
        double openTime = 0.0, sweptVolume = 0.0;
        float residual = 0.0f;
    };
    std::vector<zeroResult> zeroResults;
    struct jacobiResult
    {
        float h = 0.0f;
        int fluidSteps = 0, checkpoint = 0, iterations = 0;
        double post = 0.0, residual = 0.0;
        double sourceRate = 0.0, compatibility = 0.0;
        double localSource = 0.0, localSnapshot = 0.0, localSwept = 0.0;
        double localAfter = 0.0, localResidual = 0.0;
        int regions = 0;
    };
    std::vector<jacobiResult> jacobiResults;
    if( sweepZeroMacJacobi )
    {
        jacobiResults.reserve( 2 * 2 * checkpoints.size() * 4 );
        std::puts( "zero_mac_jacobi,cell_m,fluid_substeps,checkpoint_16,"
            "jacobi_iterations,post_face_m3,pressure_residual,divergence_after,"
            "source_imbalance_m3_per_s,compatibility_m3_per_s,regions,"
            "local_pressure_source_m3,local_snapshot_m3,local_swept_divergence_m3,"
            "local_post_mac_outward_m3,local_post_continuity_m3,"
            "source_ledger_error_m3_per_s,compatibility_ledger_error_m3_per_s" );
        std::puts( "zero_mac_jacobi_cross,fluid_substeps,checkpoint_16,"
            "jacobi_iterations,coarse_minus_fine_post_m3,"
            "coarse_minus_fine_residual,delta_gap_from_200_m3" );
    }
    if( probeZeroMac )
    {
        zeroResults.reserve( 2 * 2 * checkpoints.size() );
        std::puts( "zero_mac_pressure,cell_m,fluid_substeps,checkpoint_16,"
            "zero_pre_m3,zero_post_m3,zero_projection_delta_m3,"
            "pressure_residual,time_mean_open_seconds,solid_swept_m3" );
        std::puts( "zero_mac_cross,fluid_substeps,checkpoint_16,"
            "affine_pre_gap_m3,zero_pre_gap_m3,"
            "affine_projection_gap_m3,zero_projection_gap_m3,"
            "velocity_dependent_projection_gap_m3,zero_post_gap_m3" );
    }
    std::puts( "matched_world_restart,cell_m,fluid_substeps,"
        "checkpoint_16,horizon_steps,angle_rad,"
        "captured_evolved_pre_m3,captured_matched_world_pre_m3,"
        "evolved_post_m3,matched_world_post_m3,"
        "time_mean_open_seconds,solid_swept_m3" );
    std::puts( "matched_world_cross,fluid_substeps,checkpoint_16,"
        "horizon_steps,evolved_coarse_minus_fine_m3,"
        "matched_coarse_minus_fine_m3,"
        "history_excess_gap_m3,captured_evolved_pre_gap_m3,"
        "captured_matched_pre_gap_m3" );

    for( float h : { 0.25f, 0.125f } )
        for( int fluidSteps : { 64, 128 } )
        {
            const int segments = static_cast<int>( std::lround( 0.25f / h ) );
            const float dt = totalDt / fluidSteps;
            fluidField2 field{ 32 * segments, 32 * segments, h, { -4.0f, -4.0f } };
            field.setCutCellProjectionEnabled( true );
            field.setDisjointAnalyticCutCellEnabled( true );
            field.setSweptApertureAuditEnabled( true );
            if( field.isConnectedResidualTracerRemapEnabled() ||
                field.getSmallCellPressureMergeFraction() != 0.0f )
                fail( "experimental production physics unexpectedly enabled" );
            fluidShapeObstacle2 capsule{
                capsule2{ { -0.63f, 0.0f }, { 0.63f, 0.0f }, 0.19f },
                { center, rot2::FromRadians( startAngle ) }, center,
                {}, 0.0f, 0.0f, 0.0f };
            std::array<fluidObstacleForce2, 1> feedback{};
            field.step( dt, std::span<const fluidShapeObstacle2>( &capsule, 1 ),
                feedback, 120 );
            field.addVelocity( { -0.24f, -0.30f }, { 1.3f, 0.65f }, 0.65f );
            field.addVelocity( { 0.35f, 0.25f }, { -0.55f, 0.8f }, 0.60f );

            // Every row reads the SAME 0.25m world-space V(16,16)
            // segment: one coarse face or two summed half-length fine faces.
            const auto measure = [&]( const fluidField2& f ) -> double
            {
                double volume = 0.0;
                for( int i = 0; i < segments; ++i )
                {
                    const int x = 16 * segments + i;
                    const int y = 16 * segments;
                    const auto budget = f.getCutCellVolumeBudget( x, y - 1, dt );
                    if( !budget.valid || !std::isfinite( budget.macOutwardVolume[3] ) )
                        fail( "invalid shared physical-face flux budget" );
                    volume += budget.macOutwardVolume[3];
                }
                return volume;
            };
            const auto geometry = [&]( const fluidField2& f )
            {
                double openTime = 0.0, swept = 0.0;
                for( int i = 0; i < segments; ++i )
                {
                    const int x = 16 * segments + i;
                    const int y = 16 * segments;
                    openTime += f.getSweptVTimeMeanOpenFraction( x, y ) *
                        dt / segments;
                    swept += f.getCutCellVolumeBudget(
                        x, y - 1, dt ).solidSweptOutwardVolume[3];
                }
                return std::pair{ openTime, swept };
            };
            std::optional<fluidField2> history, manufactured;
            double naturalPre = 0.0, matchedPre = 0.0;
            int currentStep = -1, observed = 0;

            field.setPrePressureSnapshotObserver( [&]( const fluidField2& frozen )
            {
                const int oneBased = currentStep + 1;
                const int cp = oneBased * 16 / fluidSteps;
                if( oneBased * 16 != cp * fluidSteps ||
                    std::find( checkpoints.begin(), checkpoints.end(), cp ) ==
                        checkpoints.end() ) return;
                if( observed != 0 || !frozen.hasSweptApertureAudit() )
                    fail( "missing or duplicated checkpointed frozen snapshot" );
                observed = 1;
                history.emplace( frozen );
                manufactured.emplace( frozen );
                history->setPrePressureSnapshotObserver( {} );
                manufactured->setPrePressureSnapshotObserver( {} );
                naturalPre = measure( *history );
                if( !manufactured->setFrozenMacVelocityForAudit( worldVelocity ) )
                    fail( "manufactured continuous world velocity override rejected" );
                matchedPre = measure( *manufactured );
                if( !history->replayFrozenCutCellPressure( 120, true ).has_value() ||
                    !manufactured->replayFrozenCutCellPressure( 120, true ).has_value() )
                    fail( "pressure-only completion failed on copied states" );
                if( probeZeroMac )
                {
                    // The third copy retains the SAME moving-solid geometry,
                    // pressure source and closed-face wall velocities. Only
                    // its open interior MAC velocities become zero. This
                    // does not isolate an exact continuous physical flux.
                    fluidField2 zero = frozen;
                    zero.setPrePressureSnapshotObserver( {} );
                    if( !zero.setFrozenMacVelocityForAudit(
                        []( vec2 ) { return vec2{}; } ) )
                        fail( "zero-open-MAC frozen override rejected" );
                    const double zeroPre = measure( zero );
                    if( !zero.replayFrozenCutCellPressure( 120 ).has_value() )
                        fail( "zero-open-MAC pressure replay failed" );
                    const double zeroPost = measure( zero );
                    const auto [zeroOpen, zeroSwept] = geometry( zero );
                    const auto [naturalOpen, naturalSwept] = geometry( *history );
                    const float residual = zero.getCutCellPressure().pressureResidual;
                    if( sweepZeroMacJacobi )
                    {
                        // Every budget uses a fresh copy of the EXACT same
                        // zero-open-MAC frozen pre-pressure geometry/velocity.
                        // Zero interior MAC flux still leaves moving-solid
                        // pressure sources and a compatibility constraint.
                        const auto auditLedger = [&]( const fluidField2& f )
                        {
                            const int width = f.getWidth();
                            const int height = f.getHeight();
                            const int count = width * height;
                            std::vector<unsigned char> visited( count, 0 );
                            std::vector<int> queue;
                            queue.reserve( count );
                            double sourceTotal = 0.0;
                            double absoluteRegionSource = 0.0;
                            int regions = 0;
                            for( int sy = 0; sy < height; ++sy )
                                for( int sx = 0; sx < width; ++sx )
                                {
                                    const int start = sy * width + sx;
                                    if( visited[start] ||
                                        f.getCellFluidFraction( sx, sy ) <= 0.0f )
                                        continue;
                                    visited[start] = 1;
                                    queue.clear();
                                    queue.push_back( start );
                                    double regionSource = 0.0;
                                    for( std::size_t head = 0; head < queue.size(); ++head )
                                    {
                                        const int cell = queue[head];
                                        const int x = cell % width, y = cell / width;
                                        const auto budget = f.getCutCellVolumeBudget(
                                            x, y, dt );
                                        if( !budget.valid || !budget.hasVolumeHistory )
                                            fail( "zero-MAC pressure source history missing" );
                                        regionSource += budget.pressureSourceDeltaVolume;
                                        const auto visit = [&]( int nx, int ny, float open )
                                        {
                                            if( open <= 0.0f || nx < 0 || ny < 0 ||
                                                nx >= width || ny >= height ||
                                                f.getCellFluidFraction( nx, ny ) <= 0.0f )
                                                return;
                                            const int neighbor = ny * width + nx;
                                            if( visited[neighbor] ) return;
                                            visited[neighbor] = 1;
                                            queue.push_back( neighbor );
                                        };
                                        visit( x - 1, y, f.getUFaceOpenFraction( x, y ) );
                                        visit( x + 1, y, f.getUFaceOpenFraction( x + 1, y ) );
                                        visit( x, y - 1, f.getVFaceOpenFraction( x, y ) );
                                        visit( x, y + 1, f.getVFaceOpenFraction( x, y + 1 ) );
                                    }
                                    sourceTotal += regionSource;
                                    absoluteRegionSource += std::abs( regionSource );
                                    ++regions;
                                }
                            return std::tuple{ sourceTotal / dt,
                                absoluteRegionSource / dt, regions };
                        };
                        std::optional<jacobiResult> previous;
                        for( int iterations : { 40, 80, 120, 200 } )
                        {
                            fluidField2 trial = frozen;
                            trial.setPrePressureSnapshotObserver( {} );
                            if( !trial.setFrozenMacVelocityForAudit(
                                []( vec2 ) { return vec2{}; } ) )
                                fail( "frozen zero-MAC Jacobi input rejected" );
                            const double pre = measure( trial );
                            if( std::abs( pre ) > 1.0e-12 ||
                                !trial.replayFrozenCutCellPressure( iterations ).has_value() )
                                fail( "zero-MAC Jacobi replay not started from zero face flux" );
                            const double post = measure( trial );
                            const auto& pressure = trial.getCutCellPressure();
                            const auto [sourceRate, regionSourceRate, regions] =
                                auditLedger( trial );
                            const double calculatedImbalance =
                                sourceRate + pressure.unresolvedVolumeRate;
                            const double calculatedCompatibility =
                                regionSourceRate + std::abs(
                                    static_cast<double>( pressure.unresolvedVolumeRate ) );
                            const double sourceError =
                                calculatedImbalance - pressure.sourceImbalance;
                            const double compatibilityError =
                                calculatedCompatibility - pressure.compatibilityCorrection;
                            const auto [openTime, swept] = geometry( trial );
                            // These source totals nearly cancel across
                            // cells, so a 1 m^3/s scale would mask errors.
                            // Bound float accumulation separately from the
                            // much smaller measured net source.
                            const double sourceLedgerTolerance = 1.0e-6 +
                                1.0e-5 * std::max( std::abs( sourceRate ),
                                    regionSourceRate );
                            if( !std::isfinite( post ) ||
                                !std::isfinite( pressure.pressureResidual ) ||
                                !std::isfinite( pressure.divergenceAfter ) ||
                                !std::isfinite( sourceError ) ||
                                !std::isfinite( compatibilityError ) ||
                                regions != pressure.connectedRegions ||
                                std::abs( sourceError ) > sourceLedgerTolerance ||
                                std::abs( compatibilityError ) >
                                    sourceLedgerTolerance ||
                                std::abs( openTime - zeroOpen ) > 1.0e-7 ||
                                std::abs( swept - zeroSwept ) > 1.0e-7 )
                                fail( "zero-MAC source/compatibility ledger mismatch" );
                            double localSource = 0.0, localSnapshot = 0.0;
                            double localSwept = 0.0, localAfter = 0.0;
                            double localResidual = 0.0;
                            for( int segment = 0; segment < segments; ++segment )
                                for( int offset : { -1, 0 } )
                                {
                                    const int x = 16 * segments + segment;
                                    const int y = 16 * segments + offset;
                                    const auto b = trial.getCutCellVolumeBudget( x, y, dt );
                                    if( !b.valid || !b.hasVolumeHistory )
                                        fail( "local moving-volume budget unavailable" );
                                    localSource += b.pressureSourceDeltaVolume;
                                    localSnapshot += b.snapshotDeltaVolume;
                                    localSwept += b.solidSweptDivergenceVolume;
                                    localAfter += b.macNetOutwardVolume;
                                    localResidual += b.pressureResidualVolume;
                                }
                            const jacobiResult record{ h, fluidSteps, cp, iterations,
                                post, pressure.pressureResidual, sourceRate,
                                pressure.compatibilityCorrection, localSource,
                                localSnapshot, localSwept, localAfter, localResidual,
                                pressure.connectedRegions };
                            if( previous &&
                                ( previous->regions != record.regions ||
                                    std::abs( previous->sourceRate - record.sourceRate ) >
                                        1.0e-7 ||
                                    std::abs( previous->compatibility -
                                        record.compatibility ) > 1.0e-7 ||
                                    std::abs( previous->localSource -
                                        record.localSource ) > 1.0e-8 ||
                                    std::abs( previous->localSnapshot -
                                        record.localSnapshot ) > 1.0e-8 ||
                                    std::abs( previous->localSwept -
                                        record.localSwept ) > 1.0e-8 ) )
                                fail( "frozen pressure source changed with Jacobi iteration count" );
                            previous = record;
                            jacobiResults.push_back( record );
                            std::printf( "zero_mac_jacobi,%.9g,%d,%d,%d,"
                                "%.12g,%.9g,%.9g,%.12g,%.12g,%d,"
                                "%.12g,%.12g,%.12g,%.12g,%.12g,"
                                "%.12g,%.12g\n",
                                h, fluidSteps, cp, iterations, post,
                                pressure.pressureResidual, pressure.divergenceAfter,
                                pressure.sourceImbalance, pressure.compatibilityCorrection,
                                pressure.connectedRegions, localSource, localSnapshot,
                                localSwept, localAfter, localResidual,
                                sourceError, compatibilityError );
                            if( iterations == 120 &&
                                ( std::abs( post - zeroPost ) > 1.0e-10 ||
                                    std::abs( pressure.pressureResidual - residual ) >
                                        1.0e-7 ) )
                                fail( "120-iteration frozen zero-MAC trial differs from baseline" );
                        }
                    }
                    if( !std::isfinite( zeroPre ) || !std::isfinite( zeroPost ) ||
                        !std::isfinite( residual ) ||
                        std::abs( zeroOpen - naturalOpen ) > 1.0e-7 ||
                        std::abs( zeroSwept - naturalSwept ) > 1.0e-7 )
                        fail( "zero-MAC copy changed geometry or produced nonfinite pressure" );
                    zeroResults.push_back( { h, fluidSteps, cp,
                        zeroPre, zeroPost, zeroOpen, zeroSwept, residual } );
                    std::printf( "zero_mac_pressure,%.9g,%d,%d,"
                        "%.12g,%.12g,%.12g,%.9g,%.12g,%.12g\n",
                        h, fluidSteps, cp, zeroPre, zeroPost,
                        zeroPost - zeroPre, residual, zeroOpen, zeroSwept );
                }
            } );

            for( int step = 0; step < fluidSteps; ++step )
            {
                currentStep = step;
                observed = 0;
                const float alpha = static_cast<float>( step + 1 ) / fluidSteps;
                capsule.transform.rotation = rot2::FromRadians(
                    startAngle + alpha * ( endAngle - startAngle ) );
                capsule.angularVelocity = angularSpeed;
                field.step( dt, std::span<const fluidShapeObstacle2>( &capsule, 1 ),
                    feedback, 120 );
                const int oneBased = step + 1;
                const int cp = oneBased * 16 / fluidSteps;
                const bool due = oneBased * 16 == cp * fluidSteps &&
                    std::find( checkpoints.begin(), checkpoints.end(), cp ) !=
                        checkpoints.end();
                if( ( observed == 1 ) != due )
                    fail( "frozen checkpoint callback coverage mismatch" );
                if( !due ) continue;
                if( !history || !manufactured )
                    fail( "missing pair of completed restarts" );

                // The natural replay copy MUST reproduce the live original
                // at this exact state (not just pressure iteration count).
                if( std::abs( measure( *history ) - measure( field ) ) > 1.0e-8 ||
                    Length( history->getFluidMomentum() -
                        field.getFluidMomentum() ) > 1.0e-5f )
                    fail( "completed natural replay does not equal live post-Step field" );

                for( int horizon = 0; horizon <= 4; ++horizon )
                {
                    if( horizon > 0 )
                    {
                        const float forwardAlpha = static_cast<float>(
                            step + horizon + 1 ) / fluidSteps;
                        capsule.transform.rotation = rot2::FromRadians(
                            startAngle + forwardAlpha * ( endAngle - startAngle ) );
                        std::array<fluidObstacleForce2, 1> naturalFeedback{}, matchedFeedback{};
                        history->step( dt,
                            std::span<const fluidShapeObstacle2>( &capsule, 1 ),
                            naturalFeedback, 120 );
                        manufactured->step( dt,
                            std::span<const fluidShapeObstacle2>( &capsule, 1 ),
                            matchedFeedback, 120 );
                    }
                    if( std::find( horizons.begin(), horizons.end(), horizon ) ==
                        horizons.end() ) continue;
                    const double naturalPost = measure( *history );
                    const double matchedPost = measure( *manufactured );
                    const auto [naturalOpen, naturalSwept] = geometry( *history );
                    const auto [matchedOpen, matchedSwept] = geometry( *manufactured );
                    if( !std::isfinite( naturalPost ) ||
                        !std::isfinite( matchedPost ) ||
                        std::abs( naturalOpen - matchedOpen ) > 1.0e-7 ||
                        std::abs( naturalSwept - matchedSwept ) > 1.0e-7 )
                        fail( "restart changed the prescribed solid geometry/flux" );
                    results.push_back( { h, fluidSteps, cp, horizon,
                        naturalPre, matchedPre, naturalPost, matchedPost,
                        naturalOpen, naturalSwept } );
                    std::printf( "matched_world_restart,%.9g,%d,%d,%d,%.9g,"
                        "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n",
                        h, fluidSteps, cp, horizon,
                        startAngle + static_cast<float>( step + horizon + 1 ) /
                            fluidSteps * ( endAngle - startAngle ),
                        naturalPre, matchedPre, naturalPost, matchedPost,
                        naturalOpen, naturalSwept );
                }
                history.reset();
                manufactured.reset();
            }
        }

    if( results.size() != 32 )
        fail( "not all 2x2x2x4 physical-face restart checkpoints were recorded" );
    if( probeZeroMac && zeroResults.size() != 8 )
        fail( "not all 2x2x2 zero-open-MAC pressure probes were recorded" );
    bool foundInputHistoryDifference = false, foundPropagation = false;
    for( int fluidSteps : { 64, 128 } )
        for( int checkpoint : checkpoints )
            for( int horizon : horizons )
            {
                const result *coarse = nullptr, *fine = nullptr;
                for( const result& row : results )
                    if( row.fluidSteps == fluidSteps &&
                        row.checkpoint == checkpoint && row.horizon == horizon )
                    {
                        if( row.h == 0.25f ) coarse = &row;
                        else if( row.h == 0.125f ) fine = &row;
                    }
                if( !coarse || !fine )
                    fail( "missing matched world-time and physical-face pair" );
                const double historyGap = coarse->naturalPost - fine->naturalPost;
                const double matchedGap = coarse->matchedPost - fine->matchedPost;
                const double excess = historyGap - matchedGap;
                const double historyPre = coarse->naturalPre - fine->naturalPre;
                const double matchedPre = coarse->matchedPre - fine->matchedPre;
                if( !std::isfinite( historyGap ) ||
                    !std::isfinite( matchedGap ) || !std::isfinite( excess ) ||
                    !std::isfinite( historyPre ) ||
                    !std::isfinite( matchedPre ) )
                    fail( "nonfinite matched-state cross-grid comparison" );
                foundInputHistoryDifference |= horizon == 0 &&
                    std::abs( historyPre - matchedPre ) > 1.0e-8;
                foundPropagation |= horizon == 4 &&
                    std::abs( excess ) > 1.0e-8;
                std::printf( "matched_world_cross,%d,%d,%d,"
                    "%.12g,%.12g,%.12g,%.12g,%.12g\n",
                    fluidSteps, checkpoint, horizon,
                    historyGap, matchedGap, excess, historyPre, matchedPre );
                if( probeZeroMac && horizon == 0 )
                {
                    const zeroResult *coarseZero = nullptr, *fineZero = nullptr;
                    for( const zeroResult& row : zeroResults )
                        if( row.fluidSteps == fluidSteps && row.checkpoint == checkpoint )
                        {
                            if( row.h == 0.25f ) coarseZero = &row;
                            else if( row.h == 0.125f ) fineZero = &row;
                        }
                    if( !coarseZero || !fineZero ||
                        std::abs( coarseZero->openTime - coarse->openTime ) > 1.0e-7 ||
                        std::abs( fineZero->openTime - fine->openTime ) > 1.0e-7 ||
                        std::abs( coarseZero->sweptVolume - coarse->sweptVolume ) > 1.0e-7 ||
                        std::abs( fineZero->sweptVolume - fine->sweptVolume ) > 1.0e-7 )
                        fail( "zero-MAC and affine copies do not share moving geometry" );
                    const double zeroPreGap = coarseZero->pre - fineZero->pre;
                    const double zeroPostGap = coarseZero->post - fineZero->post;
                    const double affineProjectionGap = matchedGap - matchedPre;
                    const double zeroProjectionGap = zeroPostGap - zeroPreGap;
                    const double velocityDependentGap =
                        affineProjectionGap - zeroProjectionGap;
                    if( !std::isfinite( velocityDependentGap ) ||
                        std::abs( ( matchedGap - zeroPostGap ) -
                            ( matchedPre - zeroPreGap ) - velocityDependentGap ) > 1.0e-9 )
                        fail( "matched affine/zero pressure decomposition does not close" );
                    std::printf( "zero_mac_cross,%d,%d,"
                        "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n",
                        fluidSteps, checkpoint, matchedPre, zeroPreGap,
                        affineProjectionGap, zeroProjectionGap,
                        velocityDependentGap, zeroPostGap );
                }
            }
    if( sweepZeroMacJacobi )
    {
        if( jacobiResults.size() != 32 )
            fail( "zero-MAC Jacobi convergence checkpoints missing" );
        for( int steps : { 64, 128 } )
            for( int checkpoint : checkpoints )
            {
                double referenceGap = 0.0;
                for( int iteration : { 40, 80, 120, 200 } )
                {
                    const jacobiResult *coarse = nullptr, *fine = nullptr;
                    for( const jacobiResult& sample : jacobiResults )
                        if( sample.fluidSteps == steps &&
                            sample.checkpoint == checkpoint &&
                            sample.iterations == iteration )
                        {
                            if( sample.h == 0.25f ) coarse = &sample;
                            else if( sample.h == 0.125f ) fine = &sample;
                        }
                    if( !coarse || !fine )
                        fail( "zero-MAC Jacobi cross-grid pair missing" );
                    const double gap = coarse->post - fine->post;
                    if( iteration == 200 ) referenceGap = gap;
                }
                for( int iteration : { 40, 80, 120, 200 } )
                {
                    const jacobiResult *coarse = nullptr, *fine = nullptr;
                    for( const jacobiResult& sample : jacobiResults )
                        if( sample.fluidSteps == steps &&
                            sample.checkpoint == checkpoint &&
                            sample.iterations == iteration )
                        {
                            if( sample.h == 0.25f ) coarse = &sample;
                            else if( sample.h == 0.125f ) fine = &sample;
                        }
                    if( !coarse || !fine )
                        fail( "zero-MAC Jacobi cross-grid pair missing" );
                    const double gap = coarse->post - fine->post;
                    std::printf( "zero_mac_jacobi_cross,%d,%d,%d,"
                        "%.12g,%.12g,%.12g\n",
                        steps, checkpoint, iteration, gap,
                        coarse->residual - fine->residual, gap - referenceGap );
                }
            }
    }
    if( !foundInputHistoryDifference || !foundPropagation )
        fail( "matched-state restart did not exercise history replacement and propagation" );
}

// S2c reference diagnostic: the SAME frozen, zero-open-MAC pressure RHS
// is solved with increasingly long Jacobi budgets. Independently compute
// the discrete compatible continuity residual from the resulting MAC flux,
// current volume source and per-connected-component source mean. This is
// not the exact continuum moving-wall flux, nor a Schur PCG reference.
// Independent S2c reference of the noncoupled Cut-cell pressure equation.
// Assemble its physical *volume* operator from public face apertures and the
// frozen moving-volume pressure-source ledger, then solve the symmetric
// Neumann graph using double-precision diagonal-preconditioned CG. This
// deliberately does not call the engine's Jacobi/Schur pressure solvers.
struct independentCutCellPressureResult
{
    int iterations = 0, activeCells = 0, components = 0;
    int activePressureGroups = 0, anchoredPressureGroups = 0;
    int activeToInactiveOpenFaces = 0;
    double activeToInactiveApertureSum = 0.0;
    double initialRmsPerSecond = 0.0, finalRmsPerSecond = 0.0;
    double targetFaceVolume = 0.0;
    double sourceVolume = 0.0;
    double compatibilityRebalanceL1Volume = 0.0;
    bool converged = false;
};

independentCutCellPressureResult solveIndependentFrozenCutCellPressure(
    const fluidField2& source, float subDt, int targetSegments,
    bool omitInactiveAnchors = false )
{
    const int width = source.getWidth(), height = source.getHeight();
    const int count = width * height;
    const double scale = static_cast<double>( source.getCellSize() ) *
        source.getCellSize() * subDt;
    if( count == 0 || !( scale > 0.0 ) )
        throw std::runtime_error( "invalid independent reference grid" );
    std::vector<unsigned char> active( count, 0 );
    std::vector<std::array<double, 4>> weights( count );
    std::vector<double> diagonal( count, 0.0 ), rhs( count, 0.0 );
    std::vector<double> potential( count, 0.0 ), residual( count, 0.0 );
    std::vector<double> preconditioned( count, 0.0 ), direction( count, 0.0 );
    std::vector<double> product( count, 0.0 );
    independentCutCellPressureResult output{};

    for( int y = 0; y < height; ++y )
        for( int x = 0; x < width; ++x )
        {
            const int index = y * width + x;
            const auto ledger = source.getCutCellVolumeBudget( x, y, subDt );
            if( !ledger.valid || !ledger.hasVolumeHistory )
                throw std::runtime_error( "missing independent reference source" );
            if( ledger.active != ( source.getCellFluidFraction( x, y ) > 0.0f ) )
                throw std::runtime_error( "reference active-cell mask mismatch" );
            active[index] = ledger.active ? 1 : 0;
            if( ledger.active )
            {
                ++output.activeCells;
                // Correction adds A(q) to net outward MAC volume.
                rhs[index] = -ledger.pressureSourceDeltaVolume;
                output.sourceVolume += ledger.pressureSourceDeltaVolume;
            }
            // A positive face on a fully covered cell can still appear
            // in the real analytic aperture graph. Query ALL cells so
            // our compatibility-component walk matches the engine.
            const std::array<double, 4> aperture{{
                source.getUFaceOpenFraction( x, y ),
                source.getUFaceOpenFraction( x + 1, y ),
                source.getVFaceOpenFraction( x, y ),
                source.getVFaceOpenFraction( x, y + 1 )
            }};
            weights[index] = aperture;
            for( const double w : aperture )
            {
                if( !std::isfinite( w ) || w < 0.0 || w > 1.0 )
                    throw std::runtime_error( "invalid independent reference aperture" );
                if( ledger.active ) diagonal[index] += w;
            }
        }

    const auto neighbors = [width]( int index )
    {
        return std::array<int, 4>{{
            index - 1, index + 1, index - width, index + width
        }};
    };
    // The production compatibility graph visits any positive-open face,
    // including a node with zero fluid fraction. Such nodes contribute
    // zero divergence but count toward a component's mean-source divisor.
    // Reconstruct this graph separately from the active pressure unknowns.
    std::vector<int> fullComponent( count, -1 );
    for( int seed = 0; seed < count; ++seed )
    {
        if( !active[seed] || fullComponent[seed] >= 0 ) continue;
        const int regionId = output.components++;
        std::vector<int> region{ seed };
        fullComponent[seed] = regionId;
        for( std::size_t head = 0; head < region.size(); ++head )
        {
            const int index = region[head];
            const auto next = neighbors( index );
            for( int k = 0; k < 4; ++k )
            {
                if( weights[index][k] <= 0.0 ) continue;
                if( next[k] < 0 || next[k] >= count )
                    throw std::runtime_error( "open face exits independent grid" );
                if( fullComponent[next[k]] >= 0 ) continue;
                fullComponent[next[k]] = regionId;
                region.push_back( next[k] );
            }
        }
        double negativeSource = 0.0;
        for( int index : region )
            if( active[index] ) negativeSource += rhs[index];
        const double meanNegativeSource = negativeSource / region.size();
        for( int index : region )
            if( active[index] ) rhs[index] -= meanNegativeSource;
    }

    // Optional isolated counterfactual: cut only active-to-inactive
    // pressure edges AFTER reconstructing the original production source
    // compatibility graph. The normal independently assembled reference
    // always retains these anchored diagonal weights.
    if( omitInactiveAnchors )
        for( int index = 0; index < count; ++index )
        {
            if( !active[index] ) continue;
            const auto next = neighbors( index );
            for( int k = 0; k < 4; ++k )
            {
                const double opening = weights[index][k];
                if( opening <= 0.0 || active[next[k]] ) continue;
                weights[index][k] = 0.0;
                diagonal[index] -= opening;
                if( diagonal[index] < -1.0e-9 )
                    throw std::runtime_error( "negative counterfactual diagonal" );
            }
        }
    // Pressure rows for inactive cells are *not* Jacobi unknowns; their
    // pressure stays 0. A positive aperture from active to inactive
    // normally contributes a diagonal anchored (Dirichlet-like) edge.
    // Other active components are genuine mean-zero Neumann systems.
    std::vector<int> activeComponent( count, -1 );
    std::vector<std::vector<int>> componentCells;
    std::vector<unsigned char> anchored;
    for( int seed = 0; seed < count; ++seed )
    {
        if( !active[seed] || activeComponent[seed] >= 0 ) continue;
        const int regionId = static_cast<int>( componentCells.size() );
        componentCells.emplace_back();
        anchored.push_back( 0 );
        std::vector<int>& region = componentCells.back();
        region.push_back( seed );
        activeComponent[seed] = regionId;
        for( std::size_t head = 0; head < region.size(); ++head )
        {
            const int index = region[head];
            const auto next = neighbors( index );
            for( int k = 0; k < 4; ++k )
            {
                const double opening = weights[index][k];
                if( opening <= 0.0 ) continue;
                if( !active[next[k]] )
                {
                    anchored[regionId] = 1;
                    ++output.activeToInactiveOpenFaces;
                    output.activeToInactiveApertureSum += opening;
                    continue;
                }
                if( activeComponent[next[k]] >= 0 ) continue;
                activeComponent[next[k]] = regionId;
                region.push_back( next[k] );
            }
        }
    }
    output.activePressureGroups = static_cast<int>( componentCells.size() );
    for( unsigned char value : anchored )
        output.anchoredPressureGroups += value != 0;
    if( omitInactiveAnchors )
        for( std::size_t group = 0; group < componentCells.size(); ++group )
        {
            if( anchored[group] ) continue;
            const auto& region = componentCells[group];
            double sum = 0.0;
            for( int index : region ) sum += rhs[index];
            output.compatibilityRebalanceL1Volume += std::abs( sum );
            const double mean = sum / region.size();
            for( int index : region ) rhs[index] -= mean;
        }

    // Each stored weight is the positive graph edge coefficient in the
    // volume-form SPD operator A=q_i*sum(open)-sum(open*q_neighbor).
    // One unit of q is dt² times one unit of physical pressure.
    const auto apply = [&]( const std::vector<double>& input,
        std::vector<double>& values )
    {
        std::fill( values.begin(), values.end(), 0.0 );
        for( int index = 0; index < count; ++index )
        {
            if( !active[index] ) continue;
            const auto next = neighbors( index );
            values[index] = diagonal[index] * input[index];
            for( int k = 0; k < 4; ++k )
                if( weights[index][k] > 0.0 )
                    values[index] -= weights[index][k] * input[next[k]];
        }
    };
    const auto inner = [&]( const std::vector<double>& a,
        const std::vector<double>& b )
    {
        double sum = 0.0;
        for( int i = 0; i < count; ++i )
            if( active[i] ) sum += a[i] * b[i];
        return sum;
    };
    // Only true unanchored Neumann pressure groups have gauge freedom.
    const auto removeMeans = [&]( std::vector<double>& values )
    {
        for( std::size_t group = 0; group < componentCells.size(); ++group )
        {
            if( anchored[group] ) continue;
            const auto& region = componentCells[group];
            double sum = 0.0;
            for( int index : region ) sum += values[index];
            const double mean = sum / region.size();
            for( int index : region ) values[index] -= mean;
        }
    };
    const auto rms = [&]( const std::vector<double>& values )
    {
        return std::sqrt( inner( values, values ) / output.activeCells ) / scale;
    };
    residual = rhs;
    removeMeans( residual );
    output.initialRmsPerSecond = rms( residual );

    double currentRms = output.initialRmsPerSecond;
    const double stopRms = 1.0e-5; // diagnostic tolerance, 1/s
    constexpr int maxIterations = 10000;
    if( currentRms > stopRms )
    {
        for( int i = 0; i < count; ++i )
            if( active[i] && diagonal[i] > 0.0 )
                preconditioned[i] = residual[i] / diagonal[i];
        removeMeans( preconditioned );
        direction = preconditioned;
        double rz = inner( residual, preconditioned );
        for( int iteration = 1; iteration <= maxIterations; ++iteration )
        {
            apply( direction, product );
            const double denominator = inner( direction, product );
            if( !( denominator > 0.0 ) || !std::isfinite( denominator ) ||
                !( rz > 0.0 ) || !std::isfinite( rz ) )
                break;
            const double alpha = rz / denominator;
            for( int i = 0; i < count; ++i )
            {
                if( !active[i] ) continue;
                potential[i] += alpha * direction[i];
                residual[i] -= alpha * product[i];
            }
            removeMeans( residual );
            output.iterations = iteration;
            currentRms = rms( residual );
            if( currentRms <= stopRms ) break;
            for( int i = 0; i < count; ++i )
                preconditioned[i] = active[i] && diagonal[i] > 0.0
                    ? residual[i] / diagonal[i] : 0.0;
            removeMeans( preconditioned );
            const double nextRz = inner( residual, preconditioned );
            if( !( nextRz > 0.0 ) || !std::isfinite( nextRz ) )
                break;
            const double beta = nextRz / rz;
            for( int i = 0; i < count; ++i )
                direction[i] = preconditioned[i] + beta * direction[i];
            removeMeans( direction );
            rz = nextRz;
        }
    }

    // Re-evaluate A*q-b independently of the CG recurrence, which can
    // underestimate the true residual when roundoff accumulates.
    apply( potential, product );
    for( int index = 0; index < count; ++index )
        residual[index] = product[index] - rhs[index];
    output.finalRmsPerSecond = rms( residual );
    output.converged = std::isfinite( output.finalRmsPerSecond ) &&
        output.finalRmsPerSecond < stopRms;

    const int y = 16 * targetSegments;
    for( int segment = 0; segment < targetSegments; ++segment )
    {
        const int x = 16 * targetSegments + segment;
        const double aperture = weights[( y - 1 ) * width + x][3];
        output.targetFaceVolume += aperture *
            ( potential[( y - 1 ) * width + x] - potential[y * width + x] );
    }
    if( !std::isfinite( output.targetFaceVolume ) ||
        !std::isfinite( output.sourceVolume ) ||
        output.activeCells == 0 )
        throw std::runtime_error( "nonfinite independent pressure reference" );
    return output;
}

// Report every physically shared MAC face whose discrete endpoint opening
// is positive even though exactly one neighboring cell is fully covered.
// A moving boundary may cross this face during the step, so report the
// swept time-mean opening separately rather than treating it as the
// endpoint pressure aperture or declaring a physical leak.
void auditFrozenInactiveNeighborFaces( const fluidField2& source,
    float subDt, int checkpoint, int expectedCount, double expectedOpening,
    vec2 capsuleCenter, float capsuleAngle )
{
    const float h = source.getCellSize();
    const vec2 origin = source.getOrigin();
    // Independent midpoint geometry oracle for this prescribed single capsule.
    // The fluid solver's analytic rectangle areas and clipped face intervals
    // are intentionally NOT used. These are sampled estimates, not exact
    // geometric predicates or a replacement for the production Cut-cell grid.
    constexpr int samples = 96;
    const int discreteSamples = source.getCutCellSampleCount();
    if( discreteSamples != 4 || source.isAnalyticCapsuleCutCellEnabled() ||
        source.getCutCellTransition().usedDisjointAnalyticGeometry )
        throw std::runtime_error( "prescribed anchor reference no longer uses sampled Cut-cells" );
    constexpr double halfLength = 0.63;
    constexpr double radiusSquared = 0.19 * 0.19;
    const double axisX = std::cos( static_cast<double>( capsuleAngle ) );
    const double axisY = std::sin( static_cast<double>( capsuleAngle ) );
    const auto freeAt = [&]( double px, double py )
    {
        double dx = px - capsuleCenter.x, dy = py - capsuleCenter.y;
        const double along = std::clamp( dx * axisX + dy * axisY,
            -halfLength, halfLength );
        dx -= along * axisX;
        dy -= along * axisY;
        return dx * dx + dy * dy > radiusSquared;
    };
    const auto sampleCell = [&]( int x, int y, int count )
    {
        int open = 0;
        for( int sy = 0; sy < count; ++sy )
            for( int sx = 0; sx < count; ++sx )
            {
                const double px = origin.x + ( x + ( sx + 0.5 ) / count ) * h;
                const double py = origin.y + ( y + ( sy + 0.5 ) / count ) * h;
                open += freeAt( px, py );
            }
        return static_cast<double>( open ) / ( count * count );
    };
    const auto sampleFace = [&]( char axis, int x, int y, int count )
    {
        int open = 0;
        for( int i = 0; i < count; ++i )
        {
            const double px = origin.x + ( x +
                ( axis == 'V' ? ( i + 0.5 ) / count : 0.0 ) ) * h;
            const double py = origin.y + ( y +
                ( axis == 'U' ? ( i + 0.5 ) / count : 0.0 ) ) * h;
            open += freeAt( px, py );
        }
        return static_cast<double>( open ) / count;
    };
    int seen = 0;
    double sumOpening = 0.0;
    std::puts( "zero_mac_anchor_face,cell_m,checkpoint_16,axis,face_x,face_y,"
        "world_x,world_y,endpoint_open,swept_mean_open,"
        "side_a_fluid_fraction,side_b_fluid_fraction,"
        "side_a_pressure_source_m3,side_b_pressure_source_m3,"
        "solid_face_speed_mps,discrete_reference_face_open,"
        "discrete_reference_side_a_fluid,discrete_reference_side_b_fluid,"
        "reference_samples,reference_face_open,"
        "reference_side_a_fluid,reference_side_b_fluid" );
    const auto emit = [&]( char axis, int x, int y, vec2 point,
        int ax, int ay, int bx, int by,
        float opening, float timeMean, float solidSpeed )
    {
        if( opening <= 0.0f ) return;
        const auto a = source.getCutCellVolumeBudget( ax, ay, subDt );
        const auto b = source.getCutCellVolumeBudget( bx, by, subDt );
        if( !a.valid || !b.valid )
            throw std::runtime_error( "invalid frozen anchor face ledger" );
        const bool aActive = a.currentFraction > 0.0f;
        const bool bActive = b.currentFraction > 0.0f;
        if( aActive == bActive ) return;
        if( !std::isfinite( opening ) || !std::isfinite( timeMean ) ||
            !std::isfinite( solidSpeed ) ||
            opening < 0.0f || opening > 1.0f ||
            timeMean < 0.0f || timeMean > 1.0f )
            throw std::runtime_error( "nonfinite or invalid frozen anchor" );
        // Reproduce the exact 4-point/4x4 midpoint discretization first.
        // A mismatch here would refute the sampling-resolution diagnosis.
        const double discreteFace = sampleFace( axis, x, y, discreteSamples );
        const double discreteA = sampleCell( ax, ay, discreteSamples );
        const double discreteB = sampleCell( bx, by, discreteSamples );
        if( std::abs( discreteFace - opening ) > 1.0e-6 ||
            std::abs( discreteA - a.currentFraction ) > 1.0e-6 ||
            std::abs( discreteB - b.currentFraction ) > 1.0e-6 )
            throw std::runtime_error(
                "independent coarse capsule sampling does not reproduce Cut-cell fractions" );
        const double referenceFace = sampleFace( axis, x, y, samples );
        const double referenceA = sampleCell( ax, ay, samples );
        const double referenceB = sampleCell( bx, by, samples );
        if( !std::isfinite( referenceFace ) ||
            !std::isfinite( referenceA ) || !std::isfinite( referenceB ) )
            throw std::runtime_error( "invalid independent capsule geometry reference" );
        ++seen;
        sumOpening += opening;
        std::printf( "zero_mac_anchor_face,%.9g,%d,%c,%d,%d,"
            "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
            "%.12g,%.12g,%.9g,%.9g,%.9g,%.9g,"
            "%d,%.9g,%.9g,%.9g\n",
            h, checkpoint, axis, x, y, point.x, point.y,
            opening, timeMean, a.currentFraction, b.currentFraction,
            a.pressureSourceDeltaVolume, b.pressureSourceDeltaVolume,
            solidSpeed, discreteFace, discreteA, discreteB,
            samples, referenceFace, referenceA, referenceB );
    };
    for( int y = 0; y < source.getHeight(); ++y )
        for( int x = 1; x < source.getWidth(); ++x )
        {
            const float opening = source.getUFaceOpenFraction( x, y );
            const vec2 point = origin + h * vec2{
                static_cast<float>( x ), static_cast<float>( y ) + 0.5f };
            emit( 'U', x, y, point, x - 1, y, x, y, opening,
                source.getSweptUTimeMeanOpenFraction( x, y ),
                source.getUFaceSolidVelocity( x, y ) );
        }
    for( int y = 1; y < source.getHeight(); ++y )
        for( int x = 0; x < source.getWidth(); ++x )
        {
            const float opening = source.getVFaceOpenFraction( x, y );
            const vec2 point = origin + h * vec2{
                static_cast<float>( x ) + 0.5f, static_cast<float>( y ) };
            emit( 'V', x, y, point, x, y - 1, x, y, opening,
                source.getSweptVTimeMeanOpenFraction( x, y ),
                source.getVFaceSolidVelocity( x, y ) );
        }
    if( seen != expectedCount ||
        std::abs( sumOpening - expectedOpening ) > 1.0e-7 )
        throw std::runtime_error(
            "frozen anchor face scan disagrees with independent pressure graph" );
}

void auditFrozenZeroMacReference( bool locateResidualHotspots = false,
    bool compareDampedJacobi = false, bool compareIndependentPcg = false,
    bool auditAnchors = false )
{
    constexpr float totalDt = 1.0f / 30.0f;
    constexpr int fluidSteps = 128;
    constexpr float startAngle = 0.18f, endAngle = 1.28f;
    constexpr vec2 center{ -0.20f, -0.12f };
    constexpr std::array<int, 2> checkpoints{{ 11, 12 }};
    constexpr std::array<int, 4> budgets{{ 200, 800, 1600, 3200 }};
    const auto fail = []( const char* message )
    {
        std::fprintf( stderr, "frozen zero-MAC reference: %s\n", message );
        std::exit( 1 );
    };
    struct result
    {
        float h = 0.0f;
        int checkpoint = 0, iterations = 0;
        double faceVolume = 0.0;
        double compatibleRms = 0.0;
        double compatibilityFloorRms = 0.0;
        double sourceVolume = 0.0;
        float legacyResidual = 0.0f;
        int regions = 0;
        double checkerboardShare = 0.0;
    };
    struct independentRecord
    {
        float h = 0.0f;
        int checkpoint = 0;
        independentCutCellPressureResult reference{};
    };
    std::vector<result> results, dampedResults;
    std::vector<independentRecord> independentResults, counterfactualResults;
    results.reserve( 2 * checkpoints.size() * budgets.size() );
    if( compareDampedJacobi ) dampedResults.reserve( results.capacity() );
    if( compareIndependentPcg ) independentResults.reserve( 4 );
    if( auditAnchors ) counterfactualResults.reserve( 4 );
    std::puts( "zero_mac_reference,cell_m,fluid_substeps,checkpoint_16,"
        "jacobi_iterations,target_face_m3,compatible_rms_per_s,"
        "component_source_floor_rms_per_s,legacy_pressure_residual,"
        "active_source_volume_m3,fluid_regions" );
    std::puts( "zero_mac_reference_cross,fluid_substeps,checkpoint_16,"
        "jacobi_iterations,coarse_minus_fine_target_m3,"
        "coarse_minus_fine_compatible_rms_per_s,"
        "difference_from_3200_target_m3,reference_criteria_met" );
    if( compareIndependentPcg )
    {
        if( !compareDampedJacobi || locateResidualHotspots )
            fail( "independent PCG requires a matched clean frozen damped pair" );
        std::puts( "zero_mac_independent_pcg,cell_m,checkpoint_16,"
            "active_cells,compatibility_regions,pressure_groups,"
            "anchored_groups,open_active_to_inactive_faces,"
            "open_active_to_inactive_sum,iterations,initial_rms_per_s,"
            "final_explicit_rms_per_s,converged,source_m3,"
            "independent_target_face_m3,damped_3200_target_face_m3,"
            "independent_minus_damped_face_m3" );
        std::puts( "zero_mac_independent_pcg_cross,checkpoint_16,"
            "independent_coarse_minus_fine_face_m3,"
            "damped_3200_coarse_minus_fine_face_m3,"
            "independent_minus_damped_gap_m3,both_converged" );
        if( auditAnchors )
        {
            std::puts( "zero_mac_anchor_counterfactual,cell_m,checkpoint_16,"
                "actual_anchor_faces,actual_anchor_aperture_sum,"
                "counterfactual_compatibility_rebalance_L1_m3,"
                "actual_reference_face_m3,omitted_anchor_face_m3,"
                "omitted_minus_actual_m3,actual_pcg_rms_per_s,"
                "omitted_pcg_rms_per_s,omitted_converged" );
            std::puts( "zero_mac_anchor_cross,checkpoint_16,"
                "actual_coarse_minus_fine_m3,omitted_coarse_minus_fine_m3,"
                "counterfactual_minus_actual_gap_m3,both_converged" );
        }
    }
    if( compareDampedJacobi )
    {
        std::puts( "zero_mac_damped,cell_m,checkpoint_16,jacobi_iterations,"
            "legacy_compatible_rms_per_s,damped_compatible_rms_per_s,"
            "legacy_checkerboard_squared_share,damped_checkerboard_squared_share,"
            "legacy_target_face_m3,damped_target_face_m3,damped_minus_legacy_m3,"
            "component_source_floor_per_s" );
        std::puts( "zero_mac_damped_cross,checkpoint_16,jacobi_iterations,"
            "legacy_coarse_minus_fine_m3,damped_coarse_minus_fine_m3,"
            "damped_coarse_compatible_rms_per_s,damped_fine_compatible_rms_per_s,"
            "damped_reference_criteria_met" );
    }
    if( locateResidualHotspots )
    {
        std::puts( "zero_mac_hotspot_summary,cell_m,checkpoint_16,"
            "iterations,active_cells,global_compatible_rms_per_s,"
            "top8_squared_residual_share,tiny_fraction_squared_share,"
            "tiny_positive_aperture_squared_share,"
            "near_target_squared_share,max_abs_residual_per_s,"
            "checkerboard_signed_mean_per_s,checkerboard_squared_share" );
        std::puts( "zero_mac_jacobi_parity,cell_m,checkpoint_16,"
            "even_iteration,odd_compatible_rms_per_s,"
            "even_compatible_rms_per_s,odd_face_m3,even_face_m3,"
            "even_minus_odd_face_m3" );
        std::puts( "zero_mac_hotspot,cell_m,checkpoint_16,iterations,rank,"
            "x,y,world_center_x_m,world_center_y_m,compatible_residual_per_s,"
            "fraction,min_positive_open,max_open,positive_faces,"
            "source_m3,snapshot_m3,swept_divergence_m3,"
            "post_mac_net_m3,manhattan_from_target_cells" );
    }

    for( float h : { 0.25f, 0.125f } )
    {
        const int segments = static_cast<int>( std::lround( 0.25f / h ) );
        const float subDt = totalDt / fluidSteps;
        fluidField2 field{ 32 * segments, 32 * segments, h, { -4.0f, -4.0f } };
        field.setCutCellProjectionEnabled( true );
        field.setDisjointAnalyticCutCellEnabled( true );
        field.setSweptApertureAuditEnabled( true );
        if( field.isConnectedResidualTracerRemapEnabled() ||
            field.getSmallCellPressureMergeFraction() != 0.0f )
            fail( "experimental fluid physics enabled in reference" );
        fluidShapeObstacle2 capsule{
            capsule2{ { -0.63f, 0.0f }, { 0.63f, 0.0f }, 0.19f },
            { center, rot2::FromRadians( startAngle ) }, center,
            {}, 0.0f, 0.0f, 0.0f };
        capsule.angularVelocity = ( endAngle - startAngle ) / totalDt;
        std::array<fluidObstacleForce2, 1> feedback{};
        field.step( subDt, std::span<const fluidShapeObstacle2>( &capsule, 1 ),
            feedback, 120 );
        field.addVelocity( { -0.24f, -0.30f }, { 1.3f, 0.65f }, 0.65f );
        field.addVelocity( { 0.35f, 0.25f }, { -0.55f, 0.8f }, 0.60f );

        const auto targetFace = [&]( const fluidField2& source )
        {
            double volume = 0.0;
            for( int segment = 0; segment < segments; ++segment )
            {
                const int x = 16 * segments + segment;
                const int y = 16 * segments;
                const auto budget = source.getCutCellVolumeBudget( x, y - 1, subDt );
                if( !budget.valid ) fail( "target face budget unavailable" );
                volume += budget.macOutwardVolume[3];
            }
            return volume;
        };

        // An independent public-query evaluation of the compatible
        // post-projection residual. For a closed Neumann component, pressure
        // cannot eliminate the mean initial source. Remove that mean in
        // VOLUME units per cell before computing RMS in 1/second.
        struct hotspotCell
        {
            int x = 0, y = 0;
            double residual = 0.0;
            double source = 0.0, snapshot = 0.0, swept = 0.0, mac = 0.0;
            float fluidFraction = 0.0f;
            float minPositiveOpen = 0.0f, maxOpen = 0.0f;
            int positiveFaces = 0;
            int targetDistance = 0;
        };
        const auto compatibleResidual = [&]( const fluidField2& source,
            int checkpoint, int iterations )
        {
            const int width = source.getWidth();
            const int height = source.getHeight();
            const int cells = width * height;
            std::vector<unsigned char> visited( cells, 0 );
            std::vector<int> queue;
            queue.reserve( cells );
            double squared = 0.0, squaredFloor = 0.0, summedSource = 0.0;
            double parityVolume = 0.0;
            int fluidCells = 0, components = 0;
            std::vector<hotspotCell> cellsWithResidual;
            if( locateResidualHotspots ) cellsWithResidual.reserve( cells );
            for( int start = 0; start < cells; ++start )
            {
                const int sx = start % width, sy = start / width;
                if( visited[start] || source.getCellFluidFraction( sx, sy ) <= 0.0f )
                    continue;
                visited[start] = 1;
                queue.clear();
                queue.push_back( start );
                double regionSource = 0.0;
                for( std::size_t head = 0; head < queue.size(); ++head )
                {
                    const int index = queue[head];
                    const int x = index % width, y = index / width;
                    const auto budget = source.getCutCellVolumeBudget( x, y, subDt );
                    if( !budget.valid || !budget.hasVolumeHistory )
                        fail( "missing pressure source for connected residual" );
                    regionSource += budget.pressureSourceDeltaVolume;
                    const auto visit = [&]( int nx, int ny, float opening )
                    {
                        if( opening <= 0.0f || nx < 0 || nx >= width ||
                            ny < 0 || ny >= height ||
                            source.getCellFluidFraction( nx, ny ) <= 0.0f )
                            return;
                        const int neighbor = ny * width + nx;
                        if( visited[neighbor] ) return;
                        visited[neighbor] = 1;
                        queue.push_back( neighbor );
                    };
                    visit( x - 1, y, source.getUFaceOpenFraction( x, y ) );
                    visit( x + 1, y, source.getUFaceOpenFraction( x + 1, y ) );
                    visit( x, y - 1, source.getVFaceOpenFraction( x, y ) );
                    visit( x, y + 1, source.getVFaceOpenFraction( x, y + 1 ) );
                }
                const double meanSource = regionSource / queue.size();
                summedSource += regionSource;
                fluidCells += static_cast<int>( queue.size() );
                ++components;
                for( const int cell : queue )
                {
                    const auto budget = source.getCutCellVolumeBudget(
                        cell % width, cell / width, subDt );
                    const double corrected = budget.pressureResidualVolume - meanSource;
                    squared += corrected * corrected;
                    squaredFloor += meanSource * meanSource;
                    parityVolume += ( ( cell % width + cell / width ) & 1 )
                        ? -corrected : corrected;
                    if( locateResidualHotspots )
                    {
                        const int x = cell % width, y = cell / width;
                        const std::array<float, 4> apertures{{
                            source.getUFaceOpenFraction( x, y ),
                            source.getUFaceOpenFraction( x + 1, y ),
                            source.getVFaceOpenFraction( x, y ),
                            source.getVFaceOpenFraction( x, y + 1 )
                        }};
                        float minPositive = 1.0f, maxOpen = 0.0f;
                        int positiveFaces = 0;
                        for( const float aperture : apertures )
                        {
                            if( aperture <= 0.0f ) continue;
                            ++positiveFaces;
                            minPositive = std::min( minPositive, aperture );
                            maxOpen = std::max( maxOpen, aperture );
                        }
                        if( positiveFaces == 0 ) minPositive = 0.0f;
                        const int targetX = 16 * segments;
                        const int targetY = 16 * segments;
                        const int distance = std::abs( x - targetX ) +
                            std::min( std::abs( y - targetY ),
                                std::abs( y - ( targetY - 1 ) ) );
                        const double scale = static_cast<double>( h ) * h * subDt;
                        cellsWithResidual.push_back( { x, y,
                            corrected / scale, budget.pressureSourceDeltaVolume,
                            budget.snapshotDeltaVolume,
                            budget.solidSweptDivergenceVolume,
                            budget.macNetOutwardVolume, budget.currentFraction,
                            minPositive, maxOpen, positiveFaces, distance } );
                    }
                }
            }
            if( fluidCells == 0 ) fail( "reference has no active fluid cells" );
            const double scale = static_cast<double>( h ) * h * subDt;
            if( locateResidualHotspots )
            {
                if( cellsWithResidual.size() != static_cast<std::size_t>( fluidCells ) )
                    fail( "hotspot ledger did not cover every active cell" );
                std::sort( cellsWithResidual.begin(), cellsWithResidual.end(),
                    []( const hotspotCell& a, const hotspotCell& b )
                    {
                        if( std::abs( a.residual ) != std::abs( b.residual ) )
                            return std::abs( a.residual ) > std::abs( b.residual );
                        if( a.y != b.y ) return a.y < b.y;
                        return a.x < b.x;
                    } );
                double totalSquared = 0.0, topSquared = 0.0;
                double tinyFractionSquared = 0.0, tinyApertureSquared = 0.0;
                double nearTargetSquared = 0.0, signedCheckerboardSum = 0.0;
                for( std::size_t i = 0; i < cellsWithResidual.size(); ++i )
                {
                    const hotspotCell& row = cellsWithResidual[i];
                    if( !std::isfinite( row.residual ) ||
                        !std::isfinite( row.source ) ||
                        !std::isfinite( row.mac ) ||
                        !std::isfinite( row.fluidFraction ) )
                        fail( "nonfinite compatible residual hotspot" );
                    const double energy = row.residual * row.residual;
                    totalSquared += energy;
                    signedCheckerboardSum +=
                        ( ( row.x + row.y ) & 1 ) ? -row.residual : row.residual;
                    if( i < 8 ) topSquared += energy;
                    if( row.fluidFraction < 0.01f )
                        tinyFractionSquared += energy;
                    if( row.positiveFaces > 0 && row.minPositiveOpen < 0.01f )
                        tinyApertureSquared += energy;
                    if( row.targetDistance <= 3 ) nearTargetSquared += energy;
                }
                if( std::abs( totalSquared / fluidCells -
                    squared / ( fluidCells * scale * scale ) ) >
                    1.0e-7 * std::max( 1.0, totalSquared / fluidCells ) )
                    fail( "independent hotspot residual does not match global RMS" );
                const double denominator = totalSquared > 0.0 ? totalSquared : 1.0;
                const double parityMean = signedCheckerboardSum / fluidCells;
                const double parityShare =
                    signedCheckerboardSum * signedCheckerboardSum /
                    ( fluidCells * denominator );
                if( !std::isfinite( parityMean ) || !std::isfinite( parityShare ) ||
                    parityShare > 1.0 + 1.0e-6 )
                    fail( "checkerboard residual projection violated its bound" );
                std::printf( "zero_mac_hotspot_summary,%.9g,%d,%d,%d,"
                    "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
                    "%.12g,%.12g\n",
                    h, checkpoint, iterations, fluidCells,
                    std::sqrt( totalSquared / fluidCells ),
                    topSquared / denominator,
                    tinyFractionSquared / denominator,
                    tinyApertureSquared / denominator,
                    nearTargetSquared / denominator,
                    std::abs( cellsWithResidual.front().residual ),
                    parityMean, parityShare );
                for( std::size_t i = 0; i < std::min<std::size_t>(
                    8, cellsWithResidual.size() ); ++i )
                {
                    const hotspotCell& row = cellsWithResidual[i];
                    const vec2 world = source.getCellCenter( row.x, row.y );
                    std::printf( "zero_mac_hotspot,%.9g,%d,%d,%zu,"
                        "%d,%d,%.9g,%.9g,%.12g,"
                        "%.9g,%.9g,%.9g,%d,"
                        "%.12g,%.12g,%.12g,%.12g,%d\n",
                        h, checkpoint, iterations, i + 1,
                        row.x, row.y, world.x, world.y, row.residual,
                        row.fluidFraction, row.minPositiveOpen,
                        row.maxOpen, row.positiveFaces,
                        row.source, row.snapshot, row.swept, row.mac,
                        row.targetDistance );
                }
            }
            const double checkerboardShare = squared > 0.0
                ? parityVolume * parityVolume / ( fluidCells * squared ) : 0.0;
            if( !std::isfinite( checkerboardShare ) ||
                checkerboardShare < 0.0 || checkerboardShare > 1.0 + 1.0e-5 )
                fail( "invalid independently projected checkerboard share" );
            return std::tuple{
                std::sqrt( squared / fluidCells ) / scale,
                std::sqrt( squaredFloor / fluidCells ) / scale,
                summedSource, components, checkerboardShare };
        };

        int currentStep = -1, observed = 0;
        field.setPrePressureSnapshotObserver( [&]( const fluidField2& frozen )
        {
            const int step = currentStep + 1;
            const int cp = step * 16 / fluidSteps;
            if( step * 16 != cp * fluidSteps ||
                std::find( checkpoints.begin(), checkpoints.end(), cp ) ==
                    checkpoints.end() ) return;
            if( observed != 0 ) fail( "duplicated frozen checkpoint" );
            ++observed;
            std::optional<result> previous;
            for( int iterations : { 199, 200, 799, 800, 1599, 1600, 3199, 3200 } )
            {
                if( !locateResidualHotspots && ( iterations & 1 ) ) continue;
                fluidField2 trial = frozen;
                trial.setPrePressureSnapshotObserver( {} );
                if( !trial.setFrozenMacVelocityForAudit(
                    []( vec2 ) { return vec2{}; } ) )
                    fail( "frozen zero-MAC override refused" );
                // Actual open-face MAC speed is zero before the replay;
                // only the moving-volume source remains in pressure RHS.
                if( std::abs( targetFace( trial ) ) > 1.0e-12 )
                    fail( "reference input target face was not zero" );
                if( !trial.replayFrozenCutCellPressure( iterations ).has_value() )
                    fail( "bounded frozen noncoupled Jacobi replay refused" );
                const auto [rms, floorRms, sourceVolume, regions, checkerboardShare] =
                    compatibleResidual( trial, cp, iterations );
                const auto& pressure = trial.getCutCellPressure();
                const double face = targetFace( trial );
                if( !std::isfinite( rms ) || !std::isfinite( floorRms ) ||
                    !std::isfinite( sourceVolume ) || !std::isfinite( face ) ||
                    !std::isfinite( pressure.pressureResidual ) ||
                    pressure.usedCoupledProjection || pressure.coupledSolveFailed ||
                    pressure.connectedRegions != regions )
                    fail( "invalid independently reconstructed Jacobi reference" );
                if( previous && ( regions != previous->regions ||
                    std::abs( sourceVolume - previous->sourceVolume ) > 1.0e-8 ) )
                    fail( "frozen pressure source changed with Jacobi budget" );
                if( previous && iterations == 3200 &&
                    rms > previous->compatibleRms * 1.01 + 1.0e-6 )
                    fail( "extended Jacobi reference residual unexpectedly increased" );
                const result row{ h, cp, iterations, face, rms, floorRms,
                    sourceVolume, pressure.pressureResidual, regions,
                    checkerboardShare };
                if( locateResidualHotspots && ( iterations & 1 ) == 0 &&
                    previous && previous->iterations == iterations - 1 )
                    std::printf( "zero_mac_jacobi_parity,%.9g,%d,%d,"
                        "%.12g,%.12g,%.12g,%.12g,%.12g\n",
                        h, cp, iterations, previous->compatibleRms, rms,
                        previous->faceVolume, face,
                        face - previous->faceVolume );
                previous = row;
                if( ( iterations & 1 ) == 0 ) results.push_back( row );
                std::printf( "zero_mac_reference,%.9g,%d,%d,%d,"
                    "%.12g,%.12g,%.12g,%.9g,%.12g,%d\n",
                    h, fluidSteps, cp, iterations, face, rms, floorRms,
                    pressure.pressureResidual, sourceVolume, regions );
            }
            if( compareDampedJacobi )
                for( const int iterations : budgets )
                {
                    // Both paths start from one immutable pre-projection
                    // snapshot, including the SAME zeroed open MAC input.
                    // Only omega differs; no production step uses damping.
                    fluidField2 trial = frozen;
                    trial.setPrePressureSnapshotObserver( {} );
                    if( !trial.setFrozenMacVelocityForAudit(
                        []( vec2 ) { return vec2{}; } ) ||
                        std::abs( targetFace( trial ) ) > 1.0e-12 )
                        fail( "damped replay did not receive zero-open-MAC input" );
                    if( !trial.replayFrozenCutCellPressure(
                        iterations, false, 2.0f / 3.0f ).has_value() )
                        fail( "frozen damped Jacobi copy replay rejected" );
                    const auto [rms, floorRms, sourceVolume, regions, parityShare] =
                        compatibleResidual( trial, cp, iterations );
                    const auto& pressure = trial.getCutCellPressure();
                    const double face = targetFace( trial );
                    const result *baseline = nullptr;
                    for( const result& record : results )
                        if( record.h == h && record.checkpoint == cp &&
                            record.iterations == iterations )
                        {
                            baseline = &record;
                            break;
                        }
                    if( !baseline ||
                        !std::isfinite( rms ) || !std::isfinite( face ) ||
                        !std::isfinite( floorRms ) ||
                        !std::isfinite( parityShare ) ||
                        pressure.usedCoupledProjection || pressure.coupledSolveFailed ||
                        pressure.connectedRegions != regions ||
                        baseline->regions != regions ||
                        std::abs( baseline->sourceVolume - sourceVolume ) > 1.0e-8 ||
                        std::abs( baseline->compatibilityFloorRms - floorRms ) > 1.0e-8 )
                        fail( "damped and undamped frozen pressure sources differ" );
                    dampedResults.push_back( { h, cp, iterations, face,
                        rms, floorRms, sourceVolume, pressure.pressureResidual,
                        regions, parityShare } );
                    if( compareIndependentPcg && iterations == 3200 )
                    {
                        const auto independent = solveIndependentFrozenCutCellPressure(
                            trial, subDt, segments );
                        if( independent.components != regions ||
                            std::abs( independent.sourceVolume - sourceVolume ) > 1.0e-8 )
                            fail( "independent pressure assembly changed frozen source topology" );
                        independentResults.push_back( { h, cp, independent } );
                        std::printf( "zero_mac_independent_pcg,%.9g,%d,"
                            "%d,%d,%d,%d,%d,%.12g,%d,%.12g,%.12g,%d,%.12g,"
                            "%.12g,%.12g,%.12g\n",
                            h, cp, independent.activeCells,
                            independent.components,
                            independent.activePressureGroups,
                            independent.anchoredPressureGroups,
                            independent.activeToInactiveOpenFaces,
                            independent.activeToInactiveApertureSum,
                            independent.iterations,
                            independent.initialRmsPerSecond,
                            independent.finalRmsPerSecond,
                            independent.converged ? 1 : 0,
                            independent.sourceVolume,
                            independent.targetFaceVolume, face,
                            independent.targetFaceVolume - face );
                        if( !independent.converged )
                            fail( "independent pressure reference did not meet explicit residual" );
                        if( auditAnchors )
                        {
                            auditFrozenInactiveNeighborFaces( trial, subDt, cp,
                                independent.activeToInactiveOpenFaces,
                                independent.activeToInactiveApertureSum,
                                center, startAngle +
                                    ( endAngle - startAngle ) * cp / 16.0f );
                            const auto omitted = solveIndependentFrozenCutCellPressure(
                                trial, subDt, segments, true );
                            if( !omitted.converged ||
                                omitted.activeCells != independent.activeCells ||
                                omitted.components != independent.components ||
                                std::abs( omitted.sourceVolume -
                                    independent.sourceVolume ) > 1.0e-8 ||
                                omitted.anchoredPressureGroups != 0 ||
                                omitted.activeToInactiveOpenFaces != 0 )
                                fail( "counterfactual pressure edge removal invalid" );
                            if( independent.activeToInactiveOpenFaces == 0 &&
                                ( std::abs( independent.targetFaceVolume -
                                    omitted.targetFaceVolume ) > 1.0e-9 ||
                                    std::abs( omitted.compatibilityRebalanceL1Volume ) >
                                        1.0e-8 ) )
                                fail( "counterfactual altered no-anchor control" );
                            counterfactualResults.push_back( { h, cp, omitted } );
                            std::printf( "zero_mac_anchor_counterfactual,%.9g,%d,"
                                "%d,%.12g,%.12g,%.12g,%.12g,%.12g,"
                                "%.12g,%.12g,%d\n",
                                h, cp, independent.activeToInactiveOpenFaces,
                                independent.activeToInactiveApertureSum,
                                omitted.compatibilityRebalanceL1Volume,
                                independent.targetFaceVolume,
                                omitted.targetFaceVolume,
                                omitted.targetFaceVolume - independent.targetFaceVolume,
                                independent.finalRmsPerSecond,
                                omitted.finalRmsPerSecond,
                                omitted.converged ? 1 : 0 );
                        }
                    }
                    std::printf( "zero_mac_damped,%.9g,%d,%d,"
                        "%.12g,%.12g,%.12g,%.12g,"
                        "%.12g,%.12g,%.12g,%.12g\n",
                        h, cp, iterations, baseline->compatibleRms, rms,
                        baseline->checkerboardShare, parityShare,
                        baseline->faceVolume, face,
                        face - baseline->faceVolume, floorRms );
                }
        } );

        for( int step = 0; step < fluidSteps; ++step )
        {
            currentStep = step;
            observed = 0;
            capsule.transform.rotation = rot2::FromRadians( startAngle +
                static_cast<float>( step + 1 ) / fluidSteps *
                ( endAngle - startAngle ) );
            field.step( subDt, std::span<const fluidShapeObstacle2>( &capsule, 1 ),
                feedback, 120 );
            const int cp = ( step + 1 ) * 16 / fluidSteps;
            const bool due = ( step + 1 ) * 16 == cp * fluidSteps &&
                std::find( checkpoints.begin(), checkpoints.end(), cp ) !=
                    checkpoints.end();
            if( ( observed == 1 ) != due )
                fail( "frozen reference checkpoint coverage mismatch" );
        }
    }

    if( results.size() != 16 ||
        ( compareDampedJacobi && dampedResults.size() != 16 ) )
        fail( "expected two grids, two checkpoints and four Jacobi references" );
    for( int checkpoint : checkpoints )
    {
        const result *refCoarse = nullptr, *refFine = nullptr;
        for( const auto& record : results )
            if( record.checkpoint == checkpoint && record.iterations == 3200 )
            {
                if( record.h == 0.25f ) refCoarse = &record;
                else if( record.h == 0.125f ) refFine = &record;
            }
        if( !refCoarse || !refFine ) fail( "missing 3200-iteration reference pair" );
        const double referenceGap = refCoarse->faceVolume - refFine->faceVolume;
        for( int iterations : budgets )
        {
            const result *coarse = nullptr, *fine = nullptr;
            for( const auto& record : results )
                if( record.checkpoint == checkpoint &&
                    record.iterations == iterations )
                {
                    if( record.h == 0.25f ) coarse = &record;
                    else if( record.h == 0.125f ) fine = &record;
                }
            if( !coarse || !fine ) fail( "missing matching cross-grid reference" );
            const double gap = coarse->faceVolume - fine->faceVolume;
            // Convergence is explicitly assessed, never inferred from a
            // fixed iteration count or from close target flux alone.
            const bool referenceCriteriaMet =
                refCoarse->compatibleRms < 1.0e-3 &&
                refFine->compatibleRms < 1.0e-3 &&
                std::abs( gap - referenceGap ) < 1.0e-6;
            std::printf( "zero_mac_reference_cross,%d,%d,%d,"
                "%.12g,%.12g,%.12g,%d\n",
                fluidSteps, checkpoint, iterations, gap,
                coarse->compatibleRms - fine->compatibleRms,
                gap - referenceGap, referenceCriteriaMet ? 1 : 0 );
        }
    }
    if( auditAnchors )
    {
        if( counterfactualResults.size() != 4 || independentResults.size() != 4 )
            fail( "anchored-face counterfactual checkpoint coverage incomplete" );
        for( int cp : checkpoints )
        {
            const independentRecord *referenceCoarse = nullptr;
            const independentRecord *referenceFine = nullptr;
            const independentRecord *omittedCoarse = nullptr;
            const independentRecord *omittedFine = nullptr;
            for( const auto& row : independentResults )
                if( row.checkpoint == cp )
                {
                    if( row.h == 0.25f ) referenceCoarse = &row;
                    else if( row.h == 0.125f ) referenceFine = &row;
                }
            for( const auto& row : counterfactualResults )
                if( row.checkpoint == cp )
                {
                    if( row.h == 0.25f ) omittedCoarse = &row;
                    else if( row.h == 0.125f ) omittedFine = &row;
                }
            if( !referenceCoarse || !referenceFine || !omittedCoarse ||
                !omittedFine ) fail( "anchor counterfactual missing matched pair" );
            const double referenceGap =
                referenceCoarse->reference.targetFaceVolume -
                referenceFine->reference.targetFaceVolume;
            const double omittedGap =
                omittedCoarse->reference.targetFaceVolume -
                omittedFine->reference.targetFaceVolume;
            std::printf( "zero_mac_anchor_cross,%d,%.12g,%.12g,%.12g,%d\n",
                cp, referenceGap, omittedGap, omittedGap - referenceGap,
                omittedCoarse->reference.converged &&
                    omittedFine->reference.converged ? 1 : 0 );
        }
    }
    if( compareIndependentPcg )
    {
        if( independentResults.size() != 4 )
            fail( "independent pressure reference coverage incomplete" );
        for( int cp : checkpoints )
        {
            const independentRecord *coarse = nullptr, *fine = nullptr;
            const result *dampedCoarse = nullptr, *dampedFine = nullptr;
            for( const independentRecord& row : independentResults )
                if( row.checkpoint == cp )
                {
                    if( row.h == 0.25f ) coarse = &row;
                    else if( row.h == 0.125f ) fine = &row;
                }
            for( const result& row : dampedResults )
                if( row.checkpoint == cp && row.iterations == 3200 )
                {
                    if( row.h == 0.25f ) dampedCoarse = &row;
                    else if( row.h == 0.125f ) dampedFine = &row;
                }
            if( !coarse || !fine || !dampedCoarse || !dampedFine )
                fail( "independent pressure reference cross-grid pair missing" );
            const double independentGap = coarse->reference.targetFaceVolume -
                fine->reference.targetFaceVolume;
            const double dampedGap = dampedCoarse->faceVolume -
                dampedFine->faceVolume;
            std::printf( "zero_mac_independent_pcg_cross,%d,"
                "%.12g,%.12g,%.12g,%d\n",
                cp, independentGap, dampedGap, independentGap - dampedGap,
                coarse->reference.converged && fine->reference.converged ? 1 : 0 );
        }
    }
    if( compareDampedJacobi )
        for( const int cp : checkpoints )
            for( const int iterations : budgets )
            {
                const auto lookup = [&]( const std::vector<result>& rows, float h )
                    -> const result*
                {
                    for( const result& row : rows )
                        if( row.h == h && row.checkpoint == cp &&
                            row.iterations == iterations ) return &row;
                    return nullptr;
                };
                const result *legacyCoarse = lookup( results, 0.25f );
                const result *legacyFine = lookup( results, 0.125f );
                const result *dampedCoarse = lookup( dampedResults, 0.25f );
                const result *dampedFine = lookup( dampedResults, 0.125f );
                if( !legacyCoarse || !legacyFine || !dampedCoarse || !dampedFine )
                    fail( "missing matched damped Jacobi grid comparison" );
                const double legacyGap = legacyCoarse->faceVolume -
                    legacyFine->faceVolume;
                const double dampedGap = dampedCoarse->faceVolume -
                    dampedFine->faceVolume;
                const bool residualThresholdMet =
                    dampedCoarse->compatibleRms < 1.0e-3 &&
                    dampedFine->compatibleRms < 1.0e-3;
                std::printf( "zero_mac_damped_cross,%d,%d,"
                    "%.12g,%.12g,%.12g,%.12g,%d\n",
                    cp, iterations, legacyGap, dampedGap,
                    dampedCoarse->compatibleRms, dampedFine->compatibleRms,
                    residualThresholdMet ? 1 : 0 );
            }
}

// Opt-in, read-only endpoint volume ledger. These are not time-centered
// aperture integrals; every row is the LAST Fluid substep of a World frame.
// Different h/cadences have different obstacle histories, so compare only
// the two pressure variants inside one fixed h/cadence for causal A/B.
// Orthogonal time/space comparisons on exactly the same physical 0.25m
// U/V face segments. Each fine-grid shared face is counted once; never
// compare an h=0.125 face alone with a full h=0.25 coarse face.
void auditPrescribedSpatialTemporalConvergence()
{
    std::puts( "prescribed_spatial_cross,mode,fluid_substeps,face_type,face_x,"
        "face_y,coarse_h,refined_h,coarse_mac_linear_m3,refined_mac_linear_m3,"
        "coarse_open_time_s,refined_open_time_s,"
        "linear_delta_m3,open_time_delta_s" );
    std::puts( "prescribed_spatial_cross_summary,mode,fluid_substeps,faces,"
        "rms_linear_h025_vs_h0125_m3,rms_open_time_s,"
        "max_abs_linear_delta_m3,sign_mismatch_faces" );
    for( bool rotating : { false, true } )
    {
        const auto coarse = auditPrescribedMovingCutCellSubsteps(
            true, rotating, 0.25f, true );
        const auto refined = auditPrescribedMovingCutCellSubsteps(
            true, rotating, 0.125f, true );
        if( coarse.size() != refined.size() || coarse.size() != 40 )
        {
            std::fputs( "spatial reference samples did not match\n", stderr );
            std::exit( 1 );
        }
        // Samples are emitted in physical-face-major and cadence-minor
        // order, so each row is the exact same time/face test.
        for( int cadence : { 16, 32, 64, 128 } )
        {
            double squaredFlux = 0.0, squaredOpen = 0.0, maxAbsFlux = 0.0;
            int signsDifferent = 0, matching = 0;
            for( std::size_t i = 0; i < coarse.size(); ++i )
            {
                const auto& c = coarse[i];
                const auto& r = refined[i];
                if( c.substeps != r.substeps || c.faceType != r.faceType ||
                    c.x != r.x || c.y != r.y )
                {
                    std::fputs( "nonmatching physical face/time samples\n", stderr );
                    std::exit( 1 );
                }
                if( c.substeps != cadence ) continue;
                const double delta = c.macLinear - r.macLinear;
                const double openDelta = c.timeOpen - r.timeOpen;
                squaredFlux += delta * delta;
                squaredOpen += openDelta * openDelta;
                maxAbsFlux = std::max( maxAbsFlux, std::abs( delta ) );
                signsDifferent += c.macLinear * r.macLinear < 0.0;
                ++matching;
                std::printf( "prescribed_spatial_cross,%s,%d,%d,%d,%d,"
                    "0.25,0.125,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n",
                    rotating ? "rotation" : "translation",
                    cadence, c.faceType, c.x, c.y, c.macLinear, r.macLinear,
                    c.timeOpen, r.timeOpen, delta, openDelta );
            }
            if( matching != 10 || !std::isfinite( squaredFlux ) ||
                !std::isfinite( squaredOpen ) || !std::isfinite( maxAbsFlux ) )
            {
                std::fputs( "invalid spatial comparison\n", stderr );
                std::exit( 1 );
            }
            std::printf( "prescribed_spatial_cross_summary,%s,%d,%d,"
                "%.12g,%.12g,%.12g,%d\n",
                rotating ? "rotation" : "translation", cadence, matching,
                std::sqrt( squaredFlux / matching ),
                std::sqrt( squaredOpen / matching ), maxAbsFlux, signsDifferent );
        }
    }
}

void auditCutCellVolumeBudget()
{
    std::puts( "cell_m,fluid_substeps,frame,pressure_pool,cell_x,cell_y,"
        "history,swept_history,fluid_fraction_previous,fluid_fraction_current,"
        "snapshot_delta_m3,pressure_source_delta_m3,swept_divergence_m3,"
        "mac_endpoint_out_m3,pressure_residual_m3,snapshot_residual_m3,"
        "active_cells,modified_cells,modified_solid_cells,"
        "pressure_rms_m3,snapshot_rms_m3,"
        "max_pressure_residual_m3,worst_pressure_x,worst_pressure_y,"
        "max_snapshot_residual_m3,worst_snapshot_x,worst_snapshot_y,"
        "total_snapshot_delta_m3,total_pressure_source_delta_m3,"
        "total_mac_endpoint_out_m3,swept_snapshot_mismatch_l1_m3,"
        "pcg_status,merged_pressure_cells" );
    for( float h : { 0.5f, 0.25f } )
        for( int fluidSubsteps : { 2, 4, 8 } )
            for( bool pooling : { false, true } )
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
                scene.setWindSmallCellPressureMergeFraction( pooling ? 1.0e-4f : 0.0f );
                const auto shapes = scene.getShapes();
                scene.getWorld().SetBodyLinearVelocity(
                    shapes[0].bodyHandle, { 0.7f, -0.10f } );
                scene.getWorld().SetBodyAngularVelocity( shapes[1].bodyHandle, 0.5f );

                for( int frame = 0; frame < 20; ++frame )
                {
                    scene.step( dt, worldSubsteps );
                    if( frame != 5 && frame != 10 && frame != 19 ) continue;
                    const fluidField2& field = *scene.getWindField();
                    const int targetX = static_cast<int>( 18.0f * 0.25f / h );
                    const int targetY = static_cast<int>( 13.0f * 0.25f / h );
                    const float fluidDt = dt / fluidSubsteps;
                    const auto cell =
                        field.getCutCellVolumeBudget( targetX, targetY, fluidDt );
                    const auto all = field.getCutCellVolumeSummary( fluidDt );
                    if( !cell.valid || !cell.hasVolumeHistory || !all.valid ||
                        all.activeCells <= 0 ||
                        !std::isfinite( cell.pressureResidualVolume ) ||
                        !std::isfinite( cell.snapshotResidualVolume ) ||
                        !std::isfinite( all.activePressureResidualRmsVolume ) ||
                        !std::isfinite( all.activeSnapshotResidualRmsVolume ) )
                    {
                        std::fprintf( stderr, "invalid Cut-cell endpoint volume audit\n" );
                        std::exit( 1 );
                    }
                    // Validate dimensional and sign conventions against
                    // the exact four-face ledger, not a velocity-only proxy.
                    const double sumMac =
                        cell.macOutwardVolume[0] + cell.macOutwardVolume[1] +
                        cell.macOutwardVolume[2] + cell.macOutwardVolume[3];
                    if( std::abs( sumMac - cell.macNetOutwardVolume ) > 1.0e-12 ||
                        std::abs( cell.pressureResidualVolume -
                            ( sumMac + cell.pressureSourceDeltaVolume ) ) > 1.0e-12 )
                    {
                        std::fprintf( stderr, "Cut-cell endpoint flux/sign mismatch\n" );
                        std::exit( 1 );
                    }

                    std::printf( "%.9g,%d,%d,%d,%d,%d,"
                        "%d,%d,%.9g,%.9g,"
                        "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
                        "%d,%d,%d,%.9g,%.9g,"
                        "%.9g,%d,%d,%.9g,%d,%d,"
                        "%.9g,%.9g,%.9g,%.9g,%d,%d\n",
                        h, fluidSubsteps, frame, pooling ? 1 : 0, targetX, targetY,
                        cell.hasVolumeHistory ? 1 : 0, cell.hasSweptHistory ? 1 : 0,
                        cell.previousFraction, cell.currentFraction,
                        cell.snapshotDeltaVolume, cell.pressureSourceDeltaVolume,
                        cell.solidSweptDivergenceVolume, cell.macNetOutwardVolume,
                        cell.pressureResidualVolume, cell.snapshotResidualVolume,
                        all.activeCells, all.modifiedCells, all.modifiedFullyCoveredCells,
                        all.activePressureResidualRmsVolume, all.activeSnapshotResidualRmsVolume,
                        all.maxAbsPressureResidualVolume, all.worstPressureX, all.worstPressureY,
                        all.maxAbsSnapshotResidualVolume, all.worstSnapshotX, all.worstSnapshotY,
                        all.totalSnapshotDeltaVolume, all.totalPressureSourceDeltaVolume,
                        all.totalMacNetOutwardVolume,
                        all.totalAbsSweptSnapshotMismatchVolume,
                        static_cast<int>( field.getCutCellPressure().coupledStatus ),
                        field.getCutCellPressure().mergedPressureCells );
                }
            }
}



// Independent signed-distance / one-dimensional integration oracle for the
// tiny lower-right open wedge of the rotating Capsule at h=.125, cp=12/16.
// No engine cell-area/clipped-face helper is called, and no solver state is
// changed. For this controlled corner, bottom/right are open only near their
// intersection and the opposite sides are strictly inside the Capsule.
struct capsuleCornerWedge2
{
    double cornerClearance = 0.0;
    double bottomOpenLength = 0.0;
    double rightOpenLength = 0.0;
    double area128 = 0.0;
    double area1024 = 0.0;
};

capsuleCornerWedge2 integrateCapsuleCornerWedge(
    double ax, double ay, double bx, double by, double radius,
    double x0, double y0, double h )
{
    const double x1 = x0 + h, y1 = y0 + h;
    const double dx = bx - ax, dy = by - ay;
    const double lengthSquared = dx * dx + dy * dy;
    if( !( lengthSquared > 0.0 ) || !( radius > 0.0 ) )
        throw std::runtime_error( "invalid capsule corner distance oracle" );
    const auto signedDistance = [&]( double x, double y )
    {
        const double t = std::clamp(
            ( ( x - ax ) * dx + ( y - ay ) * dy ) / lengthSquared, 0.0, 1.0 );
        return std::hypot( x - ( ax + t * dx ), y - ( ay + t * dy ) ) - radius;
    };
    const double corner = signedDistance( x1, y0 );
    if( !( corner > 0.0 ) ||
        !( signedDistance( x0, y0 ) < 0.0 ) ||
        !( signedDistance( x1, y1 ) < 0.0 ) ||
        !( signedDistance( x0, y1 ) < 0.0 ) )
        throw std::runtime_error( "capsule corner witness topology changed" );
    // The exterior is to the RIGHT of the bottom crossing and BELOW
    // the right crossing. Bisection is confined to independently
    // verified sign changes, not to engine's analytic clipping intervals.
    const auto crossing = []( double lo, double hi, auto positiveAtHigh )
    {
        for( int iteration = 0; iteration < 75; ++iteration )
        {
            const double middle = 0.5 * ( lo + hi );
            if( positiveAtHigh( middle ) ) hi = middle;
            else lo = middle;
        }
        return 0.5 * ( lo + hi );
    };
    const double bottomCross = crossing( x0, x1,
        [&]( double x ) { return signedDistance( x, y0 ) > 0.0; } );
    const double rightCross = crossing( y0, y1,
        [&]( double y ) { return signedDistance( x1, y ) < 0.0; } );

    capsuleCornerWedge2 output{};
    output.cornerClearance = corner;
    output.bottomOpenLength = x1 - bottomCross;
    output.rightOpenLength = rightCross - y0;
    const auto integrate = [&]( int count )
    {
        const double step = output.bottomOpenLength / count;
        double sum = 0.0;
        for( int i = 0; i < count; ++i )
        {
            const double x = bottomCross + ( i + 0.5 ) * step;
            const double yCross = crossing( y0, y1,
                [&]( double y ) { return signedDistance( x, y ) < 0.0; } );
            sum += yCross - y0;
        }
        return step * sum;
    };
    output.area128 = integrate( 128 );
    output.area1024 = integrate( 1024 );
    if( !( output.bottomOpenLength > 0.0 ) ||
        !( output.rightOpenLength > 0.0 ) ||
        !( output.area1024 > 0.0 ) ||
        !std::isfinite( output.area1024 ) ||
        std::abs( output.area128 - output.area1024 ) >
            1.0e-3 * output.area1024 + 1.0e-15 )
        throw std::runtime_error( "independent capsule wedge quadrature failed" );
    return output;
}

void auditTinyCapsuleCorner( const fluidField2& frozen,
    const fluidShapeObstacle2& capsule, int mode, int checkpoint )
{
    if( frozen.getCellSize() != 0.125f || checkpoint != 12 ) return;
    constexpr int x = 28, y = 26;
    const double h = frozen.getCellSize();
    const vec2 origin = frozen.getOrigin();
    const double x0 = origin.x + x * h, y0 = origin.y + y * h;
    const auto* shape = std::get_if<capsule2>( &capsule.geometry );
    if( !shape ) throw std::runtime_error( "independent corner audit expects Capsule" );

    const vec2 actualA = TransformPoint( capsule.transform, shape->center1 );
    const vec2 actualB = TransformPoint( capsule.transform, shape->center2 );
    const double radius = shape->radius;
    // Pure double-angle reference; the production endpoints above retain
    // its float rot2/TransformPoint path for an exact input comparison.
    constexpr float startAngle = 0.18f, endAngle = 1.28f;
    const double nominalAngle = static_cast<double>( startAngle ) +
        12.0 / 16.0 * ( static_cast<double>( endAngle ) -
            static_cast<double>( startAngle ) );
    const double cx = capsule.transform.position.x;
    const double cy = capsule.transform.position.y;
    const double halfLength = 0.5 * static_cast<double>(
        shape->center2.x - shape->center1.x );
    const double idealAx = cx - halfLength * std::cos( nominalAngle );
    const double idealAy = cy - halfLength * std::sin( nominalAngle );
    const double idealBx = cx + halfLength * std::cos( nominalAngle );
    const double idealBy = cy + halfLength * std::sin( nominalAngle );

    std::puts( "capsule_corner_oracle,analytic,endpoint_policy,radius_ulp,"
        "corner_clearance_m,bottom_open_m,right_open_m,"
        "exterior_area_m2,exterior_fraction,quadrature_difference_m2,"
        "engine_cell_fraction,engine_bottom_face_open,"
        "engine_right_face_open,oracle_bottom_face_open,"
        "oracle_right_face_open" );
    for( int policy = 0; policy < 2; ++policy )
        for( int ulp = -1; ulp <= 1; ++ulp )
        {
            const double testedRadius = ulp == -1
                ? std::nextafter( shape->radius, 0.0f )
                : ulp == 1
                    ? std::nextafter( shape->radius,
                        std::numeric_limits<float>::infinity() )
                    : radius;
            const capsuleCornerWedge2 wedge = policy == 0
                ? integrateCapsuleCornerWedge(
                    actualA.x, actualA.y, actualB.x, actualB.y,
                    testedRadius, x0, y0, h )
                : integrateCapsuleCornerWedge(
                    idealAx, idealAy, idealBx, idealBy,
                    testedRadius, x0, y0, h );
            const double bottomOpen = wedge.bottomOpenLength / h;
            const double rightOpen = wedge.rightOpenLength / h;
            const float engineBottom = frozen.getVFaceOpenFraction( x, y );
            const float engineRight = frozen.getUFaceOpenFraction( x + 1, y );
            if( policy == 0 && ulp == 0 && mode == 1 &&
                ( std::abs( bottomOpen - engineBottom ) > 1.0e-5 ||
                  std::abs( rightOpen - engineRight ) > 1.0e-5 ) )
                throw std::runtime_error( "independent capsule faces disagree with analytic clipping" );
            std::printf( "capsule_corner_oracle,%d,%d,%d,"
                "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
                "%.12g,%.12g,%.12g,%.12g,%.12g\n",
                mode, policy, ulp, wedge.cornerClearance,
                wedge.bottomOpenLength, wedge.rightOpenLength,
                wedge.area1024, wedge.area1024 / ( h * h ),
                wedge.area128 - wedge.area1024,
                frozen.getCellFluidFraction( x, y ),
                engineBottom, engineRight, bottomOpen, rightOpen );
        }
}

// S2c geometry-only control: hold the prescribed rotating Capsule, grid,
// fluid cadence and checkpoint fixed while switching only the existing
// opt-in analytic Capsule geometry. The underlying Fluid histories are
// separately evolved; pressure results are independently assembled from
// each frozen geometry after zeroing the MAC input. Not a same-state
// pressure-operator counterfactual or a production stabilization proposal.
void auditFrozenCapsuleGeometryComparison()
{
    constexpr float totalDt = 1.0f / 30.0f;
    constexpr int fluidSteps = 128;
    constexpr float startAngle = 0.18f, endAngle = 1.28f;
    constexpr vec2 center{ -0.20f, -0.12f };
    constexpr std::array<int, 2> checkpoints{{ 11, 12 }};
    const auto fail = []( const char* message ) { throw std::runtime_error( message ); };
    struct capture
    {
        bool valid = false;
        int width = 0, height = 0, active = 0, anchors = 0;
        double anchorOpening = 0.0;
        std::vector<float> cells, uOpen, vOpen;
        std::vector<double> sources;
        independentCutCellPressureResult pressure{};
    };

    std::puts( "capsule_geometry_summary,cell_m,checkpoint_16,analytic,"
        "active_cells,anchor_faces,anchor_aperture_sum,pcg_iterations,"
        "pcg_explicit_rms_per_s,reference_target_face_m3,"
        "pressure_source_active_m3,pcg_converged" );
    std::puts( "capsule_geometry_pair,cell_m,checkpoint_16,"
        "sampled_active,analytic_active,sampled_anchors,analytic_anchors,"
        "sampled_anchor_aperture,analytic_anchor_aperture,"
        "reclassified_zero_cells,max_cell_fraction_difference,"
        "max_face_aperture_difference,pressure_source_L1_difference_m3,"
        "analytic_minus_sampled_target_face_m3" );
    std::puts( "capsule_geometry_anchor_pair,cell_m,checkpoint_16,axis,x,y,"
        "sampled_open,analytic_open,sampled_fraction_a,sampled_fraction_b,"
        "analytic_fraction_a,analytic_fraction_b,"
        "sampled_source_a_m3,sampled_source_b_m3,"
        "analytic_source_a_m3,analytic_source_b_m3" );

    for( float h : { 0.25f, 0.125f } )
    {
        const int segments = static_cast<int>( std::lround( 0.25f / h ) );
        const float subDt = totalDt / fluidSteps;
        std::array<std::array<capture, 2>, 2> observations{};
        for( int mode = 0; mode < 2; ++mode )
        {
            fluidField2 field{ 32 * segments, 32 * segments, h, { -4.0f, -4.0f } };
            field.setCutCellProjectionEnabled( true );
            field.setDisjointAnalyticCutCellEnabled( true );
            field.setAnalyticCapsuleCutCellEnabled( mode != 0 );
            if( field.getSmallCellPressureMergeFraction() != 0.0f ||
                field.isConnectedResidualTracerRemapEnabled() )
                fail( "experimental pressure or tracer physics enabled in geometry control" );
            fluidShapeObstacle2 capsule{
                capsule2{ { -0.63f, 0.0f }, { 0.63f, 0.0f }, 0.19f },
                { center, rot2::FromRadians( startAngle ) }, center,
                {}, 0.0f, 0.0f, 0.0f };
            capsule.angularVelocity = ( endAngle - startAngle ) / totalDt;
            std::array<fluidObstacleForce2, 1> feedback{};
            field.step( subDt, std::span<const fluidShapeObstacle2>( &capsule, 1 ),
                feedback, 120 );
            field.addVelocity( { -0.24f, -0.30f }, { 1.3f, 0.65f }, 0.65f );
            field.addVelocity( { 0.35f, 0.25f }, { -0.55f, 0.8f }, 0.60f );

            int currentStep = -1;
            field.setPrePressureSnapshotObserver( [&]( const fluidField2& frozen )
            {
                const int step = currentStep + 1;
                const int cp = step * 16 / fluidSteps;
                if( step * 16 != cp * fluidSteps ||
                    std::find( checkpoints.begin(), checkpoints.end(), cp ) ==
                        checkpoints.end() ) return;
                const int slot = cp == checkpoints[0] ? 0 : 1;
                capture& row = observations[mode][slot];
                if( row.valid || frozen.isAnalyticCapsuleCutCellEnabled() != ( mode != 0 ) ||
                    frozen.getCutCellTransition().usedDisjointAnalyticGeometry )
                    fail( "invalid Capsule frozen geometry checkpoint" );

                fluidField2 trial = frozen;
                trial.setPrePressureSnapshotObserver( {} );
                if( !trial.setFrozenMacVelocityForAudit(
                    []( vec2 ) { return vec2{}; } ) )
                    fail( "frozen Capsule geometry MAC override refused" );
                row.width = trial.getWidth();
                row.height = trial.getHeight();
                const int width = row.width, height = row.height;
                row.cells.resize( width * height );
                row.sources.resize( width * height );
                row.uOpen.resize( ( width + 1 ) * height );
                row.vOpen.resize( width * ( height + 1 ) );
                const auto cellIndex = [width]( int x, int y ) { return y * width + x; };
                for( int y = 0; y < height; ++y )
                    for( int x = 0; x < width; ++x )
                    {
                        const int cell = cellIndex( x, y );
                        const auto budget = trial.getCutCellVolumeBudget( x, y, subDt );
                        if( !budget.valid || !budget.hasVolumeHistory )
                            fail( "missing frozen Capsule geometry source" );
                        row.cells[cell] = budget.currentFraction;
                        row.sources[cell] = budget.pressureSourceDeltaVolume;
                        row.active += budget.currentFraction > 0.0f;
                    }
                for( int y = 0; y < height; ++y )
                    for( int x = 0; x <= width; ++x )
                    {
                        const float open = trial.getUFaceOpenFraction( x, y );
                        row.uOpen[y * ( width + 1 ) + x] = open;
                        if( x > 0 && x < width && open > 0.0f &&
                            ( row.cells[cellIndex( x - 1, y )] > 0.0f ) !=
                            ( row.cells[cellIndex( x, y )] > 0.0f ) )
                        {
                            ++row.anchors;
                            row.anchorOpening += open;
                        }
                    }
                for( int y = 0; y <= height; ++y )
                    for( int x = 0; x < width; ++x )
                    {
                        const float open = trial.getVFaceOpenFraction( x, y );
                        row.vOpen[y * width + x] = open;
                        if( y > 0 && y < height && open > 0.0f &&
                            ( row.cells[cellIndex( x, y - 1 )] > 0.0f ) !=
                            ( row.cells[cellIndex( x, y )] > 0.0f ) )
                        {
                            ++row.anchors;
                            row.anchorOpening += open;
                        }
                    }
                row.pressure = solveIndependentFrozenCutCellPressure(
                    trial, subDt, segments );
                if( row.pressure.activeCells != row.active ||
                    row.pressure.activeToInactiveOpenFaces != row.anchors ||
                    std::abs( row.pressure.activeToInactiveApertureSum -
                        row.anchorOpening ) > 1.0e-7 ||
                    !std::isfinite( row.pressure.targetFaceVolume ) ||
                    !std::isfinite( row.pressure.finalRmsPerSecond ) )
                    fail( "independent Capsule pressure/geometry ledger mismatch" );
                auditTinyCapsuleCorner( frozen, capsule, mode, cp );
                row.valid = true;
                std::printf( "capsule_geometry_summary,%.9g,%d,%d,%d,%d,"
                    "%.12g,%d,%.12g,%.12g,%.12g,%d\n",
                    h, cp, mode, row.active, row.anchors, row.anchorOpening,
                    row.pressure.iterations, row.pressure.finalRmsPerSecond,
                    row.pressure.targetFaceVolume, row.pressure.sourceVolume,
                    row.pressure.converged ? 1 : 0 );
            } );

            for( int step = 0; step < fluidSteps; ++step )
            {
                currentStep = step;
                capsule.transform.rotation = rot2::FromRadians( startAngle +
                    static_cast<float>( step + 1 ) / fluidSteps *
                    ( endAngle - startAngle ) );
                field.step( subDt, std::span<const fluidShapeObstacle2>( &capsule, 1 ),
                    feedback, 120 );
            }
            if( !observations[mode][0].valid || !observations[mode][1].valid )
                fail( "missing Capsule geometry checkpoint" );
        }

        for( int slot = 0; slot < 2; ++slot )
        {
            const capture& sampled = observations[0][slot];
            const capture& analytic = observations[1][slot];
            const int cp = checkpoints[slot];
            if( sampled.width != analytic.width ||
                sampled.height != analytic.height ||
                sampled.cells.size() != analytic.cells.size() ||
                sampled.uOpen.size() != analytic.uOpen.size() ||
                sampled.vOpen.size() != analytic.vOpen.size() )
                fail( "Capsule geometry comparison grid mismatch" );
            double maxCellDifference = 0.0, maxFaceDifference = 0.0;
            double sourceDifference = 0.0;
            int reclassified = 0;
            for( std::size_t i = 0; i < sampled.cells.size(); ++i )
            {
                maxCellDifference = std::max( maxCellDifference,
                    std::abs( static_cast<double>( sampled.cells[i] ) - analytic.cells[i] ) );
                sourceDifference += std::abs( sampled.sources[i] - analytic.sources[i] );
                reclassified += sampled.cells[i] == 0.0f && analytic.cells[i] > 0.0f;
            }
            for( std::size_t i = 0; i < sampled.uOpen.size(); ++i )
                maxFaceDifference = std::max( maxFaceDifference, std::abs(
                    static_cast<double>( sampled.uOpen[i] ) - analytic.uOpen[i] ) );
            for( std::size_t i = 0; i < sampled.vOpen.size(); ++i )
                maxFaceDifference = std::max( maxFaceDifference, std::abs(
                    static_cast<double>( sampled.vOpen[i] ) - analytic.vOpen[i] ) );

            std::printf( "capsule_geometry_pair,%.9g,%d,%d,%d,%d,%d,"
                "%.12g,%.12g,%d,%.12g,%.12g,%.12g,%.12g\n",
                h, cp, sampled.active, analytic.active,
                sampled.anchors, analytic.anchors,
                sampled.anchorOpening, analytic.anchorOpening,
                reclassified, maxCellDifference, maxFaceDifference,
                sourceDifference,
                analytic.pressure.targetFaceVolume - sampled.pressure.targetFaceVolume );

            const int width = sampled.width, height = sampled.height;
            const auto emit = [&]( char axis, int x, int y, int ax, int ay,
                int bx, int by, float sampledOpen, float analyticOpen )
            {
                const int ai = ay * width + ax, bi = by * width + bx;
                const bool sampledAnchor = sampledOpen > 0.0f &&
                    ( ( sampled.cells[ai] > 0.0f ) != ( sampled.cells[bi] > 0.0f ) );
                const bool analyticAnchor = analyticOpen > 0.0f &&
                    ( ( analytic.cells[ai] > 0.0f ) != ( analytic.cells[bi] > 0.0f ) );
                if( !sampledAnchor && !analyticAnchor ) return;
                std::printf( "capsule_geometry_anchor_pair,%.9g,%d,%c,%d,%d,"
                    "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
                    "%.12g,%.12g,%.12g,%.12g\n",
                    h, cp, axis, x, y, sampledOpen, analyticOpen,
                    sampled.cells[ai], sampled.cells[bi],
                    analytic.cells[ai], analytic.cells[bi],
                    sampled.sources[ai], sampled.sources[bi],
                    analytic.sources[ai], analytic.sources[bi] );
            };
            for( int y = 0; y < height; ++y )
                for( int x = 1; x < width; ++x )
                {
                    const int i = y * ( width + 1 ) + x;
                    emit( 'U', x, y, x - 1, y, x, y,
                        sampled.uOpen[i], analytic.uOpen[i] );
                }
            for( int y = 1; y < height; ++y )
                for( int x = 0; x < width; ++x )
                {
                    const int i = y * width + x;
                    emit( 'V', x, y, x, y - 1, x, y,
                        sampled.vOpen[i], analytic.vOpen[i] );
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
    bool smallCellTransferLedgerAudit = false;
    bool smallCellConnectedResidualAudit = false;
    bool smallCellConnectedCapacityAudit = false;
    bool smallCellGroupedFaceAudit = false;
    bool smallCellTimeVaryingFluxAudit = false;
    bool prescribedSubstepsAudit = false;
    bool prescribedConvergenceAudit = false;
    bool prescribedSpatialAudit = false;
    bool prescribedRotationPressureSplitAudit = false;
    bool frozenPressureIterationAudit = false;
    bool matchedWorldPressureRestartAudit = false;
    bool zeroMacPressureAudit = false;
    bool zeroMacConvergenceAudit = false;
    bool zeroMacReferenceAudit = false;
    bool zeroMacResidualMapAudit = false;
    bool zeroMacDampedJacobiAudit = false;
    bool zeroMacIndependentPcgAudit = false;
    bool zeroMacAnchorAudit = false;
    bool capsuleGeometryAudit = false;
    bool cutCellVolumeAudit = false;
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
        smallCellTransferLedgerAudit |= std::string_view( argv[i] ) == "--small-cell-applied-transfer-audit";
        smallCellConnectedResidualAudit |= std::string_view( argv[i] ) == "--small-cell-connected-residual-audit";
        smallCellConnectedCapacityAudit |= std::string_view( argv[i] ) == "--small-cell-connected-capacity-audit";
        smallCellGroupedFaceAudit |= std::string_view( argv[i] ) == "--small-cell-grouped-face-audit";
        smallCellTimeVaryingFluxAudit |= std::string_view( argv[i] ) == "--small-cell-time-varying-flux-audit";
        prescribedSubstepsAudit |= std::string_view( argv[i] ) == "--small-cell-prescribed-substeps-audit";
        prescribedConvergenceAudit |= std::string_view( argv[i] ) == "--small-cell-prescribed-convergence-audit";
        prescribedSpatialAudit |= std::string_view( argv[i] ) == "--small-cell-prescribed-spatial-audit";
        prescribedRotationPressureSplitAudit |= std::string_view( argv[i] ) == "--small-cell-rotation-pressure-split-audit";
        frozenPressureIterationAudit |= std::string_view( argv[i] ) == "--small-cell-frozen-pressure-iterations-audit";
        matchedWorldPressureRestartAudit |= std::string_view( argv[i] ) == "--small-cell-matched-world-restart-audit";
        zeroMacPressureAudit |= std::string_view( argv[i] ) == "--small-cell-zero-mac-pressure-audit";
        zeroMacConvergenceAudit |= std::string_view( argv[i] ) == "--small-cell-zero-mac-convergence-audit";
        zeroMacReferenceAudit |= std::string_view( argv[i] ) == "--small-cell-zero-mac-reference-audit";
        zeroMacResidualMapAudit |= std::string_view( argv[i] ) == "--small-cell-zero-mac-residual-map-audit";
        zeroMacDampedJacobiAudit |= std::string_view( argv[i] ) == "--small-cell-zero-mac-damped-jacobi-audit";
        zeroMacIndependentPcgAudit |= std::string_view( argv[i] ) == "--small-cell-zero-mac-independent-pcg-audit";
        zeroMacAnchorAudit |= std::string_view( argv[i] ) == "--small-cell-zero-mac-anchor-audit";
        capsuleGeometryAudit |= std::string_view( argv[i] ) == "--small-cell-capsule-geometry-audit";
        cutCellVolumeAudit |= std::string_view( argv[i] ) == "--cut-cell-volume-budget-audit";
    }
    if( couplingAudit ) { auditCoupling(); return 0; }
    if( faceStageAudit ) { auditFaceStages(); return 0; }
    if( frozenPressureAudit ) { auditFrozenPressure(); return 0; }
    if( smallCellMergeAudit ) { auditFrozenPressure( true ); return 0; }
    if( smallCellThresholdSweep ) { auditFrozenPressure( true, true ); return 0; }
    if( smallCellCadenceAudit ) { auditSmallCellPressureCadences(); return 0; }
    if( smallCellWorldFeedbackAudit ) { auditSmallCellWorldFeedback(); return 0; }
    if( smallCellLocalTracerAudit ) { auditSmallCellLocalTracer(); return 0; }
    if( smallCellTransferLedgerAudit ) { auditSmallCellAppliedTransfers(); return 0; }
    if( smallCellConnectedResidualAudit ) { auditSmallCellAppliedTransfers( true ); return 0; }
    if( smallCellConnectedCapacityAudit ) { auditSmallCellAppliedTransfers( true, true ); return 0; }
    if( smallCellGroupedFaceAudit ) { auditSmallCellAppliedTransfers( true, true, true ); return 0; }
    if( smallCellTimeVaryingFluxAudit ) { auditSmallCellAppliedTransfers( true, true, true, true ); return 0; }
    if( prescribedSubstepsAudit ) { auditPrescribedMovingCutCellSubsteps(); return 0; }
    if( prescribedConvergenceAudit )
    {
        auditPrescribedMovingCutCellSubsteps( true, false );
        auditPrescribedMovingCutCellSubsteps( true, true );
        return 0;
    }
    if( prescribedSpatialAudit ) { auditPrescribedSpatialTemporalConvergence(); return 0; }
    if( prescribedRotationPressureSplitAudit ) { auditPrescribedRotatingFacePressureSplit(); return 0; }
    if( frozenPressureIterationAudit ) { auditRotatingFrozenPressureIterationSweep(); return 0; }
    if( matchedWorldPressureRestartAudit ) { auditMatchedWorldMacPressureRestart(); return 0; }
    if( zeroMacPressureAudit ) { auditMatchedWorldMacPressureRestart( true ); return 0; }
    if( zeroMacConvergenceAudit )
    {
        auditMatchedWorldMacPressureRestart( true, true );
        return 0;
    }
    if( zeroMacReferenceAudit ) { auditFrozenZeroMacReference(); return 0; }
    if( zeroMacResidualMapAudit )
    {
        auditFrozenZeroMacReference( true );
        return 0;
    }
    if( zeroMacDampedJacobiAudit )
    {
        auditFrozenZeroMacReference( false, true );
        return 0;
    }
    if( zeroMacIndependentPcgAudit )
    {
        auditFrozenZeroMacReference( false, true, true );
        return 0;
    }
    if( zeroMacAnchorAudit )
    {
        auditFrozenZeroMacReference( false, true, true, true );
        return 0;
    }
    if( capsuleGeometryAudit ) { auditFrozenCapsuleGeometryComparison(); return 0; }
    if( cutCellVolumeAudit ) { auditCutCellVolumeBudget(); return 0; }
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
