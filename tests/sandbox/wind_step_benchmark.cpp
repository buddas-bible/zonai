#include "rigidBodyDemo.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <limits>
#include <iterator>
#include <numeric>
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
// S2c exactly grid-aligned Capsule tangent-side regression. Legacy and
// inclusive runs consume identical MAC input/obstacle snapshots; the latter
// changes only the analytic Capsule face early-rejection rule. Neither path
// is used by normal World code until a caller explicitly opts in.
void auditCapsuleTangentFacePressure()
{
    std::puts( "capsule_tangent_summary,case_id,inclusive,active_cells,"
        "source_u_mps,source_v_mps,exchange_x_ns,"
        "exchange_y_ns,abs_six_cell_y_ns,signed_six_cell_y_ns,"
        "coupled_failed,pressure_relative_residual,tracer_kg,"
        "step_ms" );
    std::puts( "capsule_tangent_face,inclusive,x,face_y,"
        "expected_flat_contact_m,raw_blocked_m,"
        "fluid_y_ns,solid_y_ns,wall_y_ns,total_y_ns" );
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.configureWindField( 48, 28, 0.25f );
    scene.setWindCutCellProjectionEnabled( true );
    scene.setWindCutCellCoupledProjectionEnabled( true );
    scene.setWindDisjointAnalyticEnabled( true );
    scene.setWindSynchronizedCouplingEnabled( true );
    scene.setWindObstacleCouplingEnabled( true );
    scene.setWindDragCoefficient( 0.0f );
    scene.setWindSmallCellPressureMergeFraction( 0.0f );
    const auto shapes = scene.getShapes();
    if( shapes.size() < 3 || !scene.getWindField() )
        throw std::runtime_error( "missing Capsule tangent World fixture" );
    scene.getWorld().SetBodyLinearVelocity(
        shapes[0].bodyHandle, { 0.7f, -0.10f } );
    scene.getWorld().SetBodyAngularVelocity( shapes[1].bodyHandle, 0.5f );

    std::array<fluidShapeObstacle2, 3> obstacles{};
    for( int i = 0; i < 3; ++i )
    {
        const bodyId body = shapes[i].bodyHandle;
        const transform2 transform = scene.getWorld().GetBodyTransform( body );
        const float mass = scene.getWorld().GetBodyMass( body );
        const float inertia = scene.getWorld().GetBodyRotationalInertia( body );
        if( !( mass > 0.0f && inertia > 0.0f ) )
            throw std::runtime_error( "invalid Capsule tangent body mass" );
        obstacles[i] = {
            scene.getWorld().GetShape( shapes[i].shapeHandle ).geometry,
            transform,
            TransformPoint( transform, scene.getWorld().GetBodyLocalCenter( body ) ),
            scene.getWorld().GetBodyLinearVelocity( body ),
            scene.getWorld().GetBodyAngularVelocity( body ),
            1.0f / mass, 1.0f / inertia
        };
    }
    const auto* capsule = std::get_if<capsule2>( &obstacles[1].geometry );
    if( !capsule ) throw std::runtime_error( "expected Capsule in fixture" );
    const vec2 axisStart = TransformPoint(
        obstacles[1].transform, capsule->center1 );
    const vec2 axisEnd = TransformPoint(
        obstacles[1].transform, capsule->center2 );
    if( std::abs( axisStart.y - axisEnd.y ) > 1.0e-6f )
        throw std::runtime_error( "Capsule tangent fixture not horizontal" );

    const auto entry = *scene.getWindField();
    const float h = entry.getCellSize();
    const float sourceX = entry.getOrigin().x +
        std::min( 2.5f, 0.25f * entry.getWidth() * h );
    const float sourceU = 8.0f * dt;
    const float sourceV = 4.0f * dt * std::sin( 1.7f * dt );
    const double startX = std::min( axisStart.x, axisEnd.x );
    const double endX = std::max( axisStart.x, axisEnd.x );

    for( const int scenario : { 0, 1, 2 } )
    {
        auto fixture = obstacles;
        std::span<const fluidShapeObstacle2> chosen( fixture );
        if( scenario == 1 )
        {
            fixture[1].transform.rotation = rot2::FromRadians( 0.23f );
            const bodyId body = shapes[1].bodyHandle;
            fixture[1].center = TransformPoint( fixture[1].transform,
                scene.getWorld().GetBodyLocalCenter( body ) );
        }
        else if( scenario == 2 )
        {
            // A circle touching a grid line has only a zero-length tangent
            // at the extreme point, unlike a Capsule core flat side.
            fixture[0].geometry = circle2{ {}, 0.25f };
            fixture[0].transform.position.y = 1.0f;
            fixture[0].center = fixture[0].transform.position;
            chosen = std::span<const fluidShapeObstacle2>(
                fixture.data(), 1 );
        }

        std::array<fluidField2, 2> replay{{ entry, entry }};
        std::array<double, 2> residualY{};
        int changedFaces = 0;
        for( int policy = 0; policy < 2; ++policy )
        {
            auto& field = replay[policy];
            field.setAnalyticCapsuleTangentFaceAuditEnabled( policy == 1 );
            if( scenario == 2 ) field.setAnalyticCircleCutCellEnabled( true );
            field.setStepTimingEnabled( true );
            field.setTracerConcentration( 10, 12, 1.0f );
            const double beforeMass = field.getTracerMass();
            field.addVelocity( { sourceX, 0.0f },
                { sourceU, sourceV }, 1.7f );
            std::vector<fluidObstacleForce2> feedback( chosen.size() );
            field.step( dt, chosen, feedback, 200 );
            const auto& pressure = field.getCutCellPressure();
            if( !pressure.usedCoupledProjection ||
                pressure.coupledSolveFailed ||
                ( scenario != 2 && !pressure.hasAnalyticImpulseBudget ) ||
                pressure.mergedPressureCells != 0 )
            {
                std::fprintf( stderr,
                    "tangent_invalid,scenario=%d,policy=%d,status=%d,"
                    "coupled=%d,failed=%d,has_impulse=%d,merged=%d,"
                    "iterations=%d,relative_residual=%.12g,active=%d\\n",
                    scenario, policy, static_cast<int>( pressure.coupledStatus ),
                    pressure.usedCoupledProjection ? 1 : 0,
                    pressure.coupledSolveFailed ? 1 : 0,
                    pressure.hasAnalyticImpulseBudget ? 1 : 0,
                    pressure.mergedPressureCells, pressure.coupledIterations,
                    static_cast<double>( pressure.coupledRelativeResidual ),
                    pressure.activeCells );
                throw std::runtime_error( "invalid tangent pressure coupled solve" );
            }
            const double mass = field.getTracerMass();
            if( !std::isfinite( mass ) ||
                std::abs( mass - beforeMass ) > 1.0e-6 )
                throw std::runtime_error( "Capsule tangent tracer mass drift" );
            // Existing pressure impulse ledger covers analytic multi-shape
            // Schur only; the single-Circle control checks pressure directly.
            const auto ledger = scenario == 2
                ? std::optional<std::vector<fluidCutCellPressureCellImpulseAudit2>>{}
                : field.getCutCellPressureCellImpulseAudit( dt );
            if( scenario != 2 && !ledger )
                throw std::runtime_error( "missing tangent multi-shape cell ledger" );
            double signedSix = 0.0, absoluteSix = 0.0;
            if( scenario == 0 )
                for( const int faceY : { 17, 19 } )
                    for( int x = 24; x <= 26; ++x )
                    {
                        const int cellY = faceY == 17 ? 16 : 19;
                        const auto& cell = ( *ledger )[cellY * field.getWidth() + x];
                        const double faceCoordinate = field.getOrigin().y + faceY * h;
                        const double tangentY = axisStart.y +
                            ( faceY == 17 ? -capsule->radius : capsule->radius );
                        if( std::abs( faceCoordinate - tangentY ) > 1.0e-6 )
                            throw std::runtime_error( "Capsule side not grid-aligned" );
                        const double faceLeft = field.getOrigin().x + x * h;
                        const double expectedLength = std::max( 0.0,
                            std::min( faceLeft + h, endX ) -
                            std::max( faceLeft, startX ) );
                        const double observedLength = h *
                            ( 1.0 - field.getVFaceOpenFraction( x, faceY ) );
                        if( std::abs( expectedLength - h ) > 1.0e-6 ||
                            cell.fraction < 1.0 - 1.0e-6 )
                            throw std::runtime_error( "invalid tangent face fixture" );
                        if( policy == 1 &&
                            ( std::abs( observedLength - expectedLength ) > 1.0e-6 ||
                              std::abs( cell.residualY() ) > 2.0e-6 ) )
                            throw std::runtime_error(
                                "inclusive tangent did not restore MAC/Schur cancellation" );
                        if( policy == 0 && observedLength > 1.0e-6 )
                            throw std::runtime_error( "legacy tangent unexpectedly included" );
                        if( std::abs( cell.wallY ) > 1.0e-8 )
                            throw std::runtime_error( "tangent interior cell has wall impulse" );
                        signedSix += cell.residualY();
                        absoluteSix += std::abs( cell.residualY() );
                        std::printf( "capsule_tangent_face,%d,%d,%d,"
                            "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n",
                            policy, x, faceY, expectedLength, observedLength,
                            cell.fluidY, cell.solidY, cell.wallY, cell.residualY() );
                    }

            residualY[policy] = pressure.impulse.residual().y;
            std::printf( "capsule_tangent_summary,%d,%d,%d,"
                "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
                "%d,%.12g,%.12g,%.12g\n",
                scenario, policy, pressure.activeCells,
                static_cast<double>( sourceU ), static_cast<double>( sourceV ),
                static_cast<double>( pressure.impulse.residual().x ),
                residualY[policy], absoluteSix, signedSix,
                pressure.coupledSolveFailed ? 1 : 0,
                static_cast<double>( pressure.coupledRelativeResidual ),
                mass, field.getStepTiming().total );
        }
        int changedUFaces = 0, changedVFaces = 0;
        int fullLengthFaces = 0, partialLengthFaces = 0;
        std::puts( "capsule_tangent_changed_face,case_id,UV,x,y,legacy_open,"
            "inclusive_open,raw_restored_length_m,flat_core_oracle_length_m,"
            "is_tangent_flat_side" );
        for( int y = 0; y < entry.getHeight(); ++y )
            for( int x = 0; x < entry.getWidth(); ++x )
            {
                if( std::abs( replay[0].getCellFluidFraction( x, y ) -
                    replay[1].getCellFluidFraction( x, y ) ) > 1.0e-6 )
                    throw std::runtime_error(
                        "tangent policy changed cell area rather than face clip" );
                if( scenario != 0 && std::abs(
                    replay[0].getCellPressureForVisualization( x, y ) -
                    replay[1].getCellPressureForVisualization( x, y ) ) > 1.0e-6 )
                    throw std::runtime_error(
                        "tangent policy changed rotated/Circle pressure" );
                const auto compareFace = [&]( bool vertical )
                {
                    const float oldOpen = vertical
                        ? replay[0].getUFaceOpenFraction( x, y )
                        : replay[0].getVFaceOpenFraction( x, y );
                    const float newOpen = vertical
                        ? replay[1].getUFaceOpenFraction( x, y )
                        : replay[1].getVFaceOpenFraction( x, y );
                    if( std::abs( newOpen - oldOpen ) <= 1.0e-6f )
                        return;
                    ++changedFaces;
                    if( vertical ) ++changedUFaces;
                    else ++changedVFaces;
                    const double length = static_cast<double>( h ) *
                        ( static_cast<double>( oldOpen ) - newOpen );
                    const double coordinate = vertical
                        ? entry.getOrigin().x + x * h
                        : entry.getOrigin().y + y * h;
                    const bool onFlatSide = scenario == 0 && !vertical &&
                        ( std::abs( coordinate - ( axisStart.y - capsule->radius ) ) <
                              1.0e-6 ||
                          std::abs( coordinate - ( axisStart.y + capsule->radius ) ) <
                              1.0e-6 );
                    const double faceLeft = entry.getOrigin().x + x * h;
                    const double oracle = onFlatSide
                        ? std::max( 0.0, std::min( faceLeft + h, endX ) -
                            std::max( faceLeft, startX ) )
                        : 0.0;
                    std::printf( "capsule_tangent_changed_face,%d,%c,%d,%d,"
                        "%.12g,%.12g,%.12g,%.12g,%d\n",
                        scenario, vertical ? 'U' : 'V', x, y,
                        static_cast<double>( oldOpen ),
                        static_cast<double>( newOpen ), length, oracle,
                        onFlatSide ? 1 : 0 );
                    if( scenario == 0 )
                    {
                        if( !onFlatSide || length <= 0.0 ||
                            std::abs( length - oracle ) > 1.0e-6 )
                            throw std::runtime_error(
                                "tangent face differed outside independent flat-side oracle" );
                        if( std::abs( length - h ) <= 1.0e-6 )
                            ++fullLengthFaces;
                        else ++partialLengthFaces;
                    }
                };
                if( x > 0 ) compareFace( true );
                if( y > 0 ) compareFace( false );
            }
        std::printf( "capsule_tangent_comparison,%d,%d,%.12g,%.12g\n",
            scenario, changedFaces, residualY[0], residualY[1] );
        if( scenario == 0 )
        {
            std::printf( "capsule_tangent_changed_summary,%d,%d,%d,%d,%d\n",
                changedFaces, changedUFaces, changedVFaces,
                fullLengthFaces, partialLengthFaces );
            if( changedFaces != 10 || changedUFaces != 0 ||
                changedVFaces != 10 || fullLengthFaces != 6 ||
                partialLengthFaces != 4 ||
                std::abs( residualY[0] ) < 1.0e-3 ||
                std::abs( residualY[1] ) >
                    0.05 * std::abs( residualY[0] ) )
                throw std::runtime_error(
                    "tangent policy failed to reduce grid-aligned pressure mismatch" );
        }
        else if( changedFaces != 0 ||
            std::abs( residualY[1] - residualY[0] ) > 1.0e-6 )
            throw std::runtime_error( "tangent policy changed rotated or Circle control" );
    }
}

// Controlled moving-Capsule S2c policy A/B. Both policies are fixed before
// the first step, and the same prescribed obstacle path and Wind forcing are
// supplied to independent copies. The frame-9 entry velocities are compared
// explicitly; a later trajectory difference is not a frozen-state comparison.
void auditCapsuleTangentMovingPressure()
{
    constexpr int width = 64, height = 64;
    constexpr float h = 0.125f, dt = 1.0f / 60.0f;
    constexpr float radius = 0.19f, halfLength = 0.63f;
    constexpr float startY = -3.02f, lowerY = -3.55f;
    constexpr vec2 origin{ -4.0f, -4.0f };
    constexpr int donorX = 26, donorY = 4;
    std::puts( "capsule_tangent_motion,aligned_frame9,frame,policy,pose_y,"
        "pre_mac_max_diff,pre_fraction_max_diff,changed_u,changed_v,"
        "swept,donor_fraction,donor_snapshot_m3,donor_source_m3,"
        "donor_gcl_m3,donor_residual_m3,global_residual_m3,"
        "pressure_span,peak_fluid_face_mps,force_y_N,tracer_mass_kg,step_ms" );

    for( const bool aligned : { false, true } )
    {
        fluidField2 initial{ width, height, h, origin };
        initial.setCutCellProjectionEnabled( true );
        initial.setAnalyticCapsuleCutCellEnabled( true );
        initial.setAnalyticCapsuleDoubleGeometryEnabled( true );
        initial.setAnalyticCapsuleFullCoverageGuardEnabled( true );
        initial.setTightTranslatingCapsuleSweptBoundsAuditEnabled( true );
        if( initial.isCutCellCoupledProjectionEnabled() ||
            initial.getSmallCellPressureMergeFraction() != 0.0f )
            throw std::runtime_error( "moving tangent reference solver changed" );
        std::array<fluidField2, 2> fields{{ initial, initial }};
        fields[1].setAnalyticCapsuleTangentFaceAuditEnabled( true );
        for( auto& field : fields ) field.setStepTimingEnabled( true );
        // The original near-wall fixture retains radius=0.19f. The
        // exact-tangent control uses 3/16 m: center and radius are then
        // exactly representable floats at the selected grid line. With
        // 0.19f, double(center)-double(radius) misses -3.5 by 5.96e-8 m.
        const float testRadius = aligned ? 0.1875f : radius;
        fluidShapeObstacle2 moving{
            capsule2{ { -halfLength, 0.0f }, { halfLength, 0.0f }, testRadius },
            { { 0.0f, startY }, {} }, { 0.0f, startY },
            {}, 0.0f, 0.0f, 0.0f
        };
        float priorY = startY;
        std::array<double, 2> seedMass{};
        for( int frame = 0; frame < 27; ++frame )
        {
            const float fraction = frame < 16
                ? static_cast<float>( frame + 1 ) / 16.0f
                : static_cast<float>( 31 - frame ) / 16.0f;
            float y = startY + ( lowerY - startY ) * fraction;
            // The second path crosses a *true* grid tangent on frame 9;
            // the ordinary boundary-spike frame 9 remains unmodified.
            if( aligned && frame == 9 )
            {
                y = -3.5f + testRadius;
                if( static_cast<double>( y ) -
                    static_cast<double>( testRadius ) != -3.5 )
                    throw std::runtime_error( "tangent fixture not binary-exact" );
            }
            moving.transform.position.y = y;
            moving.center = moving.transform.position;
            moving.linearVelocity.y = ( y - priorY ) / dt;
            if( y - testRadius <= origin.y )
                throw std::runtime_error( "moving tangent Capsule hit outer wall" );
            for( auto& field : fields )
            {
                if( frame == 9 )
                    field.setTracerConcentration( donorX, donorY, 1.0f );
                const float time = ( frame + 1 ) * dt;
                field.addVelocity( { -2.0f, 0.0f },
                    { 6.0f * dt, 3.0f * dt * std::sin( 1.7f * time ) },
                    1.7f );
            }
            if( frame == 9 )
                for( int policy = 0; policy < 2; ++policy )
                {
                    seedMass[policy] = fields[policy].getTracerMass();
                    if( seedMass[policy] <= 0.0 )
                        throw std::runtime_error( "moving tangent seed missing" );
                }
            double maxEntryMacDiff = 0.0, maxEntryFractionDiff = 0.0;
            if( frame == 9 )
            {
                for( int cy = 0; cy < height; ++cy )
                    for( int cx = 0; cx < width; ++cx )
                    {
                        maxEntryFractionDiff = std::max( maxEntryFractionDiff,
                            std::abs( static_cast<double>(
                                fields[0].getCellFluidFraction( cx, cy ) ) -
                                fields[1].getCellFluidFraction( cx, cy ) ) );
                        if( cx > 0 )
                            maxEntryMacDiff = std::max( maxEntryMacDiff,
                                std::abs( static_cast<double>(
                                    *fields[0].getUFaceVelocityForAudit( cx, cy ) ) -
                                    *fields[1].getUFaceVelocityForAudit( cx, cy ) ) );
                        if( cy > 0 )
                            maxEntryMacDiff = std::max( maxEntryMacDiff,
                                std::abs( static_cast<double>(
                                    *fields[0].getVFaceVelocityForAudit( cx, cy ) ) -
                                    *fields[1].getVFaceVelocityForAudit( cx, cy ) ) );
                    }
                if( maxEntryMacDiff > 1.0e-6 ||
                    maxEntryFractionDiff > 1.0e-6 )
                    throw std::runtime_error(
                        "moving tangent frame-9 entry Fields were not matched" );
            }
            std::array<fluidObstacleForce2, 2> forces{};
            for( int policy = 0; policy < 2; ++policy )
            {
                std::array<fluidObstacleForce2, 1> feedback{};
                fields[policy].step( dt,
                    std::span<const fluidShapeObstacle2>( &moving, 1 ),
                    feedback, 80 );
                forces[policy] = feedback[0];
                if( frame >= 9 &&
                    std::abs( fields[policy].getTracerMass() -
                        seedMass[policy] ) > 1.0e-5 )
                    throw std::runtime_error( "moving tangent tracer not conserved" );
            }
            int changedU = 0, changedV = 0;
            for( int cy = 0; cy < height; ++cy )
                for( int cx = 0; cx < width; ++cx )
                {
                    if( std::abs(
                        fields[0].getCellFluidFraction( cx, cy ) -
                        fields[1].getCellFluidFraction( cx, cy ) ) > 1.0e-6 )
                        throw std::runtime_error( "moving tangent changed cell volume" );
                    if( cx > 0 && std::abs(
                        fields[0].getUFaceOpenFraction( cx, cy ) -
                        fields[1].getUFaceOpenFraction( cx, cy ) ) > 1.0e-6 )
                        ++changedU;
                    if( cy > 0 && std::abs(
                        fields[0].getVFaceOpenFraction( cx, cy ) -
                        fields[1].getVFaceOpenFraction( cx, cy ) ) > 1.0e-6 )
                        ++changedV;
                }
            if( frame == 9 )
            {
                if( aligned && ( changedU != 0 || changedV < 6 ) )
                {
                    std::fprintf( stderr,
                        "moving_tangent_bad_faces,u=%d,v=%d,y=%.12g,"
                        "radius=%.12g,bottom=%.12g\n",
                        changedU, changedV, static_cast<double>( y ),
                        static_cast<double>( testRadius ),
                        static_cast<double>( y ) - testRadius );
                    throw std::runtime_error(
                        "moving grid-tangent frame failed to restore flat-side V faces" );
                }
                if( !aligned && changedU + changedV != 0 )
                    throw std::runtime_error(
                        "ordinary frame-9 unexpectedly hit exact tangent" );
            }

            if( frame == 4 || frame == 9 || frame == 10 ||
                frame == 16 || frame == 26 )
                for( int policy = 0; policy < 2; ++policy )
                {
                    const auto& field = fields[policy];
                    const auto donor = field.getCutCellVolumeBudget(
                        donorX, donorY, dt );
                    const auto volume = field.getCutCellVolumeSummary( dt );
                    const auto flow = field.getFlowAudit( dt );
                    if( !donor.valid || !volume.valid ||
                        !std::isfinite( flow.maxFluidFaceSpeed ) ||
                        !std::isfinite( forces[policy].force.y ) )
                        throw std::runtime_error( "moving tangent invalid pressure budget" );
                    double minPressure = 0.0, maxPressure = 0.0;
                    int active = 0;
                    for( int cy = 0; cy < height; ++cy )
                        for( int cx = 0; cx < width; ++cx )
                            if( field.getCellFluidFraction( cx, cy ) > 0.0f )
                            {
                                const double p =
                                    field.getCellPressureForVisualization( cx, cy );
                                if( !std::isfinite( p ) )
                                    throw std::runtime_error( "moving tangent nonfinite pressure" );
                                if( active++ == 0 ) minPressure = maxPressure = p;
                                else
                                {
                                    minPressure = std::min( minPressure, p );
                                    maxPressure = std::max( maxPressure, p );
                                }
                            }
                    const std::array<double, 11> numbers{{
                        donor.currentFraction, donor.snapshotDeltaVolume,
                        donor.pressureSourceDeltaVolume,
                        donor.snapshotDeltaVolume -
                            donor.solidSweptDivergenceVolume,
                        donor.pressureResidualVolume,
                        volume.maxAbsPressureResidualVolume,
                        maxPressure - minPressure, flow.maxFluidFaceSpeed,
                        forces[policy].force.y, field.getTracerMass(),
                        field.getStepTiming().total
                    }};
                    std::printf( "capsule_tangent_motion,%d,%d,%d,"
                        "%.12g,%.12g,%.12g,%d,%d,%d",
                        aligned ? 1 : 0, frame, policy,
                        static_cast<double>( y ), maxEntryMacDiff,
                        maxEntryFractionDiff, changedU, changedV,
                        field.getSweptFaceFlux().usedSweptFlux ? 1 : 0 );
                    for( const double v : numbers )
                    {
                        if( !std::isfinite( v ) )
                            throw std::runtime_error( "moving tangent nonfinite metric" );
                        std::printf( ",%.12g", v );
                    }
                    std::putchar( '\n' );
                    // Identify the worst global continuity cell independently
                    // of the donor. Read-only: this cannot alter the solver.
                    if( frame == 9 || frame == 10 )
                    {
                        double worstMagnitude = -1.0;
                        int worstX = -1, worstY = -1;
                        fluidCutCellVolumeBudget2 worst{};
                        for( int cy = 0; cy < height; ++cy )
                            for( int cx = 0; cx < width; ++cx )
                            {
                                const auto candidate =
                                    field.getCutCellVolumeBudget( cx, cy, dt );
                                if( !candidate.valid || !candidate.active ) continue;
                                const double magnitude =
                                    std::abs( candidate.pressureResidualVolume );
                                if( !std::isfinite( magnitude ) )
                                    throw std::runtime_error(
                                        "moving tangent nonfinite cell residual" );
                                if( magnitude > worstMagnitude )
                                {
                                    worstMagnitude = magnitude;
                                    worstX = cx;
                                    worstY = cy;
                                    worst = candidate;
                                }
                            }
                        if( worstX < 0 )
                            throw std::runtime_error(
                                "moving tangent missing active pressure cell" );
                        std::printf( "capsule_tangent_worst_cell,%d,%d,%d,%d,%d,"
                            "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n",
                            aligned ? 1 : 0, frame, policy, worstX, worstY,
                            worst.currentFraction, worst.snapshotDeltaVolume,
                            worst.pressureSourceDeltaVolume,
                            worst.solidSweptDivergenceVolume,
                            worst.pressureResidualVolume,
                            volume.maxAbsPressureResidualVolume );
                    }
                }
            priorY = y;
        }
    }
}

// Read-only per-pressure-unknown accounting of the startup Schur exchange.
// Geometry/World inputs are identical to the prior first-history experiment.
// None, horizontal, vertical and combined Wind sources separate the forcing
// component that excites the observed Y pressure-exchange defect.
void auditFirstCutCellPressureCoefficients()
{
    std::puts( "world_pressure_coeff_summary,forcing_mode,source_u_mps,"
        "source_v_mps,active_cells,used_coupled,pcg_failed,"
        "ledger_residual_x_ns,ledger_residual_y_ns,"
        "cell_residual_x_ns,cell_residual_y_ns,"
        "cell_fluid_x_ns,cell_fluid_y_ns,"
        "cell_solid_x_ns,cell_solid_y_ns,"
        "cell_wall_x_ns,cell_wall_y_ns,"
        "largest_abs_cell_residual_y_ns,top8_abs_cell_residual_y_ns,"
        "pressure_compatibility_m3ps" );
    std::puts( "world_pressure_coeff_cell,forcing_mode,rank,x,y,"
        "fraction,pressure,open_w,open_e,open_s,open_n,"
        "fluid_x_ns,fluid_y_ns,solid_x_ns,solid_y_ns,"
        "wall_x_ns,wall_y_ns,total_x_ns,total_y_ns" );

    for( const int forcing : { 0, 1, 2, 3 } )
    {
        rigidBodyDemo scene{ demoKind::windFlow };
        scene.configureWindField( 48, 28, 0.25f );
        scene.setWindCutCellProjectionEnabled( true );
        scene.setWindCutCellCoupledProjectionEnabled( true );
        scene.setWindDisjointAnalyticEnabled( true );
        scene.setWindSynchronizedCouplingEnabled( true );
        scene.setWindObstacleCouplingEnabled( true );
        scene.setWindDragCoefficient( 0.0f );
        scene.setWindSmallCellPressureMergeFraction( 0.0f );
        const auto shapes = scene.getShapes();
        if( shapes.size() < 3 || !scene.getWindField() )
            throw std::runtime_error( "missing World for pressure coefficient audit" );
        scene.getWorld().SetBodyLinearVelocity(
            shapes[0].bodyHandle, { 0.7f, -0.10f } );
        scene.getWorld().SetBodyAngularVelocity( shapes[1].bodyHandle, 0.5f );

        std::array<fluidShapeObstacle2, 3> obstacles{};
        for( int i = 0; i < 3; ++i )
        {
            const bodyId body = shapes[i].bodyHandle;
            const transform2 transform = scene.getWorld().GetBodyTransform( body );
            const float mass = scene.getWorld().GetBodyMass( body );
            const float inertia = scene.getWorld().GetBodyRotationalInertia( body );
            if( !( mass > 0.0f && inertia > 0.0f ) )
                throw std::runtime_error( "invalid pressure coefficient Body mass" );
            obstacles[i] = {
                scene.getWorld().GetShape( shapes[i].shapeHandle ).geometry,
                transform,
                TransformPoint( transform, scene.getWorld().GetBodyLocalCenter( body ) ),
                scene.getWorld().GetBodyLinearVelocity( body ),
                scene.getWorld().GetBodyAngularVelocity( body ),
                1.0f / mass, 1.0f / inertia
            };
        }
        fluidField2 field = *scene.getWindField();
        const float h = field.getCellSize();
        const float sourceX = field.getOrigin().x +
            std::min( 2.5f, 0.25f * field.getWidth() * h );
        const float sourceU = ( forcing & 1 ) ? 8.0f * dt : 0.0f;
        const float sourceV = ( forcing & 2 )
            ? 4.0f * dt * std::sin( 1.7f * dt ) : 0.0f;
        if( forcing != 0 )
            field.addVelocity( { sourceX, 0.0f }, { sourceU, sourceV }, 1.7f );
        std::array<fluidObstacleForce2, 3> forces{};
        field.step( dt, obstacles, forces, 200 );
        const auto& projection = field.getCutCellPressure();
        auto opt = field.getCutCellPressureCellImpulseAudit( dt );
        if( !opt || !projection.usedCoupledProjection ||
            !projection.hasAnalyticImpulseBudget ||
            projection.coupledSolveFailed ||
            projection.mergedPressureCells != 0 )
            throw std::runtime_error( "invalid cell pressure exchange audit" );
        auto& cells = *opt;
        const auto& ledger = projection.impulse;
        double fluidX = 0.0, fluidY = 0.0;
        double solidX = 0.0, solidY = 0.0;
        double wallX = 0.0, wallY = 0.0;
        int active = 0;
        for( const auto& cell : cells )
        {
            if( cell.fraction > 0.0 ) ++active;
            fluidX += cell.fluidX;
            fluidY += cell.fluidY;
            solidX += cell.solidX;
            solidY += cell.solidY;
            wallX += cell.wallX;
            wallY += cell.wallY;
        }
        const double sumX = fluidX + solidX + wallX;
        const double sumY = fluidY + solidY + wallY;
        const double tolerance = 1.0e-5 * std::max( 1.0,
            std::max( static_cast<double>( Length( ledger.fluid ) ),
                static_cast<double>( Length( ledger.wall ) ) ) );
        if( std::abs( fluidX - ledger.fluid.x ) > tolerance ||
            std::abs( fluidY - ledger.fluid.y ) > tolerance ||
            std::abs( solidX - ledger.solid.x ) > tolerance ||
            std::abs( solidY - ledger.solid.y ) > tolerance ||
            std::abs( wallX - ledger.wall.x ) > tolerance ||
            std::abs( wallY - ledger.wall.y ) > tolerance ||
            std::abs( sumX - ledger.residual().x ) > tolerance ||
            std::abs( sumY - ledger.residual().y ) > tolerance )
            throw std::runtime_error(
                "pressure-cell coefficient totals do not reconstruct live PCG" );

        std::sort( cells.begin(), cells.end(), []( const auto& a, const auto& b )
        {
            const double aa = std::abs( a.residualY() );
            const double bb = std::abs( b.residualY() );
            if( aa != bb ) return aa > bb;
            return a.y != b.y ? a.y < b.y : a.x < b.x;
        } );
        const double largest = cells.empty() ? 0.0 :
            std::abs( cells.front().residualY() );
        double top8 = 0.0;
        for( std::size_t i = 0; i < std::min<std::size_t>( 8, cells.size() ); ++i )
            top8 += std::abs( cells[i].residualY() );
        const std::array<double, 14> numbers{{
            static_cast<double>( ledger.residual().x ),
            static_cast<double>( ledger.residual().y ),
            sumX, sumY, fluidX, fluidY, solidX, solidY,
            wallX, wallY, largest, top8,
            static_cast<double>( projection.compatibilityCorrection ),
            static_cast<double>( field.getFlowAudit( dt ).maxFluidFaceSpeed )
        }};
        for( double v : numbers )
            if( !std::isfinite( v ) )
                throw std::runtime_error( "nonfinite pressure coefficient result" );
        std::printf( "world_pressure_coeff_summary,%d,%.12g,%.12g,%d,%d,%d",
            forcing, static_cast<double>( sourceU ), static_cast<double>( sourceV ),
            active, projection.usedCoupledProjection ? 1 : 0,
            projection.coupledSolveFailed ? 1 : 0 );
        for( int i = 0; i < 13; ++i ) std::printf( ",%.12g", numbers[i] );
        std::putchar( '\n' );
        for( std::size_t rank = 0; rank < std::min<std::size_t>( 12, cells.size() ); ++rank )
        {
            const auto& c = cells[rank];
            std::printf( "world_pressure_coeff_cell,%d,%zu,%d,%d,"
                "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
                "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n",
                forcing, rank + 1, c.x, c.y, c.fraction, c.pressure,
                c.openWest, c.openEast, c.openSouth, c.openNorth,
                c.fluidX, c.fluidY, c.solidX, c.solidY,
                c.wallX, c.wallY, c.residualX(), c.residualY() );
        }
    }
}

// S2c startup controlled comparison: the same first Fluid input and actual
// World obstacle snapshot, with or without an explicitly primed (stationary)
// geometry-only history. No World step, no alteration of normal Fluid Step.
void auditFirstCutCellHistoryImpulse()
{
    std::puts( "world_pressure_first_history,source_on,history_primed,"
        "has_volume_history,used_swept,used_coupled,pcg_failed,"
        "fluid_jx_ns,fluid_jy_ns,solid_jx_ns,solid_jy_ns,"
        "wall_jx_ns,wall_jy_ns,residual_jx_ns,residual_jy_ns,"
        "independent_fluid_jx_ns,independent_fluid_jy_ns,"
        "independent_wall_jx_ns,independent_wall_jy_ns,"
        "solid_feedback_jx_ns,solid_feedback_jy_ns,"
        "solid_feedback_torque_ns_m,changed_cells,"
        "pressure_source_m3,compatibility_m3ps,peak_face_speed_mps" );

    for( const bool sourceOn : { false, true } )
    {
        rigidBodyDemo scene{ demoKind::windFlow };
        scene.configureWindField( 48, 28, 0.25f );
        scene.setWindCutCellProjectionEnabled( true );
        scene.setWindCutCellCoupledProjectionEnabled( true );
        scene.setWindDisjointAnalyticEnabled( true );
        scene.setWindSynchronizedCouplingEnabled( true );
        scene.setWindObstacleCouplingEnabled( true );
        scene.setWindDragCoefficient( 0.0f );
        scene.setWindSmallCellPressureMergeFraction( 0.0f );
        const auto shapes = scene.getShapes();
        if( shapes.size() < 3 || !scene.getWindField() )
            throw std::runtime_error( "first-history missing World obstacles" );
        scene.getWorld().SetBodyLinearVelocity( shapes[0].bodyHandle,
            { 0.7f, -0.10f } );
        scene.getWorld().SetBodyAngularVelocity( shapes[1].bodyHandle, 0.5f );

        std::array<fluidShapeObstacle2, 3> obstacles{};
        for( int i = 0; i < 3; ++i )
        {
            const bodyId body = shapes[i].bodyHandle;
            const transform2 transform = scene.getWorld().GetBodyTransform( body );
            const float mass = scene.getWorld().GetBodyMass( body );
            const float inertia = scene.getWorld().GetBodyRotationalInertia( body );
            if( !( mass > 0.0f && inertia > 0.0f ) )
                throw std::runtime_error( "first-history invalid Body mass" );
            obstacles[i] = {
                scene.getWorld().GetShape( shapes[i].shapeHandle ).geometry,
                transform,
                TransformPoint( transform, scene.getWorld().GetBodyLocalCenter( body ) ),
                scene.getWorld().GetBodyLinearVelocity( body ),
                scene.getWorld().GetBodyAngularVelocity( body ),
                1.0f / mass, 1.0f / inertia
            };
        }

        fluidField2 entry = *scene.getWindField();
        if( sourceOn )
        {
            const float h = entry.getCellSize();
            const float width = entry.getWidth() * h;
            const float x = entry.getOrigin().x +
                std::min( 2.5f, 0.25f * width );
            entry.addVelocity( { x, 0.0f },
                { 8.0f * dt, 4.0f * dt * std::sin( 1.7f * dt ) }, 1.7f );
        }

        const auto openMomentum = []( const fluidField2& field )
        {
            const double h = field.getCellSize();
            const double area = h * h;
            vec2 momentum{};
            for( int y = 0; y < field.getHeight(); ++y )
                for( int x = 1; x < field.getWidth(); ++x )
                {
                    if( field.getCellFluidFraction( x - 1, y ) <= 0.0f ||
                        field.getCellFluidFraction( x, y ) <= 0.0f ) continue;
                    const auto velocity = field.getUFaceVelocityForAudit( x, y );
                    if( !velocity ) throw std::runtime_error( "missing U face" );
                    momentum.x += static_cast<float>(
                        area * field.getUFaceOpenFraction( x, y ) * *velocity );
                }
            for( int y = 1; y < field.getHeight(); ++y )
                for( int x = 0; x < field.getWidth(); ++x )
                {
                    if( field.getCellFluidFraction( x, y - 1 ) <= 0.0f ||
                        field.getCellFluidFraction( x, y ) <= 0.0f ) continue;
                    const auto velocity = field.getVFaceVelocityForAudit( x, y );
                    if( !velocity ) throw std::runtime_error( "missing V face" );
                    momentum.y += static_cast<float>(
                        area * field.getVFaceOpenFraction( x, y ) * *velocity );
                }
            return momentum;
        };
        const auto wallImpulse = []( const fluidField2& field )
        {
            const int w = field.getWidth();
            const int h = field.getHeight();
            const double scale = static_cast<double>(
                field.getCellSize() ) * dt;
            double jx = 0.0, jy = 0.0;
            for( int y = 0; y < h; ++y )
            {
                jx -= field.getCellPressureForVisualization( 0, y ) * scale;
                jx += field.getCellPressureForVisualization( w - 1, y ) * scale;
            }
            for( int x = 0; x < w; ++x )
            {
                jy -= field.getCellPressureForVisualization( x, 0 ) * scale;
                jy += field.getCellPressureForVisualization( x, h - 1 ) * scale;
            }
            return std::array<double, 2>{{ jx, jy }};
        };

        fluidField2 cold = entry;
        fluidField2 primed = entry;
        if( !primed.primeCutCellHistoryForAudit( obstacles ) ||
            primed.primeCutCellHistoryForAudit( obstacles ) )
            throw std::runtime_error( "first-history prime must succeed once" );
        // Prime must not change stored MAC velocities or tracer inventory.
        // Energy getters are aperture-weighted: they legitimately differ
        // because the cold Field has no solid mask yet.
        if( std::abs( primed.getTracerMass() - cold.getTracerMass() ) > 1.0e-7 )
            throw std::runtime_error( "first-history prime changed tracer" );
        for( int y = 0; y < cold.getHeight(); ++y )
            for( int x = 1; x < cold.getWidth(); ++x )
                if( cold.getUFaceVelocityForAudit( x, y ) !=
                    primed.getUFaceVelocityForAudit( x, y ) )
                    throw std::runtime_error( "first-history prime altered U velocity" );
        for( int y = 1; y < cold.getHeight(); ++y )
            for( int x = 0; x < cold.getWidth(); ++x )
                if( cold.getVFaceVelocityForAudit( x, y ) !=
                    primed.getVFaceVelocityForAudit( x, y ) )
                    throw std::runtime_error( "first-history prime altered V velocity" );

        std::array<fluidField2*, 2> paths{{ &cold, &primed }};
        std::array<fluidCutCellPressureImpulse2, 2> impulses{};
        for( int mode = 0; mode < 2; ++mode )
        {
            fluidField2& field = *paths[mode];
            std::optional<vec2> beforePressure{};
            field.setPrePressureSnapshotObserver(
                [&]( const fluidField2& snapshot )
                {
                    if( beforePressure )
                        throw std::runtime_error( "duplicate first-history pressure entry" );
                    beforePressure = openMomentum( snapshot );
                } );
            std::array<fluidObstacleForce2, 3> forces{};
            field.step( dt, obstacles, forces, 200 );
            const fluidCutCellPressure2& pressure = field.getCutCellPressure();
            if( !beforePressure ||
                !pressure.hasAnalyticImpulseBudget ||
                !pressure.usedCoupledProjection ||
                pressure.coupledSolveFailed ||
                pressure.hasVolumeHistory != ( mode == 1 ) )
                throw std::runtime_error( "invalid first-history coupled A/B" );
            const auto independentWall = wallImpulse( field );
            const vec2 independentFluid = openMomentum( field ) - *beforePressure;
            vec2 solidFeedback{};
            double feedbackTorque = 0.0;
            for( const fluidObstacleForce2& force : forces )
            {
                solidFeedback += dt * force.force;
                feedbackTorque += static_cast<double>( dt ) * force.torque;
            }
            const auto& impulse = pressure.impulse;
            impulses[mode] = impulse;
            const double tol = 1.0e-4;
            if( Length( independentFluid - impulse.fluid ) > tol ||
                std::abs( independentWall[0] - impulse.wall.x ) > tol ||
                std::abs( independentWall[1] - impulse.wall.y ) > tol ||
                Length( solidFeedback - impulse.solid ) > tol )
                throw std::runtime_error( "first-history independent pressure ledger drift" );

            const auto summary = field.getCutCellVolumeSummary( dt );
            const auto flow = field.getFlowAudit( dt );
            const vec2 residual = impulse.residual();
            const std::array<double, 19> numbers{{
                impulse.fluid.x, impulse.fluid.y,
                impulse.solid.x, impulse.solid.y,
                impulse.wall.x, impulse.wall.y,
                residual.x, residual.y,
                independentFluid.x, independentFluid.y,
                independentWall[0], independentWall[1],
                solidFeedback.x, solidFeedback.y, feedbackTorque,
                static_cast<double>( summary.modifiedCells ),
                summary.totalPressureSourceDeltaVolume,
                pressure.compatibilityCorrection,
                flow.maxFluidFaceSpeed
            }};
            for( const double value : numbers )
                if( !std::isfinite( value ) )
                    throw std::runtime_error( "nonfinite first-history pressure comparison" );
            std::printf( "world_pressure_first_history,%d,%d,%d,%d,%d,%d",
                sourceOn ? 1 : 0, mode, pressure.hasVolumeHistory ? 1 : 0,
                field.getSweptFaceFlux().usedSweptFlux ? 1 : 0,
                pressure.usedCoupledProjection ? 1 : 0,
                pressure.coupledSolveFailed ? 1 : 0 );
            for( const double value : numbers ) std::printf( ",%.12g", value );
            std::putchar( '\n' );
        }
        // On a fixed first pose both variants must use the same endpoint
        // analytical solid coverage. Do not demand the SAME pressure or
        // residual: that is precisely the first-history hypothesis under test.
        for( int y = 0; y < cold.getHeight(); ++y )
            for( int x = 0; x < cold.getWidth(); ++x )
                if( std::abs( cold.getCellFluidFraction( x, y ) -
                    primed.getCellFluidFraction( x, y ) ) > 1.0e-6 )
                    throw std::runtime_error( "first-history endpoint geometry mismatch" );
        const vec2 difference = impulses[1].residual() - impulses[0].residual();
        std::printf( "world_pressure_first_history_delta,%d,%.12g,%.12g\n",
            sourceOn ? 1 : 0, difference.x, difference.y );
    }
}

// S2c bounded real-World pressure exchange forensic. The actual Wind sandbox
// couples Cut-cell Schur PCG to three dynamic World bodies, at World8 with
// Fluid1/2/4/8. Tracer-stage callback runs AFTER pressure; read the existing
// analytic pressure impulse ledger once per Fluid step, before the next World
// step changes the body velocity. No new pressure or force integration path.
// All cadences start from separately constructed, identical initial scenes.
void auditSmallCellWorldPressureImpulse()
{
    constexpr int frames = 8;
    std::puts( "world_pressure_exchange,fluid_splits,frame,observed_fluid_steps,"
        "coupled_steps,analytic_impulse_steps,failed_steps,"
        "pressure_fluid_jx_ns,pressure_fluid_jy_ns,"
        "pressure_solid_jx_ns,pressure_solid_jy_ns,"
        "pressure_wall_jx_ns,pressure_wall_jy_ns,"
        "pressure_exchange_residual_x_ns,pressure_exchange_residual_y_ns,"
        "world_applied_pressure_jx_ns,world_applied_pressure_jy_ns,"
        "world_applied_pressure_torque_ns_m,"
        "world_body_linear_delta_x_ns,world_body_linear_delta_y_ns,"
        "world_body_spin_delta_ns_m,"
        "world_linear_minus_pressure_x_ns,world_linear_minus_pressure_y_ns,"
        "world_spin_minus_pressure_torque_ns_m,"
        "last_face_speed_mps,tracer_mass_kg,last_pressure_status" );

    int totalAnalytic = 0;
    for( const int fluidSplits : { 1, 2, 4, 8 } )
    {
        rigidBodyDemo scene{ demoKind::windFlow };
        scene.configureWindField( 48, 28, 0.25f );
        scene.setWindCutCellProjectionEnabled( true );
        scene.setWindCutCellCoupledProjectionEnabled( true );
        scene.setWindDisjointAnalyticEnabled( true );
        scene.setWindSynchronizedCouplingEnabled( true );
        scene.setWindObstacleCouplingEnabled( true );
        scene.setWindFluidSubstepCount( fluidSplits );
        scene.setWindCouplingAuditEnabled( true );
        scene.setWindSourceStrength( 8.0f );
        scene.setWindDragCoefficient( 0.0f );
        scene.setWindSmallCellPressureMergeFraction( 0.0f );
        scene.seedWindTracer(); // Enables read-only after-pressure stage callbacks.
        const auto shapes = scene.getShapes();
        if( shapes.size() < 3 || !scene.getWindField() ||
            scene.getWindField()->getSmallCellPressureMergeFraction() != 0.0f )
            throw std::runtime_error( "invalid matched World impulse setup" );
        scene.getWorld().SetBodyLinearVelocity(
            shapes[0].bodyHandle, { 0.7f, -0.10f } );
        scene.getWorld().SetBodyAngularVelocity( shapes[1].bodyHandle, 0.5f );

        vec2 fluidImpulse{}, solidImpulse{}, wallImpulse{};
        int fluidObservations = 0, coupledObservations = 0;
        int analyticObservations = 0, failedObservations = 0;
        scene.setWindTracerStageObserverForAudit(
            [&]( const fluidField2& field, fluidTracerStage2 stage )
            {
                if( stage != fluidTracerStage2::afterSweptRemap ) return;
                ++fluidObservations;
                const fluidCutCellPressure2& pressure = field.getCutCellPressure();
                if( pressure.usedCoupledProjection ) ++coupledObservations;
                if( pressure.coupledSolveFailed ) ++failedObservations;
                if( !pressure.hasAnalyticImpulseBudget ) return;
                ++analyticObservations;
                fluidImpulse += pressure.impulse.fluid;
                solidImpulse += pressure.impulse.solid;
                wallImpulse += pressure.impulse.wall;
            } );

        for( int frame = 0; frame < frames; ++frame )
        {
            fluidImpulse = solidImpulse = wallImpulse = {};
            fluidObservations = coupledObservations = 0;
            analyticObservations = failedObservations = 0;
            std::array<vec2, 3> velocitiesBefore{};
            std::array<float, 3> angularBefore{}, masses{}, inertias{};
            for( std::size_t i = 0; i < 3; ++i )
            {
                const bodyId body = shapes[i].bodyHandle;
                masses[i] = scene.getWorld().GetBodyMass( body );
                inertias[i] = scene.getWorld().GetBodyRotationalInertia( body );
                velocitiesBefore[i] = scene.getWorld().GetBodyLinearVelocity( body );
                angularBefore[i] = scene.getWorld().GetBodyAngularVelocity( body );
                if( !( masses[i] > 0.0f && inertias[i] > 0.0f ) )
                    throw std::runtime_error( "invalid World pressure body mass" );
            }

            scene.step( dt, worldSubsteps );
            if( scene.getWindCouplingSteps() != fluidSplits ||
                fluidObservations != fluidSplits || failedObservations > 0 )
                throw std::runtime_error( "real World/Fluid pressure cadence drift" );
            totalAnalytic += analyticObservations;

            vec2 worldLinearDelta{}, appliedPressure{};
            double worldSpinDelta = 0.0, appliedTorque = 0.0;
            const auto pressureForces = scene.getWindPressureForces();
            if( pressureForces.size() < 3 )
                throw std::runtime_error( "missing applied World pressure forces" );
            for( std::size_t i = 0; i < 3; ++i )
            {
                const bodyId body = shapes[i].bodyHandle;
                worldLinearDelta += masses[i] * (
                    scene.getWorld().GetBodyLinearVelocity( body ) -
                    velocitiesBefore[i] );
                worldSpinDelta += static_cast<double>( inertias[i] ) *
                    ( scene.getWorld().GetBodyAngularVelocity( body ) -
                        angularBefore[i] );
                appliedPressure += dt * pressureForces[i].force;
                appliedTorque += dt * pressureForces[i].torque;
            }

            const vec2 exchangeResidual =
                fluidImpulse + solidImpulse + wallImpulse;
            const vec2 worldMinusPressure =
                worldLinearDelta - appliedPressure;
            const double spinMinusPressure =
                worldSpinDelta - appliedTorque;
            const fluidField2& field = *scene.getWindField();
            const auto& status = field.getCutCellPressure();
            const double faceSpeed = field.getFlowAudit( dt / fluidSplits )
                .maxFluidFaceSpeed;
            const std::array<double, 19> values{{
                fluidImpulse.x, fluidImpulse.y,
                solidImpulse.x, solidImpulse.y,
                wallImpulse.x, wallImpulse.y,
                exchangeResidual.x, exchangeResidual.y,
                appliedPressure.x, appliedPressure.y, appliedTorque,
                worldLinearDelta.x, worldLinearDelta.y, worldSpinDelta,
                worldMinusPressure.x, worldMinusPressure.y,
                spinMinusPressure, faceSpeed, field.getTracerMass()
            }};
            for( const double value : values )
                if( !std::isfinite( value ) )
                    throw std::runtime_error( "nonfinite real World pressure impulse" );

            // Only claim solver-to-World *applied-force bookkeeping* when
            // the existing analytic ledger covered EVERY Fluid substep.
            // The World delta may also include contacts, gravity, damping or
            // solver limits and therefore is a separate observed quantity.
            if( analyticObservations == fluidSplits )
            {
                const double absTol = 1.0e-4 * std::max(
                    1.0, static_cast<double>( Length( appliedPressure ) ) );
                if( Length( solidImpulse - appliedPressure ) > absTol )
                    throw std::runtime_error(
                        "Cut-cell PCG solid impulse differs from applied World pressure" );
            }
            std::printf( "world_pressure_exchange,%d,%d,%d,%d,%d,%d",
                fluidSplits, frame, fluidObservations, coupledObservations,
                analyticObservations, failedObservations );
            for( const double value : values ) std::printf( ",%.12g", value );
            std::printf( ",%d\n", static_cast<int>( status.coupledStatus ) );
        }
    }
    if( totalAnalytic == 0 )
        throw std::runtime_error( "World coupled analytic pressure impulse was never sampled" );
}

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

// Independent double-coordinate core clipping and cap cross-section reference.
// Uses the *same float-transformed endpoints* as the production Capsule, so
// the controls isolate axis and core-vertex rounding rather than rotation
// precision, pressure, or motion history. Only called in this opt-in audit.
struct capsuleAuditPoint2
{
    double x = 0.0, y = 0.0;
};

double auditDoubleCapsuleCoreArea(
    std::array<capsuleAuditPoint2, 4> polygon,
    double x0, double y0, double h )
{
    std::array<capsuleAuditPoint2, 16> current{}, next{};
    std::copy( polygon.begin(), polygon.end(), current.begin() );
    int count = 4;
    const double x1 = x0 + h, y1 = y0 + h;
    for( int side = 0; side < 4; ++side )
    {
        const auto inside = [&]( capsuleAuditPoint2 point )
        {
            if( side == 0 ) return point.x - x0;
            if( side == 1 ) return x1 - point.x;
            if( side == 2 ) return point.y - y0;
            return y1 - point.y;
        };
        int nextCount = 0;
        capsuleAuditPoint2 previous = current[count - 1];
        double before = inside( previous );
        for( int i = 0; i < count; ++i )
        {
            const capsuleAuditPoint2 point = current[i];
            const double after = inside( point );
            if( ( before >= 0.0 ) != ( after >= 0.0 ) )
            {
                const double alpha = before / ( before - after );
                next[nextCount++] = {
                    previous.x + alpha * ( point.x - previous.x ),
                    previous.y + alpha * ( point.y - previous.y )
                };
            }
            if( after >= 0.0 ) next[nextCount++] = point;
            previous = point;
            before = after;
        }
        if( nextCount < 3 ) return 0.0;
        current.swap( next );
        count = nextCount;
    }
    const capsuleAuditPoint2 origin = current[0];
    double twiceArea = 0.0;
    for( int i = 1; i + 1 < count; ++i )
        twiceArea += ( current[i].x - origin.x ) *
            ( current[i + 1].y - origin.y ) -
            ( current[i].y - origin.y ) *
            ( current[i + 1].x - origin.x );
    return 0.5 * std::abs( twiceArea );
}

double auditDoubleCapsuleCapArea(
    bool first, capsuleAuditPoint2 a, capsuleAuditPoint2 b,
    double axisX, double axisY, double radius,
    double x0, double y0, double h, int count )
{
    if( !( std::abs( axisY ) > 1.0e-12 ) || count < 1 )
        throw std::runtime_error( "invalid audit Capsule cap axis/count" );
    const capsuleAuditPoint2 center = first ? a : b;
    const double x1 = x0 + h, y1 = y0 + h;
    const double left = std::max( x0, center.x - radius );
    const double right = std::min( x1, center.x + radius );
    if( right <= left ) return 0.0;
    const double step = ( right - left ) / count;
    double lengthSum = 0.0;
    for( int i = 0; i < count; ++i )
    {
        const double x = left + ( i + 0.5 ) * step;
        const double dx = x - center.x;
        const double height = std::sqrt( std::max(
            0.0, radius * radius - dx * dx ) );
        double low = std::max( y0, center.y - height );
        double high = std::min( y1, center.y + height );
        const double capPlane = center.y - dx * axisX / axisY;
        // First cap: along-axis <= 0; last cap: >= 0.
        if( first == ( axisY > 0.0 ) ) high = std::min( high, capPlane );
        else low = std::max( low, capPlane );
        lengthSum += std::max( 0.0, high - low );
    }
    return step * lengthSum;
}

void auditCapsulePrecisionComponents(
    const fluidCapsuleCellAreaAudit2& engine,
    vec2 actualA, vec2 actualB, float radiusFloat,
    double x0, double y0, double h, double wedgeArea )
{
    const double radius = radiusFloat;
    const capsuleAuditPoint2 a{ actualA.x, actualA.y };
    const capsuleAuditPoint2 b{ actualB.x, actualB.y };
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double norm = std::hypot( dx, dy );
    if( !( norm > 0.0 ) ) throw std::runtime_error( "invalid Capsule reference axis" );
    const double unitX = dx / norm, unitY = dy / norm;
    const vec2 floatAxis{ static_cast<float>( unitX ),
        static_cast<float>( unitY ) };
    const vec2 radiusVector = radiusFloat *
        vec2{ -floatAxis.y, floatAxis.x };
    const std::array<capsuleAuditPoint2, 4> roundedCore{{
        { ( actualA - radiusVector ).x, ( actualA - radiusVector ).y },
        { ( actualB - radiusVector ).x, ( actualB - radiusVector ).y },
        { ( actualB + radiusVector ).x, ( actualB + radiusVector ).y },
        { ( actualA + radiusVector ).x, ( actualA + radiusVector ).y }
    }};
    const std::array<capsuleAuditPoint2, 4> doubleCore{{
        { a.x + radius * unitY, a.y - radius * unitX },
        { b.x + radius * unitY, b.y - radius * unitX },
        { b.x - radius * unitY, b.y + radius * unitX },
        { a.x - radius * unitY, a.y + radius * unitX }
    }};
    const double coarseCore = auditDoubleCapsuleCoreArea(
        roundedCore, x0, y0, h );
    const double preciseCore = auditDoubleCapsuleCoreArea(
        doubleCore, x0, y0, h );
    const auto capArea = [&]( bool first, bool floatAxisPolicy, int count )
    {
        return auditDoubleCapsuleCapArea( first, a, b,
            floatAxisPolicy ? static_cast<double>( floatAxis.x ) : unitX,
            floatAxisPolicy ? static_cast<double>( floatAxis.y ) : unitY,
            radius, x0, y0, h, count );
    };
    const double coarseFirst = capArea( true, true, 65536 );
    const double coarseLast = capArea( false, true, 65536 );
    const double preciseFirst = capArea( true, false, 65536 );
    const double preciseLast = capArea( false, false, 65536 );
    // Separate convergence budget for the one-dimensional integrals, not
    // an assertion that sampled integration is formally exact.
    if( std::abs( preciseFirst - capArea( true, false, 16384 ) ) > 5.0e-12 ||
        std::abs( coarseFirst - capArea( true, true, 16384 ) ) > 5.0e-12 ||
        std::abs( preciseLast - capArea( false, false, 16384 ) ) > 5.0e-12 ||
        std::abs( coarseLast - capArea( false, true, 16384 ) ) > 5.0e-12 )
        throw std::runtime_error( "Capsule cap double quadrature did not converge" );
    const double roundedFree = h * h -
        ( coarseCore + coarseFirst + coarseLast );
    const double preciseFree = h * h -
        ( preciseCore + preciseFirst + preciseLast );
    if( std::abs( engine.coreCoveredArea - coarseCore ) > 1.0e-12 ||
        std::abs( engine.firstCapCoveredArea - coarseFirst ) > 1.0e-11 ||
        std::abs( engine.lastCapCoveredArea - coarseLast ) > 1.0e-11 ||
        std::abs( engine.signedUnclampedFluidArea - roundedFree ) > 2.0e-11 ||
        !( preciseFree > 0.0 ) ||
        std::abs( preciseFree - wedgeArea ) > 5.0e-12 )
        throw std::runtime_error(
            "Capsule float/double component control disagrees with independent wedge" );
    std::puts( "capsule_precision_components,cell_x,cell_y,"
        "engine_core_m2,rounded_core_m2,double_core_m2,"
        "engine_first_cap_m2,float_axis_first_cap_m2,double_axis_first_cap_m2,"
        "engine_last_cap_m2,float_axis_last_cap_m2,double_axis_last_cap_m2,"
        "engine_signed_free_m2,rounded_signed_free_m2,"
        "double_signed_free_m2,independent_wedge_free_m2,"
        "core_float_minus_double_m2,cap_float_minus_double_m2" );
    std::printf( "capsule_precision_components,28,26,"
        "%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,"
        "%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,"
        "%.17g,%.17g\n",
        engine.coreCoveredArea, coarseCore, preciseCore,
        engine.firstCapCoveredArea, coarseFirst, preciseFirst,
        engine.lastCapCoveredArea, coarseLast, preciseLast,
        engine.signedUnclampedFluidArea, roundedFree,
        preciseFree, wedgeArea,
        coarseCore - preciseCore,
        ( coarseFirst + coarseLast ) - ( preciseFirst + preciseLast ) );
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

    if( mode != 0 )
    {
        const auto area = frozen.getAnalyticCapsuleCellAreaAudit( capsule, x, y );
        if( !area.valid || !std::isfinite( area.componentSumCoveredArea ) ||
            !std::isfinite( area.signedUnclampedFluidArea ) ||
            std::abs( area.recomputedFluidFraction - area.storedFluidFraction ) > 1.0e-7 )
            throw std::runtime_error( "Capsule analytic area decomposition not reproducible" );
        const capsuleCornerWedge2 reference = integrateCapsuleCornerWedge(
            actualA.x, actualA.y, actualB.x, actualB.y, radius, x0, y0, h );
        if( mode == 1 )
            auditCapsulePrecisionComponents( area, actualA, actualB,
                shape->radius, x0, y0, h, reference.area1024 );
        else
        {
            if( !( area.signedUnclampedFluidArea > 0.0 ) ||
                !( area.storedFluidFraction > 0.0f ) ||
                std::abs( area.signedUnclampedFluidArea - reference.area1024 ) >
                    5.0e-12 )
                throw std::runtime_error(
                    "double Capsule corner did not preserve positive exterior wedge" );
            std::printf( "capsule_double_corner_area,%.9g,%d,%d,%d,"
                "%.17g,%.17g,%.17g,%.17g\n",
                h, checkpoint, x, y, area.signedUnclampedFluidArea,
                reference.area1024, area.storedFluidFraction,
                area.signedUnclampedFluidArea - reference.area1024 );
        }
        std::puts( "capsule_corner_area_breakdown,cell_m,checkpoint_16,"
            "cell_x,cell_y,cell_area_m2,core_covered_m2,"
            "first_cap_covered_m2,last_cap_covered_m2,"
            "sum_covered_m2,clamped_covered_m2,"
            "unclamped_free_m2,stored_free_fraction,"
            "independent_free_m2,covered_minus_reference_m2" );
        std::printf( "capsule_corner_area_breakdown,%.9g,%d,%d,%d,"
            "%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,"
            "%.17g,%.12g,%.17g,%.17g\n",
            h, checkpoint, x, y, area.cellArea, area.coreCoveredArea,
            area.firstCapCoveredArea, area.lastCapCoveredArea,
            area.componentSumCoveredArea, area.clampedCoveredArea,
            area.signedUnclampedFluidArea, area.storedFluidFraction,
            reference.area1024,
            area.componentSumCoveredArea - ( area.cellArea - reference.area1024 ) );
    }
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
            if( policy == 0 && ulp == 0 && mode != 0 &&
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

// Convex Capsule: a rectangle with all four corners inside the continuous
// solid is completely covered; a corner strictly outside proves a positive
// open wedge. The witness uses only float-transformed Shape endpoints and
// independent point-to-segment signed distance (no engine cut-cell geometry).
// Near-zero distances stay uncertain rather than being forced to a class.
struct capsuleCornerTopologyWitness
{
    double minClearance = std::numeric_limits<double>::infinity();
    double maxClearance = -std::numeric_limits<double>::infinity();
    int exteriorCorners = 0;
    int classification = 0; // -1 definitely covered, +1 definitely open, 0 uncertain
};

capsuleCornerTopologyWitness classifyCapsuleCellCorners(
    vec2 first, vec2 last, double radius, vec2 origin,
    double h, int x, int y )
{
    const double ax = first.x, ay = first.y;
    const double bx = last.x, by = last.y;
    const double dx = bx - ax, dy = by - ay;
    const double squaredLength = dx * dx + dy * dy;
    if( !( squaredLength > 0.0 ) || !( radius > 0.0 ) )
        throw std::runtime_error( "invalid independent Capsule corner input" );
    capsuleCornerTopologyWitness result{};
    for( int iy = 0; iy <= 1; ++iy )
        for( int ix = 0; ix <= 1; ++ix )
        {
            const double px = static_cast<double>( origin.x ) + ( x + ix ) * h;
            const double py = static_cast<double>( origin.y ) + ( y + iy ) * h;
            const double t = std::clamp(
                ( ( px - ax ) * dx + ( py - ay ) * dy ) / squaredLength,
                0.0, 1.0 );
            const double clearance = std::hypot( px - ( ax + t * dx ),
                py - ( ay + t * dy ) ) - radius;
            result.minClearance = std::min( result.minClearance, clearance );
            result.maxClearance = std::max( result.maxClearance, clearance );
            result.exteriorCorners += clearance > 0.0;
        }
    // A tiny tolerance is only for the independent diagnostic confidence;
    // it never alters the physical cell mask or the pressure graph.
    constexpr double confidence = 1.0e-10;
    result.classification = result.maxClearance > confidence ? 1 :
        result.maxClearance < -confidence ? -1 : 0;
    return result;
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
        std::vector<float> cells, previousCells, uOpen, vOpen;
        std::vector<double> sources, snapshotDeltas, sweptDivergences;
        vec2 first{}, last{}, origin{};
        double radius = 0.0;
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
    std::puts( "capsule_geometry_precision_pair,cell_m,checkpoint_16,"
        "float_active,double_active,float_anchors,double_anchors,"
        "recovered_cells,lost_cells,float_anchor_open,double_anchor_open,"
        "float_pcg_iters,double_pcg_iters,cell_area_L1_m2,"
        "source_L1_m3,float_target_face_m3,double_target_face_m3,double_pcg_converged" );
    std::puts( "capsule_corner_topology_summary,cell_m,checkpoint_16,"
        "float_active,double_active,float_false_zero,double_false_zero,"
        "float_false_positive,double_false_positive,"
        "uncertain_corner_cells,max_false_zero_clearance_m,"
        "max_false_positive_depth_m" );
    std::puts( "capsule_corner_topology_cell,cell_m,checkpoint_16,x,y,"
        "float_fraction,double_fraction,corner_class,corner_exterior_count,"
        "minimum_corner_clearance_m,maximum_corner_clearance_m,"
        "float_previous,double_previous,float_snapshot_delta_m3,"
        "double_snapshot_delta_m3,float_pressure_source_m3,"
        "double_pressure_source_m3,float_swept_divergence_m3,"
        "double_swept_divergence_m3,float_W,float_E,float_S,float_N,"
        "double_W,double_E,double_S,double_N" );
    std::puts( "capsule_active_mask,cell_m,checkpoint_16,mode,"
        "active_cells,fnv1a64_row_major_bits" );
    std::puts( "capsule_full_coverage_summary,cell_m,checkpoint_16,"
        "double_active,guard_active,proven_covered_double,"
        "proven_covered_guard,guard_removed,guard_false_zero,"
        "guard_anchor_faces,guard_pcg_iterations,guard_pcg_converged,"
        "fraction_removed_area_m2,pressure_source_L1_m3,"
        "snapshot_L1_m3,swept_divergence_L1_m3,"
        "max_shared_face_difference,pressure_target_face_difference_m3" );
    std::puts( "capsule_full_coverage_cell,cell_m,checkpoint_16,x,y,"
        "double_fraction,guard_fraction,corner_class,"
        "min_corner_clearance_m,max_corner_clearance_m,"
        "double_previous,guard_previous,"
        "double_snapshot_delta_m3,guard_snapshot_delta_m3,"
        "double_pressure_source_m3,guard_pressure_source_m3,"
        "double_swept_divergence_m3,guard_swept_divergence_m3" );
    std::puts( "capsule_full_coverage_timing,cell_m,mode,"
        "steps,geometry_total_ms,swept_total_ms,pressure_total_ms,"
        "fluid_total_ms" );
    std::puts( "capsule_geometry_anchor_pair,cell_m,checkpoint_16,axis,x,y,"
        "sampled_open,analytic_open,sampled_fraction_a,sampled_fraction_b,"
        "analytic_fraction_a,analytic_fraction_b,"
        "sampled_source_a_m3,sampled_source_b_m3,"
        "analytic_source_a_m3,analytic_source_b_m3" );

    for( float h : { 0.25f, 0.125f } )
    {
        const int segments = static_cast<int>( std::lround( 0.25f / h ) );
        const float subDt = totalDt / fluidSteps;
        std::array<std::array<capture, 2>, 4> observations{};
        std::array<fluidStepTiming2, 4> timings{};
        for( int mode = 0; mode < 4; ++mode )
        {
            fluidField2 field{ 32 * segments, 32 * segments, h, { -4.0f, -4.0f } };
            field.setCutCellProjectionEnabled( true );
            field.setDisjointAnalyticCutCellEnabled( true );
            field.setAnalyticCapsuleCutCellEnabled( mode != 0 );
            field.setAnalyticCapsuleDoubleGeometryEnabled( mode >= 2 );
            field.setAnalyticCapsuleFullCoverageGuardEnabled( mode == 3 );
            field.setStepTimingEnabled( mode >= 2 );
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
                    frozen.isAnalyticCapsuleDoubleGeometryEnabled() != ( mode >= 2 ) ||
                    frozen.isAnalyticCapsuleFullCoverageGuardEnabled() != ( mode == 3 ) ||
                    frozen.getCutCellTransition().usedDisjointAnalyticGeometry )
                    fail( "invalid Capsule frozen geometry checkpoint" );

                fluidField2 trial = frozen;
                trial.setPrePressureSnapshotObserver( {} );
                if( !trial.setFrozenMacVelocityForAudit(
                    []( vec2 ) { return vec2{}; } ) )
                    fail( "frozen Capsule geometry MAC override refused" );
                row.width = trial.getWidth();
                row.height = trial.getHeight();
                row.origin = trial.getOrigin();
                const auto* shape = std::get_if<capsule2>( &capsule.geometry );
                if( !shape ) fail( "missing prescribed Capsule geometry" );
                row.first = TransformPoint( capsule.transform, shape->center1 );
                row.last = TransformPoint( capsule.transform, shape->center2 );
                row.radius = shape->radius;
                const int width = row.width, height = row.height;
                row.cells.resize( width * height );
                row.previousCells.resize( width * height );
                row.sources.resize( width * height );
                row.snapshotDeltas.resize( width * height );
                row.sweptDivergences.resize( width * height );
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
                        row.previousCells[cell] = budget.previousFraction;
                        row.sources[cell] = budget.pressureSourceDeltaVolume;
                        row.snapshotDeltas[cell] = budget.snapshotDeltaVolume;
                        row.sweptDivergences[cell] =
                            budget.solidSweptDivergenceVolume;
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
                // Cross-toolchain identity witness: hash the entire row-major
                // binary active mask, not just the active-cell count.
                std::uint64_t maskHash = UINT64_C( 14695981039346656037 );
                for( float fraction : row.cells )
                {
                    maskHash ^= fraction > 0.0f ? UINT64_C( 1 ) : UINT64_C( 0 );
                    maskHash *= UINT64_C( 1099511628211 );
                }
                std::printf( "capsule_active_mask,%.9g,%d,%d,%d,%016llx\n",
                    h, cp, mode, row.active,
                    static_cast<unsigned long long>( maskHash ) );
                // Recorded independently from green Windows Release and
                // Ubuntu Debug runs. Catch location changes even when the
                // active-cell *count* stays the same.
                if( mode == 3 && h == 0.125f )
                {
                    const std::uint64_t expected = cp == 11
                        ? UINT64_C( 0x8787b49d7bcf8e49 )
                        : UINT64_C( 0x8d0ee9a33f07407c );
                    if( maskHash != expected )
                        fail( "guarded Capsule active-mask parity regression" );
                }
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
                if( mode >= 2 ) timings[mode].accumulate( field.getStepTiming() );
            }
            if( mode >= 2 )
                std::printf( "capsule_full_coverage_timing,%.9g,%d,%d,"
                    "%.12g,%.12g,%.12g,%.12g\n",
                    h, mode, fluidSteps,
                    timings[mode].geometry, timings[mode].swept,
                    timings[mode].pressure, timings[mode].total );
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

            const capture& precise = observations[2][slot];
            const capture& guarded = observations[3][slot];
            if( precise.width != guarded.width ||
                precise.height != guarded.height ||
                precise.cells.size() != guarded.cells.size() ||
                precise.uOpen != guarded.uOpen ||
                precise.vOpen != guarded.vOpen )
                fail( "full-coverage guard modified shared face geometry" );
            if( precise.width != analytic.width ||
                precise.height != analytic.height ||
                precise.cells.size() != analytic.cells.size() ||
                precise.uOpen.size() != analytic.uOpen.size() ||
                precise.vOpen.size() != analytic.vOpen.size() )
                fail( "double Capsule frozen grid does not match float geometry" );
            int recoveredCells = 0, lostCells = 0;
            double areaDifferenceL1 = 0.0, sourceDifferenceL1 = 0.0;
            for( std::size_t i = 0; i < analytic.cells.size(); ++i )
            {
                recoveredCells += analytic.cells[i] == 0.0f &&
                    precise.cells[i] > 0.0f;
                lostCells += analytic.cells[i] > 0.0f &&
                    precise.cells[i] == 0.0f;
                areaDifferenceL1 += std::abs(
                    static_cast<double>( analytic.cells[i] ) - precise.cells[i] ) *
                    h * h;
                sourceDifferenceL1 += std::abs(
                    analytic.sources[i] - precise.sources[i] );
            }
            if( analytic.first.x != precise.first.x ||
                analytic.first.y != precise.first.y ||
                analytic.last.x != precise.last.x ||
                analytic.last.y != precise.last.y ||
                analytic.radius != precise.radius ||
                analytic.origin.x != precise.origin.x ||
                analytic.origin.y != precise.origin.y ||
                precise.first.x != guarded.first.x ||
                precise.first.y != guarded.first.y ||
                precise.last.x != guarded.last.x ||
                precise.last.y != guarded.last.y ||
                precise.radius != guarded.radius ||
                precise.origin.x != guarded.origin.x ||
                precise.origin.y != guarded.origin.y )
                fail( "different actual Capsule endpoints in geometry precision A/B" );
            int floatFalseZero = 0, doubleFalseZero = 0;
            int floatFalsePositive = 0, doubleFalsePositive = 0;
            int uncertain = 0, printed = 0;
            int guardedFalseZero = 0, guardedFalsePositive = 0, guardRemoved = 0;
            double guardRemovedArea = 0.0, guardSourceL1 = 0.0;
            double guardSnapshotL1 = 0.0, guardSweptL1 = 0.0;
            double maxFalseZero = 0.0, maxFalsePositiveDepth = 0.0;
            const int width = analytic.width, height = analytic.height;
            for( int y = 0; y < height; ++y )
                for( int x = 0; x < width; ++x )
                {
                    const int i = y * width + x;
                    const auto witness = classifyCapsuleCellCorners(
                        precise.first, precise.last, precise.radius,
                        precise.origin, h, x, y );
                    const bool floatActive = analytic.cells[i] > 0.0f;
                    const bool doubleActive = precise.cells[i] > 0.0f;
                    const bool guardActive = guarded.cells[i] > 0.0f;
                    guardedFalseZero += !guardActive && witness.classification == 1;
                    guardedFalsePositive += guardActive && witness.classification == -1;
                    if( doubleActive && !guardActive )
                    {
                        if( witness.classification != -1 )
                            fail( "full-coverage guard deleted an unproven solid cell" );
                        const auto closedFaces = [&]( const capture& row )
                        {
                            return row.uOpen[y * ( width + 1 ) + x] == 0.0f &&
                                row.uOpen[y * ( width + 1 ) + x + 1] == 0.0f &&
                                row.vOpen[y * width + x] == 0.0f &&
                                row.vOpen[( y + 1 ) * width + x] == 0.0f;
                        };
                        if( !closedFaces( precise ) || !closedFaces( guarded ) )
                            fail( "full-coverage guard removed open MAC-face cell" );
                        ++guardRemoved;
                        guardRemovedArea += static_cast<double>( precise.cells[i] ) * h * h;
                        std::printf( "capsule_full_coverage_cell,%.9g,%d,%d,%d,"
                            "%.12g,%.12g,%d,%.12g,%.12g,"
                            "%.12g,%.12g,%.12g,%.12g,"
                            "%.12g,%.12g,%.12g,%.12g\n",
                            h, cp, x, y, precise.cells[i], guarded.cells[i],
                            witness.classification, witness.minClearance,
                            witness.maxClearance,
                            precise.previousCells[i], guarded.previousCells[i],
                            precise.snapshotDeltas[i], guarded.snapshotDeltas[i],
                            precise.sources[i], guarded.sources[i],
                            precise.sweptDivergences[i], guarded.sweptDivergences[i] );
                    }
                    else if( precise.cells[i] != guarded.cells[i] )
                        fail( "full-coverage guard modified unguarded cell fraction" );
                    guardSourceL1 += std::abs( precise.sources[i] - guarded.sources[i] );
                    guardSnapshotL1 += std::abs(
                        precise.snapshotDeltas[i] - guarded.snapshotDeltas[i] );
                    guardSweptL1 += std::abs(
                        precise.sweptDivergences[i] - guarded.sweptDivergences[i] );
                    const bool floatFalseZeroHere =
                        !floatActive && witness.classification == 1;
                    const bool doubleFalseZeroHere =
                        !doubleActive && witness.classification == 1;
                    const bool floatFalsePositiveHere =
                        floatActive && witness.classification == -1;
                    const bool doubleFalsePositiveHere =
                        doubleActive && witness.classification == -1;
                    floatFalseZero += floatFalseZeroHere;
                    doubleFalseZero += doubleFalseZeroHere;
                    floatFalsePositive += floatFalsePositiveHere;
                    doubleFalsePositive += doubleFalsePositiveHere;
                    uncertain += witness.classification == 0;
                    if( floatFalseZeroHere || doubleFalseZeroHere )
                        maxFalseZero = std::max( maxFalseZero,
                            witness.maxClearance );
                    if( floatFalsePositiveHere || doubleFalsePositiveHere )
                        maxFalsePositiveDepth = std::max( maxFalsePositiveDepth,
                            -witness.maxClearance );
                    const bool transition = floatActive != doubleActive;
                    const bool anomaly = floatFalseZeroHere ||
                        doubleFalseZeroHere || floatFalsePositiveHere ||
                        doubleFalsePositiveHere;
                    if( !transition && !anomaly ) continue;
                    if( ++printed > 160 )
                        fail( "unexpectedly many Capsule precision topology cells" );
                    const auto printFaces = [&]( const capture& row )
                    {
                        return std::array<float, 4>{
                            row.uOpen[y * ( width + 1 ) + x],
                            row.uOpen[y * ( width + 1 ) + x + 1],
                            row.vOpen[y * width + x],
                            row.vOpen[( y + 1 ) * width + x]
                        };
                    };
                    const auto fFaces = printFaces( analytic );
                    const auto dFaces = printFaces( precise );
                    std::printf( "capsule_corner_topology_cell,%.9g,%d,%d,%d,"
                        "%.12g,%.12g,%d,%d,%.12g,%.12g,"
                        "%.12g,%.12g,%.12g,%.12g,"
                        "%.12g,%.12g,%.12g,%.12g,"
                        "%.9g,%.9g,%.9g,%.9g,"
                        "%.9g,%.9g,%.9g,%.9g\n",
                        h, cp, x, y, analytic.cells[i], precise.cells[i],
                        witness.classification, witness.exteriorCorners,
                        witness.minClearance, witness.maxClearance,
                        analytic.previousCells[i], precise.previousCells[i],
                        analytic.snapshotDeltas[i], precise.snapshotDeltas[i],
                        analytic.sources[i], precise.sources[i],
                        analytic.sweptDivergences[i],
                        precise.sweptDivergences[i],
                        fFaces[0], fFaces[1], fFaces[2], fFaces[3],
                        dFaces[0], dFaces[1], dFaces[2], dFaces[3] );
                }
            std::printf( "capsule_corner_topology_summary,%.9g,%d,"
                "%d,%d,%d,%d,%d,%d,%d,%.12g,%.12g\n",
                h, cp, analytic.active, precise.active,
                floatFalseZero, doubleFalseZero,
                floatFalsePositive, doubleFalsePositive, uncertain,
                maxFalseZero, maxFalsePositiveDepth );
            if( !std::isfinite( maxFalseZero ) ||
                !std::isfinite( maxFalsePositiveDepth ) )
                fail( "nonfinite independent Capsule topology witness" );
            if( guardedFalseZero != 0 || guardedFalsePositive != 0 ||
                guarded.anchors != precise.anchors ||
                !guarded.pressure.converged ||
                guarded.active != precise.active - guardRemoved )
                fail( "full-coverage guarded Capsule geometry/pressure regression" );
            std::printf( "capsule_full_coverage_summary,%.9g,%d,"
                "%d,%d,%d,%d,%d,%d,%d,%d,%d,"
                "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n",
                h, cp, precise.active, guarded.active,
                doubleFalsePositive, guardedFalsePositive,
                guardRemoved, guardedFalseZero, guarded.anchors,
                guarded.pressure.iterations, guarded.pressure.converged ? 1 : 0,
                guardRemovedArea, guardSourceL1, guardSnapshotL1,
                guardSweptL1, 0.0,
                guarded.pressure.targetFaceVolume -
                    precise.pressure.targetFaceVolume );
            // A strictly exterior corner proves positive open area for a
            // convex Capsule. This assertion is checked on both toolchains;
            // false-positive fully covered cells are still informational,
            // not silently suppressed in the experiment.
            if( doubleFalseZero != 0 )
                fail( "double Capsule lost an independently proven open cell" );
            if( h == 0.125f && cp == 12 &&
                ( analytic.anchors != 2 || precise.anchors != 0 ||
                  !precise.pressure.converged ) )
                fail( "double Capsule pressure-anchor recovery regression" );
            // This is a separately evolved geometry policy comparison,
            // never a same-frozen-pressure-only A/B.
            std::printf( "capsule_geometry_precision_pair,%.9g,%d,"
                "%d,%d,%d,%d,%d,%d,%.12g,%.12g,"
                "%d,%d,%.12g,%.12g,%.12g,%.12g,%d\n",
                h, cp, analytic.active, precise.active,
                analytic.anchors, precise.anchors,
                recoveredCells, lostCells,
                analytic.anchorOpening, precise.anchorOpening,
                analytic.pressure.iterations, precise.pressure.iterations,
                areaDifferenceL1, sourceDifferenceL1,
                analytic.pressure.targetFaceVolume,
                precise.pressure.targetFaceVolume,
                precise.pressure.converged ? 1 : 0 );
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

// Compare the SAME step-95 entry field across genuinely integrated smaller
// Fluid substeps. Each substep solves its own Cut-cell pressure and tracer
// transport: this is a refinement reference, NOT an exact continuous flux.
struct capsuleStep96SplitResult
{
    double endpointMacVolume = 0.0;
    double prePressureMacVolume = 0.0;
    double linearMacProxyVolume = 0.0;
    double openExposureSeconds = 0.0;
    double solidSweptFaceVolume = 0.0;
    double leftPressureSourceVolume = 0.0;
    double rightPressureSourceVolume = 0.0;
    double leftSnapshotDeltaVolume = 0.0;
    double leftPressureResidualVolume = 0.0;
    std::array<double, 4> signedAppliedPairMass{}; // swept/fallback/MAC/connected, +left->right
    // [transfer kind][W,E,S,N,nonlocal][outward/inward] — applied mass, kg.
    std::array<std::array<std::array<double, 2>, 5>, 4> donorTransfers{};
    std::array<std::array<int, 5>, 4> donorTransferCounts{};
    std::array<double, 4> donorPreMacOutwardVolume{};
    std::array<double, 4> donorPostMacOutwardVolume{};
    std::array<double, 4> donorStartMacOutwardVolume{};
    std::array<double, 4> donorTrapezoidMacOutwardVolume{};
    std::array<double, 4> donorOpenExposureSeconds{};
    std::array<double, 4> donorSolidSweptOutwardVolume{};
    // Independent analytic-geometry aperture quadrature at two fixed orders;
    // all velocity histories below are expressly HYPOTHETICAL interpolants.
    std::array<double, 4> donorAnalyticOpenExposure64{};
    std::array<double, 4> donorAnalyticOpenExposure256{};
    std::array<double, 4> donorAnalyticLinearFlux64{};
    std::array<double, 4> donorAnalyticLinearFlux256{};
    std::array<double, 4> donorAnalyticEarlyFlux256{};
    std::array<double, 4> donorAnalyticLateFlux256{};
    std::array<double, 4> donorInternalLinearFlux{};
    double donorAnalyticSnapshotDeltaVolume = 0.0;
    double donorGeometricSnapshotDeltaVolume = 0.0;
    double donorPressureSourceVolume = 0.0;
    double donorRequiredPressureCorrectionVolume = 0.0;
    double donorAppliedPressureCorrectionVolume = 0.0;
    double donorMaxAbsAnalyticFractionGap = 0.0;
    double donorMaxAbsLocalGclDefectVolume = 0.0;
    double donorMaxAbsLocalPressureResidualVolume = 0.0;
    double donorMassBefore = 0.0;
    double donorMassAfterLastSwept = 0.0;
    double donorExpectedFinalMass = 0.0;
    double maxSweptStageMassDefect = 0.0;
    double maxMacStageMassDefect = 0.0;
    double maxStepMassDefect = 0.0;
    double massBefore = 0.0;
    double massAfter = 0.0;
    double leftMassAfter = 0.0;
    double leftVolumeAfter = 0.0;
    double maxAbsRawFaceVelocity = 0.0;
    int prePressureObservations = 0;
    int sweptStages = 0;
    int macStages = 0;
};

capsuleStep96SplitResult auditCapsuleStep96SplitFlux(
    const fluidField2& stepEntry, fluidShapeObstacle2 capsule,
    float startAngle, float endAngle, int totalSteps, float duration,
    int splitCount, bool controlledDonorTracer )
{
    if( splitCount < 1 ) throw std::runtime_error( "invalid Capsule split count" );
    constexpr int faceX = 29, faceY = 26, leftX = faceX - 1;
    const int left = faceY * stepEntry.getWidth() + leftX;
    const int right = left + 1;
    const float h = stepEntry.getCellSize();
    const float dt = duration / splitCount;
    fluidField2 field = stepEntry;
    field.setTracerStageObserver( {} );
    field.setTracerTransferObserver( {} );
    field.setPrePressureSnapshotObserver( {} );
    field.setSweptApertureAuditEnabled( true );
    if( field.getSmallCellPressureMergeFraction() != 0.0f ||
        field.isConnectedResidualTracerRemapEnabled() ||
        field.isCutCellCoupledProjectionEnabled() )
        throw std::runtime_error( "Capsule split reference altered default solver" );
    if( controlledDonorTracer )
    {
        // Passive dye ONLY on copied fields. 1 kg/m^3 in the shrinking
        // cell at step 95, not an extra post-step volume or mass remap.
        if( field.getCellFluidFraction( leftX, faceY ) <= 0.0f )
            throw std::runtime_error( "Capsule step-95 controlled donor is not fluid" );
        field.setTracerConcentration( leftX, faceY, 1.0f );
    }
    const auto mass = []( const fluidField2& state )
    {
        double sum = 0.0;
        for( int y = 0; y < state.getHeight(); ++y )
            for( int x = 0; x < state.getWidth(); ++x )
                sum += state.getTracerCellMass( x, y );
        return sum;
    };
    capsuleStep96SplitResult result{};
    result.massBefore = mass( field );
    result.donorMassBefore = field.getTracerCellMass( leftX, faceY );
    // GCL reference is an independent analytic Capsule area reevaluation at
    // the two actual poses, not a rewritten/corrected solver cell fraction.
    fluidShapeObstacle2 previousCapsule = capsule;
    previousCapsule.transform.rotation = rot2::FromRadians(
        startAngle + ( 95.0f / totalSteps ) * ( endAngle - startAngle ) );
    auto previousAnalytic = field.getAnalyticCapsuleCellAreaAudit(
        previousCapsule, leftX, faceY );
    if( !previousAnalytic.valid )
        throw std::runtime_error( "missing Capsule starting analytic area witness" );
    // Canonical source-cell outward rate: u*open or v*open, m/s.
    // The start velocity is from the genuinely evolved previous Field step,
    // NOT the snapshot after advection/before the current pressure projection.
    const auto signedFaceVelocity = [&]( const fluidField2& state, int direction )
    {
        const bool isU = direction < 2;
        const int fx = direction == 1 ? faceX : leftX;
        const int fy = direction == 3 ? faceY + 1 : faceY;
        const auto velocity = isU
            ? state.getUFaceVelocityForAudit( fx, fy )
            : state.getVFaceVelocityForAudit( fx, fy );
        if( !velocity || !std::isfinite( *velocity ) )
            throw std::runtime_error( "invalid Capsule MAC face velocity" );
        return static_cast<double>( direction == 0 || direction == 2 ? -1 : 1 )
            * *velocity;
    };
    const auto faceOpen = [&]( const fluidField2& state, int direction )
    {
        const bool isU = direction < 2;
        const int fx = direction == 1 ? faceX : leftX;
        const int fy = direction == 3 ? faceY + 1 : faceY;
        const float open = isU ? state.getUFaceOpenFraction( fx, fy )
                               : state.getVFaceOpenFraction( fx, fy );
        if( !std::isfinite( open ) || open < 0.0f || open > 1.0f )
            throw std::runtime_error( "invalid Capsule MAC face opening" );
        return static_cast<double>( open );
    };
    const auto faceRate = [&]( const fluidField2& state, int direction )
    {
        return signedFaceVelocity( state, direction ) * faceOpen( state, direction );
    };
    // This ledger observes transfers, not tracer creation: compare each
    // completed Swept and MAC stage against the exact applied per-cell events.
    double segmentStartMass = result.donorMassBefore;
    double segmentSweptMass = segmentStartMass;
    std::array<double, 4> segmentNetOutwardByKind{};
    field.setTracerTransferObserver( [&]( const fluidTracerTransfer2& transfer )
    {
        if( !std::isfinite( transfer.mass ) || transfer.mass < 0.0 )
            throw std::runtime_error( "nonfinite actual Capsule tracer transfer" );
        const int kind = static_cast<int>( transfer.kind );
        if( kind < 0 || kind >= static_cast<int>( result.signedAppliedPairMass.size() ) )
            throw std::runtime_error( "unknown Capsule tracer transfer kind" );
        if( transfer.sourceCell == left && transfer.destinationCell == right )
            result.signedAppliedPairMass[kind] += transfer.mass;
        else if( transfer.sourceCell == right && transfer.destinationCell == left )
            result.signedAppliedPairMass[kind] -= transfer.mass;

        const bool outgoing = transfer.sourceCell == left;
        const bool incoming = transfer.destinationCell == left;
        if( !outgoing && !incoming ) return;
        if( outgoing && incoming )
            throw std::runtime_error( "self-transfer in Capsule donor ledger" );
        const int other = outgoing ? transfer.destinationCell : transfer.sourceCell;
        const int width = field.getWidth();
        // W/E/S/N for the donor regardless of transfer direction;
        // X records fallback across nonadjacent (possibly covered) cells.
        const int direction = other == left - 1 ? 0 :
            other == left + 1 ? 1 :
            other == left - width ? 2 :
            other == left + width ? 3 : 4;
        if( other < 0 || other >= width * field.getHeight() ||
            ( direction == 4 && transfer.kind != fluidTracerTransferKind2::fallback ) )
            throw std::runtime_error( "non-neighbor swept/MAC transfer in donor ledger" );
        result.donorTransfers[kind][direction][outgoing ? 0 : 1] += transfer.mass;
        ++result.donorTransferCounts[kind][direction];
        segmentNetOutwardByKind[kind] += outgoing ? transfer.mass : -transfer.mass;
    } );
    field.setTracerStageObserver( [&]( const fluidField2& stageField, fluidTracerStage2 stage )
    {
        const double actual = stageField.getTracerCellMass( leftX, faceY );
        if( !std::isfinite( actual ) || actual < -1.0e-12 )
            throw std::runtime_error( "negative/nonfinite Capsule stage donor mass" );
        if( stage == fluidTracerStage2::afterSweptRemap )
        {
            // Swept, fallback and optional connectedResidual run BEFORE MAC.
            const double expected = segmentStartMass -
                segmentNetOutwardByKind[0] - segmentNetOutwardByKind[1] -
                segmentNetOutwardByKind[3];
            result.maxSweptStageMassDefect = std::max(
                result.maxSweptStageMassDefect, std::abs( actual - expected ) );
            result.donorMassAfterLastSwept = actual;
            segmentSweptMass = actual;
            ++result.sweptStages;
        }
        else if( stage == fluidTracerStage2::afterMacAdvection )
        {
            const double expected = segmentSweptMass - segmentNetOutwardByKind[2];
            result.maxMacStageMassDefect = std::max(
                result.maxMacStageMassDefect, std::abs( actual - expected ) );
            ++result.macStages;
        }
    } );
    // Capture actual evolved MAC states at the solver's pre-projection
    // boundary. These are stage observations, NOT a time-resolved u(t).
    std::array<double, 4> prePressureVelocities{};
    std::array<double, 4> prePressureOpenings{};
    double prePressureMacNetThisStep = 0.0;
    field.setPrePressureSnapshotObserver( [&]( const fluidField2& frozen )
    {
        const auto budget = frozen.getCutCellVolumeBudget( leftX, faceY, dt );
        if( !budget.valid || !budget.hasSweptHistory )
            throw std::runtime_error( "missing split pre-pressure face state" );
        result.prePressureMacVolume += budget.macOutwardVolume[1];
        prePressureMacNetThisStep = budget.macNetOutwardVolume;
        for( int direction = 0; direction < 4; ++direction )
        {
            prePressureVelocities[direction] =
                signedFaceVelocity( frozen, direction );
            prePressureOpenings[direction] = faceOpen( frozen, direction );
            const double observedVolume = static_cast<double>( h ) * dt *
                prePressureVelocities[direction] * prePressureOpenings[direction];
            if( std::abs( observedVolume -
                    budget.macOutwardVolume[direction] ) > 1.0e-10 )
                throw std::runtime_error(
                    "Capsule pre-pressure MAC face budget mismatch" );
            result.donorPreMacOutwardVolume[direction] +=
                budget.macOutwardVolume[direction];
        }
        ++result.prePressureObservations;
    } );

    for( int segment = 0; segment < splitCount; ++segment )
    {
        // alpha is measured along the SAME original step-95->96 rotation,
        // using each actual smaller solver step rather than interpolated u(t).
        segmentStartMass = field.getTracerCellMass( leftX, faceY );
        segmentNetOutwardByKind = {};
        std::array<double, 4> startRates{};
        std::array<double, 4> startVelocities{};
        std::array<double, 4> startOpenings{};
        for( int direction = 0; direction < 4; ++direction )
        {
            startRates[direction] = faceRate( field, direction );
            startVelocities[direction] = signedFaceVelocity( field, direction );
            startOpenings[direction] = faceOpen( field, direction );
        }
        const double previousStoredFraction =
            field.getCellFluidFraction( leftX, faceY );
        const float alpha = ( 95.0f + static_cast<float>( segment + 1 ) /
            splitCount ) / totalSteps;
        capsule.transform.rotation = rot2::FromRadians(
            startAngle + alpha * ( endAngle - startAngle ) );
        std::array<fluidObstacleForce2, 1> feedback{};
        field.step( dt, std::span<const fluidShapeObstacle2>( &capsule, 1 ),
            feedback, 120 );
        const auto leftCell = field.getCutCellVolumeBudget( leftX, faceY, dt );
        const auto rightCell = field.getCutCellVolumeBudget( faceX, faceY, dt );
        const auto faceVelocity = field.getUFaceVelocityForAudit( faceX, faceY );
        if( !leftCell.valid || !rightCell.valid ||
            !leftCell.hasVolumeHistory || !leftCell.hasSweptHistory ||
            !field.hasSweptApertureAudit() || !faceVelocity ||
            !std::isfinite( *faceVelocity ) ||
            !std::isfinite( leftCell.macOutwardVolume[1] ) )
            throw std::runtime_error( "invalid integrated Capsule split face state" );

        result.endpointMacVolume += leftCell.macOutwardVolume[1];
        result.linearMacProxyVolume += field.getSweptULinearVelocityProxyVolume(
            faceX, faceY, dt );
        result.openExposureSeconds +=
            field.getSweptUTimeMeanOpenFraction( faceX, faceY ) * dt;
        result.solidSweptFaceVolume += leftCell.solidSweptOutwardVolume[1];
        result.leftPressureSourceVolume += leftCell.pressureSourceDeltaVolume;
        result.rightPressureSourceVolume += rightCell.pressureSourceDeltaVolume;
        result.leftSnapshotDeltaVolume += leftCell.snapshotDeltaVolume;
        result.leftPressureResidualVolume += leftCell.pressureResidualVolume;
        const auto nextAnalytic = field.getAnalyticCapsuleCellAreaAudit(
            capsule, leftX, faceY );
        if( !nextAnalytic.valid ||
            !std::isfinite( nextAnalytic.recomputedFluidFraction ) )
            throw std::runtime_error( "missing Capsule ending analytic area witness" );
        const double cellArea = static_cast<double>( h ) * h;
        const double analyticDelta = cellArea *
            ( nextAnalytic.recomputedFluidFraction -
                previousAnalytic.recomputedFluidFraction );
        const double snapshotDelta = cellArea *
            ( static_cast<double>( leftCell.currentFraction ) -
                previousStoredFraction );
        result.donorAnalyticSnapshotDeltaVolume += analyticDelta;
        result.donorGeometricSnapshotDeltaVolume += snapshotDelta;
        result.donorPressureSourceVolume += leftCell.pressureSourceDeltaVolume;
        result.donorMaxAbsAnalyticFractionGap = std::max({
            result.donorMaxAbsAnalyticFractionGap,
            std::abs( previousAnalytic.recomputedFluidFraction -
                previousStoredFraction ),
            std::abs( nextAnalytic.recomputedFluidFraction -
                static_cast<double>( leftCell.currentFraction ) )
        } );
        result.donorMaxAbsLocalGclDefectVolume = std::max(
            result.donorMaxAbsLocalGclDefectVolume,
            std::abs( snapshotDelta - leftCell.solidSweptDivergenceVolume ) );
        result.donorMaxAbsLocalPressureResidualVolume = std::max(
            result.donorMaxAbsLocalPressureResidualVolume,
            std::abs( leftCell.pressureResidualVolume ) );
        if( std::abs( snapshotDelta - leftCell.snapshotDeltaVolume ) > 1.0e-12 ||
            result.donorMaxAbsAnalyticFractionGap > 1.0e-4 )
            throw std::runtime_error( "Capsule independent geometry witness mismatch" );
        previousAnalytic = nextAnalytic;
        if( result.prePressureObservations != segment + 1 )
            throw std::runtime_error( "Capsule pressure-stage observer count mismatch" );
        // A local discrete pressure correction is the face-volume change
        // from the actual pre-projection state to the actual post-projection
        // state. It must cover the pre-state flux plus the EXISTING solver
        // source, up to the measured pressure residual. This is an identity
        // of the accepted operator, NOT the true time integral of A(t)u(t).
        const double preNet = prePressureMacNetThisStep;
        const double postNet = leftCell.macNetOutwardVolume;
        const double source = leftCell.pressureSourceDeltaVolume;
        const double requiredCorrection = -( preNet + source );
        const double appliedCorrection = postNet - preNet;
        const double closure = appliedCorrection - requiredCorrection;
        const double gclDefect = leftCell.snapshotDeltaVolume -
            leftCell.solidSweptDivergenceVolume;
        const double snapshotResidual = postNet + leftCell.snapshotDeltaVolume;
        const double sourceMinusSnapshot = source - leftCell.snapshotDeltaVolume;
        if( !std::isfinite( preNet ) || !std::isfinite( postNet ) ||
            !std::isfinite( requiredCorrection ) ||
            std::abs( closure - leftCell.pressureResidualVolume ) > 1.0e-10 ||
            std::abs( snapshotResidual -
                ( leftCell.pressureResidualVolume - sourceMinusSnapshot ) ) > 1.0e-10 )
            throw std::runtime_error( "Capsule per-substep pressure volume ledger failed" );
        result.donorRequiredPressureCorrectionVolume += requiredCorrection;
        result.donorAppliedPressureCorrectionVolume += appliedCorrection;
        std::printf( "capsule_step96_pressure_step,%d,%d,%d,"
            "%.12g,%.12g,%.12g,%.12g,%.12g,"
            "%.12g,%.12g,%.12g,%.12g,%.12g,"
            "%.12g,%.12g,%.12g\n",
            controlledDonorTracer ? 1 : 0, splitCount, segment,
            leftCell.previousFraction, leftCell.currentFraction,
            leftCell.snapshotDeltaVolume, leftCell.solidSweptDivergenceVolume,
            source, preNet, postNet, requiredCorrection, appliedCorrection,
            leftCell.pressureResidualVolume, gclDefect, sourceMinusSnapshot,
            snapshotResidual );
        for( int direction = 0; direction < 4; ++direction )
        {
            const double endRate = faceRate( field, direction );
            const double endVelocity = signedFaceVelocity( field, direction );
            const double endOpen = faceOpen( field, direction );
            const double preRate = prePressureVelocities[direction] *
                prePressureOpenings[direction];
            const double preVolume = static_cast<double>( h ) * dt * preRate;
            const double endVolume = leftCell.macOutwardVolume[direction];
            if( std::abs( endOpen - prePressureOpenings[direction] ) > 1.0e-6 ||
                std::abs( static_cast<double>( h ) * dt * endRate -
                    endVolume ) > 1.0e-10 )
                throw std::runtime_error(
                    "Capsule pre/post pressure face geometry mismatch" );
            // Pre and post are actual solver snapshots, not intermediate
            // samples of a continuous time integral. Preserve outward signs.
            std::printf( "capsule_step96_stage_face,%d,%d,%d,%c,"
                "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
                "%.12g,%.12g,%.12g,%.12g\n",
                controlledDonorTracer ? 1 : 0, splitCount, segment,
                std::array<char, 4>{{ 'W', 'E', 'S', 'N' }}[direction],
                startOpenings[direction], prePressureOpenings[direction],
                endOpen, startVelocities[direction],
                prePressureVelocities[direction], endVelocity,
                static_cast<double>( h ) * dt * startRates[direction],
                preVolume, endVolume, endVolume - preVolume );
            // Independent analytic Capsule face opening at fixed time midpoint
            // grids. This recomputes geometry with the public read-only
            // analytic face query; no pressure or tracer state is changed.
            // Velocity interpolation is NOT observed inside this substep:
            // alpha, alpha^2 and (2alpha-alpha^2) are three endpoint-matched
            // hypotheses which expose physical non-identifiability.
            const bool isU = direction < 2;
            const int fx = direction == 1 ? faceX : leftX;
            const int fy = direction == 3 ? faceY + 1 : faceY;
            const auto integrate = [&]( int samples,
                double& exposure, double& linearFlux,
                double* earlyFlux, double* lateFlux )
            {
                for( int sample = 0; sample < samples; ++sample )
                {
                    const double alpha = ( sample + 0.5 ) / samples;
                    const double globalStep = 95.0 +
                        ( segment + alpha ) / splitCount;
                    fluidShapeObstacle2 intermediateCapsule = capsule;
                    intermediateCapsule.transform.rotation = rot2::FromRadians(
                        startAngle + static_cast<float>( globalStep / totalSteps ) *
                            ( endAngle - startAngle ) );
                    const auto opening =
                        field.getAnalyticCapsuleFaceOpenFractionForAudit(
                            intermediateCapsule, isU, fx, fy );
                    if( !opening || !std::isfinite( *opening ) ||
                        *opening < 0.0 || *opening > 1.0 )
                        throw std::runtime_error(
                            "invalid independently sampled Capsule face opening" );
                    const double weight = static_cast<double>( dt ) / samples;
                    const double velocityChange = endVelocity -
                        startVelocities[direction];
                    exposure += weight * *opening;
                    linearFlux += static_cast<double>( h ) * weight * *opening *
                        ( startVelocities[direction] + alpha * velocityChange );
                    if( earlyFlux )
                        *earlyFlux += static_cast<double>( h ) * weight * *opening *
                            ( startVelocities[direction] +
                                ( 2.0 * alpha - alpha * alpha ) * velocityChange );
                    if( lateFlux )
                        *lateFlux += static_cast<double>( h ) * weight * *opening *
                            ( startVelocities[direction] +
                                alpha * alpha * velocityChange );
                }
            };
            integrate( 64, result.donorAnalyticOpenExposure64[direction],
                result.donorAnalyticLinearFlux64[direction],
                nullptr, nullptr );
            integrate( 256, result.donorAnalyticOpenExposure256[direction],
                result.donorAnalyticLinearFlux256[direction],
                &result.donorAnalyticEarlyFlux256[direction],
                &result.donorAnalyticLateFlux256[direction] );
            const double signedInternalProxy = isU
                ? field.getSweptULinearVelocityProxyVolume( fx, fy, dt )
                : field.getSweptVLinearVelocityProxyVolume( fx, fy, dt );
            result.donorInternalLinearFlux[direction] +=
                ( direction == 0 || direction == 2 ? -1.0 : 1.0 ) *
                    signedInternalProxy;
            result.donorStartMacOutwardVolume[direction] +=
                static_cast<double>( h ) * dt * startRates[direction];
            result.donorTrapezoidMacOutwardVolume[direction] +=
                static_cast<double>( h ) * dt * 0.5 *
                ( startRates[direction] + endRate );
            result.donorOpenExposureSeconds[direction] +=
                ( isU ? field.getSweptUTimeMeanOpenFraction( fx, fy )
                      : field.getSweptVTimeMeanOpenFraction( fx, fy ) ) * dt;
            result.donorPostMacOutwardVolume[direction] +=
                leftCell.macOutwardVolume[direction];
            result.donorSolidSweptOutwardVolume[direction] +=
                leftCell.solidSweptOutwardVolume[direction];
        }
        const double localPressureIdentity =
            leftCell.macNetOutwardVolume + leftCell.pressureSourceDeltaVolume;
        if( std::abs( leftCell.pressureResidualVolume -
                localPressureIdentity ) > 1.0e-12 )
            throw std::runtime_error( "Capsule local pressure ledger identity failed" );
        result.maxAbsRawFaceVelocity = std::max(
            result.maxAbsRawFaceVelocity, static_cast<double>( std::abs( *faceVelocity ) ) );
        result.leftVolumeAfter = static_cast<double>(
            leftCell.currentFraction ) * h * h;
        result.leftMassAfter = field.getTracerCellMass( leftX, faceY );
        const double appliedNetOutward = std::accumulate(
            segmentNetOutwardByKind.begin(), segmentNetOutwardByKind.end(), 0.0 );
        const double expectedMass = segmentStartMass - appliedNetOutward;
        result.maxStepMassDefect = std::max( result.maxStepMassDefect,
            std::abs( result.leftMassAfter - expectedMass ) );
        if( result.sweptStages != segment + 1 ||
            result.macStages != segment + 1 ||
            result.leftMassAfter < -1.0e-12 ||
            !std::isfinite( feedback[0].force.x ) ||
            !std::isfinite( feedback[0].force.y ) ||
            !std::isfinite( feedback[0].torque ) )
            throw std::runtime_error( "nonfinite or negative Capsule split state" );
    }
    result.massAfter = mass( field );
    const auto sumFour = []( const std::array<double, 4>& values )
    {
        return std::accumulate( values.begin(), values.end(), 0.0 );
    };
    const double sweptGeometryDelta =
        sumFour( result.donorSolidSweptOutwardVolume );
    const double startMacNet = sumFour( result.donorStartMacOutwardVolume );
    const double preMacNet = sumFour( result.donorPreMacOutwardVolume );
    const double postMacNet = sumFour( result.donorPostMacOutwardVolume );
    const double trapezoidMacNet = sumFour(
        result.donorTrapezoidMacOutwardVolume );
    const double analytic64LinearNet = sumFour( result.donorAnalyticLinearFlux64 );
    const double analytic256LinearNet = sumFour( result.donorAnalyticLinearFlux256 );
    const double internalLinearNet = sumFour( result.donorInternalLinearFlux );
    const double earlyLinearNet = sumFour( result.donorAnalyticEarlyFlux256 );
    const double lateLinearNet = sumFour( result.donorAnalyticLateFlux256 );
    // Read-only, independently reconstructed identities. These assert only
    // that the recorded geometry/pressure snapshots were accounted for,
    // NOT that trapezoid face flux is the true moving-interface integral.
    const double tolerance = 1.0e-10;
    if( std::abs( result.donorGeometricSnapshotDeltaVolume -
            result.leftSnapshotDeltaVolume ) > tolerance ||
        std::abs( result.donorAnalyticSnapshotDeltaVolume -
            result.donorGeometricSnapshotDeltaVolume ) > 1.0e-6 ||
        std::abs( result.donorPressureSourceVolume -
            result.leftPressureSourceVolume ) > tolerance ||
        std::abs( postMacNet + result.donorPressureSourceVolume -
            result.leftPressureResidualVolume ) > tolerance ||
        std::abs( result.donorAppliedPressureCorrectionVolume -
            ( postMacNet - preMacNet ) ) > tolerance ||
        std::abs( result.donorRequiredPressureCorrectionVolume +
            preMacNet + result.donorPressureSourceVolume ) > tolerance ||
        std::abs( result.donorAppliedPressureCorrectionVolume -
            result.donorRequiredPressureCorrectionVolume -
            result.leftPressureResidualVolume ) > tolerance ||
        !std::isfinite( trapezoidMacNet ) ||
        !std::isfinite( startMacNet ) || !std::isfinite( preMacNet ) ||
        !std::isfinite( analytic256LinearNet ) ||
        !std::isfinite( earlyLinearNet ) || !std::isfinite( lateLinearNet ) )
        throw std::runtime_error( "Capsule GCL/pressure reference bookkeeping failed" );
    // Distinguish precise *geometric* midpoint integration from unknown
    // within-step velocities: the 64->256 comparison is ONLY a quadrature
    // study of the same hypothetical velocity model.
    for( int direction = 0; direction < 4; ++direction )
    {
        const double lower = result.donorAnalyticOpenExposure256[direction];
        const double upper = result.donorOpenExposureSeconds[direction];
        if( !std::isfinite( lower ) || !std::isfinite( upper ) ||
            lower < 0.0 || lower > duration + 1.0e-7 ||
            upper < 0.0 || upper > duration + 1.0e-7 ||
            !std::isfinite( result.donorAnalyticLinearFlux64[direction] ) ||
            !std::isfinite( result.donorInternalLinearFlux[direction] ) )
            throw std::runtime_error( "Capsule time quadrature validity failed" );
    }
    std::printf( "capsule_step96_time_models,%d,%d,"
        "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n",
        controlledDonorTracer ? 1 : 0, splitCount,
        analytic64LinearNet, analytic256LinearNet, internalLinearNet,
        earlyLinearNet, lateLinearNet, postMacNet,
        analytic256LinearNet - postMacNet,
        earlyLinearNet - lateLinearNet );
    for( int direction = 0; direction < 4; ++direction )
        std::printf( "capsule_step96_time_face,%d,%d,%c,"
            "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
            "%.12g,%.12g,%.12g\n",
            controlledDonorTracer ? 1 : 0, splitCount,
            std::array<char, 4>{{ 'W', 'E', 'S', 'N' }}[direction],
            result.donorOpenExposureSeconds[direction],
            result.donorAnalyticOpenExposure64[direction],
            result.donorAnalyticOpenExposure256[direction],
            result.donorInternalLinearFlux[direction],
            result.donorAnalyticLinearFlux64[direction],
            result.donorAnalyticLinearFlux256[direction],
            result.donorAnalyticEarlyFlux256[direction],
            result.donorAnalyticLateFlux256[direction],
            result.donorPostMacOutwardVolume[direction],
            result.donorSolidSweptOutwardVolume[direction] );
    // Print both GCL discrepancy and time-quadrature discrepancies. Neither
    // is coerced toward zero by changing transport or the pressure operator.
    std::printf( "capsule_step96_gcl,%d,%d,"
        "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
        "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n",
        controlledDonorTracer ? 1 : 0, splitCount,
        result.donorAnalyticSnapshotDeltaVolume,
        result.donorGeometricSnapshotDeltaVolume,
        sweptGeometryDelta,
        result.donorGeometricSnapshotDeltaVolume - sweptGeometryDelta,
        result.donorPressureSourceVolume,
        result.donorPressureSourceVolume -
            result.donorGeometricSnapshotDeltaVolume,
        startMacNet, preMacNet, postMacNet,
        trapezoidMacNet, postMacNet - trapezoidMacNet,
        result.leftPressureResidualVolume,
        result.donorMaxAbsLocalGclDefectVolume,
        result.donorMaxAbsLocalPressureResidualVolume,
        result.donorMaxAbsAnalyticFractionGap );
    const std::array<char, 4> faceNames{{ 'W', 'E', 'S', 'N' }};
    for( int direction = 0; direction < 4; ++direction )
        std::printf( "capsule_step96_gcl_face,%d,%d,%c,"
            "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n",
            controlledDonorTracer ? 1 : 0, splitCount, faceNames[direction],
            result.donorOpenExposureSeconds[direction],
            result.donorStartMacOutwardVolume[direction],
            result.donorPreMacOutwardVolume[direction],
            result.donorPostMacOutwardVolume[direction],
            result.donorTrapezoidMacOutwardVolume[direction],
            result.donorSolidSweptOutwardVolume[direction] );
    double totalOutward = 0.0, totalInward = 0.0;
    for( int kind = 0; kind < 4; ++kind )
        for( int direction = 0; direction < 5; ++direction )
        {
            totalOutward += result.donorTransfers[kind][direction][0];
            totalInward += result.donorTransfers[kind][direction][1];
        }
    result.donorExpectedFinalMass =
        result.donorMassBefore - totalOutward + totalInward;
    const double inventoryTolerance =
        1.0e-12 + 1.0e-6 * result.donorMassBefore;
    if( std::abs( result.leftMassAfter -
            result.donorExpectedFinalMass ) > inventoryTolerance ||
        result.maxSweptStageMassDefect > inventoryTolerance ||
        result.maxMacStageMassDefect > inventoryTolerance ||
        result.maxStepMassDefect > inventoryTolerance )
        throw std::runtime_error(
            "Capsule donor actual-transfer ledger fails stage/global continuity" );
    for( int kind = 0; kind < 4; ++kind )
    {
        const double eastNet = result.donorTransfers[kind][1][0] -
            result.donorTransfers[kind][1][1];
        if( std::abs( eastNet - result.signedAppliedPairMass[kind] ) >
            inventoryTolerance )
            throw std::runtime_error( "Capsule donor East pair audit disagrees" );
    }
    if( result.prePressureObservations != splitCount ||
        result.sweptStages != splitCount || result.macStages != splitCount ||
        !std::isfinite( result.linearMacProxyVolume ) ||
        !std::isfinite( result.openExposureSeconds ) ||
        result.openExposureSeconds < 0.0 ||
        result.openExposureSeconds > duration + 1.0e-6 ||
        std::abs( result.massAfter - result.massBefore ) >
            1.0e-6 * std::max( 1.0, result.massBefore ) )
        throw std::runtime_error( "Capsule split stage/positivity/mass gate failed" );

    std::printf( "capsule_step96_donor_balance,%d,%d,%.12g,%.12g,"
        "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
        "%.12g,%.12g,%.12g,%.12g\n",
        controlledDonorTracer ? 1 : 0, splitCount,
        result.donorMassBefore, result.donorMassAfterLastSwept,
        result.leftMassAfter, totalOutward, totalInward,
        result.donorExpectedFinalMass,
        result.leftMassAfter - result.donorExpectedFinalMass,
        result.maxSweptStageMassDefect, result.maxMacStageMassDefect,
        result.maxStepMassDefect,
        result.donorTransfers[1][4][0], result.donorTransfers[1][4][1] );
    constexpr std::array<char, 5> routes{{ 'W', 'E', 'S', 'N', 'X' }};
    for( int direction = 0; direction < 5; ++direction )
    {
        int eventCount = 0;
        for( int kind = 0; kind < 4; ++kind )
            eventCount += result.donorTransferCounts[kind][direction];
        const auto& a = result.donorTransfers;
        std::printf( "capsule_step96_donor_route,%d,%d,%c,"
            "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%d\n",
            controlledDonorTracer ? 1 : 0, splitCount, routes[direction],
            a[0][direction][0], a[0][direction][1],
            a[1][direction][0], a[1][direction][1],
            a[2][direction][0], a[2][direction][1],
            a[3][direction][0], a[3][direction][1], eventCount );
        if( direction < 4 )
            std::printf( "capsule_step96_donor_volume,%d,%d,%c,"
                "%.12g,%.12g,%.12g\n",
                controlledDonorTracer ? 1 : 0, splitCount, routes[direction],
                result.donorPreMacOutwardVolume[direction],
                result.donorPostMacOutwardVolume[direction],
                result.donorSolidSweptOutwardVolume[direction] );
    }
    std::printf( "capsule_step96_split,%d,%d,%.12g,%.12g,%.12g,%.12g,"
        "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
        "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n",
        controlledDonorTracer ? 1 : 0, splitCount,
        result.openExposureSeconds, result.prePressureMacVolume,
        result.endpointMacVolume, result.linearMacProxyVolume,
        result.solidSweptFaceVolume, result.leftPressureSourceVolume,
        result.rightPressureSourceVolume, result.leftSnapshotDeltaVolume,
        result.leftPressureResidualVolume,
        result.signedAppliedPairMass[0], result.signedAppliedPairMass[1],
        result.signedAppliedPairMass[2], result.signedAppliedPairMass[3],
        result.massBefore, result.massAfter, result.leftMassAfter,
        result.leftVolumeAfter, result.maxAbsRawFaceVelocity );
    return result;
}

// Same pre-pressure Field, geometry, current MAC velocity and obstacle pose
// across all budgets. Split-96 reopening's 120-iteration live result is
// checked against the independent 120-iteration frozen replay afterward.
// The component mean is reconstructed from volume-valued pre-MAC + source:
// it is a Neumann compatibility floor, not the exact pressure error itself.
double auditFrozenReopeningPressure(
    const fluidField2& frozen, float dt, int splits, int part )
{
    constexpr int donorX = 28, donorY = 26;
    const int width = frozen.getWidth();
    const int height = frozen.getHeight();
    const int donor = donorY * width + donorX;
    const auto index = [&]( int x, int y ) { return y * width + x; };

    // Reconstruct EVERY open-face-connected Neumann component, not only
    // the donor's large region: other pockets can dominate the global
    // compatibility budget while the donor-region mean stays near zero.
    std::vector<int> regionOf( width * height, -1 );
    std::vector<std::vector<int>> regions;
    for( int start = 0; start < width * height; ++start )
    {
        const int startX = start % width, startY = start / width;
        if( frozen.getCellFluidFraction( startX, startY ) <= 0.0f ||
            regionOf[start] >= 0 ) continue;
        const int id = static_cast<int>( regions.size() );
        regions.emplace_back();
        auto& cells = regions.back();
        cells.push_back( start );
        regionOf[start] = id;
        for( std::size_t read = 0; read < cells.size(); ++read )
        {
            const int cell = cells[read];
            const int x = cell % width, y = cell / width;
            const auto visit = [&]( int nx, int ny, float open )
            {
                if( open <= 0.0f ||
                    frozen.getCellFluidFraction( nx, ny ) <= 0.0f )
                    return;
                const int neighbor = index( nx, ny );
                if( regionOf[neighbor] >= 0 ) return;
                regionOf[neighbor] = id;
                cells.push_back( neighbor );
            };
            if( x > 0 ) visit( x - 1, y,
                frozen.getUFaceOpenFraction( x, y ) );
            if( x + 1 < width ) visit( x + 1, y,
                frozen.getUFaceOpenFraction( x + 1, y ) );
            if( y > 0 ) visit( x, y - 1,
                frozen.getVFaceOpenFraction( x, y ) );
            if( y + 1 < height ) visit( x, y + 1,
                frozen.getVFaceOpenFraction( x, y + 1 ) );
        }
    }
    if( regionOf[donor] < 0 )
        throw std::runtime_error( "frozen reopening donor region missing" );
    const int donorRegion = regionOf[donor];
    const auto& queue = regions[donorRegion];
    if( queue.empty() )
        throw std::runtime_error( "frozen reopening donor region empty" );

    const double area = static_cast<double>( frozen.getCellSize() ) *
        frozen.getCellSize();
    double initialSource = 0.0;
    double initialMean = 0.0;
    int initialRegions = -1;
    double baselineResidual = 0.0;
    for( const int iterations : { 120, 240, 480 } )
    {
        fluidField2 replay = frozen;
        replay.setPrePressureSnapshotObserver( {} );
        const auto forces = replay.replayFrozenCutCellPressure( iterations );
        if( !forces || forces->size() != 1 ||
            replay.replayFrozenCutCellPressure().has_value() )
            throw std::runtime_error( "frozen reopening replay not one-use" );
        const auto& pressure = replay.getCutCellPressure();
        if( pressure.usedCoupledProjection || pressure.coupledSolveFailed ||
            pressure.connectedRegions < 1 )
            throw std::runtime_error( "frozen reopening unexpectedly coupled" );

        double rawSum = 0.0, postSum = 0.0, compatibleSquared = 0.0;
        for( const int cell : queue )
        {
            const int x = cell % width, y = cell / width;
            const auto before = frozen.getCutCellVolumeBudget( x, y, dt );
            const auto after = replay.getCutCellVolumeBudget( x, y, dt );
            if( !before.valid || !after.valid || !after.active ||
                !after.hasVolumeHistory || !after.hasSweptHistory )
                throw std::runtime_error( "frozen reopening component budget missing" );
            rawSum += before.macNetOutwardVolume +
                after.pressureSourceDeltaVolume;
            postSum += after.pressureResidualVolume;
        }
        const double meanVolume = rawSum / queue.size();
        for( const int cell : queue )
        {
            const int x = cell % width, y = cell / width;
            const auto budget = replay.getCutCellVolumeBudget( x, y, dt );
            const double compatible = budget.pressureResidualVolume - meanVolume;
            compatibleSquared += compatible * compatible;
        }
        const auto donorBudget =
            replay.getCutCellVolumeBudget( donorX, donorY, dt );
        const double compatibleRms =
            std::sqrt( compatibleSquared / queue.size() );
        const double source = donorBudget.pressureSourceDeltaVolume;
        if( !std::isfinite( meanVolume ) ||
            !std::isfinite( compatibleRms ) ||
            !std::isfinite( donorBudget.pressureResidualVolume ) ||
            !std::isfinite( pressure.compatibilityCorrection ) ||
            !std::isfinite( pressure.pressureResidual ) ||
            !std::isfinite( postSum ) )
            throw std::runtime_error( "nonfinite frozen reopening diagnostics" );
        if( initialRegions < 0 )
        {
            initialRegions = pressure.connectedRegions;
            initialSource = source;
            initialMean = meanVolume;
            baselineResidual = donorBudget.pressureResidualVolume;
        }
        else if( pressure.connectedRegions != initialRegions ||
            std::abs( source - initialSource ) > 1.0e-10 ||
            std::abs( meanVolume - initialMean ) > 1.0e-10 )
            throw std::runtime_error(
                "reopening frozen geometry/source changed with Jacobi budget" );

        // Recompute the raw RHS for each connected pocket from the SAME
        // frozen MAC and the pressure replay's moving-volume source.
        // A nonzero global correction can come from other pockets even
        // when this donor component's uncorrected sum is ~zero.
        double componentAbsoluteSourceRate = 0.0;
        for( std::size_t id = 0; id < regions.size(); ++id )
        {
            const auto& cells = regions[id];
            double componentRaw = 0.0, componentPost = 0.0;
            for( int cell : cells )
            {
                const int cx = cell % width, cy = cell / width;
                const auto before =
                    frozen.getCutCellVolumeBudget( cx, cy, dt );
                const auto after =
                    replay.getCutCellVolumeBudget( cx, cy, dt );
                if( !before.valid || !after.valid || !after.active )
                    throw std::runtime_error(
                        "frozen reopening regional budget is invalid" );
                componentRaw += before.macNetOutwardVolume +
                    after.pressureSourceDeltaVolume;
                componentPost += after.pressureResidualVolume;
            }
            const double componentMean = componentRaw / cells.size();
            double squared = 0.0;
            for( int cell : cells )
            {
                const auto after = replay.getCutCellVolumeBudget(
                    cell % width, cell / width, dt );
                const double remaining =
                    after.pressureResidualVolume - componentMean;
                squared += remaining * remaining;
            }
            const double rms = std::sqrt( squared / cells.size() );
            componentAbsoluteSourceRate += std::abs( componentRaw ) / dt;
            if( !std::isfinite( componentRaw ) ||
                !std::isfinite( componentPost ) || !std::isfinite( rms ) )
                throw std::runtime_error(
                    "nonfinite frozen reopening regional compatibility" );
            std::printf( "capsule_step96_reopen_region,%d,%d,%d,%zu,%zu,%d,%d",
                splits, part, iterations, id, cells.size(), cells.front(),
                static_cast<int>( id ) == donorRegion ? 1 : 0 );
            const std::array<double, 5> measures{{
                componentRaw, componentMean, componentPost,
                componentPost - componentRaw, rms
            }};
            for( double value : measures ) std::printf( ",%.12g", value );
            std::putchar( '\n' );
        }
        const double computedCompatibility =
            std::abs( static_cast<double>( pressure.unresolvedVolumeRate ) ) +
            componentAbsoluteSourceRate;
        if( !std::isfinite( computedCompatibility ) )
            throw std::runtime_error(
                "nonfinite reconstructed reopening compatibility" );
        std::printf( "capsule_step96_reopen_compat,%d,%d,%d,%zu,%d",
            splits, part, iterations, regions.size(), pressure.connectedRegions );
        const std::array<double, 5> compatibility{{
            static_cast<double>( pressure.compatibilityCorrection ),
            computedCompatibility,
            static_cast<double>( pressure.compatibilityCorrection ) -
                computedCompatibility,
            static_cast<double>( pressure.unresolvedVolumeRate ),
            componentAbsoluteSourceRate
        }};
        for( double value : compatibility ) std::printf( ",%.12g", value );
        std::putchar( '\n' );

        // The area-weighted region sum is a measured, uncorrected
        // compatibility imbalance. It may differ from the solver's
        // global compatibilityCorrection, which also includes other regions
        // and unresolved sources. Never label a 3x3 patch as a full region.
        std::printf( "capsule_step96_reopen_frozen,%d,%d,%d,%d,%zu",
            splits, part, iterations, pressure.connectedRegions, queue.size() );
        const std::array<double, 11> metrics{{
            rawSum, meanVolume, postSum, postSum - rawSum,
            donorBudget.pressureResidualVolume,
            donorBudget.pressureResidualVolume - meanVolume,
            compatibleRms, source,
            static_cast<double>( pressure.compatibilityCorrection ),
            static_cast<double>( pressure.pressureResidual ), area
        }};
        for( double value : metrics ) std::printf( ",%.12g", value );
        std::putchar( '\n' );
    }
    return baselineResidual;
}

// Reverse prescribed motion on copies of the same actually evolved closing
// step-96 Field. This does not reverse the dissipative fluid integrator.
void auditCapsuleStep96Reopening( const fluidField2& step95,
    fluidShapeObstacle2 capsule, float startAngle, float endAngle,
    int totalSteps, float duration, int splits, bool seeded )
{
    if( splits < 1 ) throw std::runtime_error( "invalid reopening split count" );
    constexpr int x = 28, y = 26;
    constexpr std::array<char, 4> faceNames{{ 'W', 'E', 'S', 'N' }};
    fluidField2 field = step95;
    field.setTracerTransferObserver( {} );
    field.setTracerStageObserver( {} );
    field.setPrePressureSnapshotObserver( {} );
    field.setSweptApertureAuditEnabled( true );
    if( field.getSmallCellPressureMergeFraction() != 0.0f ||
        field.isConnectedResidualTracerRemapEnabled() ||
        field.isCutCellCoupledProjectionEnabled() )
        throw std::runtime_error( "reopening witness altered default solver" );
    if( seeded )
    {
        if( field.getCellFluidFraction( x, y ) <= 0.0f )
            throw std::runtime_error( "invalid reopening donor seed" );
        field.setTracerConcentration( x, y, 1.0f );
    }
    const auto setPose = [&]( double position )
    {
        capsule.transform.rotation = rot2::FromRadians( startAngle +
            static_cast<float>( position / totalSteps ) *
            ( endAngle - startAngle ) );
    };
    setPose( 96.0 );
    std::array<fluidObstacleForce2, 1> warmForces{};
    field.step( duration, std::span<const fluidShapeObstacle2>( &capsule, 1 ),
        warmForces, 120 );
    const double closedFraction = field.getCellFluidFraction( x, y );
    if( !std::isfinite( closedFraction ) || closedFraction <= 0.0 )
        throw std::runtime_error( "invalid reopening closed fraction" );

    const float dt = duration / splits;
    const double faceVolumeScale = static_cast<double>( field.getCellSize() ) * dt;
    capsule.angularVelocity = -( endAngle - startAngle ) /
        ( duration * totalSteps );
    const auto faceState = [&]( const fluidField2& state, int direction )
    {
        const bool isU = direction < 2;
        const int fx = direction == 1 ? x + 1 : x;
        const int fy = direction == 3 ? y + 1 : y;
        const float open = isU ? state.getUFaceOpenFraction( fx, fy ) :
                                 state.getVFaceOpenFraction( fx, fy );
        const auto velocity = isU ? state.getUFaceVelocityForAudit( fx, fy ) :
                                    state.getVFaceVelocityForAudit( fx, fy );
        if( !velocity || !std::isfinite( *velocity ) ||
            !std::isfinite( open ) || open < 0.0f || open > 1.0f )
            throw std::runtime_error( "invalid reopening face state" );
        return std::array<double, 2>{{
            static_cast<double>( open ),
            ( direction == 0 || direction == 2 ? -1.0 : 1.0 ) * *velocity
        }};
    };
    const auto tracerMass = []( const fluidField2& state )
    {
        double total = 0.0;
        for( int cy = 0; cy < state.getHeight(); ++cy )
            for( int cx = 0; cx < state.getWidth(); ++cx )
                total += state.getTracerCellMass( cx, cy );
        return total;
    };
    // Sum a fixed 3x3 ACTIVE-cell patch. It is not a closed component.
    const auto samplePatch = [&]( const fluidField2& state )
    {
        std::array<double, 3> result{}; // net MAC volume, source, active cells
        for( int py = y - 1; py <= y + 1; ++py )
            for( int px = x - 1; px <= x + 1; ++px )
            {
                const auto cell = state.getCutCellVolumeBudget( px, py, dt );
                if( !cell.valid ) throw std::runtime_error( "invalid reopening patch" );
                if( !cell.active ) continue;
                result[0] += cell.macNetOutwardVolume;
                result[1] += cell.pressureSourceDeltaVolume;
                result[2] += 1.0;
            }
        return result;
    };
    double preNet = 0.0, appliedDonorKg = 0.0;
    std::optional<double> frozen120DonorResidual{};
    int currentPart = -1;
    int snapshots = 0;
    std::array<std::array<double, 2>, 4> preFaces{};
    std::array<double, 3> prePatch{};
    field.setTracerTransferObserver( [&]( const fluidTracerTransfer2& event )
    {
        if( !std::isfinite( event.mass ) || event.mass < 0.0 )
            throw std::runtime_error( "nonfinite reopening tracer transfer" );
        const int donor = y * field.getWidth() + x;
        if( event.sourceCell == donor ) appliedDonorKg -= event.mass;
        if( event.destinationCell == donor ) appliedDonorKg += event.mass;
    } );
    field.setPrePressureSnapshotObserver( [&]( const fluidField2& pre )
    {
        const auto donor = pre.getCutCellVolumeBudget( x, y, dt );
        if( !donor.valid ) throw std::runtime_error( "missing pre-pressure reopening state" );
        preNet = donor.macNetOutwardVolume;
        prePatch = samplePatch( pre );
        // Only the unseeded control needs the expensive three-budget replay.
        // Capture first and last real reverse substeps, without replacing
        // the live 120-iteration physics.
        if( !seeded && ( currentPart == 0 || currentPart + 1 == splits ) )
            frozen120DonorResidual = auditFrozenReopeningPressure(
                pre, dt, splits, currentPart );
        for( int direction = 0; direction < 4; ++direction )
        {
            preFaces[direction] = faceState( pre, direction );
            if( std::abs( faceVolumeScale * preFaces[direction][0] *
                    preFaces[direction][1] -
                    donor.macOutwardVolume[direction] ) > 1.0e-10 )
                throw std::runtime_error( "reopening pre-MAC face mismatch" );
        }
        ++snapshots;
    } );
    double previousFraction = closedFraction;
    for( int part = 0; part < splits; ++part )
    {
        const double massBefore = tracerMass( field );
        const double donorBefore = field.getTracerCellMass( x, y );
        appliedDonorKg = 0.0;
        frozen120DonorResidual.reset();
        currentPart = part;
        std::array<std::array<double, 2>, 4> startFaces{};
        for( int dir = 0; dir < 4; ++dir ) startFaces[dir] = faceState( field, dir );
        setPose( 96.0 - static_cast<double>( part + 1 ) / splits );
        std::array<fluidObstacleForce2, 1> feedback{};
        field.step( dt, std::span<const fluidShapeObstacle2>( &capsule, 1 ),
            feedback, 120 );
        const auto donor = field.getCutCellVolumeBudget( x, y, dt );
        const auto postPatch = samplePatch( field );
        const double fraction = field.getCellFluidFraction( x, y );
        const double required = -( preNet + donor.pressureSourceDeltaVolume );
        const double applied = donor.macNetOutwardVolume - preNet;
        const double gcl = donor.snapshotDeltaVolume -
            donor.solidSweptDivergenceVolume;
        const double donorAfter = field.getTracerCellMass( x, y );
        const double massDefect = donorAfter - donorBefore - appliedDonorKg;
        if( frozen120DonorResidual &&
            std::abs( *frozen120DonorResidual -
                donor.pressureResidualVolume ) > 1.0e-9 )
            throw std::runtime_error(
                "same-state 120-iteration frozen/live reopening pressure disagreed" );
        const double massAfter = tracerMass( field );
        const double massTolerance = 1.0e-12 +
            1.0e-6 * std::max( 1.0, massBefore );
        if( snapshots != part + 1 || !donor.valid ||
            !donor.hasVolumeHistory || !donor.hasSweptHistory ||
            !std::isfinite( fraction ) ||
            fraction + 1.0e-7 < previousFraction ||
            std::abs( applied - required -
                donor.pressureResidualVolume ) > 1.0e-10 ||
            std::abs( massDefect ) > massTolerance ||
            std::abs( massAfter - massBefore ) > massTolerance ||
            prePatch[2] != postPatch[2] ||
            !std::isfinite( feedback[0].force.x ) ||
            !std::isfinite( feedback[0].force.y ) ||
            !std::isfinite( feedback[0].torque ) )
            throw std::runtime_error( "reopening pressure/mass stage regression" );

        std::printf( "capsule_step96_reopen_step,%d,%d,%d",
            seeded ? 1 : 0, splits, part );
        const std::array<double, 19> measurements{{
            previousFraction, fraction, donor.snapshotDeltaVolume,
            donor.solidSweptDivergenceVolume, donor.pressureSourceDeltaVolume,
            preNet, donor.macNetOutwardVolume, required, applied,
            donor.pressureResidualVolume, gcl,
            prePatch[0], postPatch[0], postPatch[1],
            postPatch[0] + postPatch[1],
            donorBefore, donorAfter, appliedDonorKg, massDefect
        }};
        for( double value : measurements ) std::printf( ",%.12g", value );
        std::putchar( '\n' );
        for( int dir = 0; dir < 4; ++dir )
        {
            const auto endFace = faceState( field, dir );
            const double preVolume = faceVolumeScale *
                preFaces[dir][0] * preFaces[dir][1];
            const double postVolume = donor.macOutwardVolume[dir];
            if( std::abs( preFaces[dir][0] - endFace[0] ) > 1.0e-6 ||
                std::abs( faceVolumeScale * endFace[0] *
                    endFace[1] - postVolume ) > 1.0e-10 )
                throw std::runtime_error( "reopening post-MAC face mismatch" );
            std::printf( "capsule_step96_reopen_face,%d,%d,%d,%c",
                seeded ? 1 : 0, splits, part, faceNames[dir] );
            const std::array<double, 8> faceMeasurements{{
                startFaces[dir][0], preFaces[dir][0], endFace[0],
                startFaces[dir][1], preFaces[dir][1], endFace[1],
                preVolume, postVolume
            }};
            for( double value : faceMeasurements ) std::printf( ",%.12g", value );
            std::putchar( '\n' );
        }
        previousFraction = fraction;
    }
    if( previousFraction <= closedFraction )
        throw std::runtime_error( "reopening did not expand donor fluid volume" );
}

// Live prescribed-motion complement to the frozen zero-MAC geometry oracle.
// Both fields receive identical transforms, velocity sources and tracer seed;
// only the convex full-coverage guard differs. Pressure/advective histories
// are separately evolved and the obstacle is KINEMATIC, so returned forces
// are observations, not an integrated two-way World/body trajectory.
void auditMovingCapsuleGuardTransport( bool hotspotLedger = false,
    bool pressureOracle = false, bool step96SplitFlux = false )
{
    constexpr float h = 0.125f;
    constexpr float totalDt = 1.0f / 30.0f;
    constexpr int steps = 128;
    constexpr float subDt = totalDt / steps;
    constexpr float startAngle = 0.18f, endAngle = 1.28f;
    constexpr vec2 center{ -0.20f, -0.12f };

    fluidField2 withoutGuard{ 64, 64, h, { -4.0f, -4.0f } };
    withoutGuard.setCutCellProjectionEnabled( true );
    withoutGuard.setAnalyticCapsuleCutCellEnabled( true );
    withoutGuard.setAnalyticCapsuleDoubleGeometryEnabled( true );
    withoutGuard.setSweptTracerRemapEnabled( true );
    fluidField2 withGuard = withoutGuard;
    withGuard.setAnalyticCapsuleFullCoverageGuardEnabled( true );
    std::array<fluidField2*, 2> fields{{ &withoutGuard, &withGuard }};
    fluidShapeObstacle2 capsule{
        capsule2{ { -0.63f, 0.0f }, { 0.63f, 0.0f }, 0.19f },
        { center, rot2::FromRadians( startAngle ) }, center,
        {}, 0.0f, 0.0f, 0.0f };
    capsule.angularVelocity = ( endAngle - startAngle ) / totalDt;
    for( fluidField2* field : fields )
    {
        if( field->getSmallCellPressureMergeFraction() != 0.0f ||
            field->isConnectedResidualTracerRemapEnabled() ||
            field->isCutCellCoupledProjectionEnabled() )
            throw std::runtime_error( "moving Capsule transport control enabled unrelated physics" );
        std::array<fluidObstacleForce2, 1> forces{};
        field->step( subDt, std::span<const fluidShapeObstacle2>( &capsule, 1 ),
            forces, 120 );
    }

    // Seed only completely fluid cells near the swept Capsule region, so
    // both policies start with the same EXACT stored tracer mass.
    int seeded = 0;
    for( int y = 24; y <= 39; ++y )
        for( int x = 23; x <= 41; ++x )
        {
            if( withoutGuard.getCellFluidFraction( x, y ) != 1.0f ||
                withGuard.getCellFluidFraction( x, y ) != 1.0f ) continue;
            const double dx = ( x + 0.5 - 31.0 ) * h;
            const double dy = ( y + 0.5 - 31.0 ) * h;
            const float concentration = static_cast<float>(
                0.25 + 1.75 * std::exp( -3.5 * ( dx * dx + dy * dy ) ) );
            for( fluidField2* field : fields )
                field->setTracerConcentration( x, y, concentration );
            ++seeded;
        }
    if( seeded < 20 )
        throw std::runtime_error( "moving Capsule did not seed enough shared open cells" );
    for( fluidField2* field : fields )
    {
        field->addVelocity( { -0.24f, -0.30f }, { 1.3f, 0.65f }, 0.65f );
        field->addVelocity( { 0.35f, 0.25f }, { -0.55f, 0.8f }, 0.60f );
    }
    const auto totalMass = []( const fluidField2& field )
    {
        double mass = 0.0;
        for( int y = 0; y < field.getHeight(); ++y )
            for( int x = 0; x < field.getWidth(); ++x )
                mass += field.getTracerCellMass( x, y );
        return mass;
    };
    const double seededMass = totalMass( withoutGuard );
    if( !( seededMass > 0.0 ) ||
        totalMass( withGuard ) != seededMass )
        throw std::runtime_error( "moving Capsule policies seeded different tracer mass" );

    // S2c forensic-only witness. No callback, snapshot copy or solver change
    // is installed by the legacy moving-Capsule transport invocation.
    constexpr int hotspotStep = 96;
    constexpr int hotspotX = 29, hotspotY = 26;
    constexpr int leftX = hotspotX - 1;
    constexpr int leftCell = hotspotY * 64 + leftX;
    constexpr int rightCell = hotspotY * 64 + hotspotX;
    int activeStep = 0;
    struct hotspotRecord
    {
        std::optional<fluidField2> frozen{};
        std::array<double, 4> signedTransfer{}; // swept, fallback, MAC, connected
        std::array<std::array<double, 2>, 2> stageMass{}; // swept/MAC, left/right
        int stageSamples = 0;
        int directedTransfers = 0;
    };
    std::array<hotspotRecord, 2> hotspot{};
    if( hotspotLedger )
        for( int mode = 0; mode < 2; ++mode )
        {
            fields[mode]->setFaceStageAuditTarget( hotspotX, hotspotY, true );
            fields[mode]->setPrePressureSnapshotObserver(
                [&, mode]( const fluidField2& frozen )
                {
                    if( activeStep == hotspotStep )
                    {
                        if( hotspot[mode].frozen )
                            throw std::runtime_error( "duplicated hotspot frozen checkpoint" );
                        hotspot[mode].frozen.emplace( frozen );
                    }
                } );
            fields[mode]->setTracerTransferObserver(
                [&, mode]( const fluidTracerTransfer2& transfer )
                {
                    if( activeStep != hotspotStep ) return;
                    const bool positive = transfer.sourceCell == leftCell &&
                        transfer.destinationCell == rightCell;
                    const bool negative = transfer.sourceCell == rightCell &&
                        transfer.destinationCell == leftCell;
                    if( !positive && !negative ) return;
                    const auto kind = static_cast<int>( transfer.kind );
                    if( kind < 0 || kind >= 4 || !std::isfinite( transfer.mass ) ||
                        transfer.mass < 0.0 )
                        throw std::runtime_error( "invalid hotspot applied transfer" );
                    hotspot[mode].signedTransfer[kind] +=
                        positive ? transfer.mass : -transfer.mass;
                    ++hotspot[mode].directedTransfers;
                } );
        }

    struct transportLedger
    {
        int sweptStages = 0;
        int macStages = 0;
        double maxStageMassDefect = 0.0;
        double maxStepMassDefect = 0.0;
        double maxVolumeDrift = 0.0;
        double maxSweptMismatch = 0.0;
        double peakForce = 0.0;
        double peakTorque = 0.0;
        double sumRemappedMass = 0.0;
        double sumFallbackMass = 0.0;
        double sumUnresolvedMass = 0.0;
    };
    std::array<transportLedger, 2> ledgers{};
    for( int mode = 0; mode < 2; ++mode )
        fields[mode]->setTracerStageObserver(
            [&, mode]( const fluidField2& field, fluidTracerStage2 stage )
            {
                auto& record = ledgers[mode];
                if( stage == fluidTracerStage2::afterSweptRemap )
                    ++record.sweptStages;
                else if( stage == fluidTracerStage2::afterMacAdvection )
                    ++record.macStages;
                const double mass = totalMass( field );
                if( !std::isfinite( mass ) )
                    throw std::runtime_error( "nonfinite moving Capsule tracer stage mass" );
                record.maxStageMassDefect = std::max(
                    record.maxStageMassDefect, std::abs( mass - seededMass ) );
                if( hotspotLedger && activeStep == hotspotStep )
                {
                    const int index = stage == fluidTracerStage2::afterSweptRemap ? 0 : 1;
                    hotspot[mode].stageMass[index] = {{
                        field.getTracerCellMass( leftX, hotspotY ),
                        field.getTracerCellMass( hotspotX, hotspotY ) }};
                    ++hotspot[mode].stageSamples;
                }
            } );

    if( hotspotLedger )
    {
        std::puts( "capsule_hotspot_cell,mode,side,current_fraction,current_volume_m3,"
            "snapshot_delta_m3,reconciled_swept_outward_m3,pressure_source_m3,"
            "source_minus_snapshot_m3,pre_mac_outward_m3,"
            "post_mac_outward_m3,pre_flux_plus_current_source_m3,post_residual_m3" );
        std::puts( "capsule_hotspot_face,mode,pre_velocity_mps,post_velocity_mps,"
            "pressure_delta_mps,pressure_impulse_proxy_Ns,endpoint_open,"
            "time_mean_open,mean_open_capacity_m3_per_mps,nominal_endpoint_flux_m3,"
            "linear_time_proxy_flux_m3,small_cell_volume_m3,"
            "nominal_to_cell_volume_ratio,replay_minus_live_flux_m3,"
            "global_compatibility_m3ps" );
        std::puts( "capsule_hotspot_tracer,mode,applied_swept_signed_kg,"
            "applied_fallback_signed_kg,applied_mac_signed_kg,"
            "applied_connected_signed_kg,left_after_swept_kg,"
            "right_after_swept_kg,left_after_mac_kg,right_after_mac_kg,"
            "direct_pair_transfers" );
        std::puts( "capsule_hotspot_temporal,mode,samples,nonzero_midpoints,"
            "first_open_alpha,last_open_alpha,max_midpoint_open,"
            "mean_open,alpha_open_moment,mean_open_capacity_m3_per_mps,"
            "built_in_mean_open,endpoint_open_analytic,endpoint_open_stored" );
        std::puts( "capsule_hotspot_swept,branch,raw_face_m3,reconciled_face_m3,"
            "face_correction_m3,corrected_faces,correction_abs_flux_m3,"
            "raw_geometry_mismatch_m3,final_geometry_mismatch_m3,"
            "left_pressure_source_m3,right_pressure_source_m3,"
            "left_pre_mac_out_m3,right_pre_mac_out_m3,"
            "left_final_mac_out_m3,right_final_mac_out_m3,"
            "left_final_pressure_residual_m3,right_final_pressure_residual_m3,"
            "pressure_compatibility_m3ps,max_raw_face_velocity_mps" );
        std::puts( "capsule_hotspot_region,branch,region,region_cells,"
            "pre_mac_net_m3,pressure_source_m3,snapshot_delta_m3,"
            "swept_divergence_m3,pre_projection_rhs_m3,"
            "per_cell_removed_mean_m3,reported_compatibility_m3ps,"
            "reconstructed_compatibility_m3ps,compatibility_gap_m3ps" );
        std::puts( "capsule_hotspot_float_rhs,branch,region,region_cells,"
            "float_order_region_sum_per_s,float_order_region_mean_per_s,"
            "float_order_compatibility_m3ps,reported_compatibility_m3ps,"
            "float_minus_reported_m3ps,double_minus_float_m3ps,"
            "float_order_source_imbalance_m3ps,reported_source_imbalance_m3ps,"
            "source_imbalance_gap_m3ps,max_cell_rhs_diff_per_s" );
        if( pressureOracle )
            std::puts( "capsule_hotspot_pressure_oracle,branch,merge_fraction,"
                "merged_cells,active_roots,target_face_internal,source_sum_before_m3,"
                "source_sum_after_m3,neumann_gauge_adjustment_L1_m3,"
                "iterations,grouped_converged,initial_group_rms_per_s,"
                "final_group_rms_per_s,final_pointwise_rms_per_s,"
                "target_predicted_U_mps,max_predicted_face_mps,"
                "max_open_weighted_predicted_mps,pressure_impulse_x_Ns,"
                "pressure_impulse_y_Ns,merged_child_residual_L1_m3" );
        if( pressureOracle )
            std::puts( "capsule_hotspot_internal_transfer,branch,merge_fraction,"
                "child_cell,parent_cell,axis,face_x,face_y,"
                "required_child_outward_m3,source_cell,destination_cell,"
                "start_open,end_open,time_mean_open,pre_pressure_face_mps,"
                "needed_time_mean_speed_mps,needed_endpoint_speed_mps,"
                "reference_time_open_volume_m3,donor_previous_volume_m3,"
                "donor_current_volume_m3,donor_current_mass_kg,"
                "bounded_volume_m3,bounded_mass_kg,"
                "unfulfilled_volume_m3,pair_volume_net_m3,pair_mass_net_kg,"
                "child_pointwise_residual_m3,root_group_residual_m3" );
    }
    std::puts( "capsule_moving_velocity_hotspot,step,mode,axis,face_x,face_y,"
        "face_velocity_mps,open_fraction,open_weighted_speed_mps,"
        "nominal_open_flux_m3,lower_cell_fraction,upper_cell_fraction,"
        "lower_pressure_source_m3,upper_pressure_source_m3,"
        "lower_pressure_residual_m3,upper_pressure_residual_m3" );
    std::puts( "capsule_moving_transport,step,checkpoint_16,mode,"
        "fluid_volume_m3,snapshot_delta_m3,pressure_source_m3,"
        "swept_geometry_mismatch_m3,tracer_mass_kg,trapped_mass_kg,"
        "swept_remapped_kg,fallback_remapped_kg,unresolved_kg,"
        "max_face_velocity_mps,force_x_N,force_y_N,torque_Nm,"
        "pressure_active_cells,pressure_residual_rms_m3" );
    if( step96SplitFlux )
    {
        std::puts( "capsule_step96_reopen_region,splits,substep,"
            "pressure_iterations,region_id,region_cells,first_cell_id,"
            "contains_donor,raw_rhs_volume_m3,raw_rhs_mean_m3,"
            "post_residual_sum_m3,post_minus_raw_m3,"
            "compatible_residual_rms_m3" );
        std::puts( "capsule_step96_reopen_compat,splits,substep,"
            "pressure_iterations,reconstructed_regions,solver_regions,"
            "solver_compatibility_rate_m3ps,recomputed_compatibility_rate_m3ps,"
            "solver_minus_recomputed_m3ps,unresolved_volume_rate_m3ps,"
            "sum_abs_component_rhs_rate_m3ps" );
        std::puts( "capsule_step96_reopen_frozen,splits,substep,"
            "pressure_iterations,connected_regions,donor_region_cells,"
            "region_raw_rhs_volume_m3,region_mean_compatibility_volume_m3,"
            "region_post_residual_sum_m3,region_post_minus_raw_m3,"
            "donor_post_residual_m3,donor_compatible_residual_m3,"
            "region_compatible_rms_m3,donor_pressure_source_m3,"
            "global_compatibility_correction_m3ps,"
            "legacy_pressure_residual_native_units,cell_area_m2" );
        std::puts( "capsule_step96_reopen_step,seed,splits,substep,"
            "previous_fraction,current_fraction,snapshot_delta_m3,"
            "swept_divergence_m3,pressure_source_m3,pre_mac_net_m3,"
            "post_mac_net_m3,required_correction_m3,applied_correction_m3,"
            "pressure_residual_m3,gcl_defect_m3,patch_pre_mac_m3,"
            "patch_post_mac_m3,patch_source_m3,patch_residual_m3,"
            "donor_mass_before_kg,donor_mass_after_kg,"
            "donor_net_applied_transfer_kg,donor_mass_defect_kg" );
        std::puts( "capsule_step96_reopen_face,seed,splits,substep,"
            "face_WESN,start_open,pre_open,post_open,"
            "start_signed_mps,pre_signed_mps,post_signed_mps,"
            "pre_mac_outward_m3,post_mac_outward_m3" );
        std::puts( "capsule_step96_time_models,controlled_donor_tracer,"
            "fluid_substeps,analytic64_linear_proxy_net_m3,"
            "analytic256_linear_proxy_net_m3,internal_linear_proxy_net_m3,"
            "analytic256_early_proxy_net_m3,analytic256_late_proxy_net_m3,"
            "endpoint_post_pressure_mac_net_m3,"
            "analytic256_linear_minus_endpoint_m3,"
            "early_minus_late_model_spread_m3" );
        std::puts( "capsule_step96_time_face,controlled_donor_tracer,"
            "fluid_substeps,face_WESN,internal_open_exposure_s,"
            "analytic64_open_exposure_s,analytic256_open_exposure_s,"
            "internal_linear_proxy_m3,analytic64_linear_proxy_m3,"
            "analytic256_linear_proxy_m3,analytic256_early_proxy_m3,"
            "analytic256_late_proxy_m3,post_pressure_endpoint_mac_m3,"
            "solid_swept_outward_m3" );
        std::puts( "capsule_step96_stage_face,controlled_donor_tracer,"
            "fluid_substeps,substep,face_WESN,"
            "start_open_fraction,pre_pressure_open_fraction,"
            "post_pressure_open_fraction,start_signed_velocity_mps,"
            "pre_pressure_signed_velocity_mps,post_pressure_signed_velocity_mps,"
            "start_mac_outward_m3,pre_pressure_mac_outward_m3,"
            "post_pressure_mac_outward_m3,pressure_stage_delta_outward_m3" );
        std::puts( "capsule_step96_pressure_step,controlled_donor_tracer,"
            "fluid_substeps,substep,previous_fraction,current_fraction,"
            "snapshot_delta_m3,swept_solid_divergence_m3,pressure_source_m3,"
            "pre_pressure_mac_net_m3,post_pressure_mac_net_m3,"
            "required_pressure_correction_m3,applied_pressure_correction_m3,"
            "pressure_residual_m3,stored_minus_swept_gcl_m3,"
            "source_minus_stored_m3,post_plus_stored_m3" );
        std::puts( "capsule_step96_gcl,controlled_donor_tracer,fluid_substeps,"
            "analytic_snapshot_delta_m3,stored_snapshot_delta_m3,"
            "swept_solid_divergence_m3,stored_minus_swept_gcl_m3,"
            "pressure_source_m3,pressure_source_minus_snapshot_m3,"
            "start_mac_outward_sum_m3,pre_pressure_mac_outward_sum_m3,"
            "post_pressure_mac_outward_sum_m3,"
            "actual_endpoint_trapezoid_mac_outward_sum_m3,"
            "post_minus_trapezoid_m3,pressure_residual_sum_m3,"
            "max_local_gcl_defect_m3,max_local_pressure_residual_m3,"
            "max_analytic_vs_stored_fraction_gap" );
        std::puts( "capsule_step96_gcl_face,controlled_donor_tracer,"
            "fluid_substeps,face_WESN,face_open_exposure_s,"
            "start_mac_outward_m3,pre_pressure_mac_outward_m3,"
            "post_pressure_mac_outward_m3,"
            "actual_endpoint_trapezoid_mac_outward_m3,"
            "solid_swept_outward_m3" );
        std::puts( "capsule_step96_donor_balance,controlled_donor_tracer,"
            "fluid_substeps,donor_mass_before_kg,donor_mass_after_last_swept_kg,"
            "donor_mass_after_mac_kg,total_observed_out_kg,"
            "total_observed_in_kg,predicted_final_kg,"
            "actual_minus_predicted_kg,max_swept_stage_defect_kg,"
            "max_mac_stage_defect_kg,max_step_defect_kg,"
            "nonlocal_fallback_out_kg,nonlocal_fallback_in_kg" );
        std::puts( "capsule_step96_donor_route,controlled_donor_tracer,"
            "fluid_substeps,route_WESN_or_X,swept_out_kg,swept_in_kg,"
            "fallback_out_kg,fallback_in_kg,mac_out_kg,mac_in_kg,"
            "connected_out_kg,connected_in_kg,applied_event_count" );
        std::puts( "capsule_step96_donor_volume,controlled_donor_tracer,"
            "fluid_substeps,route_WESN,pre_pressure_endpoint_mac_out_m3,"
            "post_pressure_endpoint_mac_out_m3,solid_swept_out_m3" );
        std::puts( "capsule_step96_split,controlled_donor_tracer,fluid_substeps,"
            "face_open_exposure_s,pre_pressure_endpoint_mac_m3,"
            "post_pressure_endpoint_mac_m3,linear_velocity_aperture_proxy_m3,"
            "solid_swept_face_m3,left_pressure_source_m3,right_pressure_source_m3,"
            "left_snapshot_delta_m3,left_pressure_residual_sum_m3,"
            "actual_swept_pair_signed_kg,actual_fallback_pair_signed_kg,"
            "actual_mac_pair_signed_kg,actual_connected_pair_signed_kg,"
            "total_mass_before_kg,total_mass_after_kg,left_final_tracer_kg,"
            "left_final_fluid_volume_m3,max_raw_face_speed_mps" );
    }
    double maxVolumeDifference = 0.0, maxTracerDifference = 0.0;
    double maxForceDifference = 0.0, maxTorqueDifference = 0.0;
    double maxSnapshotSourceDifference = 0.0;
    for( int step = 0; step < steps; ++step )
    {
        activeStep = step + 1;
        capsule.transform.rotation = rot2::FromRadians( startAngle +
            static_cast<float>( step + 1 ) / steps *
            ( endAngle - startAngle ) );
        std::array<fluidObstacleForce2, 2> feedback{};
        std::array<double, 2> volume{}, tracer{};
        std::array<fluidCutCellVolumeSummary2, 2> budgets{};
        for( int mode = 0; mode < 2; ++mode )
        {
            fluidField2& field = *fields[mode];
            // The reference runs start from the IDENTICAL step-95 entry.
            // Only the copied fields get actual finer Fluid integration or
            // controlled passive tracer, never the authoritative replay.
            std::optional<capsuleStep96SplitResult> singleReference{};
            if( step96SplitFlux && activeStep == hotspotStep && mode == 0 )
            {
                for( int seed = 0; seed < 2; ++seed )
                    for( int split : { 1, 2, 4, 8 } )
                    {
                        const auto result = auditCapsuleStep96SplitFlux(
                            field, capsule, startAngle, endAngle, steps,
                            subDt, split, seed != 0 );
                        if( seed == 0 && split == 1 ) singleReference = result;
                        auditCapsuleStep96Reopening( field, capsule,
                            startAngle, endAngle, steps, subDt,
                            split, seed != 0 );
                    }
            }
            // Only the witness step needs the extra time-quadrature grids.
            if( hotspotLedger && activeStep == hotspotStep )
                field.setSweptApertureAuditEnabled( true );
            // Matched STEP-ENTRY state only: the alt is cloned before any
            // geometry/advection/pressure work on step 96, and differs ONLY
            // by enabling the existing opt-in swept-geometry reconciliation.
            // Never evolve the alternative as a separate multi-step history.
            std::optional<fluidField2> matchedCorrected{};
            std::optional<fluidField2> matchedCorrectedFrozen{};
            if( hotspotLedger && activeStep == hotspotStep && mode == 0 )
            {
                matchedCorrected.emplace( field );
                fluidField2& alternative = *matchedCorrected;
                alternative.setSweptGeometryCorrectionEnabled( true );
                alternative.setSweptApertureAuditEnabled( false );
                alternative.setTracerStageObserver( {} );
                alternative.setTracerTransferObserver( {} );
                alternative.setFaceStageAuditTarget( hotspotX, hotspotY, true );
                alternative.setPrePressureSnapshotObserver(
                    [&]( const fluidField2& snapshot )
                    {
                        if( matchedCorrectedFrozen )
                            throw std::runtime_error( "duplicate corrected frozen snapshot" );
                        matchedCorrectedFrozen.emplace( snapshot );
                    } );
                std::array<fluidObstacleForce2, 1> alternativeForces{};
                alternative.step( subDt,
                    std::span<const fluidShapeObstacle2>( &capsule, 1 ),
                    alternativeForces, 120 );
                if( !matchedCorrectedFrozen )
                    throw std::runtime_error( "missing corrected frozen snapshot" );
            }
            std::array<fluidObstacleForce2, 1> forces{};
            field.step( subDt, std::span<const fluidShapeObstacle2>( &capsule, 1 ),
                forces, 120 );
            feedback[mode] = forces[0];
            if( singleReference )
            {
                const auto actual = field.getCutCellVolumeBudget(
                    leftX, hotspotY, subDt );
                const double actualMass = totalMass( field );
                if( !actual.valid ||
                    std::abs( actual.macOutwardVolume[1] -
                        singleReference->endpointMacVolume ) > 1.0e-8 ||
                    std::abs( actual.pressureSourceDeltaVolume -
                        singleReference->leftPressureSourceVolume ) > 1.0e-8 ||
                    std::abs( actualMass - singleReference->massAfter ) > 1.0e-8 )
                    throw std::runtime_error(
                        "identical step-95 entry split-1 reference differs from live Step" );
            }
            if( hotspotLedger && activeStep == hotspotStep )
            {
                auto& record = hotspot[mode];
                if( !record.frozen || record.stageSamples != 2 )
                    throw std::runtime_error( "missing hotspot snapshot or tracer stages" );
                const auto& before = *record.frozen;
                const auto preLeft = before.getCutCellVolumeBudget(
                    leftX, hotspotY, subDt );
                const auto preRight = before.getCutCellVolumeBudget(
                    hotspotX, hotspotY, subDt );
                const auto postLeft = field.getCutCellVolumeBudget(
                    leftX, hotspotY, subDt );
                const auto postRight = field.getCutCellVolumeBudget(
                    hotspotX, hotspotY, subDt );
                const auto& face = field.getFaceStageAudit();
                if( !preLeft.valid || !preRight.valid ||
                    !postLeft.valid || !postRight.valid ||
                    !face.valid || !face.isU ||
                    face.x != hotspotX || face.y != hotspotY ||
                    !preLeft.hasVolumeHistory || !preLeft.hasSweptHistory ||
                    !field.hasSweptApertureAudit() )
                    throw std::runtime_error( "invalid actual hotspot pressure stage" );

                // Re-project an independent deep copy, keeping the real
                // World/Fluid path and all applied tracer transfers untouched.
                fluidField2 replay = before;
                replay.setPrePressureSnapshotObserver( {} );
                const auto replayFeedback = replay.replayFrozenCutCellPressure();
                if( !replayFeedback )
                    throw std::runtime_error( "hotspot frozen pressure replay refused" );
                const auto replayLeft = replay.getCutCellVolumeBudget(
                    leftX, hotspotY, subDt );
                const double replayFluxError =
                    replayLeft.macOutwardVolume[1] - postLeft.macOutwardVolume[1];
                const double faceVolume = postLeft.macOutwardVolume[1];
                const double open = field.getUFaceOpenFraction(
                    hotspotX, hotspotY );
                const double meanOpen = field.getSweptUTimeMeanOpenFraction(
                    hotspotX, hotspotY );
                const double proxy = field.getSweptULinearVelocityProxyVolume(
                    hotspotX, hotspotY, subDt );
                const double meanOpenCapacity =
                    static_cast<double>( h ) * subDt * meanOpen;
                // Re-evaluate only the selected face with analytic Capsule
                // clipping and progressively finer time midpoint grids.
                // This is an independent time-quadrature control, NOT a
                // measured time history of fluid face velocity.
                const auto capsuleAtAlpha = [&]( double alpha )
                {
                    fluidShapeObstacle2 shape = capsule;
                    const float phase = ( static_cast<float>( hotspotStep - 1 ) +
                        static_cast<float>( alpha ) ) / steps;
                    shape.transform.rotation = rot2::FromRadians(
                        startAngle + phase * ( endAngle - startAngle ) );
                    return shape;
                };
                const auto analyticEnd = field.getAnalyticCapsuleFaceOpenFractionForAudit(
                    capsuleAtAlpha( 1.0 ), true, hotspotX, hotspotY );
                if( !analyticEnd || !std::isfinite( *analyticEnd ) ||
                    std::abs( *analyticEnd - open ) > 1.0e-6 )
                    throw std::runtime_error( "Capsule endpoint analytic aperture mismatch" );
                const std::array<int, 7> sampleCounts{{
                    field.getSweptFaceFlux().timeSamples,
                    16, 64, 256, 1024, 4096, 16384
                }};
                for( const int count : sampleCounts )
                {
                    if( count <= 0 )
                        throw std::runtime_error( "invalid Capsule time quadrature count" );
                    double summedOpen = 0.0, firstOpen = -1.0, lastOpen = -1.0;
                    double alphaMoment = 0.0, peakOpen = 0.0;
                    int nonzero = 0;
                    for( int i = 0; i < count; ++i )
                    {
                        const double alpha = ( i + 0.5 ) / count;
                        const auto sampled = field.getAnalyticCapsuleFaceOpenFractionForAudit(
                            capsuleAtAlpha( alpha ), true, hotspotX, hotspotY );
                        if( !sampled || !std::isfinite( *sampled ) ||
                            *sampled < 0.0 || *sampled > 1.0 )
                            throw std::runtime_error( "nonfinite Capsule temporal aperture" );
                        summedOpen += *sampled;
                        alphaMoment += alpha * *sampled;
                        peakOpen = std::max( peakOpen, *sampled );
                        if( *sampled > 0.0 )
                        {
                            if( nonzero == 0 ) firstOpen = alpha;
                            lastOpen = alpha;
                            ++nonzero;
                        }
                    }
                    const double mean = summedOpen / count;
                    const double moment = alphaMoment / count;
                    std::printf( "capsule_hotspot_temporal,%d,%d,%d,"
                        "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
                        "%.12g,%.12g,%.12g\n",
                        mode, count, nonzero, firstOpen, lastOpen, peakOpen,
                        mean, moment, static_cast<double>( h ) * subDt * mean,
                        meanOpen, *analyticEnd, open );
                }
                const double smallVolume = static_cast<double>(
                    postLeft.currentFraction ) * h * h;
                const double impulseProxy = static_cast<double>( h ) * h * open *
                    ( face.afterPressure - face.afterAdvection );
                const double ratio = smallVolume > 0.0
                    ? std::abs( faceVolume ) / smallVolume : 0.0;
                const std::array<fluidCutCellVolumeBudget2, 2> pre{{
                    preLeft, preRight }};
                const std::array<fluidCutCellVolumeBudget2, 2> post{{
                    postLeft, postRight }};
                for( int side = 0; side < 2; ++side )
                {
                    const auto& a = pre[side];
                    const auto& b = post[side];
                    const int direction = side == 0 ? 1 : 0;
                    std::printf( "capsule_hotspot_cell,%d,%d,"
                        "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
                        "%.12g,%.12g,%.12g,%.12g\n",
                        mode, side, b.currentFraction,
                        static_cast<double>( b.currentFraction ) * h * h,
                        b.snapshotDeltaVolume, b.solidSweptDivergenceVolume,
                        b.pressureSourceDeltaVolume,
                        b.pressureSourceDeltaVolume - b.snapshotDeltaVolume,
                        a.macOutwardVolume[direction],
                        b.macOutwardVolume[direction],
                        // Frozen cutCellVolumeRate_ belongs to the prior solve.
                        // Combine its CURRENT projected source with frozen MAC.
                        a.macNetOutwardVolume + b.pressureSourceDeltaVolume,
                        b.pressureResidualVolume );
                }
                std::printf( "capsule_hotspot_face,%d,"
                    "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
                    "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n",
                    mode, face.afterAdvection, face.afterPressure,
                    face.afterPressure - face.afterAdvection,
                    impulseProxy, open, meanOpen, meanOpenCapacity,
                    faceVolume, proxy, smallVolume, ratio, replayFluxError,
                    field.getCutCellPressure().compatibilityCorrection );
                std::printf( "capsule_hotspot_tracer,%d,"
                    "%.12g,%.12g,%.12g,%.12g,"
                    "%.12g,%.12g,%.12g,%.12g,%d\n",
                    mode, record.signedTransfer[0],
                    record.signedTransfer[1], record.signedTransfer[2],
                    record.signedTransfer[3],
                    record.stageMass[0][0], record.stageMass[0][1],
                    record.stageMass[1][0], record.stageMass[1][1],
                    record.directedTransfers );
                if( !std::isfinite( replayFluxError ) ||
                    !std::isfinite( ratio ) || !std::isfinite( proxy ) ||
                    !std::isfinite( impulseProxy ) )
                    throw std::runtime_error( "nonfinite hotspot frozen ledger" );

                // Reconstruct the actual pre-compatibility RHS by combining
                // the SAME step's frozen pre-pressure MAC and the current
                // post-projection source. The frozen cutCellVolumeRate_ is
                // from the previous solve, so it MUST NOT be read here.
                const auto auditBranch = [&]( int branch,
                    const fluidField2& frozen, const fluidField2& projected )
                {
                    const auto& stage = projected.getFaceStageAudit();
                    const auto& swept = projected.getSweptFaceFlux();
                    const auto& pressure = projected.getCutCellPressure();
                    const auto beforeLeft = frozen.getCutCellVolumeBudget(
                        leftX, hotspotY, subDt );
                    const auto beforeRight = frozen.getCutCellVolumeBudget(
                        hotspotX, hotspotY, subDt );
                    const auto afterLeft = projected.getCutCellVolumeBudget(
                        leftX, hotspotY, subDt );
                    const auto afterRight = projected.getCutCellVolumeBudget(
                        hotspotX, hotspotY, subDt );
                    if( !stage.valid || !stage.hasSweptHistory ||
                        !beforeLeft.valid || !beforeRight.valid ||
                        !afterLeft.valid || !afterRight.valid ||
                        !std::isfinite( stage.rawSweptVolume ) ||
                        !std::isfinite( stage.reconciledSweptVolume ) )
                        throw std::runtime_error( "invalid matched swept-volume audit" );
                    const auto flow = projected.getFlowAudit( subDt );
                    std::printf( "capsule_hotspot_swept,%d,"
                        "%.12g,%.12g,%.12g,%d,%.12g,%.12g,%.12g,"
                        "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
                        "%.12g,%.12g,%.12g,%.12g\n",
                        branch, stage.rawSweptVolume,
                        stage.reconciledSweptVolume,
                        static_cast<double>( stage.reconciledSweptVolume ) -
                            stage.rawSweptVolume,
                        swept.correctedFaces, swept.correctionAbsoluteFlux,
                        swept.rawGeometryMismatch, swept.geometryMismatch,
                        afterLeft.pressureSourceDeltaVolume,
                        afterRight.pressureSourceDeltaVolume,
                        beforeLeft.macNetOutwardVolume,
                        beforeRight.macNetOutwardVolume,
                        afterLeft.macNetOutwardVolume,
                        afterRight.macNetOutwardVolume,
                        afterLeft.pressureResidualVolume,
                        afterRight.pressureResidualVolume,
                        pressure.compatibilityCorrection,
                        flow.maxFluidFaceSpeed );

                    const int width = projected.getWidth();
                    const int height = projected.getHeight();
                    std::vector<int> component( width * height, -1 );
                    std::vector<int> pending;
                    pending.reserve( width * height );
                    std::vector<double> regionRhs, regionMac, regionSource;
                    std::vector<double> regionSnapshot, regionSwept;
                    std::vector<double> regionFloatRhs;
                    std::vector<int> regionSize;
                    double maxCellRhsDiff = 0.0;
                    const auto floatU = [&]( int x, int y )
                    {
                        if( x <= 0 || x >= width || y < 0 || y >= height ||
                            projected.getCellFluidFraction( x - 1, y ) <= 0.0f ||
                            projected.getCellFluidFraction( x, y ) <= 0.0f )
                            return 0.0f;
                        const auto velocity = frozen.getUFaceVelocityForAudit( x, y );
                        if( !velocity ) throw std::runtime_error( "invalid frozen U face" );
                        return projected.getUFaceOpenFraction( x, y ) * *velocity;
                    };
                    const auto floatV = [&]( int x, int y )
                    {
                        if( x < 0 || x >= width || y <= 0 || y >= height ||
                            projected.getCellFluidFraction( x, y - 1 ) <= 0.0f ||
                            projected.getCellFluidFraction( x, y ) <= 0.0f )
                            return 0.0f;
                        const auto velocity = frozen.getVFaceVelocityForAudit( x, y );
                        if( !velocity ) throw std::runtime_error( "invalid frozen V face" );
                        return projected.getVFaceOpenFraction( x, y ) * *velocity;
                    };
                    // The production operator intentionally rounds each
                    // open*velocity product and the signed cell divergence
                    // to float BEFORE accumulating connected regions in
                    // double. Previous diagnostics worked in double m^3.
                    const auto floatRhs = [&]( int x, int y, double sourceVolume )
                    {
                        const float rate = static_cast<float>(
                            sourceVolume / ( static_cast<double>( h ) * h ) );
                        const float inverseCell = 1.0f / h;
                        return ( floatU( x + 1, y ) - floatU( x, y ) +
                            floatV( x, y + 1 ) - floatV( x, y ) ) *
                            inverseCell + rate / subDt;
                    };
                    std::vector<float> floatCellRhs( width * height, 0.0f );
                    for( int y = 0; y < height; ++y )
                        for( int x = 0; x < width; ++x )
                        {
                            const int start = y * width + x;
                            if( component[start] >= 0 ||
                                projected.getCellFluidFraction( x, y ) <= 0.0f )
                                continue;
                            const int region = static_cast<int>( regionRhs.size() );
                            regionRhs.push_back( 0.0 );
                            regionMac.push_back( 0.0 );
                            regionSource.push_back( 0.0 );
                            regionSnapshot.push_back( 0.0 );
                            regionSwept.push_back( 0.0 );
                            regionFloatRhs.push_back( 0.0 );
                            regionSize.push_back( 0 );
                            pending.clear();
                            pending.push_back( start );
                            component[start] = region;
                            for( std::size_t i = 0; i < pending.size(); ++i )
                            {
                                const int cell = pending[i];
                                const int cx = cell % width, cy = cell / width;
                                const auto pre = frozen.getCutCellVolumeBudget(
                                    cx, cy, subDt );
                                const auto post = projected.getCutCellVolumeBudget(
                                    cx, cy, subDt );
                                if( !pre.valid || !post.valid || !post.active )
                                    throw std::runtime_error( "invalid matched region cell" );
                                regionMac[region] += pre.macNetOutwardVolume;
                                regionSource[region] += post.pressureSourceDeltaVolume;
                                regionSnapshot[region] += post.snapshotDeltaVolume;
                                regionSwept[region] += post.solidSweptDivergenceVolume;
                                regionRhs[region] += pre.macNetOutwardVolume +
                                    post.pressureSourceDeltaVolume;
                                const float rhs = floatRhs(
                                    cx, cy, post.pressureSourceDeltaVolume );
                                if( !std::isfinite( rhs ) )
                                    throw std::runtime_error( "nonfinite reconstructed float RHS" );
                                floatCellRhs[cell] = rhs;
                                regionFloatRhs[region] += rhs;
                                maxCellRhsDiff = std::max( maxCellRhsDiff,
                                    std::abs( static_cast<double>( rhs ) -
                                        ( pre.macNetOutwardVolume +
                                            post.pressureSourceDeltaVolume ) /
                                        ( static_cast<double>( h ) * h * subDt ) ) );
                                ++regionSize[region];
                                const auto visit = [&]( int nx, int ny, float open )
                                {
                                    const int neighbor = ny * width + nx;
                                    if( open <= 0.0f || component[neighbor] >= 0 ||
                                        projected.getCellFluidFraction( nx, ny ) <= 0.0f )
                                        return;
                                    component[neighbor] = region;
                                    pending.push_back( neighbor );
                                };
                                if( cx > 0 )
                                    visit( cx - 1, cy,
                                        projected.getUFaceOpenFraction( cx, cy ) );
                                if( cx + 1 < width )
                                    visit( cx + 1, cy,
                                        projected.getUFaceOpenFraction( cx + 1, cy ) );
                                if( cy > 0 )
                                    visit( cx, cy - 1,
                                        projected.getVFaceOpenFraction( cx, cy ) );
                                if( cy + 1 < height )
                                    visit( cx, cy + 1,
                                        projected.getVFaceOpenFraction( cx, cy + 1 ) );
                            }
                        }
                    double reconstructed = std::abs( static_cast<double>(
                        pressure.unresolvedVolumeRate ) );
                    for( const double rhs : regionRhs )
                        reconstructed += std::abs( rhs ) / subDt;
                    double floatCompatibility = std::abs( static_cast<double>(
                        pressure.unresolvedVolumeRate ) );
                    double globalFloatSum = 0.0;
                    for( const double sum : regionFloatRhs )
                        floatCompatibility += std::abs( sum ) * h * h;
                    // Production sumDivergence uses active-cell row-major
                    // traversal, independent of flood-fill queue order.
                    for( const float cell : floatCellRhs ) globalFloatSum += cell;
                    const float sourceImbalance =
                        static_cast<float>( globalFloatSum * h * h ) +
                            pressure.unresolvedVolumeRate;
                    const int region = component[leftCell];
                    if( region < 0 || component[rightCell] != region ||
                        pressure.connectedRegions != static_cast<int>( regionRhs.size() ) ||
                        !std::isfinite( reconstructed ) )
                        throw std::runtime_error( "inconsistent matched pressure regions" );
                    std::printf( "capsule_hotspot_region,%d,%d,%d,"
                        "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
                        "%.12g,%.12g,%.12g\n",
                        branch, region, regionSize[region], regionMac[region],
                        regionSource[region], regionSnapshot[region],
                        regionSwept[region], regionRhs[region],
                        regionRhs[region] / regionSize[region],
                        pressure.compatibilityCorrection, reconstructed,
                        reconstructed - pressure.compatibilityCorrection );
                    std::printf( "capsule_hotspot_float_rhs,%d,%d,%d,"
                        "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
                        "%.12g,%.12g,%.12g,%.12g\n",
                        branch, region, regionSize[region],
                        regionFloatRhs[region],
                        regionFloatRhs[region] / regionSize[region],
                        floatCompatibility, pressure.compatibilityCorrection,
                        floatCompatibility - pressure.compatibilityCorrection,
                        reconstructed - floatCompatibility,
                        sourceImbalance, pressure.sourceImbalance,
                        static_cast<double>( sourceImbalance ) -
                            pressure.sourceImbalance, maxCellRhsDiff );
                    // Match the production float per-face/cell operation
                    // order rather than imposing double-volume arithmetic on
                    // a solver that intentionally stores float divergence.
                    // Both independently reconstructed global invariants
                    // must agree within a small ABSOLUTE physical volume rate.
                    constexpr double toleranceM3ps = 1.0e-10;
                    if( !std::isfinite( floatCompatibility ) ||
                        !std::isfinite( maxCellRhsDiff ) ||
                        std::abs( floatCompatibility -
                            pressure.compatibilityCorrection ) > toleranceM3ps ||
                        std::abs( static_cast<double>( sourceImbalance ) -
                            pressure.sourceImbalance ) > toleranceM3ps )
                        throw std::runtime_error(
                            "matched float-order Cut-cell source compatibility drift" );

                    if( pressureOracle )
                        for( const float threshold : { 0.0f, 1.0e-6f, 1.0e-4f } )
                        {
                            // S2c frozen-only counterfactual. q = dt^2 * pressure;
                            // A(q) is face open fraction * pressure potential
                            // difference in m^3. P^T*A*P is formed by mapping
                            // each tiny shrinking cell to a larger OPEN neighbor.
                            // No prescribed solid feedback or tracer is advanced.
                            const int cells = width * height;
                            const double volumeScale =
                                static_cast<double>( h ) * h * subDt;
                            std::vector<int> root( cells );
                            std::iota( root.begin(), root.end(), 0 );
                            int merged = 0;
                            if( threshold > 0.0f )
                                for( int cell = 0; cell < cells; ++cell )
                                {
                                    if( component[cell] < 0 ) continue;
                                    const int x = cell % width, y = cell / width;
                                    const float current = projected.getCellFluidFraction( x, y );
                                    const float previous = projected.getCutCellVolumeBudget(
                                        x, y, subDt ).previousFraction;
                                    if( !( current > 0.0f && current < threshold &&
                                            previous > current ) ) continue;
                                    float bestOpen = 0.0f;
                                    int best = -1;
                                    const auto consider = [&]( int nx, int ny, float opening )
                                    {
                                        if( opening <= bestOpen ||
                                            projected.getCellFluidFraction( nx, ny ) < threshold )
                                            return;
                                        bestOpen = opening;
                                        best = ny * width + nx;
                                    };
                                    if( x > 0 )
                                        consider( x - 1, y,
                                            projected.getUFaceOpenFraction( x, y ) );
                                    if( x + 1 < width )
                                        consider( x + 1, y,
                                            projected.getUFaceOpenFraction( x + 1, y ) );
                                    if( y > 0 )
                                        consider( x, y - 1,
                                            projected.getVFaceOpenFraction( x, y ) );
                                    if( y + 1 < height )
                                        consider( x, y + 1,
                                            projected.getVFaceOpenFraction( x, y + 1 ) );
                                    if( best >= 0 )
                                    {
                                        if( component[cell] != component[best] )
                                            throw std::runtime_error( "pressure pool crossed closed region" );
                                        root[cell] = best;
                                        ++merged;
                                    }
                                }
                            std::vector<double> originalB( cells, 0.0 );
                            std::vector<double> b( cells, 0.0 ), diagonal( cells, 0.0 );
                            double sourceBefore = 0.0, sourceAfter = 0.0;
                            int activeRoots = 0;
                            for( int cell = 0; cell < cells; ++cell )
                            {
                                const int region = component[cell];
                                if( region < 0 ) continue;
                                // Reproduce projectCutCell's float region-mean
                                // subtraction, then convert to pressure RHS.
                                const float mean = static_cast<float>(
                                    regionFloatRhs[region] / regionSize[region] );
                                const float compatible = floatCellRhs[cell] - mean;
                                originalB[cell] = -static_cast<double>( compatible ) * volumeScale;
                                sourceBefore += originalB[cell];
                                b[root[cell]] += originalB[cell];
                                if( root[cell] == cell ) ++activeRoots;
                            }
                            for( int cell = 0; cell < cells; ++cell )
                                if( component[cell] >= 0 && root[cell] == cell )
                                    sourceAfter += b[cell];
                            if( std::abs( sourceAfter - sourceBefore ) > 1.0e-11 ||
                                activeRoots != pressure.activeCells - merged )
                                throw std::runtime_error( "Galerkin source restriction lost cell volume" );

                            struct faceEdge
                            {
                                int a, b;
                                double opening;
                                bool isU;
                                int x, y;
                            };
                            std::vector<faceEdge> edges;
                            edges.reserve( static_cast<std::size_t>( cells ) * 2 );
                            const auto addEdge = [&]( int a, int d, float opening,
                                bool isU, int x, int y )
                            {
                                if( opening <= 0.0f ) return;
                                // Public getU/VFaceOpenFraction reports raw geometric
                                // aperture; projectCutCell uses cutCellU/VOpen,
                                // which masks the edge unless BOTH cells are active.
                                if( component[a] < 0 || component[d] < 0 ) return;
                                edges.push_back( { a, d, opening, isU, x, y } );
                                if( root[a] != root[d] )
                                {
                                    diagonal[root[a]] += opening;
                                    diagonal[root[d]] += opening;
                                }
                            };
                            for( int y = 0; y < height; ++y )
                                for( int x = 1; x < width; ++x )
                                    addEdge( y * width + x - 1, y * width + x,
                                        projected.getUFaceOpenFraction( x, y ), true, x, y );
                            for( int y = 1; y < height; ++y )
                                for( int x = 0; x < width; ++x )
                                    addEdge( ( y - 1 ) * width + x, y * width + x,
                                        projected.getVFaceOpenFraction( x, y ), false, x, y );

                            // One Neumann gauge per connected component. Restrict
                            // and solve on the retained pressure root nodes.
                            const auto removeGauge = [&]( std::vector<double>& v )
                            {
                                for( std::size_t region = 0; region < regionSize.size(); ++region )
                                {
                                    double sum = 0.0;
                                    int roots = 0;
                                    for( int cell = 0; cell < cells; ++cell )
                                        if( component[cell] == static_cast<int>( region ) &&
                                            root[cell] == cell )
                                        {
                                            sum += v[cell];
                                            ++roots;
                                        }
                                    if( roots == 0 )
                                        throw std::runtime_error( "empty Galerkin pressure group" );
                                    const double mean = sum / roots;
                                    for( int cell = 0; cell < cells; ++cell )
                                        if( component[cell] == static_cast<int>( region ) &&
                                            root[cell] == cell )
                                            v[cell] -= mean;
                                }
                            };
                            std::vector<double> correctedB = b;
                            removeGauge( correctedB );
                            double gaugeAdjustment = 0.0;
                            for( int cell = 0; cell < cells; ++cell )
                                if( component[cell] >= 0 && root[cell] == cell )
                                    gaugeAdjustment += std::abs( correctedB[cell] - b[cell] );
                            const auto multiply = [&]( const std::vector<double>& src,
                                std::vector<double>& out )
                            {
                                std::fill( out.begin(), out.end(), 0.0 );
                                for( const faceEdge& edge : edges )
                                {
                                    const int a = root[edge.a], d = root[edge.b];
                                    if( a == d ) continue;
                                    const double flux = edge.opening * ( src[a] - src[d] );
                                    out[a] += flux;
                                    out[d] -= flux;
                                }
                            };
                            const auto dot = [&]( const std::vector<double>& a,
                                const std::vector<double>& d )
                            {
                                double result = 0.0;
                                for( int cell = 0; cell < cells; ++cell )
                                    if( component[cell] >= 0 && root[cell] == cell )
                                        result += a[cell] * d[cell];
                                return result;
                            };
                            const auto rms = [&]( const std::vector<double>& v )
                            {
                                return std::sqrt( dot( v, v ) / activeRoots ) / volumeScale;
                            };
                            std::vector<double> q( cells, 0.0 ), residual = correctedB;
                            std::vector<double> z( cells, 0.0 ), direction( cells, 0.0 );
                            std::vector<double> product( cells, 0.0 );
                            const double initialRms = rms( residual );
                            constexpr double targetRmsPerSecond = 1.0e-4;
                            constexpr int maxIterations = 3200;
                            int iterations = 0;
                            if( initialRms > targetRmsPerSecond )
                            {
                                for( int cell = 0; cell < cells; ++cell )
                                    if( component[cell] >= 0 && root[cell] == cell &&
                                        diagonal[cell] > 0.0 )
                                        z[cell] = residual[cell] / diagonal[cell];
                                removeGauge( z );
                                direction = z;
                                double rz = dot( residual, z );
                                for( int iteration = 1; iteration <= maxIterations; ++iteration )
                                {
                                    multiply( direction, product );
                                    const double denominator = dot( direction, product );
                                    if( !( denominator > 0.0 ) || !( rz > 0.0 ) ||
                                        !std::isfinite( denominator ) || !std::isfinite( rz ) )
                                        break;
                                    const double alpha = rz / denominator;
                                    for( int cell = 0; cell < cells; ++cell )
                                        if( component[cell] >= 0 && root[cell] == cell )
                                        {
                                            q[cell] += alpha * direction[cell];
                                            residual[cell] -= alpha * product[cell];
                                        }
                                    removeGauge( residual );
                                    iterations = iteration;
                                    if( rms( residual ) <= targetRmsPerSecond ) break;
                                    for( int cell = 0; cell < cells; ++cell )
                                        z[cell] = component[cell] >= 0 && root[cell] == cell &&
                                            diagonal[cell] > 0.0
                                            ? residual[cell] / diagonal[cell] : 0.0;
                                    removeGauge( z );
                                    const double nextRz = dot( residual, z );
                                    if( !( nextRz > 0.0 ) || !std::isfinite( nextRz ) )
                                        break;
                                    const double beta = nextRz / rz;
                                    for( int cell = 0; cell < cells; ++cell )
                                        if( component[cell] >= 0 && root[cell] == cell )
                                            direction[cell] = z[cell] + beta * direction[cell];
                                    removeGauge( direction );
                                    rz = nextRz;
                                }
                            }
                            // Rebuild the TRUE grouped and per-cell operator
                            // residual, independent of the CG recurrence.
                            multiply( q, product );
                            double squaredGroup = 0.0;
                            for( int cell = 0; cell < cells; ++cell )
                                if( component[cell] >= 0 && root[cell] == cell )
                                {
                                    const double err = product[cell] - correctedB[cell];
                                    squaredGroup += err * err;
                                }
                            const double groupRms = std::sqrt(
                                squaredGroup / activeRoots ) / volumeScale;
                            std::vector<double> cellAx( cells, 0.0 );
                            double maxSpeed = 0.0, maxWeighted = 0.0;
                            double targetSpeed = 0.0, impulseX = 0.0, impulseY = 0.0;
                            for( const faceEdge& edge : edges )
                            {
                                const double volume = edge.opening *
                                    ( q[root[edge.a]] - q[root[edge.b]] );
                                cellAx[edge.a] += volume;
                                cellAx[edge.b] -= volume;
                                const auto speed = edge.isU
                                    ? frozen.getUFaceVelocityForAudit( edge.x, edge.y )
                                    : frozen.getVFaceVelocityForAudit( edge.x, edge.y );
                                if( !speed )
                                    throw std::runtime_error( "missing oracle frozen MAC input" );
                                const double delta = volume /
                                    ( edge.opening * h * subDt );
                                const double after = *speed + delta;
                                maxSpeed = std::max( maxSpeed, std::abs( after ) );
                                maxWeighted = std::max( maxWeighted,
                                    edge.opening * std::abs( after ) );
                                const double impulse = static_cast<double>( h ) * h *
                                    edge.opening * delta;
                                if( edge.isU )
                                {
                                    impulseX += impulse;
                                    if( edge.x == hotspotX && edge.y == hotspotY )
                                        targetSpeed = after;
                                }
                                else impulseY += impulse;
                            }
                            double squaredPointwise = 0.0, virtualL1 = 0.0;
                            for( int cell = 0; cell < cells; ++cell )
                            {
                                if( component[cell] < 0 ) continue;
                                const double err = cellAx[cell] - originalB[cell];
                                squaredPointwise += err * err;
                                if( root[cell] != cell ) virtualL1 += std::abs( err );
                            }
                            const double pointwiseRms = std::sqrt(
                                squaredPointwise / pressure.activeCells ) / volumeScale;
                            const bool converged = std::isfinite( groupRms ) &&
                                groupRms < targetRmsPerSecond;
                            if( !std::isfinite( maxSpeed ) || !std::isfinite( pointwiseRms ) ||
                                !std::isfinite( impulseX ) || !std::isfinite( impulseY ) ||
                                !std::isfinite( virtualL1 ) || !std::isfinite( groupRms ) )
                                throw std::runtime_error( "nonfinite independent Galerkin oracle" );
                            std::printf( "capsule_hotspot_pressure_oracle,%d,%.9g,%d,%d,%d,"
                                "%.12g,%.12g,%.12g,%d,%d,%.12g,%.12g,"
                                "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n",
                                branch, threshold, merged, activeRoots,
                                root[leftCell] == root[rightCell] ? 1 : 0,
                                sourceBefore, sourceAfter, gaugeAdjustment,
                                iterations, converged ? 1 : 0, initialRms,
                                groupRms, pointwiseRms, targetSpeed, maxSpeed,
                                maxWeighted, impulseX, impulseY, virtualL1 );
                            // Pressure grouping removed individual continuity
                            // rows. These are COUNTERFACTUAL internal face
                            // exchanges, not applied tracer/solid-fluid flux.
                            // The signed child deficit is booked equally and
                            // oppositely on its chosen parent. Do not count
                            // an already-swept donor volume a second time.
                            std::vector<double> donorRemainingVolume( cells, 0.0 );
                            std::vector<double> donorRemainingMass( cells, 0.0 );
                            std::vector<double> scratchVolumeDelta( cells, 0.0 );
                            std::vector<double> scratchMassDelta( cells, 0.0 );
                            std::vector<double> requiredPairBalance( cells, 0.0 );
                            double requestedAbsolute = 0.0, acceptedAbsolute = 0.0;
                            double signedPairVolume = 0.0, signedPairMass = 0.0;
                            for( int cell = 0; cell < cells; ++cell )
                            {
                                if( component[cell] < 0 ) continue;
                                const int x = cell % width, y = cell / width;
                                const double volume = static_cast<double>(
                                    projected.getCellFluidFraction( x, y ) ) * h * h;
                                const double mass = projected.getTracerCellMass( x, y );
                                if( !std::isfinite( volume ) || volume < 0.0 ||
                                    !std::isfinite( mass ) || mass < 0.0 )
                                    throw std::runtime_error( "invalid pooled donor inventory" );
                                donorRemainingVolume[cell] = volume;
                                donorRemainingMass[cell] = mass;
                            }
                            for( int child = 0; child < cells; ++child )
                            {
                                if( component[child] < 0 || root[child] == child ) continue;
                                const int parent = root[child];
                                const int cx = child % width, cy = child / width;
                                const int px = parent % width, py = parent / width;
                                if( std::abs( px - cx ) + std::abs( py - cy ) != 1 )
                                    throw std::runtime_error( "non-neighbor Galerkin child" );
                                const bool isU = py == cy;
                                const int fx = isU ? std::max( cx, px ) : cx;
                                const int fy = isU ? cy : std::max( cy, py );
                                const double endpointOpen = isU
                                    ? projected.getUFaceOpenFraction( fx, fy )
                                    : projected.getVFaceOpenFraction( fx, fy );
                                if( endpointOpen <= 0.0 )
                                    throw std::runtime_error( "pooled transfer crossed closed face" );
                                // A(child)+internalOutward(child) = b(child).
                                // Thus the required CHILD-outward signed
                                // amount is b(child)-A(child), not the total
                                // swept face volume or the endpoint MAC flux.
                                const double requiredChildOutward =
                                    originalB[child] - cellAx[child];
                                requiredPairBalance[child] += requiredChildOutward;
                                requiredPairBalance[parent] -= requiredChildOutward;
                                requestedAbsolute += std::abs( requiredChildOutward );
                                const int donor = requiredChildOutward >= 0.0 ? child : parent;
                                const int recipient = donor == child ? parent : child;
                                const int dx = donor % width, dy = donor / width;
                                const double donorCurrentVolume = static_cast<double>(
                                    projected.getCellFluidFraction( dx, dy ) ) * h * h;
                                const double donorPreviousVolume =
                                    projected.getCutCellVolumeBudget( dx, dy, subDt )
                                        .previousFraction * h * h;
                                const double donorCurrentMass =
                                    projected.getTracerCellMass( dx, dy );
                                // Time aperture is an analytic Capsule sample,
                                // on THIS moving physical face; the stored
                                // endpoint open fraction is not substituted.
                                constexpr int timeSamples = 256;
                                double openSum = 0.0;
                                for( int sample = 0; sample < timeSamples; ++sample )
                                {
                                    const auto opening =
                                        projected.getAnalyticCapsuleFaceOpenFractionForAudit(
                                            capsuleAtAlpha( ( sample + 0.5 ) / timeSamples ),
                                            isU, fx, fy );
                                    if( !opening || !std::isfinite( *opening ) ||
                                        *opening < 0.0 || *opening > 1.0 )
                                        throw std::runtime_error(
                                            "invalid moving pooled-face time aperture" );
                                    openSum += *opening;
                                }
                                const double meanOpen = openSum / timeSamples;
                                const auto startOpen =
                                    projected.getAnalyticCapsuleFaceOpenFractionForAudit(
                                        capsuleAtAlpha( 0.0 ), isU, fx, fy );
                                const auto faceVelocity = isU
                                    ? frozen.getUFaceVelocityForAudit( fx, fy )
                                    : frozen.getVFaceVelocityForAudit( fx, fy );
                                if( !startOpen || !faceVelocity ||
                                    !std::isfinite( *faceVelocity ) ||
                                    !std::isfinite( donorPreviousVolume ) )
                                    throw std::runtime_error(
                                        "invalid moving pooled-face reference" );
                                const double needed = std::abs( requiredChildOutward );
                                const double timeCapacityPerSpeed = h * subDt * meanOpen;
                                const double endpointCapacityPerSpeed = h * subDt * endpointOpen;
                                const double neededMeanSpeed = timeCapacityPerSpeed > 0.0
                                    ? needed / timeCapacityPerSpeed : 0.0;
                                const double neededEndpointSpeed = endpointCapacityPerSpeed > 0.0
                                    ? needed / endpointCapacityPerSpeed : 0.0;
                                // This time-open velocity CAPACITY is only a
                                // diagnostic reference at PRE-pressure speed:
                                // the real within-step MAC history is unknown.
                                // It is not a physically mandated speed cap.
                                const double referenceCapacity =
                                    timeCapacityPerSpeed * std::abs( *faceVelocity );
                                const double remainingVolume = donorRemainingVolume[donor];
                                const double remainingMass = donorRemainingMass[donor];
                                const double acceptedVolume = std::min(
                                    { needed, referenceCapacity, remainingVolume } );
                                const double concentration = remainingVolume > 0.0
                                    ? remainingMass / remainingVolume : 0.0;
                                const double acceptedMass = std::min(
                                    remainingMass, acceptedVolume * concentration );
                                donorRemainingVolume[donor] -= acceptedVolume;
                                donorRemainingMass[donor] -= acceptedMass;
                                scratchVolumeDelta[donor] -= acceptedVolume;
                                scratchVolumeDelta[recipient] += acceptedVolume;
                                scratchMassDelta[donor] -= acceptedMass;
                                scratchMassDelta[recipient] += acceptedMass;
                                acceptedAbsolute += acceptedVolume;
                                // Book both sides, even though we DO NOT
                                // mutate either cell of the actual Field.
                                const double childSigned = donor == child
                                    ? acceptedVolume : -acceptedVolume;
                                const double parentSigned = -childSigned;
                                const double massSigned = donor == child
                                    ? acceptedMass : -acceptedMass;
                                signedPairVolume += childSigned + parentSigned;
                                signedPairMass += massSigned - massSigned;
                                const double rootResidual =
                                    product[parent] - correctedB[parent];
                                if( !std::isfinite( neededMeanSpeed ) ||
                                    !std::isfinite( neededEndpointSpeed ) ||
                                    !std::isfinite( acceptedVolume ) ||
                                    acceptedVolume < 0.0 ||
                                    acceptedVolume > remainingVolume + 1.0e-12 ||
                                    acceptedMass < 0.0 ||
                                    acceptedMass > remainingMass + 1.0e-12 )
                                    throw std::runtime_error(
                                        "invalid donor-bounded pooled transfer" );
                                std::printf(
                                    "capsule_hotspot_internal_transfer,%d,%.9g,"
                                    "%d,%d,%c,%d,%d,"
                                    "%.12g,%d,%d,%.12g,%.12g,%.12g,%.12g,"
                                    "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
                                    "%.12g,%.12g,%.12g,%.12g,%.12g,"
                                    "%.12g,%.12g\n",
                                    branch, threshold, child, parent,
                                    isU ? 'U' : 'V', fx, fy,
                                    requiredChildOutward, donor, recipient,
                                    *startOpen, endpointOpen, meanOpen,
                                    *faceVelocity, neededMeanSpeed, neededEndpointSpeed,
                                    referenceCapacity, donorPreviousVolume,
                                    donorCurrentVolume, donorCurrentMass,
                                    acceptedVolume, acceptedMass,
                                    needed - acceptedVolume,
                                    childSigned + parentSigned,
                                    massSigned - massSigned,
                                    cellAx[child] - originalB[child],
                                    rootResidual );
                            }
                            double requiredL1 = 0.0;
                            double inventoryDeltaVolume = 0.0, inventoryDeltaMass = 0.0;
                            for( int cell = 0; cell < cells; ++cell )
                            {
                                if( component[cell] < 0 ) continue;
                                if( root[cell] != cell )
                                    requiredL1 += std::abs( requiredPairBalance[cell] );
                                inventoryDeltaVolume += scratchVolumeDelta[cell];
                                inventoryDeltaMass += scratchMassDelta[cell];
                                const int x = cell % width, y = cell / width;
                                const double initialVolume = static_cast<double>(
                                    projected.getCellFluidFraction( x, y ) ) * h * h;
                                const double initialMass =
                                    projected.getTracerCellMass( x, y );
                                if( initialVolume + scratchVolumeDelta[cell] < -1.0e-12 ||
                                    initialMass + scratchMassDelta[cell] < -1.0e-12 ||
                                    donorRemainingVolume[cell] < -1.0e-12 ||
                                    donorRemainingMass[cell] < -1.0e-12 )
                                    throw std::runtime_error(
                                        "hypothetical pooled transfer overdrawn inventory" );
                            }
                            if( std::abs( requiredL1 - virtualL1 ) > 1.0e-8 ||
                                std::abs( inventoryDeltaVolume ) > 1.0e-12 ||
                                std::abs( inventoryDeltaMass ) > 1.0e-12 ||
                                std::abs( signedPairVolume ) > 1.0e-12 ||
                                std::abs( signedPairMass ) > 1.0e-12 ||
                                acceptedAbsolute > requestedAbsolute + 1.0e-12 )
                                throw std::runtime_error(
                                    "pooled virtual transfer failed paired conservation audit" );
                        }
                };
                if( mode == 0 )
                {
                    if( !matchedCorrected || !matchedCorrectedFrozen )
                        throw std::runtime_error( "missing matched correction control" );
                    const auto& originalFace = field.getFaceStageAudit();
                    const auto& alternateFace = matchedCorrected->getFaceStageAudit();
                    if( !originalFace.hasSweptHistory ||
                        !alternateFace.hasSweptHistory ||
                        std::abs( static_cast<double>( originalFace.rawSweptVolume ) -
                            alternateFace.rawSweptVolume ) > 1.0e-8 ||
                        field.getSweptFaceFlux().correctedFaces != 0 )
                        throw std::runtime_error(
                            "matched histories did not preserve raw swept input" );
                    auditBranch( 0, before, field );
                    auditBranch( 1, *matchedCorrectedFrozen, *matchedCorrected );
                }
                record.frozen.reset();
                field.setSweptApertureAuditEnabled( false );
            }
            const auto summary = field.getCutCellVolumeSummary( subDt );
            const auto flow = field.getFlowAudit( subDt );
            const auto swept = field.getSweptFaceFlux();
            const auto tracerStats = field.getTracerStats();
            if( !summary.valid || !std::isfinite( flow.maxFluidFaceSpeed ) ||
                !std::isfinite( swept.geometryMismatch ) ||
                !std::isfinite( feedback[mode].force.x ) ||
                !std::isfinite( feedback[mode].force.y ) ||
                !std::isfinite( feedback[mode].torque ) )
                throw std::runtime_error( "nonfinite moving Capsule volume/pressure/force ledger" );
            budgets[mode] = summary;
            if( flow.maxFluidFaceSpeed > 1000.0f )
            {
                const int fx = flow.maxFaceX, fy = flow.maxFaceY;
                const int lx = flow.maxFaceIsU ? fx - 1 : fx;
                const int ly = flow.maxFaceIsU ? fy : fy - 1;
                const int ux = fx, uy = fy;
                if( fx < 0 || fy < 0 ||
                    lx < 0 || ly < 0 || ux >= field.getWidth() ||
                    uy >= field.getHeight() )
                    throw std::runtime_error( "invalid extreme Capsule MAC face" );
                const auto lower = field.getCutCellVolumeBudget( lx, ly, subDt );
                const auto upper = field.getCutCellVolumeBudget( ux, uy, subDt );
                if( !lower.valid || !upper.valid )
                    throw std::runtime_error( "invalid extreme Capsule face ledger" );
                const double nominalFlux = static_cast<double>( h ) * subDt *
                    flow.maxFaceOpenFraction * flow.maxFaceVelocity;
                std::printf( "capsule_moving_velocity_hotspot,%d,%d,%c,%d,%d,"
                    "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
                    "%.12g,%.12g,%.12g,%.12g\n",
                    step + 1, mode, flow.maxFaceIsU ? 'U' : 'V',
                    fx, fy, flow.maxFaceVelocity, flow.maxFaceOpenFraction,
                    flow.maxFaceOpenFraction * flow.maxFluidFaceSpeed,
                    nominalFlux, lower.currentFraction, upper.currentFraction,
                    lower.pressureSourceDeltaVolume,
                    upper.pressureSourceDeltaVolume,
                    lower.pressureResidualVolume, upper.pressureResidualVolume );
            }
            tracer[mode] = totalMass( field );
            double actualVolume = 0.0;
            for( int y = 0; y < field.getHeight(); ++y )
                for( int x = 0; x < field.getWidth(); ++x )
                    actualVolume += field.getCellFluidFraction( x, y ) * h * h;
            volume[mode] = actualVolume;
            auto& ledger = ledgers[mode];
            ledger.maxStepMassDefect = std::max( ledger.maxStepMassDefect,
                std::abs( tracer[mode] - seededMass ) );
            ledger.maxSweptMismatch = std::max( ledger.maxSweptMismatch,
                static_cast<double>( swept.geometryMismatch ) );
            ledger.peakForce = std::max( ledger.peakForce,
                static_cast<double>( Length( feedback[mode].force ) ) );
            ledger.peakTorque = std::max( ledger.peakTorque,
                std::abs( static_cast<double>( feedback[mode].torque ) ) );
            ledger.sumRemappedMass += tracerStats.sweptRemappedMass;
            ledger.sumFallbackMass += tracerStats.fallbackRemappedMass;
            ledger.sumUnresolvedMass += tracerStats.unresolvedDisplacementMass;
            if( step % 8 == 7 || step == steps - 1 )
                std::printf( "capsule_moving_transport,%d,%d,%d,"
                    "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
                    "%.12g,%.12g,%.12g,%.12g,"
                    "%.12g,%.12g,%.12g,%d,%.12g\n",
                    step + 1, ( step + 1 ) / 8, mode,
                    volume[mode], summary.totalSnapshotDeltaVolume,
                    summary.totalPressureSourceDeltaVolume,
                    swept.geometryMismatch, tracer[mode],
                    tracerStats.trappedMass, tracerStats.sweptRemappedMass,
                    tracerStats.fallbackRemappedMass,
                    tracerStats.unresolvedDisplacementMass,
                    flow.maxFluidFaceSpeed,
                    feedback[mode].force.x, feedback[mode].force.y,
                    feedback[mode].torque, summary.activeCells,
                    summary.activePressureResidualRmsVolume );
        }
        maxVolumeDifference = std::max( maxVolumeDifference,
            std::abs( volume[0] - volume[1] ) );
        maxTracerDifference = std::max( maxTracerDifference,
            std::abs( tracer[0] - tracer[1] ) );
        maxForceDifference = std::max( maxForceDifference,
            static_cast<double>( Length( feedback[0].force -
                feedback[1].force ) ) );
        maxTorqueDifference = std::max( maxTorqueDifference,
            std::abs( static_cast<double>(
                feedback[0].torque - feedback[1].torque ) ) );
        maxSnapshotSourceDifference = std::max( maxSnapshotSourceDifference,
            std::abs( budgets[0].totalPressureSourceDeltaVolume -
                budgets[1].totalPressureSourceDeltaVolume ) );
    }
    for( int mode = 0; mode < 2; ++mode )
    {
        fields[mode]->setTracerStageObserver( {} );
        if( hotspotLedger )
        {
            fields[mode]->setPrePressureSnapshotObserver( {} );
            fields[mode]->setTracerTransferObserver( {} );
        }
    }

    std::puts( "capsule_moving_summary,seeded_cells,seeded_mass_kg,"
        "guard0_swept_stages,guard1_swept_stages,"
        "guard0_mac_stages,guard1_mac_stages,"
        "max_stage_mass_defect_kg,max_step_mass_defect_kg,"
        "max_volume_difference_m3,max_tracer_difference_kg,"
        "max_force_difference_N,max_torque_difference_Nm,"
        "max_pressure_source_difference_m3,"
        "guard0_peak_force_N,guard1_peak_force_N,"
        "guard0_swept_mismatch_m3,guard1_swept_mismatch_m3,"
        "guard0_remapped_kg,guard1_remapped_kg,"
        "guard0_unresolved_kg,guard1_unresolved_kg" );
    std::printf( "capsule_moving_summary,%d,%.12g,%d,%d,%d,%d,"
        "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
        "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n",
        seeded, seededMass,
        ledgers[0].sweptStages, ledgers[1].sweptStages,
        ledgers[0].macStages, ledgers[1].macStages,
        std::max( ledgers[0].maxStageMassDefect,
            ledgers[1].maxStageMassDefect ),
        std::max( ledgers[0].maxStepMassDefect,
            ledgers[1].maxStepMassDefect ),
        maxVolumeDifference, maxTracerDifference,
        maxForceDifference, maxTorqueDifference,
        maxSnapshotSourceDifference,
        ledgers[0].peakForce, ledgers[1].peakForce,
        ledgers[0].maxSweptMismatch, ledgers[1].maxSweptMismatch,
        ledgers[0].sumRemappedMass, ledgers[1].sumRemappedMass,
        ledgers[0].sumUnresolvedMass, ledgers[1].sumUnresolvedMass );

    // Tracer transfers are conservative whether or not the guard is enabled;
    // the two pressure histories may still diverge, so force differences
    // remain measurements rather than a fabricated zero-difference target.
    const double absoluteMassBudget = 1.0e-6 * std::max( 1.0, seededMass );
    for( const transportLedger& ledger : ledgers )
        if( ledger.macStages != steps || ledger.sweptStages != steps ||
            !std::isfinite( ledger.maxStepMassDefect ) ||
            !std::isfinite( ledger.maxStageMassDefect ) ||
            ledger.maxStageMassDefect > absoluteMassBudget ||
            ledger.maxStepMassDefect > absoluteMassBudget )
            throw std::runtime_error( "moving Capsule stage-level tracer conservation regression" );
    if( !std::isfinite( maxVolumeDifference ) ||
        !std::isfinite( maxForceDifference ) ||
        !std::isfinite( maxTorqueDifference ) ||
        !std::isfinite( maxSnapshotSourceDifference ) )
        throw std::runtime_error( "nonfinite moving Capsule guard counterfactual" );
}

// Read-only controlled reproduction for the Wind Sandbox screenshot:
// a horizontal Capsule never physically intersects the bottom wall, yet
// updateSweptSolidFlux() rejects its conservative enclosing *radius* once
// the center approaches that wall. The normal user Wind source is replayed
// independently ON and OFF. This is not a coupled Body/World proof.
void auditWindBoundaryPressureSpike()
{
    constexpr int width = 64, height = 64;
    constexpr float h = 0.125f, stepDt = 1.0f / 60.0f;
    constexpr vec2 origin{ -4.0f, -4.0f };
    constexpr float radius = 0.19f, halfLength = 0.63f;
    constexpr float boundRadius = halfLength + radius;
    constexpr float startY = -3.02f, lowerY = -3.55f;
    constexpr int halfSteps = 16, totalSteps = 2 * halfSteps;
    constexpr vec2 source{ -2.0f, 0.0f };
    constexpr float sourceStrength = 6.0f;
    const auto fail = []( const char* message )
    {
        throw std::runtime_error( message );
    };

    std::puts( "wind_boundary_pressure_step,source_on,step,pose_y_m,"
        "conservative_bottom_clearance_m,actual_capsule_bottom_clearance_m,"
        "expected_swept,used_swept,has_volume_history,swept_time_samples,"
        "snapshot_volume_rate_m3ps,source_imbalance_m3ps,"
        "compatibility_rate_m3ps,pressure_residual_native,"
        "continuity_before_per_s,continuity_after_per_s,"
        "pre_max_open_face_mps,post_max_open_face_mps,"
        "max_pressure_stage_delta_mps,max_delta_face_u,"
        "max_delta_face_x,max_delta_face_y,max_delta_face_open,"
        "pressure_min_relative,pressure_max_relative,pressure_peak_relative,"
        "pressure_peak_x,pressure_peak_y,"
        "max_endpoint_continuity_volume_m3,worst_cell_x,worst_cell_y,"
        "total_pressure_source_volume_m3,total_snapshot_delta_m3,"
        "total_mac_endpoint_volume_m3,swept_snapshot_mismatch_l1_m3,"
        "force_x_N,force_y_N,torque_Nm" );
    std::puts( "wind_boundary_pressure_counterfactual,source_on,step,"
        "default_swept,tight_swept,"
        "default_snapshot_delta_m3,tight_snapshot_delta_m3,"
        "default_source_delta_m3,tight_source_delta_m3,"
        "default_pressure_span,tight_pressure_span,"
        "default_max_open_face_mps,tight_max_open_face_mps,"
        "default_max_pressure_residual_m3,tight_max_pressure_residual_m3,"
        "default_compatibility_rate_m3ps,tight_compatibility_rate_m3ps,"
        "default_force_y_N,tight_force_y_N,"
        "tight_pre_pressure_max_open_face_mps" );
    std::puts( "wind_boundary_pressure_cell_hotspot,source_on,step,"
        "role,cell_x,cell_y,previous_fraction,current_fraction,"
        "snapshot_delta_m3,pressure_source_m3,swept_divergence_m3,"
        "post_mac_net_m3,pressure_residual_m3,relative_pressure,"
        "open_w,open_e,open_s,open_n" );
    std::puts( "wind_boundary_pressure_face_hotspot,source_on,step,"
        "face_UV,face_x,face_y,face_open,pre_pressure_speed_mps,"
        "post_pressure_speed_mps,first_fraction,second_fraction,"
        "first_pressure_residual_m3,second_pressure_residual_m3" );
    std::puts( "wind_boundary_factorial,coverage_guard,tight_swept,step,"
        "used_swept,phantom_closed_active_cells,phantom_source_m3,"
        "pressure_span,peak_open_face_speed_mps,peak_cell_residual_m3,"
        "compatibility_rate_m3ps,snapshot_delta_m3,"
        "pressure_source_delta_m3,force_y_N" );
    std::puts( "wind_boundary_capsule_guard,coverage_guard,step,"
        "used_swept,phantom_active_cells,phantom_source_m3,"
        "max_pressure_residual_m3,compatibility_rate_m3ps,"
        "pressure_span,maximum_open_face_speed_mps,"
        "snapshot_volume_delta_m3,total_pressure_source_m3,"
        "capsule_force_y_N,legacy_pressure_residual_native" );
    std::puts( "wind_boundary_time_face,coverage_guard,step,face_WESN,"
        "outward_sign,start_analytic_open,end_analytic_open,"
        "stored_endpoint_open,swept_internal_mean_open,"
        "analytic64_mean_open,analytic256_mean_open,"
        "step_entry_speed_mps,pre_pressure_speed_mps,"
        "post_pressure_speed_mps,pre_endpoint_outward_m3,"
        "post_endpoint_outward_m3,linear_velocity_proxy_outward_m3,"
        "time_mean_open_fixed_post_speed_outward_m3,"
        "pressure_stage_outward_delta_m3" );
    std::puts( "wind_boundary_time_balance,coverage_guard,step,"
        "donor_x,donor_y,donor_fraction,"
        "donor_snapshot_delta_m3,donor_swept_solid_divergence_m3,"
        "donor_pressure_source_m3,donor_post_mac_outward_m3,"
        "donor_pressure_residual_m3,"
        "four_face_pre_endpoint_outward_m3,"
        "four_face_post_endpoint_outward_m3,"
        "four_face_linear_velocity_proxy_outward_m3,"
        "four_face_fixed_post_time_open_outward_m3,"
        "linear_proxy_plus_source_m3,"
        "endpoint_minus_proxy_m3,"
        "snapshot_minus_swept_gcl_m3,"
        "obstacle_force_y_N,obstacle_torque_Nm" );
    std::puts( "wind_boundary_refined_step,coverage_guard,splits,part,"
        "alpha_end,pose_y_m,donor_fraction,donor_snapshot_m3,"
        "donor_swept_solid_m3,donor_pressure_source_m3,"
        "donor_pre_endpoint_mac_m3,donor_post_endpoint_mac_m3,"
        "donor_time_mean_fixed_post_mac_m3,"
        "donor_time_linear_velocity_proxy_m3,"
        "donor_pressure_residual_m3,"
        "donor_gcl_mismatch_m3,force_y_N,torque_Nm,"
        "tracer_mass_kg,donor_tracer_mass_kg" );
    std::puts( "wind_boundary_refined_summary,coverage_guard,splits,"
        "donor_start_mass_kg,donor_final_mass_kg,donor_applied_transfer_delta_kg,"
        "initial_total_tracer_kg,final_total_tracer_kg,"
        "sum_snapshot_m3,sum_swept_solid_m3,sum_pressure_source_m3,"
        "sum_pre_endpoint_mac_m3,sum_post_endpoint_mac_m3,"
        "sum_time_mean_fixed_post_mac_m3,"
        "sum_time_linear_velocity_proxy_m3,"
        "sum_pressure_residual_m3,sum_gcl_mismatch_m3,"
        "sum_force_y_impulse_Ns,sum_torque_impulse_Nms,"
        "applied_swept_kg,applied_fallback_kg,applied_mac_kg,"
        "applied_connected_kg,applied_event_count" );
    std::puts( "wind_boundary_pressure_summary,source_on,measured_swept_steps,"
        "measured_fallback_steps,swept_to_fallback,fallback_to_swept,"
        "fall_backs_with_physical_clearance,peak_swept_pressure_span,"
        "peak_fallback_pressure_span,peak_swept_stage_delta_mps,"
        "peak_fallback_stage_delta_mps,peak_swept_endpoint_residual_m3,"
        "peak_fallback_endpoint_residual_m3" );

    for( const bool wind : { false, true } )
    {
        fluidField2 field{ width, height, h, origin };
        field.setCutCellProjectionEnabled( true );
        field.setAnalyticCapsuleCutCellEnabled( true );
        if( field.isCutCellCoupledProjectionEnabled() ||
            field.getSmallCellPressureMergeFraction() != 0.0f ||
            field.isConnectedResidualTracerRemapEnabled() )
            fail( "boundary pressure audit changed normal solver settings" );

        fluidShapeObstacle2 capsule{
            capsule2{ { -halfLength, 0.0f }, { halfLength, 0.0f }, radius },
            { { 0.0f, startY }, rot2::FromRadians( 0.0f ) },
            { 0.0f, startY }, {}, 0.0f, 0.0f, 0.0f
        };
        const int uCount = ( width + 1 ) * height;
        const int vCount = width * ( height + 1 );
        std::vector<float> preU( uCount, 0.0f ), preV( vCount, 0.0f );
        fluidFlowAudit2 preFlow{};
        int pressureSnapshots = 0;
        field.setPrePressureSnapshotObserver( [&]( const fluidField2& pre )
        {
            ++pressureSnapshots;
            preFlow = pre.getFlowAudit( stepDt );
            for( int y = 0; y < height; ++y )
                for( int x = 1; x < width; ++x )
                {
                    const auto speed = pre.getUFaceVelocityForAudit( x, y );
                    if( !speed || !std::isfinite( *speed ) )
                        fail( "boundary pressure pre-U is nonfinite" );
                    preU[y * ( width + 1 ) + x] = *speed;
                }
            for( int y = 1; y < height; ++y )
                for( int x = 0; x < width; ++x )
                {
                    const auto speed = pre.getVFaceVelocityForAudit( x, y );
                    if( !speed || !std::isfinite( *speed ) )
                        fail( "boundary pressure pre-V is nonfinite" );
                    preV[y * width + x] = *speed;
                }
        } );

        int sweptCount = 0, fallbackCount = 0;
        int toFallback = 0, toSwept = 0, fallbackInside = 0;
        double sweptPeakSpan = 0.0, fallbackPeakSpan = 0.0;
        double sweptPeakDelta = 0.0, fallbackPeakDelta = 0.0;
        double sweptPeakResidual = 0.0, fallbackPeakResidual = 0.0;
        float previousY = startY;
        bool previousUsed = false;
        for( int frame = 0; frame < totalSteps; ++frame )
        {
            const float fraction = frame < halfSteps
                ? static_cast<float>( frame + 1 ) / halfSteps
                : static_cast<float>( totalSteps - frame - 1 ) / halfSteps;
            const float y = startY + ( lowerY - startY ) * fraction;
            capsule.transform.position = { 0.0f, y };
            capsule.center = capsule.transform.position;
            capsule.linearVelocity = { 0.0f, ( y - previousY ) / stepDt };
            capsule.angularVelocity = 0.0f;
            const float conservativeClearance =
                std::min( y, previousY ) - boundRadius - origin.y;
            const float actualClearance = y - radius - origin.y;
            if( actualClearance <= 0.0f )
                fail( "controlled Capsule actually crossed the fluid wall" );

            if( wind )
            {
                const float time = ( frame + 1 ) * stepDt;
                field.addVelocity( source, {
                    sourceStrength * stepDt,
                    0.5f * sourceStrength * stepDt * std::sin( 1.7f * time )
                }, 1.7f );
            }
            // Identical fluid entry state, imposed pose and source on both
            // copies. Only the conservative-radius eligibility changes.
            // These four checkpoints lie inside the real shape AABB, but
            // outside its enclosing circle and must use snapshot fallback
            // on the untouched solver path.
            const bool compareTight = frame == 4 || frame == 9 ||
                frame == 16 || frame == 26;
            std::optional<fluidField2> tight;
            if( compareTight )
            {
                tight.emplace( field );
                tight->setPrePressureSnapshotObserver( {} );
                tight->setTightTranslatingCapsuleSweptBoundsAuditEnabled( true );
            }
            std::array<fluidObstacleForce2, 1> feedback{};
            field.step( stepDt, std::span<const fluidShapeObstacle2>( &capsule, 1 ),
                feedback, 80 );
            if( pressureSnapshots != frame + 1 )
                fail( "missing boundary pre-pressure snapshot" );

            const fluidSweptFaceFlux2 swept = field.getSweptFaceFlux();
            const bool expected = frame > 0 && conservativeClearance > 0.0f;
            if( swept.usedSweptFlux != expected )
                fail( "Swept guard did not match conservative-radius eligibility" );

            const fluidFlowAudit2 postFlow = field.getFlowAudit( stepDt );
            const fluidCutCellPressure2 projection = field.getCutCellPressure();
            const fluidCutCellVolumeSummary2 summary =
                field.getCutCellVolumeSummary( stepDt );
            if( !summary.valid || summary.activeCells <= 0 ||
                !std::isfinite( postFlow.maxFluidFaceSpeed ) ||
                !std::isfinite( projection.pressureResidual ) ||
                !std::isfinite( projection.sourceImbalance ) ||
                !std::isfinite( projection.compatibilityCorrection ) ||
                !std::isfinite( feedback[0].force.x ) ||
                !std::isfinite( feedback[0].force.y ) ||
                !std::isfinite( feedback[0].torque ) )
                fail( "invalid finite boundary pressure / force budget" );

            double peakDelta = 0.0;
            bool deltaIsU = true;
            int deltaX = -1, deltaY = -1;
            float deltaOpen = 0.0f, deltaBefore = 0.0f, deltaAfter = 0.0f;
            const auto consider = [&]( bool isU, int x, int y,
                float open, float before, std::optional<float> after )
            {
                if( !( open > 0.0f ) || !after || !std::isfinite( *after ) )
                    return;
                const double change =
                    std::abs( static_cast<double>( *after ) - before );
                if( change <= peakDelta ) return;
                peakDelta = change;
                deltaIsU = isU;
                deltaX = x;
                deltaY = y;
                deltaOpen = open;
                deltaBefore = before;
                deltaAfter = *after;
            };
            for( int cellY = 0; cellY < height; ++cellY )
                for( int faceX = 1; faceX < width; ++faceX )
                    if( field.getCellFluidFraction( faceX - 1, cellY ) > 0.0f &&
                        field.getCellFluidFraction( faceX, cellY ) > 0.0f )
                        consider( true, faceX, cellY,
                            field.getUFaceOpenFraction( faceX, cellY ),
                            preU[cellY * ( width + 1 ) + faceX],
                            field.getUFaceVelocityForAudit( faceX, cellY ) );
            for( int faceY = 1; faceY < height; ++faceY )
                for( int cellX = 0; cellX < width; ++cellX )
                    if( field.getCellFluidFraction( cellX, faceY - 1 ) > 0.0f &&
                        field.getCellFluidFraction( cellX, faceY ) > 0.0f )
                        consider( false, cellX, faceY,
                            field.getVFaceOpenFraction( cellX, faceY ),
                            preV[faceY * width + cellX],
                            field.getVFaceVelocityForAudit( cellX, faceY ) );

            double pressureSum = 0.0, minPressure = 0.0, maxPressure = 0.0;
            int active = 0;
            for( int cy = 0; cy < height; ++cy )
                for( int cx = 0; cx < width; ++cx )
                    if( field.getCellFluidFraction( cx, cy ) > 0.0f )
                    {
                        const double p = field.getCellPressureForVisualization( cx, cy );
                        if( !std::isfinite( p ) )
                            fail( "nonfinite boundary visual pressure" );
                        if( active == 0 ) minPressure = maxPressure = p;
                        else
                        {
                            minPressure = std::min( minPressure, p );
                            maxPressure = std::max( maxPressure, p );
                        }
                        pressureSum += p;
                        ++active;
                    }
            const double mean = pressureSum / active;
            double peakRelative = 0.0;
            int peakX = -1, peakY = -1;
            for( int cy = 0; cy < height; ++cy )
                for( int cx = 0; cx < width; ++cx )
                    if( field.getCellFluidFraction( cx, cy ) > 0.0f )
                    {
                        const double p = field.getCellPressureForVisualization( cx, cy );
                        if( std::abs( p - mean ) <= peakRelative ) continue;
                        peakRelative = std::abs( p - mean );
                        peakX = cx;
                        peakY = cy;
                    }

            if( frame > 0 )
            {
                if( swept.usedSweptFlux ) ++sweptCount;
                else
                {
                    ++fallbackCount;
                    if( actualClearance > 0.0f ) ++fallbackInside;
                }
                if( previousUsed && !swept.usedSweptFlux ) ++toFallback;
                if( frame > 1 && !previousUsed && swept.usedSweptFlux ) ++toSwept;
            }
            previousUsed = swept.usedSweptFlux;
            const double span = maxPressure - minPressure;
            if( tight )
            {
                float tightPrePressureMax = 0.0f;
                int tightSnapshots = 0;
                tight->setPrePressureSnapshotObserver(
                    [&]( const fluidField2& pre )
                    {
                        ++tightSnapshots;
                        tightPrePressureMax =
                            pre.getFlowAudit( stepDt ).maxFluidFaceSpeed;
                    } );
                std::array<fluidObstacleForce2, 1> tightFeedback{};
                tight->step( stepDt,
                    std::span<const fluidShapeObstacle2>( &capsule, 1 ),
                    tightFeedback, 80 );
                const auto tightSwept = tight->getSweptFaceFlux();
                const auto tightSummary = tight->getCutCellVolumeSummary( stepDt );
                const auto tightProjection = tight->getCutCellPressure();
                const auto tightFlow = tight->getFlowAudit( stepDt );
                double tightMin = 0.0, tightMax = 0.0;
                int tightActive = 0;
                for( int cy = 0; cy < height; ++cy )
                    for( int cx = 0; cx < width; ++cx )
                        if( tight->getCellFluidFraction( cx, cy ) > 0.0f )
                        {
                            const double p =
                                tight->getCellPressureForVisualization( cx, cy );
                            if( !std::isfinite( p ) )
                                fail( "counterfactual pressure is nonfinite" );
                            if( tightActive == 0 ) tightMin = tightMax = p;
                            else
                            {
                                tightMin = std::min( tightMin, p );
                                tightMax = std::max( tightMax, p );
                            }
                            ++tightActive;
                        }
                if( swept.usedSweptFlux || !tightSwept.usedSweptFlux ||
                    !tightSummary.valid || tightActive != active ||
                    tightSnapshots != 1 ||
                    std::abs( summary.totalSnapshotDeltaVolume -
                        tightSummary.totalSnapshotDeltaVolume ) > 1.0e-10 ||
                    !std::isfinite( tightFlow.maxFluidFaceSpeed ) ||
                    !std::isfinite( tightProjection.compatibilityCorrection ) ||
                    !std::isfinite( tightFeedback[0].force.y ) )
                    fail( "tight Capsule clone was not an equivalent endpoint" );
                std::printf(
                    "wind_boundary_pressure_counterfactual,%d,%d,%d,%d",
                    wind ? 1 : 0, frame,
                    swept.usedSweptFlux ? 1 : 0,
                    tightSwept.usedSweptFlux ? 1 : 0 );
                const std::array<double, 15> comparison{{
                    summary.totalSnapshotDeltaVolume,
                    tightSummary.totalSnapshotDeltaVolume,
                    summary.totalPressureSourceDeltaVolume,
                    tightSummary.totalPressureSourceDeltaVolume,
                    span, tightMax - tightMin,
                    postFlow.maxFluidFaceSpeed,
                    tightFlow.maxFluidFaceSpeed,
                    summary.maxAbsPressureResidualVolume,
                    tightSummary.maxAbsPressureResidualVolume,
                    projection.compatibilityCorrection,
                    tightProjection.compatibilityCorrection,
                    feedback[0].force.y,
                    tightFeedback[0].force.y,
                    tightPrePressureMax
                }};
                for( double value : comparison ) std::printf( ",%.12g", value );
                std::putchar( '\n' );
            }
            if( swept.usedSweptFlux )
            {
                sweptPeakSpan = std::max( sweptPeakSpan, span );
                sweptPeakDelta = std::max( sweptPeakDelta, peakDelta );
                sweptPeakResidual = std::max(
                    sweptPeakResidual, summary.maxAbsPressureResidualVolume );
            }
            else if( frame > 0 )
            {
                fallbackPeakSpan = std::max( fallbackPeakSpan, span );
                fallbackPeakDelta = std::max( fallbackPeakDelta, peakDelta );
                fallbackPeakResidual = std::max(
                    fallbackPeakResidual, summary.maxAbsPressureResidualVolume );
            }

            std::printf( "wind_boundary_pressure_step,%d,%d",
                wind ? 1 : 0, frame );
            const std::array<double, 5> geometry{{
                y, conservativeClearance, actualClearance,
                expected ? 1.0 : 0.0,
                swept.usedSweptFlux ? 1.0 : 0.0
            }};
            for( double value : geometry ) std::printf( ",%.12g", value );
            std::printf( ",%d,%d", projection.hasVolumeHistory ? 1 : 0,
                swept.timeSamples );
            const std::array<double, 11> physics{{
                projection.volumeRate, projection.sourceImbalance,
                projection.compatibilityCorrection, projection.pressureResidual,
                projection.divergenceBefore, projection.divergenceAfter,
                preFlow.maxFluidFaceSpeed, postFlow.maxFluidFaceSpeed,
                peakDelta, deltaIsU ? 1.0 : 0.0, static_cast<double>( deltaX )
            }};
            for( double value : physics ) std::printf( ",%.12g", value );
            std::printf( ",%d,%.12g", deltaY, static_cast<double>( deltaOpen ) );
            const std::array<double, 3> pressure{{
                minPressure - mean, maxPressure - mean, peakRelative
            }};
            for( double value : pressure ) std::printf( ",%.12g", value );
            std::printf( ",%d,%d", peakX, peakY );
            const std::array<double, 7> volume{{
                summary.maxAbsPressureResidualVolume,
                static_cast<double>( summary.worstPressureX ),
                static_cast<double>( summary.worstPressureY ),
                summary.totalPressureSourceDeltaVolume,
                summary.totalSnapshotDeltaVolume,
                summary.totalMacNetOutwardVolume,
                summary.totalAbsSweptSnapshotMismatchVolume
            }};
            for( double value : volume ) std::printf( ",%.12g", value );
            std::printf( ",%.12g,%.12g,%.12g\n",
                static_cast<double>( feedback[0].force.x ),
                static_cast<double>( feedback[0].force.y ),
                static_cast<double>( feedback[0].torque ) );
            const auto printCell = [&]( const char* role, int cx, int cy )
            {
                if( cx < 0 || cy < 0 || cx >= width || cy >= height )
                    return;
                const auto budget = field.getCutCellVolumeBudget(
                    cx, cy, stepDt );
                if( !budget.valid )
                    fail( "invalid boundary hotspot cell budget" );
                std::printf( "wind_boundary_pressure_cell_hotspot,%d,%d,%s,%d,%d",
                    wind ? 1 : 0, frame, role, cx, cy );
                const std::array<double, 12> values{{
                    budget.previousFraction, budget.currentFraction,
                    budget.snapshotDeltaVolume, budget.pressureSourceDeltaVolume,
                    budget.solidSweptDivergenceVolume, budget.macNetOutwardVolume,
                    budget.pressureResidualVolume,
                    static_cast<double>(
                        field.getCellPressureForVisualization( cx, cy ) ) - mean,
                    field.getUFaceOpenFraction( cx, cy ),
                    field.getUFaceOpenFraction( cx + 1, cy ),
                    field.getVFaceOpenFraction( cx, cy ),
                    field.getVFaceOpenFraction( cx, cy + 1 )
                }};
                for( double value : values ) std::printf( ",%.12g", value );
                std::putchar( '\n' );
            };
            printCell( "worst_continuity", summary.worstPressureX,
                summary.worstPressureY );
            printCell( "peak_relative_pressure", peakX, peakY );
            if( deltaX >= 0 && deltaY >= 0 )
            {
                const int firstX = deltaIsU ? deltaX - 1 : deltaX;
                const int firstY = deltaIsU ? deltaY : deltaY - 1;
                const int secondX = deltaX, secondY = deltaY;
                printCell( "peak_delta_first", firstX, firstY );
                printCell( "peak_delta_second", secondX, secondY );
                const auto first = field.getCutCellVolumeBudget(
                    firstX, firstY, stepDt );
                const auto second = field.getCutCellVolumeBudget(
                    secondX, secondY, stepDt );
                std::printf( "wind_boundary_pressure_face_hotspot,%d,%d,%c,%d,%d",
                    wind ? 1 : 0, frame, deltaIsU ? 'U' : 'V',
                    deltaX, deltaY );
                const std::array<double, 7> face{{
                    deltaOpen, deltaBefore, deltaAfter,
                    first.currentFraction, second.currentFraction,
                    first.pressureResidualVolume, second.pressureResidualVolume
                }};
                for( double value : face ) std::printf( ",%.12g", value );
                std::putchar( '\n' );
            }
            previousY = y;
        }
        if( toFallback < 1 || toSwept < 1 || fallbackInside < 1 ||
            sweptCount < 1 || fallbackCount < 1 )
            fail( "Capsule did not cross both Swept guard thresholds while inside" );

        std::printf( "wind_boundary_pressure_summary,%d,%d,%d,%d,%d,%d",
            wind ? 1 : 0, sweptCount, fallbackCount, toFallback,
            toSwept, fallbackInside );
        const std::array<double, 6> peaks{{
            sweptPeakSpan, fallbackPeakSpan,
            sweptPeakDelta, fallbackPeakDelta,
            sweptPeakResidual, fallbackPeakResidual
        }};
        for( double value : peaks ) std::printf( ",%.12g", value );
        std::putchar( '\n' );
    }
    // Independent S2c coverage-guard axis. Both copies use the SAME
    // double-precision analytic Capsule geometry and prescribed motion;
    // guard alone differs, from the first snapshot onward. Do NOT change
    // coverage policy mid-run: the normal setter invalidates volume history.
    // A strictly covered cell with all four apertures zero must not remain
    // an active source-only pressure row due to roundoff.
    for( bool coverageGuard : { false, true } )
    {
        fluidField2 guardedField{ width, height, h, origin };
        guardedField.setCutCellProjectionEnabled( true );
        guardedField.setAnalyticCapsuleCutCellEnabled( true );
        guardedField.setAnalyticCapsuleDoubleGeometryEnabled( true );
        guardedField.setAnalyticCapsuleFullCoverageGuardEnabled( coverageGuard );
        fluidShapeObstacle2 guardedCapsule{
            capsule2{ { -halfLength, 0.0f }, { halfLength, 0.0f }, radius },
            { { 0.0f, startY }, rot2::FromRadians( 0.0f ) },
            { 0.0f, startY }, {}, 0.0f, 0.0f, 0.0f
        };
        float priorY = startY;
        for( int frame = 0; frame < totalSteps; ++frame )
        {
            const float fraction = frame < halfSteps
                ? static_cast<float>( frame + 1 ) / halfSteps
                : static_cast<float>( totalSteps - frame - 1 ) / halfSteps;
            const float y = startY + ( lowerY - startY ) * fraction;
            guardedCapsule.transform.position = { 0.0f, y };
            guardedCapsule.center = guardedCapsule.transform.position;
            guardedCapsule.linearVelocity = { 0.0f, ( y - priorY ) / stepDt };
            const bool checkpoint = frame == 4 || frame == 9 ||
                frame == 16 || frame == 26;
            // Both guard settings independently evolve their own default
            // history. For each guard and checkpoint, the tight path is a
            // copy of exactly the SAME pre-step Field (including pressure
            // and volume history). Only its Swept eligibility changes.
            // Comparing guard=0 to guard=1 is NOT a same-step counterfactual:
            // the full-coverage setter must be chosen before history starts.
            std::optional<fluidField2> tightReplay;
            if( checkpoint )
            {
                tightReplay.emplace( guardedField );
                tightReplay->setPrePressureSnapshotObserver( {} );
                tightReplay->setTightTranslatingCapsuleSweptBoundsAuditEnabled( true );
                // Capture aperture-time geometry only on the matched frame-9
                // counterfactual. Default fluid stepping stays unchanged.
                if( frame == 9 )
                    tightReplay->setSweptApertureAuditEnabled( true );
            }
            std::array<fluidObstacleForce2, 1> feedback{};
            guardedField.step( stepDt,
                std::span<const fluidShapeObstacle2>( &guardedCapsule, 1 ),
                feedback, 80 );
            if( checkpoint )
            {
                int phantom = 0;
                double phantomSource = 0.0;
                double pMin = 0.0, pMax = 0.0;
                int active = 0;
                for( int cy = 0; cy < height; ++cy )
                    for( int cx = 0; cx < width; ++cx )
                    {
                        const float current = guardedField.getCellFluidFraction(
                            cx, cy );
                        if( current <= 0.0f ) continue;
                        const double pressure =
                            guardedField.getCellPressureForVisualization( cx, cy );
                        if( active == 0 ) pMin = pMax = pressure;
                        else
                        {
                            pMin = std::min( pMin, pressure );
                            pMax = std::max( pMax, pressure );
                        }
                        ++active;
                        if( current >= 1.0e-12f ||
                            guardedField.getUFaceOpenFraction( cx, cy ) > 0.0f ||
                            guardedField.getUFaceOpenFraction( cx + 1, cy ) > 0.0f ||
                            guardedField.getVFaceOpenFraction( cx, cy ) > 0.0f ||
                            guardedField.getVFaceOpenFraction( cx, cy + 1 ) > 0.0f )
                            continue;
                        ++phantom;
                        phantomSource += guardedField.getCutCellVolumeBudget(
                            cx, cy, stepDt ).pressureSourceDeltaVolume;
                    }
                const auto budget =
                    guardedField.getCutCellVolumeSummary( stepDt );
                const auto& pressure = guardedField.getCutCellPressure();
                const auto flow = guardedField.getFlowAudit( stepDt );
                if( !budget.valid || active < 1 ||
                    !std::isfinite( phantomSource ) ||
                    !std::isfinite( pMin ) || !std::isfinite( pMax ) ||
                    !std::isfinite( feedback[0].force.y ) ||
                    !std::isfinite( flow.maxFluidFaceSpeed ) )
                    fail( "invalid double-Capsule coverage control" );
                std::printf( "wind_boundary_capsule_guard,%d,%d,%d,%d",
                    coverageGuard ? 1 : 0, frame,
                    guardedField.getSweptFaceFlux().usedSweptFlux ? 1 : 0,
                    phantom );
                const std::array<double, 9> metrics{{
                    phantomSource, budget.maxAbsPressureResidualVolume,
                    pressure.compatibilityCorrection, pMax - pMin,
                    flow.maxFluidFaceSpeed,
                    budget.totalSnapshotDeltaVolume,
                    budget.totalPressureSourceDeltaVolume,
                    feedback[0].force.y, pressure.pressureResidual
                }};
                for( double value : metrics ) std::printf( ",%.12g", value );
                std::putchar( '\n' );

                const auto recordFactorial = [&]( const fluidField2& sample,
                    const fluidObstacleForce2& force, bool tight )
                {
                    const auto volume =
                        sample.getCutCellVolumeSummary( stepDt );
                    const auto& pressure = sample.getCutCellPressure();
                    const auto speed = sample.getFlowAudit( stepDt );
                    double minimum = 0.0, maximum = 0.0;
                    int count = 0, phantomCells = 0;
                    double phantomSource = 0.0;
                    for( int cy = 0; cy < height; ++cy )
                        for( int cx = 0; cx < width; ++cx )
                        {
                            const float fraction =
                                sample.getCellFluidFraction( cx, cy );
                            if( fraction <= 0.0f ) continue;
                            const double value =
                                sample.getCellPressureForVisualization( cx, cy );
                            if( count == 0 ) minimum = maximum = value;
                            else
                            {
                                minimum = std::min( minimum, value );
                                maximum = std::max( maximum, value );
                            }
                            ++count;
                            if( fraction >= 1.0e-12f ||
                                sample.getUFaceOpenFraction( cx, cy ) > 0.0f ||
                                sample.getUFaceOpenFraction( cx + 1, cy ) > 0.0f ||
                                sample.getVFaceOpenFraction( cx, cy ) > 0.0f ||
                                sample.getVFaceOpenFraction( cx, cy + 1 ) > 0.0f )
                                continue;
                            ++phantomCells;
                            phantomSource += sample.getCutCellVolumeBudget(
                                cx, cy, stepDt ).pressureSourceDeltaVolume;
                        }
                    if( !volume.valid || count <= 0 ||
                        !std::isfinite( phantomSource ) ||
                        !std::isfinite( maximum - minimum ) ||
                        !std::isfinite( speed.maxFluidFaceSpeed ) ||
                        !std::isfinite( pressure.compatibilityCorrection ) ||
                        !std::isfinite( force.force.y ) )
                        fail( "nonfinite factorial pressure/volume sample" );
                    if( coverageGuard && phantomCells != 0 )
                        fail( "certified full-coverage guard left phantom cells" );
                    std::printf( "wind_boundary_factorial,%d,%d,%d,%d,%d",
                        coverageGuard ? 1 : 0, tight ? 1 : 0,
                        frame, sample.getSweptFaceFlux().usedSweptFlux ? 1 : 0,
                        phantomCells );
                    const std::array<double, 8> numbers{{
                        phantomSource, maximum - minimum,
                        speed.maxFluidFaceSpeed,
                        volume.maxAbsPressureResidualVolume,
                        pressure.compatibilityCorrection,
                        volume.totalSnapshotDeltaVolume,
                        volume.totalPressureSourceDeltaVolume,
                        force.force.y
                    }};
                    for( double value : numbers ) std::printf( ",%.12g", value );
                    std::putchar( '\n' );
                    return std::array<double, 2>{{
                        volume.totalSnapshotDeltaVolume,
                        static_cast<double>( count )
                    }};
                };
                const auto defaultGeometry = recordFactorial(
                    guardedField, feedback[0], false );
                // The two solver paths start from the same saved Field. These
                // four donor faces are read before pressure, so the endpoint
                // model is not mistaken for a measured intra-step u(t).
                constexpr int donorX = 26, donorY = 4;
                constexpr std::array<int, 4> faceX{{ 26, 27, 26, 26 }};
                constexpr std::array<int, 4> faceY{{ 4, 4, 4, 5 }};
                constexpr std::array<bool, 4> faceIsU{{ true, true, false, false }};
                constexpr std::array<double, 4> outward{{ -1.0, 1.0, -1.0, 1.0 }};
                constexpr std::array<char, 4> faceLabel{{ 'W', 'E', 'S', 'N' }};
                const auto readFace = [&]( const fluidField2& observed, int index )
                {
                    const auto velocity = faceIsU[index]
                        ? observed.getUFaceVelocityForAudit( faceX[index], faceY[index] )
                        : observed.getVFaceVelocityForAudit( faceX[index], faceY[index] );
                    if( !velocity || !std::isfinite( *velocity ) )
                        fail( "invalid frame-9 donor face velocity" );
                    return static_cast<double>( *velocity );
                };
                // Preserve one immutable frame-9 pre-step Field for all
                // 1/2/4/8 split counterfactuals. The refined trajectories
                // are independently stepped and are NOT samples of the
                // unsplit step's unknowable continuous u(t).
                std::optional<fluidField2> frame9Entry;
                if( frame == 9 ) frame9Entry.emplace( *tightReplay );
                std::array<double, 4> entryVelocity{}, preVelocity{};
                int capturedPrePressure = 0;
                if( frame == 9 )
                {
                    for( int i = 0; i < 4; ++i )
                        entryVelocity[i] = readFace( *tightReplay, i );
                    tightReplay->setPrePressureSnapshotObserver(
                        [&]( const fluidField2& pre )
                        {
                            ++capturedPrePressure;
                            for( int i = 0; i < 4; ++i )
                                preVelocity[i] = readFace( pre, i );
                        } );
                }
                std::array<fluidObstacleForce2, 1> tightForce{};
                tightReplay->step( stepDt,
                    std::span<const fluidShapeObstacle2>(
                        &guardedCapsule, 1 ), tightForce, 80 );
                const auto tightGeometry = recordFactorial(
                    *tightReplay, tightForce[0], true );
                if( guardedField.getSweptFaceFlux().usedSweptFlux ||
                    !tightReplay->getSweptFaceFlux().usedSweptFlux ||
                    std::abs( defaultGeometry[0] - tightGeometry[0] ) >
                        1.0e-10 ||
                    defaultGeometry[1] != tightGeometry[1] )
                    fail( "factorial guard path changed current geometry" );
                if( frame == 9 )
                {
                    if( capturedPrePressure != 1 ||
                        !tightReplay->hasSweptApertureAudit() ||
                        !tightReplay->getSweptFaceFlux().usedSweptFlux )
                        fail( "frame-9 aperture / pre-pressure history unavailable" );
                    const auto budget =
                        tightReplay->getCutCellVolumeBudget( donorX, donorY, stepDt );
                    if( !budget.valid || !budget.active ||
                        !( budget.currentFraction > 0.0 ) )
                        fail( "frame-9 donor no longer a true open Cut-cell" );

                    // Independent analytic Capsule geometry quadrature at
                    // 64/256 time midpoints. Neither reconstructs real u(t).
                    // The production aperture audit uses the Swept integrator's
                    // accepted sample count and is recorded separately.
                    const auto analyticOpen = [&]( double alpha, int index )
                    {
                        fluidShapeObstacle2 at = guardedCapsule;
                        at.transform.position.y =
                            priorY + static_cast<float>( alpha ) * ( y - priorY );
                        at.center = at.transform.position;
                        const auto open =
                            tightReplay->getAnalyticCapsuleFaceOpenFractionForAudit(
                                at, faceIsU[index], faceX[index], faceY[index] );
                        if( !open || !std::isfinite( *open ) ||
                            *open < 0.0 || *open > 1.0 )
                            fail( "invalid analytic frame-9 Capsule opening" );
                        return *open;
                    };
                    const auto analyticMean = [&]( int samples, int index )
                    {
                        double sum = 0.0;
                        for( int k = 0; k < samples; ++k )
                            sum += analyticOpen( ( k + 0.5 ) / samples, index );
                        return sum / samples;
                    };

                    double preNet = 0.0, postNet = 0.0;
                    double proxyNet = 0.0, fixedPostNet = 0.0;
                    for( int i = 0; i < 4; ++i )
                    {
                        const bool isU = faceIsU[i];
                        const int xFace = faceX[i], yFace = faceY[i];
                        const double endOpen = isU
                            ? tightReplay->getUFaceOpenFraction( xFace, yFace )
                            : tightReplay->getVFaceOpenFraction( xFace, yFace );
                        const double meanOpen = isU
                            ? tightReplay->getSweptUTimeMeanOpenFraction( xFace, yFace )
                            : tightReplay->getSweptVTimeMeanOpenFraction( xFace, yFace );
                        const double endSpeed = readFace( *tightReplay, i );
                        const double scale = outward[i] * h * stepDt;
                        const double preVolume = scale * endOpen * preVelocity[i];
                        const double postVolume = scale * endOpen * endSpeed;
                        // This proxy assumes linear MAC velocity between the
                        // measured step ENTRY and post-pressure endpoints.
                        const double proxyVolume = outward[i] * ( isU
                            ? tightReplay->getSweptULinearVelocityProxyVolume(
                                xFace, yFace, stepDt )
                            : tightReplay->getSweptVLinearVelocityProxyVolume(
                                xFace, yFace, stepDt ) );
                        const double fixedPostVolume = scale * meanOpen * endSpeed;
                        const double startOpen = analyticOpen( 0.0, i );
                        const double analyticEnd = analyticOpen( 1.0, i );
                        const double mean64 = analyticMean( 64, i );
                        const double mean256 = analyticMean( 256, i );
                        const std::array<double, 14> values{{
                            outward[i], startOpen, analyticEnd, endOpen,
                            meanOpen, mean64, mean256,
                            entryVelocity[i], preVelocity[i], endSpeed,
                            preVolume, postVolume, proxyVolume,
                            fixedPostVolume
                        }};
                        for( double value : values )
                            if( !std::isfinite( value ) )
                                fail( "nonfinite frame-9 time face quantity" );
                        if( !( meanOpen >= 0.0 && meanOpen <= 1.0 ) ||
                            std::abs( analyticEnd - endOpen ) > 1.0e-4 )
                            fail( "frame-9 analytic endpoint differs from live face" );
                        std::printf( "wind_boundary_time_face,%d,%d,%c",
                            coverageGuard ? 1 : 0, frame, faceLabel[i] );
                        for( double value : values )
                            std::printf( ",%.12g", value );
                        std::printf( ",%.12g\n", postVolume - preVolume );
                        preNet += preVolume;
                        postNet += postVolume;
                        proxyNet += proxyVolume;
                        fixedPostNet += fixedPostVolume;
                    }
                    // Local time-integrated fluid flux is UNKNOWN: proxyNet
                    // and fixedPostNet are counterfactuals, not conservation
                    // certificates. Solid swept flux and pressure source are
                    // separate ledgers. Prescribed Capsule force is not an
                    // equal-and-opposite dynamic Body impulse.
                    std::printf( "wind_boundary_time_balance,%d,%d,%d,%d",
                        coverageGuard ? 1 : 0, frame, donorX, donorY );
                    const std::array<double, 15> values{{
                        budget.currentFraction,
                        budget.snapshotDeltaVolume,
                        budget.solidSweptDivergenceVolume,
                        budget.pressureSourceDeltaVolume,
                        budget.macNetOutwardVolume,
                        budget.pressureResidualVolume,
                        preNet, postNet, proxyNet, fixedPostNet,
                        proxyNet + budget.pressureSourceDeltaVolume,
                        postNet - proxyNet,
                        budget.snapshotDeltaVolume -
                            budget.solidSweptDivergenceVolume,
                        tightForce[0].force.y, tightForce[0].torque
                    }};
                    for( double value : values )
                    {
                        if( !std::isfinite( value ) )
                            fail( "nonfinite frame-9 space-time budget" );
                        std::printf( ",%.12g", value );
                    }
                    std::putchar( '\n' );
                    // The four face endpoint sum must reproduce the public
                    // Cut-cell MAC budget exactly up to float accumulation.
                    if( std::abs( postNet - budget.macNetOutwardVolume ) > 1.0e-7 )
                        fail( "frame-9 signed endpoint MAC budget drift" );

                    // Compare independently computed, physically stepped MAC
                    // states at 1/2/4/8 time resolutions, all from the same
                    // pre-step Field. Reintegrating pressure and advection
                    // changes the numerical trajectory; this is convergence
                    // evidence, NOT the exact u(t) in the 1-step model.
                    for( const int splits : { 1, 2, 4, 8 } )
                    {
                        fluidField2 refined = *frame9Entry;
                        refined.setPrePressureSnapshotObserver( {} );
                        refined.setTracerTransferObserver( {} );
                        refined.setTracerStageObserver( {} );
                        refined.setSweptApertureAuditEnabled( true );
                        // Identical passive-dye seed in all four copies.
                        // The unseeded reference is used only to verify
                        // pressure/geometry are insensitive to this seed.
                        if( refined.getCellFluidFraction( donorX, donorY ) <= 0.0f )
                            fail( "frame-9 donor was not fluid before movement" );
                        refined.setTracerConcentration( donorX, donorY, 1.0f );
                        const double startMass = refined.getTracerMass();
                        const double startDonorMass =
                            refined.getTracerCellMass( donorX, donorY );
                        if( startMass <= 0.0 || startDonorMass <= 0.0 )
                            fail( "frame-9 tracer seed was not accepted" );

                        const float subDt = stepDt / splits;
                        const double faceScale =
                            static_cast<double>( h ) * subDt;
                        const int donorCell = donorY * width + donorX;
                        double appliedDonorMassDelta = 0.0;
                        std::array<double, 4> appliedKindMass{};
                        int appliedCount = 0, prePressureCount = 0;
                        std::array<double, 4> refinedPreVelocity{};
                        refined.setPrePressureSnapshotObserver(
                            [&]( const fluidField2& beforePressure )
                            {
                                ++prePressureCount;
                                for( int i = 0; i < 4; ++i )
                                    refinedPreVelocity[i] =
                                        readFace( beforePressure, i );
                            } );
                        refined.setTracerTransferObserver(
                            [&]( const fluidTracerTransfer2& event )
                            {
                                if( !std::isfinite( event.mass ) ||
                                    event.mass < 0.0 )
                                    fail( "invalid refined applied tracer transfer" );
                                const int kind = static_cast<int>( event.kind );
                                if( kind < 0 || kind >= 4 )
                                    fail( "unknown refined tracer transfer kind" );
                                appliedKindMass[kind] += event.mass;
                                ++appliedCount;
                                if( event.sourceCell == donorCell )
                                    appliedDonorMassDelta -= event.mass;
                                if( event.destinationCell == donorCell )
                                    appliedDonorMassDelta += event.mass;
                            } );
                        double sumSnapshot = 0.0, sumSwept = 0.0;
                        double sumSource = 0.0, sumPre = 0.0, sumPost = 0.0;
                        double sumMeanPost = 0.0, sumLinear = 0.0;
                        double sumResidual = 0.0, sumGcl = 0.0;
                        double impulseY = 0.0, angularImpulse = 0.0;
                        for( int part = 0; part < splits; ++part )
                        {
                            const float alpha =
                                static_cast<float>( part + 1 ) / splits;
                            const float poseY = part + 1 == splits ? y :
                                priorY + alpha * ( y - priorY );
                            fluidShapeObstacle2 moving = guardedCapsule;
                            moving.transform.position.y = poseY;
                            moving.center = moving.transform.position;
                            moving.linearVelocity.y = ( y - priorY ) / stepDt;

                            std::array<fluidObstacleForce2, 1> forces{};
                            const int previousPreCount = prePressureCount;
                            refined.step( subDt,
                                std::span<const fluidShapeObstacle2>(
                                    &moving, 1 ), forces, 80 );
                            const auto local = refined.getCutCellVolumeBudget(
                                donorX, donorY, subDt );
                            if( prePressureCount != previousPreCount + 1 ||
                                !local.valid || !local.active ||
                                !refined.getSweptFaceFlux().usedSweptFlux ||
                                !refined.hasSweptApertureAudit() )
                                fail( "incomplete frame-9 refined pressure history" );
                            double pre = 0.0, post = 0.0, meanPost = 0.0;
                            double linear = 0.0;
                            for( int i = 0; i < 4; ++i )
                            {
                                const double open = faceIsU[i]
                                    ? refined.getUFaceOpenFraction(
                                        faceX[i], faceY[i] )
                                    : refined.getVFaceOpenFraction(
                                        faceX[i], faceY[i] );
                                const double mean = faceIsU[i]
                                    ? refined.getSweptUTimeMeanOpenFraction(
                                        faceX[i], faceY[i] )
                                    : refined.getSweptVTimeMeanOpenFraction(
                                        faceX[i], faceY[i] );
                                const double after = readFace( refined, i );
                                const double signedScale = outward[i] * faceScale;
                                pre += signedScale * open *
                                    refinedPreVelocity[i];
                                post += signedScale * open * after;
                                // Piecewise constant post-pressure velocity
                                // times true time-quadrature aperture only.
                                meanPost += signedScale * mean * after;
                                // Independent piecewise linear velocity
                                // assumption on each newly solved substep.
                                linear += outward[i] * ( faceIsU[i]
                                    ? refined.getSweptULinearVelocityProxyVolume(
                                        faceX[i], faceY[i], subDt )
                                    : refined.getSweptVLinearVelocityProxyVolume(
                                        faceX[i], faceY[i], subDt ) );
                            }
                            if( std::abs( post - local.macNetOutwardVolume ) >
                                    1.0e-7 )
                                fail( "refined endpoint MAC outward budget drift" );
                            const double mass = refined.getTracerMass();
                            const double donorMass =
                                refined.getTracerCellMass( donorX, donorY );
                            const std::array<double, 16> row{{
                                alpha, poseY, local.currentFraction,
                                local.snapshotDeltaVolume,
                                local.solidSweptDivergenceVolume,
                                local.pressureSourceDeltaVolume,
                                pre, post, meanPost, linear,
                                local.pressureResidualVolume,
                                local.snapshotDeltaVolume -
                                    local.solidSweptDivergenceVolume,
                                forces[0].force.y, forces[0].torque,
                                mass, donorMass
                            }};
                            std::printf( "wind_boundary_refined_step,%d,%d,%d",
                                coverageGuard ? 1 : 0, splits, part );
                            for( double value : row )
                            {
                                if( !std::isfinite( value ) )
                                    fail( "nonfinite frame-9 refined step budget" );
                                std::printf( ",%.12g", value );
                            }
                            std::putchar( '\n' );
                            sumSnapshot += local.snapshotDeltaVolume;
                            sumSwept += local.solidSweptDivergenceVolume;
                            sumSource += local.pressureSourceDeltaVolume;
                            sumPre += pre;
                            sumPost += post;
                            sumMeanPost += meanPost;
                            sumLinear += linear;
                            sumResidual += local.pressureResidualVolume;
                            sumGcl += local.snapshotDeltaVolume -
                                local.solidSweptDivergenceVolume;
                            impulseY += static_cast<double>(
                                forces[0].force.y ) * subDt;
                            angularImpulse += static_cast<double>(
                                forces[0].torque ) * subDt;
                        }
                        const double endMass = refined.getTracerMass();
                        const double endDonorMass =
                            refined.getTracerCellMass( donorX, donorY );
                        const std::array<double, 20> summary{{
                            startDonorMass, endDonorMass,
                            appliedDonorMassDelta, startMass, endMass,
                            sumSnapshot, sumSwept, sumSource,
                            sumPre, sumPost, sumMeanPost, sumLinear,
                            sumResidual, sumGcl, impulseY, angularImpulse,
                            appliedKindMass[0], appliedKindMass[1],
                            appliedKindMass[2], appliedKindMass[3]
                        }};
                        std::printf( "wind_boundary_refined_summary,%d,%d",
                            coverageGuard ? 1 : 0, splits );
                        for( const double value : summary )
                        {
                            if( !std::isfinite( value ) )
                                fail( "nonfinite frame-9 refined cumulative budget" );
                            std::printf( ",%.12g", value );
                        }
                        std::printf( ",%d\n", appliedCount );
                        const double massTolerance = 1.0e-5 *
                            std::max( 1.0, startMass );
                        if( std::abs( endMass - startMass ) > massTolerance ||
                            std::abs( endDonorMass - startDonorMass -
                                appliedDonorMassDelta ) > massTolerance )
                            fail( "refined frame-9 applied tracer mass drift" );
                        if( std::abs( sumSnapshot -
                                budget.snapshotDeltaVolume ) > 1.0e-7 )
                            fail( "refined frame-9 final geometry volume drift" );
                        if( splits == 1 &&
                            ( std::abs( sumSource -
                                    budget.pressureSourceDeltaVolume ) > 1.0e-7 ||
                              std::abs( sumPost -
                                    budget.macNetOutwardVolume ) > 1.0e-7 ||
                              std::abs( impulseY - static_cast<double>(
                                    tightForce[0].force.y ) * stepDt ) > 1.0e-5 ) )
                            fail( "same-entry 1-split tracer altered pressure physics" );
                    }
                }
            }
            priorY = y;
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
    bool smallCellWorldImpulseAudit = false;
    bool firstCutCellHistoryAudit = false;
    bool firstPressureCoefficientAudit = false;
    bool capsuleTangentPressureAudit = false;
    bool capsuleTangentMovingAudit = false;
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
    bool capsuleMovingTransportAudit = false;
    bool capsuleHotspotLedgerAudit = false;
    bool capsulePressureOracleAudit = false;
    bool capsuleStep96SplitFluxAudit = false;
    bool cutCellVolumeAudit = false;
    bool windBoundaryPressureAudit = false;
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
        smallCellWorldImpulseAudit |= std::string_view( argv[i] ) == "--small-cell-world-impulse-audit";
        firstCutCellHistoryAudit |= std::string_view( argv[i] ) == "--small-cell-first-history-audit";
        firstPressureCoefficientAudit |= std::string_view( argv[i] ) == "--small-cell-pressure-coefficient-audit";
        capsuleTangentPressureAudit |= std::string_view( argv[i] ) == "--capsule-tangent-pressure-audit";
        capsuleTangentMovingAudit |= std::string_view( argv[i] ) == "--capsule-tangent-moving-audit";
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
        capsuleMovingTransportAudit |= std::string_view( argv[i] ) == "--small-cell-capsule-moving-transport-audit";
        capsuleHotspotLedgerAudit |= std::string_view( argv[i] ) == "--small-cell-capsule-hotspot-ledger-audit";
        capsulePressureOracleAudit |= std::string_view( argv[i] ) ==
            "--small-cell-capsule-noncoupled-pressure-oracle";
        capsuleStep96SplitFluxAudit |= std::string_view( argv[i] ) ==
            "--small-cell-capsule-step96-split-flux-audit";
        cutCellVolumeAudit |= std::string_view( argv[i] ) == "--cut-cell-volume-budget-audit";
        windBoundaryPressureAudit |= std::string_view( argv[i] ) == "--wind-boundary-pressure-spike-audit";
    }
    if( couplingAudit ) { auditCoupling(); return 0; }
    if( faceStageAudit ) { auditFaceStages(); return 0; }
    if( frozenPressureAudit ) { auditFrozenPressure(); return 0; }
    if( smallCellMergeAudit ) { auditFrozenPressure( true ); return 0; }
    if( smallCellThresholdSweep ) { auditFrozenPressure( true, true ); return 0; }
    if( smallCellCadenceAudit ) { auditSmallCellPressureCadences(); return 0; }
    if( smallCellWorldFeedbackAudit ) { auditSmallCellWorldFeedback(); return 0; }
    if( smallCellWorldImpulseAudit ) { auditSmallCellWorldPressureImpulse(); return 0; }
    if( firstCutCellHistoryAudit ) { auditFirstCutCellHistoryImpulse(); return 0; }
    if( firstPressureCoefficientAudit ) { auditFirstCutCellPressureCoefficients(); return 0; }
    if( capsuleTangentPressureAudit ) { auditCapsuleTangentFacePressure(); return 0; }
    if( capsuleTangentMovingAudit ) { auditCapsuleTangentMovingPressure(); return 0; }
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
    if( capsuleMovingTransportAudit ) { auditMovingCapsuleGuardTransport(); return 0; }
    if( capsuleHotspotLedgerAudit || capsulePressureOracleAudit )
    {
        auditMovingCapsuleGuardTransport( true, capsulePressureOracleAudit );
        return 0;
    }
    if( capsuleStep96SplitFluxAudit )
    {
        auditMovingCapsuleGuardTransport( false, false, true );
        return 0;
    }
    if( cutCellVolumeAudit ) { auditCutCellVolumeBudget(); return 0; }
    if( windBoundaryPressureAudit ) { auditWindBoundaryPressureSpike(); return 0; }
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
