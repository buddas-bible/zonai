#include "rigidBodyDemo.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
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
        "peak_adjacent_cell_normal_speed,peak_face_flux_normal_velocity,fluid_ms,world_ms" );
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
                            "%.7g,%.7g,%d,%d,%d,%d,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.4f,%.4f\n",
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
                            peak.maxFaceVelocity * peak.maxFaceOpenFraction,
                            scene.getWindFluidMilliseconds(), scene.getWindWorldMilliseconds() );
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
    for( int i = 1; i < argc; ++i )
    {
        csv |= std::string_view( argv[i] ) == "--csv";
        reference |= std::string_view( argv[i] ) == "--ic0-reference";
        dense |= std::string_view( argv[i] ) == "--ic0-dense";
        schurReference |= std::string_view( argv[i] ) == "--schur-reference";
        warmStartProbe |= std::string_view( argv[i] ) == "--pressure-reuse-probe";
        warmStartSweep |= std::string_view( argv[i] ) == "--pcg-warmstart-sweep";
        couplingAudit |= std::string_view( argv[i] ) == "--coupling-spike-audit";
    }
    if( couplingAudit ) { auditCoupling(); return 0; }
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
