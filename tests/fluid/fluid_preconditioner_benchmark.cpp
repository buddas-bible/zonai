#include "fluid/fluidField2.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string_view>
#include <vector>

using namespace zonai;

namespace
{

constexpr int rounds = 5;
constexpr float dt = 1.0f / 60.0f;

struct result
{
    double stepMilliseconds = 0.0;
    double pressureMilliseconds = 0.0;
    double iterations = 0.0;
    double residual = 0.0;
    fluidCoupledPressureTiming2 phases{};
    int converged = 0;
    int attempts = 0;
};

double median( std::vector<double> samples )
{
    if( samples.empty() ) return 0.0;
    std::sort( samples.begin(), samples.end() );
    return samples[samples.size() / 2];
}

std::array<fluidShapeObstacle2, 2> makeObstacles( int mode )
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
    return bodies;
}

result measure( float h, int mode, int preconditioner )
{
    const auto bodies = makeObstacles( mode );
    std::vector<double> totalSamples, pressureSamples, iterationSamples, residualSamples;
    std::vector<fluidCoupledPressureTiming2> phaseSamples;
    result stats{};
    for( int i = 0; i < rounds + 1; ++i )
    {
        // Each timed trial starts with an identical cold field and pressure RHS.
        // Field construction and velocity seeding are excluded from total step time.
        fluidField2 field{ static_cast<int>( 12.0f / h ), static_cast<int>( 8.0f / h ),
            h, { -6.0f, -4.0f } };
        field.setCutCellProjectionEnabled( true );
        field.setCutCellCoupledProjectionEnabled( true );
        field.setDisjointAnalyticCutCellEnabled( true );
        field.setAnalyticCutCellIncompleteCholeskyEnabled( preconditioner != 0 );
        field.setAnalyticCutCellCachedIc0Enabled( preconditioner >= 2 );
        field.setAnalyticCutCellActiveIc0SweepEnabled( preconditioner >= 3 );
        field.setAnalyticCutCellCachedSchurEdgesEnabled( preconditioner == 4 );
        field.setCutCellPressureTimingEnabled( true );
        field.addVelocity( { -1.2f, 0.14f }, { 1.4f, -0.27f }, 1.6f );
        std::array<fluidObstacleForce2, 2> feedback{};
        const auto start = std::chrono::steady_clock::now();
        field.step( dt, bodies, feedback, 200 );
        const double stepMs = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start ).count();
        if( i == 0 ) continue; // Warm up the CPU/cache; no adaptive solver warm start.
        const auto& pressure = field.getCutCellPressure();
        ++stats.attempts;
        stats.converged += pressure.usedCoupledProjection ? 1 : 0;
        totalSamples.push_back( stepMs );
        pressureSamples.push_back( pressure.coupledSolveMilliseconds );
        iterationSamples.push_back( pressure.coupledIterations );
        residualSamples.push_back( pressure.coupledRelativeResidual );
        phaseSamples.push_back( pressure.coupledTiming );
    }
    stats.stepMilliseconds = median( totalSamples );
    stats.pressureMilliseconds = median( pressureSamples );
    stats.iterations = median( iterationSamples );
    stats.residual = median( residualSamples );
    const auto phaseMedian = [&]( auto getter )
    {
        std::vector<double> samples;
        samples.reserve( phaseSamples.size() );
        for( const auto& item : phaseSamples ) samples.push_back( getter( item ) );
        return median( std::move( samples ) );
    };
    auto& p = stats.phases;
    p.total = phaseMedian( []( const auto& v ) { return v.total; } );
    p.assembly = phaseMedian( []( const auto& v ) { return v.assembly; } );
    p.factorization = phaseMedian( []( const auto& v ) { return v.factorization; } );
    p.multiply = phaseMedian( []( const auto& v ) { return v.multiply; } );
    p.preconditioner = phaseMedian( []( const auto& v ) { return v.preconditioner; } );
    p.vector = phaseMedian( []( const auto& v ) { return v.vector; } );
    p.validation = phaseMedian( []( const auto& v ) { return v.validation; } );
    p.output = phaseMedian( []( const auto& v ) { return v.output; } );
    p.matrixProducts = static_cast<int>( phaseMedian( []( const auto& v ) { return v.matrixProducts; } ) );
    p.preconditionerApplications =
        static_cast<int>( phaseMedian( []( const auto& v ) { return v.preconditionerApplications; } ) );
    p.cachedLaplacianEdges =
        static_cast<int>( phaseMedian( []( const auto& v ) { return v.cachedLaplacianEdges; } ) );
    return stats;
}

} // namespace

int main( int argc, char** argv )
{
    const bool csv = argc > 1 && std::string_view( argv[1] ) == "--csv";
    if( csv )
        std::puts( "geometry,cell_m,preconditioner,converged,attempts,pcg_median_ms,"
            "step_median_ms,iterations_median,relative_residual_median,pcg_speedup_ic0_over_jacobi,"
            "assembly_ms,factorization_ms,operator_ms,preconditioner_ms,vector_ms,validation_ms,output_ms,"
            "operator_calls,preconditioner_calls,"
            "cached_ic0_vs_reference_speedup,cached_preconditioner_vs_reference_speedup,"
            "active_ic0_vs_dense_speedup,active_preconditioner_vs_dense_speedup,"
            "schur_cached_vs_reference_speedup,schur_operator_cached_vs_reference_speedup,cached_laplacian_edges" );
    else
        std::puts( "Analytic Cut-cell Schur PCG, cold start, unit density, 200 iterations max. "
            "Medians are informative, not CI performance gates." );

    constexpr std::array<const char*, 3> labels{ "overlap", "separated", "contained" };
    for( float h : { 0.5f, 0.25f } )
        for( int mode = 0; mode < 3; ++mode )
        {
            const result jacobi = measure( h, mode, 0 );
            const result reference = measure( h, mode, 1 );
            const result dense = measure( h, mode, 2 );
            const result active = measure( h, mode, 3 );
            const result edgeCached = measure( h, mode, 4 );
            // Only compare timings when both modes have fully converged.
            // A fail-closed Jacobi step cannot be used to claim a speedup.
            const bool comparable = reference.converged == reference.attempts &&
                active.converged == active.attempts;
            const bool denseComparable = dense.converged == dense.attempts &&
                active.converged == active.attempts;
            const double speedup = jacobi.converged == jacobi.attempts &&
                active.converged == active.attempts && active.pressureMilliseconds > 0.0
                ? jacobi.pressureMilliseconds / active.pressureMilliseconds : 0.0;
            const double cachedSpeedup = comparable && active.pressureMilliseconds > 0.0
                ? reference.pressureMilliseconds / active.pressureMilliseconds : 0.0;
            const double preconditionerSpeedup = comparable && active.phases.preconditioner > 0.0
                ? reference.phases.preconditioner / active.phases.preconditioner : 0.0;
            const double activeSpeedup = denseComparable && active.pressureMilliseconds > 0.0
                ? dense.pressureMilliseconds / active.pressureMilliseconds : 0.0;
            const double activePreconditionerSpeedup = denseComparable && active.phases.preconditioner > 0.0
                ? dense.phases.preconditioner / active.phases.preconditioner : 0.0;
            const bool edgeComparable = active.converged == active.attempts &&
                edgeCached.converged == edgeCached.attempts;
            const double edgeSpeedup = edgeComparable && edgeCached.pressureMilliseconds > 0.0
                ? active.pressureMilliseconds / edgeCached.pressureMilliseconds : 0.0;
            const double edgeOperatorSpeedup = edgeComparable && edgeCached.phases.multiply > 0.0
                ? active.phases.multiply / edgeCached.phases.multiply : 0.0;
            const auto print = [&]( const char* name, const result& stats )
            {
                const auto& p = stats.phases;
                if( csv )
                {
                    std::printf( "%s,%.2f,%s,%d,%d,%.5f,%.5f,%.0f,%.6g,%.4f",
                        labels[mode], h, name, stats.converged, stats.attempts,
                        stats.pressureMilliseconds, stats.stepMilliseconds, stats.iterations,
                        stats.residual, speedup );
                    std::printf( ",%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d\n",
                        p.assembly, p.factorization, p.multiply, p.preconditioner,
                        p.vector, p.validation, p.output, p.matrixProducts,
                        p.preconditionerApplications, cachedSpeedup, preconditionerSpeedup,
                        activeSpeedup, activePreconditionerSpeedup, edgeSpeedup,
                        edgeOperatorSpeedup, p.cachedLaplacianEdges );
                }
                else
                    std::printf( "%s h=%.2f %s %d/%d converged: PCG %.4f ms, A*v %.4f ms (%d), "
                        "preconditioner %.4f ms (%d), assembly %.4f ms, IC0 setup %.4f ms, "
                        "vector %.4f ms, validation %.4f ms, relative %.6g\n",
                        labels[mode], h, name, stats.converged, stats.attempts,
                        stats.pressureMilliseconds, p.multiply, p.matrixProducts,
                        p.preconditioner, p.preconditionerApplications,
                        p.assembly, p.factorization, p.vector, p.validation, stats.residual );
            };
            print( "Jacobi", jacobi );
            print( "IC0-reference", reference );
            print( "IC0-cached-dense", dense );
            print( "IC0-active-sweep", active );
            print( "IC0-schur-edge-cached", edgeCached );
        }
}
