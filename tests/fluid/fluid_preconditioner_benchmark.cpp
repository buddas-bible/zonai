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

result measure( float h, int mode, bool ic0 )
{
    const auto bodies = makeObstacles( mode );
    std::vector<double> totalSamples, pressureSamples, iterationSamples, residualSamples;
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
        field.setAnalyticCutCellIncompleteCholeskyEnabled( ic0 );
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
    }
    stats.stepMilliseconds = median( totalSamples );
    stats.pressureMilliseconds = median( pressureSamples );
    stats.iterations = median( iterationSamples );
    stats.residual = median( residualSamples );
    return stats;
}

} // namespace

int main( int argc, char** argv )
{
    const bool csv = argc > 1 && std::string_view( argv[1] ) == "--csv";
    if( csv )
        std::puts( "geometry,cell_m,preconditioner,converged,attempts,pcg_median_ms,"
            "step_median_ms,iterations_median,relative_residual_median,pcg_speedup_ic0_over_jacobi" );
    else
        std::puts( "Analytic Cut-cell Schur PCG, cold start, unit density, 200 iterations max. "
            "Medians are informative, not CI performance gates." );

    constexpr std::array<const char*, 3> labels{ "overlap", "separated", "contained" };
    for( float h : { 0.5f, 0.25f } )
        for( int mode = 0; mode < 3; ++mode )
        {
            const result jacobi = measure( h, mode, false );
            const result ic0 = measure( h, mode, true );
            // Only report speedup if BOTH methods converge to the same tolerance.
            const double speedup = jacobi.converged == jacobi.attempts &&
                ic0.converged == ic0.attempts && ic0.pressureMilliseconds > 0.0
                ? jacobi.pressureMilliseconds / ic0.pressureMilliseconds : 0.0;
            const auto print = [&]( const char* name, const result& stats )
            {
                if( csv )
                    std::printf( "%s,%.2f,%s,%d,%d,%.5f,%.5f,%.0f,%.6g,%.4f\n",
                        labels[mode], h, name, stats.converged, stats.attempts,
                        stats.pressureMilliseconds, stats.stepMilliseconds, stats.iterations,
                        stats.residual, speedup );
                else
                    std::printf( "%s h=%.2f %s %d/%d converged, PCG %.4f ms, "
                        "step %.4f ms, iterations %.0f, relResidual %.6g, comparable speedup %.3fx\n",
                        labels[mode], h, name, stats.converged, stats.attempts,
                        stats.pressureMilliseconds, stats.stepMilliseconds, stats.iterations,
                        stats.residual, speedup );
            };
            print( "Jacobi", jacobi );
            print( "IC0", ic0 );
        }
}
