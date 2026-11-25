#pragma once

#include <cstddef>
#include <span>
#include <utility>
#include <vector>

namespace zonai::fluidDetail
{

struct intervalPressurePanel
{
    float alpha;
    double weight, lo;
};

struct intervalPressureQuadrature
{
    std::vector<intervalPressurePanel> panels;
    double weightSum = 0.0;
};

// Uniform midpoint mode retains unit weights and binary32 midpoint
// arithmetic. Refined mode uses dyadic subdivisions of the end panels.
[[nodiscard]] intervalPressureQuadrature buildIntervalPressureQuadrature( int samples, int endpointDepth );

// Canonical interior face order and metadata retained for endpoint reconstruction.
struct intervalPressureEdge
{
    int first, second;
    double a, velocity;
    bool isU;
    int index;
};

struct intervalPressureFaceInput
{
    std::span<const double> meanOpen;
    std::span<const float> velocity, endpointOpen;
};

struct intervalPressureGraph
{
    std::vector<intervalPressureEdge> edges;
    std::vector<double> diagonal, rawDivergence;
    std::vector<int> parent, count;
    int active = 0, regions = 0, closedEndFaces = 0;
    double sourceSum = 0.0;
};

// Endpoint openness is masked by the adjacent endpoint cell fractions.
// Build in canonical U-then-V order, with minimum-index component roots.
[[nodiscard]] intervalPressureGraph buildIntervalPressureGraph( int width, int height,
    double h, double dt, std::span<const double> meanFraction,
    std::span<const float> previousFraction, std::span<const float> endpointFraction,
    const intervalPressureFaceInput& u, const intervalPressureFaceInput& v );

// Internal kernels: caller supplies valid graph indices, matching extents,
// positive component counts and nonaliasing scratch/output vectors.
void projectIntervalPressureGauge( std::span<double> values, std::span<const int> parent,
    std::span<const int> count, std::span<double> sums );
void multiplyIntervalPressureGraph( std::span<const intervalPressureEdge> edges,
    std::span<const double> pressure, std::span<double> out );

// Use the same arithmetic in diagnostics and accepted reconstruction.
[[nodiscard]] inline double intervalFaceVelocity( const intervalPressureEdge& e,
    std::span<const double> pressure, double h, double dt )
{
    return e.velocity + dt / h * ( pressure[e.first] - pressure[e.second] );
}

[[nodiscard]] inline double intervalFaceTransfer( const intervalPressureEdge& e,
    double velocity, double h, double dt )
{
    return dt * h * e.a * velocity;
}

struct intervalPressureReconstruction
{
    std::vector<float> u, v;
    std::vector<double> fluxVolume, intervalU, intervalV;
    int uncoveredEndpointFaces = 0;
};

// Reconstruct a candidate without mutating the field. Keep pre-pressure
// velocities on closed/unsupported endpoint faces; integrate signed
// interval transfers independently of the endpoint aperture.
[[nodiscard]] intervalPressureReconstruction reconstructIntervalPressure(
    int width, int height, double h, double dt, const intervalPressureGraph& graph,
    std::span<const double> pressure, std::span<const float> previousFraction,
    std::span<const float> endpointFraction, const intervalPressureFaceInput& u,
    const intervalPressureFaceInput& v, bool captureIntervalVelocity );

struct intervalPressureRhsResult
{
    double compatibilityL1 = 0.0;
    int failedComponent = -1;
    double componentRhs = 0.0, componentVolume = 0.0;
};

// Build the RHS and remove component incompatibility. A component with
// nonzero net RHS and no endpoint capacity is rejected in weighted mode.
// On that failure rhs is diagnostic-only and must not be passed to PCG.
// Graph topology must come from buildIntervalPressureGraph; buffers must not alias.
[[nodiscard]] intervalPressureRhsResult buildIntervalPressureRhs(
    const intervalPressureGraph& graph, std::span<const float> endpointFraction,
    double scale, bool volumeWeightedCompatibility, std::span<double> rhs, std::span<double> sums );

struct intervalPressureResponse
{
    double fx = 0.0;
    double fy = 0.0;
    double torque = 0.0;
};

// Normalize accumulated cut-cell fractions, apertures and body coefficients
// with the same panel weight sum. No geometry or field mutation.
void normalizeIntervalPressureQuadrature( double weightSum, std::span<double> meanU,
    std::span<double> meanV, std::span<double> fraction, std::span<intervalPressureResponse> coefficients );

struct intervalPressureBody
{
    double inverseMass = 0.0;
    double inverseInertia = 0.0;
};

// Dense cell-major, owner-minor B coefficients. The caller supplies
// matching extents and nonaliasing buffers. Transpose includes fixed owners;
// the Schur product and diagonal use their inverse mass and inertia.
void transposeIntervalPressureBodies( std::span<const intervalPressureResponse> coefficients,
    std::span<const double> pressure, std::span<intervalPressureResponse> force );
void addIntervalPressureSchur( std::span<const intervalPressureResponse> coefficients,
    std::span<const intervalPressureBody> bodies, std::span<const intervalPressureResponse> force,
    double responseScale, std::span<double> out );
void addIntervalPressureSchurDiagonal( std::span<const intervalPressureResponse> coefficients,
    std::span<const intervalPressureBody> bodies, double responseScale, std::span<double> diagonal );

// Non-owning operator for one frozen interval. Graph, coefficients and
// body masses stay fixed; only force and gauge scratch are written.
// Buffers must outlive this operator and must not alias.
struct intervalPressureOperator
{
    const intervalPressureGraph& graph;
    std::span<const intervalPressureResponse> coefficients;
    std::span<const intervalPressureBody> bodies;
    std::span<intervalPressureResponse> force;
    std::span<double> sums;
    double responseScale;
    bool skipFixedBodyProducts;

    intervalPressureOperator( const intervalPressureGraph& graph,
        std::span<const intervalPressureResponse> coefficients,
        std::span<const intervalPressureBody> bodies, std::span<intervalPressureResponse> force,
        std::span<double> sums, double responseScale, bool skipFixedBodyProducts );
    void project( std::span<double> values ) const;
    void transpose( std::span<const double> pressure ) const;
    void multiply( std::span<const double> pressure, std::span<double> out ) const;
    void addDiagonal( std::span<double> diagonal ) const;
};

struct intervalPressureResult
{
    std::vector<double> pressure, projectedProduct;
    double initialSquared, seedSquared, trueSquared, unprojectedSquared;
    int iterations;
};

// Fixed-order projected Jacobi-PCG on a prepared interval operator.
// graph.diagonal includes the Schur diagonal (addDiagonal once before solving).
// The final traction is retained in operator force scratch even for fixed geometry.
// Closing-row bounds constrain dt^2 times the true unprojected residual.
// No geometry, World state or pressure commit.
[[nodiscard]] intervalPressureResult solveIntervalPressure(
    const std::vector<double>& rhs, const intervalPressureOperator& op,
    std::span<const float> seed, bool warmStart, double dt, double relativeSquaredTolerance,
    const std::vector<std::pair<std::size_t, double>>& closingRows );

}
