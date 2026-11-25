#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include "fluid/fluidTransport2.h"

namespace zonai
{
struct fluidShapeObstacle2;
}

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

struct intervalPressurePoseMotion
{
    float angle, omega;
};

// Prepare the shortest angular displacement using the frozen interval dt.
// Start and finish must contain the same owners in the same order.
[[nodiscard]] std::vector<intervalPressurePoseMotion> prepareIntervalPressurePoseMotion(
    std::span<const fluidShapeObstacle2> start, std::span<const fluidShapeObstacle2> finish, float dt );

// Only pose and COM are updated. poses must be a nonaliasing scratch copy
// of finish; endpoint geometry, velocities and inverse mass remain frozen.
void sampleIntervalPressurePoses( std::span<const fluidShapeObstacle2> start,
    std::span<const fluidShapeObstacle2> finish, std::span<const intervalPressurePoseMotion> motion,
    float dt, float alpha, std::span<fluidShapeObstacle2> poses );

struct intervalPressureGeometrySample
{
    std::span<const float> fraction, uOpen, vOpen;
};

struct intervalPressureMissingFace
{
    bool isU;
    int x, y;
};

// Track the union of midpoint cell support and the remaining unsupported
// endpoint-open faces. All arrays use the frozen MAC layout.
class intervalPressureCoverage
{
public:
    intervalPressureCoverage( int width, int height, const intervalPressureGeometrySample& endpoint,
        bool refineEndpointFaces );
    void observe( const intervalPressureGeometrySample& sample );
    [[nodiscard]] std::vector<int> missingCells( std::span<const float> previous,
        std::span<const float> endpoint ) const;
    [[nodiscard]] const std::vector<intervalPressureMissingFace>& missingFaces() const noexcept
    {
        return missingFaces_;
    }

private:
    int width_, height_;
    std::vector<bool> supported_;
    std::vector<intervalPressureMissingFace> missingFaces_;
};

// Accumulate cell fractions and masked interior apertures in canonical
// order, writing a separate zero-boundary aperture snapshot for transport.
void accumulateIntervalPressureGeometry( int width, int height,
    const intervalPressureGeometrySample& sample, double weight,
    std::span<double> fraction, std::span<double> meanU, std::span<double> meanV,
    std::span<float> panelU, std::span<float> panelV );

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

// Weighted geometry and traction coefficients from the same ordered
// quadrature panels. The means are normalized, transport panels retain
// their original weights and panel-start capacities.
struct intervalPressureGeometryIntegral
{
    std::vector<double> meanU, meanV, fraction;
    std::vector<intervalPressureResponse> coefficients;
    std::vector<transportPanel> transportPanels;
    double weightSum = 0.0;
};

struct intervalPressureGeometryPreparation
{
    intervalPressureGeometryIntegral integral;
    std::vector<int> missed;
    std::vector<intervalPressureMissingFace> missedFaces;
    int endpointDepth = 0;
};

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

struct intervalPressureLinearOptions
{
    double responseScale = 1.0;
    bool volumeWeightedCompatibility = false;
    bool skipFixedBodyProducts = false;
    bool warmStart = false;
    double relativeSquaredTolerance = 1.0e-16;
    double closingVolumeRelativeTolerance = 0.0;
};

// Own the prepared operator buffers; non-owning operator views are created
// only after this object has reached its destination. A missing solution
// denotes incompatible RHS capacity, never an accepted pressure.
struct intervalPressureLinearState
{
    intervalPressureGraph graph;
    std::vector<double> rhs, sums;
    intervalPressureRhsResult rhsResult;
    std::vector<intervalPressureBody> bodies;
    std::vector<intervalPressureResponse> force;
    std::vector<std::pair<std::size_t, double>> closingRows;
    std::optional<intervalPressureResult> solution;
    double scale = 0.0;
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
