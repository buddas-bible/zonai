#pragma once

#include <cstddef>
#include <functional>
#include <span>
#include <utility>
#include <vector>

namespace zonai::fluidDetail
{

using intervalPressureMultiply = std::function<void( const std::vector<double>&, std::vector<double>& )>;
using intervalPressureGauge = std::function<void( std::vector<double>& )>;

// Canonical interior face order and metadata retained for endpoint reconstruction.
struct intervalPressureEdge
{
    int first, second;
    double a, velocity;
    bool isU;
    int index;
};

// Internal kernels: caller supplies valid graph indices, matching extents,
// positive component counts and nonaliasing scratch/output vectors.
void projectIntervalPressureGauge( std::span<double> values, std::span<const int> parent,
    std::span<const int> count, std::span<double> sums );
void multiplyIntervalPressureGraph( std::span<const intervalPressureEdge> edges,
    std::span<const double> pressure, std::span<double> out );

struct intervalPressureResponse
{
    double fx = 0.0;
    double fy = 0.0;
    double torque = 0.0;
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

struct intervalPressureResult
{
    std::vector<double> pressure, projectedProduct;
    double initialSquared, seedSquared, trueSquared, unprojectedSquared;
    int iterations;
};

// Fixed-order projected Jacobi-PCG on a caller-assembled positive operator.
// The multiply and gauge callbacks must preserve the deterministic graph
// order and vector extents. No geometry, World state or pressure commit.
// Closing-row bounds constrain dt^2 times the true unprojected residual.
[[nodiscard]] intervalPressureResult solveIntervalPressure(
    const std::vector<double>& rhs, const std::vector<double>& diagonal,
    const std::vector<int>& parent, std::span<const float> seed, bool warmStart,
    double dt, double relativeSquaredTolerance,
    const std::vector<std::pair<std::size_t, double>>& closingRows,
    const intervalPressureMultiply& multiply, const intervalPressureGauge& zeroMean );

}
