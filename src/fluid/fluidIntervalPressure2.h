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
