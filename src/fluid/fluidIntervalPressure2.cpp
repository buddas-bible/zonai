#include "fluid/fluidIntervalPressure2.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace zonai::fluidDetail
{

intervalPressureResult solveIntervalPressure(
    const std::vector<double>& rhs, const std::vector<double>& diagonal,
    const std::vector<int>& parent, std::span<const float> seed, bool warmStart,
    double dt, double relativeSquaredTolerance,
    const std::vector<std::pair<std::size_t, double>>& closingRows,
    const intervalPressureMultiply& multiply, const intervalPressureGauge& zeroMean )
{
    const std::size_t n = rhs.size();
    if( diagonal.size() != n || parent.size() != n || !multiply || !zeroMean ||
        !std::isfinite( dt ) || dt <= 0.0 ||
        !std::isfinite( relativeSquaredTolerance ) || relativeSquaredTolerance <= 0.0 )
        throw std::invalid_argument( "interval pressure: invalid linear system input" );
    for( std::size_t i = 0; i < n; ++i )
        if( !std::isfinite( rhs[i] ) || !std::isfinite( diagonal[i] ) || diagonal[i] < 0.0 ||
            parent[i] < -1 || ( parent[i] >= 0 && static_cast<std::size_t>( parent[i] ) >= n ) )
            throw std::invalid_argument( "interval pressure: invalid linear system row" );
    for( const auto& [i, bound] : closingRows )
        if( i >= n || !std::isfinite( bound ) || bound < 0.0 )
            throw std::invalid_argument( "interval pressure: invalid closing row" );
    std::vector<double> p( n ), r = rhs, z( n ), d( n ), ap( n );
    const auto dot = [&]( const std::vector<double>& a, const std::vector<double>& b )
    {
        double sum = 0.0;
        for( std::size_t i = 0; i < n; ++i ) sum += a[i] * b[i];
        return sum;
    };
    const auto precondition = [&]()
    {
        for( std::size_t i = 0; i < n; ++i )
            z[i] = diagonal[i] > 0.0 ? r[i] / diagonal[i] : 0.0;
        zeroMean( z );
    };
    const double rhsSquared = dot( rhs, rhs );
    if( warmStart )
    {
        if( seed.size() != n )
            throw std::runtime_error( "interval pressure: invalid warm pressure size" );
        for( std::size_t i = 0; i < n; ++i )
        {
            if( !std::isfinite( seed[i] ) )
                throw std::runtime_error( "interval pressure: nonfinite warm pressure" );
            if( parent[i] >= 0 ) p[i] = seed[i];
        }
        // Reproject the seed pressure onto the forward graph gauge.
        // Recompute the residual; no old search direction is reused.
        zeroMean( p );
        multiply( p, ap ); zeroMean( ap );
        for( std::size_t i = 0; i < n; ++i ) r[i] = rhs[i] - ap[i];
    }
    precondition();
    d = z;
    double rz = dot( r, z ), squared = dot( r, r );
    const double seedSquared = squared;
    const double initial = warmStart ? rhsSquared : squared;
    const double tolerance = std::max( 1.0e-24, initial * relativeSquaredTolerance );
    // R4am: the fixed-geometry volume residual equals dt^2 times
    // the pressure row residual. A small global norm alone does not
    // bound error relative to the fluid expelled by a closing cell.
    const auto closingResidualSatisfied = [&]( const std::vector<double>& residual )
    {
        for( const auto& [i, bound] : closingRows )
            if( !( dt * dt * std::abs( residual[i] ) <= bound ) ) return false;
        return true;
    };
    int iterations = 0;
    for( ; iterations < 2000 && ( squared > tolerance || !closingResidualSatisfied( r ) ); ++iterations )
    {
        multiply( d, ap ); zeroMean( ap );
        const double denominator = dot( d, ap );
        if( !std::isfinite( denominator ) || denominator <= 0.0 || !std::isfinite( rz ) || rz <= 0.0 )
            throw std::runtime_error( "interval pressure: experimental PCG breakdown" );
        const double alpha = rz / denominator;
        for( std::size_t i = 0; i < n; ++i ) { p[i] += alpha * d[i]; r[i] -= alpha * ap[i]; }
        squared = dot( r, r );
        if( !std::isfinite( squared ) ) throw std::runtime_error( "interval pressure: nonfinite PCG residual" );
        if( squared <= tolerance && closingResidualSatisfied( r ) ) { ++iterations; break; }
        precondition();
        const double next = dot( r, z );
        const double beta = next / rz;
        for( std::size_t i = 0; i < n; ++i ) d[i] = z[i] + beta * d[i];
        rz = next;
    }
    zeroMean( p );
    multiply( p, ap );
    double unprojectedSquared = 0.0;
    for( std::size_t i = 0; i < n; ++i )
    {
        const double error = rhs[i] - ap[i];
        unprojectedSquared += error * error;
    }
    if( !closingRows.empty() )
    {
        // Validate independently recomputed unprojected rows.
        for( std::size_t i = 0; i < n; ++i ) r[i] = rhs[i] - ap[i];
        if( !closingResidualSatisfied( r ) )
            throw std::runtime_error( "interval pressure: true closing-cell residual failed" );
    }
    zeroMean( ap );
    double trueSquared = 0.0;
    for( std::size_t i = 0; i < n; ++i )
    {
        const double error = rhs[i] - ap[i];
        trueSquared += error * error;
    }
    if( !std::isfinite( trueSquared ) || trueSquared > tolerance )
        throw std::runtime_error( "interval pressure: experimental true residual failed" );
    return { std::move( p ), std::move( ap ), initial, seedSquared,
        trueSquared, unprojectedSquared, iterations };
}

}
