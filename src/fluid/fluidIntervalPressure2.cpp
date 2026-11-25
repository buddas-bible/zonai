#include "fluid/fluidIntervalPressure2.h"
#include "fluid/fluidField2.h"
#include "math/deterministicMath.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <string>
#include <stdexcept>

namespace zonai::fluidDetail
{

intervalPressureQuadrature buildIntervalPressureQuadrature( int samples, int endpointDepth )
{
    if( samples <= 0 || endpointDepth < 0 || endpointDepth > 16 ||
        ( endpointDepth > 0 && samples < 2 ) ||
        samples > std::numeric_limits<int>::max() - 2 * endpointDepth )
        throw std::invalid_argument( "interval pressure: unsupported quadrature schedule" );

    intervalPressureQuadrature result;
    result.panels.reserve( static_cast<std::size_t>( samples + 2 * endpointDepth ) );
    if( endpointDepth == 0 )
    {
        for( int step = 0; step < samples; ++step )
            result.panels.push_back( { ( step + 0.5f ) / samples, 1.0, static_cast<double>( step ) / samples } );
        result.weightSum = samples;
    }
    else
    {
        // Subdivide only the first and last uniform panels. Keep all
        // quadrature samples interior, without an endpoint aperture floor.
        const auto panel = [&]( double lo, double hi )
        {
            const double weight = hi - lo;
            const float alpha = static_cast<float>( 0.5 * ( lo + hi ) );
            if( !( alpha > 0.0f && alpha < 1.0f && weight > 0.0 ) )
                throw std::runtime_error( "interval pressure: invalid refined panel" );
            result.panels.push_back( { alpha, weight, lo } );
            result.weightSum += weight;
        };
        const double width = 1.0 / samples;
        double tiny = width;
        for( int i = 0; i < endpointDepth; ++i ) tiny *= 0.5;
        panel( 0.0, tiny );
        for( double a = tiny; a < width; a *= 2.0 ) panel( a, 2.0 * a );
        for( int step = 1; step < samples - 1; ++step )
            panel( step * width, ( step + 1 ) * width );
        for( double a = width; a > tiny; a *= 0.5 ) panel( 1.0 - a, 1.0 - 0.5 * a );
        panel( 1.0 - tiny, 1.0 );
        if( std::abs( result.weightSum - 1.0 ) > 1.0e-14 )
            throw std::runtime_error( "interval pressure: refined quadrature weights do not sum to one" );
    }
    return result;
}

std::vector<intervalPressurePoseMotion> prepareIntervalPressurePoseMotion(
    std::span<const fluidShapeObstacle2> start, std::span<const fluidShapeObstacle2> finish, float dt )
{
    if( start.size() != finish.size() || !std::isfinite( dt ) || dt <= 0.0f )
        throw std::invalid_argument( "interval pressure: invalid pose history" );
    std::vector<intervalPressurePoseMotion> motion( finish.size() );
    for( std::size_t i = 0; i < finish.size(); ++i )
    {
        motion[i].angle = deterministicAtan2( start[i].transform.rotation.s, start[i].transform.rotation.c );
        const float endAngle = deterministicAtan2( finish[i].transform.rotation.s, finish[i].transform.rotation.c );
        motion[i].omega = std::remainder( endAngle - motion[i].angle, 2.0f * std::numbers::pi_v<float> ) / dt;
        if( !std::isfinite( motion[i].angle ) || !std::isfinite( motion[i].omega ) )
            throw std::invalid_argument( "interval pressure: invalid angular pose history" );
    }
    return motion;
}

void sampleIntervalPressurePoses( std::span<const fluidShapeObstacle2> start,
    std::span<const fluidShapeObstacle2> finish, std::span<const intervalPressurePoseMotion> motion,
    float dt, float alpha, std::span<fluidShapeObstacle2> poses )
{
    if( start.size() != finish.size() || motion.size() != finish.size() || poses.size() != finish.size() ||
        !std::isfinite( dt ) || dt <= 0.0f || !std::isfinite( alpha ) || alpha < 0.0f || alpha > 1.0f )
        throw std::invalid_argument( "interval pressure: invalid pose sample" );
    for( std::size_t i = 0; i < finish.size(); ++i )
    {
        poses[i].transform.position = start[i].transform.position +
            alpha * ( finish[i].transform.position - start[i].transform.position );
        poses[i].transform.rotation = rot2::FromRadians( motion[i].angle + motion[i].omega * dt * alpha );
        poses[i].center = start[i].center + alpha * ( finish[i].center - start[i].center );
    }
}

namespace
{
void validateIntervalPressureGeometrySample( int width, int height, const intervalPressureGeometrySample& sample )
{
    const auto maxIndex = static_cast<std::size_t>( std::numeric_limits<int>::max() );
    if( width <= 0 || height <= 0 || width == std::numeric_limits<int>::max() ||
        height == std::numeric_limits<int>::max() )
        throw std::invalid_argument( "interval pressure: invalid geometry dimensions" );
    const auto w = static_cast<std::size_t>( width ), rows = static_cast<std::size_t>( height );
    if( w + 1 > maxIndex / rows || w > maxIndex / ( rows + 1 ) ||
        sample.fraction.size() != w * rows || sample.uOpen.size() != ( w + 1 ) * rows ||
        sample.vOpen.size() != w * ( rows + 1 ) )
        throw std::invalid_argument( "interval pressure: invalid geometry extents" );
}

float intervalPressureOpen( int width, const intervalPressureGeometrySample& sample, bool isU, int x, int y )
{
    const int right = y * width + x, left = right - ( isU ? 1 : width );
    if( sample.fraction[left] == 0.0f || sample.fraction[right] == 0.0f ) return 0.0f;
    return isU ? sample.uOpen[y * ( width + 1 ) + x] : sample.vOpen[right];
}
}

intervalPressureCoverage::intervalPressureCoverage( int width, int height,
    const intervalPressureGeometrySample& endpoint, bool refineEndpointFaces )
    : width_( width ), height_( height )
{
    validateIntervalPressureGeometrySample( width_, height_, endpoint );
    supported_.resize( endpoint.fraction.size() );
    if( !refineEndpointFaces ) return;
    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
            if( intervalPressureOpen( width_, endpoint, true, x, y ) > 0.0f )
                missingFaces_.push_back( { true, x, y } );
    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
            if( intervalPressureOpen( width_, endpoint, false, x, y ) > 0.0f )
                missingFaces_.push_back( { false, x, y } );
}

void intervalPressureCoverage::observe( const intervalPressureGeometrySample& sample )
{
    validateIntervalPressureGeometrySample( width_, height_, sample );
    for( std::size_t i = 0; i < supported_.size(); ++i )
        supported_[i] = supported_[i] || sample.fraction[i] > 0.0f;
    std::erase_if( missingFaces_, [&]( const intervalPressureMissingFace& f )
    {
        return intervalPressureOpen( width_, sample, f.isU, f.x, f.y ) > 0.0f;
    } );
}

std::vector<int> intervalPressureCoverage::missingCells( std::span<const float> previous,
    std::span<const float> endpoint ) const
{
    if( previous.size() != supported_.size() || endpoint.size() != supported_.size() )
        throw std::invalid_argument( "interval pressure: invalid coverage cell extents" );
    std::vector<int> missed;
    for( std::size_t i = 0; i < supported_.size(); ++i )
        if( !supported_[i] && endpoint[i] != previous[i] )
            missed.push_back( static_cast<int>( i ) );
    return missed;
}

void accumulateIntervalPressureGeometry( int width, int height,
    const intervalPressureGeometrySample& sample, double weight,
    std::span<double> fraction, std::span<double> meanU, std::span<double> meanV,
    std::span<float> panelU, std::span<float> panelV )
{
    validateIntervalPressureGeometrySample( width, height, sample );
    if( fraction.size() != sample.fraction.size() || meanU.size() != sample.uOpen.size() ||
        meanV.size() != sample.vOpen.size() || panelU.size() != meanU.size() || panelV.size() != meanV.size() ||
        !std::isfinite( weight ) || weight <= 0.0 )
        throw std::invalid_argument( "interval pressure: invalid geometry accumulation input" );
    std::fill( panelU.begin(), panelU.end(), 0.0f );
    std::fill( panelV.begin(), panelV.end(), 0.0f );
    for( std::size_t i = 0; i < fraction.size(); ++i ) fraction[i] += weight * sample.fraction[i];
    for( int y = 0; y < height; ++y )
        for( int x = 1; x < width; ++x )
        {
            const int index = y * ( width + 1 ) + x;
            const float aperture = intervalPressureOpen( width, sample, true, x, y );
            panelU[index] = aperture;
            meanU[index] += weight * aperture;
        }
    for( int y = 1; y < height; ++y )
        for( int x = 0; x < width; ++x )
        {
            const int index = y * width + x;
            const float aperture = intervalPressureOpen( width, sample, false, x, y );
            panelV[index] = aperture;
            meanV[index] += weight * aperture;
        }
}

intervalPressureGraph buildIntervalPressureGraph( int width, int height,
    double h, double dt, std::span<const double> meanFraction,
    std::span<const float> previousFraction, std::span<const float> endpointFraction,
    const intervalPressureFaceInput& u, const intervalPressureFaceInput& v )
{
    const auto maxIndex = static_cast<std::size_t>( std::numeric_limits<int>::max() );
    if( width <= 0 || height <= 0 || width == std::numeric_limits<int>::max() ||
        height == std::numeric_limits<int>::max() || !std::isfinite( h ) || h <= 0.0 ||
        !std::isfinite( dt ) || dt <= 0.0 )
        throw std::invalid_argument( "interval pressure: invalid graph dimensions or scales" );
    const auto w = static_cast<std::size_t>( width ), rows = static_cast<std::size_t>( height );
    if( w + 1 > maxIndex / rows || w > maxIndex / ( rows + 1 ) )
        throw std::invalid_argument( "interval pressure: graph index overflow" );
    const std::size_t n = w * rows, nu = ( w + 1 ) * rows, nv = w * ( rows + 1 );
    if( meanFraction.size() != n || previousFraction.size() != n || endpointFraction.size() != n ||
        u.meanOpen.size() != nu || u.velocity.size() != nu || u.endpointOpen.size() != nu ||
        v.meanOpen.size() != nv || v.velocity.size() != nv || v.endpointOpen.size() != nv )
        throw std::invalid_argument( "interval pressure: invalid graph extents" );

    intervalPressureGraph graph;
    auto& [edges, diagonal, rawDivergence, parent, count, active, regions, closedEndFaces, sourceSum] = graph;
    diagonal.resize( n ); rawDivergence.resize( n );
    parent.assign( n, -1 ); count.resize( n );
    for( std::size_t i = 0; i < n; ++i )
    {
        if( !std::isfinite( meanFraction[i] ) || !std::isfinite( previousFraction[i] ) ||
            !std::isfinite( endpointFraction[i] ) || meanFraction[i] < 0.0 ||
            previousFraction[i] < 0.0f || endpointFraction[i] < 0.0f )
            throw std::invalid_argument( "interval pressure: invalid cell fraction" );
        const double change = static_cast<double>( endpointFraction[i] ) - previousFraction[i];
        if( meanFraction[i] <= 0.0 )
        {
            if( change != 0.0 )
                throw std::runtime_error( "interval pressure: changed volume missed by quadrature at cell " +
                    std::to_string( i ) );
            continue;
        }
        parent[i] = static_cast<int>( i );
        ++active;
        sourceSum += change;
        rawDivergence[i] = change / dt;
    }
    const auto root = [&]( int i )
    {
        while( parent[i] != i ) i = parent[i];
        return i;
    };
    const auto addEdge = [&]( bool isU, int x, int y )
    {
        const auto& face = isU ? u : v;
        const int index = isU ? y * ( width + 1 ) + x : y * width + x;
        const double a = face.meanOpen[index];
        if( !std::isfinite( a ) || a < 0.0 )
            throw std::invalid_argument( "interval pressure: invalid mean face aperture" );
        if( a <= 0.0 ) return;
        const int right = y * width + x;
        const int left = right - ( isU ? 1 : width );
        if( parent[left] < 0 || parent[right] < 0 )
            throw std::runtime_error( "interval pressure: open face lacks interval-active cell" );
        const int l = root( left ), r = root( right );
        if( l != r ) parent[std::max( l, r )] = std::min( l, r );
        const double velocity = face.velocity[index];
        const float endOpen = endpointFraction[left] == 0.0f || endpointFraction[right] == 0.0f ?
            0.0f : face.endpointOpen[index];
        if( !std::isfinite( velocity ) || !std::isfinite( endOpen ) || endOpen < 0.0f )
            throw std::invalid_argument( "interval pressure: invalid active face state" );
        closedEndFaces += endOpen == 0.0f;
        edges.push_back( { left, right, a, velocity, isU, index } );
        diagonal[left] += a; diagonal[right] += a;
        const double flux = a * velocity / h;
        rawDivergence[left] += flux; rawDivergence[right] -= flux;
    };
    for( int y = 0; y < height; ++y )
        for( int x = 1; x < width; ++x ) addEdge( true, x, y );
    for( int y = 1; y < height; ++y )
        for( int x = 0; x < width; ++x ) addEdge( false, x, y );
    for( std::size_t i = 0; i < n; ++i )
        if( parent[i] >= 0 )
        {
            parent[i] = root( static_cast<int>( i ) );
            ++count[parent[i]];
        }
    for( int componentCount : count ) regions += componentCount > 0;
    if( active == 0 || edges.empty() ) throw std::runtime_error( "interval pressure: empty interval graph" );
    return graph;
}

void projectIntervalPressureGauge( std::span<double> values, std::span<const int> parent,
    std::span<const int> count, std::span<double> sums )
{
    std::fill( sums.begin(), sums.end(), 0.0 );
    for( std::size_t i = 0; i < values.size(); ++i )
        if( parent[i] >= 0 ) sums[parent[i]] += values[i];
    for( std::size_t i = 0; i < values.size(); ++i )
        if( parent[i] >= 0 ) values[i] -= sums[parent[i]] / count[parent[i]];
}

void multiplyIntervalPressureGraph( std::span<const intervalPressureEdge> edges,
    std::span<const double> pressure, std::span<double> out )
{
    std::fill( out.begin(), out.end(), 0.0 );
    for( const auto& e : edges )
    {
        const double d = e.a * ( pressure[e.first] - pressure[e.second] );
        out[e.first] += d; out[e.second] -= d;
    }
}

intervalPressureReconstruction reconstructIntervalPressure(
    int width, int height, double h, double dt, const intervalPressureGraph& graph,
    std::span<const double> pressure, std::span<const float> previousFraction,
    std::span<const float> endpointFraction, const intervalPressureFaceInput& u,
    const intervalPressureFaceInput& v, bool captureIntervalVelocity )
{
    const auto maxIndex = static_cast<std::size_t>( std::numeric_limits<int>::max() );
    if( width <= 0 || height <= 0 || width == std::numeric_limits<int>::max() ||
        height == std::numeric_limits<int>::max() || !std::isfinite( h ) || h <= 0.0 ||
        !std::isfinite( dt ) || dt <= 0.0 )
        throw std::invalid_argument( "interval pressure: invalid reconstruction dimensions or scales" );
    const auto w = static_cast<std::size_t>( width ), rows = static_cast<std::size_t>( height );
    if( w + 1 > maxIndex / rows || w > maxIndex / ( rows + 1 ) )
        throw std::invalid_argument( "interval pressure: reconstruction index overflow" );
    const std::size_t n = w * rows, nu = ( w + 1 ) * rows, nv = w * ( rows + 1 );
    if( pressure.size() != n || previousFraction.size() != n || endpointFraction.size() != n ||
        graph.parent.size() != n ||
        u.meanOpen.size() != nu || u.velocity.size() != nu || u.endpointOpen.size() != nu ||
        v.meanOpen.size() != nv || v.velocity.size() != nv || v.endpointOpen.size() != nv )
        throw std::invalid_argument( "interval pressure: invalid reconstruction extents" );

    intervalPressureReconstruction result;
    result.u.assign( u.velocity.begin(), u.velocity.end() );
    result.v.assign( v.velocity.begin(), v.velocity.end() );
    result.fluxVolume.resize( n );
    if( captureIntervalVelocity )
    {
        result.intervalU.resize( nu ); result.intervalV.resize( nv );
    }
    const double area = h * h;
    for( std::size_t i = 0; i < n; ++i )
    {
        if( !std::isfinite( pressure[i] ) || !std::isfinite( previousFraction[i] ) ||
            !std::isfinite( endpointFraction[i] ) || previousFraction[i] < 0.0f || endpointFraction[i] < 0.0f )
            throw std::invalid_argument( "interval pressure: invalid reconstruction cell" );
        result.fluxVolume[i] = area * previousFraction[i];
    }
    const auto endpointOpen = [&]( const intervalPressureFaceInput& face, int index, int left, int right )
    {
        return endpointFraction[left] == 0.0f || endpointFraction[right] == 0.0f ?
            0.0f : face.endpointOpen[index];
    };
    for( const auto& e : graph.edges )
    {
        const auto& face = e.isU ? u : v;
        const int stride = e.isU ? width + 1 : width;
        if( e.index < 0 || static_cast<std::size_t>( e.index ) >= face.meanOpen.size() )
            throw std::invalid_argument( "interval pressure: invalid reconstruction face index" );
        const int x = e.index % stride, y = e.index / stride;
        if( e.isU ? ( x <= 0 || x >= width || y >= height ) :
            ( y <= 0 || y >= height || x >= width ) )
            throw std::invalid_argument( "interval pressure: noninterior reconstruction face" );
        const int right = y * width + x, left = right - ( e.isU ? 1 : width );
        if( e.first != left || e.second != right || !std::isfinite( e.a ) || e.a <= 0.0 ||
            e.a != face.meanOpen[e.index] )
            throw std::invalid_argument( "interval pressure: inconsistent reconstruction edge" );
        const double velocity = intervalFaceVelocity( e, pressure, h, dt );
        const double transfer = intervalFaceTransfer( e, velocity, h, dt );
        if( !std::isfinite( velocity ) || !std::isfinite( transfer ) )
            throw std::runtime_error( "interval pressure: nonfinite reconstructed transfer" );
        if( endpointOpen( face, e.index, left, right ) > 0.0f )
        {
            if( std::abs( velocity ) > std::numeric_limits<float>::max() )
                throw std::runtime_error( "interval pressure: endpoint velocity overflow" );
            ( e.isU ? result.u : result.v )[e.index] = static_cast<float>( velocity );
        }
        result.fluxVolume[left] -= transfer;
        result.fluxVolume[right] += transfer;
        if( captureIntervalVelocity )
            ( e.isU ? result.intervalU : result.intervalV )[e.index] = velocity;
    }
    for( int y = 0; y < height; ++y )
        for( int x = 1; x < width; ++x )
        {
            const int index = y * ( width + 1 ) + x, right = y * width + x;
            result.uncoveredEndpointFaces += endpointOpen( u, index, right - 1, right ) > 0.0f &&
                u.meanOpen[index] <= 0.0;
        }
    for( int y = 1; y < height; ++y )
        for( int x = 0; x < width; ++x )
        {
            const int index = y * width + x;
            result.uncoveredEndpointFaces += endpointOpen( v, index, index - width, index ) > 0.0f &&
                v.meanOpen[index] <= 0.0;
        }
    return result;
}

intervalPressureRhsResult buildIntervalPressureRhs(
    const intervalPressureGraph& graph, std::span<const float> endpointFraction,
    double scale, bool volumeWeightedCompatibility, std::span<double> rhs, std::span<double> sums )
{
    const std::size_t n = graph.rawDivergence.size();
    const auto& parent = graph.parent;
    const auto& count = graph.count;
    const auto& rawDivergence = graph.rawDivergence;
    if( rhs.size() != n || sums.size() != n || endpointFraction.size() != n ||
        parent.size() != n || count.size() != n || !std::isfinite( scale ) || scale <= 0.0 )
        throw std::invalid_argument( "interval pressure: invalid RHS input" );
    for( std::size_t i = 0; i < n; ++i )
    {
        if( parent[i] < -1 || ( parent[i] >= 0 &&
            ( static_cast<std::size_t>( parent[i] ) >= n || count[parent[i]] <= 0 ) ) ||
            !std::isfinite( rawDivergence[i] ) || !std::isfinite( endpointFraction[i] ) ||
            endpointFraction[i] < 0.0f )
            throw std::invalid_argument( "interval pressure: invalid RHS row" );
        rhs[i] = -scale * rawDivergence[i];
        if( !std::isfinite( rhs[i] ) )
            throw std::invalid_argument( "interval pressure: nonfinite RHS" );
    }
    if( volumeWeightedCompatibility )
    {
        // Compatibility is an RHS correction, not the pressure gauge.
        std::vector<double> componentRhs( n ), componentVolume( n );
        for( std::size_t i = 0; i < n; ++i )
            if( parent[i] >= 0 )
            {
                componentRhs[parent[i]] += rhs[i];
                componentVolume[parent[i]] += endpointFraction[i];
            }
        for( std::size_t i = 0; i < n; ++i )
            if( parent[i] >= 0 )
            {
                const int c = parent[i];
                if( componentVolume[c] > 0.0 )
                    rhs[i] -= componentRhs[c] *
                        ( static_cast<double>( endpointFraction[i] ) / componentVolume[c] );
                else if( componentRhs[c] != 0.0 )
                    return { 0.0, c, componentRhs[c], componentVolume[c] };
            }
    }
    // Remove remaining roundoff (or apply the original uniform projection).
    projectIntervalPressureGauge( rhs, parent, count, sums );
    double compatibilityL1 = 0.0;
    for( std::size_t i = 0; i < n; ++i )
        compatibilityL1 += std::abs( rhs[i] + scale * rawDivergence[i] );
    return { compatibilityL1 };
}

void normalizeIntervalPressureQuadrature( double weightSum, std::span<double> meanU,
    std::span<double> meanV, std::span<double> fraction, std::span<intervalPressureResponse> coefficients )
{
    if( !std::isfinite( weightSum ) || weightSum <= 0.0 )
        throw std::invalid_argument( "interval pressure: invalid quadrature weight sum" );
    for( double& a : meanU ) a /= weightSum;
    for( double& a : meanV ) a /= weightSum;
    for( double& a : fraction ) a /= weightSum;
    for( auto& c : coefficients )
    {
        c.fx /= weightSum; c.fy /= weightSum; c.torque /= weightSum;
    }
}

void transposeIntervalPressureBodies( std::span<const intervalPressureResponse> coefficients,
    std::span<const double> pressure, std::span<intervalPressureResponse> force )
{
    const std::size_t owners = force.size();
    std::fill( force.begin(), force.end(), intervalPressureResponse{} );
    for( std::size_t i = 0; i < pressure.size(); ++i )
        for( std::size_t j = 0; j < owners; ++j )
        {
            const auto& c = coefficients[i * owners + j];
            auto& f = force[j];
            f.fx += c.fx * pressure[i]; f.fy += c.fy * pressure[i]; f.torque += c.torque * pressure[i];
        }
}

void addIntervalPressureSchur( std::span<const intervalPressureResponse> coefficients,
    std::span<const intervalPressureBody> bodies, std::span<const intervalPressureResponse> force,
    double responseScale, std::span<double> out )
{
    const std::size_t owners = bodies.size();
    for( std::size_t i = 0; i < out.size(); ++i )
        for( std::size_t j = 0; j < owners; ++j )
        {
            const auto& c = coefficients[i * owners + j];
            const auto& f = force[j];
            out[i] += responseScale * ( bodies[j].inverseMass * ( c.fx * f.fx + c.fy * f.fy ) +
                bodies[j].inverseInertia * c.torque * f.torque );
        }
}

void addIntervalPressureSchurDiagonal( std::span<const intervalPressureResponse> coefficients,
    std::span<const intervalPressureBody> bodies, double responseScale, std::span<double> diagonal )
{
    const std::size_t owners = bodies.size();
    for( std::size_t i = 0; i < diagonal.size(); ++i )
        for( std::size_t j = 0; j < owners; ++j )
        {
            const auto& c = coefficients[i * owners + j];
            diagonal[i] += responseScale * ( bodies[j].inverseMass * ( c.fx * c.fx + c.fy * c.fy ) +
                bodies[j].inverseInertia * c.torque * c.torque );
        }
}

intervalPressureOperator::intervalPressureOperator( const intervalPressureGraph& graph,
    std::span<const intervalPressureResponse> coefficients,
    std::span<const intervalPressureBody> bodies, std::span<intervalPressureResponse> force,
    std::span<double> sums, double responseScale, bool skipFixedBodyProducts )
    : graph( graph ), coefficients( coefficients ), bodies( bodies ), force( force ),
      sums( sums ), responseScale( responseScale ), skipFixedBodyProducts( skipFixedBodyProducts )
{
    const std::size_t n = graph.parent.size(), owners = bodies.size();
    if( graph.diagonal.size() != n || graph.count.size() != n || sums.size() != n ||
        force.size() != owners || ( owners != 0 && n > std::numeric_limits<std::size_t>::max() / owners ) ||
        coefficients.size() != n * owners || !std::isfinite( responseScale ) || responseScale < 0.0 ||
        ( skipFixedBodyProducts && responseScale != 0.0 ) )
        throw std::invalid_argument( "interval pressure: invalid operator extents or scale" );
    for( int root : graph.parent )
        if( root < -1 || ( root >= 0 &&
            ( static_cast<std::size_t>( root ) >= n || graph.count[root] <= 0 ) ) )
            throw std::invalid_argument( "interval pressure: invalid operator component" );
    for( const auto& e : graph.edges )
        if( e.first < 0 || e.second < 0 || static_cast<std::size_t>( e.first ) >= n ||
            static_cast<std::size_t>( e.second ) >= n || !std::isfinite( e.a ) || e.a <= 0.0 )
            throw std::invalid_argument( "interval pressure: invalid operator edge" );
}

void intervalPressureOperator::project( std::span<double> values ) const
{
    projectIntervalPressureGauge( values, graph.parent, graph.count, sums );
}

void intervalPressureOperator::transpose( std::span<const double> pressure ) const
{
    transposeIntervalPressureBodies( coefficients, pressure, force );
}

void intervalPressureOperator::multiply( std::span<const double> pressure, std::span<double> out ) const
{
    multiplyIntervalPressureGraph( graph.edges, pressure, out );
    if( skipFixedBodyProducts ) return;
    transpose( pressure );
    addIntervalPressureSchur( coefficients, bodies, force, responseScale, out );
}

void intervalPressureOperator::addDiagonal( std::span<double> diagonal ) const
{
    addIntervalPressureSchurDiagonal( coefficients, bodies, responseScale, diagonal );
}

intervalPressureResult solveIntervalPressure(
    const std::vector<double>& rhs, const intervalPressureOperator& op,
    std::span<const float> seed, bool warmStart, double dt, double relativeSquaredTolerance,
    const std::vector<std::pair<std::size_t, double>>& closingRows )
{
    const std::size_t n = rhs.size();
    const auto& diagonal = op.graph.diagonal;
    const auto& parent = op.graph.parent;
    if( diagonal.size() != n || parent.size() != n ||
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
        op.project( z );
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
        op.project( p );
        op.multiply( p, ap ); op.project( ap );
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
        op.multiply( d, ap ); op.project( ap );
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
    op.project( p );
    op.multiply( p, ap );
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
    op.project( ap );
    double trueSquared = 0.0;
    for( std::size_t i = 0; i < n; ++i )
    {
        const double error = rhs[i] - ap[i];
        trueSquared += error * error;
    }
    if( !std::isfinite( trueSquared ) || trueSquared > tolerance )
        throw std::runtime_error( "interval pressure: experimental true residual failed" );
    if( op.skipFixedBodyProducts ) op.transpose( p );
    return { std::move( p ), std::move( ap ), initial, seedSquared,
        trueSquared, unprojectedSquared, iterations };
}

}
