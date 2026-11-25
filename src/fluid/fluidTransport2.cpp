#include "fluid/fluidTransport2.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <limits>

namespace zonai::fluidDetail
{

std::vector<double> rebuildTransportLedger( const std::vector<double>& start,
    const std::vector<transportRepairFace>& faces )
{
    auto ledger = start;
    for( const auto& face : faces )
    {
        ledger[face.first] -= face.amount;
        ledger[face.second] += face.amount;
    }
    return ledger;
}
std::optional<transportRepairResult> repairClosingTransport(
    const std::vector<double>& start, const std::vector<double>& endpoint,
    const std::vector<transportRepairFace>& faces )
{
    const std::size_t n = start.size();
    if( endpoint.size() != n ) return std::nullopt;
    std::vector<std::vector<std::size_t>> incident( n );
    for( std::size_t i = 0; i < n; ++i )
        if( !std::isfinite( start[i] ) || !std::isfinite( endpoint[i] ) ||
            start[i] < 0.0 || endpoint[i] < 0.0 ) return std::nullopt;
    for( std::size_t f = 0; f < faces.size(); ++f )
    {
        const auto& edge = faces[f];
        if( edge.first < 0 || edge.second < 0 || edge.first == edge.second ||
            static_cast<std::size_t>( edge.first ) >= n ||
            static_cast<std::size_t>( edge.second ) >= n ||
            !std::isfinite( edge.amount ) ) return std::nullopt;
        incident[edge.first].push_back( f ); incident[edge.second].push_back( f );
    }
    transportRepairResult result;
    result.faces = faces;
    result.ledger = rebuildTransportLedger( start, faces );
    const auto baseline = result.ledger;
    constexpr double relativeDeficitLimit = 1.0e-6; // experimental, not a production tolerance
    constexpr double epsilon = std::numeric_limits<double>::epsilon();
    for( std::size_t i = 0; i < n; ++i )
    {
        if( !std::isfinite( result.ledger[i] ) ) return std::nullopt;
        if( result.ledger[i] >= 0.0 ) continue;
        if( endpoint[i] != 0.0 || start[i] <= 0.0 ||
            -result.ledger[i] > relativeDeficitLimit * start[i] ) return std::nullopt;
        const double guard = 16.0 * epsilon * start[i];
        const double need = -result.ledger[i] + guard;
        double remaining = need;
        // Stable input-face order, no hash traversal or tie dependence.
        for( std::size_t f : incident[i] )
        {
            auto& edge = result.faces[f];
            const bool fromFirst = edge.first == static_cast<int>( i );
            const int neighbor = fromFirst ? edge.second : edge.first;
            const double outward = fromFirst ? edge.amount : -edge.amount;
            if( outward <= 0.0 || endpoint[neighbor] <= 0.0 ) continue;
            const double reserve = 16.0 * epsilon * std::max( start[neighbor],
                std::abs( result.ledger[neighbor] ) );
            const double capacity = std::min( outward,
                std::max( 0.0, result.ledger[neighbor] - reserve ) );
            const double requested = std::min( remaining, capacity );
            if( requested <= 0.0 ) continue;
            const double revised = outward - requested;
            const double actual = outward - revised;
            if( actual <= 0.0 ) continue;
            edge.amount = fromFirst ? revised : -revised;
            result.ledger[neighbor] -= actual;
            remaining -= actual;
            result.reduction += actual;
            ++result.changedFaces;
            if( remaining <= 0.0 ) break;
        }
        if( remaining > guard * 0.5 ) return std::nullopt;
        result.ledger = rebuildTransportLedger( start, result.faces );
        if( result.ledger[i] < 0.0 || result.ledger[i] > 2.0 * guard ) return std::nullopt;
        ++result.repaired;
    }
    double sumDelta = 0.0, scale = 0.0;
    for( std::size_t i = 0; i < n; ++i )
    {
        if( !std::isfinite( result.ledger[i] ) || result.ledger[i] < 0.0 ) return std::nullopt;
        sumDelta += result.ledger[i] - baseline[i];
        scale += std::abs( baseline[i] ) + std::abs( result.ledger[i] );
    }
    result.conservationError = std::abs( sumDelta );
    if( !std::isfinite( scale ) || result.conservationError > 32.0 * epsilon * scale )
        return std::nullopt;
    return result;
}
std::vector<double> buildTransportPanelAmounts(
    const std::vector<transportRepairFace>& faces,
    const std::vector<transportFaceSample>& mapping,
    const std::vector<float>& uOpen, const std::vector<float>& vOpen,
    double weight, double weightSum )
{
    if( faces.size() != mapping.size() || !std::isfinite( weight ) ||
        !std::isfinite( weightSum ) || weight <= 0.0 || weightSum <= 0.0 || weight > weightSum )
        throw std::invalid_argument( "fluid transport: invalid panel weights or mapping size" );
    std::vector<double> amounts( mapping.size() );
    for( std::size_t j = 0; j < mapping.size(); ++j )
    {
        const auto& f = mapping[j];
        const auto& apertureSamples = f.isU ? uOpen : vOpen;
        if( f.index < 0 || static_cast<std::size_t>( f.index ) >= apertureSamples.size() ||
            !std::isfinite( f.mean ) || f.mean <= 0.0 || !std::isfinite( faces[j].amount ) )
            throw std::invalid_argument( "fluid transport: invalid mapped face" );
        const double aperture = apertureSamples[f.index];
        if( !std::isfinite( aperture ) || aperture < 0.0 || aperture > 1.0 )
            throw std::invalid_argument( "fluid transport: invalid sampled aperture" );
        const double amount = faces[j].amount * ( ( weight / weightSum ) * aperture / f.mean );
        if( !std::isfinite( amount ) )
            throw std::runtime_error( "fluid transport: nonfinite sampled transfer" );
        amounts[j] = amount;
    }
    return amounts;
}

throughflowAudit consumeThroughflowPanel(
    const std::vector<transportRepairFace>& faces, const std::vector<double>& amounts,
    std::vector<double>& outputVolume, std::vector<double>& outputMass,
    std::vector<double>& outputUniform, throughflowCommitMode mode )
{
    constexpr int maxPasses = 8;
    const std::size_t n = outputVolume.size();
    if( faces.size() != amounts.size() || outputMass.size() != n || outputUniform.size() != n ||
        &outputVolume == &outputMass || &outputVolume == &outputUniform ||
        &outputMass == &outputUniform || &amounts == &outputVolume ||
        &amounts == &outputMass || &amounts == &outputUniform )
        throw std::invalid_argument( "fluid transport: mismatched or aliased arrays" );
    if( mode != throughflowCommitMode::completeOnly && mode != throughflowCommitMode::allowPartial )
        throw std::invalid_argument( "fluid transport: invalid commit mode" );
    for( std::size_t i = 0; i < n; ++i )
        if( !std::isfinite( outputVolume[i] ) || outputVolume[i] < 0.0 ||
            !std::isfinite( outputMass[i] ) || outputMass[i] < 0.0 ||
            !std::isfinite( outputUniform[i] ) || outputUniform[i] < 0.0 )
            throw std::invalid_argument( "fluid transport: invalid initial state" );
    std::vector<double> pending( amounts.size() ), outgoing( n );
    for( std::size_t j = 0; j < amounts.size(); ++j )
    {
        const auto& face = faces[j];
        if( face.first < 0 || face.second < 0 || face.first == face.second ||
            static_cast<std::size_t>( face.first ) >= n ||
            static_cast<std::size_t>( face.second ) >= n || !std::isfinite( amounts[j] ) )
            throw std::invalid_argument( "fluid transport: invalid face transfer" );
        pending[j] = std::abs( amounts[j] );
    }
    // Work on independent copies, including for partial research commits.
    // Numerical failure must not expose a partly executed panel.
    auto volume = outputVolume, mass = outputMass, uniform = outputUniform;
    throughflowAudit result;
    for( int pass = 0; pass < maxPasses; ++pass )
    {
        std::fill( outgoing.begin(), outgoing.end(), 0.0 );
        bool hasPending = false;
        for( std::size_t j = 0; j < amounts.size(); ++j )
        {
            if( pending[j] == 0.0 ) continue;
            hasPending = true;
            const auto& edge = faces[j];
            auto& demand = outgoing[amounts[j] > 0.0 ? edge.first : edge.second];
            demand += pending[j];
            if( !std::isfinite( demand ) )
                throw std::runtime_error( "fluid transport: outgoing volume overflow" );
        }
        if( !hasPending ) break;
        ++result.passes;
        auto remainingVolume = volume, remainingMass = mass, remainingUniform = uniform;
        std::vector<double> receivedVolume( n ), receivedMass( n ), receivedUniform( n );
        bool progress = false;
        for( std::size_t j = 0; j < amounts.size(); ++j )
        {
            if( pending[j] == 0.0 ) continue;
            const auto& edge = faces[j];
            const int donor = amounts[j] > 0.0 ? edge.first : edge.second;
            const int recipient = amounts[j] > 0.0 ? edge.second : edge.first;
            const double capacity = volume[donor];
            if( capacity <= 0.0 ) continue;
            const double requested = outgoing[donor] <= capacity ? pending[j] :
                pending[j] * ( capacity / outgoing[donor] );
            const double moved = std::min( pending[j], std::min( remainingVolume[donor], requested ) );
            if( moved <= 0.0 ) continue;
            const double ratio = moved / capacity;
            const double movedMass = std::min( remainingMass[donor], mass[donor] * ratio );
            const double movedUniform = std::min( remainingUniform[donor], uniform[donor] * ratio );
            pending[j] -= moved;
            remainingVolume[donor] -= moved; receivedVolume[recipient] += moved;
            remainingMass[donor] -= movedMass; receivedMass[recipient] += movedMass;
            remainingUniform[donor] -= movedUniform; receivedUniform[recipient] += movedUniform;
            progress = true;
        }
        for( std::size_t i = 0; i < n; ++i )
        {
            volume[i] = remainingVolume[i] + receivedVolume[i];
            mass[i] = remainingMass[i] + receivedMass[i];
            uniform[i] = remainingUniform[i] + receivedUniform[i];
            if( !std::isfinite( volume[i] ) || volume[i] < 0.0 ||
                !std::isfinite( mass[i] ) || mass[i] < 0.0 ||
                !std::isfinite( uniform[i] ) || uniform[i] < 0.0 )
                throw std::runtime_error( "fluid transport: invalid throughflow state" );
        }
        if( !progress ) break;
    }
    for( double amount : pending ) result.unserved += amount;
    if( !std::isfinite( result.unserved ) )
        throw std::runtime_error( "fluid transport: remaining volume overflow" );
    result.committed = result.unserved == 0.0 || mode == throughflowCommitMode::allowPartial;
    if( result.committed )
    {
        outputVolume.swap( volume );
        outputMass.swap( mass );
        outputUniform.swap( uniform );
    }
    return result;
}

}
